from helios.settings import ProgressBarStrategy
from helios.validation import TimeInterval

from enum import StrEnum
from pydantic import BaseModel, model_validator
from typing import Callable, Optional
import math
import queue
import threading

from tqdm.auto import tqdm

import _helios


class HookPoint(StrEnum):
    LEG_START = "leg_start"
    LEG_END = "leg_end"
    SIM_TIME_ONCE = "sim_time_once"
    SIM_TIME_PERIODIC = "sim_time_periodic"


class HookPayload(StrEnum):
    METADATA_ONLY = "metadata_only"
    SINCE_LAST = "since_last"
    ALL_POINTS = "all_points"


class HookEndOfLegPolicy(StrEnum):
    NONE = "none"
    FLUSH = "flush"
    FLUSH_AND_RESET = "flush_and_reset"


class SurveyHook(BaseModel):
    point: HookPoint
    callback: Callable
    barrier: bool = False
    payload: HookPayload = HookPayload.METADATA_ONLY
    end_of_leg_policy: Optional[HookEndOfLegPolicy] = None
    sim_time: Optional[TimeInterval] = None
    period: Optional[TimeInterval] = None

    @model_validator(mode="after")
    def _validate_trigger_shape(self):
        if not callable(self.callback):
            raise ValueError("callback must be callable")

        if self.point == HookPoint.SIM_TIME_ONCE:
            if self.sim_time is None:
                raise ValueError("sim_time is required for SIM_TIME_ONCE")
            if self.period is not None:
                raise ValueError("period is invalid for SIM_TIME_ONCE")
            if self.end_of_leg_policy not in (None, HookEndOfLegPolicy.NONE):
                raise ValueError(
                    "end_of_leg_policy is only valid for SIM_TIME_PERIODIC"
                )
            if self.end_of_leg_policy is None:
                self.end_of_leg_policy = HookEndOfLegPolicy.NONE
        elif self.point == HookPoint.SIM_TIME_PERIODIC:
            if self.period is None:
                raise ValueError("period is required for SIM_TIME_PERIODIC")
            if self.period <= 0:
                raise ValueError("period must be > 0 for SIM_TIME_PERIODIC")
            if self.sim_time is None:
                self.sim_time = 0.0
            if self.end_of_leg_policy is None:
                self.end_of_leg_policy = HookEndOfLegPolicy.FLUSH
        else:
            if self.sim_time is not None or self.period is not None:
                raise ValueError(
                    "time trigger fields are only valid for timed hook points"
                )
            if self.end_of_leg_policy not in (None, HookEndOfLegPolicy.NONE):
                raise ValueError(
                    "end_of_leg_policy is only valid for SIM_TIME_PERIODIC"
                )
            if self.end_of_leg_policy is None:
                self.end_of_leg_policy = HookEndOfLegPolicy.NONE
        return self


_PROGRESS_BAR_EVENT_QUEUE_SIZE = 128
_PROGRESS_BAR_POLL_TIMEOUT_S = 0.05
_PROGRESS_BAR_PERIOD_S = 0.05
_TIME_BAR_FORMAT_WITH_TOTAL = (
    "{l_bar}{bar}| {n:.2f}/{total:.2f} [{elapsed}<{remaining}, {rate_fmt}{postfix}]"
)
_TIME_BAR_FORMAT_WITHOUT_TOTAL = (
    "{desc}: {n:.2f}{unit} [{elapsed}, {rate_fmt}{postfix}]"
)


def _coerce_non_negative(value, default=0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(parsed):
        return default
    return max(parsed, 0.0)


def _coerce_total(elapsed_s: float, remaining_s: float) -> Optional[float]:
    total_s = _coerce_non_negative(elapsed_s) + _coerce_non_negative(remaining_s)
    return total_s if total_s > 0.0 else None


def _extract_time_state(ctx) -> tuple[float, Optional[float], float, Optional[float]]:
    sim_time_s = _coerce_non_negative(getattr(ctx, "sim_time_s", 0.0))
    elapsed_time_s = _coerce_non_negative(
        getattr(ctx, "elapsed_time_s", sim_time_s), default=sim_time_s
    )
    remaining_time_s = _coerce_non_negative(getattr(ctx, "remaining_time_s", 0.0))
    leg_elapsed_time_s = _coerce_non_negative(
        getattr(ctx, "leg_elapsed_time_s", elapsed_time_s), default=elapsed_time_s
    )
    leg_remaining_time_s = _coerce_non_negative(
        getattr(ctx, "leg_remaining_time_s", 0.0)
    )
    return (
        elapsed_time_s,
        _coerce_total(elapsed_time_s, remaining_time_s),
        leg_elapsed_time_s,
        _coerce_total(leg_elapsed_time_s, leg_remaining_time_s),
    )


class _ProgressBarCallbacks:
    def __init__(
        self,
        strategy: ProgressBarStrategy,
        num_legs: int,
    ):
        self._strategy = strategy
        self._num_legs = max(1, int(num_legs))
        self._events: queue.Queue[
            tuple[str, int, float, Optional[float], float, Optional[float]]
        ] = queue.Queue(maxsize=_PROGRESS_BAR_EVENT_QUEUE_SIZE)
        self._stop = threading.Event()
        self._closed = False
        self._worker = threading.Thread(
            target=self._run, name="helios-progressbar", daemon=True
        )

        self._legs_bar = None
        self._time_bar = None

        if self._has_legs_bar:
            self._legs_bar = tqdm(
                total=self._num_legs,
                desc="Legs",
                unit="leg",
                dynamic_ncols=True,
                position=0,
                leave=True,
            )

        if self._has_time_bar:
            position = 1 if self._has_legs_bar else 0
            desc = (
                "Simulation time"
                if self._strategy != ProgressBarStrategy.PER_LEG_TIME
                else "Current leg time"
            )
            self._time_bar = tqdm(
                total=None,
                desc=desc,
                unit="s",
                bar_format=_TIME_BAR_FORMAT_WITHOUT_TOTAL,
                dynamic_ncols=True,
                position=position,
                leave=True,
            )

        self._legs_completed = 0
        self._last_global_elapsed_s = 0.0
        self._last_leg_elapsed_s = 0.0
        self._worker.start()

    @property
    def hooks(self) -> tuple[SurveyHook, ...]:
        hooks: list[SurveyHook] = []

        if self._strategy in (
            ProgressBarStrategy.LEGS,
            ProgressBarStrategy.LEGS_TIME,
            ProgressBarStrategy.PER_LEG_TIME,
        ):
            hooks.append(
                SurveyHook(point=HookPoint.LEG_START, callback=self._on_leg_start)
            )
            hooks.append(SurveyHook(point=HookPoint.LEG_END, callback=self._on_leg_end))
        elif self._strategy == ProgressBarStrategy.TIME:
            hooks.append(SurveyHook(point=HookPoint.LEG_END, callback=self._on_leg_end))

        if self._strategy in (
            ProgressBarStrategy.TIME,
            ProgressBarStrategy.LEGS_TIME,
            ProgressBarStrategy.PER_LEG_TIME,
        ):
            end_of_leg_policy = (
                HookEndOfLegPolicy.FLUSH_AND_RESET
                if self._strategy == ProgressBarStrategy.PER_LEG_TIME
                else HookEndOfLegPolicy.FLUSH
            )
            hooks.append(
                SurveyHook(
                    point=HookPoint.SIM_TIME_PERIODIC,
                    callback=self._on_time,
                    sim_time=0.0,
                    period=_PROGRESS_BAR_PERIOD_S,
                    end_of_leg_policy=end_of_leg_policy,
                )
            )

        return tuple(hooks)

    @property
    def _has_legs_bar(self) -> bool:
        return self._strategy in (
            ProgressBarStrategy.LEGS,
            ProgressBarStrategy.LEGS_TIME,
            ProgressBarStrategy.PER_LEG_TIME,
        )

    @property
    def _has_time_bar(self) -> bool:
        return self._strategy in (
            ProgressBarStrategy.TIME,
            ProgressBarStrategy.LEGS_TIME,
            ProgressBarStrategy.PER_LEG_TIME,
        )

    def _on_leg_start(self, ctx, points=None, trajectories=None):
        self._enqueue_event("leg_start", ctx)

    def _on_leg_end(self, ctx, points=None, trajectories=None):
        self._enqueue_event("leg_end", ctx)

    def _on_time(self, ctx, points=None, trajectories=None):
        self._enqueue_event("time", ctx)

    def _enqueue_event(self, event_kind: str, ctx):
        if self._closed:
            return
        (
            global_elapsed_s,
            global_total_s,
            leg_elapsed_s,
            leg_total_s,
        ) = _extract_time_state(ctx)
        event = (
            event_kind,
            int(ctx.leg_index),
            global_elapsed_s,
            global_total_s,
            leg_elapsed_s,
            leg_total_s,
        )
        try:
            self._events.put_nowait(event)
            return
        except queue.Full:
            pass

        # Drop stale UI events instead of blocking simulation callbacks.
        try:
            self._events.get_nowait()
        except queue.Empty:
            return

        try:
            self._events.put_nowait(event)
        except queue.Full:
            pass

    def _run(self):
        while not self._stop.is_set() or not self._events.empty():
            try:
                (
                    event_kind,
                    leg_index,
                    global_elapsed_s,
                    global_total_s,
                    leg_elapsed_s,
                    leg_total_s,
                ) = self._events.get(timeout=_PROGRESS_BAR_POLL_TIMEOUT_S)
            except queue.Empty:
                continue
            try:
                self._apply_event(
                    event_kind,
                    leg_index,
                    global_elapsed_s,
                    global_total_s,
                    leg_elapsed_s,
                    leg_total_s,
                )
            except Exception:
                # Progress bars are best effort and must never affect simulation.
                continue

    def _apply_event(
        self,
        event_kind: str,
        leg_index: int,
        global_elapsed_s: float,
        global_total_s: Optional[float],
        leg_elapsed_s: float,
        leg_total_s: Optional[float],
    ):
        if event_kind == "leg_start":
            if self._has_legs_bar and self._legs_bar is not None:
                current_leg = min(self._num_legs, max(1, leg_index + 1))
                self._legs_bar.set_description(f"Legs ({current_leg}/{self._num_legs})")
                self._legs_bar.refresh()

            if (
                self._strategy == ProgressBarStrategy.PER_LEG_TIME
                and self._time_bar is not None
            ):
                self._last_leg_elapsed_s = 0.0
                self._time_bar.reset(total=leg_total_s)
                self._set_time_bar_format(self._time_bar, leg_total_s)
                self._time_bar.set_description(f"Leg {leg_index + 1} time")
                self._time_bar.refresh()
            return

        if event_kind == "leg_end":
            self._update_legs(leg_index)
        if self._time_bar is None or event_kind not in ("leg_end", "time"):
            return

        if self._strategy == ProgressBarStrategy.PER_LEG_TIME:
            self._last_leg_elapsed_s = self._set_bar_value(
                bar=self._time_bar,
                current_value=leg_elapsed_s,
                previous_value=self._last_leg_elapsed_s,
                total_value=leg_total_s,
            )
            return

        if self._strategy in (ProgressBarStrategy.TIME, ProgressBarStrategy.LEGS_TIME):
            self._last_global_elapsed_s = self._set_bar_value(
                bar=self._time_bar,
                current_value=global_elapsed_s,
                previous_value=self._last_global_elapsed_s,
                total_value=global_total_s,
            )

    def _update_legs(self, leg_index: int):
        if not self._has_legs_bar or self._legs_bar is None:
            return
        target_completed = min(self._num_legs, max(self._legs_completed, leg_index + 1))
        delta = target_completed - self._legs_completed
        if delta > 0:
            self._legs_bar.update(delta)
            self._legs_completed = target_completed

    @staticmethod
    def _set_bar_value(
        bar,
        current_value: float,
        previous_value: float,
        total_value: Optional[float],
    ) -> float:
        if total_value is not None:
            total_value = _coerce_non_negative(total_value, default=0.0)
            current_total = getattr(bar, "total", None)
            if current_total is not None:
                current_total = _coerce_non_negative(current_total, default=0.0)
                if current_total <= 0.0:
                    current_total = None

            if total_value > 0.0 and current_total is None:
                bar.reset(total=total_value)
                previous_value = 0.0
            elif current_total is None or not math.isclose(
                current_total, total_value, rel_tol=1e-9, abs_tol=1e-12
            ):
                bar.total = total_value

        bar_total = getattr(bar, "total", None)
        if bar_total is not None:
            bar_total = _coerce_non_negative(bar_total, default=0.0)
            if bar_total <= 0:
                bar_total = None

        _ProgressBarCallbacks._set_time_bar_format(bar, bar_total)

        if bar_total is not None:
            current_value = min(current_value, bar_total)
            previous_value = min(previous_value, bar_total)

        delta = current_value - previous_value
        if delta > 0:
            bar.update(delta)
        elif delta < 0:
            bar.n = current_value
            bar.refresh()
        elif getattr(bar, "n", current_value) != current_value:
            bar.n = current_value
            bar.refresh()
        return current_value

    @staticmethod
    def _set_time_bar_format(bar, total_value: Optional[float]):
        if getattr(bar, "unit", None) != "s":
            return
        format_template = (
            _TIME_BAR_FORMAT_WITH_TOTAL
            if total_value is not None
            else _TIME_BAR_FORMAT_WITHOUT_TOTAL
        )
        if getattr(bar, "bar_format", None) != format_template:
            bar.bar_format = format_template

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        self._worker.join(timeout=2.0)

        while True:
            try:
                (
                    event_kind,
                    leg_index,
                    global_elapsed_s,
                    global_total_s,
                    leg_elapsed_s,
                    leg_total_s,
                ) = self._events.get_nowait()
            except queue.Empty:
                break
            try:
                self._apply_event(
                    event_kind,
                    leg_index,
                    global_elapsed_s,
                    global_total_s,
                    leg_elapsed_s,
                    leg_total_s,
                )
            except Exception:
                continue

        if self._legs_bar is not None:
            self._legs_bar.close()
        if self._time_bar is not None:
            self._time_bar.close()


def build_progressbar_callbacks(
    strategy: ProgressBarStrategy,
    num_legs: int,
) -> tuple[tuple[SurveyHook, ...], Optional[_ProgressBarCallbacks]]:
    if strategy == ProgressBarStrategy.NONE:
        return (), None
    progressbars = _ProgressBarCallbacks(strategy=strategy, num_legs=num_legs)
    return progressbars.hooks, progressbars


CPP_HOOK_POINT_MAP = {
    HookPoint.LEG_START: _helios.CppHookPoint.LEG_START,
    HookPoint.LEG_END: _helios.CppHookPoint.LEG_END,
    HookPoint.SIM_TIME_ONCE: _helios.CppHookPoint.SIM_TIME_ONCE,
    HookPoint.SIM_TIME_PERIODIC: _helios.CppHookPoint.SIM_TIME_PERIODIC,
}


CPP_HOOK_PAYLOAD_MAP = {
    HookPayload.METADATA_ONLY: _helios.CppHookPayload.METADATA_ONLY,
    HookPayload.SINCE_LAST: _helios.CppHookPayload.SINCE_LAST,
    HookPayload.ALL_POINTS: _helios.CppHookPayload.ALL_POINTS,
}


CPP_HOOK_END_OF_LEG_POLICY_MAP = {
    HookEndOfLegPolicy.NONE: _helios.CppHookEndOfLegPolicy.NONE,
    HookEndOfLegPolicy.FLUSH: _helios.CppHookEndOfLegPolicy.FLUSH,
    HookEndOfLegPolicy.FLUSH_AND_RESET: _helios.CppHookEndOfLegPolicy.FLUSH_AND_RESET,
}

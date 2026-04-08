import pytest

import helios.callbacks as callbacks_module

from helios import HookEndOfLegPolicy, HookPayload, HookPoint, Survey, SurveyHook
from helios.settings import ExecutionSettings, OutputFormat, ProgressBarStrategy
from tqdm.std import tqdm as tqdm_std


def _make_short_survey(scanner, tripod, scene, legs=1):
    survey = Survey(scanner=scanner, platform=tripod, scene=scene)
    for i in range(legs):
        survey.add_leg(
            x=float(i),
            y=0.0,
            z=0.0,
            pulse_frequency=1000,
            scan_angle="20 deg",
            head_rotation="10 deg/s",
            rotation_start_angle="0 deg",
            rotation_stop_angle="2 deg",
            scan_frequency=120,
            trajectory_time_interval=0.01,
        )
    return survey


def test_leg_start_and_leg_end_order(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene, legs=2)
    events = []

    def on_start(ctx, points=None, trajectories=None):
        events.append(("start", ctx.leg_index))

    def on_end(ctx, points=None, trajectories=None):
        events.append(("end", ctx.leg_index))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(point=HookPoint.LEG_START, callback=on_start),
            SurveyHook(point=HookPoint.LEG_END, callback=on_end),
        ),
    )

    assert events == [("start", 0), ("end", 0), ("start", 1), ("end", 1)]


def test_sim_time_once_fires_once(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    fired = []

    def on_once(ctx, points=None, trajectories=None):
        fired.append(ctx.sim_time_s)

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_ONCE,
                callback=on_once,
                sim_time=0.02,
            ),
        ),
    )

    assert len(fired) == 1


def test_sim_time_periodic_skips_missed_intervals(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    sim_times = []

    def on_periodic(ctx, points=None, trajectories=None):
        sim_times.append(ctx.sim_time_s)

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                sim_time=0.0,
                period=0.0001,
            ),
        ),
    )

    assert len(sim_times) > 0
    theoretical_nominal = int(sim_times[-1] / 0.0001) + 1
    assert len(sim_times) < theoretical_nominal / 2


def test_payload_metadata_only(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    seen = []

    def on_end(ctx, points=None, trajectories=None):
        seen.append((points is None, trajectories is None, ctx.payload_points))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.LEG_END,
                callback=on_end,
                payload=HookPayload.METADATA_ONLY,
            ),
        ),
    )

    assert seen
    assert all(
        p_none and t_none and payload_points == 0
        for p_none, t_none, payload_points in seen
    )


def test_payload_since_last_non_overlapping_points_and_trajectories(
    tls_scanner, tripod, scene
):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    payload_point_sizes = []
    payload_traj_sizes = []

    def on_periodic(ctx, points=None, trajectories=None):
        payload_point_sizes.append(0 if points is None else len(points))
        payload_traj_sizes.append(0 if trajectories is None else len(trajectories))

    measurements, trajectories = survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.SINCE_LAST,
                sim_time=0.0,
                period=0.0001,
            ),
        ),
    )

    assert sum(payload_point_sizes) == len(measurements)
    assert sum(payload_traj_sizes) == len(trajectories)


def test_payload_since_last_periodic_tail_flushes_remaining_points(tls_survey):
    payload_point_sizes = []

    def on_periodic(ctx, points=None, trajectories=None):
        payload_point_sizes.append(0 if points is None else len(points))

    measurements, _ = tls_survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.SINCE_LAST,
                sim_time=1000.0,
                period=1000.0,
                end_of_leg_policy=HookEndOfLegPolicy.FLUSH,
            ),
        ),
    )

    assert len(measurements) > 0
    assert len(payload_point_sizes) == 1
    assert sum(payload_point_sizes) == len(measurements)


def test_periodic_end_of_leg_policy_none_disables_tail_flush(tls_survey):
    payload_point_sizes = []

    def on_periodic(ctx, points=None, trajectories=None):
        payload_point_sizes.append(0 if points is None else len(points))

    measurements, _ = tls_survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.SINCE_LAST,
                sim_time=1000.0,
                period=1000.0,
                end_of_leg_policy=HookEndOfLegPolicy.NONE,
            ),
        ),
    )

    assert len(measurements) > 0
    assert len(payload_point_sizes) == 0


def test_periodic_end_of_leg_policy_flush_and_reset_resets_timer(
    tls_scanner, tripod, scene
):
    survey = Survey(scanner=tls_scanner, platform=tripod, scene=scene)
    for x in (0.0, 1.0):
        survey.add_leg(
            x=x,
            y=0.0,
            z=0.0,
            pulse_frequency=2000,
            scan_angle="20 deg",
            head_rotation="10 deg/s",
            rotation_start_angle="0 deg",
            rotation_stop_angle="10 deg",
            scan_frequency=120,
            trajectory_time_interval=0.01,
        )

    callbacks = []

    def on_periodic(ctx, points=None, trajectories=None):
        callbacks.append((ctx.leg_index, ctx.scheduled_time_s, ctx.sim_time_s))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.SINCE_LAST,
                sim_time=0.0,
                period=1.5,
                end_of_leg_policy=HookEndOfLegPolicy.FLUSH_AND_RESET,
            ),
        ),
    )

    leg1_callbacks = [c for c in callbacks if c[0] == 1]
    assert len(leg1_callbacks) == 1
    assert leg1_callbacks[0][1] == pytest.approx(leg1_callbacks[0][2])


def test_payload_all_points_and_trajectories_grows(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    point_sizes = []
    traj_sizes = []

    def on_periodic(ctx, points=None, trajectories=None):
        point_sizes.append(0 if points is None else len(points))
        traj_sizes.append(0 if trajectories is None else len(trajectories))

    measurements, trajectories = survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=on_periodic,
                payload=HookPayload.ALL_POINTS,
                sim_time=0.0,
                period=0.01,
            ),
        ),
    )

    assert point_sizes
    assert traj_sizes
    assert all(x <= y for x, y in zip(point_sizes, point_sizes[1:]))
    assert all(x <= y for x, y in zip(traj_sizes, traj_sizes[1:]))
    assert point_sizes[-1] == len(measurements)
    assert traj_sizes[-1] == len(trajectories)


def test_barrier_true_includes_pending_points(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    payload_sizes = []

    def on_once(ctx, points=None, trajectories=None):
        payload_sizes.append(0 if points is None else len(points))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=4),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_ONCE,
                callback=on_once,
                payload=HookPayload.SINCE_LAST,
                barrier=True,
                sim_time=0.05,
            ),
        ),
    )

    assert payload_sizes and payload_sizes[0] > 0


def test_barrier_true_includes_pending_trajectories(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    payload_sizes = []

    def on_once(ctx, points=None, trajectories=None):
        payload_sizes.append(0 if trajectories is None else len(trajectories))

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=4),
        callbacks=(
            SurveyHook(
                point=HookPoint.SIM_TIME_ONCE,
                callback=on_once,
                payload=HookPayload.SINCE_LAST,
                barrier=True,
                sim_time=0.05,
            ),
        ),
    )

    assert payload_sizes and payload_sizes[0] > 0


def test_callback_exception_stops_and_propagates(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)

    def on_once(ctx, points=None, trajectories=None):
        raise RuntimeError("callback boom")

    with pytest.raises(RuntimeError, match="callback boom"):
        survey.run(
            format=OutputFormat.NPY,
            execution_settings=ExecutionSettings(num_threads=1),
            callbacks=(
                SurveyHook(
                    point=HookPoint.SIM_TIME_ONCE,
                    callback=on_once,
                    sim_time=0.01,
                ),
            ),
        )


def test_progressbar_callbacks_append_and_close(
    monkeypatch, tls_scanner, tripod, scene
):
    survey = _make_short_survey(tls_scanner, tripod, scene, legs=2)
    closed = []
    bars = []

    class FakeTqdm:
        def __init__(
            self,
            total=None,
            desc="",
            unit="it",
            bar_format=None,
            dynamic_ncols=True,
            position=0,
            leave=True,
        ):
            self.total = total
            self.desc = desc
            self.unit = unit
            self.bar_format = bar_format
            self.dynamic_ncols = dynamic_ncols
            self.position = position
            self.leave = leave
            self.n = 0.0
            bars.append(self)

        def update(self, delta):
            self.n += float(delta)

        def refresh(self):
            return None

        def close(self):
            closed.append(self)

        def reset(self, total=None):
            self.total = total
            self.n = 0.0

        def set_description(self, desc):
            self.desc = desc

        def __bool__(self):
            if self.total is None:
                raise TypeError("bool() undefined when iterable == total == None")
            return True

    monkeypatch.setattr("helios.callbacks.tqdm", FakeTqdm)

    user_leg_end = []

    def on_end(ctx, points=None, trajectories=None):
        user_leg_end.append(ctx.leg_index)

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(
            num_threads=1, progressbar=ProgressBarStrategy.LEGS_TIME
        ),
        callbacks=(SurveyHook(point=HookPoint.LEG_END, callback=on_end),),
    )

    assert user_leg_end == [0, 1]
    assert len(bars) == 2
    assert len(closed) == 2
    assert [bar.unit for bar in closed] == ["leg", "s"]

    legs_bar = next(bar for bar in bars if bar.unit == "leg")
    time_bar = next(bar for bar in bars if bar.unit == "s")
    assert legs_bar.total == 2
    assert legs_bar.n == pytest.approx(2.0)
    assert time_bar.total is not None
    assert time_bar.total > 0
    assert time_bar.n > 0


def test_per_leg_progressbar_resets_on_each_leg(
    monkeypatch, tls_scanner, tripod, scene
):
    survey = _make_short_survey(tls_scanner, tripod, scene, legs=2)
    bars = []

    class FakeTqdm:
        def __init__(
            self,
            total=None,
            desc="",
            unit="it",
            bar_format=None,
            dynamic_ncols=True,
            position=0,
            leave=True,
        ):
            self.total = total
            self.desc = desc
            self.unit = unit
            self.bar_format = bar_format
            self.dynamic_ncols = dynamic_ncols
            self.position = position
            self.leave = leave
            self.n = 0.0
            self.n_before_first_update = {}
            bars.append(self)

        def update(self, delta):
            self.n_before_first_update.setdefault(self.desc, self.n)
            self.n += float(delta)

        def refresh(self):
            return None

        def close(self):
            return None

        def reset(self, total=None):
            self.total = total
            self.n = 0.0

        def set_description(self, desc):
            self.desc = desc

        def __bool__(self):
            if self.total is None:
                raise TypeError("bool() undefined when iterable == total == None")
            return True

    monkeypatch.setattr("helios.callbacks.tqdm", FakeTqdm)

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(
            num_threads=1, progressbar=ProgressBarStrategy.PER_LEG_TIME
        ),
    )

    legs_bar = next(bar for bar in bars if bar.unit == "leg")
    leg_time_bar = next(bar for bar in bars if bar.unit == "s")
    assert legs_bar.n == pytest.approx(2.0)
    assert leg_time_bar.total is not None
    assert leg_time_bar.total > 0
    assert leg_time_bar.n_before_first_update["Leg 1 time"] == pytest.approx(0.0)
    assert leg_time_bar.n_before_first_update["Leg 2 time"] == pytest.approx(0.0)


def test_per_leg_progressbar_handles_unknown_totals_without_bool(monkeypatch):
    class FakeTqdm:
        def __init__(
            self,
            total=None,
            desc="",
            unit="it",
            bar_format=None,
            dynamic_ncols=True,
            position=0,
            leave=True,
        ):
            self.total = total
            self.bar_format = bar_format
            self.n = 0.0

        def update(self, delta):
            self.n += float(delta)

        def refresh(self):
            return None

        def close(self):
            return None

        def reset(self, total=None):
            self.total = total
            self.n = 0.0

        def set_description(self, desc):
            return None

        def __bool__(self):
            if self.total is None:
                raise TypeError("bool() undefined when iterable == total == None")
            return True

    monkeypatch.setattr("helios.callbacks.tqdm", FakeTqdm)

    bars = callbacks_module._ProgressBarCallbacks(
        strategy=ProgressBarStrategy.PER_LEG_TIME,
        num_legs=1,
    )
    try:
        bars._time_bar.total = None
        bars._apply_event("leg_start", 0, 0.0, None, 0.0, None)
        bars._apply_event("time", 0, 0.5, None, 0.5, None)
        assert bars._time_bar.n == pytest.approx(0.5)
    finally:
        bars.close()


def test_set_bar_value_formats_seconds_with_two_decimals():
    class FakeBar:
        def __init__(self):
            self.total = None
            self.n = 0.0
            self.unit = "s"
            self.bar_format = None

        def reset(self, total=None):
            self.total = total
            self.n = 0.0

        def update(self, delta):
            self.n += float(delta)

        def refresh(self):
            return None

    bar = FakeBar()
    callbacks_module._ProgressBarCallbacks._set_bar_value(
        bar=bar,
        current_value=0.0056981950000000005,
        previous_value=0.0,
        total_value=0.0056981950000000005,
    )

    rendered = tqdm_std.format_meter(
        n=bar.n,
        total=bar.total,
        elapsed=0.1,
        unit=bar.unit,
        bar_format=bar.bar_format,
    )
    assert "0.01/0.01" in rendered
    assert "0.0056981950000000005" not in rendered


def test_set_bar_value_updates_when_total_changes():
    class FakeBar:
        def __init__(self):
            self.total = 0.1
            self.n = 0.1
            self.unit = "s"
            self.bar_format = None

        def reset(self, total=None):
            self.total = total
            self.n = 0.0

        def update(self, delta):
            self.n += float(delta)

        def refresh(self):
            return None

    bar = FakeBar()
    callbacks_module._ProgressBarCallbacks._set_bar_value(
        bar=bar,
        current_value=0.2,
        previous_value=0.1,
        total_value=0.2,
    )

    assert bar.total == pytest.approx(0.2)
    assert bar.n == pytest.approx(0.2)


def test_set_bar_value_keeps_current_value_when_total_becomes_known():
    class FakeBar:
        def __init__(self):
            self.total = None
            self.n = 7.0
            self.unit = "s"
            self.bar_format = None

        def reset(self, total=None):
            self.total = total
            self.n = 0.0

        def update(self, delta):
            self.n += float(delta)

        def refresh(self):
            return None

    bar = FakeBar()
    callbacks_module._ProgressBarCallbacks._set_bar_value(
        bar=bar,
        current_value=0.2,
        previous_value=0.0,
        total_value=0.3,
    )

    assert bar.total == pytest.approx(0.3)
    assert bar.n == pytest.approx(0.2)


def test_hook_context_exposes_elapsed_remaining_time(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene)
    seen = []

    def on_end(ctx, points=None, trajectories=None):
        seen.append(
            (
                ctx.elapsed_time_s,
                ctx.remaining_time_s,
                ctx.leg_elapsed_time_s,
                ctx.leg_remaining_time_s,
            )
        )

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(SurveyHook(point=HookPoint.LEG_END, callback=on_end),),
    )

    assert len(seen) == 1
    elapsed, remaining, leg_elapsed, leg_remaining = seen[0]
    assert elapsed >= 0.0
    assert remaining >= 0.0
    assert leg_elapsed >= 0.0
    assert leg_remaining >= 0.0


def test_leg_start_hook_resets_per_leg_time_state(tls_scanner, tripod, scene):
    survey = _make_short_survey(tls_scanner, tripod, scene, legs=2)
    seen = []

    def on_start(ctx, points=None, trajectories=None):
        seen.append(
            (
                ctx.leg_index,
                ctx.leg_progress,
                ctx.leg_elapsed_time_s,
                ctx.leg_remaining_time_s,
            )
        )

    survey.run(
        format=OutputFormat.NPY,
        execution_settings=ExecutionSettings(num_threads=1),
        callbacks=(SurveyHook(point=HookPoint.LEG_START, callback=on_start),),
    )

    assert seen == [
        (0, pytest.approx(0.0), pytest.approx(0.0), pytest.approx(0.0)),
        (1, pytest.approx(0.0), pytest.approx(0.0), pytest.approx(0.0)),
    ]

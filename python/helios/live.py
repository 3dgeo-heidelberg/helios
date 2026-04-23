from helios.callbacks import (
    SurveyHook,
    HookPoint,
    HookPayload,
    HookEndOfLegPolicy,
)
from helios.utils import extract_position
from helios.scene import StaticScene

import numpy as np
import threading

from dataclasses import dataclass
from typing import Optional, Any, Literal
import time

import vedo
from vedo import Mesh, Plotter, Points, Line

from collections import deque
from IPython import get_ipython


@dataclass(slots=True)
class LiveUpdate:
    ctx: Any
    measurements: Optional[np.ndarray]
    trajectories: Optional[np.ndarray]


class LiveProducer:
    def __init__(self, period: float = 0.1):
        self.period = float(period)
        self._lock = threading.Lock()
        self._has_data = threading.Event()
        self._pending: deque[LiveUpdate] = deque()
        self.closed = threading.Event()

    def callbacks(self):
        return (
            SurveyHook(
                point=HookPoint.SIM_TIME_PERIODIC,
                callback=self._on_hook,
                barrier=False,
                payload=HookPayload.SINCE_LAST,
                end_of_leg_policy=HookEndOfLegPolicy.FLUSH,
                sim_time=0.0,
                period=self.period,
            ),
        )

    def _on_hook(self, ctx, measurements, trajectories):
        if self.closed.is_set():
            return

        update = LiveUpdate(
            ctx=ctx,
            measurements=extract_position(measurements),
            trajectories=extract_position(trajectories),
        )

        with self._lock:
            self._pending.append(update)
            self._has_data.set()

    def drain(self) -> tuple[LiveUpdate, ...]:
        with self._lock:
            if not self._pending:
                self._has_data.clear()
                return ()
            updates = tuple(self._pending)
            self._pending.clear()
            self._has_data.clear()
            return updates

    def wait_for_data(self, timeout: float) -> bool:
        return self._has_data.wait(timeout=timeout)

    def close(self) -> None:
        self.closed.set()


class LiveViewer:
    """
    Streaming renderer that creates small persistent Vedo actors per chunk.
    This avoids repeatedly rebuilding one ever-growing polydata object.
    """

    def __init__(
        self,
        period: float = 0.1,
        measurement_point_size: float = 2.0,
        trajectory_point_size: float = 3.0,
        trajectory_line_width: float = 3.0,
        trajectory_style: Literal["points", "line"] = "points",
        frame_hz: float = 30.0,
        window_title: str = "Helios Live",
    ):
        if Plotter is None:
            raise ImportError(
                "There is a problem with the installed version of vedo. Please reinstall vedo with 'pip install vedo --force-reinstall'.",
            )

        self.period = float(period)
        self.measurement_point_size = float(measurement_point_size)
        self.trajectory_point_size = float(trajectory_point_size)
        self.trajectory_line_width = float(trajectory_line_width)
        self.trajectory_style = trajectory_style
        self.frame_interval_s = 1.0 / float(frame_hz)
        self.window_title = window_title

        self.scene: Optional[StaticScene] = None
        self.producer = LiveProducer(period=self.period)

        self._viewer_thread: Optional[threading.Thread] = None
        self._viewer_started = False
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._error: Optional[BaseException] = None

        self.plotter: Optional[Plotter] = None
        self._scene_actors: list[Any] = []

    def attach_to_survey(self, survey) -> None:
        if self.scene is None:
            self.scene = survey.scene

    def callbacks(self):
        return self.producer.callbacks()

    def start(self) -> None:
        if self._viewer_started:
            return
        if self.scene is None:
            raise RuntimeError("Call attach_to_survey(survey) before start().")

        self._stop_event.clear()
        self._ready_event.clear()
        self._error = None
        self._ensure_scene_actors()
        self._start_viewer_thread()
        self._viewer_started = True

        if not self._ready_event.wait(timeout=1.0):
            self._stop_event.set()
            self._stop_thread_event()
            raise RuntimeError("Viewer failed to start within timeout")

        if self._error is not None:
            self._stop_event.set()
            self._stop_thread_event()
            err = self._error
            self._error = None
            raise err

    def close_producer_input(self) -> None:
        self.producer.close()

    def _start_viewer_thread(self) -> None:
        self._viewer_thread = threading.Thread(
            target=self._viewer_main,
            name="helios-live-vedo",
            daemon=False,
        )
        self._viewer_thread.start()

    def _stop_thread_event(self) -> None:
        self._stop_event.set()
        if self._viewer_thread is not None:
            self._viewer_thread.join(timeout=0.5)
            self._viewer_thread = None

    def _window_is_closed(self) -> bool:
        if self.plotter is None:
            return True

        interactor = getattr(self.plotter, "interactor", None)
        if interactor is None or not hasattr(interactor, "GetDone"):
            return False

        try:
            return bool(interactor.GetDone())
        except Exception as exc:
            err = RuntimeError("Failed while checking Vedo window state")
            err.__cause__ = exc
            self._error = err
            self._stop_event.set()
            return True

    def _viewer_main(self) -> None:
        try:
            self._viewer_setup()
            self._ready_event.set()

            last_render = time.perf_counter()
            while not self._stop_event.is_set():
                if self._window_is_closed():
                    break

                processed = self._consume_pending()
                now = time.perf_counter()

                if processed > 0 or (now - last_render) >= self.frame_interval_s:
                    self._render_once()
                    last_render = now
                else:
                    self.producer.wait_for_data(timeout=0.005)

            self._consume_pending()
            if not self._window_is_closed():
                self._render_once()
        except BaseException as exc:
            self._error = exc
            self._ready_event.set()
        finally:
            self._viewer_close()
            self._viewer_started = False

    def _is_in_jupyter(self) -> bool:
        try:
            ip = get_ipython()
            if ip is None:
                return False

            shell_name = ip.__class__.__name__
            return shell_name == "ZMQInteractiveShell"
        except Exception:
            return False

    def _ensure_scene_actors(self) -> None:
        if self.scene is None:
            raise RuntimeError("Scene is not initialized")
        if not self._scene_actors:
            self._scene_actors = self._create_scene_actors()

    def _viewer_setup(self) -> None:
        if self.scene is None:
            raise RuntimeError("Scene is not initialized")
        vedo.settings.default_backend = "vtk"

        if self._is_in_jupyter() and hasattr(vedo, "embedWindow"):
            try:
                vedo.embedWindow("ipyvtk")
            except Exception:
                pass

        self.plotter = Plotter(
            title=self.window_title,
            axes=1,
            interactive=False,
            offscreen=False,
        )

        if self._scene_actors:
            self.plotter.show(*self._scene_actors, resetcam=True, interactive=False)
        else:
            self.plotter.show(interactive=False)

        self._process_events()

    def _create_scene_actors(
        self,
    ) -> list[Any]:
        if self.scene is None:
            return []

        actors: list[Any] = []
        for part in self.scene.scene_parts:
            buffers = part.get_visualization_buffers()

            triangle_vertices = np.asarray(buffers.triangle_vertices, dtype=np.float32)
            triangle_indices = np.asarray(buffers.triangle_indices, dtype=np.int32)
            if triangle_vertices.size > 0 and triangle_indices.size > 0:
                mesh = Mesh([triangle_vertices, triangle_indices]).alpha(0.25)
                mesh.pickable(False)
                actors.append(mesh)

            voxel_centers = np.asarray(buffers.voxel_centers, dtype=np.float32)
            if voxel_centers.size > 0:
                vox = Points(voxel_centers, r=2.0).c("gray").alpha(0.35).pickable(False)
                actors.append(vox)

        return actors

    def _consume_pending(self) -> int:
        if self.plotter is None:
            return 0

        processed = 0
        for update in self.producer.drain():
            if update.measurements is not None and update.measurements.size > 0:
                meas = Points(update.measurements, r=self.measurement_point_size)
                meas.c("red").pickable(False)
                self.plotter.add(meas)
                processed += 1

            if update.trajectories is not None and update.trajectories.size > 0:
                traj_xyz = update.trajectories
                if self.trajectory_style == "line":
                    traj = Line(traj_xyz, c="black", lw=self.trajectory_line_width)
                else:
                    traj = Points(traj_xyz, r=self.trajectory_point_size).c("black")
                traj.pickable(False)
                self.plotter.add(traj)
                processed += 1

        return processed

    def _render_once(self) -> bool:
        if self.plotter is None:
            return False
        self.plotter.render()
        self._process_events()
        return True

    def _process_events(self) -> None:
        if self.plotter is None:
            return
        interactor = getattr(self.plotter, "interactor", None)
        if interactor is not None and hasattr(interactor, "ProcessEvents"):
            try:
                interactor.ProcessEvents()
            except Exception:
                pass

    def _viewer_close(self) -> None:
        self.producer.close()
        if self.plotter is not None:
            try:
                close = getattr(self.plotter, "close", None)
                if callable(close):
                    close()
            finally:
                self.plotter = None
                self._scene_actors = []

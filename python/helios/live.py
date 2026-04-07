import queue
import numpy as np
import threading

from pydantic import BaseModel, PrivateAttr, Field
from dataclasses import dataclass
from typing import Optional, Any, Literal

from helios.callbacks import (
    SurveyHook,
    HookPoint,
    HookPayload,
    HookEndOfLegPolicy,
)
from helios.utils import (
    extract_position,
    meas_dtype,
    traj_dtype,
)
from helios.scene import StaticScene, ScenePart

import time


@dataclass(slots=True)
class LiveUpdate:
    ctx: Any
    measurements: Optional[np.ndarray]
    trajectories: Optional[np.ndarray]
    dropped_count: int = 0


class LiveProducer:
    def __init__(self, period=0.1):
        self.period = float(period)
        self._lock = threading.Lock()
        self._latest: Optional[LiveUpdate] = None
        self.closed = threading.Event()
        self.dropped_updates = 0

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
            measurements=measurements,
            trajectories=trajectories,
        )

        with self._lock:
            if self._latest is not None:
                self.dropped_updates += 1
            self._latest = update

    def poll(self) -> Optional[LiveUpdate]:
        if self.closed.is_set():
            return None

        with self._lock:
            update = self._latest
            self._latest = None
            return update

    def close(self) -> None:
        self.closed.set()


class LiveAccumulator:
    def __init__(self):
        self.measurements = np.empty((0,), dtype=meas_dtype)
        self.trajectories = np.empty((0,), dtype=traj_dtype)
        self.has_new_measurements = False
        self.has_new_trajectories = False

    def consume(self, update: LiveUpdate) -> None:
        if update.measurements is not None and len(update.measurements) > 0:
            if len(self.measurements) == 0:
                self.measurements = update.measurements
            else:
                self.measurements = np.concatenate(
                    (self.measurements, update.measurements), axis=0
                )
            self.has_new_measurements = True
        if update.trajectories is not None and len(update.trajectories) > 0:
            if len(self.trajectories) == 0:
                self.trajectories = update.trajectories
            else:
                self.trajectories = np.concatenate(
                    (self.trajectories, update.trajectories), axis=0
                )
            self.has_new_trajectories = True

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, bool, bool]:
        measurement_xyz = extract_position(self.measurements)
        trajectory_xyz = extract_position(self.trajectories)
        has_new_measurements = self.has_new_measurements
        has_new_trajectories = self.has_new_trajectories
        self.has_new_measurements = False
        self.has_new_trajectories = False
        return (
            measurement_xyz,
            trajectory_xyz,
            has_new_measurements,
            has_new_trajectories,
        )


class Open3DLiveViewer:
    def __init__(
        self,
        period: float = 0.1,
        trajectory_style: Literal["points", "line"] = "points",
    ):
        self.period = float(period)
        self.scene = None
        self.producer = LiveProducer(
            period=self.period,
        )
        self.accumulator = LiveAccumulator()
        self.trajectory_style = trajectory_style

        self._viewer_thread: Optional[threading.Thread] = None
        self._viewer_started = False
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._measurement_added = False
        self._trajectory_added = False
        self._error: Optional[BaseException] = None

        self.visualizer: Optional[Any] = None
        self.measurement_cloud: Optional[Any] = None
        self.trajectory_cloud: Optional[Any] = None
        self._force_geometry_refresh = True

        try:
            import open3d as o3d
        except ImportError as exc:
            raise ImportError(
                "Open3D is required for Open3DLiveViewer. "
                "Install it with `pip install open3d`."
            ) from exc

        self._o3d = o3d

    def attach_to_survey(self, survey) -> None:
        if self.scene is None:
            self.scene = survey.scene

    def callbacks(self):
        return self.producer.callbacks()

    def start(self, timeout=5.0) -> None:
        if self._viewer_started:
            return
        if self.scene is None or self.producer is None or self.accumulator is None:
            raise RuntimeError(
                "Open3DLiveViewer is not fully initialized. "
                "Call attach to survey(survey) before start()."
            )
        self._stop_event.clear()
        self._ready_event.clear()
        self._error = None
        self._start_viewer_thread()
        self._viewer_started = True

        if not self._ready_event.wait(timeout=timeout):
            self._stop_event.set()
            raise RuntimeError("Viewer failed to start within timeout")
        if self._error is not None:
            err = self._error
            self._error = None
            raise err

    def stop(self) -> None:
        "Stop the viewer thread and close resources."
        if not self._viewer_started:
            return
        self._stop_viewer_thread()
        self._viewer_started = False
        if self._error is not None:
            err = self._error
            self._error = None
            raise err

    def _start_viewer_thread(self) -> None:
        self._viewer_thread = threading.Thread(
            target=self._viewer_main,
            name="helios-live-open3d",
            daemon=False,
        )
        self._viewer_thread.start()

    def _stop_viewer_thread(self) -> None:
        self._stop_event.set()
        if self._viewer_thread is not None:
            self._viewer_thread.join(timeout=5.0)
            self._viewer_thread = None

    def _viewer_main(self) -> None:
        try:
            self._viewer_setup()
            self._ready_event.set()
            while not self._stop_event.is_set():
                self._consume_pending()
                if not self._render_once():
                    break
                time.sleep(0.01)

            self._consume_pending()
            self._force_geometry_refresh = True
            self._render_once()
        except BaseException as exc:
            self._error = exc
            self._ready_event.set()
        finally:
            self._viewer_close()

    def _viewer_setup(self) -> None:
        o3d = self._o3d
        self.visualizer = o3d.visualization.Visualizer()
        self.visualizer.create_window(window_name="Helios Live")
        render_option = self.visualizer.get_render_option()
        render_option.mesh_show_back_face = True
        self.measurement_cloud = o3d.geometry.PointCloud()
        if self.trajectory_style == "points":
            self.trajectory_cloud = o3d.geometry.PointCloud()
        elif self.trajectory_style == "line":
            self.trajectory_cloud = o3d.geometry.LineSet()
        else:
            raise ValueError(
                f"Invalid trajectory_style: {self.trajectory_style}. Choose 'points' or 'line'."
            )
        self._measurement_added = False
        self._trajectory_added = False
        self._add_scene_geometry()
        self.visualizer.poll_events()
        self.visualizer.update_renderer()

    def _create_o3d_geometries(self, buffers) -> list:
        """Create Open3D geometries from the provided buffers.
        This is a helper function to convert the raw buffers from ScenePart
        into Open3D geometries."""
        o3d = self._o3d
        geometries = []
        triangle_vertices = np.asarray(buffers.triangle_vertices, dtype=np.float64)
        triangle_indices = np.asarray(buffers.triangle_indices, dtype=np.int32)
        voxel_centers = np.asarray(buffers.voxel_centers, dtype=np.float64)
        if triangle_vertices.size > 0 and triangle_indices.size > 0:
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(triangle_vertices)
            mesh.triangles = o3d.utility.Vector3iVector(triangle_indices)
            mesh.compute_vertex_normals()
            geometries.append(mesh)
        if voxel_centers.size > 0:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(voxel_centers)
            geometries.append(pcd)
        return geometries

    def _add_scene_geometry(self) -> None:
        """Assumes scene.scene_parts contains objects with .o3dGeometry
        already built/available.
        """
        if self.visualizer is None or self.scene is None:
            raise RuntimeError("Viewer or scene not initialized!")
        for part in self.scene.scene_parts:
            buffers = part.get_visualization_buffers()
            geometries = self._create_o3d_geometries(buffers)
            for geom in geometries:
                self.visualizer.add_geometry(geom)

    def _consume_pending(self) -> int:
        processed = 0
        while True:
            update = self.producer.poll()
            if update is None:
                break
            self.accumulator.consume(update)
            processed += 1
        return processed

    def _render_once(self) -> bool:
        if self.visualizer is None:
            return False
        o3d = self._o3d
        (
            measurement_xyz,
            trajectory_xyz,
            has_new_measurements,
            has_new_trajectories,
        ) = self.accumulator.snapshot()
        if has_new_measurements:
            self.measurement_cloud.points = o3d.utility.Vector3dVector(measurement_xyz)
            if not self._measurement_added:
                self.visualizer.add_geometry(self.measurement_cloud)
                self._measurement_added = True
            else:
                self.visualizer.update_geometry(self.measurement_cloud)
        if has_new_trajectories:
            if self.trajectory_style == "points":
                self.trajectory_cloud.points = o3d.utility.Vector3dVector(
                    trajectory_xyz
                )
            else:
                self.trajectory_cloud.points = o3d.utility.Vector3dVector(
                    trajectory_xyz
                )
                if len(trajectory_xyz) >= 2:
                    lines = np.column_stack(
                        (
                            np.arange(len(trajectory_xyz) - 1, dtype=np.int32),
                            np.arange(1, len(trajectory_xyz), dtype=np.int32),
                        )
                    )
                    self.trajectory_cloud.lines = o3d.utility.Vector2iVector(lines)
                else:
                    self.trajectory_cloud.lines = o3d.utility.Vector2iVector(
                        np.empty((0, 2), dtype=np.int32)
                    )

            if not self._trajectory_added:
                if self.trajectory_style == "points" or len(trajectory_xyz) >= 2:
                    self.visualizer.add_geometry(self.trajectory_cloud)
                    self._trajectory_added = True
            else:
                self.visualizer.update_geometry(self.trajectory_cloud)

        alive = self.visualizer.poll_events()
        self.visualizer.update_renderer()
        return alive

    def _viewer_close(self) -> None:
        if self.producer is not None:
            self.producer.close()
        if self.visualizer is not None:
            try:
                self.visualizer.destroy_window()
            finally:
                self.visualizer = None
                self.measurement_cloud = None
                self.trajectory_cloud = None

    def close_producer_input(self) -> None:
        """Called when simulation is done.
        Stops producer but keeps viewer alive.
        """
        if self.producer is not None:
            self.producer.close()

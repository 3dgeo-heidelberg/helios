from types import SimpleNamespace
import sys

import numpy as np
import pytest

import helios.survey as survey_module
from helios.live import LiveAccumulator, LiveProducer, LiveUpdate, Open3DLiveViewer
from helios.settings import OutputFormat
from helios.utils import extract_position, meas_dtype, traj_dtype


def _make_measurements(positions):
    arr = np.zeros((len(positions),), dtype=meas_dtype)
    for i, pos in enumerate(positions):
        arr[i]["position"] = pos
    return arr


def _make_trajectories(positions):
    arr = np.zeros((len(positions),), dtype=traj_dtype)
    for i, pos in enumerate(positions):
        arr[i]["position"] = pos
    return arr


class _FakeVector:
    def __init__(self, array):
        self.array = np.asarray(array)


class _FakePointCloud:
    def __init__(self):
        self.points = None


class _FakeLineSet:
    def __init__(self):
        self.points = None
        self.lines = None


class _FakeTriangleMesh:
    def __init__(self):
        self.vertices = None
        self.triangles = None
        self.compute_vertex_normals_called = False

    def compute_vertex_normals(self):
        self.compute_vertex_normals_called = True


class _FakeRenderOption:
    def __init__(self):
        self.mesh_show_back_face = False


class _FakeVisualizer:
    def __init__(self, poll_events_result=True):
        self.poll_events_result = poll_events_result
        self.added = []
        self.updated = []
        self.create_window_calls = []
        self.destroy_window_called = False
        self.render_option = _FakeRenderOption()
        self.poll_events_calls = 0
        self.update_renderer_calls = 0

    def create_window(self, window_name):
        self.create_window_calls.append(window_name)

    def get_render_option(self):
        return self.render_option

    def add_geometry(self, geometry):
        self.added.append(geometry)

    def update_geometry(self, geometry):
        self.updated.append(geometry)

    def poll_events(self):
        self.poll_events_calls += 1
        return self.poll_events_result

    def update_renderer(self):
        self.update_renderer_calls += 1

    def destroy_window(self):
        self.destroy_window_called = True


class _FakeO3DModule:
    def __init__(self, visualizer=None):
        self._visualizer = visualizer or _FakeVisualizer()
        self.visualization = SimpleNamespace(Visualizer=lambda: self._visualizer)
        self.geometry = SimpleNamespace(
            PointCloud=_FakePointCloud,
            LineSet=_FakeLineSet,
            TriangleMesh=_FakeTriangleMesh,
        )
        self.utility = SimpleNamespace(
            Vector3dVector=lambda arr: _FakeVector(arr),
            Vector2iVector=lambda arr: _FakeVector(arr),
            Vector3iVector=lambda arr: _FakeVector(arr),
        )


@pytest.fixture
def fake_o3d(monkeypatch):
    fake = _FakeO3DModule()
    monkeypatch.setitem(sys.modules, "open3d", fake)
    return fake


def test_extract_position_returns_position_field():
    arr = _make_measurements([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])

    result = extract_position(arr)

    np.testing.assert_array_equal(
        result,
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
    )


def test_live_update_defaults():
    update = LiveUpdate(ctx="ctx", measurements=None, trajectories=None)

    assert update.ctx == "ctx"
    assert update.measurements is None
    assert update.trajectories is None
    assert update.dropped_count == 0


def test_live_producer_callbacks_configured_as_expected():
    producer = LiveProducer(period=0.25)

    callbacks = producer.callbacks()

    assert len(callbacks) == 1
    hook = callbacks[0]
    assert hook.callback == producer._on_hook
    assert hook.period == 0.25


def test_live_producer_poll_returns_none_when_empty():
    producer = LiveProducer()

    assert producer.poll() is None


def test_live_producer_keeps_latest_and_clears_slot():
    producer = LiveProducer()
    measurements = _make_measurements([(1.0, 2.0, 3.0)])
    trajectories = _make_trajectories([(4.0, 5.0, 6.0)])

    producer._on_hook("ctx-1", measurements, trajectories)

    update = producer.poll()

    assert update is not None
    assert update.ctx == "ctx-1"
    np.testing.assert_array_equal(update.measurements, measurements)
    np.testing.assert_array_equal(update.trajectories, trajectories)
    assert producer.poll() is None


def test_live_producer_overwrites_unpolled_update_and_counts_drop():
    producer = LiveProducer()

    producer._on_hook("first", _make_measurements([(1.0, 1.0, 1.0)]), None)
    producer._on_hook("second", _make_measurements([(2.0, 2.0, 2.0)]), None)

    update = producer.poll()

    assert producer.dropped_updates == 1
    assert update is not None
    assert update.ctx == "second"
    np.testing.assert_array_equal(
        update.measurements["position"],
        np.array([[2.0, 2.0, 2.0]]),
    )


def test_live_producer_close_prevents_new_updates_and_polling():
    producer = LiveProducer()
    producer.close()

    producer._on_hook("ctx", _make_measurements([(1.0, 2.0, 3.0)]), None)

    assert producer.closed.is_set()
    assert producer.poll() is None


def test_live_accumulator_snapshot_empty_is_stable():
    accumulator = LiveAccumulator()

    measurement_xyz, trajectory_xyz, has_new_measurements, has_new_trajectories = (
        accumulator.snapshot()
    )

    assert measurement_xyz.shape == (0, 3)
    assert trajectory_xyz.shape == (0, 3)
    assert has_new_measurements is False
    assert has_new_trajectories is False


def test_live_accumulator_consumes_measurements_only():
    accumulator = LiveAccumulator()

    accumulator.consume(
        LiveUpdate(
            ctx="ctx",
            measurements=_make_measurements([(1.0, 2.0, 3.0)]),
            trajectories=None,
        )
    )

    measurement_xyz, trajectory_xyz, has_new_measurements, has_new_trajectories = (
        accumulator.snapshot()
    )

    np.testing.assert_array_equal(
        measurement_xyz,
        np.array([[1.0, 2.0, 3.0]]),
    )
    assert trajectory_xyz.shape == (0, 3)
    assert has_new_measurements is True
    assert has_new_trajectories is False


def test_live_accumulator_consumes_trajectories_only():
    accumulator = LiveAccumulator()

    accumulator.consume(
        LiveUpdate(
            ctx="ctx",
            measurements=None,
            trajectories=_make_trajectories([(10.0, 20.0, 30.0)]),
        )
    )

    measurement_xyz, trajectory_xyz, has_new_measurements, has_new_trajectories = (
        accumulator.snapshot()
    )

    assert measurement_xyz.shape == (0, 3)
    np.testing.assert_array_equal(
        trajectory_xyz,
        np.array([[10.0, 20.0, 30.0]]),
    )
    assert has_new_measurements is False
    assert has_new_trajectories is True


def test_live_accumulator_appends_and_snapshot_resets_flags():
    accumulator = LiveAccumulator()

    accumulator.consume(
        LiveUpdate(
            ctx="ctx-1",
            measurements=_make_measurements([(1.0, 2.0, 3.0)]),
            trajectories=_make_trajectories([(10.0, 20.0, 30.0)]),
        )
    )
    accumulator.consume(
        LiveUpdate(
            ctx="ctx-2",
            measurements=_make_measurements([(4.0, 5.0, 6.0)]),
            trajectories=_make_trajectories([(40.0, 50.0, 60.0)]),
        )
    )

    measurement_xyz, trajectory_xyz, has_new_measurements, has_new_trajectories = (
        accumulator.snapshot()
    )

    np.testing.assert_array_equal(
        measurement_xyz,
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
    )
    np.testing.assert_array_equal(
        trajectory_xyz,
        np.array([[10.0, 20.0, 30.0], [40.0, 50.0, 60.0]]),
    )
    assert has_new_measurements is True
    assert has_new_trajectories is True
    assert accumulator.has_new_measurements is False
    assert accumulator.has_new_trajectories is False


def test_resolve_live_session_returns_none_for_false_and_none():
    assert survey_module._resolve_live_session(False) is None
    assert survey_module._resolve_live_session(None) is None


def test_resolve_live_session_true_creates_viewer(fake_o3d):
    live_session = survey_module._resolve_live_session(True)

    assert isinstance(live_session, Open3DLiveViewer)


def test_resolve_live_session_existing_viewer_is_returned(fake_o3d):
    viewer = Open3DLiveViewer()

    assert survey_module._resolve_live_session(viewer) is viewer


def test_resolve_live_session_rejects_invalid_value():
    with pytest.raises(
        TypeError, match="live must be False, True, or Open3DLiveViewer instance"
    ):
        survey_module._resolve_live_session("bad")


def test_open3d_live_viewer_attach_to_survey_sets_scene_once(fake_o3d):
    viewer = Open3DLiveViewer()
    survey1 = SimpleNamespace(scene="scene-1")
    survey2 = SimpleNamespace(scene="scene-2")

    viewer.attach_to_survey(survey1)
    viewer.attach_to_survey(survey2)

    assert viewer.scene == "scene-1"


def test_open3d_live_viewer_callbacks_delegate_to_producer(fake_o3d, monkeypatch):
    viewer = Open3DLiveViewer()
    expected = ("hook-1",)

    monkeypatch.setattr(viewer.producer, "callbacks", lambda: expected)

    assert viewer.callbacks() == expected


def test_open3d_live_viewer_start_requires_attached_scene(fake_o3d):
    viewer = Open3DLiveViewer()

    with pytest.raises(RuntimeError, match="not fully initialized"):
        viewer.start()


def test_viewer_setup_creates_geometry_and_configures_rendering(fake_o3d):
    viewer = Open3DLiveViewer(trajectory_style="line")
    viewer.scene = SimpleNamespace(scene_parts=[])

    viewer._viewer_setup()

    assert isinstance(viewer.visualizer, _FakeVisualizer)
    assert isinstance(viewer.measurement_cloud, _FakePointCloud)
    assert isinstance(viewer.trajectory_cloud, _FakeLineSet)
    assert viewer.visualizer.create_window_calls == ["Helios Live"]
    assert viewer.visualizer.render_option.mesh_show_back_face is True


def test_create_o3d_geometries_returns_mesh_and_point_cloud(fake_o3d):
    viewer = Open3DLiveViewer()
    buffers = SimpleNamespace(
        triangle_vertices=np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            dtype=np.float64,
        ),
        triangle_indices=np.array([[0, 1, 2]], dtype=np.int32),
        voxel_centers=np.array([[5.0, 5.0, 5.0]], dtype=np.float64),
    )

    geometries = viewer._create_o3d_geometries(buffers)

    assert len(geometries) == 2
    assert isinstance(geometries[0], _FakeTriangleMesh)
    assert geometries[0].compute_vertex_normals_called is True
    assert isinstance(geometries[1], _FakePointCloud)


def test_render_once_line_style_builds_segments(fake_o3d):
    viewer = Open3DLiveViewer(trajectory_style="line")
    viewer.visualizer = _FakeVisualizer()
    viewer.measurement_cloud = _FakePointCloud()
    viewer.trajectory_cloud = _FakeLineSet()
    viewer.accumulator = SimpleNamespace(
        snapshot=lambda: (
            np.empty((0, 3)),
            np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [2.0, 2.0, 2.0]]),
            False,
            True,
        )
    )

    alive = viewer._render_once()

    assert alive is True
    assert viewer._trajectory_added is True
    np.testing.assert_array_equal(
        viewer.trajectory_cloud.lines.array,
        np.array([[0, 1], [1, 2]], dtype=np.int32),
    )


def test_viewer_close_closes_producer_and_releases_visualizer(fake_o3d):
    viewer = Open3DLiveViewer()
    viewer.visualizer = _FakeVisualizer()
    viewer.measurement_cloud = _FakePointCloud()
    viewer.trajectory_cloud = _FakePointCloud()

    viewer._viewer_close()

    assert viewer.producer.closed.is_set()
    assert viewer.visualizer is None
    assert viewer.measurement_cloud is None
    assert viewer.trajectory_cloud is None


def test_survey_run_with_live_viewer_starts_and_closes_input(survey, monkeypatch):
    calls = []

    class _FakeViewer:
        def attach_to_survey(self, survey_obj):
            calls.append(("attach", survey_obj))

        def callbacks(self):
            return ()

        def start(self):
            calls.append(("start", None))

        def close_producer_input(self):
            calls.append(("close_producer_input", None))

    viewer = _FakeViewer()
    monkeypatch.setattr(survey_module, "_resolve_live_session", lambda live: viewer)

    points, trajectory = survey.run(format=OutputFormat.NPY, live=True)

    assert points.shape[0] > 0
    assert trajectory.shape[0] > 0
    assert calls == [
        ("attach", survey),
        ("start", None),
        ("close_producer_input", None),
    ]


def test_survey_run_closes_live_input_when_playback_fails(survey, monkeypatch):
    calls = []

    class _FakeViewer:
        def attach_to_survey(self, survey_obj):
            calls.append(("attach", survey_obj))

        def callbacks(self):
            return ()

        def start(self):
            calls.append(("start", None))

        def close_producer_input(self):
            calls.append(("close_producer_input", None))

    viewer = _FakeViewer()
    monkeypatch.setattr(survey_module, "_resolve_live_session", lambda live: viewer)
    monkeypatch.setattr(
        survey_module,
        "_start_playback_interruptible",
        lambda playback: (_ for _ in ()).throw(RuntimeError("playback failed")),
    )

    with pytest.raises(RuntimeError, match="playback failed"):
        survey.run(format=OutputFormat.NPY, live=True)

    assert calls == [
        ("attach", survey),
        ("start", None),
        ("close_producer_input", None),
    ]

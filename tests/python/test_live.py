from __future__ import annotations

import importlib
import sys
import types
from types import SimpleNamespace

import numpy as np
import pytest

from helios.settings import OutputFormat
from helios.utils import extract_position, meas_dtype, traj_dtype
import inspect


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


class _FakeInteractor:
    def __init__(self, done: bool = False):
        self.done = done
        self.process_events_calls = 0

    def GetDone(self):
        return self.done

    def ProcessEvents(self):
        self.process_events_calls += 1


class _FakeActor:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.alpha_value = None
        self.color_value = kwargs.get("c")
        self.pickable_value = None
        self.lw_value = kwargs.get("lw")
        self.r_value = kwargs.get("r", None)

    def alpha(self, value):
        self.alpha_value = value
        return self

    def c(self, value):
        self.color_value = value
        return self

    def pickable(self, value):
        self.pickable_value = value
        return self


class _FakePoints(_FakeActor):
    pass


class _FakeLine(_FakeActor):
    pass


class _FakeMesh(_FakeActor):
    pass


class _FakePlotter:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.shown = []
        self.added = []
        self.render_calls = 0
        self.close_calls = 0
        self.interactor = _FakeInteractor(done=False)

    def show(self, *actors, **kwargs):
        self.shown.extend(actors)
        return self

    def add(self, actor):
        self.added.append(actor)
        return self

    def render(self):
        self.render_calls += 1
        return self

    def close(self):
        self.close_calls += 1


def _import_modules_with_fake_vedo(monkeypatch):
    fake_plotter = _FakePlotter()

    fake_vedo = SimpleNamespace(
        Plotter=lambda *args, **kwargs: fake_plotter,
        Points=_FakePoints,
        Mesh=_FakeMesh,
        Line=_FakeLine,
        settings=SimpleNamespace(default_backend=None),
        embedWindow=lambda *args, **kwargs: None,
    )

    fake_ipython = SimpleNamespace(get_ipython=lambda: None)

    monkeypatch.setitem(sys.modules, "vedo", fake_vedo)
    monkeypatch.setitem(sys.modules, "IPython", fake_ipython)

    sys.modules.pop("helios.live", None)
    sys.modules.pop("helios.survey", None)

    live_module = importlib.import_module("helios.live")
    survey_module = importlib.import_module("helios.survey")

    return live_module, survey_module, fake_plotter


def test_extract_position_returns_position_field():
    arr = _make_measurements([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])

    result = extract_position(arr)

    np.testing.assert_array_equal(
        result,
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
    )


def test_extract_position_returns_none_for_empty_input():
    assert extract_position(None) is None
    assert extract_position(np.zeros((0,), dtype=meas_dtype)) is None


def test_live_update_keeps_payload():
    live_module, _, _ = _import_modules_with_fake_vedo(pytest.MonkeyPatch())
    update = live_module.LiveUpdate(
        ctx="ctx",
        measurements=None,
        trajectories=None,
    )

    assert update.ctx == "ctx"
    assert update.measurements is None
    assert update.trajectories is None


def test_live_producer_callbacks_configured_as_expected(monkeypatch):
    live_module, _, _ = _import_modules_with_fake_vedo(monkeypatch)
    producer = live_module.LiveProducer(period=0.25)

    callbacks = producer.callbacks()

    assert len(callbacks) == 1
    hook = callbacks[0]
    assert hook.callback == producer._on_hook
    assert hook.period == 0.25


def test_live_producer_drain_returns_empty_tuple_when_empty(monkeypatch):
    live_module, _, _ = _import_modules_with_fake_vedo(monkeypatch)
    producer = live_module.LiveProducer()

    assert producer.drain() == ()


def test_live_producer_appends_and_drains_all_updates(monkeypatch):
    live_module, _, _ = _import_modules_with_fake_vedo(monkeypatch)
    producer = live_module.LiveProducer()

    measurements = _make_measurements([(1.0, 2.0, 3.0)])
    trajectories = _make_trajectories([(4.0, 5.0, 6.0)])

    producer._on_hook("ctx-1", measurements, trajectories)

    updates = producer.drain()

    assert len(updates) == 1
    update = updates[0]
    assert update.ctx == "ctx-1"
    np.testing.assert_array_equal(update.measurements, np.array([[1.0, 2.0, 3.0]]))
    np.testing.assert_array_equal(update.trajectories, np.array([[4.0, 5.0, 6.0]]))
    assert producer.drain() == ()


def test_live_producer_wait_for_data_is_set_after_hook(monkeypatch):
    live_module, _, _ = _import_modules_with_fake_vedo(monkeypatch)
    producer = live_module.LiveProducer()

    assert producer.wait_for_data(0.0) is False

    producer._on_hook("ctx", _make_measurements([(1.0, 2.0, 3.0)]), None)

    assert producer.wait_for_data(0.0) is True
    assert producer.drain() != ()
    assert producer.wait_for_data(0.0) is False


def test_live_producer_close_prevents_new_updates(monkeypatch):
    live_module, _, _ = _import_modules_with_fake_vedo(monkeypatch)
    producer = live_module.LiveProducer()
    producer.close()

    producer._on_hook("ctx", _make_measurements([(1.0, 2.0, 3.0)]), None)

    assert producer.closed.is_set()
    assert producer.drain() == ()


def test_resolve_live_session_returns_none_for_false_and_none(monkeypatch):
    _, survey_module, _ = _import_modules_with_fake_vedo(monkeypatch)

    assert survey_module._resolve_live_session(False) is None
    assert survey_module._resolve_live_session(None) is None


def test_resolve_live_session_true_creates_viewer(monkeypatch):
    live_module, survey_module, _ = _import_modules_with_fake_vedo(monkeypatch)

    live_session = survey_module._resolve_live_session(True)

    assert isinstance(live_session, live_module.LiveViewer)


def test_resolve_live_session_existing_viewer_is_returned(monkeypatch):
    live_module, survey_module, _ = _import_modules_with_fake_vedo(monkeypatch)
    viewer = live_module.LiveViewer()

    assert survey_module._resolve_live_session(viewer) is viewer


def test_resolve_live_session_rejects_invalid_value(monkeypatch):
    _, survey_module, _ = _import_modules_with_fake_vedo(monkeypatch)

    with pytest.raises(
        TypeError, match="live must be False, True, or LiveViewer instance"
    ):
        survey_module._resolve_live_session("bad")


def test_live_viewer_attach_to_survey_sets_scene_once(monkeypatch):
    live_module, _, _ = _import_modules_with_fake_vedo(monkeypatch)
    viewer = live_module.LiveViewer()
    survey1 = SimpleNamespace(scene="scene-1")
    survey2 = SimpleNamespace(scene="scene-2")

    viewer.attach_to_survey(survey1)
    viewer.attach_to_survey(survey2)

    assert viewer.scene == "scene-1"


def test_ensure_scene_actors_builds_static_scene(monkeypatch):
    live_module, _, _ = _import_modules_with_fake_vedo(monkeypatch)

    viewer = live_module.LiveViewer()
    viewer.scene = SimpleNamespace(
        scene_parts=[
            SimpleNamespace(
                get_visualization_buffers=lambda: SimpleNamespace(
                    triangle_vertices=np.array(
                        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                        dtype=np.float32,
                    ),
                    triangle_indices=np.array([[0, 1, 2]], dtype=np.int32),
                    voxel_centers=np.array([[5.0, 5.0, 5.0]], dtype=np.float32),
                )
            )
        ]
    )

    viewer._ensure_scene_actors()

    assert len(viewer._scene_actors) == 2
    assert isinstance(viewer._scene_actors[0], _FakeMesh)
    assert isinstance(viewer._scene_actors[1], _FakePoints)


def test_viewer_setup_uses_prebuilt_static_actors(monkeypatch):
    live_module, _, fake_plotter = _import_modules_with_fake_vedo(monkeypatch)

    viewer = live_module.LiveViewer()
    viewer.scene = SimpleNamespace(scene_parts=[])
    viewer._scene_actors = [_FakeMesh(), _FakePoints(np.array([[1.0, 2.0, 3.0]]))]

    viewer._viewer_setup()

    assert viewer.plotter is fake_plotter
    assert fake_plotter.shown == viewer._scene_actors


def test_create_scene_actors_returns_mesh_and_points(monkeypatch):
    live_module, _, _ = _import_modules_with_fake_vedo(monkeypatch)

    viewer = live_module.LiveViewer()
    viewer.scene = SimpleNamespace(
        scene_parts=[
            SimpleNamespace(
                get_visualization_buffers=lambda: SimpleNamespace(
                    triangle_vertices=np.array(
                        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                        dtype=np.float32,
                    ),
                    triangle_indices=np.array([[0, 1, 2]], dtype=np.int32),
                    voxel_centers=np.array([[5.0, 5.0, 5.0]], dtype=np.float32),
                )
            )
        ]
    )

    actors = viewer._create_scene_actors()

    assert len(actors) == 2
    assert isinstance(actors[0], _FakeMesh)
    assert isinstance(actors[1], _FakePoints)


def test_consume_pending_adds_measurements_and_trajectory_points(monkeypatch):
    live_module, _, fake_plotter = _import_modules_with_fake_vedo(monkeypatch)

    viewer = live_module.LiveViewer(trajectory_style="points")
    viewer.plotter = fake_plotter

    producer = live_module.LiveProducer()
    producer._on_hook(
        "ctx",
        _make_measurements([(1.0, 2.0, 3.0)]),
        _make_trajectories([(4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]),
    )
    viewer.producer = producer

    processed = viewer._consume_pending()

    assert processed == 2
    assert len(fake_plotter.added) == 2
    assert isinstance(fake_plotter.added[0], _FakePoints)
    assert isinstance(fake_plotter.added[1], _FakePoints)
    assert fake_plotter.added[0].color_value == "red"
    assert fake_plotter.added[1].color_value == "black"


def test_consume_pending_creates_line_when_requested(monkeypatch):
    live_module, _, fake_plotter = _import_modules_with_fake_vedo(monkeypatch)

    viewer = live_module.LiveViewer(trajectory_style="line")
    viewer.plotter = fake_plotter

    producer = live_module.LiveProducer()
    producer._on_hook(
        "ctx",
        None,
        _make_trajectories([(4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]),
    )
    viewer.producer = producer

    processed = viewer._consume_pending()

    assert processed == 1
    assert len(fake_plotter.added) == 1
    assert isinstance(fake_plotter.added[0], _FakeLine)
    assert fake_plotter.added[0].color_value == "black"


def test_window_is_closed_uses_interactor_done(monkeypatch):
    live_module, _, fake_plotter = _import_modules_with_fake_vedo(monkeypatch)

    viewer = live_module.LiveViewer()
    viewer.plotter = fake_plotter

    fake_plotter.interactor.done = True

    assert viewer._window_is_closed() is True


def test_viewer_close_closes_producer_and_releases_plotter(monkeypatch):
    live_module, _, fake_plotter = _import_modules_with_fake_vedo(monkeypatch)

    viewer = live_module.LiveViewer()
    viewer.plotter = fake_plotter

    viewer._viewer_close()

    assert viewer.producer.closed.is_set()
    assert viewer.plotter is None
    assert fake_plotter.close_calls == 1


def test_survey_run_closes_live_input(monkeypatch, survey):
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

    def _fake_start_playback_interruptible(playback):
        survey.scanner._cpp_object.all_measurements = np.zeros((1,), dtype=meas_dtype)
        survey.scanner._cpp_object.all_trajectories = np.zeros((1,), dtype=traj_dtype)
        survey.scanner._cpp_object.all_measurements[0]["position"] = (0.0, 0.0, 0.0)
        survey.scanner._cpp_object.all_trajectories[0]["position"] = (0.0, 0.0, 0.0)

    run_fn = inspect.unwrap(type(survey).run)

    monkeypatch.setitem(
        run_fn.__globals__,
        "_resolve_live_session",
        lambda live: _FakeViewer(),
    )
    monkeypatch.setitem(
        run_fn.__globals__,
        "_start_playback_interruptible",
        _fake_start_playback_interruptible,
    )

    result = survey.run(format=OutputFormat.NPY, live=True)

    assert result is not None
    assert calls == [
        ("attach", survey),
        ("start", None),
        ("close_producer_input", None),
    ]

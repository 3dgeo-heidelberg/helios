import helios
from helios import survey
from helios.platforms import (
    Platform,
    DynamicPlatformSettings,
    load_traj_csv,
    tripod,
    simple_linearpath,
    StaticPlatformSettings,
)
from helios.scanner import Scanner, riegl_lms_q560, riegl_vz_1000, riegl_vz_400
from helios.scene import DynamicScene, StaticScene, ScenePart
from helios.settings import ExecutionSettings, OutputFormat, ProgressBarStrategy
from helios.survey import *
from helios.utils import meas_dtype, set_rng_seed, traj_dtype
from helios import HeliosException

import copy
import gc
import laspy
import numpy as np
from numpy.lib.recfunctions import unstructured_to_structured
import pytest
import weakref


def test_construct_survey_from_xml():
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")

    assert survey.name == "toyblocks_als"
    assert len(survey.legs) == 6
    assert isinstance(survey.platform, Platform)
    assert isinstance(survey.scanner, Scanner)
    assert isinstance(survey.scene, StaticScene)


def test_construct_dynamic_survey_from_xml(xml_dynamic_test_survey):
    assert isinstance(xml_dynamic_test_survey.scene, DynamicScene)


def test_dynamic_survey_is_one_shot(monkeypatch):
    dynamic_survey = Survey.from_xml("data/surveys/dyn/tls_dyn_cube.xml")
    starts = []

    def record_start(playback):
        starts.append(playback)
        dynamic_survey.scanner._cpp_object.all_measurements = np.zeros(
            (1,), dtype=meas_dtype
        )
        dynamic_survey.scanner._cpp_object.all_trajectories = np.zeros(
            (1,), dtype=traj_dtype
        )

    monkeypatch.setattr(survey, "_start_playback_interruptible", record_start)
    execution_settings = ExecutionSettings(
        num_threads=1, progressbar=ProgressBarStrategy.NONE
    )

    dynamic_survey.run(format=OutputFormat.NPY, execution_settings=execution_settings)

    assert len(starts) == 1
    with pytest.raises(RuntimeError, match="reset support is planned"):
        dynamic_survey.run(
            format=OutputFormat.NPY, execution_settings=execution_settings
        )
    assert len(starts) == 1


def test_pre_playback_validation_error_does_not_consume_dynamic_scene(monkeypatch):
    dynamic_survey = Survey.from_xml("data/surveys/dyn/tls_dyn_cube.xml")
    starts = []

    def record_start(playback):
        starts.append(playback)
        dynamic_survey.scanner._cpp_object.all_measurements = np.zeros(
            (1,), dtype=meas_dtype
        )
        dynamic_survey.scanner._cpp_object.all_trajectories = np.zeros(
            (1,), dtype=traj_dtype
        )

    monkeypatch.setattr(survey, "_start_playback_interruptible", record_start)
    execution_settings = ExecutionSettings(
        num_threads=1, progressbar=ProgressBarStrategy.NONE
    )

    with pytest.raises(ValueError, match="Unknown parameters: unsupported_option"):
        dynamic_survey.run(
            format=OutputFormat.NPY,
            execution_settings=execution_settings,
            unsupported_option=True,
        )

    dynamic_survey.run(format=OutputFormat.NPY, execution_settings=execution_settings)
    assert len(starts) == 1


def test_dynamic_survey_moves_scene_and_callbacks_preserve_one_shot(
    dynamic_test_survey,
    dynamic_test_static_control_survey,
    dynamic_test_execution_settings,
):
    callback_times = []

    def record_time(context, points=None, trajectories=None):
        callback_times.append(context.sim_time_s)

    set_rng_seed(42)
    points, _ = dynamic_test_survey.run(
        format=OutputFormat.NPY,
        execution_settings=dynamic_test_execution_settings,
        callbacks=(
            helios.SurveyHook(
                point=helios.HookPoint.SIM_TIME_PERIODIC,
                callback=record_time,
                sim_time=0,
                period=0.2,
            ),
        ),
    )

    set_rng_seed(42)
    static_points, _ = dynamic_test_static_control_survey.run(
        format=OutputFormat.NPY,
        execution_settings=dynamic_test_execution_settings,
    )

    assert points.shape[0] > 0
    assert points.dtype["hit_object_id"].kind in ("i", "u")
    assert 2 in points["hit_object_id"]
    assert callback_times == sorted(callback_times)
    assert len(callback_times) >= 2

    dynamic_cube = points[points["hit_object_id"] == 2]
    static_cube = static_points[static_points["hit_object_id"] == 2]
    assert dynamic_cube.shape[0] > 0
    assert static_cube.shape[0] > 0
    assert (
        dynamic_cube["position"][:, 0].mean() > static_cube["position"][:, 0].mean() + 4
    )

    with pytest.raises(RuntimeError, match="reset support is planned"):
        dynamic_test_survey.run(
            format=OutputFormat.NPY,
            execution_settings=dynamic_test_execution_settings,
        )


def test_dynamic_survey_las_output_accepts_numeric_dynamic_object_id(
    tmp_path, xml_dynamic_test_survey, dynamic_test_execution_settings
):
    output_path = xml_dynamic_test_survey.run(
        format=OutputFormat.LAS,
        output_dir=tmp_path,
        execution_settings=dynamic_test_execution_settings,
    )

    files = list(output_path.rglob("*.las"))
    assert len(files) == 1
    las = laspy.read(files[0])
    assert len(las.points) > 0
    assert las.hitObjectId.dtype.kind in ("i", "u")
    assert 2 in las.hitObjectId


def test_dynamic_scene_cannot_be_shared_between_surveys():
    dynamic_scene = DynamicScene.from_xml("data/scenes/dyn/dyn_cube_scene.xml")
    owner = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=dynamic_scene)

    with pytest.raises(ValueError, match="already owned by another Survey"):
        Survey(scanner=riegl_vz_400(), platform=tripod(), scene=dynamic_scene)

    assert owner.scene is dynamic_scene


def test_dynamic_scene_owner_is_released_on_reassignment():
    first_scene = DynamicScene.from_xml("data/scenes/dyn/dyn_cube_scene.xml")
    replacement_scene = DynamicScene.from_xml("data/scenes/dyn/dyn_cube_scene.xml")
    first_owner = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=first_scene)

    first_owner.scene = replacement_scene
    second_owner = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=first_scene)

    assert first_owner.scene is replacement_scene
    assert second_owner.scene is first_scene


def test_dynamic_scene_owner_is_released_after_garbage_collection():
    dynamic_scene = DynamicScene.from_xml("data/scenes/dyn/dyn_cube_scene.xml")
    owner = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=dynamic_scene)
    owner_ref = weakref.ref(owner)

    del owner
    gc.collect()

    assert owner_ref() is None
    replacement_owner = Survey(
        scanner=riegl_vz_400(), platform=tripod(), scene=dynamic_scene
    )
    assert replacement_owner.scene is dynamic_scene


def test_dynamic_survey_clone_and_deepcopy_are_rejected():
    dynamic_survey = Survey.from_xml("data/surveys/dyn/tls_dyn_cube.xml")

    with pytest.raises(NotImplementedError, match="Survey with a DynamicScene"):
        dynamic_survey.clone()
    with pytest.raises(NotImplementedError, match="Survey with a DynamicScene"):
        copy.deepcopy(dynamic_survey)


def test_dynamic_survey_rejects_live_viewing_before_attachment(monkeypatch):
    dynamic_survey = Survey.from_xml("data/surveys/dyn/tls_dyn_cube.xml")

    def unexpected_live_resolution(live):
        raise AssertionError("live session resolution must not be reached")

    monkeypatch.setattr(survey, "_resolve_live_session", unexpected_live_resolution)

    with pytest.raises(NotImplementedError, match="Live viewing.*DynamicScene"):
        dynamic_survey.run(live=True)

    viewer = survey.LiveViewer.__new__(survey.LiveViewer)
    with pytest.raises(NotImplementedError, match="Live viewing.*DynamicScene"):
        dynamic_survey.run(live=viewer)

    assert dynamic_survey.scene._playback_state().name == "FRESH"


def test_leg_clone_and_deepcopy(survey):
    leg = survey.legs[0]
    leg._is_survey_and_legs_integrated = True
    leg.runtime_note = "temporary"
    original_pulse_frequency = leg.scanner_settings.pulse_frequency
    execution_settings = ExecutionSettings(num_threads=1)

    for copied in (leg.clone(), copy.deepcopy(leg)):
        assert copied is not leg
        assert copied.scanner_settings is not leg.scanner_settings
        assert copied.platform_settings is not leg.platform_settings
        assert not hasattr(copied, "_is_survey_and_legs_integrated")
        assert not hasattr(copied, "runtime_note")

        copied.scanner_settings.pulse_frequency = original_pulse_frequency + 1
        assert leg.scanner_settings.pulse_frequency == original_pulse_frequency

        cloned_survey = Survey(
            scanner=riegl_vz_400(),
            platform=tripod(),
            scene=StaticScene(
                scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
            ),
        )
        cloned_survey.add_leg(leg=copied)
        points, trajectory = cloned_survey.run(
            format=OutputFormat.NPY, execution_settings=execution_settings
        )
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


def test_survey_clone_run_matches_point_cloud(survey):
    cloned_survey = survey.clone()
    execution_settings = ExecutionSettings(num_threads=1)

    set_rng_seed(42)
    points, trajectory = survey.run(
        format=OutputFormat.NPY, execution_settings=execution_settings
    )

    set_rng_seed(42)
    cloned_points, cloned_trajectory = cloned_survey.run(
        format=OutputFormat.NPY, execution_settings=execution_settings
    )

    assert cloned_survey.scene is not survey.scene
    np.testing.assert_array_equal(points, cloned_points)
    np.testing.assert_array_equal(trajectory, cloned_trajectory)


def test_surveys_sharing_scene_match_deep_copied_scenes():
    execution_settings = ExecutionSettings(
        num_threads=1,
        kdt_num_threads=1,
        kdt_geom_num_threads=1,
        progressbar=ProgressBarStrategy.NONE,
    )
    scanner_settings = ScannerSettings(
        pulse_frequency=2000,
        scan_frequency=20,
        scan_angle="10 deg",
        head_rotation="20 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="1 deg",
    )
    platform_settings = StaticPlatformSettings(x=0, y=0, z=0)
    shared_scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )

    def make_survey(scanner_factory, scene):
        survey = Survey(scanner=scanner_factory(), platform=tripod(), scene=scene)
        survey.add_leg(
            scanner_settings=copy.deepcopy(scanner_settings),
            platform_settings=copy.deepcopy(platform_settings),
        )
        return survey

    survey_pairs = [
        (
            make_survey(riegl_vz_400, shared_scene),
            make_survey(riegl_vz_400, copy.deepcopy(shared_scene)),
        ),
        (
            make_survey(riegl_vz_1000, shared_scene),
            make_survey(riegl_vz_1000, copy.deepcopy(shared_scene)),
        ),
    ]

    assert survey_pairs[0][0].scene is shared_scene
    assert survey_pairs[1][0].scene is shared_scene

    for shared_survey, copied_scene_survey in survey_pairs:
        shared_points, shared_trajectory = shared_survey.run(
            format=OutputFormat.NPY, execution_settings=execution_settings
        )
        copied_points, copied_trajectory = copied_scene_survey.run(
            format=OutputFormat.NPY, execution_settings=execution_settings
        )

        assert 0 < shared_points.shape[0] <= 10
        np.testing.assert_array_equal(shared_points, copied_points)
        np.testing.assert_array_equal(shared_trajectory, copied_trajectory)


def test_add_leg_parameters():
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")

    platform_settings = PlatformSettings(x=5)
    scanner_settings = ScannerSettings(pulse_frequency=1000)
    survey.add_leg(
        platform_settings=platform_settings,
        scanner_settings=scanner_settings,
        y=5,
        head_rotation=12,
    )
    assert survey.legs[-1].platform_settings.x == 5
    assert survey.legs[-1].platform_settings.y == 5
    assert survey.legs[-1].scanner_settings.pulse_frequency == 1000
    assert survey.legs[-1].scanner_settings.head_rotation == 12

    with pytest.raises(ValueError):
        survey.add_leg(foobar=12)


def test_add_leg_updates_provided_leg_with_explicit_settings(survey):
    from helios.leg import Leg

    existing_leg = Leg(
        platform_settings=PlatformSettings(x=1, y=2, z=3),
        scanner_settings=ScannerSettings(pulse_frequency=100),
        trajectory_settings=TrajectorySettings(
            start_time=0, end_time=1, teleport_to_start=False
        ),
    )
    platform_settings = PlatformSettings(x=11, y=22, z=33)
    scanner_settings = ScannerSettings(pulse_frequency=777)
    trajectory_settings = TrajectorySettings(
        start_time=10, end_time=20, teleport_to_start=True
    )

    survey.add_leg(
        leg=existing_leg,
        platform_settings=platform_settings,
        scanner_settings=scanner_settings,
        trajectory_settings=trajectory_settings,
    )

    appended = survey.legs[-1]
    assert appended is existing_leg
    assert appended.platform_settings.x == 11
    assert appended.platform_settings.y == 22
    assert appended.platform_settings.z == 33
    assert appended.scanner_settings.pulse_frequency == 777
    assert appended.trajectory_settings.start_time == 10
    assert appended.trajectory_settings.end_time == 20
    assert appended.trajectory_settings.teleport_to_start is True


def test_survey_run_numpy_output(survey):
    points, trajectory = survey.run(format=OutputFormat.NPY)

    assert points.shape[0] == 352
    assert trajectory.shape[0] == 101
    assert points.dtype == meas_dtype
    assert trajectory.dtype == traj_dtype


def test_survey_run_las_output(survey, tmp_path):
    path = survey.run(output_dir=tmp_path, format=OutputFormat.LAS)

    # Ensure there is one LAS file
    files = list(path.rglob("*.las"))
    assert len(files) == 1

    # Read the output
    las = laspy.read(files[0])
    assert las.X.shape[0] == 352
    column_names = list(las.point_format.dimension_names)
    names_to_check = [
        "X",
        "Y",
        "Z",
        "intensity",
        "return_number",
        "number_of_returns",
        "gps_time",
        "classification",
        "echo_width",
        "fullwaveIndex",
        "hitObjectId",
    ]
    for name in names_to_check:
        assert name in column_names


def test_survey_run_laz_output(survey, tmp_path):
    path = survey.run(output_dir=tmp_path, format=OutputFormat.LAZ)

    # Ensure there is one LAZ file
    files = list(path.rglob("*.laz"))
    assert len(files) == 1

    # Read the output
    las = laspy.read(files[0])
    assert las.X.shape[0] == 352


def test_survey_run_xyz_output(survey, tmp_path):
    path = survey.run(output_dir=tmp_path, format=OutputFormat.XYZ)

    # Ensure there is one XYZ file
    files = list(path.rglob("*.xyz"))
    assert len(files) == 1

    # Read the output
    points = np.genfromtxt(files[0], delimiter="")
    assert points.shape[0] == 352


def test_survey_run_laspy_output(survey):
    las, traj = survey.run(format=OutputFormat.LASPY)

    assert len(las.points) == 352
    assert all(las.return_number == np.ones_like(las.return_number))
    assert traj.shape == (101,)


def test_set_gpstime(survey):
    survey.gps_time = datetime.now(timezone.utc)
    # This timestamp is at the gps week start.
    # A timezone must be given to make this system time independent.
    survey.gps_time = "2025-02-09T00:00:09+00:00"
    with pytest.raises(ValueError):
        survey.gps_time = "foobar"

    points, _ = survey.run()

    assert np.all(points["gps_time"] >= 0)
    assert np.all(points["gps_time"] < 1)


def test_survey_run_unknown_parameters(survey):
    with pytest.raises(ValueError):
        survey.run(unknown_parameter=12)


def test_survey_run_trajectory_for_all_scanner_types(light_als_multiscanner_survey):
    points, trajectory = light_als_multiscanner_survey.run()
    assert points.shape[0] > 0
    assert trajectory.shape[0] > 0


def test_full_waveform_settings_effect(light_als_multiscanner_survey):
    points1, _ = light_als_multiscanner_survey.run(format=OutputFormat.NPY)

    light_als_multiscanner_survey.full_waveform_settings.beam_sample_quality = 5
    points2, _ = light_als_multiscanner_survey.run(format=OutputFormat.NPY)

    # A higher beam sample quality should result in more points
    assert points1.shape[0] < points2.shape[0]


def test_traj_from_np(survey):
    traj = np.arange((70)).reshape((10, 7))
    traj = unstructured_to_structured(traj, dtype=traj_csv_dtype)

    survey.trajectory = traj
    assert survey.trajectory.shape == (10,)
    assert survey.trajectory["x"].shape == (10,)
    assert len(survey.trajectory[0]) == 7


def test_invalid_leg_adding():
    """
    Test that adding a leg via the `append` method raises an error.
    """
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")
    new_leg = Leg(
        platform_settings=PlatformSettings(),
        scanner_settings=ScannerSettings(),
    )
    assert len(survey.legs) == 6

    survey.add_leg(new_leg)
    assert len(survey.legs) == 7

    with pytest.raises(AttributeError, match="object has no attribute 'append'"):
        survey.append(new_leg)


def test_survey_flag_from_xml_set():
    from helios.utils import is_xml_loaded

    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")
    assert is_xml_loaded(survey)


def test_survey_tls_multi_scan_not_from_xml(tripod, multi_tls_scanner, scene):
    scanner_settings = ScannerSettings(
        pulse_frequency=2000,
        scan_frequency=20,
        head_rotation="30 deg/s",
        rotation_start_angle=0,
        rotation_stop_angle=1,
    )
    platform_settings = PlatformSettings(x=0, y=0, z=0)
    survey = Survey(scanner=multi_tls_scanner, platform=tripod, scene=scene)
    survey.add_leg(
        platform_settings=platform_settings, scanner_settings=scanner_settings
    )
    m, t = survey.run(format=OutputFormat.NPY)
    assert m.shape[0] > 0
    assert t.shape[0] > 0
    assert np.unique(m["channel_id"]).size > 1


def test_survey_als_multi_scan_not_from_xml(airplane, multi_als_scanner):
    scene = StaticScene.from_xml("data/scenes/toyblocks/light_toyblocks_scene.xml")
    survey = Survey(scanner=multi_als_scanner, platform=airplane, scene=scene)

    scanner_settings1 = ScannerSettings(
        pulse_frequency=2000,
        scan_angle="20 deg",
        scan_frequency=40,
        trajectory_time_interval=0.2,
    )
    platform_settings1 = DynamicPlatformSettings(x=-30, y=-50, z=100, speed_m_s=10)
    survey.add_leg(
        platform_settings=platform_settings1, scanner_settings=scanner_settings1
    )
    scanner_settings2 = ScannerSettings(
        is_active=False,
        pulse_frequency=2000,
        scan_angle="20 deg",
        scan_frequency=40,
        trajectory_time_interval=0.2,
    )
    platform_settings2 = DynamicPlatformSettings(x=70, y=-50, z=100, speed_m_s=10)
    survey.add_leg(
        platform_settings=platform_settings2, scanner_settings=scanner_settings2
    )
    scanner_settings3 = ScannerSettings(
        pulse_frequency=2000,
        scan_angle="20 deg",
        scan_frequency=40,
        trajectory_time_interval=0.2,
    )
    platform_settings3 = DynamicPlatformSettings(x=70, y=0, z=100, speed_m_s=10)
    survey.add_leg(
        platform_settings=platform_settings3, scanner_settings=scanner_settings3
    )
    scanner_settings4 = ScannerSettings(
        is_active=False,
        pulse_frequency=2000,
        scan_angle="20 deg",
        scan_frequency=40,
        trajectory_time_interval=0.2,
    )
    platform_settings4 = DynamicPlatformSettings(x=-30, y=0, z=100, speed_m_s=10)
    survey.add_leg(
        platform_settings=platform_settings4, scanner_settings=scanner_settings4
    )

    m, t = survey.run(format=OutputFormat.NPY)
    assert m.shape[0] > 0
    assert t.shape[0] > 0
    # Add this test to check that serial id is specified correctly
    assert survey.legs[0]._cpp_object.serial_id == 0


def test_run_interpolated_survey():
    execution_settings = ExecutionSettings(
        num_threads=1,
    )

    scanner_settings1 = ScannerSettings(
        is_active=True,
        pulse_frequency=2000,
        scan_frequency=20,
        scan_angle=0,
        trajectory_time_interval=0.05,
    )

    trajectory_settings1 = TrajectorySettings(start_time=0, end_time=0.1)

    scanner_settings2 = ScannerSettings(
        is_active=True,
        pulse_frequency=2000,
        scan_frequency=20,
        scan_angle=0,
        trajectory_time_interval=0.05,
    )

    trajectory_settings2 = TrajectorySettings(
        start_time=0.1, end_time=0.2, teleport_to_start=True
    )
    trajectory = load_traj_csv(
        csv="data/trajectories/flyandrotate.trj",
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )
    scene = StaticScene.from_xml("data/scenes/demo/box_scene.xml")
    platform = Platform.load_interpolate_platform(
        trajectory=trajectory,
        platform_file="data/platforms.xml",
        platform_id="sr22",
        interpolation_method="ARINC 705",
        sync_gps_time=True,
    )
    scanner = riegl_lms_q560()
    survey1 = Survey(scanner=scanner, platform=platform, scene=scene)
    survey1.add_leg(
        scanner_settings=scanner_settings1, trajectory_settings=trajectory_settings1
    )
    survey1.add_leg(
        scanner_settings=scanner_settings2, trajectory_settings=trajectory_settings2
    )
    m1, t1 = survey1.run(execution_settings=execution_settings)

    surv2 = Survey.from_xml("data/surveys/demo/box_survey_interp.xml")
    surv2.legs = surv2.legs[:2]
    surv2.legs[0].scanner_settings.pulse_frequency = 2000
    surv2.legs[0].scanner_settings.scan_frequency = 20
    surv2.legs[0].scanner_settings.trajectory_time_interval = 0.05
    surv2.legs[0].trajectory_settings.start_time = 0
    surv2.legs[0].trajectory_settings.end_time = 0.1
    surv2.legs[1].scanner_settings.pulse_frequency = 2000
    surv2.legs[1].scanner_settings.scan_frequency = 20
    surv2.legs[1].scanner_settings.trajectory_time_interval = 0.05
    surv2.legs[1].trajectory_settings.start_time = 0.1
    surv2.legs[1].trajectory_settings.end_time = 0.2
    m2, t2 = surv2.run(execution_settings=execution_settings)

    assert m1.shape[0] > 0
    assert m2.shape[0] > 0
    assert t1.shape[0] > 0
    assert t2.shape[0] > 0
    assert np.allclose(t1["position"][0], t2["position"][0], rtol=1e-1, atol=1e-1)
    assert np.allclose(t1["position"][-1], t2["position"][-1], rtol=1e-1, atol=1e-1)


def test_survey_run_with_scene_fixture(scene):
    """
    Test that a survey can be run with an in-memory scene.
    """

    scanner_settings = ScannerSettings(
        pulse_frequency=2000,
        scan_frequency=20,
        min_vertical_angle="-10 deg",
        max_vertical_angle="10 deg",
        head_rotation="30 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="20 deg",
    )
    platform_settings = PlatformSettings(x=0, y=0, z=0)

    platform = tripod()
    scanner = riegl_vz_400()

    survey = Survey(scanner=scanner, platform=platform, scene=scene)
    survey.add_leg(
        platform_settings=platform_settings, scanner_settings=scanner_settings
    )

    points, trajectory = survey.run(format=OutputFormat.NPY)

    assert points.shape[0] > 0
    assert trajectory.shape[0] > 0


def test_static_plat_settings_valid_in_add_leg():
    scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )
    scanner = riegl_vz_400()
    platform = tripod()
    scanner_settings = ScannerSettings(
        pulse_frequency=100_000,
        scan_frequency=120,
        min_vertical_angle="-40 deg",
        max_vertical_angle="60 deg",
        head_rotation="10 deg/s",
    )
    survey = Survey(scanner=scanner, platform=platform, scene=scene)

    survey.add_leg(
        scanner_settings=scanner_settings,
        x=1.0,
        y=25.5,
        z=1.5,
        force_on_ground=True,
        rotation_start_angle="-100 deg",
        rotation_stop_angle="225 deg",
    )

    survey.add_leg(
        scanner_settings=scanner_settings,
        x=-4.0,
        y=-2.5,
        z=1.5,
        force_on_ground=True,
        rotation_start_angle="-15 deg",
        rotation_stop_angle="45 deg",
    )

    for leg in survey.legs:
        assert leg.platform_settings.force_on_ground is True
        assert leg.platform_settings._cpp_object.force_on_ground is True
        assert type(leg.platform_settings) == StaticPlatformSettings


def test_dynamic_plat_settings_valid_in_add_leg():
    scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )
    scanner = riegl_vz_400()
    platform = tripod()
    scanner_settings = ScannerSettings(
        pulse_frequency=100_000,
        scan_frequency=120,
        min_vertical_angle="-40 deg",
        max_vertical_angle="60 deg",
        head_rotation="10 deg/s",
    )
    survey = Survey(scanner=scanner, platform=platform, scene=scene)

    survey.add_leg(
        scanner_settings=scanner_settings,
        x=1.0,
        y=25.5,
        z=1.5,
        speed_m_s=3.0,
        rotation_start_angle="-100 deg",
        rotation_stop_angle="225 deg",
    )

    survey.add_leg(
        scanner_settings=scanner_settings,
        x=-4.0,
        y=-2.5,
        z=1.5,
        speed_m_s=3.0,
        rotation_start_angle="-15 deg",
        rotation_stop_angle="45 deg",
    )

    for leg in survey.legs:
        assert leg.platform_settings.speed_m_s == 3.0
        assert leg.platform_settings._cpp_object.speed_m_s == 3.0
        assert type(leg.platform_settings) == DynamicPlatformSettings


def test_survey_run_with_incorrect_ver_hor_resolution():
    """
    Test that a survey raises an error when vertical or horizontal resolution is set incorrectly.
    """
    scanner_settings1 = ScannerSettings(
        pulse_frequency=100_000,
        scan_frequency=120,
        min_vertical_angle=-40,
        max_vertical_angle=60,
        head_rotation=10,
    )

    scanner_settings2 = ScannerSettings(
        pulse_frequency=100_000,
        vertical_resolution=0.2,  # we mean degrees, but did not specify units
        horizontal_resolution=0.2,
        min_vertical_angle=-40,
        max_vertical_angle=60,
    )

    scanner = riegl_vz_400()
    platform = simple_linearpath()
    scene = StaticScene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")

    survey = Survey(scanner=scanner, platform=platform, scene=scene)

    survey.add_leg(
        scanner_settings=scanner_settings1,
        x=1.0,
        y=25.5,
        z=1.5,
        force_on_ground=True,
        rotation_start_angle=100,
        rotation_stop_angle=225,
    )

    survey.add_leg(
        scanner_settings=scanner_settings2,
        x=-4.0,
        y=-2.5,
        z=1.5,
        force_on_ground=True,
        rotation_start_angle=-45,
        rotation_stop_angle=45,
    )

    with pytest.raises(RuntimeError):
        survey.run(format=OutputFormat.NPY)


def test_survey_run_with_hor_ver_resolution(scene):
    """
    Test that a survey runs successfully when vertical or horizontal resolution is set with correct units.
    """
    scanner_settings = ScannerSettings(
        pulse_frequency=2000,
        vertical_resolution="2 deg",
        horizontal_resolution="2 deg",
        min_vertical_angle="-10 deg",
        max_vertical_angle="10 deg",
    )

    scanner = riegl_vz_400()
    platform = tripod()

    survey = Survey(scanner=scanner, platform=platform, scene=scene)

    survey.add_leg(
        scanner_settings=scanner_settings,
        x=0.0,
        y=0.0,
        z=0.0,
        rotation_start_angle="0 deg",
        rotation_stop_angle="10 deg",
    )

    points, trajectory = survey.run(format=OutputFormat.NPY)

    assert points.shape[0] > 0
    assert trajectory.shape[0] > 0


def test_survey_run_with_non_symmetric_vert_angle_limits_and_max_larger_than_effective_max_scan_angle(
    wall_scene,
):
    """
    Test that the simulated output has the correct extent when vertical angle limits are not symmetric around zero
    and the max vertical angle is larger than the effecte max scan angle.
    """
    scanner_settings = ScannerSettings(
        pulse_frequency=300_000,
        vertical_resolution="0.05 deg",
        horizontal_resolution="0.05 deg",
        min_vertical_angle="-40 deg",
        max_vertical_angle="60 deg",
    )

    scanner = riegl_vz_400()
    platform = tripod()

    survey = Survey(scanner=scanner, platform=platform, scene=wall_scene)

    survey.add_leg(
        scanner_settings=scanner_settings,
        x=0.0,
        y=0.0,
        z=-1.7,
        rotation_start_angle="-1 deg",
        rotation_stop_angle="1 deg",
    )

    points, _ = survey.run(format=OutputFormat.NPY)

    eps = 0.1
    assert abs(np.max(points["position"][:, 2]) - 86.6) < eps
    assert abs(np.min(points["position"][:, 2]) - (-41.95)) < eps


def test_survey_run_with_invalid_vert_angle_limits(scene):
    """
    Test that an error is raised if the range of vertical angle limits is larger than the effective max scan angle of the device.
    """
    scanner_settings = ScannerSettings(
        pulse_frequency=300_000,
        vertical_resolution="0.05 deg",
        horizontal_resolution="0.05 deg",
        min_vertical_angle="-50 deg",
        max_vertical_angle="60 deg",
    )

    scanner = riegl_vz_400()
    platform = tripod()

    survey = Survey(scanner=scanner, platform=platform, scene=scene)

    survey.add_leg(
        scanner_settings=scanner_settings,
        x=0.0,
        y=0.0,
        z=-1.7,
        rotation_start_angle="-1 deg",
        rotation_stop_angle="1 deg",
    )

    with pytest.raises(HeliosException):
        survey.run(format=OutputFormat.NPY)

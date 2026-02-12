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
from helios.scanner import Scanner, riegl_lms_q560, riegl_vz_400
from helios.scene import StaticScene, ScenePart
from helios.settings import ExecutionSettings
from helios.survey import *

import laspy
from numpy.lib.recfunctions import unstructured_to_structured
import pytest


def test_construct_survey_from_xml():
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")

    assert survey.name == "toyblocks_als"
    assert len(survey.legs) == 6
    assert isinstance(survey.platform, Platform)
    assert isinstance(survey.scanner, Scanner)
    assert isinstance(survey.scene, StaticScene)


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
    las.X.shape[0] == 352


def test_survey_run_laz_output(survey, tmp_path):
    path = survey.run(output_dir=tmp_path, format=OutputFormat.LAZ)

    # Ensure there is one LAZ file
    files = list(path.rglob("*.laz"))
    assert len(files) == 1

    # Read the output
    las = laspy.read(files[0])
    las.X.shape[0] == 200


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
        rotation_stop_angle=20,
    )
    platform_settings = PlatformSettings(x=0, y=0, z=0)
    survey = Survey(scanner=multi_tls_scanner, platform=tripod, scene=scene)
    survey.add_leg(
        platform_settings=platform_settings, scanner_settings=scanner_settings
    )
    m, t = survey.run(format=OutputFormat.NPY)
    assert m.shape[0] > 0
    assert t.shape[0] > 0


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
        trajectory_time_interval=0.5,
    )

    trajectory_settings1 = TrajectorySettings(start_time=0, end_time=1)

    scanner_settings2 = ScannerSettings(
        is_active=True,
        pulse_frequency=2000,
        scan_frequency=20,
        scan_angle=0,
        trajectory_time_interval=0.5,
    )

    trajectory_settings2 = TrajectorySettings(
        start_time=1, end_time=2, teleport_to_start=True
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
    surv2.legs[0].scanner_settings.pulse_frequency = 2000
    surv2.legs[0].scanner_settings.scan_frequency = 20
    surv2.legs[0].scanner_settings.trajectory_time_interval = 0.5
    surv2.legs[0].trajectory_settings.start_time = 0
    surv2.legs[0].trajectory_settings.end_time = 1
    surv2.legs[1].scanner_settings.pulse_frequency = 2000
    surv2.legs[1].scanner_settings.scan_frequency = 20
    surv2.legs[1].scanner_settings.trajectory_time_interval = 0.5
    surv2.legs[1].trajectory_settings.start_time = 1
    surv2.legs[1].trajectory_settings.end_time = 2
    m2, t2 = surv2.run(execution_settings=execution_settings)

    assert m1.shape[0] > 0
    assert m2.shape[0] > 0
    assert np.allclose(t1["position"][0], t2["position"][0], rtol=1e-1, atol=1e-1)


def test_survey_run_with_binary_scene(tmp_path, scene):
    """
    Test that a survey can be run with a binary scene.
    """
    binary_path = tmp_path / "box_scene_case23.scene"
    scene.to_binary(str(binary_path))
    scene = StaticScene.from_binary(str(binary_path))

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

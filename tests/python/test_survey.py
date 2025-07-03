from helios.platforms import Platform, tripod, sr22, DynamicPlatformSettings
from helios.scanner import Scanner, livox_mid100, vlp16
from helios.scene import StaticScene
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

    assert points.shape[0] == 200
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
    las.X.shape[0] == 200


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
    assert points.shape[0] == 200


def test_survey_run_laspy_output(survey):
    las, traj = survey.run(format=OutputFormat.LASPY)

    assert len(las.points) == 200
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

    assert np.all(points["gps_time"] > 0)
    assert np.all(points["gps_time"] < 1)


def test_survey_run_unknown_parameters(survey):
    with pytest.raises(ValueError):
        survey.run(unknown_parameter=12)


def test_survey_run_trajectory_for_all_scanner_types():
    survey = Survey.from_xml("data/surveys/demo/light_als_toyblocks_multiscanner.xml")
    survey.legs[0].scanner_settings.trajectory_time_interval = 0.1
    points, trajectory = survey.run()
    assert points.shape[0] > 0
    assert trajectory.shape[0] > 0


def test_full_waveform_settings_effect():
    survey = Survey.from_xml("data/surveys/demo/light_als_toyblocks_multiscanner.xml")
    points1, _ = survey.run(format=OutputFormat.NPY)

    survey.full_waveform_settings.beam_sample_quality = 5
    points2, _ = survey.run(format=OutputFormat.NPY)

    # A higher beam sample quality should result in more points
    assert points1.shape[0] < points2.shape[0]


def test_load_csv_traj(survey):
    survey.load_traj_csv(csv="data/trajectories/cycloid.trj")
    assert survey.trajectory.shape == (51,)
    t = np.void(
        [(3.7, -60.0, 60.0, 330.7, 13.002584, 1.122905, 400.0)], dtype=traj_csv_dtype
    )
    assert all([a == b for a, b in zip(survey.trajectory[0], t[0])])


def test_load_csv_traj_reordering(survey):
    survey.load_traj_csv(
        csv="data/trajectories/cycloid.trj",
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )
    assert survey.trajectory.shape == (51,)
    t = np.void(
        [(3.7, 13.002584, 1.122905, 400.0, -60.0, 60.0, 330.7)], dtype=traj_csv_dtype
    )
    assert all([a == b for a, b in zip(survey.trajectory[0], t[0])])


def test_traj_from_np(survey):
    traj = np.arange((70)).reshape((10, 7))
    traj = unstructured_to_structured(traj, dtype=traj_csv_dtype)

    survey.trajectory = traj
    assert survey.trajectory.shape == (10,)
    assert survey.trajectory["x"].shape == (10,)
    assert len(survey.trajectory[0]) == 7


def test_load_csv_traj(survey):
    survey.load_traj_csv(csv="data/trajectories/cycloid.trj")
    assert survey.trajectory.shape == (51,)
    t = np.void(
        [(3.7, -60.0, 60.0, 330.7, 13.002584, 1.122905, 400.0)], dtype=traj_csv_dtype
    )
    assert all([a == b for a, b in zip(survey.trajectory[0], t[0])])


def test_load_csv_traj_reordering(survey):
    survey.load_traj_csv(
        csv="data/trajectories/cycloid.trj",
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )
    assert survey.trajectory.shape == (51,)
    t = np.void(
        [(3.7, 13.002584, 1.122905, 400.0, -60.0, 60.0, 330.7)], dtype=traj_csv_dtype
    )
    assert all([a == b for a, b in zip(survey.trajectory[0], t[0])])


def test_traj_from_np(survey):
    traj = np.arange((70)).reshape((10, 7))
    traj = unstructured_to_structured(traj, dtype=traj_csv_dtype)

    survey.trajectory = traj
    assert survey.trajectory.shape == (10,)
    assert survey.trajectory["x"].shape == (10,)
    assert len(survey.trajectory[0]) == 7


def test_load_csv_traj(survey):
    survey.load_traj_csv(csv="data/trajectories/cycloid.trj")
    assert survey.trajectory.shape == (51,)
    t = np.void(
        [(3.7, -60.0, 60.0, 330.7, 13.002584, 1.122905, 400.0)], dtype=traj_csv_dtype
    )
    assert all([a == b for a, b in zip(survey.trajectory[0], t[0])])


def test_load_csv_traj_reordering(survey):
    survey.load_traj_csv(
        csv="data/trajectories/cycloid.trj",
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )
    assert survey.trajectory.shape == (51,)
    t = np.void(
        [(3.7, 13.002584, 1.122905, 400.0, -60.0, 60.0, 330.7)], dtype=traj_csv_dtype
    )
    assert all([a == b for a, b in zip(survey.trajectory[0], t[0])])


def test_traj_from_np(survey):
    traj = np.arange((70)).reshape((10, 7))
    traj = unstructured_to_structured(traj, dtype=traj_csv_dtype)

    survey.trajectory = traj
    assert survey.trajectory.shape == (10,)
    assert survey.trajectory["x"].shape == (10,)
    assert len(survey.trajectory[0]) == 7


def test_survey_tls_multi_scan_not_from_xml():
    scanner_settings = ScannerSettings(pulse_frequency=18750, scan_frequency=0, head_rotation=3600,
                                        rotation_start_angle=0, rotation_stop_angle=3600)
    platform_settings = PlatformSettings(x=0, y=0, z=0)
    scene = StaticScene.from_xml("data/scenes/demo/box_scene.xml")
    scanner = vlp16()
    platform = tripod()
    survey = Survey(scanner=scanner, platform=platform, scene=scene)
    survey.add_leg(
        platform_settings=platform_settings,
        scanner_settings=scanner_settings
    )
    m,t = survey.run(format=OutputFormat.NPY)
    assert m.shape[0] > 0
    assert t.shape[0] > 0


def test_survey_als_multi_scan_not_from_xml():
    scene = StaticScene.from_xml("data/scenes/toyblocks/light_toyblocks_scene.xml")
    scanner = livox_mid100()
    platform = sr22()
    survey = Survey(scanner=scanner, platform=platform, scene=scene)

    scanner_settings1 = ScannerSettings(pulse_frequency=10000, scan_angle="20 deg",
                                        scan_frequency=100, trajectory_time_interval= 0.01)
    platform_settings1 = DynamicPlatformSettings(x=-30, y=-50, z = 100, speed_m_s=30)
    survey.add_leg(
        platform_settings=platform_settings1,
        scanner_settings=scanner_settings1
    )
    scanner_settings2 = ScannerSettings(is_active= False, pulse_frequency=10000, scan_angle="20 deg", 
                                        scan_frequency=100, trajectory_time_interval= 0.01)
    platform_settings2 = DynamicPlatformSettings(x=70, y=-50, z = 100, speed_m_s=30)
    survey.add_leg(
        platform_settings=platform_settings2,
        scanner_settings=scanner_settings2
    )
    scanner_settings3 = ScannerSettings(pulse_frequency=10000, scan_angle="20 deg", 
                                        scan_frequency=100, trajectory_time_interval= 0.01)
    platform_settings3 = DynamicPlatformSettings(x=70, y=0, z = 100, speed_m_s=30)
    survey.add_leg(
        platform_settings=platform_settings3,
        scanner_settings=scanner_settings3
    )
    scanner_settings4 = ScannerSettings(is_active= False, pulse_frequency=10000, scan_angle="20 deg", 
                                        scan_frequency=100, trajectory_time_interval= 0.01)
    platform_settings4 = DynamicPlatformSettings(x=-30, y=0, z = 100, speed_m_s=30)
    survey.add_leg(
        platform_settings=platform_settings4,
        scanner_settings=scanner_settings4
    )

    m, t = survey.run(format=OutputFormat.NPY)
    assert m.shape[0] > 0
    assert t.shape[0] > 0
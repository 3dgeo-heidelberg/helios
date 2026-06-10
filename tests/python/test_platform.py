from helios.platforms import *
from helios.scanner import *
from helios.scene import ScenePart, StaticScene
from helios.survey import *
from helios.utils import *

import copy
import math
import pytest
from pydantic import ValidationError
from helios import HeliosException
import helios.platforms as platform_module

EXPECTED_BEAM_DIRECTION_SIGNATURES = {
    "CANONICAL": np.array(
        [
            [-0.568614479348, 0.538883486203, -0.621516019241],
            [-0.566368812246, 0.538793846681, -0.623640568993],
            [-0.564470989375, 0.529115739479, -0.633568493843],
            [-0.564118107684, 0.538698226268, -0.625759523778],
            [-0.562137201970, 0.528998087853, -0.635737987862],
            [-0.561862387512, 0.538596625514, -0.627872863313],
            [-0.559797476488, 0.528875555739, -0.637900800955],
            [-0.559601673621, 0.538489045035, -0.629980567365],
            [-0.557451836523, 0.528748145010, -0.640056910833],
            [-0.557335987949, 0.538375485511, -0.632082615753],
        ],
        dtype=float,
    ),
    "ARINC 705": np.array(
        [
            [-0.818428131042, 0.087774114409, -0.567865388239],
            [-0.817721379005, 0.055331640751, -0.572948650273],
            [-0.816630052169, 0.087744701593, -0.570452649425],
            [-0.816522580471, 0.120470721054, -0.564604003661],
            [-0.816244179941, 0.023291734736, -0.577237328839],
            [-0.815906933544, 0.055473185649, -0.575515943714],
            [-0.815851969812, -0.008892114231, -0.578192436528],
            [-0.815497640914, 0.153688948802, -0.557981455498],
            [-0.814823821503, 0.087715099688, -0.573034205958],
            [-0.814747513185, 0.120273129932, -0.567204428733],
        ],
        dtype=float,
    ),
}


def sorted_vectors(vectors, decimals=12):
    vectors = np.asarray(vectors)
    vectors = np.round(vectors, decimals=decimals)

    order = np.lexsort(
        (
            vectors[:, 2],
            vectors[:, 1],
            vectors[:, 0],
        )
    )

    return vectors[order]


def test_preinstantiated_platforms():
    for platform_name in list_platforms():
        platform_factory = getattr(platform_module, platform_name)
        assert isinstance(platform_factory(), Platform)


def test_list_platforms_matches_registry():
    assert list_platforms() == list(platform_module.PLATFORM_REGISTRY.keys())


def test_platform_from_name():
    for platform_name in list_platforms():
        assert isinstance(platform_from_name(platform_name), Platform)


def test_platform_from_name_invalid():
    with pytest.raises(ValueError, match="Unknown platform"):
        platform_from_name("not-a-valid-platform")


def test_platform_settings_mls():
    survey = Survey.from_xml("data/surveys/toyblocks/mls_toyblocks.xml")

    platform_settings = PlatformSettings(
        x=10,
        y=0,
    )
    scanner_settings = ScannerSettings(pulse_frequency=1000)
    platform_settings.do_force_on_ground(survey.scene)
    survey.add_leg(
        platform_settings=platform_settings,
        scanner_settings=scanner_settings,
    )

    assert math.isclose(
        platform_settings.z, survey.legs[0].platform_settings._cpp_object.position[2]
    )


def test_platform_settings_tls():
    survey = Survey.from_xml("data/surveys/toyblocks/tls_toyblocks.xml")

    platform_settings = PlatformSettings(
        x=10,
        y=0,
    )

    platform_settings.do_force_on_ground(survey.scene)
    survey.add_leg(
        platform_settings=platform_settings,
    )

    assert math.isclose(
        platform_settings.z, survey.legs[0].platform_settings._cpp_object.position[2]
    )


def test_platform_flag_from_xml_set():
    from helios.utils import is_xml_loaded

    platform = Platform.from_xml("data/platforms.xml", platform_id="sr22")
    assert is_xml_loaded(platform)


def test_load_csv_traj_reordering():
    trajectory = load_traj_csv(
        csv="data/trajectories/cycloid.trj",
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )
    assert trajectory.shape == (51,)
    t = np.void(
        [(3.7, -1.04719755, 1.04719755, 5.77180384, 13.002584, 1.122905, 400.0)],
        dtype=traj_csv_dtype,
    )
    assert all([np.isclose(a, b) for a, b in zip(trajectory[0], t[0])])
    assert trajectory.dtype.names == (
        "t",
        "roll",
        "pitch",
        "yaw",
        "x",
        "y",
        "z",
    ), (
        f"Expected names: ('t', 'roll', 'pitch', 'yaw', 'x', 'y', 'z'), got {trajectory.dtype.names}"
    )


def test_load_interpolate_platform_invalid_id():
    trajectory = load_traj_csv(
        csv="data/trajectories/cycloid.trj",
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )

    with pytest.raises(HeliosException):
        ip = Platform.load_interpolate_platform(
            trajectory=trajectory,
            platform_file="data/platforms.xml",
            platform_id="blah",
        )


def test_load_interpolate_platform_invalid_trajectory():
    trajectory1 = np.zeros((2, 7), dtype=traj_csv_dtype)
    with pytest.raises(ValueError):
        Platform.load_interpolate_platform(
            trajectory=trajectory1,
            platform_file="data/platforms.xml",
            platform_id="sr22",
        )

    trajectory2 = np.zeros((51, 7, 2))
    with pytest.raises(ValueError):
        ip = Platform.load_interpolate_platform(
            trajectory=trajectory2,
            platform_file="data/platforms.xml",
            platform_id="sr22",
        )

    trajectory3 = list(range(51))
    with pytest.raises(ValueError):
        ip = Platform.load_interpolate_platform(
            trajectory=trajectory3,
            platform_file="data/platforms.xml",
            platform_id="sr22",
        )
    trajectory4 = np.array(
        [3.7, -1.04719755, 1.04719755, 5.77180384, 13.002584, 1.122905, 400.0]
    )

    with pytest.raises(ValueError):
        ip = Platform.load_interpolate_platform(
            trajectory=trajectory4,
            platform_file="data/platforms.xml",
            platform_id="sr22",
        )


def test_load_interpolate_platform():
    trajectory = load_traj_csv(
        csv="data/trajectories/cycloid.trj",
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )

    ip = Platform.load_interpolate_platform(
        trajectory=trajectory,
        platform_file="data/platforms.xml",
        platform_id="sr22",
    )

    assert isinstance(ip, Platform)


def test_load_interpolate_platform_wrong_rotation_spec():
    trajectory = load_traj_csv(
        csv="data/trajectories/cycloid.trj",
        xIndex=4,
        yIndex=5,
        zIndex=6,
        rollIndex=1,
        pitchIndex=2,
        yawIndex=3,
    )

    with pytest.raises(ValidationError):
        Platform.load_interpolate_platform(
            trajectory=trajectory,
            platform_file="data/platforms.xml",
            platform_id="sr22",
            interpolation_method="blah",
        )


def test_dynamic_platform_settings_clone_and_deepcopy():
    settings = DynamicPlatformSettings(
        x=1.0,
        y=2.0,
        z=3.0,
        speed_m_s=4.0,
        trajectory_settings=TrajectorySettings(start_time=5.0, end_time=6.0),
    )
    settings._scene_shift_done = True
    settings.runtime_marker = "temporary"

    for copied in (settings.clone(), copy.deepcopy(settings)):
        assert copied is not settings
        assert copied.trajectory_settings is not settings.trajectory_settings
        assert copied.x == 1.0
        assert copied.y == 2.0
        assert copied.z == 3.0
        assert copied.speed_m_s == 4.0
        assert copied.trajectory_settings.start_time == 5.0
        assert copied.trajectory_settings.end_time == 6.0
        assert not hasattr(copied, "_scene_shift_done")
        assert not hasattr(copied, "runtime_marker")

        copied.trajectory_settings.start_time = 7.0
        assert settings.trajectory_settings.start_time == 5.0

        survey = Survey(
            scanner=riegl_vz_400(),
            platform=tripod(),
            scene=StaticScene(
                scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
            ),
        )
        survey.add_leg(
            platform_settings=copied,
            scanner_settings=ScannerSettings(
                pulse_frequency=2000,
                scan_angle="20 deg",
                head_rotation="10 deg/s",
                rotation_start_angle="0 deg",
                rotation_stop_angle="10 deg",
                scan_frequency=120,
            ),
        )
        points, trajectory = survey.run(
            format=OutputFormat.NPY, execution_settings=ExecutionSettings(num_threads=1)
        )
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


def test_platform_clone_and_deepcopy_run_survey():
    platform = tripod()
    platform.runtime_note = "temporary"

    for copied in (platform.clone(), copy.deepcopy(platform)):
        assert copied is not platform
        assert copied._cpp_object is not platform._cpp_object
        assert not hasattr(copied, "runtime_note")

        survey = Survey(
            scanner=riegl_vz_400(),
            platform=copied,
            scene=StaticScene(
                scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
            ),
        )
        survey.add_leg(
            scanner_settings=ScannerSettings(
                pulse_frequency=2000,
                scan_angle="20 deg",
                head_rotation="10 deg/s",
                rotation_start_angle="0 deg",
                rotation_stop_angle="10 deg",
                scan_frequency=120,
            ),
            x=0,
            y=0,
            z=0,
        )
        points, trajectory = survey.run(
            format=OutputFormat.NPY, execution_settings=ExecutionSettings(num_threads=1)
        )
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


@pytest.mark.parametrize(
    "settings",
    [
        PlatformSettings(x=1.0, y=2.0, z=3.0),
        StaticPlatformSettings(x=1.0, y=2.0, z=3.0, force_on_ground=False),
    ],
)
def test_platform_settings_clone_and_deepcopy_run_survey(settings):
    settings.runtime_note = "temporary"

    for copied in (settings.clone(), copy.deepcopy(settings)):
        assert copied is not settings
        assert copied._cpp_object is not settings._cpp_object
        assert copied.x == settings.x
        assert copied.y == settings.y
        assert copied.z == settings.z
        assert not hasattr(copied, "runtime_note")

        survey = Survey(
            scanner=riegl_vz_400(),
            platform=tripod(),
            scene=StaticScene(
                scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
            ),
        )
        survey.add_leg(
            platform_settings=copied,
            scanner_settings=ScannerSettings(
                pulse_frequency=2000,
                scan_angle="20 deg",
                head_rotation="10 deg/s",
                rotation_start_angle="0 deg",
                rotation_stop_angle="10 deg",
                scan_frequency=120,
            ),
        )
        points, trajectory = survey.run(
            format=OutputFormat.NPY, execution_settings=ExecutionSettings(num_threads=1)
        )
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


def test_trajectory_settings_clone_and_deepcopy():
    settings = TrajectorySettings(start_time=5.0, end_time=6.0, teleport_to_start=True)
    settings.runtime_note = "temporary"
    survey = Survey.from_xml("data/surveys/demo/als_interpolated_trajectory.xml")
    for leg in survey.legs:
        leg.scanner_settings.pulse_frequency = 2000
        leg.scanner_settings.scan_frequency = 20
        leg.scanner_settings.head_rotation = "30 deg/s"

    for copied in (settings.clone(), copy.deepcopy(settings)):
        assert copied is not settings
        assert copied.start_time == 5.0
        assert copied.end_time == 6.0
        assert copied.teleport_to_start is True
        assert not hasattr(copied, "runtime_note")

        cloned_survey = survey.clone()
        cloned_survey.legs[0].trajectory_settings = copied
        points, trajectory = cloned_survey.run(
            format=OutputFormat.NPY, execution_settings=ExecutionSettings(num_threads=1)
        )
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


def make_dummy_trajectory():
    traj = np.zeros(
        4,
        dtype=traj_csv_dtype,
    )

    traj["t"] = [0.0, 1.0, 2.0, 3.0]

    traj["roll"] = np.radians([10.0, 20.0, 30.0, 40.0])
    traj["pitch"] = np.radians([5.0, -10.0, 15.0, -20.0])
    traj["yaw"] = np.radians([15.0, 35.0, -25.0, 60.0])

    traj["x"] = [0.0, 10.0, 20.0, 30.0]
    traj["y"] = [0.0, 0.0, 0.0, 0.0]
    traj["z"] = [100.0, 100.0, 100.0, 100.0]

    return traj


def run_interpolated_survey(
    *,
    trajectory,
    interpolation_method: str,
):
    scanner = scanner_from_name("riegl_lms_q560")
    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene = StaticScene(scene_parts=[box])

    platform = Platform.load_interpolate_platform(
        trajectory=trajectory,
        platform_file="data/platforms.xml",
        platform_id="sr22",
        sync_gps_time=True,
        interpolation_method=interpolation_method,
        is_roll_pitch_yaw_in_radians=True,
    )

    scanner_settings = ScannerSettings(
        is_active=True,
        pulse_frequency=10_000 * units.Hz,
        scan_frequency=20 * units.Hz,
        trajectory_time_interval=0.01,
    )

    trajectory_settings = TrajectorySettings(
        start_time=0.0,
        end_time=3.0,
    )

    survey = Survey(
        scanner=scanner,
        platform=platform,
        scene=scene,
    )

    survey.add_leg(
        scanner_settings=scanner_settings,
        trajectory_settings=trajectory_settings,
    )

    points, traj = survey.run()
    return points, traj


def test_canonical_and_arinc_differ_for_mixed_roll_pitch_yaw():
    trajectory = make_dummy_trajectory()

    canonical_points, canonical_traj = run_interpolated_survey(
        trajectory=trajectory,
        interpolation_method="CANONICAL",
    )

    arinc_points, arinc_traj = run_interpolated_survey(
        trajectory=trajectory,
        interpolation_method="ARINC 705",
    )

    np.testing.assert_allclose(
        canonical_traj["position"],
        arinc_traj["position"],
        rtol=0.0,
        atol=1e-9,
    )

    assert not np.allclose(
        sorted_vectors(canonical_points["beam_direction"])[:10],
        sorted_vectors(arinc_points["beam_direction"])[:10],
        rtol=1e-7,
        atol=1e-9,
    )


def expected_position_from_input(input_traj, t):
    matches = input_traj[np.isclose(input_traj["t"], t, rtol=0.0, atol=1e-12)]
    assert len(matches) == 1

    row = matches[0]
    return np.array([row["x"], row["y"], row["z"]], dtype=float)


@pytest.mark.parametrize("interpolation_method", ["CANONICAL", "ARINC 705"])
def test_interpolated_trajectory_positions_match_input_at_frontier_times(
    interpolation_method,
):
    trajectory = make_dummy_trajectory()

    _, output_traj = run_interpolated_survey(
        trajectory=trajectory,
        interpolation_method=interpolation_method,
    )

    for t in [0.0, 1.0, 2.0]:
        idx = np.argmin(np.abs(output_traj["gps_time"] - t))
        actual = output_traj[idx]

        np.testing.assert_allclose(
            actual["gps_time"],
            t,
            rtol=0.0,
            atol=2e-4,
        )

        np.testing.assert_allclose(
            actual["position"],
            expected_position_from_input(trajectory, t),
            rtol=0.0,
            atol=2e-3,
        )


@pytest.mark.parametrize("interpolation_method", ["CANONICAL", "ARINC 705"])
def test_interpolated_platform_regression_signature(interpolation_method):
    trajectory = make_dummy_trajectory()

    points, _ = run_interpolated_survey(
        trajectory=trajectory,
        interpolation_method=interpolation_method,
    )

    np.testing.assert_allclose(
        sorted_vectors(points["beam_direction"])[:10],
        EXPECTED_BEAM_DIRECTION_SIGNATURES[interpolation_method],
        rtol=1e-7,
        atol=1e-9,
        err_msg=f"{interpolation_method} beam_direction regression mismatch",
    )

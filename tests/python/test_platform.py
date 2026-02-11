from helios.platforms import *
from helios.survey import *

import math
import pytest
from pydantic import ValidationError
from helios import HeliosException


def test_preinstantiated_platforms():
    assert isinstance(sr22(), Platform)
    assert isinstance(quadcopter(), Platform)
    assert isinstance(copter_linearpath(), Platform)
    assert isinstance(tractor(), Platform)
    assert isinstance(tractor_leftside(), Platform)
    assert isinstance(vehicle_linearpath(), Platform)
    assert isinstance(vmx_450_car_left(), Platform)
    assert isinstance(vmx_450_car_right(), Platform)
    assert isinstance(vmq_1ha_car(), Platform)
    assert isinstance(simple_linearpath(), Platform)
    assert isinstance(tripod(), Platform)


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
    ), f"Expected names: ('t', 'roll', 'pitch', 'yaw', 'x', 'y', 'z'), got {trajectory.dtype.names}"


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

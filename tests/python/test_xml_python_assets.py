import numpy as np
import pytest

from helios.platforms import Platform, platform_from_name, tripod
from helios.scanner import Scanner, ScannerSettings, scanner_from_name
from helios.scene import ScenePart, StaticScene
from helios.settings import ExecutionSettings, OutputFormat, ProgressBarStrategy
from helios.survey import Survey
from helios.utils import set_rng_seed

SCANNERS = (
    ("leica_als50", "data/scanners_als.xml", "leica_als50"),
    ("leica_als50_ii", "data/scanners_als.xml", "leica_als50-ii"),
    ("optech_2033", "data/scanners_als.xml", "optech_2033"),
    ("optech_3100", "data/scanners_als.xml", "optech_3100"),
    ("optech_galaxy", "data/scanners_als.xml", "optech_galaxy"),
    ("riegl_lms_q560", "data/scanners_als.xml", "riegl_lms-q560"),
    ("riegl_lms_q780", "data/scanners_als.xml", "riegl_lms-q780"),
    ("riegl_vq_780i", "data/scanners_als.xml", "riegl_vq_780i"),
    ("riegl_vux_1uav", "data/scanners_als.xml", "riegl_vux-1uav"),
    ("riegl_vux_1uav22", "data/scanners_als.xml", "riegl_vux-1uav22"),
    ("riegl_vux_1ha22", "data/scanners_als.xml", "riegl_vux-1ha22"),
    ("riegl_vq_880g", "data/scanners_als.xml", "riegl_vq-880g"),
    ("riegl_vq_1560i", "data/scanners_als.xml", "riegl_vq-1560i"),
    ("livox_mid70_als", "data/scanners_als.xml", "livox_mid-70"),
    ("livox_mid100_als", "data/scanners_als.xml", "livox-mid-100"),
    (
        "dji_zenmuse_l2_repetitive",
        "data/scanners_als.xml",
        "dji-zenmuse-l2-repetitive",
    ),
    (
        "dji_zenmuse_l2_non_repetitive",
        "data/scanners_als.xml",
        "dji-zenmuse-l2-non-repetitive",
    ),
    ("riegl_vz_400", "data/scanners_tls.xml", "riegl_vz400"),
    ("riegl_vz_1000", "data/scanners_tls.xml", "riegl_vz1000"),
    ("riegl_vz_2000i", "data/scanners_tls.xml", "riegl_vz2000i"),
    ("riegl_vz_600i", "data/scanners_tls.xml", "riegl_vz600i"),
    ("riegl_vq_450", "data/scanners_tls.xml", "riegl_vq-450"),
    ("livox_mid40", "data/scanners_tls.xml", "livox_mid-40"),
    ("livox_mid100", "data/scanners_tls.xml", "livox-mid-100"),
    ("livox_mid70", "data/scanners_tls.xml", "livox_mid-70"),
    ("vlp16", "data/scanners_tls.xml", "vlp16"),
    ("velodyne_hdl_64e", "data/scanners_tls.xml", "velodyne_hdl-64e"),
    ("tractor_scanner", "data/scanners_tls.xml", "tractorscanner"),
    ("pano_scanner", "data/scanners_tls.xml", "panoscanner"),
)

PLATFORMS = (
    ("sr22", "sr22"),
    ("copter_linearpath", "copter_linearpath"),
    ("tractor", "tractor"),
    ("tractor_leftside", "tractor_leftside"),
    ("vehicle_linearpath", "vehicle_linearpath"),
    ("vmx_450_car_left", "vmx-450-car-left"),
    ("vmx_450_car_right", "vmx-450-car-right"),
    ("vmq_1ha_car", "vmq-1ha-car-0"),
    ("simple_linearpath", "simple_linearpath"),
    ("tripod", "tripod"),
)

EXECUTION_SETTINGS = ExecutionSettings(
    num_threads=1,
    progressbar=ProgressBarStrategy.NONE,
)


def _devices(scanner):
    if hasattr(scanner, "channels"):
        return scanner.channels
    return (scanner.device,)


def _enclosing_box_scene(scanner):
    devices = _devices(scanner)
    minimum_range = max(device.detector.range_min for device in devices)
    maximum_range = min(device.detector.range_max for device in devices)
    distance = max(10, minimum_range + 10)
    if np.isfinite(maximum_range) and distance >= maximum_range:
        distance = (minimum_range + maximum_range) / 2

    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    return StaticScene([box.scale(distance / 50)])


def _run(scanner, platform):
    scan_frequency = 10
    for device in _devices(scanner):
        if hasattr(device.optics, "scan_frequency_min"):
            scan_frequency = max(scan_frequency, device.optics.scan_frequency_min)
            # Admit the shared low-frequency operating point equally in both
            # definitions, including scanners whose sweep is head-controlled.
            if device.optics.scan_frequency_max < scan_frequency:
                device.optics.scan_frequency_max = scan_frequency

    scan_angle = min(
        0.1,
        min(
            getattr(device.optics, "scan_angle_max", 0.1)
            for device in _devices(scanner)
        ),
    )
    survey = Survey(
        scanner=scanner,
        platform=platform,
        scene=_enclosing_box_scene(scanner),
    )
    survey.add_leg(
        x=0,
        y=0,
        z=0,
        scanner_settings=ScannerSettings(
            pulse_frequency=2000,
            scan_angle=scan_angle,
            scan_frequency=scan_frequency,
            max_duration=0.002,
            trajectory_time_interval=1,
        ),
    )
    # Dynamic platforms derive their heading and motion from the next leg.
    survey.add_leg(
        x=1,
        y=0,
        z=0,
        scanner_settings=ScannerSettings(
            is_active=False,
            pulse_frequency=2000,
            scan_angle=scan_angle,
            scan_frequency=scan_frequency,
            max_duration=0.001,
            trajectory_time_interval=1,
        ),
    )

    set_rng_seed(42)
    return survey.run(
        format=OutputFormat.NPY,
        execution_settings=EXECUTION_SETTINGS,
    )


def _assert_simulations_equal(expected, actual):
    expected_points, expected_trajectory = expected
    actual_points, actual_trajectory = actual

    assert 0 < len(expected_points) <= 512
    assert len(expected_points) == len(actual_points)
    assert len(expected_trajectory) == len(actual_trajectory)

    for field in expected_points.dtype.names:
        np.testing.assert_allclose(
            actual_points[field],
            expected_points[field],
            rtol=1e-7,
            atol=1e-7,
        )
    for field in expected_trajectory.dtype.names:
        np.testing.assert_allclose(
            actual_trajectory[field],
            expected_trajectory[field],
            rtol=1e-7,
            atol=1e-7,
        )


@pytest.mark.parametrize(
    ("scanner_name", "scanner_file", "scanner_id"),
    SCANNERS,
    ids=[scanner_name for scanner_name, _, _ in SCANNERS],
)
def test_python_scanner_matches_xml(scanner_name, scanner_file, scanner_id):
    python_result = _run(scanner_from_name(scanner_name), tripod())
    xml_result = _run(
        Scanner.from_xml(scanner_file, scanner_id=scanner_id),
        tripod(),
    )

    _assert_simulations_equal(python_result, xml_result)


@pytest.mark.parametrize(
    ("platform_name", "platform_id"),
    PLATFORMS,
    ids=[platform_name for platform_name, _ in PLATFORMS],
)
def test_python_platform_matches_xml(platform_name, platform_id):
    python_result = _run(
        scanner_from_name("riegl_vz_400"),
        platform_from_name(platform_name),
    )
    xml_result = _run(
        scanner_from_name("riegl_vz_400"),
        Platform.from_xml("data/platforms.xml", platform_id=platform_id),
    )

    _assert_simulations_equal(python_result, xml_result)

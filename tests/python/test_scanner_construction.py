import copy

import _helios
import numpy as np
import pytest

from helios import ExecutionSettings, FullWaveformSettings, Rotation, units
from helios.platforms import StaticPlatform
from helios.scanner import (
    ConicOptics,
    Detector,
    MultiScanner,
    RotatingOptics,
    ScannerHead,
    ScanningDevice,
    SingleScanner,
)
from helios.scene import ScenePart, StaticScene
from helios.survey import Survey


def make_device(device_id="channel", **kwargs):
    values = {
        "id": device_id,
        "supported_pulse_frequencies": (100_000 * units.Hz,),
        "optics": ConicOptics(
            scan_angle_max=45 * units.deg,
            scan_frequency_min=1 * units.Hz,
            scan_frequency_max=100 * units.Hz,
        ),
    }
    values.update(kwargs)
    return ScanningDevice(**values)


def quaternion(rotation):
    return np.array((rotation.q0, rotation.q1, rotation.q2, rotation.q3))


def test_rotation_factories_and_scene_part_share_conversion():
    rotation = Rotation.from_axis_angle(axis=(0, 0, 1), angle=30 * units.deg)
    with_model = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    with_parameters = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    with_model.rotate(rotation=rotation)
    with_parameters.rotate(axis=(0, 0, 1), angle=30 * units.deg)

    np.testing.assert_allclose(
        [vertex.position for vertex in with_model._cpp_object.all_vertices],
        [vertex.position for vertex in with_parameters._cpp_object.all_vertices],
    )

    composed = Rotation.from_axis_angles(
        rotations=(("x", -90 * units.deg), ("z", 90 * units.deg))
    )
    expected = _helios.Rotation((0, 0, 1), np.pi / 2).apply_to(
        _helios.Rotation((1, 0, 0), -np.pi / 2)
    )
    np.testing.assert_allclose(quaternion(composed), quaternion(expected))


def test_single_scanner_construction_and_backend_propagation():
    device = make_device(
        "riegl_vz400",
        supported_pulse_frequencies=(
            100_000 * units.Hz,
            300_000 * units.Hz,
        ),
        beam_origin_position=(0, 0, 0.2),
        pulse_length=5 * units.ns,
        wavelength=1064 * units.nm,
        atmospheric_visibility=23 * units.km,
        optics=RotatingOptics(
            scan_angle_max=120 * units.deg,
            scan_angle_effective_max=50 * units.deg,
            scan_frequency_min=3 * units.Hz,
            scan_frequency_max=120 * units.Hz,
        ),
        detector=Detector(accuracy=0.005 * units.m, range_min=1.5 * units.m),
        head=ScannerHead(
            rotation_axis=(0, 0, 1),
            rotation_speed_max=60 * units.deg / units.s,
        ),
        full_waveform=FullWaveformSettings(beam_sample_quality=3),
    )
    scanner = SingleScanner(device=device)

    assert scanner.device._cpp_object is scanner._cpp_object.device
    assert scanner.device.pulse_length == 5.0
    assert scanner.device.wavelength == pytest.approx(1064e-9)
    assert scanner.device.atmospheric_visibility == 23_000.0
    assert scanner.device.full_waveform.window_size == pytest.approx(1.25e-9)

    scanner.device.pulse_length = 8 * units.ns
    assert scanner._cpp_object.device.pulse_length == 8.0
    scanner._cpp_object.device.average_power = 7.0
    assert scanner.device.average_power == 7.0

    cloned = copy.deepcopy(scanner)
    assert isinstance(cloned, SingleScanner)
    assert cloned.device.id == "riegl_vz400"


def test_multiscanner_channels_are_scanner_owned_and_independent():
    shared_optics = ConicOptics(
        scan_angle_max=45 * units.deg,
        scan_frequency_min=40 * units.Hz,
        scan_frequency_max=600 * units.Hz,
    )
    scanner = MultiScanner(
        channels=(
            make_device(
                "0",
                optics=shared_optics,
                wavelength=532 * units.nm,
                beam_origin_attitude=Rotation.from_axis_angle(
                    axis=(0, 1, 0), angle=14 * units.deg
                ),
            ),
            make_device(
                "1",
                optics=shared_optics,
                wavelength=1064 * units.nm,
                beam_origin_attitude=Rotation.from_axis_angle(
                    axis=(0, 1, 0), angle=-14 * units.deg
                ),
            ),
        ),
    )

    assert len(scanner.channels) == 2
    assert scanner.channels[0]._cpp_object is scanner._cpp_object.channels[0]
    assert scanner.channels[-1].id == "1"

    scanner.channels[0].optics.scan_angle_max = 20 * units.deg
    assert scanner.channels[0]._cpp_object.optics.scan_angle_max == pytest.approx(
        np.deg2rad(20)
    )
    assert scanner.channels[1].optics.scan_angle_max == pytest.approx(np.deg2rad(45))


def test_programmatic_scanner_yaml_roundtrip(tmp_path):
    scanner = SingleScanner(
        device=make_device("device", pulse_length=6 * units.ns),
    )

    path = scanner.to_yaml(tmp_path / "scanner.yaml", shallow=True)
    assert "model_ref" not in path.read_text(encoding="utf-8")
    loaded = SingleScanner.from_yaml(path)

    assert loaded.device.id == "device"
    assert loaded.device.pulse_length == 6.0
    assert isinstance(loaded.device.optics, ConicOptics)


def test_programmatic_scanner_and_platform_run_survey():
    scanner = SingleScanner(
        device=make_device(
            "programmatic",
            supported_pulse_frequencies=(2_000 * units.Hz,),
            optics=RotatingOptics(
                scan_angle_max=120 * units.deg,
                scan_angle_effective_max=50 * units.deg,
                scan_frequency_min=3 * units.Hz,
                scan_frequency_max=120 * units.Hz,
            ),
            detector=Detector(accuracy=0.005 * units.m, range_min=1.5 * units.m),
            head=ScannerHead(
                rotation_axis=(0, 0, 1),
                rotation_speed_max=60 * units.deg / units.s,
            ),
        ),
    )
    platform = StaticPlatform(scanner_mount_position=(0, 0, 1.5))
    scene = StaticScene(
        scene_parts=(ScenePart.from_obj("data/sceneparts/basic/box/box100.obj"),)
    )
    survey = Survey(scanner=scanner, platform=platform, scene=scene)
    survey.add_leg(
        x=0,
        y=-20,
        z=0,
        pulse_frequency=2_000 * units.Hz,
        scan_frequency=120 * units.Hz,
        scan_angle=20 * units.deg,
        head_rotation=10 * units.deg / units.s,
        rotation_start_angle=0 * units.deg,
        rotation_stop_angle=10 * units.deg,
    )

    points, trajectory = survey.run(
        execution_settings=ExecutionSettings(num_threads=1, progressbar="none")
    )

    assert points.shape[0] > 0
    assert trajectory.shape[0] > 0


def test_platform_construction_and_backend_propagation(tmp_path):
    platform = StaticPlatform(
        scanner_mount_position=(0, 0, 1.5),
    )

    assert platform._cpp_object.scanner_mount_position == (0.0, 0.0, 1.5)

    path = platform.to_yaml(tmp_path / "platform.yaml", shallow=True)
    assert "model_ref" not in path.read_text(encoding="utf-8")

"""Concrete energy-model bindings and the lifetime of their borrowed inputs."""

import gc
import math
import weakref

import _helios
import pytest


def make_energy_device():
    device = _helios.ScanningDevice(
        0,
        "energy-test",
        0.0003,
        (0, 0, 0),
        _helios.Rotation(),
        [100000],
        5.0,
        4.0,
        0.99,
        0.15,
        23.0,
        1064e-9,
    )
    settings = _helios.FWFSettings()
    settings.beam_sample_quality = 3
    device.fwf_settings = settings
    device.prepare_simulation()
    return device


@pytest.fixture
def energy_device():
    return make_energy_device()


def test_concrete_energy_model_calculations(energy_device):
    model = _helios.EnergyModel(device=energy_device)
    area = model.compute_target_area(target_range_m=100.0, subray_index=0)
    cutoff = math.atan(2.0 * math.tan(0.0003 / 2.0))
    assert area == pytest.approx(math.pi * (100 * math.tan(cutoff / 5)) ** 2)
    assert model.compute_target_area(200.0, 0) == pytest.approx(4 * area)
    material = _helios.Material()
    material.reflectance = 0.25
    ratio = model.compute_power_ratio(
        incidence_cosine=1.0, target_range_m=100.0, material=material, subray_index=0
    )
    power = model.compute_received_power(1.0, 100.0, material, 0)
    assert power == pytest.approx(4 * ratio, rel=1e-12, abs=0)
    assert energy_device.compute_received_power(1.0, 100.0, material, 0) == power
    assert energy_device.compute_power_ratio(1.0, 100.0, material, 0) == ratio
    sigma = 4 * math.pi * 0.25 * area
    assert model.compute_received_power_with_sigma(100.0, sigma, 0) == pytest.approx(
        power, rel=1e-12, abs=0
    )
    assert model.compute_power_ratio_with_sigma(100.0, sigma, 0) == pytest.approx(
        ratio, rel=1e-12, abs=0
    )
    assert model.compute_emitted_power(subray_index=0) == pytest.approx(
        4 * energy_device.subrays[0].share
    )
    for name in (
        "ReceivedPowerArgs",
        "EmittedPowerArgs",
        "TargetAreaArgs",
        "CrossSectionArgs",
    ):
        assert not hasattr(_helios, name)
    for name in (
        "compute_intensity",
        "compute_intensity_from_cosine",
        "compute_cross_section",
        "compute_transfer_from_cosine",
    ):
        assert not hasattr(model, name)
    with pytest.raises(IndexError):
        model.compute_target_area(100.0, len(energy_device.subrays))


@pytest.mark.parametrize("source", ["constructor", "device"])
def test_energy_model_keeps_device_alive(source):
    device = make_energy_device()
    device_ref = weakref.ref(device)
    if source == "constructor":
        model = _helios.EnergyModel(device)
    else:
        device.prepare_simulation()
        model = device.energy_model
    del device
    gc.collect()
    assert device_ref() is not None
    del model
    gc.collect()
    assert device_ref() is None


def test_device_prepares_concrete_energy_model(energy_device):
    energy_device.prepare_simulation()
    assert isinstance(energy_device.energy_model, _helios.EnergyModel)
    expected = _helios.EnergyModel(energy_device).compute_target_area(100.0, 0)
    assert energy_device.energy_model.compute_target_area(100.0, 0) == expected


@pytest.mark.parametrize("quality", [1, 3, 8])
@pytest.mark.parametrize("factor", [0.5, 1.0, 2.0])
def test_subray_table_captured_power(energy_device, quality, factor):
    settings = _helios.FWFSettings()
    settings.beam_sample_quality = quality
    settings.beam_sampling_factor = factor
    energy_device.fwf_settings = settings
    if quality != 3 or factor != 2.0:
        with pytest.raises(RuntimeError, match="prepare"):
            _ = energy_device.subrays
    energy_device.prepare_simulation()
    rays = energy_device.subrays
    captured = -math.expm1(-2 * factor**2)
    assert sum(ray.share for ray in rays) == pytest.approx(captured, rel=1e-12)
    assert sum(
        energy_device.energy_model.compute_emitted_power(i) for i in range(len(rays))
    ) == pytest.approx(4 * captured, rel=1e-12)
    with pytest.raises(AttributeError):
        rays[0].share = 1.0
    energy_device.prepare_simulation()
    assert len(energy_device.subrays) == len(rays)


@pytest.mark.parametrize("angle", [0.0, 0.2, math.pi / 4, 1.2, math.pi / 2])
def test_cosine_power_matches_phong_formula(energy_device, angle):
    model = _helios.EnergyModel(energy_device)
    material = _helios.Material()
    material.reflectance = 0.4
    material.diffuse_components = [0.75, 0, 0, 0]
    material.specular_components = [0.25, 0, 0, 0]
    material.specularity = 0.25
    material.specular_exponent = 2.5
    cosine = math.cos(angle)
    response = 0.4 * (0.75 * cosine + 0.25 * abs(math.cos(2 * angle)) ** 2.5)
    # Divide out an independently evaluated direction-independent reference.
    reference = _helios.Material()
    reference.reflectance = 1.0
    baseline = model.compute_received_power(1.0, 100.0, reference, 0)
    assert model.compute_received_power(cosine, 100.0, material, 0) == pytest.approx(
        baseline * response, rel=1e-12, abs=0
    )


@pytest.mark.parametrize("side", [-1.0, 1.0])
def test_triangle_incidence_cosine(side):
    triangle = _helios.Triangle(
        _helios.Vertex(0, 0, 0),
        _helios.Vertex(1, 0, 0),
        _helios.Vertex(0, 1, 0),
    )
    angle = 0.4
    origin, point = (0, 0, 1), (0, 0, 0)
    direction = (math.sin(angle), 0, side * math.cos(angle))
    assert triangle.incidence_angle_cosine(origin, direction, point) == pytest.approx(
        math.cos(angle)
    )
    assert triangle.incidence_angle(origin, direction, point) == pytest.approx(angle)

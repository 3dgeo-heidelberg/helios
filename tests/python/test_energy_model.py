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
        1.0,
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


@pytest.mark.parametrize("argument_type", ["received", "cross_section"])
def test_energy_arguments_keep_material_alive(argument_type):
    material = _helios.Material()
    material.reflectance = 0.25
    material_ref = weakref.ref(material)
    if argument_type == "received":
        args = _helios.ReceivedPowerArgs(
            target_range=100.0,
            incidence_angle=0.3,
            material=material,
            subray_index=1,
        )
        assert args.target_range == 100.0
        assert args.incidence_angle == 0.3
        assert args.subray_index == 1
    else:
        args = _helios.CrossSectionArgs(
            material=material,
            brdf=0.25,
            target_area=2.0,
        )
        assert args.brdf == 0.25
        assert args.target_area == 2.0

    assert args.material is material
    with pytest.raises(AttributeError):
        args.material = _helios.Material()
    del material
    gc.collect()
    assert material_ref() is not None
    assert args.material.reflectance == 0.25
    del args
    gc.collect()
    assert material_ref() is None


@pytest.mark.parametrize(
    "argument_class, kwargs",
    [
        (
            _helios.ReceivedPowerArgs,
            {
                "target_range": 100.0,
                "incidence_angle": 0.3,
                "subray_index": 1,
            },
        ),
        (
            _helios.EmittedPowerArgs,
            {
                "subray_index": 1,
            },
        ),
        (
            _helios.TargetAreaArgs,
            {
                "target_range_squared": 10000.0,
                "subray_index": 1,
            },
        ),
        (_helios.CrossSectionArgs, {"brdf": 0.25, "target_area": 2.0}),
    ],
)
def test_energy_argument_fields_are_readonly(argument_class, kwargs):
    if argument_class in (_helios.ReceivedPowerArgs, _helios.CrossSectionArgs):
        args = argument_class(material=_helios.Material(), **kwargs)
    else:
        args = argument_class(**kwargs)
    for field, value in kwargs.items():
        assert getattr(args, field) == value
        with pytest.raises(AttributeError):
            setattr(args, field, value)


def test_concrete_energy_model_calculations(energy_device):
    model = _helios.EnergyModel(device=energy_device)
    area_args = _helios.TargetAreaArgs(10000.0, 0)
    area = model.compute_target_area(area_args)
    cutoff = math.atan(2.0 * math.tan(0.0003 / 2.0))
    assert area == pytest.approx(math.pi * (100 * math.tan(cutoff / 5)) ** 2)
    sigma = model.compute_cross_section(
        _helios.CrossSectionArgs(_helios.Material(), 0.25, area)
    )
    assert sigma == pytest.approx(4 * math.pi * 0.25 * area)
    power = model.compute_emitted_power(_helios.EmittedPowerArgs(0))
    assert math.isfinite(power) and power > 0
    with pytest.raises(TypeError):
        model.compute_emitted_power(area_args)
    with pytest.raises(TypeError):
        model.compute_received_power(area_args)
    assert not hasattr(_helios, "BaseEnergyModel")


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
    args = _helios.TargetAreaArgs(10000.0, 0)
    expected = _helios.EnergyModel(energy_device).compute_target_area(args)
    assert energy_device.energy_model.compute_target_area(args) == expected


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
        energy_device.energy_model.compute_emitted_power(_helios.EmittedPowerArgs(i))
        for i in range(len(rays))
    ) == pytest.approx(4 * captured, rel=1e-12)
    with pytest.raises(AttributeError):
        rays[0].share = 1.0
    energy_device.prepare_simulation()
    assert len(energy_device.subrays) == len(rays)


@pytest.mark.parametrize("angle", [0.0, 0.2, math.pi / 4, 1.2, math.pi / 2])
def test_cosine_intensity_matches_angle_api(energy_device, angle):
    model = _helios.EnergyModel(energy_device)
    material = _helios.Material()
    material.reflectance = 0.4
    material.diffuse_components = [0.75, 0, 0, 0]
    material.specular_components = [0.25, 0, 0, 0]
    material.specularity = 0.25
    material.specular_exponent = 2.5
    expected = model.compute_received_power(
        _helios.ReceivedPowerArgs(100.0, angle, material, 0)
    )
    assert model.compute_intensity_from_cosine(
        math.cos(angle), 100.0, material, 0
    ) == pytest.approx(expected, rel=1e-12)


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

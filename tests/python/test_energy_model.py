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
            subray_radius_step=1,
        )
        assert args.target_range == 100.0
        assert args.incidence_angle == 0.3
        assert args.subray_radius_step == 1
    else:
        args = _helios.CrossSectionArgs(
            material=material,
            bdrf=0.25,
            target_area=2.0,
        )
        assert args.bdrf == 0.25
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
                "subray_radius_step": 1,
            },
        ),
        (
            _helios.EmittedPowerArgs,
            {
                "target_range": 100.0,
                "target_range_squared": 10000.0,
                "range_min": 0.1,
                "subray_radius_step": 1,
            },
        ),
        (
            _helios.TargetAreaArgs,
            {
                "target_range_squared": 10000.0,
                "subray_radius_step": 1,
            },
        ),
        (_helios.CrossSectionArgs, {"bdrf": 0.25, "target_area": 2.0}),
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
    # BSQ 3: the central patch's angular radius is 0.00003 rad.
    assert area == pytest.approx(math.pi * 0.003**2)
    sigma = model.compute_cross_section(
        _helios.CrossSectionArgs(_helios.Material(), 0.25, area)
    )
    assert sigma == pytest.approx(4 * math.pi * 0.25 * area)
    power = model.compute_emitted_power(
        _helios.EmittedPowerArgs(100.0, 10000.0, 0.1, 0)
    )
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

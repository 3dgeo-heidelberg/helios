# ruff: noqa
from helios.platforms import *
from numpy.lib.recfunctions import unstructured_to_structured


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


def test_platform_defaults():
    Platform()
    sps = StaticPlatformSettings()
    dps = DynamicPlatformSettings()
    pd = Platform(platform_settings=dps)
    ps = Platform(platform_settings=sps)


def test_platform_printable():
    str(Platform())

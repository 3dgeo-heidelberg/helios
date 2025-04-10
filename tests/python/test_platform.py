from helios.platforms import *
from helios.survey import *

import math

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
    survey = Survey.from_xml("data/surveys/demo/mls_wheat_demo.xml")

    platform_settings = PlatformSettings(x=10, y = 0,)
    scanner_settings = ScannerSettings(pulse_frequency=1000)
    platform_settings.force_on_ground(survey.scene)
    survey.add_leg(
        platform_settings=platform_settings,
        scanner_settings=scanner_settings,
    )

    assert math.isclose(platform_settings.z, survey.legs[0].platform_settings._cpp_object.position[2])

def test_platform_settings_tls():
    survey = Survey.from_xml("data/surveys/demo/tls_arbaro_demo_angular_resolution.xml")

    platform_settings = PlatformSettings(x=10, y = 0,)

    platform_settings.force_on_ground(survey.scene)
    survey.add_leg(
        platform_settings=platform_settings,
    )

    assert math.isclose(platform_settings.z, survey.legs[0].platform_settings._cpp_object.position[2])
                          



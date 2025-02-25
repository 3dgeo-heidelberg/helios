from helios.platforms import *


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

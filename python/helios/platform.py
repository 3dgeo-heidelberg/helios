from helios.util import get_asset_directories
from helios.validation import Validatable, ValidatedCppManagedProperty, UpdateableMixin
from pathlib import Path

import _helios


class PlatformSettingsBase(Validatable, UpdateableMixin):
    pass


class PlatformSettings(PlatformSettingsBase):
    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self._cpp_object = _helios.PlatformSettings()
        self.x = x
        self.y = y
        self.z = z

    x: float = ValidatedCppManagedProperty("x")
    y: float = ValidatedCppManagedProperty("y")
    z: float = ValidatedCppManagedProperty("z")


class StaticPlatformSettings(PlatformSettingsBase):
    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self._cpp_object = _helios.PlatformSettings()
        self.x = x
        self.y = y
        self.z = z

    x: float = ValidatedCppManagedProperty("x")
    y: float = ValidatedCppManagedProperty("y")
    z: float = ValidatedCppManagedProperty("z")


class Platform(Validatable):
    @classmethod
    def from_xml(cls, platform_file: Path, platform_id: str = ""):

        _cpp_platform = _helios.read_platform_from_xml(
            platform_file, [str(p) for p in get_asset_directories()], platform_id
        )
        return cls._from_cpp_object(_cpp_platform)


#
# Predefined platforms
#


def sr22():
    return Platform.from_xml("data/platforms.xml", platform_id="sr22")


def quadcopter():
    return Platform.from_xml("data/platforms.xml", platform_id="quadcopter")


def copter_linearpath():
    return Platform.from_xml("data/platforms.xml", platform_id="copter_linearpath")


def tractor():
    return Platform.from_xml("data/platforms.xml", platform_id="tractor")


def tractor_leftside():
    return Platform.from_xml("data/platforms.xml", platform_id="tractor_leftside")


def vehicle_linearpath():
    return Platform.from_xml("data/platforms.xml", platform_id="vehicle_linearpath")


def vmx_450_car_left():
    return Platform.from_xml("data/platforms.xml", platform_id="vmx-450-car-left")


def vmx_450_car_right():
    return Platform.from_xml("data/platforms.xml", platform_id="vmx-450-car-right")


def vmq_1ha_car():
    return Platform.from_xml("data/platforms.xml", platform_id="vmq-1ha-car-0")


def simple_linearpath():
    return Platform.from_xml("data/platforms.xml", platform_id="simple_linearpath")


def tripod():
    return Platform.from_xml("data/platforms.xml", platform_id="tripod")

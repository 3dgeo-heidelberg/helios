from helios.utils import get_asset_directories
from helios.validation import (
    AssetPath,
    Model,
    UpdateableMixin,
    validate_xml_file,
)

from pydantic import validate_call

import _helios


class PlatformSettingsBase(Model, UpdateableMixin, cpp_class=_helios.PlatformSettings):
    pass


class PlatformSettings(PlatformSettingsBase):
    x: float = 0
    y: float = 0
    z: float = 0

    def force_on_ground(self, scene: _helios.Scene):
        """
        Move waypoint z coordinate to ground level
        """
        
        ground_point = scene._cpp_object.ground_point_at((self.x, self.y, self.z))
        self.z = ground_point[2]


class StaticPlatformSettings(PlatformSettingsBase):
    x: float = 0
    y: float = 0
    z: float = 0


class Platform(Model, cpp_class=_helios.Platform):
    @classmethod
    @validate_call
    def from_xml(cls, platform_file: AssetPath, platform_id: str = ""):

        # Validate the XML
        validate_xml_file(platform_file, "xsd/platform.xsd")

        _cpp_platform = _helios.read_platform_from_xml(
            str(platform_file), [str(p) for p in get_asset_directories()], platform_id
        )
        return cls._from_cpp(_cpp_platform)


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

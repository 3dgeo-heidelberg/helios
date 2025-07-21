from helios.scene import StaticScene
from helios.utils import get_asset_directories
from helios.validation import (
    AssetPath,
    Model,
    UpdateableMixin,
    validate_xml_file,
)
from pydantic import Field, validate_call
from typing import Annotated, Any, Literal, Optional

import numpy as np

import _helios


class Printable:
    def __str__(self):
        from pprint import pformat

        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)


traj_csv_dtype = np.dtype(
    [
        ("t", "f8"),
        ("x", "f8"),
        ("y", "f8"),
        ("z", "f8"),
        ("roll", "f8"),
        ("pitch", "f8"),
        ("yaw", "f8"),
    ]
)


class PlatformSettingsBase(
    Printable, Model, UpdateableMixin, cpp_class=_helios.PlatformSettings
):
    pass


class TrajectorySettings(PlatformSettingsBase, cpp_class=_helios.TrajectorySettings):
    start_time: float = 0
    end_time: float = 0
    teleport_to_start: bool = False


class PlatformSettings(PlatformSettingsBase):
    x: float = 0
    y: float = 0
    z: float = 0

    def force_on_ground(self, scene: StaticScene):
        """
        Move waypoint z coordinate to ground level
        """

        ground_point = scene._cpp_object.ground_point_at((self.x, self.y, self.z))
        self.z = ground_point[2]


class StaticPlatformSettings(PlatformSettingsBase):
    x: float = 0
    y: float = 0
    z: float = 0
    force_on_ground = False


class StaticPlatformSettings(PlatformSettings):
    pass


class DynamicPlatformSettings(PlatformSettings):
    trajectory_settings: TrajectorySettings = TrajectorySettings()
    speed_m_s: Annotated[float, Field(ge=0)] = 70


class Platform(Printable, Model, cpp_class=_helios.Platform):
    # TODO: should platform_settings get set from xml as well?
    platform_settings: Optional[PlatformSettings] = None

    @classmethod
    @validate_call
    def from_xml(cls, platform_file: AssetPath, platform_id: str = ""):

        # Validate the XML
        validate_xml_file(platform_file, "xsd/platform.xsd")

        _cpp_platform = _helios.read_platform_from_xml(
            str(platform_file), [str(p) for p in get_asset_directories()], platform_id
        )
        platform = cls._from_cpp(_cpp_platform)
        platform._is_loaded_from_xml = True
        return platform


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

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
from numpydantic import NDArray

import numpy as np

import _helios


class Printable:
    def __str__(self):
        from pprint import pformat

        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)


traj_csv_dtype = np.dtype(
    [
        ("t", "f8"),
        ("roll", "f8"),
        ("pitch", "f8"),
        ("yaw", "f8"),
        ("x", "f8"),
        ("y", "f8"),
        ("z", "f8"),
    ]
)


def load_traj_csv(
    csv: AssetPath,
    tIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 0,
    rollIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 1,
    pitchIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 2,
    yawIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 3,
    xIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 4,
    yIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 5,
    zIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 6,
    trajectory_separator: Annotated[
        str, Field(strict=True, min_length=1, max_length=1)
    ] = ",",
    rpy_in_radians: bool = False,
):
    """Load a csv trajectory from a file.

    The parameters define how the csv is parsed.
    All the ..Index parameters define the column order of the csv.


    Args:
        csv: File path to csv to load.
        tIndex: Column number of time field
        xIndex: Column number of x coordinates
        yIndex: Column number of y coordinates
        zIndex: Column number of z coordinates
        rollIndex: Column number of roll
        pitchIndex: Column number of pitch
        yawIndex: Column number of yaw
        trajectory_separator: Char which separates columns.
    """

    indices = {
        "t": tIndex,
        "x": xIndex,
        "y": yIndex,
        "z": zIndex,
        "roll": rollIndex,
        "pitch": pitchIndex,
        "yaw": yawIndex,
    }

    usecols = [indices[name] for name in traj_csv_dtype.names]
    traj = np.loadtxt(
        csv, dtype=traj_csv_dtype, delimiter=trajectory_separator, usecols=usecols
    )
    if not rpy_in_radians:
        traj["roll"] = np.radians(traj["roll"])
        traj["pitch"] = np.radians(traj["pitch"])
        traj["yaw"] = np.radians(traj["yaw"])

    # TODO: decide on traj structure, flat or nested
    return traj


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

    @classmethod
    @validate_call
    def load_interpolate_platform(
        cls,
        trajectory: NDArray,
        platform_file: AssetPath,
        platform_id: str = "",
        interpolation_method: Literal["CANONICAL", "ARINC 705"] = "ARINC 705",
        sync_gps_time: bool = False,
    ):
        """Load a platform from an XML file with interpolation enabled."""

        # Validate the XML
        validate_xml_file(platform_file, "xsd/platform.xsd")

        _cpp_platform = _helios.read_platform_from_xml(
            str(platform_file), [str(p) for p in get_asset_directories()], platform_id
        )

        _cpp_interpolated_platform = _helios.load_interpolated_platform(
            _cpp_platform, trajectory, interpolation_method, sync_gps_time
        )
        cppplatform = cls._from_cpp(_cpp_interpolated_platform)
        return cppplatform


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

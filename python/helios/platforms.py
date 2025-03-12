from typing import Annotated, Any, Optional
from helios.utils import get_asset_directories
from helios.validation import (
    AssetPath,
    Model,
    UpdateableMixin,
    validate_xml_file,
)
from pydantic import Field, GetPydanticSchema, StringConstraints, validate_call
import numpy as np

import _helios


class Printable:
    def __str__(self):
        from pprint import pformat

        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)


HandleAsAny = GetPydanticSchema(lambda _s, h: h(Any))
traj_dtype = np.dtype(
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


class TrajectoryParserSettings(Printable, Model):
    trajectory: Optional[AssetPath] = None
    tIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 0
    xIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 1
    yIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 2
    zIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 3
    rollIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 4
    pitchIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 5
    yawIndex: Annotated[int, Field(strict=True, ge=0, le=6)] = 6
    trajectory_separator: Annotated[
        str, Field(strict=True, min_length=1, max_length=1)
    ] = ","
    slopeFilterThreshold: float = 0.0
    syncGPStime: bool = False
    interpolationDomain: Annotated[
        str, StringConstraints(pattern=r"(^$)|(position)|(position_and_attitude)")
    ] = ""


class TrajectorySettings(PlatformSettingsBase, cpp_class=_helios.TrajectorySettings):
    start_time: float = 0
    end_time: float = 0
    teleport_to_start: bool = False
    trajectory_parser_settings: Optional[TrajectoryParserSettings] = None


class PlatformSettings(PlatformSettingsBase):
    x: float = 0
    y: float = 0
    z: float = 0
    is_on_ground = False


class StaticPlatformSettings(PlatformSettings):
    pass


class DynamicPlatformSettings(PlatformSettings):
    trajectory_settings: TrajectorySettings = TrajectorySettings()
    speed_m_s: Annotated[float, Field(ge=0)] = 70


class Platform(Printable, Model, cpp_class=_helios.Platform):
    # TODO: should these get set when calling `from_xml`?
    platform_settings: Optional[PlatformSettings] = None
    trajectory: Optional[Annotated[np.ndarray, HandleAsAny]] = None

    @classmethod
    @validate_call
    def from_xml(cls, platform_file: AssetPath, platform_id: str = ""):

        # Validate the XML
        validate_xml_file(platform_file, "xsd/platform.xsd")

        _cpp_platform = _helios.read_platform_from_xml(
            str(platform_file), [str(p) for p in get_asset_directories()], platform_id
        )
        cppplatform = cls._from_cpp(_cpp_platform)
        return cppplatform

    # TODO: load traj from csv
    def load_traj_csv(self, csv: AssetPath):
        if not isinstance(self.platform_settings, DynamicPlatformSettings):
            raise TypeError(
                "Error: Trying to load a trajectory into a non-dynamic "
                f"Platform. Platform is of type {type(self.platform_settings)}."
            )
        if not self.platform_settings.trajectory_settings.trajectory_parser_settings:
            raise ValueError(
                "Error: Trajectory settings miss trajectory_parser_settings!"
            )

        indices = [
            "tIndex",
            "xIndex",
            "yIndex",
            "zIndex",
            "rollIndex",
            "pitchIndex",
            "yawIndex",
        ]
        tps = self.platform_settings.trajectory_settings.trajectory_parser_settings
        traj = np.loadtxt(csv, dtype=traj_dtype, delimiter=tps.trajectory_separator)
        default_tps = TrajectoryParserSettings()
        if not all([getattr(default_tps, i) == getattr(tps, i) for i in indices]):
            # reorder columns
            idxs = [getattr(tps, i) for i in indices]
            traj = traj[[traj.dtype.names[i] for i in idxs]]
        self.trajectory = traj
        return self


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

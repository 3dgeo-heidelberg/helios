from helios.scene import StaticScene
from helios.utils import (
    classonlymethod,
    get_asset_directories,
    _validate_trajectory_array,
)
from helios.validation import (
    AssetPath,
    Model,
    UpdateableMixin,
    validate_xml_file,
    get_all_annotations,
)
from pydantic import Field, validate_call
from typing import Annotated, Any, Callable, Literal, Optional, Type
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


def _specify_platform_settings_type(parameters: dict) -> None:
    """Specify the most suitable PlatformSettings subclass based on the given parameters."""
    candidates: list[Type[PlatformSettings]] = [
        StaticPlatformSettings,
        DynamicPlatformSettings,
        PlatformSettings,
    ]

    param_keys = set(parameters.keys())
    best_cls = PlatformSettings
    best_score = -1
    for cls in candidates:
        ann_keys = set(get_all_annotations(cls).keys())
        score = len(ann_keys & param_keys)
        if score > best_score:
            best_score = score
            best_cls = cls

    return best_cls()


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

    def do_force_on_ground(self, scene: StaticScene):
        """
        Move waypoint z coordinate to ground level
        """

        ground_point = scene._cpp_object.ground_point_at((self.x, self.y, self.z))
        self.z = ground_point[2]


class StaticPlatformSettings(PlatformSettingsBase):
    x: float = 0
    y: float = 0
    z: float = 0
    force_on_ground: bool = False


class DynamicPlatformSettings(PlatformSettings):
    trajectory_settings: TrajectorySettings = TrajectorySettings()
    speed_m_s: Annotated[float, Field(ge=0)] = 70


class Platform(Printable, Model, cpp_class=_helios.Platform):
    # TODO: should platform_settings get set from xml as well?
    platform_settings: Optional[PlatformSettings] = None

    @classonlymethod
    @validate_call
    def from_xml(cls, platform_file: AssetPath, platform_id: str = ""):
        """Classmethod to load a platform from an XML file. The XML file should conform to the schema defined in "xsd/platform.xsd". The platform_id parameter can be used to specify which platform to load if the XML file contains multiple platforms."""

        # Validate the XML
        validate_xml_file(platform_file, "xsd/platform.xsd")

        _cpp_platform = _helios.read_platform_from_xml(
            str(platform_file), [str(p) for p in get_asset_directories()], platform_id
        )
        platform = cls._from_cpp(_cpp_platform)
        platform._is_loaded_from_xml = True
        return platform

    @classonlymethod
    @validate_call
    def load_interpolate_platform(
        cls,
        trajectory: NDArray,
        platform_file: AssetPath,
        platform_id: str = "",
        interpolation_method: Literal["CANONICAL", "ARINC 705"] = "ARINC 705",
        sync_gps_time: bool = False,
    ):
        """Load a platform from an XML file with interpolation enabled.
        Args:
            trajectory: 1-D structured NumPy array of shape (n,).
            platform_file: File path to platform XML file.
            platform_id: ID of desired platform.
            interpolation_method: Interpolation method to use. Options are "CANONICAL" and "ARINC 705".
            sync_gps_time: Whether to sync GPS time with platform's start time.
        """

        # Validate the XML
        validate_xml_file(platform_file, "xsd/platform.xsd")

        _validate_trajectory_array(trajectory)

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

PLATFORM_REGISTRY: dict[str, tuple[str, str]] = {
    "sr22": ("data/platforms.xml", "sr22"),
    "quadcopter": ("data/platforms.xml", "quadcopter"),
    "copter_linearpath": ("data/platforms.xml", "copter_linearpath"),
    "tractor": ("data/platforms.xml", "tractor"),
    "tractor_leftside": ("data/platforms.xml", "tractor_leftside"),
    "vehicle_linearpath": ("data/platforms.xml", "vehicle_linearpath"),
    "vmx_450_car_left": ("data/platforms.xml", "vmx-450-car-left"),
    "vmx_450_car_right": ("data/platforms.xml", "vmx-450-car-right"),
    "vmq_1ha_car": ("data/platforms.xml", "vmq-1ha-car-0"),
    "simple_linearpath": ("data/platforms.xml", "simple_linearpath"),
    "tripod": ("data/platforms.xml", "tripod"),
}


def list_platforms() -> list[str]:
    """List all predefined platform names."""
    return list(PLATFORM_REGISTRY.keys())


@validate_call
def platform_from_name(platform_name: str) -> Platform:
    """Create a predefined platform by its string name."""
    try:
        platform_file, platform_id = PLATFORM_REGISTRY[platform_name]
    except KeyError as exc:
        valid_names = ", ".join(list_platforms())
        raise ValueError(
            f"Unknown platform '{platform_name}'. Available platforms: {valid_names}"
        ) from exc
    return Platform.from_xml(platform_file, platform_id=platform_id)


def _make_predefined_platform(platform_name: str) -> Callable[[], Platform]:
    def _platform_factory():
        return platform_from_name(platform_name)

    _platform_factory.__name__ = platform_name
    _platform_factory.__qualname__ = platform_name
    _platform_factory.__doc__ = f"Create predefined platform '{platform_name}'."
    return _platform_factory


for _platform_name in PLATFORM_REGISTRY:
    globals()[_platform_name] = _make_predefined_platform(_platform_name)

del _platform_name

from helios.scene import StaticScene
from helios.utils import (
    classonlymethod,
    get_asset_directories,
    _prepare_trajectory_array,
)
from helios.validation import (
    AssetPath,
    Model,
    UpdateableMixin,
    get_all_annotations,
    validate_xml_file,
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

    :param csv: File path to csv to load.
    :param tIndex: Column number of time field
    :param xIndex: Column number of x coordinates
    :param yIndex: Column number of y coordinates
    :param zIndex: Column number of z coordinates
    :param rollIndex: Column number of roll
    :param pitchIndex: Column number of pitch
    :param yawIndex: Column number of yaw
    :param trajectory_separator: Char which separates columns.
    :param rpy_in_radians: Whether roll, pitch, and yaw in the csv are in radians. If false, they are assumed to be in degrees and will be converted to radians.
    :type csv: AssetPath
    :type tIndex: int
    :type xIndex: int
    :type yIndex: int
    :type zIndex: int
    :type rollIndex: int
    :type pitchIndex: int
    :type yawIndex: int
    :type trajectory_separator: str
    :type rpy_in_radians: bool
    
    :return: A structured numpy array with fields 't', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'. 'roll', 'pitch', and 'yaw' are in radians.
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
    """Class representing the settings for a trajectory. These can be set for each leg of a survey and are applied to the trajectory loaded in the `Platform` instance
    
    :param start_time: The time in seconds at which the trajectory starts. This is used to select a subset of the trajectory to be used for the leg by GPS time.
    :param end_time: The time in seconds at which the trajectory ends. This is used to select a subset of the trajectory to be used for the leg by GPS time.
    :param teleport_to_start: Whether to teleport to the start of the trajectory at the beginning of the leg. If false, the platform will start at the position of the end of the previous leg. If true, the platform will be teleported to the start of the trajectory at the beginning of the leg.
    """
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

        :param scene: The scene to query for the ground level.
        :type scene: StaticScene
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
        platform._disable_yaml_serialization_for_descendants()
        platform._set_constructor_provenance(
            "from_xml", platform_file=platform_file, platform_id=platform_id
        )
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
        is_roll_pitch_yaw_in_radians: bool = True,
    ):
        """Load a platform from an XML file with interpolation enabled.

        Args:
            trajectory: 1-D structured NumPy array of shape (n,).
            platform_file: File path to platform XML file.
            platform_id: ID of desired platform.
            interpolation_method: Interpolation method to use. Options are "CANONICAL" and "ARINC 705".
            sync_gps_time: Whether to sync GPS time with platform's start time.
            is_roll_pitch_yaw_in_radians: Whether roll, pitch, and yaw in the trajectory are in radians.
        """

        # Validate the XML
        validate_xml_file(platform_file, "xsd/platform.xsd")

        trajectory = _prepare_trajectory_array(trajectory)

        _cpp_platform = _helios.read_platform_from_xml(
            str(platform_file), [str(p) for p in get_asset_directories()], platform_id
        )

        _cpp_interpolated_platform = _helios.load_interpolated_platform(
            _cpp_platform,
            trajectory,
            interpolation_method,
            sync_gps_time,
            is_roll_pitch_yaw_in_radians,
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
    """List all predefined platform names.
    
    :return: A list of all predefined platform names that can be used to create platforms with `platform_from_name()`.
    :rtype: list[str]
    """
    return list(PLATFORM_REGISTRY.keys())


@validate_call
def platform_from_name(platform_name: str) -> Platform:
    """Create a predefined platform by its string name.
    
    :param platform_name: The name of the predefined platform to create. Use `list_platforms()` to see all available platforms.
    :type platform_name: str

    :return: The created platform instance.
    :rtype: Platform
    """
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

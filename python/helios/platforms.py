from helios.scene import DynamicScene, StaticScene
from helios.rotation import Rotation
from helios.utils import (
    classonlymethod,
    get_asset_directories,
    _prepare_trajectory_array,
)
from helios.validation import (
    AssetPath,
    Model,
    R3Vector,
    Speed,
    UpdateableMixin,
    get_all_annotations,
    units,
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

    def do_force_on_ground(self, scene: StaticScene | DynamicScene):
        """
        Move waypoint z coordinate to ground level

        :param scene: The scene to query for the ground level.
        :type scene: StaticScene | DynamicScene
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
    speed_m_s: Speed = 70 * units.m / units.s


class Platform(Printable, Model, cpp_class=_helios.Platform):
    # TODO: should platform_settings get set from xml as well?
    platform_settings: Optional[PlatformSettings] = None
    scanner_mount_position: R3Vector = (0.0, 0.0, 0.0)
    scanner_mount_attitude: Rotation = Rotation()

    def _never_serialize_shallow(self) -> bool:
        return True

    @classmethod
    def _from_cpp(cls, value):
        if cls is Platform:
            if isinstance(value, _helios.GroundVehiclePlatform):
                return GroundVehiclePlatform._from_cpp(value)
            if isinstance(value, _helios.LinearPathPlatform):
                return LinearPathPlatform._from_cpp(value)
            if type(value) is _helios.Platform:
                return StaticPlatform._from_cpp(value)
        return Model._from_cpp.__func__(cls, value)

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
            "from_xml",
            "platform_file",
            platform_file=platform_file,
            platform_id=platform_id,
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


class StaticPlatform(Platform, cpp_class=_helios.Platform):
    pass


class LinearPathPlatform(Platform, cpp_class=_helios.LinearPathPlatform):
    pass


class GroundVehiclePlatform(Platform, cpp_class=_helios.GroundVehiclePlatform):
    drag: float = 1.0


#
# Predefined platforms
#

PlatformFactory = Callable[[], Platform]
PLATFORM_REGISTRY: dict[str, PlatformFactory] = {}


def registered_platform(name: str):
    """Register a predefined platform factory under ``name``."""

    def decorator(factory: PlatformFactory) -> PlatformFactory:
        if name in PLATFORM_REGISTRY:
            raise ValueError(f"Platform '{name}' is already registered")
        PLATFORM_REGISTRY[name] = factory
        return factory

    return decorator


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
        factory = PLATFORM_REGISTRY[platform_name]
    except KeyError as exc:
        valid_names = ", ".join(list_platforms())
        raise ValueError(
            f"Unknown platform '{platform_name}'. Available platforms: {valid_names}"
        ) from exc
    return factory()


@registered_platform("sr22")
def sr22() -> Platform:
    return LinearPathPlatform(
        scanner_mount_position=np.array((0, 0, 0.7)) * units.m,
        scanner_mount_attitude=Rotation.from_axis_angles(
            rotations=(("x", -np.pi / 2 * units.rad), ("z", np.pi / 2 * units.rad))
        ),
    )


@registered_platform("copter_linearpath")
def copter_linearpath() -> Platform:
    return LinearPathPlatform(
        scanner_mount_position=np.array((0, 0, 0.2)) * units.m,
        scanner_mount_attitude=Rotation.from_axis_angles(
            rotations=(("x", 175 * units.deg),)
        ),
    )


@registered_platform("tractor")
def tractor() -> Platform:
    return GroundVehiclePlatform(
        drag=0.005,
        scanner_mount_position=np.array((0, 1, 4)) * units.m,
        scanner_mount_attitude=Rotation.from_axis_angles(
            rotations=(("z", np.pi / 2 * units.rad), ("y", -np.pi / 6 * units.rad)),
            frame="local",
        ),
    )


@registered_platform("tractor_leftside")
def tractor_leftside() -> Platform:
    return GroundVehiclePlatform(
        drag=0.01,
        scanner_mount_position=np.array((0, 0.8, 4)) * units.m,
        scanner_mount_attitude=Rotation.from_axis_angles(
            rotations=(("y", -35 * units.deg),)
        ),
    )


@registered_platform("vehicle_linearpath")
def vehicle_linearpath() -> Platform:
    return LinearPathPlatform(
        scanner_mount_position=np.array((0, 0, 2.4)) * units.m,
        scanner_mount_attitude=Rotation.from_axis_angles(
            rotations=(("z", np.pi / 2 * units.rad),), frame="local"
        ),
    )


@registered_platform("vmx_450_car_left")
def vmx_450_car_left() -> Platform:
    return LinearPathPlatform(
        scanner_mount_position=np.array((-0.18, -2.2, 2.2)) * units.m,
        scanner_mount_attitude=Rotation.from_axis_angles(
            rotations=(("x", 40 * units.deg), ("z", 150 * units.deg))
        ),
    )


@registered_platform("vmx_450_car_right")
def vmx_450_car_right() -> Platform:
    return LinearPathPlatform(
        scanner_mount_position=np.array((0.18, -2.2, 2.2)) * units.m,
        scanner_mount_attitude=Rotation.from_axis_angles(
            rotations=(("x", 40 * units.deg), ("z", 210 * units.deg))
        ),
    )


@registered_platform("vmq_1ha_car")
def vmq_1ha_car() -> Platform:
    return LinearPathPlatform(
        scanner_mount_position=np.array((0, -1.15, 2.2)) * units.m,
        scanner_mount_attitude=Rotation.from_axis_angles(
            rotations=(("x", np.pi / 4 * units.rad), ("z", 0 * units.rad))
        ),
    )


@registered_platform("simple_linearpath")
def simple_linearpath() -> Platform:
    return LinearPathPlatform()


@registered_platform("tripod")
def tripod() -> Platform:
    return StaticPlatform(scanner_mount_position=np.array((0, 0, 1.5)) * units.m)

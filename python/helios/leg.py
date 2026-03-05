from helios.platforms import PlatformSettings, PlatformSettingsBase, TrajectorySettings
from helios.scanner import ScannerSettings, ScannerSettingsBase
from helios.validation import Model

from typing import Optional

import _helios


class Leg(Model, cpp_class=_helios.Leg):

    """
    Class representing a leg of a survey. A leg is a combination of platform settings (defining the platform position and, if moving, the speed),
    scanner settings and trajectory settings.
    It represents a single segment of a survey where the platform and scanner settings are constant.
    In the waypoint mode, the leg will be defined by a single or by a start and end waypoint.
    For interpolated trajectories, the leg will be defined by the trajectory.

    :param platform_settings: The settings for the platform for this leg. See `PlatformSettings` for an overview of the available options.
    :param scanner_settings: The settings for the scanner for this leg. See `ScannerSettings` for an overview of the available options.
    :param trajectory_settings: The settings for the trajectory for this leg. See `TrajectorySettings` for an overview of the available options.
    """

    scanner_settings: ScannerSettingsBase
    platform_settings: PlatformSettingsBase
    trajectory_settings: Optional[TrajectorySettings]

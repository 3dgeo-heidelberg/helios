from helios.platforms import PlatformSettings, PlatformSettings, TrajectorySettings
from helios.scanner import ScannerSettings, ScannerSettingsBase
from helios.validation import Model

from typing import Optional

import _helios


class Leg(Model, cpp_class=_helios.Leg):
    scanner_settings: ScannerSettingsBase
    platform_settings: PlatformSettings
    trajectory_settings: Optional[TrajectorySettings]

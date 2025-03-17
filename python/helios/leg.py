from helios.platforms import PlatformSettingsBase, PlatformSettings
from helios.scanner import ScannerSettings, ScannerSettingsBase
from helios.validation import Model

import _helios


class Leg(Model, cpp_class=_helios.Leg):
    platform_settings: PlatformSettingsBase = PlatformSettings()
    scanner_settings: ScannerSettingsBase = ScannerSettings()

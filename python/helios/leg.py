from helios.platform import PlatformSettingsBase, PlatformSettings
from helios.scanner import ScannerSettings, ScannerSettingsBase
from helios.validation import Model, Property

import _helios


class Leg(Model, cpp_class=_helios.Leg):
    platform_settings: PlatformSettingsBase = Property(
        "platform_settings", wraptype=PlatformSettings, default=PlatformSettings()
    )
    scanner_settings: ScannerSettingsBase = Property(
        "scanner_settings", wraptype=ScannerSettings, default=ScannerSettings()
    )

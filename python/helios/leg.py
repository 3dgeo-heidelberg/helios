from helios.platform import PlatformSettingsBase, PlatformSettings
from helios.scanner import ScannerSettings, ScannerSettingsBase
from helios.validation import Model, Property

import _helios


class Leg(Model, cpp_class=_helios.Leg):
    platform_settings: PlatformSettingsBase = Property(
        cpp="platform_settings", wraptype=PlatformSettings, default=PlatformSettings()
    )
    scanner_settings: ScannerSettingsBase = Property(
        cpp="scanner_settings", wraptype=ScannerSettings, default=ScannerSettings()
    )

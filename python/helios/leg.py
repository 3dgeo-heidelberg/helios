from helios.platform import PlatformSettingsBase, PlatformSettings
from helios.scanner import ScannerSettings, ScannerSettingsBase
from helios.validation import ValidatedCppModel, ValidatedCppManagedProperty

import _helios


class Leg(ValidatedCppModel, cpp_class=_helios.Leg):
    platform_settings: PlatformSettingsBase = ValidatedCppManagedProperty(
        "platform_settings", wraptype=PlatformSettings, default=PlatformSettings()
    )
    scanner_settings: ScannerSettingsBase = ValidatedCppManagedProperty(
        "scanner_settings", wraptype=ScannerSettings, default=ScannerSettings()
    )

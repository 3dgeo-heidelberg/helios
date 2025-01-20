from helios.platform import PlatformSettingsBase, PlatformSettings
from helios.scanner import ScannerSettings, ScannerSettingsBase
from helios.validation import Validatable, ValidatedCppManagedProperty

import _helios


class Leg(Validatable):
    def __init__(
        self,
        platform_settings: PlatformSettings = PlatformSettings(),
        scanner_settings: ScannerSettings = ScannerSettings(),
        **parameters,
    ):
        """Construct a leg object programmatically."""

        # Instantiate the underlying C++ object
        self._cpp_object = _helios.Leg()

        # Set the platform and scanner settings
        self.platform_settings = platform_settings
        self.scanner_settings = scanner_settings

        # Update platform and scanner settings with any additional parameters
        self.scanner_settings.update_from_dict(parameters, skip_exceptions=True)
        self.platform_settings.update_from_dict(parameters, skip_exceptions=True)
        if parameters:
            raise ValueError(
                f"Unknown parameters provided: {', '.join(parameters.keys())}"
            )

    platform_settings: PlatformSettingsBase = ValidatedCppManagedProperty(
        "platform_settings", PlatformSettings
    )
    scanner_settings: ScannerSettingsBase = ValidatedCppManagedProperty(
        "scanner_settings", ScannerSettings
    )

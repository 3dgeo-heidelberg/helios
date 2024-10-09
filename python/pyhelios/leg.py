from pyhelios.utils import Validatable, ValidatedCppManagedProperty
from pyhelios.platforms import PlatformSettings
from pyhelios.scanner import ScannerSettings
from typing import Optional, List, Tuple, Annotated, Type, Any

import _helios



class ScanningStrip(Validatable):
    def __init__(self, strip_id: Optional[str] = "", legs: Optional[List['Leg']] = None) -> None:
        
        self._cpp_object = _helios.ScanningStrip(strip_id)
        self.strip_id = strip_id


    strip_id: Optional[str] = ValidatedCppManagedProperty("strip_id")

class Leg(Validatable):
    def __init__(self, platform_settings: Optional[PlatformSettings] = None, scanner_settings: Optional[ScannerSettings] = None, 
                 scanning_strip: Optional[ScanningStrip] = None, length: Optional[float] = 0.0, serial_id: Optional[int] = 0, belongs_to_strip: Optional[bool] = False) -> None:

        self._cpp_object = _helios.Leg()
        self.platform_settings = platform_settings
        self.scanner_settings = scanner_settings
        self.strip = scanning_strip

        
        self.length = length
        self.serial_id = serial_id
 

    platform_settings: Optional[PlatformSettings] = ValidatedCppManagedProperty("platform_settings")
    scanner_settings: Optional[ScannerSettings] = ValidatedCppManagedProperty("scanner_settings")
    scanning_strip: Optional[ScanningStrip] = ValidatedCppManagedProperty("scanning_strip")
    length: Optional[float] = ValidatedCppManagedProperty("length")
    serial_id: Optional[int] = ValidatedCppManagedProperty("serial_id")


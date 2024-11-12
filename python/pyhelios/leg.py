from pyhelios.utils import Validatable, ValidatedCppManagedProperty
from pyhelios.platforms import PlatformSettings
from pyhelios.scanner import ScannerSettings
from pyhelios.primitives import TrajectorySettings
from typing import Optional, Type, Dict, Any

import xml.etree.ElementTree as ET
import _helios

class ScanningStrip(Validatable):
    def __init__(self, strip_id: Optional[str] = "", legs: Optional[dict[int, 'Leg']] = None) -> None:
        
        self._cpp_object = _helios.ScanningStrip(strip_id)
        self.strip_id = strip_id
        self.legs = legs or {}

    strip_id: Optional[str] = ValidatedCppManagedProperty("strip_id")


class Leg(Validatable):
    def __init__(self, platform_settings: Optional[PlatformSettings] = None, scanner_settings: Optional[ScannerSettings] = None, 
                 strip: Optional[ScanningStrip] = None, trajectory_settings: Optional[TrajectorySettings] = None,
                 length: Optional[float] = 0.0, serial_id: Optional[int] = 0) -> None:
        self._cpp_object = _helios.Leg()
        self.platform_settings = platform_settings or PlatformSettings()
        self.scanner_settings = scanner_settings or ScannerSettings()
        self.strip = strip or ScanningStrip()
        self.trajectory_settings = trajectory_settings
        self.length = length
        self.serial_id = serial_id
 
    platform_settings: Optional[PlatformSettings] = ValidatedCppManagedProperty("platform_settings")
    scanner_settings: Optional[ScannerSettings] = ValidatedCppManagedProperty("scanner_settings")
    strip: Optional[ScanningStrip] = ValidatedCppManagedProperty("strip")
    trajectory_settings: Optional[TrajectorySettings] = ValidatedCppManagedProperty("trajectory_settings")
    length: Optional[float] = ValidatedCppManagedProperty("length")
    serial_id: Optional[int] = ValidatedCppManagedProperty("serial_id")
    
    @classmethod
    def _set_settings(cls, settings_node: ET.Element, settings_templates: Optional[Dict[str, Any]], settings_class: Type[Any]) -> Any:
        template_id = settings_node.get('template')
        if template_id and template_id in settings_templates: # TODO add logic for case when we face unknown template
            
            return settings_class.from_xml_node(settings_node, settings_templates[template_id])
        else:
            return settings_class.from_xml_node(settings_node)

    @classmethod
    def from_xml(cls, leg_origin_node: ET.Element, id: int, strips: Optional[Dict[str, ScanningStrip]] = None, 
                 platform_settings_templates: Optional[Dict[str, PlatformSettings]] = None, scanner_settings_templates: Optional[Dict[str, ScannerSettings]] = None) -> 'Leg':
        leg = cls()
        #TODO: add "Obtain trajectory interpolator"
        leg.serial_id = id
        strip_id = leg_origin_node.get('stripId')
        
        if strip_id:
            strip = strips.get(strip_id, ScanningStrip(strip_id))
            strips[strip_id] = strip
            leg.strip = strip
            strip.legs[id] = leg
        
        platform_settings_node = leg_origin_node.find("platformSettings")
        if platform_settings_node is not None:
            leg.platform_settings = cls._set_settings(platform_settings_node, platform_settings_templates, PlatformSettings)
        
        scanner_settings_node = leg_origin_node.find("scannerSettings")
        if scanner_settings_node is not None:
            leg.scanner_settings = cls._set_settings(scanner_settings_node, scanner_settings_templates, ScannerSettings)

        trajectory_settings_node = leg_origin_node.find("TrajectorySettings")
        if trajectory_settings_node is not None:
            leg.trajectory_settings = TrajectorySettings.from_xml_node(trajectory_settings_node)

        return cls._validate(leg)
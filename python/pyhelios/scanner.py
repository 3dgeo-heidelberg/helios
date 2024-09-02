from pyhelios.utils import Validatable, ValidatedCppManagedProperty
from pyhelios.primitives import FWFSettings, AbstractBeamDeflector, FWFSettings
from typing import Optional, List, Tuple, Annotated, Type, Any

import _helios

class ScannerSettings(Validatable):
    def __init__(self, id: Optional[str] = "", is_active: Optional[bool] = True, head_rotation: Optional[float] = .0, 
                 rotation_start_angle: Optional[float] = .0, rotation_stop_angle: Optional[float] = .0, pulse_frequency: Optional[int] = 0,
                 scan_angle: Optional[float] = .0, min_vertical_angle: Optional[float] = .0, max_vertical_angle: Optional[float] = .0,
                 scan_frequency: Optional[float] = .0, beam_divergence_angle: Optional[float] = .003, trajectory_time_interval: Optional[float] = .0,
                 vertical_resolution: Optional[float] = .0, horizontal_resolution: Optional[float] = .0) -> None:

        self._cpp_object = _helios.ScannerSettings()
        self.id = id
        self.is_active = is_active
        self.head_rotation = head_rotation
        self.rotation_start_angle = rotation_start_angle
        self.rotation_stop_angle = rotation_stop_angle
        self.pulse_frequency = pulse_frequency
        self.scan_angle = scan_angle
        self.min_vertical_angle = min_vertical_angle
        self.max_vertical_angle = max_vertical_angle
        self.scan_frequency = scan_frequency
        self.beam_divergence_angle = beam_divergence_angle
        self.trajectory_time_interval = trajectory_time_interval
        self.vertical_resolution = vertical_resolution
        self.horizontal_resolution = horizontal_resolution
    
    id: Optional[str] = ValidatedCppManagedProperty("id")
    is_active: Optional[bool] = ValidatedCppManagedProperty("is_active")
    head_rotation: Optional[float] = ValidatedCppManagedProperty("head_rotation")
    rotation_start_angle: Optional[float] = ValidatedCppManagedProperty("rotation_start_angle")
    rotation_stop_angle: Optional[float] = ValidatedCppManagedProperty("rotation_stop_angle")
    pulse_frequency: Optional[int] = ValidatedCppManagedProperty("pulse_frequency")
    scan_angle: Optional[float] = ValidatedCppManagedProperty("scan_angle")
    min_vertical_angle: Optional[float] = ValidatedCppManagedProperty("min_vertical_angle")
    max_vertical_angle: Optional[float] = ValidatedCppManagedProperty("max_vertical_angle")
    scan_frequency: Optional[float] = ValidatedCppManagedProperty("scan_frequency")
    beam_divergence_angle: Optional[float] = ValidatedCppManagedProperty("beam_divergence_angle")
    trajectory_time_interval: Optional[float] = ValidatedCppManagedProperty("trajectory_time_interval")
    vertical_resolution: Optional[float] = ValidatedCppManagedProperty("vertical_resolution")
    horizontal_resolution: Optional[float] = ValidatedCppManagedProperty("horizontal_resolution")

class ScannerHead(Validatable):
    def __init__(self, rotate_per_sec_max: Optional[float], rotation_axis: Optional[List[float]] = [1., 0., 0.], rotate_per_sec: Optional[float] = .0, rotate_stop: Optional[float] = .0, rotate_start: Optional[float] = .0,
                 rotate_range: Optional[float] = .0, current_rotate_angle: Optional[float] = .0) -> None:
        self._cpp_object = _helios.ScannerHead(rotation_axis, rotate_per_sec_max)
        self.rotate_per_sec_max = rotate_per_sec_max
        self.rotate_per_sec = rotate_per_sec
        self.rotate_stop = rotate_stop
        self.rotate_start = rotate_start
        self.rotate_range = rotate_range
        self.current_rotate_angle = current_rotate_angle

    rotate_per_sec_max: Optional[float] = ValidatedCppManagedProperty("rotate_per_sec_max")
    rotate_per_sec: Optional[float] = ValidatedCppManagedProperty("rotate_per_sec")
    rotate_stop: Optional[float] = ValidatedCppManagedProperty("rotate_stop")
    rotate_start: Optional[float] = ValidatedCppManagedProperty("rotate_start")
    rotate_range: Optional[float] = ValidatedCppManagedProperty("rotate_range")
    current_rotate_angle: Optional[float] = ValidatedCppManagedProperty("current_rotate_angle")


class Scanner(Validatable):
    def __init__(self, id: Optional[str],  scanner_settings: Optional[ScannerSettings] = None, FWF_settings: Optional[FWFSettings] = None, scanner_head: Optional[ScannerHead] = None,
                  beam_deflector: Optional[AbstractBeamDeflector] = None, detector: Optional['AbstractDetector'] = None, supported_pulse_freqs_hz: Optional[List[int]] = [0],
                  num_rays: Optional [int] = 0, pulse_length: Optional[float] = .0) -> None:
        self._cpp_object = _helios.Scanner(id, supported_pulse_freqs_hz)
        self.fwf_settings = FWF_settings
        self.num_rays = num_rays
        self.pulse_length = pulse_length

    FWF_settings: Optional[FWFSettings] = ValidatedCppManagedProperty("fwf_settings")
    num_rays: Optional[int] = ValidatedCppManagedProperty("num_rays")
    pulse_length: Optional[float] = ValidatedCppManagedProperty("pulse_length")


class AbstractDetector(Validatable):
    def __init__(self, scanner: Optional[Scanner], range_max: Optional[float], accuracy: Optional[float] = .0, range_min: Optional[float] = .0) -> None:
        self._cpp_object = _helios.AbstractDetector(scanner._cpp_object, accuracy, range_min, range_max)
        self.accuracy = accuracy
        self.range_min = range_min
        self.range_max = range_max
    
    
    accuracy: Optional[float] = ValidatedCppManagedProperty("accuracy")
    range_min: Optional[float] = ValidatedCppManagedProperty("range_min")
    range_max: Optional[float] = ValidatedCppManagedProperty("range_max")

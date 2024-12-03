from pyhelios.utils import Validatable, ValidatedCppManagedProperty, AssetManager, calc_propagation_time_legacy, create_property
from pyhelios.primitives import FWFSettings, AbstractBeamDeflector, FWFSettings, Rotation, RisleyBeamDeflector, PolygonMirrorBeamDeflector, FiberArrayBeamDeflector, ConicBeamDeflector, OscillatingMirrorBeamDeflector, EnergyModel, Measurement, Trajectory
from pyhelios.platforms import Platform
import sys
import threading
import math
import numpy as np
from typing import Optional, List, Tuple, Annotated, Type, Any
from types import SimpleNamespace
from pydantic import Field, field_validator, model_validator, ConfigDict
import xml.etree.ElementTree as ET
import _helios

class ScannerSettings(Validatable):
    def __init__(self, id: Optional[str] = "DEFAULT_TEMPLATE1_HELIOSCPP", is_active: Optional[bool] = True, head_rotation: Optional[float] = .0, 
                 rotation_start_angle: Optional[float] = .0, rotation_stop_angle: Optional[float] = .0, pulse_frequency: Optional[int] = 0,
                 scan_angle: Optional[float] = .0, min_vertical_angle: Optional[float] = -1, max_vertical_angle: Optional[float] = -1,
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

    @classmethod
    def from_xml_node(cls, node: ET.Element, template: Optional['ScannerSettings'] = None) -> 'ScannerSettings':
    
        settings = cls.copy_template(template) if template else cls()
        
        settings.id = node.get('id', settings.id)
        settings.is_active = node.get('active', str(settings.is_active)).lower() == 'true'
        settings.head_rotation = float(node.get('headRotate_deg', settings.head_rotation))
        settings.rotation_start_angle = float(node.get('headRotateStart_deg', settings.rotation_start_angle))
        settings.rotation_stop_angle = float(node.get('headRotateStop_deg', settings.rotation_stop_angle))
        settings.pulse_frequency = int(node.get('pulseFreq_hz', settings.pulse_frequency))
        settings.scan_angle = float(node.get('scanAngle_deg', settings.scan_angle))
        settings.min_vertical_angle = float(node.get('verticalAngleMin_deg', settings.min_vertical_angle))
        settings.max_vertical_angle = float(node.get('verticalAngleMax_deg', settings.max_vertical_angle))
        settings.scan_frequency = float(node.get('scanFreq_hz', settings.scan_frequency))
        settings.beam_divergence_angle = float(node.get('beamDivergence_rad', settings.beam_divergence_angle))
        settings.vertical_resolution = float(node.get('verticalResolution_deg', settings.vertical_resolution))
        settings.horizontal_resolution = float(node.get('horizontalResolution_deg', settings.horizontal_resolution))
        settings.trajectory_time_interval = float(node.get('trajectoryTimeInterval_s', settings.trajectory_time_interval))

        if settings.rotation_stop_angle < settings.rotation_start_angle and settings.head_rotation > 0:
            raise ValueError("Rotation stop angle must be greater than rotation start angle if head rotation is positive")
        
        if settings.rotation_start_angle < settings.rotation_stop_angle and settings.head_rotation < 0:
            raise ValueError("Rotation start angle must be greater than rotation stop angle if head rotation is negative")

        return cls._validate(settings)

    @classmethod
    def copy_template(cls, template: 'ScannerSettings') -> 'ScannerSettings':
        """Create a copy of the template to be used"""
   
        return ScannerSettings(
            id=template.id,
            is_active=template.is_active,
            head_rotation=template.head_rotation,
            rotation_start_angle=template.rotation_start_angle,
            rotation_stop_angle=template.rotation_stop_angle,
            pulse_frequency=template.pulse_frequency,
            scan_angle=template.scan_angle,
            min_vertical_angle=template.min_vertical_angle,
            max_vertical_angle=template.max_vertical_angle,
            scan_frequency=template.scan_frequency,
            beam_divergence_angle=template.beam_divergence_angle,
            trajectory_time_interval=template.trajectory_time_interval,
            vertical_resolution=template.vertical_resolution,
            horizontal_resolution=template.horizontal_resolution
        )      


class ScannerHead(Validatable):
    def __init__(self, rotate_per_sec_max: Optional[float] = float('inf'), rotation_axis: Optional[List[float]] = None, rotate_per_sec: Optional[float] = .0, rotate_stop: Optional[float] = .0, rotate_start: Optional[float] = .0,
                 rotate_range: Optional[float] = .0, current_rotate_angle: Optional[float] = .0) -> None:
        if rotation_axis is None:
            rotation_axis = [1., 0., 0.]

        self._cpp_object = _helios.ScannerHead(rotation_axis, rotate_per_sec_max)
        self.rotation_axis = rotation_axis
        self.rotate_per_sec_max = rotate_per_sec_max
        self.rotate_per_sec = rotate_per_sec
        self.rotate_stop = rotate_stop
        self.rotate_start = rotate_start
        self.rotate_range = rotate_range
        self.current_rotate_angle = current_rotate_angle

    rotation_axis: Optional[List[float]] = ValidatedCppManagedProperty("rotation_axis")
    rotate_per_sec_max: Optional[float] = ValidatedCppManagedProperty("rotate_per_sec_max")
    rotate_per_sec: Optional[float] = ValidatedCppManagedProperty("rotate_per_sec")
    rotate_stop: Optional[float] = ValidatedCppManagedProperty("rotate_stop")
    rotate_start: Optional[float] = ValidatedCppManagedProperty("rotate_start")
    rotate_range: Optional[float] = ValidatedCppManagedProperty("rotate_range")
    current_rotate_angle: Optional[float] = ValidatedCppManagedProperty("current_rotate_angle")

    @classmethod
    def from_xml_node(cls, head_node: ET.Element) -> 'ScannerHead':
        if head_node is None:
            raise ValueError("No head node found")

        head_rotate_axis = [0, 0, 1]
        axis_node = head_node.find("headRotateAxis")
     
        if axis_node is not None:
            x = float(axis_node.get('x', 0.0))
            y = float(axis_node.get('y', 0.0))
            z = float(axis_node.get('z', 1.0))
            head_rotate_axis = [x, y, z]
            if math.sqrt(sum([coord**2 for coord in head_rotate_axis])) <= 0.1:
                head_rotate_axis = [0, 0, 1]  
        
        rotate_per_sec_max = float(head_node.get('headRotatePerSecMax_deg', 0.0))
        return cls._validate(cls(rotation_axis=head_rotate_axis, rotate_per_sec_max=rotate_per_sec_max))
    
    def clone(self):
        return ScannerHead(self.rotate_per_sec_max, self.rotation_axis, self.rotate_per_sec, self.rotate_stop, self.rotate_start, self.rotate_range, self.current_rotate_angle)


class EvalScannerHead(ScannerHead):
    def __init__(self, rotation_axis: Optional[List[float]] = None, rotate_per_sec_max: Optional[float] = float('inf')):
        if rotation_axis is None:
            rotation_axis = [1., 0., 0.] 
        super().__init__(rotation_axis, rotate_per_sec_max)
        self._cpp_object = _helios.EvalScannerHead(rotation_axis, rotate_per_sec_max)
        

class ScanningDevice(Validatable):
    def __init__(self, 
                 dev_idx: int,
                 id: str,
                 beam_div_rad: float,
                 emitter_position: List[float],
                 emitter_attitude: Rotation,
                 pulse_freqs: List[int],
                 pulse_length: float,
                 avg_power: float,
                 beam_quality: float,
                 optical_efficiency: float,
                 receiver_diameter: float,
                 atmospheric_visibility: float,
                 wavelength: float,
                 last_pulse_was_hit: Optional[bool] = False,
                 fwf_settings: Optional[FWFSettings] = None
                 ) -> None:
        
        self._cpp_object = _helios.ScanningDevice(
            dev_idx, id, beam_div_rad, emitter_position, emitter_attitude._cpp_object, pulse_freqs, 
            pulse_length, avg_power, beam_quality, optical_efficiency, 
            receiver_diameter, atmospheric_visibility, wavelength
        )

        self.dev_idx = dev_idx
        self.id = id
        self.beam_div_rad = beam_div_rad
        self.emitter_position = emitter_position
        self.emitter_attitude = emitter_attitude
        self.pulse_freqs = pulse_freqs
        self.pulse_length = pulse_length
        self.avg_power = avg_power
        self.beam_quality = beam_quality
        self.optical_efficiency = optical_efficiency
        self.receiver_diameter = receiver_diameter
        self.atmospheric_visibility = atmospheric_visibility
        self.wavelength = wavelength
        self.last_pulse_was_hit = last_pulse_was_hit or False
        self.fwf_settings = fwf_settings or FWFSettings()

        self.max_nor = 0
        self.beam_deflector = AbstractBeamDeflector()
        self.detector = AbstractDetector(Scanner("0"))
        self.scanner_head = ScannerHead()

        self.num_rays: int = 0
        self.num_time_bins: int = -1
        self.timevawe: List[float] = []
        self.peak_intensity_index: int = -1

        self.received_energy_min: float = 0.0001
        self.cached_dr2: float = 0.0
        self.cached_bt2: float = 0.0
        self.cached_subray_rotation: List[Rotation] = [Rotation()]
        self.cached_div_angles: List[float] = []
        self.cached_radius_steps: List[int] = []
        self.energy_model: Optional[EnergyModel] = None
    
    fwf_settings: FWFSettings = ValidatedCppManagedProperty("fwf_settings")
    received_energy_min: float = ValidatedCppManagedProperty("received_energy_min")
    last_pulse_was_hit: bool = ValidatedCppManagedProperty("last_pulse_was_hit")
    cached_dr2: float = ValidatedCppManagedProperty("cached_dr2")
    cached_bt2: float = ValidatedCppManagedProperty("cached_bt2")
    cached_subray_rotation: List[Rotation] = ValidatedCppManagedProperty("cached_subray_rotation")

    def calc_rays_number(self) -> None:
        count = 1 
        for radius_step in range(self.fwf_settings.beam_sample_quality):
            circle_steps = int(2 * math.pi * radius_step)
            count += circle_steps

        self.num_rays = count

    def prepare_simulation(self, legacy_energy_model: Optional[bool] = False) -> None:
        beam_sample_quality = self.fwf_settings.beam_sample_quality
        radius_step = self.beam_div_rad / beam_sample_quality
        axis = np.array([1, 0, 0])
        axis2 = np.array([0, 1, 0])
        norm_axis = np.linalg.norm(np.array(axis))
        norm_axis2 = np.linalg.norm(np.array(axis2))
        
        for i in range(beam_sample_quality):
            div_angle_rad = i * radius_step
            self.cached_div_angles.append(div_angle_rad)
            half_div_angle_rad = -0.5 * div_angle_rad
            coeff_dif_angle = math.sin(half_div_angle_rad) / norm_axis

            r1 = Rotation(math.cos(half_div_angle_rad),
                          coeff_dif_angle * axis[0],
                          coeff_dif_angle * axis[1],
                          coeff_dif_angle * axis[2])

            circle_steps = int(2 * math.pi) * i
            if circle_steps > 0:
                circle_step = 2 * math.pi / circle_steps
                for j in range(circle_steps):
                    div_angle_rad2 = j * circle_step
                    half_div_angle_rad2 = -0.5 * div_angle_rad2
                    coeff_dif_angle2 = math.sin(half_div_angle_rad2) / norm_axis2
                    r2 = Rotation(math.cos(half_div_angle_rad2),
                                    coeff_dif_angle2 * axis2[0],
                                    coeff_dif_angle2 * axis2[1],
                                    coeff_dif_angle2 * axis2[2])
                    
                    r2= r2.apply_to(r1)
                    self.cached_subray_rotation.append(r2)
                    self.cached_radius_steps.append(i)
        
        if legacy_energy_model:
            #TODO call the improved energy model
            pass
        
        else:
            self.energy_model = EnergyModel(self) 
        
    def clone(self):
        return ScanningDevice(self.dev_idx, self.id, self.beam_div_rad, self.emitter_position, self.emitter_attitude, self.pulse_freqs, self.pulse_length, self.avg_power, self.beam_quality, self.optical_efficiency, self.receiver_diameter, self.atmospheric_visibility, self.wavelength, self.last_pulse_was_hit, self.fwf_settings)


class Scanner(Validatable):
    def __init__(self, id: Optional[str], supported_pulse_freqs_hz: Optional[List[int]] = None, platform: Optional[Platform] = None, pulse_freq_hz: Optional[int] = 0) -> None:
        if supported_pulse_freqs_hz is None:
            supported_pulse_freqs_hz = [0]
        self._cpp_object = _helios.Scanner(id, supported_pulse_freqs_hz)
        self.id = id
        self.platform = platform
        self.supported_pulse_freqs_hz = supported_pulse_freqs_hz
        self.trajectory_time_interval: float = 0.0
        self.is_state_active: bool = True
        self.pulse_freq_hz = pulse_freq_hz

        self.all_measurements: List[Measurement] = []
        self.all_trajectories: List[Trajectory] = []
        self.all_output_paths: List[str] = []

        self.write_waveform: Optional[bool] = False
        self.calc_echowidth: Optional[bool] = False
        self.fullwavenoise: Optional[bool] = False
        self.is_platform_noise_disabled: Optional[bool] = False

        self.cycle_measurements: Optional[List[Measurement]] = []
        self.cycle_trajectories: Optional[List[Trajectory]] = []
        self.cycle_measurements_mutex = None
        self.all_measurements_mutex = None 
          
#TODO: print IMPORTANT NOTE: YOU SHOULD CHECK THE LOGIC OF USAGE OF PARENT CLASS INITIALIZATION!!!!!!!!!!
    id: Optional[str] = ValidatedCppManagedProperty("id")
    trajectory_time_interval: float = ValidatedCppManagedProperty("trajectory_time_interval")
    is_state_active: bool = ValidatedCppManagedProperty("is_state_active")
    all_output_paths: List[str] = ValidatedCppManagedProperty("all_output_paths")
    all_measurements: List[Measurement] = ValidatedCppManagedProperty("all_measurements")
    all_trajectories: List[Trajectory] = ValidatedCppManagedProperty("all_trajectories")
    cycle_measurements: Optional[List[Measurement]] = ValidatedCppManagedProperty("cycle_measurements")
    cycle_trajectories: Optional[List[Trajectory]] = ValidatedCppManagedProperty("cycle_trajectories")
    platform: Optional[Platform] = ValidatedCppManagedProperty("platform")

    @classmethod
    def from_xml(cls, filename: str, id: Optional[str] = None) -> 'Scanner':
        file_path = AssetManager().find_file_by_name(filename, auto_add=True)
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        scanner_element = root.find(f".//scanner[@id='{id}']")
        if scanner_element is None:
            raise ValueError(f"No scanner found with id: {id}")
        
        emitter_position, emitter_attitude = cls._parse_emitter(scanner_element)
        pulse_freqs = cls._parse_pulse_frequencies(scanner_element)

        setting_characteristics = cls._combine_scanner_characteristics(scanner_element)
        multi_scanner_element = scanner_element.find('channels')
        if multi_scanner_element is None:
            scanner = cls._create_single_scanner(scanner_element, pulse_freqs, setting_characteristics, emitter_position, emitter_attitude)
        else:
            scanner = cls._create_multi_scanner(scanner_element, pulse_freqs, setting_characteristics, emitter_position, emitter_attitude)
       
        return scanner

    @classmethod
    def _parse_emitter(cls, scanner_element) -> Tuple[List[float], Rotation]:
        beam_origin = scanner_element.find('beamOrigin')
        
        emitter_position = [0.0, 0.0, 0.0]

        if beam_origin is not None:
            x = float(beam_origin.get('x', '0.0'))
            y = float(beam_origin.get('y', '0.0'))
            z = float(beam_origin.get('z', '0.0'))
            emitter_position = [x, y, z]

        # Parse the rotation
        emitter_attitude = Rotation.from_xml_node(beam_origin)
        
        return emitter_position, emitter_attitude

    @classmethod
    def _parse_pulse_frequencies(cls, root) -> List[int]:
        pulse_freqs_string = root.get('pulseFreqs_Hz', "0")
        return [int(freq) for freq in pulse_freqs_string.split(',')]

    @classmethod
    def _combine_scanner_characteristics(cls, root) -> SimpleNamespace:

        settings_characteristics = SimpleNamespace(
            beam_div_rad=float(root.get('beamDivergence_rad', '0.0003')),
            pulse_length=float(root.get('pulseLength_ns', '4.0')),
            average_power=float(root.get('averagePower_w', '4.0')),
            beam_quality=float(root.get('beamQualityFactor', '1.0')),
            efficiency=float(root.get('opticalEfficiency', '0.99')),
            receiver_diameter=float(root.get('receiverDiameter_m', '0.15')),
            atmospheric_visibility=float(root.get('atmosphericVisibility_km', '23.0')),
            wavelength=int(root.get('wavelength_nm', '1064'))
        )
        return settings_characteristics
    
    def retrieve_current_settings(self, idx: Optional[int] = 0) -> ScannerSettings:
        current_settings = ScannerSettings()
        current_settings.id = self.id + "_settings"
        current_settings.trajectory_time_interval = self.trajectory_time_interval/1000000000
        if isinstance(self, SingleScanner):
            # For SingleScanner, use the single scanning device
            current_settings.pulse_frequency = self.pulse_freq_hz
            current_settings.is_active = self.is_state_active
            current_settings.beam_divergence_angle = self.scanning_device.beam_div_rad
            current_settings.head_rotation = self.scanning_device.scanner_head.rotate_start
            current_settings.rotation_start_angle = self.scanning_device.scanner_head.current_rotate_angle
            current_settings.rotation_stop_angle = self.scanning_device.scanner_head.rotate_stop
            current_settings.scan_angle = self.scanning_device.beam_deflector.scan_angle
            current_settings.scan_frequency = self.scanning_device.beam_deflector.scan_freq_max
            current_settings.min_vertical_angle = self.scanning_device.beam_deflector.vertical_angle_min
            current_settings.max_vertical_angle = self.scanning_device.beam_deflector.vertical_angle_max

        elif isinstance(self, MultiScanner):
            # For MultiScanner, use the scanning device at index idx
            current_settings.pulse_frequency = self.scanning_devices[idx].pulse_freqs
            current_settings.is_active = self.is_state_active
            current_settings.beam_divergence_angle = self.scanning_devices[idx].beam_div_rad
            current_settings.head_rotation = self.scanning_devices[idx].scanner_head.rotate_start
            current_settings.rotation_start_angle = self.scanning_devices[idx].scanner_head.current_rotate_angle
            current_settings.rotation_stop_angle = self.scanning_devices[idx].scanner_head.rotate_stop
            current_settings.scan_angle = self.scanning_devices[idx].beam_deflector.scan_angle
            current_settings.scan_frequency = self.scanning_devices[idx].beam_deflector.scan_freq_max
            current_settings.min_vertical_angle = self.scanning_devices[idx].beam_deflector.vertical_angle_min
            current_settings.max_vertical_angle = self.scanning_devices[idx].beam_deflector.vertical_angle_max
        
        return current_settings
    
    @classmethod
    def _create_single_scanner(cls, scanner_element, pulse_freqs, setting_characteristics, emitter_position, emitter_attitude) -> 'SingleScanner':
        # Create the single scanner instance
        scanner = SingleScanner(
            id=scanner_element.get('id', 'default'),
            average_power=setting_characteristics.average_power,
            pulse_freqs=pulse_freqs,
            beam_quality=setting_characteristics.beam_quality,
            efficiency=setting_characteristics.efficiency,
            receiver_diameter=setting_characteristics.receiver_diameter,
            atmospheric_visibility=setting_characteristics.atmospheric_visibility,
            wavelength=setting_characteristics.wavelength,
            beam_div_rad=setting_characteristics.beam_div_rad,
            beam_origin=emitter_position,
            beam_orientation=emitter_attitude,
            pulse_length=setting_characteristics.pulse_length
        )

        # Optionally set additional properties (detector, deflector, head, etc.)
        scanner.max_NOR = int(scanner_element.get('maxNOR', 0))
        scanner.beam_deflector = AbstractBeamDeflector.from_xml_node(scanner_element)
        scanner.detector = AbstractDetector.from_xml_node(scanner_element, scanner)
        scanner.scanner_head = ScannerHead.from_xml_node(scanner_element)
      
        # Apply waveform settings if present
        fwf_node = scanner_element.find('FWFSettings')
        fwf_settings = FWFSettings()
        fwf_settings.pulse_length = setting_characteristics.pulse_length

        scanner.apply_settings_FWF(fwf_settings.from_xml_node(fwf_node))
        return scanner


    @classmethod
    def _create_multi_scanner(cls, scanner_element, pulse_freqs, settings, emitter_position, emitter_attitude) -> 'MultiScanner':
        # Handle multi-scanner setup similar to the C++ code
        channels = scanner_element.find('channels')
        n_channels = sum(1 for _ in channels.findall('channel'))

        scan_devs = []
        for idx in range(n_channels):
            scan_dev = ScanningDevice(
                idx, 
                scanner_element.get('id', 'default'),
                settings.beam_div_rad,
                emitter_position,
                emitter_attitude,
                pulse_freqs,
                settings.pulse_length,
                settings.average_power,
                settings.beam_quality,
                settings.efficiency,
                settings.receiver_diameter,
                settings.atmospheric_visibility,
                settings.wavelength * 1e-9,  # Placeholder for range error expression
            )
            scan_dev.received_energy_min = float(scanner_element.get('receivedEnergyMin', '0.0001'))
            scan_devs.append(scan_dev)

        scanner = MultiScanner(scan_devs, str(scanner_element.get('id', 'default')), pulse_freqs)

        # Set properties like beam deflector, detector, scanner head, etc.
        fwf_settings = FWFSettings.from_xml_node(scanner_element.find('FWFSettings'))
        abs_beam_def = AbstractBeamDeflector.from_xml_node(scanner_element)
        abs_detector = AbstractDetector.from_xml_node(scanner_element, scanner)
        scanner_head = ScannerHead.from_xml_node(scanner_element) 

        cls._fill_scan_devs_from_channels(scanner, scanner_element, channels, abs_beam_def, abs_detector,
                                          scanner_head, fwf_settings)
        return scanner

    @classmethod
    def _fill_scan_devs_from_channels(cls, scanner, scanner_element, channels, abs_deflector, abs_detector, scanner_head, fwf_settings) -> None:
        # Iterate over the channels and fill the scan devices, also it should have a counter for the index from 0
        for idx, channel in enumerate(channels.findall('channel')):
            scanner.active_scanner_index = idx
            scanner._cpp_object.set_device_index(idx, idx)

            scanner.device_id = channel.get('id', 'DeviceID')

            emitter_position, emitter_attitude = cls._parse_emitter(channel)
            scanner.head_relative_emitter_position = emitter_position
            scanner.head_relative_emitter_attitude = emitter_attitude
            scanner.apply_settings_FWF(fwf_settings.from_xml_node(channel.find('FWFSettings')))

            update_deflector = True
          
            optics_type = channel.get('optics')
            if optics_type is not None: 
                deflectors_match = (optics_type == "oscillating" and isinstance(abs_deflector, OscillatingMirrorBeamDeflector)
                ) or (
                    optics_type == "conic" and isinstance(abs_deflector, ConicBeamDeflector)
                ) or (
                    optics_type == "line" and isinstance(abs_deflector, FiberArrayBeamDeflector)
                ) or (
                    optics_type == "rotating" and isinstance(abs_deflector, PolygonMirrorBeamDeflector)
                ) or (
                    optics_type == "risley" and isinstance(abs_deflector, RisleyBeamDeflector))

                if not deflectors_match:
                    new_deflector = AbstractBeamDeflector.from_xml_node(channel)
                    scanner.beam_deflector = new_deflector
                    update_deflector = False  # Don't update if we just replaced the deflector
                
                if update_deflector:
                    scanner.beam_deflector = abs_deflector.clone()
                    current_deflector = scanner.beam_deflector
                    current_deflector.scan_freq_min = float(channel.get('scanFreqMin_Hz', current_deflector.scan_freq_min))
                    current_deflector.scan_freq_max = float(channel.get('scanFreqMax_Hz', current_deflector.scan_freq_max))

                    if 'scanAngleMax_deg' in channel.attrib:
                        current_deflector.scan_angle_max = math.radians(float(channel.get('scanAngleMax_deg', 0.0)))

                    # Handle specific updates for Oscillating Mirror Beam Deflector
                    if isinstance(current_deflector, OscillatingMirrorBeamDeflector):
                        current_deflector.scan_product = int(channel.get('scanProduct', current_deflector.scan_product))

                    # Handle specific updates for Fiber Array Beam Deflector
                    elif isinstance(current_deflector, FiberArrayBeamDeflector):
                        current_deflector.num_fibers = int(channel.get('numFibers', current_deflector.num_fibers))

                    # Handle specific updates for Polygon Mirror Beam Deflector
                    elif isinstance(current_deflector, PolygonMirrorBeamDeflector):
                        current_deflector.scan_angle_max = math.radians(float(channel.get('scanAngleEffectiveMax_deg', math.degrees(current_deflector.scan_angle_max))))

                    # Handle specific updates for Risley Beam Deflector
                    elif isinstance(current_deflector, RisleyBeamDeflector):
                        current_deflector.rotor_freq_1 = float(channel.get('rotorFreq1_Hz', 7294)) / (2 * math.pi)
                        current_deflector.rotor_freq_2 = float(channel.get('rotorFreq2_Hz', -4664)) / (2 * math.pi)

            current_detector = abs_detector.clone()
            current_detector.range_max = float(channel.get('rangeMax_m', current_detector.range_max))
            current_detector.accuracy = float(channel.get('accuracy_m', current_detector.accuracy))
            current_detector.range_min = float(channel.get('rangeMin_m', current_detector.range_min))
            scanner.detector = current_detector

            current_head = scanner_head.clone()
            current_head.rotate_per_sec_max = math.radians(float(channel.get('headRotatePerSecMax_deg', math.degrees(current_head.rotate_per_sec_max))))
            axis_node = channel.find("headRotateAxis")
            if axis_node is not None:
                    axis_str = axis_node.text.split()
                    axis = [float(coord) for coord in axis_str]
                    current_head.rotation_axis = axis
            scanner.scanner_head = current_head
            
            scanner.beam_div_rad = float(channel.get('beamDivergence_rad', scanner.beam_div_rad))
            scanner.pulse_length = float(channel.get('pulseLength_ns', scanner.pulse_length))
            if channel.get("wavelength_nm") is not None:
                scanner.wavelength = int(channel.get('wavelength_nm', 1064))
            scanner.max_nor = int(channel.get('maxNOR', 0))
            scanner.received_energy_min = float(channel.get('receivedEnergyMin', scanner.received_energy_min))

    class Config:
        arbitrary_types_allowed = True


class SingleScanner(Scanner):
    def __init__(self, id: str, average_power: float, pulse_freqs: List[int],
                 beam_quality: float, efficiency: float, receiver_diameter: float, 
                 atmospheric_visibility: float, wavelength: int, beam_div_rad: Optional[float] = 0, 
                 beam_origin: Optional[List[float]] = None, beam_orientation: Optional[Rotation] = None,  
                 pulse_length: Optional[float] = 0, scanner_settings: Optional[ScannerSettings] = None, write_waveform=False, 
                 write_pulse=False, calc_echowidth=False, full_wave_noise=False, 
                 platform_noise_disabled=False) -> None:
        
        
        beam_origin = beam_origin or [0.0, 0.0, 0.0]
        
        beam_orientation = beam_orientation or Rotation()
            
        super().__init__(id, supported_pulse_freqs_hz=pulse_freqs)
        self._cpp_object = _helios.SingleScanner(
            beam_div_rad, beam_origin, beam_orientation._cpp_object, pulse_freqs, pulse_length, id, average_power, 
            beam_quality, efficiency, receiver_diameter, atmospheric_visibility, wavelength, 
            write_waveform, write_pulse, calc_echowidth, full_wave_noise, platform_noise_disabled)
        
        self.id = id
        self.scanner_settings = scanner_settings
        self.scanning_device = ScanningDevice(0, id, beam_div_rad, beam_origin, beam_orientation, pulse_freqs, pulse_length, average_power, beam_quality, efficiency, receiver_diameter, atmospheric_visibility, wavelength)
        self.write_waveform = write_waveform
        self.write_pulse = write_pulse
        self.calc_echowidth = calc_echowidth
        self.full_wave_noise = full_wave_noise
        self.platform_noise_disabled = platform_noise_disabled
        self.pulse_freqs = pulse_freqs
        self.supported_pulse_freqs_hz = pulse_freqs
    
    @classmethod
    def from_xml(cls, filename: str, id: str = None) -> 'SingleScanner':
        file_path = AssetManager().find_file_by_name(filename, auto_add=True)
        tree = ET.parse(file_path)
        root = tree.getroot()

        id = root.get('id', id)
        beam_div_rad = float(root.get('beamDivergence_rad'))
        beam_origin = [float(x) for x in root.get('beam_origin').split(',')]
        beam_orientation = root.get('beam_orientation')
        pulse_freqs = [int(x) for x in root.get('pulseFreqs_Hz').split(',')]
        pulse_length = float(root.get('pulseLength_ns'))
        average_power = float(root.get('average_power'))
        beam_quality = float(root.get('beam_quality'))
        efficiency = float(root.get('efficiency'))
        receiver_diameter = float(root.get('receiver_diameter'))
        atmospheric_visibility = float(root.get('atmospheric_visibility'))
        wavelength = int(root.get('wavelength'))
       
        scanner_instance = cls(
            id=id,
            beam_div_rad=beam_div_rad,
            beam_origin=beam_origin,
            beam_orientation=beam_orientation,
            pulse_freqs=pulse_freqs,
            pulse_length=pulse_length,
            average_power=average_power,
            beam_quality=beam_quality,
            efficiency=efficiency,
            receiver_diameter=receiver_diameter,
            atmospheric_visibility=atmospheric_visibility,
            wavelength=wavelength,
            range_err_expr=None
        )
        return cls._validate(scanner_instance)
    
    device_id = create_property('id', 'set_device_id')
    average_power = create_property('avg_power', 'set_average_power')
    beam_div_rad = create_property('beam_div_rad', 'set_beam_divergence')
    
    beam_quality = create_property('beam_quality', 'set_beam_quality')
    efficiency = create_property('optical_efficiency', 'set_optical_efficiency')
    receiver_diameter = create_property('receiver_diameter', 'set_receiver_diameter')
    atmospheric_visibility = create_property('atmospheric_visibility', 'set_atmospheric_visibility')
    wavelength = create_property('wavelength', 'set_wavelength')
    beam_origin = create_property('emitter_position', 'set_head_relative_emitter_position')
    beam_orientation = create_property('emitter_attitude', 'set_head_relative_emitter_attitude')
    max_NOR = create_property('max_nor', 'set_max_nor')
    beam_deflector = create_property('beam_deflector', 'set_beam_deflector')
    detector = create_property('detector', 'set_detector')
    scanner_head = create_property('scanner_head', 'set_scanner_head')
    fwf_settings = create_property('fwf_settings', 'set_fwf_settings')
    num_time_bins = create_property('num_time_bins', 'set_num_time_bins')
    pulse_length = create_property('pulse_length', 'set_pulse_length')
    timewave = create_property('timewave', 'set_time_wave')
    peak_intensity_index = create_property('peak_intensity_index', 'set_peak_intensity_index')
    
    def prepare_discretization(self):
        self.num_time_bins = int(self.pulse_length / self.fwf_settings.bin_size)
        self.timewave = [0.0] * self.num_time_bins

        self.peak_intensity_index = calc_propagation_time_legacy(self.timewave, self.num_time_bins, self.fwf_settings.bin_size, self.pulse_length, 7.0)

    def apply_settings_FWF(self, settings: FWFSettings):
        self._cpp_object.apply_settings_FWF(settings._cpp_object, 0)
        self.fwf_settings = settings
        self.scanning_device.calc_rays_number()
        self.prepare_discretization()
    
    def prepare_simulation(self, is_legacy_energy_model: bool = False):
        self.scanning_device.prepare_simulation(is_legacy_energy_model)
    
    def clone(self):
        new_scanner = SingleScanner(self.id, self.average_power, self.pulse_freqs, self.beam_quality, self.efficiency, self.receiver_diameter, self.atmospheric_visibility, self.wavelength, self.beam_div_rad, self.beam_origin, self.beam_orientation, self.pulse_length, self.scanner_settings, self.write_waveform, self.write_pulse, self.calc_echowidth, self.full_wave_noise, self.platform_noise_disabled)
        new_scanner.max_NOR = self.max_NOR
        new_scanner.beam_deflector = self.beam_deflector
        new_scanner.detector = self.detector
        new_scanner.scanner_head = self.scanner_head
        new_scanner.fwf_settings = self.fwf_settings
        new_scanner.num_time_bins = self.num_time_bins
        new_scanner.timewave = self.timewave
        new_scanner.peak_intensity_index = self.peak_intensity_index
        new_scanner._cpp_object = self._cpp_object.clone()


class MultiScanner(Scanner):
    def __init__(self, 
                 scanning_devices: List[ScanningDevice],
                 id: str,
                 pulse_freqs: Optional[List[int]] = None,
                 device_rotation: Optional[Rotation] = None,
                 global_position: Optional[List[float]] = None,
                 active_scanner_index: Optional[int] = -1,
                 write_waveform: Optional[bool] = False,
                 calc_echowidth: Optional[bool] = False,
                 full_wave_noise: Optional[bool] = False,
                 platform_noise_disabled: Optional[bool] = False) -> None:
      
        pulse_freqs = pulse_freqs or [0]

        super().__init__(id)
        self._cpp_object = _helios.MultiScanner([sd._cpp_object for sd in scanning_devices], id, pulse_freqs)
  
        self.id = id
        self.scanning_devices = scanning_devices
        self.device_rotation = device_rotation or Rotation()
        self.global_position = global_position or [0.0, 0.0, 0.0]
        self.active_scanner_index = active_scanner_index
        self.write_waveform = write_waveform
        self.calc_echowidth = calc_echowidth
        self.full_wave_noise = full_wave_noise
        self.platform_noise_disabled = platform_noise_disabled

    def _get_active_device(self):
        return self.scanning_devices[self.active_scanner_index]
    
    device_id = create_property('id', 'set_device_id', index_function=lambda self: self.active_scanner_index)
    head_relative_emitter_position = create_property('emitter_position', 'set_head_relative_emitter_position', index_function=lambda self: self.active_scanner_index)
    head_relative_emitter_attitude = create_property('emitter_attitude', 'set_head_relative_emitter_attitude', index_function=lambda self: self.active_scanner_index)
    num_time_bins = create_property('num_time_bins', 'set_num_time_bins', index_function=lambda self: self.active_scanner_index)
    timewave = create_property('timewave', 'set_time_wave', index_function=lambda self: self.active_scanner_index)
    pulse_length = create_property('pulse_length', 'set_pulse_length', index_function=lambda self: self.active_scanner_index)
    fwf_settings = create_property('fwf_settings', 'set_fwf_settings', index_function=lambda self: self.active_scanner_index)
    peak_intensity_index = create_property('peak_intensity_index', 'set_peak_intensity_index', index_function=lambda self: self.active_scanner_index)
    beam_deflector = create_property('beam_deflector', 'set_beam_deflector', index_function=lambda self: self.active_scanner_index)
    detector = create_property('detector', 'set_detector', index_function=lambda self: self.active_scanner_index)
    scanner_head = create_property('scanner_head', 'set_scanner_head', index_function=lambda self: self.active_scanner_index)
    beam_div_rad = create_property('beam_div_rad', 'set_beam_divergence', index_function=lambda self: self.active_scanner_index)
    wavelength = create_property('wavelength', 'set_wavelength', index_function=lambda self: self.active_scanner_index)
    max_nor = create_property('max_nor', 'set_max_nor', index_function=lambda self: self.active_scanner_index)
    received_energy_min = create_property('received_energy_min', 'set_received_energy_min', index_function=lambda self: self.active_scanner_index)
    average_power = create_property('average_power', 'set_average_power', index_function=lambda self: self.active_scanner_index)
    pulse_freqs = create_property('pulse_freqs', 'set_pulse_freqs', index_function=lambda self: self.active_scanner_index)
    beam_quality = create_property('beam_quality', 'set_beam_quality', index_function=lambda self: self.active_scanner_index)
    efficiency = create_property('efficiency', 'set_optical_efficiency', index_function=lambda self: self.active_scanner_index)
    receiver_diameter = create_property('receiver_diameter', 'set_receiver_diameter', index_function=lambda self: self.active_scanner_index)
    
    def prepare_discretization(self):
        self.num_time_bins = int(self.pulse_length / self.fwf_settings.bin_size)
        self.timewave = [0.0] * self.num_time_bins

        self.peak_intensity_index = calc_propagation_time_legacy(self.timewave, self.num_time_bins, self.fwf_settings.bin_size, self.pulse_length, 7.0)

    def apply_settings_FWF(self, settings: FWFSettings):
        active_device = self._get_active_device()
        self._cpp_object.apply_settings_FWF(settings._cpp_object, self.active_scanner_index)
        self.fwf_settings = settings
        active_device.calc_rays_number()
        self.prepare_discretization()

    def prepare_simulation(self, is_legacy_energy_model: bool = False):
        num_devices = len(self.scanning_devices)
        for i in range(num_devices):
            self.scanning_devices[i].prepare_simulation(is_legacy_energy_model)
    
    def clone(self):
        new_scanner = MultiScanner(self.scanning_devices, self.id, self.pulse_freqs, self.device_rotation, self.global_position, self.active_scanner_index, self.write_waveform, self.calc_echowidth, self.full_wave_noise, self.platform_noise_disabled)
        new_scanner._cpp_object = self._cpp_object.clone()
        new_scanner.scanning_devices = [sd.clone() for sd in self.scanning_devices]
        new_scanner.device_rotation = self.device_rotation
        new_scanner.global_position = self.global_position
        new_scanner.active_scanner_index = self.active_scanner_index
        new_scanner.pulse_freqs = self.pulse_freqs
        new_scanner.write_waveform = self.write_waveform
        new_scanner.calc_echowidth = self.calc_echowidth
        new_scanner.full_wave_noise = self.full_wave_noise
        new_scanner.platform_noise_disabled = self.platform_noise_disabled
        return new_scanner


class AbstractDetector(Validatable):
    def __init__(self, scanner: Optional[Scanner], range_max: Optional[float] =  sys.float_info.max,
                  accuracy: Optional[float] = .0, range_min: Optional[float] = .0) -> None:
        scanner = scanner or Scanner()
        self._cpp_object = _helios.AbstractDetector(scanner._cpp_object, accuracy, range_min, range_max)
        self.scanner = scanner
        self.accuracy = accuracy
        self.range_min = range_min
        self.range_max = range_max
    
    accuracy: Optional[float] = ValidatedCppManagedProperty("accuracy")
    range_min: Optional[float] = ValidatedCppManagedProperty("range_min")
    range_max: Optional[float] = ValidatedCppManagedProperty("range_max")

    @classmethod
    def from_xml_node(cls, detector_node: ET.Element, scanner: Scanner) -> 'AbstractDetector':
    
        range_max = float(detector_node.get('rangeMax_m', 1e20))
        accuracy = float(detector_node.get('accuracy_m', '0.0'))
        range_min = float(detector_node.get('rangeMin_m', '0.0'))
    
        return FullWaveformPulseDetector._validate(FullWaveformPulseDetector(scanner=scanner, range_max=range_max, accuracy=accuracy, range_min=range_min))

    def clone(self):
        new_detector = AbstractDetector(self.scanner, self.range_max, self.accuracy, self.range_min)
        new_detector._cpp_object = self._cpp_object.clone()
        return new_detector


class FullWaveformPulseDetector(AbstractDetector):
    def __init__(self, scanner: Optional[Scanner], range_max: Optional[float], accuracy: Optional[float] = .0, range_min: Optional[float] = .0, pulse_length: Optional[float] = .0) -> None:
        scanner = scanner or Scanner()
        self._cpp_object = _helios.FullWaveformPulseDetector(scanner._cpp_object, accuracy, range_min, range_max)
        self.scanner = scanner
        self.accuracy = accuracy
        self.range_min = range_min
        self.range_max = range_max
        self.pulse_length = pulse_length
    
    def clone(self):
        new_detector = FullWaveformPulseDetector(self.scanner, self.range_max, self.accuracy, self.range_min, self.pulse_length)
        new_detector._cpp_object = self._cpp_object.clone()
        return new_detector


LEICAALS50 = Scanner.from_xml("data/scanners_als.xml", id="leica_als50")

LEICAALS50_II = Scanner.from_xml("data/scanners_als.xml", id="leica_als50-ii")

OPTECH_2033 = Scanner.from_xml("data/scanners_als.xml", id="optech_2033")

OPTECH_3100 = Scanner.from_xml("data/scanners_als.xml", id="optech_3100")

OPTECH_GALAXY = Scanner.from_xml("data/scanners_als.xml", id="optech_galaxy")

RIEGL_LMS_Q560 = Scanner.from_xml("data/scanners_als.xml", id="riegl_lms-q560")

RIEGL_LMS_Q780 = Scanner.from_xml("data/scanners_als.xml", id="riegl_lms-q780")

RIEGL_VQ_780i = Scanner.from_xml("data/scanners_als.xml", id="riegl_vq_780i")

RIEGL_VUX_1UAV = Scanner.from_xml("data/scanners_als.xml", id="riegl_vux-1uav")

RIEGL_VUX_1UAV22 = Scanner.from_xml("data/scanners_als.xml", id="riegl_vux-1uav22")

RIEGL_VUX_1HA22 = Scanner.from_xml("data/scanners_als.xml", id="riegl_vux-1ha22")

RIEGL_VQ_880g = Scanner.from_xml("data/scanners_als.xml", id="riegl_vq-880g")

RIEGL_VQ_1560i = Scanner.from_xml("data/scanners_als.xml", id="riegl_vq-1560i")

LIVOX_MID70 = Scanner.from_xml("data/scanners_als.xml", id="livox_mid-70")

LIVOX_MID100 = Scanner.from_xml("data/scanners_als.xml", id="livox-mid-100")

LIVOX_MID100a = Scanner.from_xml("data/scanners_als.xml", id="livox-mid-100a")

LIVOX_MID100b = Scanner.from_xml("data/scanners_als.xml", id="livox-mid-100b")

LIVOX_MID100c = Scanner.from_xml("data/scanners_als.xml", id="livox-mid-100c")


#TLS 

RIEGL_VZ_400 = Scanner.from_xml("data/scanners_tls.xml", id="riegl_vz400")

RIEGL_VZ_1000 = Scanner.from_xml("data/scanners_tls.xml", id="riegl_vz1000")

RIEGL_VQ_450 = Scanner.from_xml("data/scanners_tls.xml", id="riegl_vq-450")

LIVOX_MID70_TLS = Scanner.from_xml("data/scanners_tls.xml", id="livox_mid-70")

VLP16 = Scanner.from_xml("data/scanners_tls.xml", id="vlp16")

VELODYNE_HDL_64E = Scanner.from_xml("data/scanners_tls.xml", id="velodyne_hdl-64e")

TRACTOR_SCANNER = Scanner.from_xml("data/scanners_tls.xml", id="tractorscanner")

PANO_SCANNER = Scanner.from_xml("data/scanners_tls.xml", id="panoscanner")
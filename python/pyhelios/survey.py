from pyhelios.scene import Scene
from pyhelios.utils import Validatable, ValidatedCppManagedProperty, AssetManager, PyHeliosException
from pyhelios.scanner import Scanner, AbstractDetector, ScannerSettings, SingleScanner, MultiScanner
from pyhelios.platforms import PlatformSettings, Platform
from pyhelios.primitives import Rotation, FWFSettings, SimulationCycleCallback, Measurement, Trajectory
import threading
from pyhelios.leg import Leg

import os 
from threading import Thread
from typing import Optional, List, Dict
import xml.etree.ElementTree as ET
import _helios

from datetime import datetime, timezone
import time

class Survey(Validatable):
    def __init__(self, name: Optional[str] = "", num_runs: Optional[int] = -1, 
                 sim_speed_factor: Optional[float] = 1., scanner: Optional[Scanner] = None, 
                 legs: Optional[List[Leg]] = None) -> None:
        
        self._cpp_object = _helios.Survey()
        self.name = name
        self.num_runs = num_runs
        self.sim_speed_factor = sim_speed_factor
        self.scanner = scanner
        self.legs = legs or []

        self.is_legacy_energy_model: Optional[bool] = False # this is used in c++ Simulation class
        self.fixed_gps_time_start: Optional[str] = ""

        self.is_started: Optional[bool] = False
        self.is_paused: Optional[bool] = False
        self.is_stopped: Optional[bool] = False 
        self.is_finished: Optional[bool] = False 
        self.output_path: Optional[str] = ""
        self.las_scale: Optional[float] = 0.0001
        self.las_output: Optional[bool] = False
        self.las_10: Optional[bool] = False
        self.zip_output: Optional[bool] = False
        self.split_by_channel: Optional[bool] = False
        self.playback: Optional[_helios.SurveyPlayback] = None
        self.callback: Optional[_helios.SimulationCycleCallback] = None
        

        self.kd_factory_type: Optional[int] = 4
        self.kdt_num_jobs: Optional[int] = 0
        self.kdt_geom_jobs: Optional[int] = 0
        self.kdt_sah_loss_nodes: Optional[int] = 32
        self.parallelization_strategy: Optional[int] = 1

        self.export_to_file: Optional[bool] = False
        self.write_waveform: Optional[bool] = False
        self.calc_echowidth: Optional[bool] = False
        self.fullwavenoise: Optional[bool] = False
        self.is_platform_noise_disabled: Optional[bool] = False
        self.final_output: Optional[bool] = False
        self.scanner_settings_templates: Dict[str, ScannerSettings] = {}
        self.platform_settings_templates: Dict[str, PlatformSettings] = {}
        self.callback_frequency: Optional[int] = 0
       
    name: Optional[str] = ValidatedCppManagedProperty("name")
    num_runs: Optional[int] = ValidatedCppManagedProperty("num_runs")
    sim_speed_factor: Optional[float] = ValidatedCppManagedProperty("sim_speed_factor")
    scanner: Optional[Scanner] = ValidatedCppManagedProperty("scanner")
    legs: Optional[List[Leg]] = ValidatedCppManagedProperty("legs")
     
    @classmethod
    def from_xml(cls, filename: str) -> 'Survey':
        # Locate the XML file using the AssetManager
        file_path = AssetManager().find_file_by_name(filename, auto_add=True)

        # Parse the XML file
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        # Extract main survey attributes
        survey_node = root.find('survey')
        if survey_node is None:
            raise ValueError("Invalid XML file, missing 'survey' tag")
        name = survey_node.get('name', "")
        # Optional scanner parsing if specified in XML
        scanner_path, scanner_id = survey_node.get('scanner').split('#') if survey_node.get('scanner') else (None, None)
        scanner = Scanner.from_xml(scanner_path, scanner_id) if scanner_path else None
        
        platform_path, platform_id = survey_node.get('platform').split('#') if survey_node.get('scanner') else (None, None)
        scanner.platform = Platform.from_xml(platform_path, platform_id) if platform_path else None

        if survey_node.find("FWFSettings") is not None:
            scanner.apply_settings_FWF(FWFSettings.from_xml_node(survey_node.find("FWFSettings")))
        
        num_runs = int(survey_node.get('numRuns', 1))

        speed = float(survey_node.get('simSpeedFactor', 1.0))

        sim_speed_factor = 1 / speed if speed > 0 else 1.0

        #TODO: add detector overloading -  XmlSurveyLoader::handleCoreOverloading
        # TODO: add interpolated legs + platform

        platform_settings_templates = {}
        for template_node in root.findall('platformSettings'):
            template_id = template_node.get('id')
            
            if template_id:
                template = PlatformSettings.from_xml_node(template_node)
                platform_settings_templates[template_id] = template

        scanner_settings_templates = {}
        for template_node in root.findall('scannerSettings'):
            template_id = template_node.get('id')
            
            if template_id:
                template = ScannerSettings.from_xml_node(template_node)
                scanner_settings_templates[template_id] = template
                
        # Extract legs information
        legs = []
        for idx, leg_node in enumerate(survey_node.findall('leg')):
          
            leg = Leg.from_xml(leg_node, idx, platform_settings_templates=platform_settings_templates, scanner_settings_templates = scanner_settings_templates)
            legs.append(leg)
            # TODO:  waypoints for interpolated legs

        # integrateSurveyAndLegs 
        if(scanner.beam_deflector is not None):
            for leg in legs:
                if not (leg.scanner_settings.vertical_resolution == 0 and leg.scanner_settings.horizontal_resolution == 0):
                    leg.scanner_settings.scan_frequency = (leg.scanner_settings.pulse_frequency * leg.scanner_settings.vertical_resolution) / (2 * scanner.beam_deflector.scan_angle_max) 
                    leg.scanner_settings.head_rotation =  leg.scanner_settings.horizontal_resolution * leg.scanner_settings.scan_frequency

        # validate Survey to derived func!!!!!!!!!!!
        for leg in legs:
            scan_settings = leg.scanner_settings
            beam_deflector = scanner.beam_deflector
            if scan_settings.scan_frequency < beam_deflector.scan_freq_min:
                raise ValueError(f"Leg {leg.serial_id} scan frequency is below the minimum frequency of the beam deflector.\n The requested scanning frequency cannot be achieved by this scanner. \n")
            if scan_settings.scan_frequency > beam_deflector.scan_freq_max and beam_deflector.scan_freq_max != 0:
                raise ValueError(f"Leg {leg.serial_id} scan frequency is above the maximum frequency of the beam deflector.\n The requested scanning frequency cannot be achieved by this scanner.\n")

        #SCene parsing
        scene_path, scene_id = survey_node.get('scene').split('#') if survey_node.get('scene') else (None, None)

        kd_factory_type = 4
        kdt_num_jobs = os.cpu_count()
        kdt_sah_loss_nodes = 32
        scanner.platform.scene = Scene.read_from_xml(scene_path, scene_id, kd_factory_type=kd_factory_type, kdt_num_jobs=kdt_num_jobs, kdt_sah_loss_nodes=kdt_sah_loss_nodes)
        # CHECK AND IF SPECTRAL LIB IS WORKING add it

        for scene_part in scanner.platform.scene.scene_parts:
            if scene_part.sorh is None:
                continue
            
            num_primitives = len(scene_part.primitives)
            baseline_primitives = scene_part.sorh.baseline.primitives
            for i in range(num_primitives):
                baseline_primitives.primitives[i].material = scene_part.primitives[i].material

        
        #TODO: add shifting of interpolated objects and a random offset
   
        pos1 = scanner.platform.scene._cpp_object.bbox_crs.vertices[0].position
        pos2 = scanner.platform.scene._cpp_object.bbox_crs.vertices[1].position

        # Check that the positions are valid tuples with the same length (x, y, z)
        if len(pos1) != len(pos2):
            raise ValueError("Positions of the vertices must have the same length (x, y, z).")

        # Calculate the bounding box size (assuming 3D coordinates: x, y, z)
        bbox_size = tuple(pos2[i] - pos1[i] for i in range(len(pos1)))

        # Halve the bbox size (center the bounding box)
        bbox_size = tuple(coord / 2 for coord in bbox_size)

        # Calculate the center of the bounding box for shifting purposes
        shift = tuple(pos1[i] + bbox_size[i] for i in range(len(pos1)))
        
        for leg in legs:
            if leg.platform_settings is not None:
                leg.platform_settings.position = (leg.platform_settings.position[0] - shift[0], leg.platform_settings.position[1] - shift[1], leg.platform_settings.position[2] - shift[2])
        
        scanner._cpp_object.initialize_sequential_generators()
        scanner.platform.scene._cpp_object.build_kd_grove()

        # Create the validated Survey instance
        survey_instance = cls(
            name=name,
            scanner=scanner,
            num_runs=num_runs,
            sim_speed_factor=sim_speed_factor,
            legs=legs
        )

        survey_instance.scanner.write_waveform = survey_instance.write_waveform
        survey_instance.scanner.calc_echowidth = survey_instance.calc_echowidth
        survey_instance.scanner.fullwavenoise = survey_instance.fullwavenoise
        survey_instance.scanner.is_platform_noise_disabled = survey_instance.is_platform_noise_disabled 
        survey_instance.scanner_settings_templates = scanner_settings_templates
        survey_instance.platform_settings_templates = platform_settings_templates
        # Validate the created instance using _validate method
        survey_instance = cls._validate(survey_instance)
        
        return survey_instance
    
    def run(self, num_threads: Optional[int] = 0, chunk_size: Optional[int] = 32, warehouse_factor: Optional[int] = 1,
            callback_frequency: Optional[int] = 0, blocking: Optional[bool] = True, export_to_file: Optional[bool] = True):
            
            if num_threads == 0:
                num_threads = os.cpu_count()
            
            fms = None
            
            if export_to_file: #TODO: finish with export to file
                fms = _helios.FMSFacadeFactory().build_facade(self.output_path, self.las_scale, self.las_output, self.las_10, self.zip_output, self.split_by_channel, self._cpp_object)
           
            det = self.scanner.scanning_device.detector if isinstance(self.scanner, SingleScanner) else self.scanner.scanning_devices[0].detector

            ptpf = _helios.PulseThreadPoolFactory(self.parallelization_strategy, num_threads-1, det.accuracy, chunk_size, warehouse_factor)  
            pulse_thread_pool = ptpf.make_pulse_thread_pool()

            current_time = datetime.now(timezone.utc).isoformat(timespec='seconds') 
            self.playback = _helios.SurveyPlayback(self._cpp_object, _helios.FMSFacade(), self.parallelization_strategy, pulse_thread_pool, chunk_size, current_time, self.is_legacy_energy_model, export_to_file)
            self.playback.callback = self.callback._cpp_object
            self.playback.callback_frequency = callback_frequency
            self.scanner._cpp_object.cycle_measurements_mutex  = None 

            self.scanner._cpp_object.cycle_measurements = []
            self.scanner._cpp_object.cycle_trajectories = []
            self.scanner._cpp_object.all_measurements = []
            self.scanner._cpp_object.all_trajectories = []
            self.scanner._cpp_object.all_output_paths = []
            #NOTE: threaded version was realised, but hidden for this version, to simplify the usage
            self.playback.start()
            
            self.is_started = True

    def pause(self):
        if not self.is_started:
            raise PyHeliosException("PyHeliosSimulation was not started so it cannot be paused")
        
        if self.is_stopped:
            raise PyHeliosException("PyHeliosSimulation was stopped so it cannot be paused")

            
        if self.is_finished:
            raise PyHeliosException("PyHeliosSimulation was finished so it cannot be paused")

        self.playback.pause(True)
        self.is_paused = True
    
    def stop(self):
        if not self.is_started:
            raise PyHeliosException("PyHeliosSimulation was not started so it cannot be stopped")
        
        if self.is_stopped:
            raise PyHeliosException("PyHeliosSimulation was already stopped")
        
        if self.is_finished:
            raise PyHeliosException("PyHeliosSimulation was finished so it cannot be stopped")
        
        self.playback.stop()
        self.is_stopped = True
    
    def resume(self):
        if not self.is_started:
            raise PyHeliosException("PyHeliosSimulation was not started so it cannot be resumed")
        
        if self.is_stopped:
            raise PyHeliosException("PyHeliosSimulation was stopped so it cannot be resumed")
        
        if self.playback.is_finished:
            raise PyHeliosException("PyHeliosSimulation was finished so it cannot be resumed")
        if not self.is_paused:
            raise PyHeliosException("PyHeliosSimulation was not paused so it cannot be resumed")
        
        self.playback.resume()
        self.is_paused = False

    def join_output(self):
        if not self.is_started or self.is_paused:
            raise PyHeliosException("PyHeliosSimulation is not running so it cannot be joined")
        
        outpath = ""
        if self.export_to_file:
            outpath = str(self.scanner.fms.write.getMeasurementWriterOutputPath())
        
        if self.callback_frequency > 0 and self.callback is not None:
            if not self.playback.is_finished:
                # Return empty vectors if the simulation is not finished yet
                return ([], [], outpath, [outpath], False)
            else:
                # Return collected data if the simulation is finished
                self.is_finished = True
                return (self.scanner.all_measurements, 
                        self.scanner.all_trajectories, 
                        outpath, 
                        self.scanner.all_output_paths, 
                        True)

        if self.thread and self.thread.is_alive():
            self.thread.join()
        
        if self.playback.fms is not None:
            self.playback.fms.disconnect() 
        
        self.finished = True

   
        if not self.final_output:
            return ([], [], outpath, [], False)
        
        return (self.scanner.all_measurements, 
                self.scanner.all_trajectories, 
                outpath, 
                self.scanner.all_output_paths, 
                True)
   
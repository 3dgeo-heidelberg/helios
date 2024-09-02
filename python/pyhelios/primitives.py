from pyhelios.utils import Validatable, ValidatedCppManagedProperty

from pydantic import ConfigDict
from typing import Optional, List
import numpy as np

import _helios

class Rotation(Validatable):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    def __init__(self, q0: Optional[float] = .0, q1: Optional[float] = .0, q2: Optional[float] = .0, q3: Optional[float] = .0):
        
        self._cpp_object = _helios.Rotation(q0, q1, q2, q3, False)
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
    
    q0: Optional[float] = ValidatedCppManagedProperty("q0")
    q1: Optional[float] = ValidatedCppManagedProperty("q1")
    q2: Optional[float] = ValidatedCppManagedProperty("q2")
    q3: Optional[float] = ValidatedCppManagedProperty("q3")


class Vertex(Validatable):
    def __init__(self, pos: Optional[List[float]]=[]) -> None:
        self._cpp_object = _helios.Vertex()


class DetailedVoxel(Validatable):
    def __init__(self, center: Optional[List[float]], voxel_size: Optional[float], int_values: Optional[List[int]] = None, float_values: Optional[List[float]] = None) -> None:   #def __init__(self, nb_echos: Optional[int] = 0, nb_sampling: Optional[int] = 0, max_pad: Optional[float] = .0): #:
        
        self._cpp_object = _helios.DetailedVoxel((center, voxel_size, int_values, float_values) if int_values is not None and float_values is not None else ())
        
        self.nb_echos = self._cpp_object.nb_echos
        
        self.nb_sampling = self._cpp_object.nb_sampling
        self.max_pad = self._cpp_object.max_pad 
    
    nb_echos: Optional[int] = ValidatedCppManagedProperty("nb_echos")
    nb_sampling: Optional[int] = ValidatedCppManagedProperty("nb_sampling")
    max_pad: Optional[float] = ValidatedCppManagedProperty("max_pad")
    

class Triangle(Validatable):
    def __init__(self, v0: Optional[Vertex], v1: Optional[Vertex], v2: Optional[Vertex]) -> None:
        self._cpp_object = _helios.Triangle(v0._cpp_object, v1._cpp_object, v2._cpp_object)


class Material(Validatable):
    def __init__(self, name: Optional[str], mat_file_path: Optional[str], is_ground: Optional[bool] = False, 
                 use_vertex_colors: Optional[bool] = False, reflectance: Optional[float] = None, specularity: Optional[float] = .0, ) -> None:
        self._cpp_object = _helios.Material()
        self.name = name
        self.mat_file_path = mat_file_path
        self.is_ground = is_ground
        self.use_vertex_colord = use_vertex_colors
        self.reflectance = reflectance
        self.specularity = specularity
    
    name: Optional[str] = ValidatedCppManagedProperty("name")
    mat_file_path: Optional[str] = ValidatedCppManagedProperty("mat_file_path")
    is_ground: Optional[bool] = ValidatedCppManagedProperty("is_ground")
    use_vertex_colors: Optional[bool] = ValidatedCppManagedProperty("use_vertex_color")
    reflectance: Optional[float] = ValidatedCppManagedProperty("reflectance")
    specularity: Optional[float] = ValidatedCppManagedProperty("specularity")
        

class AABB(Validatable):
    def __init__(self) -> None:
        self._cpp_object = _helios.AABB.create()


class Primitive(Validatable):
    def __init__(self,  material: Optional[Material] = None, aabb: Optional[AABB] = None,
                 detailed_voxel: Optional[DetailedVoxel] = None, vertices: Optional[List[Vertex]] = None, triangles: Optional[List[Triangle]] = None, 
                 scene_parts: Optional[List['ScenePart']] = None) -> None:
        self._cpp_object = _helios.Primitive()
        if scene_parts is not None:
            from pyhelios.scene import ScenePart
            self.scene_parts = scene_parts
        else:
            self.scene_parts = []


class Trajectory(Validatable):
    def __init__(self, gps_time: Optional[float] = .0, position: Optional[List[float]] = [0, 0, 0], roll: Optional[float] = .0, pitch: Optional[float] = .0, yaw: Optional[float] = .0) -> None:
        self._cpp_object = _helios.Trajectory()
        self.position = position
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
    gps_time: Optional[float] = ValidatedCppManagedProperty("gps_time")
    position: Optional[List[float]] = ValidatedCppManagedProperty("position")
    roll: Optional[float] = ValidatedCppManagedProperty("roll")
    pitch: Optional[float] = ValidatedCppManagedProperty("pitch")
    yaw: Optional[float] = ValidatedCppManagedProperty("yaw")


class Measurement(Validatable):
    def __init__(self, hit_object_id: Optional[str] = "", position: Optional[List[float]] = [0, 0, 0], beam_direction: Optional[List[float]] = [0, 0, 0], 
                beam_origin: Optional[List[float]] = [0, 0, 0], distance: Optional[float] = .0, intensity: Optional[float] = .0, 
                echo_width: Optional[float] = .0, return_number: Optional[int] = 0, pulse_return_number: Optional[int] = 0, 
                fullwave_index: Optional[int] = 0, classification: Optional[int] = 0, gps_time: Optional[float] = .0):
        
        self._cpp_object = _helios.Measurement()
        self.hit_object_id = hit_object_id
        self.position = position
        self.beam_direction = beam_direction
        self.beam_origin = beam_origin
        self.distance = distance
        self.intensity = intensity
        self.echo_width = echo_width
        self.return_number = return_number
        self.pulse_return_number = pulse_return_number
        self.fullwave_index = fullwave_index
        self.classification = classification
        self.gps_time = gps_time
        
    hit_object_id: Optional[str] = ValidatedCppManagedProperty("hit_object_id")
    position: Optional[List[float]] = ValidatedCppManagedProperty("position")
    beam_direction: Optional[List[float]] = ValidatedCppManagedProperty("beam_direction")
    beam_origin: Optional[List[float]] = ValidatedCppManagedProperty("beam_origin")
    distance: Optional[float] = ValidatedCppManagedProperty("distance")
    intensity: Optional[float] = ValidatedCppManagedProperty("intensity")
    echo_width: Optional[float] = ValidatedCppManagedProperty("echo_width")
    return_number: Optional[int] = ValidatedCppManagedProperty("return_number")
    pulse_return_number: Optional[int] = ValidatedCppManagedProperty("pulse_return_number")
    fullwave_index: Optional[int] = ValidatedCppManagedProperty("fullwave_index")
    classification: Optional[int] = ValidatedCppManagedProperty("classification")
    gps_time: Optional[float] = ValidatedCppManagedProperty("gps_time")


class FWFSettings(Validatable):
    def __init__(self, bin_size_ns: Optional[float] = .0, beam_sample_quality: Optional[int] = 0) -> None:
        self._cpp_object = _helios.FWFSettings()
        self.bin_size_ns = bin_size_ns
        self.beam_sample_quality = beam_sample_quality
    
    bin_size_ns: Optional[float] = ValidatedCppManagedProperty("bin_size_ns")
    beam_sample_quality: Optional[int] = ValidatedCppManagedProperty("beam_sample_quality")


class AbstractBeamDeflector(Validatable):
    def __init__(self, scan_angle_max: Optional[float] = .0, scan_freq_max: Optional[float] = .0, scan_freq_min: Optional[float] = .0, scan_freq: Optional[float] = .0, scan_angle: Optional[float] = .0,
                 vertical_angle_min: Optional[float] = .0, vertical_angle_max: Optional[float] = .0, current_beam_angle: Optional[float] = .0, angle_diff_rad: Optional[float] = .0) -> None:
        self._cpp_object = _helios.AbstractBeamDeflector(scan_angle_max, scan_freq_max, scan_freq_min)
        self.scan_angle_max = scan_angle_max
        self.scan_freq_max = scan_freq_max
        self.scan_freq_min = scan_freq_min
        self.scan_freq = scan_freq
        self.scan_angle = scan_angle
        self.vertical_angle_min = vertical_angle_min
        self.vertical_angle_max = vertical_angle_max
        self.current_beam_angle = current_beam_angle
        self.angle_diff_rad = angle_diff_rad

    scan_angle_max: Optional[float] = ValidatedCppManagedProperty("scan_angle_max")
    scan_freq_max: Optional[float] = ValidatedCppManagedProperty("scan_freq_max")
    scan_freq_min: Optional[float] = ValidatedCppManagedProperty("scan_freq_min")
    scan_freq: Optional[float] = ValidatedCppManagedProperty("scan_freq")
    scan_angle: Optional[float] = ValidatedCppManagedProperty("scan_angle")
    vertical_angle_min: Optional[float] = ValidatedCppManagedProperty("vertical_angle_min")
    vertical_angle_max: Optional[float] = ValidatedCppManagedProperty("vertical_angle_max")
    current_beam_angle: Optional[float] = ValidatedCppManagedProperty("current_beam_angle")
    angle_diff_rad: Optional[float] = ValidatedCppManagedProperty("angle_diff_rad")

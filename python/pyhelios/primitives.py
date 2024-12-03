from pyhelios.utils import Validatable, ValidatedCppManagedProperty, RandomnessGenerator

from pydantic import BaseModel, Field
from typing import Optional, List, Dict
from enum import Enum
import threading
import re
import numpy as np
import xml.etree.ElementTree as ET
from collections import deque
import math
from abc import ABC, abstractmethod
import _helios
 

class PrimitiveType(Enum):
    NONE = _helios.PrimitiveType.NONE
    TRIANGLE = _helios.PrimitiveType.TRIANGLE
    VOXEL = _helios.PrimitiveType.VOXEL


class Rotation(Validatable):
    """
        q0: float - The scalar part of the quaternion
        q1: float - The x component of the vector part of the quaternion
        q2: float - The y component of the vector part of the quaternion
        q3: float - The z component of the vector part of the quaternion
    """

    def __init__(self, q0: Optional[float] = .0, q1: Optional[float] = .0, q2: Optional[float] = .0, q3: Optional[float] = .0, needs_normalization: Optional[bool] = False):
        
        self._cpp_object = _helios.Rotation(q0, q1, q2, q3, needs_normalization)
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        #TODO: add normalization procedure
    
    q0: Optional[float] = ValidatedCppManagedProperty("q0")
    q1: Optional[float] = ValidatedCppManagedProperty("q1")
    q2: Optional[float] = ValidatedCppManagedProperty("q2")
    q3: Optional[float] = ValidatedCppManagedProperty("q3")

    @classmethod
    def from_xml_node(cls, beam_origin_node: ET.Element) -> 'Rotation':
        if beam_origin_node is None:
            return cls._validate(cls(q0=1., q1=1., q2=0., q3=0.))

        rotation_node = beam_origin_node.findall('rot')
        result_rotation = cls(q0=1.0, q1=0.0, q2=0.0, q3=0.0)  # Identity rotation

        for node in rotation_node:
            axis = node.get('axis')
            angle_deg = float(node.get('angle_deg', 0.0))
            angle_rad = math.radians(angle_deg)  # Convert to radians
            if angle_rad != 0:
                rotation = cls._create_rotation_from_axis_angle(axis, angle_rad)
                result_rotation = result_rotation.apply_rotation(rotation)  # Combine rotations
        return result_rotation._validate(result_rotation)

    @staticmethod
    def _create_rotation_from_axis_angle(axis: str, angle: float) -> 'Rotation':
        if axis.lower() == 'x':
            return Rotation(
                math.cos(angle / 2),
                math.sin(angle / 2),
                0.0,
                0.0
            )
        elif axis.lower() == 'y':
            return Rotation(
                math.cos(angle / 2),
                0.0,
                math.sin(angle / 2),
                0.0
            )
        elif axis.lower() == 'z':
            return Rotation(
                math.cos(angle / 2),
                0.0,
                0.0,
                -math.sin(angle / 2)
            )
    
    def scale(self, factor: float) -> None:
        """Scale the positions of all vertices by a given factor."""
        for vertex in self.vertices:
            vertex.position[0] *= factor
            vertex.position[1] *= factor
            vertex.position[2] *= factor

    def translate(self, shift: List[float]) -> None:
        """Translate the positions of all vertices by a given shift."""
        for vertex in self.vertices:
            vertex.position[0] += shift[0]
            vertex.position[1] += shift[1]
            vertex.position[2] += shift[2]

    def apply_rotation(self, r: 'Rotation') -> 'Rotation':
        return  Rotation(r.q0 * self.q0 - (r.q1 * self.q1 + r.q2 * self.q2 + r.q3 * self.q3),
                         r.q1 * self.q0 + r.q0 * self.q1 + (r.q2 * self.q3 - r.q3 * self.q2),
                         r.q2 * self.q0 + r.q0 * self.q2 + (r.q3 * self.q1 - r.q1 * self.q3),
                         r.q3 * self.q0 + r.q0 * self.q3 + (r.q1 * self.q2 - r.q2 * self.q1))
    
    def apply_vector_rotation(self, vec: List[float]) -> List[float]:
        x = vec[0]
        y = vec[1]
        z = vec[2]
        s = self.q1 * x + self.q2 * y + self.q3 * z
        return [2*(self.q0*(self.q0 * x - (self.q2 * z - self.q3 * y)) + s * self.q1) - x,
                2*(self.q0*(self.q0 * y - (self.q3 * x - self.q1 * z)) + s * self.q2) - y,
                2*(self.q0*(self.q0 * z - (self.q1 * y - self.q2 * x)) + s * self.q3) - z]

    def clone(self):
        return Rotation(self.q0, self.q1, self.q2, self.q3)


class Vertex(Validatable):
    def __init__(self, position: Optional[List[float]]=None, tex_coords: Optional[List[float]] = None, normal: Optional[List[float]]=None) -> None:
        self._cpp_object = _helios.Vertex()
        self.position = position or [0., 0., 0.]
        self.normal = normal or [0., 0., 0.]
        self.tex_coords = tex_coords or [0., 0.]

    position: Optional[List[float]] = ValidatedCppManagedProperty("position")
    normal: Optional[List[float]] = ValidatedCppManagedProperty("normal")
    tex_coords: Optional[List[float]] = ValidatedCppManagedProperty("tex_coords")
  
    def clone(self):
        new_vertex = Vertex(self.position[:], self.tex_coords[:])
        new_vertex._cpp_object = self._cpp_object.clone()
        return new_vertex
    
    
class DetailedVoxel(Validatable):
    def __init__(self, center: Optional[List[float]], voxel_size: Optional[float], int_values: Optional[List[int]] = None, float_values: Optional[List[float]] = None) -> None:   #def __init__(self, nb_echos: Optional[int] = 0, nb_sampling: Optional[int] = 0, max_pad: Optional[float] = .0): #:
        
        self._cpp_object = _helios.DetailedVoxel((center, voxel_size, int_values, float_values) if int_values is not None and float_values is not None else ())
        self.center = center
        self.voxel_size = voxel_size
        self.int_values = int_values
        self.float_values = float_values    
        self.nb_echos = self.int_values[0] if self.int_values is not None else 0
    
        self.nb_sampling = self.int_values[1] if self.int_values is not None else 0
        self.max_pad = 0.0 
    
    nb_echos: Optional[int] = ValidatedCppManagedProperty("nb_echos")
    nb_sampling: Optional[int] = ValidatedCppManagedProperty("nb_sampling")
    max_pad: Optional[float] = ValidatedCppManagedProperty("max_pad")

    def clone(self):
        new_voxel = DetailedVoxel(self.center, self.voxel_size, self.int_values, self.float_values)
        new_voxel._cpp_object = self._cpp_object.clone()
        return new_voxel


class EnergyModel(Validatable):
    def __init__(self, scanning_device: 'ScanningDevice') -> None:
        self._cpp_object = _helios.EnergyModel(scanning_device._cpp_object)
        from pyhelios.scanner import ScanningDevice
        self.scanning_device = scanning_device


class BaseEnergyModel(EnergyModel):
    def __init__(self, scanning_device: 'ScanningDevice') -> None:
        super().__init__(scanning_device)
        self._cpp_object = _helios.BaseEnergyModel(scanning_device._cpp_object)


class Material(Validatable):
    def __init__(self, mat_file_path: Optional[str] = "", name: Optional[str] = "default", is_ground: Optional[bool] = False, 
                 use_vertex_colors: Optional[bool] = False, reflectance: Optional[float] = .0, specularity: Optional[float] = .0,
                 ambient_components: Optional[List[float]] = None, diffuse_components: Optional[List[float]] = None,
                 specular_components: Optional[List[float]] = None, map_kd: Optional[str] = "", spectra: Optional[str] = "") -> None:
        self._cpp_object = _helios.Material()
        self.name = name
        self.mat_file_path = mat_file_path
        self.is_ground = is_ground
        self.use_vertex_colors = use_vertex_colors
        self.reflectance = reflectance
        self.specularity = specularity
        self.ambient_components = ambient_components or [0.0, 0.0, 0.0, 0.0]
        self.diffuse_components = diffuse_components or [0.0, 0.0, 0.0, 0.0]
        self.specular_components = specular_components or [0.0, 0.0, 0.0, 0.0]
        self.classification = 0
        self.specular_exponent = 10
        self.map_kd = map_kd
        self.spectra = spectra
    
    name: Optional[str] = ValidatedCppManagedProperty("name")
    mat_file_path: Optional[str] = ValidatedCppManagedProperty("mat_file_path")
    is_ground: Optional[bool] = ValidatedCppManagedProperty("is_ground")
    use_vertex_colors: Optional[bool] = ValidatedCppManagedProperty("use_vertex_colors")
    reflectance: Optional[float] = ValidatedCppManagedProperty("reflectance")
    specularity: Optional[float] = ValidatedCppManagedProperty("specularity")
    ambient_components: Optional[List[float]] = ValidatedCppManagedProperty("ambient_components")
    diffuse_components: Optional[List[float]] = ValidatedCppManagedProperty("diffuse_components")
    specular_components: Optional[List[float]] = ValidatedCppManagedProperty("specular_components")
    classification: Optional[int] = ValidatedCppManagedProperty("classification")
    specular_exponent: Optional[float] = ValidatedCppManagedProperty("specular_exponent")
    map_kd: Optional[str] = ValidatedCppManagedProperty("map_kd")
    spectra: Optional[str] = ValidatedCppManagedProperty("spectra")

    def calculate_specularity(self,):
        ds_sum = sum(self.diffuse_components) + sum(self.specular_components)
        if ds_sum > 0:
            self.specularity = sum(self.specular_components) / ds_sum

    
    @classmethod
    def load_materials(cls, filePathString: str) -> Dict[str, 'Material']:
        newMats = {}
        is_first_material = True
       
        with open(filePathString, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line in ["\r", "\r\n", "\n"]:
                    continue

                lineParts = re.split(r'\s+', line)

                # Wavefront .mtl standard attributes
                if lineParts[0] == "newmtl" and len(lineParts) >= 2:
                    if not is_first_material:
                        newMats[newMat.name] = newMat
                    newMat = Material()
                    newMat.mat_file_path = filePathString
                    newMat.name = lineParts[1]
                    is_first_material = False
                elif lineParts[0] == "Ka" and len(lineParts) >= 4:
                    newMat.ambient_components = [float(x) for x in lineParts[1:4]] + [0.0]
                elif lineParts[0] == "Kd" and len(lineParts) >= 4:
                    newMat.diffuse_components = [float(x) for x in lineParts[1:4]] + [0.0]
                elif lineParts[0] == "Ks" and len(lineParts) >= 4:
                    newMat.specular_components = [float(x) for x in lineParts[1:4]] + [0.0]
                elif lineParts[0] == "Ns" and len(lineParts) >= 2:
                    newMat.specular_exponent = float(lineParts[1])
                elif lineParts[0] == "map_Kd" and len(lineParts) >= 2:
                    newMat.map_kd = lineParts[1]

                # HELIOS-specific additions
                elif lineParts[0] == "helios_reflectance" and len(lineParts) >= 2:
                    newMat.reflectance = float(lineParts[1])
                elif lineParts[0] == "helios_isGround" and len(lineParts) >= 2:
                    newMat.is_ground = lineParts[1].lower() in ["true", "1"]
                elif lineParts[0] == "helios_useVertexColors" and len(lineParts) >= 2:
                    newMat.use_vertex_colors = bool(int(lineParts[1]))
                elif lineParts[0] == "helios_classification" and len(lineParts) >= 2:
                    newMat.classification = int(lineParts[1])
                elif lineParts[0] == "helios_spectra" and len(lineParts) >= 2:
                    newMat.spectra = lineParts[1]

        newMat.calculate_specularity()
        newMat = cls._validate(newMat)
        newMats[newMat.name] = newMat
 
        return newMats

    @classmethod
    def parse_materials(cls, params: Dict[str, any]) -> List[Optional['Material']]:
        materials = []
        matfile = params.get("matfile")
        if not matfile:
            return

        mats = cls.load_materials(matfile)
        matname = params.get("matname")
        if matname and matname in mats:
            materials.append(mats[matname])
        elif mats:
            materials.append(next(iter(mats.values())))  # Pick the first material

        random_materials_count = params.get("randomMaterials")
        if random_materials_count:
            random_range = params.get("randomRange", 1.0)
            uns = UniformNoiseSource(0, random_range)

            base_material = materials[0] if materials else Material()  # Default base material
            for _ in range(random_materials_count):
                randomized_material = Material(name=base_material.name, **base_material.__dict__)
                # Apply random adjustments to reflectance and other properties
                randomized_material.reflectance += uns.next()
                randomized_material.reflectance = max(0.0, min(randomized_material.reflectance, 1.0))  # Clamp between 0 and 1
                randomized_material.calculate_specularity()
                materials.append(randomized_material)

        return materials
        

class Primitive(Validatable, ABC):
    def __init__(self,  material: Optional[Material] = None, aabb: Optional['AABB'] = None,
                 detailed_voxel: Optional[DetailedVoxel] = None, triangles: Optional[List['Triangle']] = None,
                 scene_part: Optional['ScenePart'] = None) -> None:
        self._cpp_object = _helios.Primitive()
        self.material = material
        if scene_part is not None:
            from pyhelios.scene import ScenePart
            self.scene_part = scene_part
        else:
            self.scene_part = None
    
    @property
    @abstractmethod
    def vertices(self) -> List[Vertex]:
        pass
    
    def rotate(self, r: Rotation) -> None:
        for vertex in self.vertices:
            vertex.position = r.apply_vector_rotation(vertex.position)
            vertex.normal = r.apply_vector_rotation(vertex.normal)
    
    def scale(self, factor: float) -> None:
        for vertex in self.vertices:
            vertex.position[0] *= factor
            vertex.position[1] *= factor
            vertex.position[2] *= factor
    
    def translate(self, shift: List[float]) -> None:
        for vertex in self.vertices:
            vertex.position[0] += shift[0]
            vertex.position[1] += shift[1]
            vertex.position[2] += shift[2]
            

class AABB(Primitive):
    def __init__(self) -> None:
        super().__init__()
        self._cpp_object = _helios.AABB()
        self._vertices: Optional[List[Vertex]] = None
        self.bounds: Optional[List[List[float]]] = None   

    @property
    def vertices(self) -> List[Vertex]:
        """Return the vertices for this AABB."""
        return self._vertices    
    

class Triangle(Primitive):
    def __init__(self, v0: Optional[Vertex], v1: Optional[Vertex], v2: Optional[Vertex]) -> None:
        super().__init__()
        self._cpp_object = _helios.Triangle(v0._cpp_object, v1._cpp_object, v2._cpp_object)
        self._vertices: List[Vertex] = [v0, v1, v2]
        self.face_normal_set = False
        self.face_normal: List[float] = [0,0,0]

    @property
    def vertices(self) -> List[Vertex]:
        """Return the vertices for this Triangle."""
        return self._vertices

    def get_face_normal(self):
        if not self.face_normal_set:
            self.update_face_normal()
            self.face_normal_set = True
        return self.face_normal

    def update_face_normal(self):
        # Compute the normal using the cross product of two edges of the triangle
        v0, v1, v2 = [np.array(v.position) for v in self.vertices]
        edge1 = v1 - v0
        edge2 = v2 - v0
        cross_prod = np.cross(edge1, edge2)
        if np.linalg.norm(cross_prod) > 1e-8:
            self.face_normal = cross_prod / np.linalg.norm(cross_prod)
        else:
            self.face_normal = [0, 0, 0]  # Normalize the normal

    def clone(self):
        new_triangle = Triangle(self.v0, self.v1, self.v2)
        new_triangle._cpp_object = self._cpp_object.clone()
        return new_triangle

class Trajectory(Validatable):
    def __init__(self, gps_time: Optional[float] = .0, position: Optional[List[float]] = None, roll: Optional[float] = .0, pitch: Optional[float] = .0, yaw: Optional[float] = .0) -> None:
        self._cpp_object = _helios.Trajectory()
        self.position = position or [0., 0., 0.]
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    
    gps_time: Optional[float] = ValidatedCppManagedProperty("gps_time")
    position: Optional[List[float]] = ValidatedCppManagedProperty("position")
    roll: Optional[float] = ValidatedCppManagedProperty("roll")
    pitch: Optional[float] = ValidatedCppManagedProperty("pitch")
    yaw: Optional[float] = ValidatedCppManagedProperty("yaw")

    def clone(self):
        return Trajectory(self.gps_time, self.position, self.roll, self.pitch, self.yaw) 

class TrajectorySettings(Validatable):
    def __init__(self, start_time: Optional[float] = np.finfo(float).min, end_time: Optional[float] = np.finfo(float).max, teleport_to_start: Optional[bool] = False) -> None:
        self._cpp_object = _helios.TrajectorySettings()
        self.start_time = start_time
        self.end_time = end_time
        self.teleport_to_start = teleport_to_start

    start_time: Optional[float] = ValidatedCppManagedProperty("start_time")
    end_time: Optional[float] = ValidatedCppManagedProperty("end_time")
    teleport_to_start: Optional[bool] = ValidatedCppManagedProperty("teleport_to_start")

    @classmethod
    def from_xml_node(cls, trajectory_settings_node: ET.Element) -> 'TrajectorySettings':
        if trajectory_settings_node is None:
            return cls._validate(cls())
        start_time = float(trajectory_settings_node.get('startTime', np.finfo(float).min))
        end_time = float(trajectory_settings_node.get('endTime', np.finfo(float).max))
        teleport_to_start = bool(trajectory_settings_node.get('teleportToStart', False))
        return cls._validate(cls(start_time=start_time, end_time=end_time, teleport_to_start=teleport_to_start))


class Measurement(Validatable):
    def __init__(self, hit_object_id: Optional[str] = "", position: Optional[List[float]] = None, beam_direction: Optional[List[float]] = None, 
                beam_origin: Optional[List[float]] = None, distance: Optional[float] = .0, intensity: Optional[float] = .0, 
                echo_width: Optional[float] = .0, return_number: Optional[int] = 0, pulse_return_number: Optional[int] = 0, 
                fullwave_index: Optional[int] = 0, classification: Optional[int] = 0, gps_time: Optional[float] = .0):
        
        self._cpp_object = _helios.Measurement()
        self.hit_object_id = hit_object_id
        self.position = position or [0., 0., 0.]
        self.beam_direction = beam_direction or [0., 0., 0.]
        self.beam_origin = beam_origin or [0., 0., 0.]
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

    def clone(self):
        return Measurement(self.hit_object_id, self.position, self.beam_direction, self.beam_origin, self.distance, self.intensity, self.echo_width, self.return_number, self.pulse_return_number, self.fullwave_index, self.classification, self.gps_time)


class FWFSettings(Validatable):
    def __init__(self, bin_size: Optional[float] = .25, beam_sample_quality: Optional[int] = 3,
                 pulse_length: Optional[float] = 4.0, max_fullwave_range: Optional[float] = 0.0,
                 aperture_diameter: Optional[float] = 0.15, win_size: Optional[float] = None) -> None:
        self._cpp_object = _helios.FWFSettings()
        self.bin_size = bin_size
        self.beam_sample_quality = beam_sample_quality
        self.pulse_length  = pulse_length 
        self.win_size = win_size if win_size is not None else pulse_length / 4
        self.max_fullwave_range = max_fullwave_range
        self.aperture_diameter = aperture_diameter
    
    bin_size: Optional[float] = ValidatedCppManagedProperty("bin_size")
    beam_sample_quality: Optional[int] = ValidatedCppManagedProperty("beam_sample_quality")
    pulse_length: Optional[float] = ValidatedCppManagedProperty("pulse_length")
    win_size: Optional[float] = ValidatedCppManagedProperty("win_size")
    max_fullwave_range: Optional[float] = ValidatedCppManagedProperty("max_fullwave_range")
    aperture_diameter: Optional[float] = ValidatedCppManagedProperty("aperture_diameter")

    @classmethod
    def from_xml_node(cls, node: ET.Element) -> 'FWFSettings':
        if node is None:
            return cls._validate(cls())
        bin_size = float(node.get('binSize_ns', 0.25))
        beam_sample_quality = int(node.get('beamSampleQuality', 3))
        max_fullwave_range = float(node.get('maxFullwaveRange_ns', 0.0))
        aperture_diameter = float(node.get('apertureDiameter_m', 0.15))
        win_size = float(node.get('winSize_ns', str(cls().pulse_length / 4)))
        return cls._validate(cls(bin_size=bin_size, beam_sample_quality=beam_sample_quality, max_fullwave_range=max_fullwave_range, aperture_diameter=aperture_diameter, win_size=win_size))
    
    def clone(self):
        return FWFSettings(self.bin_size, self.beam_sample_quality, self.pulse_length, self.max_fullwave_range, self.aperture_diameter, self.win_size)


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

    @classmethod
    def from_xml_node(cls, deflector_origin_node: ET.Element) -> 'AbstractBeamDeflector':
       
        optics_type = deflector_origin_node.get('optics')
        scan_freq_max = float(deflector_origin_node.get('scanFreqMax_Hz', 0.0))
        scan_freq_min = float(deflector_origin_node.get('scanFreqMin_Hz', 0.0))
        scan_angle_max = float(deflector_origin_node.get('scanAngleMax_deg', 0.0))
        
        if optics_type == "oscillating":
            scan_product = int(deflector_origin_node.get("scanProduct", 1000000))
            return OscillatingMirrorBeamDeflector(scan_angle_max, scan_freq_max, scan_freq_min, scan_product) 

        elif optics_type == "conic":
            return ConicBeamDeflector(scan_angle_max, scan_freq_max, scan_freq_min)

        elif optics_type == "line":
            num_fibers = int(deflector_origin_node.get("numFibers", 1))
            return FiberArrayBeamDeflector(scan_angle_max, scan_freq_max, scan_freq_min, num_fibers) 

        elif optics_type == "rotating":
            scan_angle_effective_max_deg = float(deflector_origin_node.get("scanAngleEffectiveMax_deg", 0.0))
            scan_angle_effective_max_rad = math.radians(scan_angle_effective_max_deg)
            return PolygonMirrorBeamDeflector(scan_angle_max, scan_freq_max, scan_freq_min, scan_angle_effective_max_rad) 

        elif optics_type == "risley":
            rotor_freq_1_hz = int(deflector_origin_node.get("rotorFreq1_Hz", 7294))
            rotor_freq_2_hz = int(deflector_origin_node.get("rotorFreq2_Hz", -4664))
            return RisleyBeamDeflector(scan_angle_max, rotor_freq_1_hz, rotor_freq_2_hz)
    
    def clone(self):
        new_deflector = AbstractBeamDeflector(self.scan_angle_max, self.scan_freq_max, self.scan_freq_min, self.scan_freq, self.scan_angle, self.vertical_angle_min, self.vertical_angle_max, self.current_beam_angle, self.angle_diff_rad)
        new_deflector._cpp_object = self._cpp_object.clone()
        return new_deflector
        

class OscillatingMirrorBeamDeflector(AbstractBeamDeflector):
    def __init__(self, scan_angle_max: float, scan_freq_max: float, scan_freq_min: float, scan_product: int) -> None:
        super().__init__()
        self._cpp_object = _helios.OscillatingMirrorBeamDeflector(scan_angle_max, scan_freq_max, scan_freq_min, scan_product)
        self.scan_product = scan_product
    
    def clone(self):
        new_deflector = OscillatingMirrorBeamDeflector(self.scan_angle_max, self.scan_freq_max, self.scan_freq_min, self.scan_product)
        new_deflector._cpp_object = self._cpp_object.clone()
        return new_deflector
    
    scan_product: Optional[int] = ValidatedCppManagedProperty("scan_product")


class ConicBeamDeflector(AbstractBeamDeflector):
    def __init__(self, scan_angle_max:float, scan_freq_max: float, scan_freq_min:float) -> None:
        super().__init__()
        self._cpp_object = _helios.ConicBeamDeflector(scan_angle_max, scan_freq_max, scan_freq_min)
        self.scan_angle_max = scan_angle_max
        self.scan_freq_max = scan_freq_max
        self.scan_freq_min = scan_freq_min

    def clone(self):
        new_deflector = ConicBeamDeflector(self.scan_angle_max, self.scan_freq_max, self.scan_freq_min)
        new_deflector._cpp_object = self._cpp_object.clone()
        return new_deflector


class FiberArrayBeamDeflector(AbstractBeamDeflector):
    def __init__(self, scan_angle_max: float, scan_freq_max: float, scan_freq_min: float, num_fibers: int) -> None:
        super().__init__()
        self._cpp_object = _helios.FiberArrayBeamDeflector(scan_angle_max, scan_freq_max, scan_freq_min, num_fibers)
        self.scan_angle_max = scan_angle_max
        self.scan_freq_max = scan_freq_max
        self.scan_freq_min = scan_freq_min
        self.num_fibers = num_fibers
    
    num_fibers: Optional[int] = ValidatedCppManagedProperty("num_fibers")

    def clone(self):
        new_deflector = FiberArrayBeamDeflector(self.scan_angle_max, self.scan_freq_max, self.scan_freq_min, self.num_fibers)
        new_deflector._cpp_object = self._cpp_object.clone()
        return new_deflector

 
class PolygonMirrorBeamDeflector(AbstractBeamDeflector):
    def __init__(self, scan_freq_max: float, scan_freq_min: float, scan_angle_max: float, scan_angle_effective_max: float) -> None:
        super().__init__()
        self._cpp_object = _helios.PolygonMirrorBeamDeflector(scan_freq_max, scan_freq_min, scan_angle_max, scan_angle_effective_max)
        self.scan_angle_max = scan_angle_max
        self.scan_freq_max = scan_freq_max
        self.scan_freq_min = scan_freq_min
        self.scan_angle_effective_max = scan_angle_effective_max
    
    def clone(self):
        new_deflector = PolygonMirrorBeamDeflector(self.scan_angle_max, self.scan_freq_max, self.scan_freq_min, self.scan_angle_effective_max)
        new_deflector._cpp_object = self._cpp_object.clone()
        return new_deflector


class RisleyBeamDeflector(AbstractBeamDeflector):
    def __init__(self, scan_angle_max: float, rotor_speed_rad_1: float, rotor_speed_rad_2: float) -> None:
        super().__init__()
        self._cpp_object = _helios.RisleyBeamDeflector(scan_angle_max, rotor_speed_rad_1, rotor_speed_rad_2)
        self.rotor_speed_rad_1 = rotor_speed_rad_1
        self.rotor_speed_rad_2 = rotor_speed_rad_2

    rotor_freq_1: Optional[float] = ValidatedCppManagedProperty("rotor_speed_rad_1")
    rotor_freq_2: Optional[float] = ValidatedCppManagedProperty("rotor_speed_rad_2")

    def clone(self):
        new_deflector = RisleyBeamDeflector(self.scan_angle_max, self.rotor_speed_rad_1, self.rotor_speed_rad_2)
        new_deflector._cpp_object = self._cpp_object.clone()
        return new_deflector


class NoiseSource(Validatable):
    def __init__(self, 
                 clip_min: Optional[float] = 0.0, 
                 clip_max: Optional[float] = 1.0, 
                 clip_enabled: Optional[bool] = False, 
                 fixed_lifespan: Optional[float] = None, 
                 fixed_value_remaining_uses: Optional[int] = None):
     
        self._cpp_object = _helios.NoiseSource()
        self.clip_min = clip_min
        self.clip_max = clip_max
        self.clip_enabled = clip_enabled
        self.fixed_lifespan = fixed_lifespan
        self.fixed_value_remaining_uses = fixed_value_remaining_uses

    clip_min: Optional[float] = ValidatedCppManagedProperty("clip_min")
    clip_max: Optional[float] = ValidatedCppManagedProperty("clip_max")
    clip_enabled: Optional[bool] = ValidatedCppManagedProperty("clip_enabled")
    fixed_lifespan: Optional[float] = ValidatedCppManagedProperty("fixed_lifespan")
    fixed_value_remaining_uses: Optional[int] = ValidatedCppManagedProperty("fixed_value_remaining_uses")

    @property
    def fixed_value_enabled(self) -> bool:
        """
        Read-only property that checks if fixed value is enabled.
        This binds to the C++ isFixedValueEnabled method.
        """
        return self._cpp_object.fixed_value_enabled

    def next(self) -> float:
        """
        Implement the logic for generating the next noise value.
        Mirrors the logic from the C++ `NoiseSource::next()`.
        """
        if self.fixed_lifespan == 0:
            return self.fixed_value  # Return fixed value immediately if lifespan is 0

        if self.fixed_lifespan > 0 and self.fixed_value_remaining_uses > 0:
            self.fixed_value_remaining_uses -= 1
            return self.fixed_value  # Decrement remaining uses and return fixed value

        # Otherwise, generate a new noise value
        self.fixed_value = self.clip(self.noise_function())
        
        # If fixed lifespan is greater than 1, update remaining uses
        if self.fixed_lifespan and self.fixed_lifespan > 1:
            self.fixed_value_remaining_uses = self.fixed_lifespan - 1
        
        return self.fixed_value
    
    def clip(self, value: float) -> float:
        """
        Clips the noise value to be within the min and max bounds (clip_min and clip_max) if clipping is enabled.
        This mirrors the C++ `NoiseSource::clip()` function.
        """
        if self.clip_enabled:
            value = max(self.clip_min, min(value, self.clip_max))
        
        return value
    

class RandomNoiseSource(NoiseSource):
    """Python class representing RandomNoiseSource, inheriting from NoiseSource."""

    # Constructor for initializing the Python class and the corresponding C++ object
    def __init__(self, seed: Optional[str] = None):
        # Initialize the C++ RandomNoiseSource object, either with a seed or the default constructor
        if seed is not None:
            self._cpp_object = _helios.RandomNoiseSourceDouble(seed)
        else:
            self._cpp_object = _helios.RandomNoiseSourceDouble()

        # Initialize properties for validation
        self.clip_min = self._cpp_object.clip_min
        self.clip_max = self._cpp_object.clip_max
        self.clip_enabled = self._cpp_object.clip_enabled
        self.fixed_lifespan = self._cpp_object.fixed_lifespan
        self.fixed_value_remaining_uses = self._cpp_object.fixed_value_remaining_uses

    # Properties with validation that link to C++ object
    clip_min: Optional[float] = ValidatedCppManagedProperty("clip_min")
    clip_max: Optional[float] = ValidatedCppManagedProperty("clip_max")
    clip_enabled: Optional[bool] = ValidatedCppManagedProperty("clip_enabled")
    fixed_lifespan: Optional[int] = ValidatedCppManagedProperty("fixed_lifespan")
    fixed_value_remaining_uses: Optional[int] = ValidatedCppManagedProperty("fixed_value_remaining_uses")


class UniformNoiseSource(RandomNoiseSource):
    """Python class representing UniformNoiseSource, inheriting from RandomNoiseSource."""

    def __init__(self, seed: Optional[str] = None, min: Optional[float] = 0.0, max: Optional[float] = 1.0):
        """Initialize the Python object, calling the appropriate C++ constructor."""
        if seed is not None:
            self._cpp_object = _helios.UniformNoiseSource(seed, min, max)
        else:
            self._cpp_object = _helios.UniformNoiseSource(min, max)

        # Initialize properties for validation
        self.min = self._cpp_object.min
        self.max = self._cpp_object.max

        # Initialize randomness generator (in Python)
        self.randomness_generator = RandomnessGenerator(seed=seed) 
        self.randomness_generator.compute_uniform_real_distribution(min, max)

    # Properties with validation that link to the C++ object
    min: Optional[float] = ValidatedCppManagedProperty("min")
    max: Optional[float] = ValidatedCppManagedProperty("max")

    def noise_function(self) -> float:
        """Generate the next value in the uniform distribution."""
        return self.randomness_generator.uniform_real_distribution_next()
    
    def configure_uniform_noise(self, min_value: float, max_value: float):
        """Configure the uniform noise range in the C++ object."""
        if min_value >= max_value:
            raise ValueError("min_value must be less than max_value.")
        
        self._cpp_object.configure_uniform_noise(min_value, max_value)
        self.randomness_generator.compute_uniform_real_distribution(min_value, max_value)


class SwapOnRepeatHandler(Validatable):
        def __init__(self, baseline: Optional['ScenePart'] = None, keep_crs: Optional[bool] = False, discard_on_replay: Optional[bool] = False) -> None:
            self._cpp_object = _helios.SwapOnRepeatHandler()
            self.baseline = baseline
            self.keep_crs = keep_crs
            self.times_to_live: deque[int] = deque()
            self.discard_on_replay = discard_on_replay
            self.num_target_swaps: int = 0
            self.num_current_swaps: int = 0
        
        baseline: Optional['ScenePart'] = ValidatedCppManagedProperty("baseline")
        keep_crs: Optional[bool] = ValidatedCppManagedProperty("keep_crs")
        discard_on_replay: Optional[bool] = ValidatedCppManagedProperty("discard_on_replay")

        def push_time_to_live(self, time_to_live: int):
            self.times_to_live.append(time_to_live)
            self._cpp_object.push_time_to_live(time_to_live)

        def prepare(self, sp: 'ScenePart', swap_filters: deque):
            # Calculate num_target_swaps based on the length of swap_filters
            self.num_target_swaps = len(self._cpp_object.swap_filters)
            
            # Calculate num_target_replays by summing up all time-to-live values
            self.num_target_replays = sum(self.times_to_live)
            
            # Set num_current_swaps to zero
            self.num_current_swaps = 0

            # Clone ScenePart object for baseline and set its primitives' parts to None
            self.baseline = sp.clone()
            for primitive in self.baseline.primitives:
                primitive.part = None


class KDTreeFactoryMaker(BaseModel):
    loss_nodes_default: int = Field(21, description="Default loss nodes for SAH KDTree factories")

    @staticmethod
    def make_simple_kd_tree():
        return _helios.SimpleKDTreeFactory()

    @staticmethod
    def make_multithreaded_simple_kd_tree(node_jobs: int, geom_jobs: int):
        kdtree_factory = _helios.SimpleKDTreeFactory()
        geom_strategy = _helios.SimpleKDTreeGeometricStrategy()
        return _helios.MultiThreadKDTreeFactory(kdtree_factory, geom_strategy, node_jobs, geom_jobs)

    @staticmethod
    def make_sah_kd_tree_factory(loss_nodes: Optional[int] = None):
        loss_nodes = loss_nodes or KDTreeFactoryMaker().loss_nodes_default
        return _helios.SAHKDTreeFactory(loss_nodes)

    @staticmethod
    def make_multithreaded_sah_kd_tree_factory(node_jobs: int, geom_jobs: int, loss_nodes: Optional[int] = None):
        loss_nodes = loss_nodes or KDTreeFactoryMaker().loss_nodes_default
        kdtree_factory = _helios.SAHKDTreeFactory(loss_nodes)
        geom_strategy = _helios.SAHKDTreeGeometricStrategy(kdtree_factory)
        return _helios.MultiThreadKDTreeFactory(kdtree_factory, geom_strategy, node_jobs, geom_jobs)

    @staticmethod
    def make_axis_sah_kd_tree_factory(loss_nodes: Optional[int] = None):
        loss_nodes = loss_nodes or KDTreeFactoryMaker().loss_nodes_default
        return _helios.AxisSAHKDTreeFactory(loss_nodes)

    @staticmethod
    def make_multithreaded_axis_sah_kd_tree_factory(node_jobs: int, geom_jobs: int, loss_nodes: Optional[int] = None):
        loss_nodes = loss_nodes or KDTreeFactoryMaker().loss_nodes_default
        kdtree_factory = _helios.AxisSAHKDTreeFactory(loss_nodes)
        geom_strategy = _helios.AxisSAHKDTreeGeometricStrategy(kdtree_factory)
        return _helios.MultiThreadKDTreeFactory(kdtree_factory, geom_strategy, node_jobs, geom_jobs)

    @staticmethod
    def make_fast_sah_kd_tree_factory(loss_nodes: Optional[int] = 32):
        return _helios.FastSAHKDTreeFactory(loss_nodes)

    @staticmethod
    def make_multithreaded_fast_sah_kd_tree_factory(node_jobs: int, geom_jobs: int, loss_nodes: Optional[int] = 32):
        kdtree_factory = _helios.FastSAHKDTreeFactory(loss_nodes)
        geom_strategy = _helios.FastSAHKDTreeGeometricStrategy(kdtree_factory)
        return _helios.MultiThreadSAHKDTreeFactory(kdtree_factory, geom_strategy, node_jobs, geom_jobs)
    

class SimulationCycleCallback:
    def __init__(self) -> None:
        self._cpp_object = _helios.SimulationCycleCallback()
        self.is_callback_in_progress = False  # Flag to prevent recursion

    def __call__(self, measurements: List[Measurement], trajectories: List[Trajectory], outpath: str):
        # Prevent recursion
        if self.is_callback_in_progress:
            return
        self.is_callback_in_progress = True  # Set flag to prevent recursion
        try:
            self.measurements = measurements
            self.trajectories = trajectories
            self.output_path = outpath
        finally:
            self.is_callback_in_progress = False 
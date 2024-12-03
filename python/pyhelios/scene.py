from pyhelios.utils import Validatable, ValidatedCppManagedProperty, AssetManager
from pyhelios.primitives import Rotation, Primitive, AABB, Vertex, Triangle, Material, SwapOnRepeatHandler, PrimitiveType, KDTreeFactoryMaker

from pydantic import Field, ConfigDict
from osgeo import gdal
from osgeo import osr
from osgeo import ogr
import os
import re
import threading
import numpy as np
from typing import Optional, List, Tuple, Annotated, Type, Any, Dict
import xml.etree.ElementTree as ET
import sys
from collections import defaultdict, deque
from numpy import array, zeros
from numpy.linalg import norm

import _helios

gdal.DontUseExceptions()

class ScenePart(Validatable):
    def __init__(self, 
                 id: Optional[str] = "", 
                 origin: Optional[List[float]] = None, 
                 rotation: Optional[Rotation] = None, 
                 scale: Optional[float] = 1.0, 
                 bound: Optional[List[float]] = None,
                 primitives: Optional[List[Primitive]] = None,
                 translation: Optional[List[float]] = None,
                 rotation_method: Optional[str] = 'global') -> None:  
              
        self._cpp_object = _helios.ScenePart()
        self.id = id
        self.origin = origin or [0.0, 0.0, 0.0]
        self.rotation = rotation or Rotation(1,0,0,0)
        self.scale = scale
        self.bound = bound
        self.primitives = primitives or []
        self.translation = translation or [0.0, 0.0, 0.0]
        self.rotation_method = rotation_method
        self.env = None
        self.sorh = None
        self.is_force_on_ground = 0
        self.primitive_type: PrimitiveType = Field(default=PrimitiveType.NONE)

    id: Optional[str] = ValidatedCppManagedProperty("id")
    origin: Optional[Tuple[float, float, float]] = ValidatedCppManagedProperty("origin")
    rotation: Optional[Rotation] = ValidatedCppManagedProperty("rotation")
    scale: Optional[float] = ValidatedCppManagedProperty("scale")
    bound: Optional[Tuple[float, float, float]] = ValidatedCppManagedProperty("bound")
    primitives: Optional[List[Primitive]] = ValidatedCppManagedProperty("primitives")
    is_force_on_ground: Optional[int] = ValidatedCppManagedProperty("is_force_on_ground")

    @classmethod
    def read_from_xml(cls: Type['ScenePart'], xml: str) -> 'ScenePart':
        root = ET.fromstring(xml)
        scene_part = cls()

        part_element = root.find("part")
        if part_element is not None:
            filter_element = part_element.find("filter")
            if filter_element is not None:
                filter_type = filter_element.attrib.get("type")
                if filter_type == 'scale':
                    scene_part.scale = float(filter_element.attrib.get("value"))
                elif filter_type == 'translate':
                    scene_part.translation = [float(v) for v in filter_element.attrib.get("value").split(";")]
                elif filter_type == 'rotate':
                    scene_part._parse_rotation(filter_element)
                elif filter_type == 'geotiffloader':
                    scene_part._load_tiff(xml)
        scene_part = cls._validate(scene_part)

        return scene_part
    
    def _create_params(self, filter_type: ET.Element) -> dict:
        params = {}
        for param_elem in filter_type.findall('param'):
            param_type = param_elem.get("type", "string")
            key = param_elem.get("key")
            value_str = param_elem.get("value")

            if param_type == "string":
                params[key] = value_str
            elif param_type in ["boolean", "bool"]:
                params[key] = value_str.lower() == "true"
            elif param_type == "double":
                try:
                    params[key] = float(value_str)
                except ValueError:
                    raise ValueError(f"Invalid double value for key '{key}': {value_str}")
            elif param_type in ["integer", "int"]:
                try:
                    params[key] = int(value_str)
                except ValueError:
                    raise ValueError(f"Invalid integer value for key '{key}': {value_str}")
            elif param_type == "vec3":
                try:
                    vec = value_str.split(";")
                    if len(vec) != 3:
                        raise ValueError(f"Invalid vec3 value for key '{key}': {value_str}")
                    params[key] = [float(vec[0]), float(vec[1]), float(vec[2])]
                except (ValueError, IndexError):
                    raise ValueError(f"Invalid vec3 format for key '{key}': {value_str}")
            elif param_type == "rotation":
                params[key] = Rotation.from_xml_node(param_elem)
            else:
                raise ValueError(f"Unknown parameter type '{param_type}' for key '{key}'")
      
        return params
    
    def _apply_filter(self, filter_type: str, params: dict) -> None:
        if filter_type == "scale":
            self.scale_loader(params)
        elif filter_type == "translate":
            self.translate_loader(params)
        elif filter_type == "rotate":
            self.rotate_loader(params)
        if filter_type == "geotiffloader":
            self._geotiff_loader(params)
        elif filter_type == "objloader":
            self._obj_loader(params)
        #TODO: implement xyzloader and detailed_voxels_loader
    
    def load_scene_filter(self, filter_element: ET.Element) -> None:
        for filter_elem in filter_element.findall('filter'):
            filter_type = filter_elem.attrib.get('type')
            params = self._create_params(filter_elem)
            
            self._apply_filter(filter_type, params)
            if filter_type in ["geotiffloader", "detailed_voxels_loader", "xyzloader", "objloader"] and filter_element.find('swap') is not None:
                if self.sorh is not None:
                    raise ValueError("Multiple swap filters are not supported.")
                    
                self.sorh = self.load_scene_part_swaps(filter_elem, params)

    def load_scene_part_swaps(self, filter_elem: ET.Element, params: dict) -> Optional[SwapOnRepeatHandler]:
        swap_nodes = filter_elem.findall("swap")
        if not swap_nodes:
            return None
        
        sorh = SwapOnRepeatHandler()

        for swap_node in swap_nodes:
            # Get swapStep and keepCRS attributes
            swap_step = int(swap_node.get("swapStep", 1))
            keep_crs = bool(swap_node.get("keepCRS", sorh.keep_crs))
            
            sorh.push_time_to_live(swap_step)
            sorh.keep_crs = keep_crs
            
            # Prepare to collect filters for the swap
            swap_filters = deque()
            
            # Check for null geometry filter
            if swap_node.get("force_null", "false") == "true":
                swap_filters.append("null")
            else:
                # Load filters if they exist
                for filter_node in swap_node.findall("filter"):
                    filter_type = filter_node.attrib.get('type')
                    params = self._create_params(filter_elem)
    
                    self._apply_filter(filter_type, params)
                    swap_filters.append(params)
            
            # Add the swap filters to the handler
            sorh.swap_filters.append(swap_filters)
        
        # Prepare the SwapOnRepeatHandler with the scene part
        sorh.prepare(self, swap_filters)
        return sorh
    
    def validate_scene_part(self, part_elem: ET.Element) -> bool:
        # Check if the scene part is valid
        if self.primitive_type == "NONE" and not self.primitives:
            path = "#NULL#"
            path_type = "path"
            found_path = False
            for filter_elem in part_elem.findall('filter'):
                for param_elem in filter_elem.findall('param'):
                    if 'key' in param_elem.attrib:
                        key = param_elem.attrib['key']
                        if key in ["filepath", "efilepath"]:
                            path = param_elem.attrib.get('value', '#NULL#')
                            path_type = "extended path expression" if key == "efilepath" else "path"
                            found_path = True
                            break
                if found_path:
                    break

            # Log a warning or error for invalid scene parts
            print(f"XmlSceneLoader::validate_scene_part detected an invalid scene part with id \"{self.id}\".\n"
                  f"The {path_type} \"{path}\" was given.\n"
                  "It leads to the loading of an invalid scene part, which is automatically ignored.")
            return False
        return True


    def load_scene_part_id(self, part_elem: ET.Element, idx: int) -> bool:
        """
        This method loads the `id` attribute of a scene part from an XML element.
        If no ID is present, it generates an ID from the index.
        
        Parameters:
            part_elem (ET.Element): The XML element representing the scene part.
            idx (int): The index of the part in the scene.

        Returns:
            bool: Returns True if the part should be split, False otherwise.
        """
        part_id = ""
        split_part = True

        # Try to find the 'id' attribute
        part_id_attr = part_elem.attrib.get("id")

        if part_id_attr is not None:
            part_id = part_id_attr
            try:
                # Try to convert the ID to an integer (to simulate checking if it's numerical)
                int(part_id)
            except ValueError:
                # If it's not numerical, log a warning and proceed
                print(f'Warning: Scene part id "{part_id}" is non-numerical. '
                      'This is not compatible with LAS format specification.')
        
        # If no id was found, assign the partIndex as the id
        if not part_id:
            self.id = str(idx)
        else:
            self.id = part_id
            split_part = False

        return split_part

    def scale_loader(self, params: dict) -> None:
        for key, value in params.items():
            if key == "scale":
                self.scale = float(value)

    def translate_loader(self, params: dict) -> None:
        for key, value in params.items():
            if key == "offset":
                self.origin = value
            if key == "onGround":
                self.is_force_on_ground = int(value)

    def rotate_loader(self, params: dict) -> None:
        for key, value in params.items():
            if key == "rotation":
                rotation = value
                self.rotation = rotation.apply_rotation(self.rotation)
               
    def _parse_rotation(self, filter_element: ET.Element) -> None:
        """Helper method to parse rotation data from XML."""
        self.rotation_method = filter_element.attrib.get('rotations', 'global')
        rotation_param = filter_element.find('param')
        if rotation_param is not None and rotation_param.attrib.get('type') == 'rotation':
            self.rotation = [
                (rot.attrib.get('axis', ''), rot.attrib.get('angle_deg', ''))
                for rot in rotation_param.findall('rot')
            ]
    
    def _obj_loader(self, params: dict) -> None:
        y_is_up = self._determine_up_axis(params)
        for key, value in params.items():
            
            if 'filepath' in key:
                file_path = AssetManager().find_file_by_name(value, auto_add=False)
                self._load_obj(file_path, y_is_up)       
    
    def _load_obj(self, file_path: str, y_is_up: bool) -> None:
        with open(file_path, 'r') as file:
            primitives = []
            vertices = []
            normals = []
            tex_coords = []
            materials = {}
            current_mat_type = "default"
            mat = Material()
            mat.use_vertex_colors = True
            mat.mat_file_path = str(file_path)
            if current_mat_type not in materials:
                materials[current_mat_type] = [] 
            materials[current_mat_type].append(mat)

            for line in file:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    line_parts = re.split(r"\s+", line)
                    if line_parts[0] == "v":
                        vertices.append(self._read_obj_vertex(line_parts, y_is_up))
                        
                    elif line_parts[0] == "vn":
                        normals.append(self._read_obj_normal_vector(line_parts, y_is_up))
                    elif line_parts[0] == "vt":
                        tex_coords.append([float(line_parts[1]), float(line_parts[2])])
                    elif line_parts[0] == "f":

                        # We need to check if the name that is in currentmat is in the materials dict, the we need to save to variable the value from the dict. If not we create Material instance set name to currentMat and insert this pair to dict
                        if current_mat in materials.keys():
                            mat = materials[current_mat]
                        else:
                            mat = Material()
                            mat.name = current_mat
                            materials.update({current_mat_type: [mat]})
                        self._read_obj_primitive(primitives, line_parts, vertices, tex_coords, normals, mat, file_path)
                    elif line_parts[0] == "usemtl":
                        current_mat = line_parts[1]
                       
                    elif line_parts[0] == "mtllib":
                        file_path_string = os.path.join(os.path.dirname(file_path), line_parts[1])
                        new_materials = Material.load_materials(file_path_string)
                        materials.update(new_materials)
            self.primitives.extend(primitives)
                  
    def _determine_up_axis(self, params: dict) -> bool:
        up_axis = params.get("up", "z")
        if up_axis == "y":
            return True
        elif up_axis != "z":
            raise RuntimeWarning(f"Invalid up axis value: {up_axis}. Defaulting to z-axis.")
        return False

    def _read_obj_vertex(self, line_parts: List[str], y_is_up: bool) -> Vertex:
        if y_is_up:
            position = [float(line_parts[1]), -float(line_parts[3]), float(line_parts[2])]
        else:
            position = [float(line_parts[1]), float(line_parts[2]), float(line_parts[3])]
        
        return Vertex(position=position)
                    
    def _read_obj_normal_vector(self, line_parts: List[str], y_is_up: bool):
        if y_is_up:
            return [float(line_parts[1]), -float(line_parts[3]), float(line_parts[2])]
        else:
            return [float(line_parts[1]), float(line_parts[2]), float(line_parts[3])]

    def _read_obj_primitive(self, primitives: List[Primitive], line_parts: List[str], vertices: List[Vertex],
                      texcoords: List[List[float]], normals: List[List[float]], current_mat: Material, mat_file_path: str) -> None:
        verts = []

        for part in line_parts[1:]:
            fields = part.split('/')
            vi = int(fields[0]) - 1
            ti = int(fields[1]) - 1 if len(fields) > 1 and fields[1] else -1
            ni = int(fields[2]) - 1 if len(fields) > 2 and fields[2] else -1
            vert = self._build_primitive_vertex(vertices[vi], vertices[vi], ti, ni, texcoords, normals)
            verts.append(vert)

        if len(verts) == 3:
            tri = Triangle(v0=verts[0], v1=verts[1], v2=verts[2])
            tri.material = current_mat
            primitives.append(tri)
          
        elif len(verts) == 4:
            tri1 = Triangle(v0=verts[0], v1=verts[1], v2=verts[2])
            tri2 = Triangle(v0=verts[0], v1=verts[2], v2=verts[3])
            tri1.material = current_mat
            tri2.material = current_mat
            primitives.append(tri1)
            primitives.append(tri2)
        
    def _build_primitive_vertex(self, dst_vert: Vertex, src_vert: Vertex, tex_idx: int, normal_idx: int,
                         texcoords: List[List[float]], normals: List[List[float]]):
        dst_vert = src_vert.clone() 
        if tex_idx >= 0:
            dst_vert.tex_coords = texcoords[tex_idx]
        if normal_idx >= 0:
            dst_vert.normal = normals[normal_idx]
        return dst_vert

    def _geotiff_loader(self, params: dict) -> None:
        """Load TIFF file and generate a 3D mesh using GDAL and Open3D."""

        file_path = AssetManager().find_file_by_name(params["filepath"], auto_add=False)
        if file_path is None:
            raise FileNotFoundError(f"No filepath was provided for the GeoTIFF file.")
        
        tiff_dataset = gdal.Open(file_path, gdal.GA_ReadOnly)
        if tiff_dataset is None:
            raise RuntimeError(f"Failed to open GeoTIFF file at {file_path}")
        
        coord_ref_sys = tiff_dataset.GetSpatialRef()
        if coord_ref_sys is not None:
            source_crs = coord_ref_sys.Clone()
        else:
            source_crs = osr.SpatialReference()
        
        raster_band = tiff_dataset.GetRasterBand(1)  # Get the first raster band (band 1)
        if raster_band is None:
            raise RuntimeError("Failed to obtain raster band from TIFF file.")
        
        # Get the raster dimensions (width and height)
        raster_width = raster_band.XSize
        raster_height = raster_band.YSize

        # envelope data
        self.env = {"MinX": 0, "MaxX": 0, "MinY": 0, "MaxY": 0}
        layer = tiff_dataset.GetLayer(0)  # Assuming only one layer
        geom = None
        if layer is not None:
            # Get spatial filter if available
            geom = layer.GetSpatialFilter()

        if geom is not None:
            # Envelope from geometry
            env = geom.GetEnvelope()
            self.env["MinX"], self.env["MaxX"], self.env["MinY"], self.env["MaxY"] = env
        elif layer is not None:
            # Envelope from layer extent
            env = ogr.Envelope()
            if layer.GetExtent(env, True) == ogr.OGRERR_NONE:
                self.env["MinX"], self.env["MaxX"], self.env["MinY"], self.env["MaxY"] = env.MinX, env.MaxX, env.MinY, env.MaxY
            else:
                raise RuntimeError("Failed to retrieve envelope from layer.")
        else:
            # Envelope from GeoTransform (fallback)
            transform = tiff_dataset.GetGeoTransform()
            if transform:
                if transform[1] > 0:
                    self.env["MinX"] = transform[0]
                    self.env["MaxX"] = self.env["MinX"] + transform[1] * raster_width
                else:
                    self.env["MaxX"] = transform[0]
                    self.env["MinX"] = self.env["MaxX"] + transform[1] * raster_width
                
                if transform[5] < 0:  # Negative value for pixel height
                    self.env["MaxY"] = transform[3]
                    self.env["MinY"] = self.env["MaxY"] + transform[5] * raster_height
                else:
                    self.env["MinY"] = transform[3]
                    self.env["MaxY"] = self.env["MinY"] + transform[5] * raster_height

            else:
                raise RuntimeError("No valid GeoTransform found.")

        width = self.env["MaxX"] - self.env["MinX"]
        height = self.env["MaxY"] - self.env["MinY"]

        # Calculate pixel size
        pixel_width = width / raster_width
        pixel_height = height / raster_height

        #fill Vertices
        # Get no-data value from the raster
        nodata = raster_band.GetNoDataValue()
        if nodata is None:
            nodata = float('nan')
        
        half_pixel_width = pixel_width / 2.0
        half_pixel_height = pixel_height / 2.0
        eps = 1e-6
        vertices = [[None for _ in range(raster_height)] for _ in range(raster_width)]
        for x in range(raster_width):
            for y in range(raster_height):
                z = np.array([0.0], dtype=np.float32)  # Buffer to store the pixel value
                err = raster_band.ReadRaster(
                    xoff=x,
                    yoff=y,
                    xsize=1,
                    ysize=1,
                    buf_xsize=1,
                    buf_ysize=1,
                    buf_type=gdal.GDT_Float32,
                    buf_obj=z
                )

                if err is None:
                    continue

                z_val = z[0]
                if np.abs(z_val - nodata) < eps:
                    v = None  # No data, so no vertex
                else:
                    # Create the vertex with 3D position and texture coordinates
                    v = Vertex()
                    v.position = [
                        self.env["MinX"]+ x * pixel_width + half_pixel_width,
                        self.env["MinY"] - y * pixel_height - half_pixel_height + height,
                        z_val
                    ]
                    v.tex_coords = [
                        x / raster_width,
                        (raster_height - y) / raster_height
                    ]
                # Store the vertex in the grid
                vertices[x][y] = v

        # Build triangles from the vertices
        for x in range(raster_width - 1):
            for y in range(raster_height - 1):
                # Get the four vertices that form the two triangles in this square
                vert0 = vertices[x][y]
                vert1 = vertices[x][y + 1]
                vert2 = vertices[x + 1][y + 1]
                vert3 = vertices[x + 1][y]

                # Create the first triangle (vert0, vert1, vert3) if all vertices are valid
                if vert0 is not None and vert1 is not None and vert3 is not None:
                    tri1 = Triangle(vert0, vert1, vert3)
                    self.primitives.append(tri1)

                # Create the second triangle (vert1, vert2, vert3) if all vertices are valid
                if vert1 is not None and vert2 is not None and vert3 is not None:
                    tri2 = Triangle(vert1, vert2, vert3)
                    self.primitives.append(tri2)

        # parse Material
        materials = []
        materials = Material.parse_materials(params)
       
        mat_file_path = params.get("matfile", "")
        mat = Material(mat_file_path=mat_file_path, name="default") if not materials else materials[0]
        for prim in self.primitives:
            prim.material = mat
        self.smooth_vertex_normals()

    def compute_transform(self, holistic: Optional[bool] = False) -> None:
        for primitive in self.primitives:
            primitive.scene_part = self
            primitive.rotate(self.rotation)
    
            # If holistic is True, scale the vertices of the primitive
            if holistic:
                for vertex in primitive.vertices:
                    vertex.position[0] *= self.scale
                    vertex.position[1] *= self.scale
                    vertex.position[2] *= self.scale
            
            # Scale the primitive
            primitive.scale(self.scale)
            # Translate the primitive
            primitive.translate(self.origin)
            
    def smooth_vertex_normals(self):
        # Dictionary to store the list of triangles for each vertex
        vt_map = defaultdict(list)

        for primitive in self.primitives:
            if isinstance(primitive, Triangle):  # Check if primitive is a Triangle
                for vert in primitive.vertices:  # Use the vertices property
                    vt_map[vert].append(primitive)

        for vertex, triangles in vt_map.items():
            vertex.normal = np.zeros(3)  # Initialize vertex normal as a zero vector
            for triangle in triangles:
                vertex.normal += triangle.get_face_normal()  # Accumulate face normals
            if len(triangles) > 0:
                vertex.normal /= np.linalg.norm(vertex.normal) 


class Scene(Validatable):
    model_config = ConfigDict(arbitrary_types_allowed=True)
    def __init__(self, 
            scene_parts: Optional[List[ScenePart]], bbox: Optional[AABB] = None, bbox_crs: Optional[AABB] = None) -> None:
        
        self._cpp_object = _helios.Scene()
        self._scene_parts = scene_parts
        self.bbox = bbox
        self.bbox_crs = bbox_crs

        self._kd_grove_factory: _helios.KDGroveFactory
        self.kd_grove: Optional[_helios.KDGrove]
        self.kd_factory_type: int = 1
        self.kdt_num_jobs: int = 1
        self.kdt_geom_jobs: int = 1
        self.kdt_sah_loss_nodes: int = 21
        self.primitives: Optional[List[Primitive]] = []

    @property
    def scene_parts(self) -> Optional[List[ScenePart]]:
        return self._scene_parts

    @property
    def kd_grove_factory(self) -> Optional[_helios.KDGroveFactory]:
        return self._kd_grove_factory

    @kd_grove_factory.setter
    def kd_grove_factory(self, value: Optional[_helios.KDGroveFactory]) -> None:
        self._kd_grove_factory = value
        self._cpp_object.kd_grove_factory = value

    def add_scene_part(self, scene_part: ScenePart) -> None:
        if scene_part is not None:
            self._scene_parts.append(scene_part)

    scene_parts: Optional[List[ScenePart]] = ValidatedCppManagedProperty("scene_parts")
    kd_grove_factory: Optional[_helios.KDGroveFactory] = ValidatedCppManagedProperty("kd_grove_factory")
    bbox: Optional[AABB] = ValidatedCppManagedProperty("bbox")
    bbox_crs: Optional[AABB] = ValidatedCppManagedProperty("bbox_crs")
    primitives: Optional[List[Primitive]] = ValidatedCppManagedProperty("primitives")
    
    def get_scene_parts_size(self) -> int:
        return len(self._scene_parts)
    
    @classmethod
    def read_from_xml(cls: Type['Scene'], filename: str, id: Optional[str] = '', kd_factory_type: Optional[int] = 1, kdt_num_jobs: Optional[int] = 1, kdt_geom_jobs: Optional[int] = 1,  kdt_sah_loss_nodes: Optional[int] = 21) -> 'Scene':

        file_path = AssetManager.find_file_by_name(filename, auto_add=True)
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        scene_element = root.find(f".//scene[@id='{id}']")
        
        if scene_element is None:
            raise ValueError(f"No scanner found with id: {id}")
        
        is_dyn_scene = False
        holistic = False #TODO Implement holistis for xyzloader
        scene_parts = []
        scene = StaticScene._validate(StaticScene())
        for idx, part_elem in enumerate(scene_element.findall('part')):
            scene_part = ScenePart()
            scene_part.load_scene_filter(part_elem)
            is_split_part = scene_part.load_scene_part_id(part_elem, idx)
            if not scene_part.validate_scene_part(part_elem):
                # If invalid, skip to the next element
                continue
            
            #TODO Implement reading of dynamic scene parts
            #TODO: scene loading specification
            scene._digest_scene_part(scene_part, idx, holistic, is_split_part, is_dyn_scene)
         
            scene_parts.append(scene_part)

        scene.scene_parts = scene_parts
       
        scene._cpp_object.primitives = [primitive._cpp_object for primitive in scene.primitives]
   
        for part in scene.scene_parts:
            part._cpp_object.primitives = [primitive._cpp_object for primitive in part.primitives]
            for primitive in part.primitives:
                if primitive.material is not None:
                    primitive._cpp_object.material = primitive.material._cpp_object

                if primitive.vertices is not None:
                    abd = [vertex._cpp_object for vertex in primitive.vertices]
                    primitive._cpp_object.vertices = abd

        #NOTE: Error starts here. The problem is with Plane class, calculated in Scene::doForceOnGround() 
        scene._cpp_object.finalize_loading()
     
        scene.kd_factory_type = kd_factory_type
        scene.kdt_num_jobs = kdt_num_jobs
        scene.kdt_geom_jobs = kdt_geom_jobs
        scene.kdt_sah_loss_nodes = kdt_sah_loss_nodes
        kd_tree_type = scene.define_kd_type()
        scene._cpp_object.kd_grove_factory = _helios.KDGroveFactory(kd_tree_type)

        return cls._validate(scene)
    

    

    def _digest_scene_part(self, scene_part: ScenePart, part_index: int, holistic: Optional[bool] = False, split_part: Optional[bool] = False, dyn_object: Optional[bool] = False) -> None:
        scene_part.compute_transform(holistic)

        if not dyn_object:
            self.static_objects.append(scene_part)
            self._cpp_object.append_static_object_part(scene_part._cpp_object)

        self.primitives.extend(scene_part.primitives)
        #TODO: implement splitting of scene parts
        '''
        if (split_part):
            part_index_offset = len(scene_part.subpart_limit) - 1
            if scene_part.split_subparts():
                part_index += part_index_offset
        '''
       
        num_vertices = len(scene_part.primitives[0].vertices)

        if num_vertices == 3:
            scene_part.primitive_type = PrimitiveType.TRIANGLE
        elif num_vertices == 2:
            scene_part.primitive_type = PrimitiveType.VOXEL

    def define_kd_type(self):
        """Define the KD tree factory based on the factory type and number of jobs."""
        self.kdt_num_jobs = self.kdt_num_jobs or (os.cpu_count() or 1)
        self.kdt_geom_jobs = self.kdt_geom_jobs or self.kdt_num_jobs
        
        # Define a mapping of factory types to corresponding maker functions
        factory_makers = {
            1: {
                'single': KDTreeFactoryMaker.make_simple_kd_tree,
                'multi': KDTreeFactoryMaker.make_multithreaded_simple_kd_tree,
            },
            2: {
                'single': KDTreeFactoryMaker.make_sah_kd_tree_factory,
                'multi': KDTreeFactoryMaker.make_multithreaded_sah_kd_tree_factory,
            },
            3: {
                'single': KDTreeFactoryMaker.make_axis_sah_kd_tree_factory,
                'multi': KDTreeFactoryMaker.make_multithreaded_axis_sah_kd_tree_factory,
            },
            4: {
                'single': KDTreeFactoryMaker.make_fast_sah_kd_tree_factory,
                'multi': KDTreeFactoryMaker.make_multithreaded_fast_sah_kd_tree_factory,
            }
        }

        # Determine whether to use a multi-threaded or single-threaded factory
        factory_type = 'multi' if self.kdt_num_jobs > 1 else 'single'
        
        # Get the appropriate factory maker function based on the type
        if self.kd_factory_type in factory_makers:
            factory_func = factory_makers[self.kd_factory_type][factory_type]

            # Pass the required parameters to the factory function
            if factory_type == 'multi':
                # For multithreaded factories, pass node_jobs, geom_jobs, and optionally loss_nodes
                if self.kd_factory_type == 1:  # SimpleKDTreeFactory doesn't use loss_nodes
                    return factory_func(self.kdt_num_jobs, self.kdt_geom_jobs)
                else:
                    return factory_func(self.kdt_num_jobs, self.kdt_geom_jobs, self.kdt_sah_loss_nodes)
            else:
                # For single-threaded factories
                if self.kd_factory_type == 1:  # SimpleKDTreeFactory doesn't use any arguments
                    return factory_func()
                else:
                    return factory_func(self.kdt_sah_loss_nodes)
            
    class Config:
        arbitrary_types_allowed = True              

class StaticScene(Scene):
    def __init__(self, 
                 static_objects: Optional[List[ScenePart]] = [], 
                 bbox: Optional[AABB] = None, 
                 bbox_crs: Optional[AABB] = None) -> None:
        super().__init__(static_objects, bbox, bbox_crs)
        self._cpp_object = _helios.StaticScene()
        self.static_objects: Optional[List[ScenePart]] = static_objects
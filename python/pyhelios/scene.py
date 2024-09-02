from pyhelios.utils import Validatable, ValidatedCppManagedProperty
from pyhelios.primitives import Rotation, Primitive, AABB

from typing import Optional, List, Tuple, Annotated, Type, Any
import _helios

class ScenePart(Validatable):

    def __init__(self, 
                 id: Optional[str] = "", 
                 origin: Optional[List[float]] = [0.0, 0.0, 0.0], 
                 rotation: Optional[Rotation] =  None, 
                 scale: Optional[float] = 1.0, 
                 bound: Optional[List[float]] = None,
                 primitives: Optional[List[Primitive]] = None) -> None:

              
            self._cpp_object = _helios.ScenePart()
            self.id = id
            self.origin = origin
            self.rotation = rotation
            self.scale = scale
            self.bound = bound
            if primitives is not None:
                self.primitives = primitives
            else:
                self.primitives = []

    
    
    id: Optional[str] = ValidatedCppManagedProperty("id")
    origin: Optional[Tuple[float, float, float]] = ValidatedCppManagedProperty("origin")
    rotation: Optional[Rotation] = ValidatedCppManagedProperty("rotation")
    scale: Optional[float] = ValidatedCppManagedProperty("scale")
    bound: Optional[tuple[float, float, float]] = ValidatedCppManagedProperty("bound")
    primitives: Optional[List[Primitive]] = ValidatedCppManagedProperty("primitives")


class Scene(Validatable):

    def __init__(self, 
            scene_parts: Optional[List[ScenePart]], bbox: Optional[AABB] = None, bbox_crs: Optional[AABB] = None) -> None:
        
        self._cpp_object = _helios.Scene()
        self.scene_parts = scene_parts
        self.bbox = bbox
        self.bbox_crs = bbox_crs

    scene_parts: Optional[List[ScenePart]] = ValidatedCppManagedProperty("scene_parts")
    bbox: Optional[AABB] = ValidatedCppManagedProperty("bbox")
    bbox_crs: Optional[AABB] = ValidatedCppManagedProperty


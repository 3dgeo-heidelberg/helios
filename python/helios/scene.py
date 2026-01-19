# from typing import Optional, List
# from pydantic import BaseModel
# import numpy as np
# from helios.scenepart import ScenePart, ObjectType


# class Scene(BaseModel):
#     kdgrove_factory: Optional[KDGroveFactory]
#     kdgrove: Optional[KDGrove]
#     bbox: Optional[AABB]
#     bbox_crs: Optional[AABB]
#     kd_raycaster: Optional[KDGroveRaycaster]
#     primitives: List[Primitive]
#     _scene_parts: List[ScenePart]
#     material: Optional[Material] = None


#     @property
#     def scene_parts(self) -> List[ScenePart]:
#         return self._scene_parts

#     @scene_parts.setter
#     def scene_parts(self, scene_parts: List[ScenePart]):
#         self._scene_parts = scene_parts

#     def has_moving_objects(self) -> bool:
#         return any(scene_part.object_type == ObjectType.DYN_MOVING_OBJECT for scene_part in self._scene_parts)

#     def add_scene_part(self, scene_part: ScenePart):
#         if scene_part not in self._scene_parts:
#             self._scene_parts.append(scene_part)

#     def show(self, annotate_ids: bool = False, show_bboxes: bool = False, show_crs: bool = False, show_kd: bool = False, show_kd_raycaster: bool = False):
#         # visualize scene prior to simulation (e.g. with open3D/PyVista)
#         pass

#     def get_scene_part(self, scenepart_id: int) -> ScenePart:
#         try:
#             return self._scene_parts[scenepart_id]
#         except IndexError:
#             raise ValueError(f"No ScenePart with this id")

#     def delete_scene_part(self, scenepart_id: int)-> bool:
#         if 0 <= scenepart_id < len(self._scene_parts):
#             del self._scene_parts[scenepart_id]
#             return True
#         return False

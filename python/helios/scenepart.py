# from typing import Optional, List
# from pydantic import BaseModel, Field
# import numpy as np
# import os
# from pathlib import Path
# import open3d as o3d
# from enum import Enum

# class PrimitiveType(str, Enum):
#     NONE = "NONE"
#     TRIANGLE = "TRIANGLE"
#     VOXEL = "VOXEL"

# class ObjectType(str, Enum):
#     STATIC_OBJECT = "STATIC_OBJECT"
#     DYN_OBJECT = "DYN_OBJECT"
#     DYN_MOVING_OBJECT = "DYN_MOVING_OBJECT"

# class ScenePart(BaseModel):
#     name: str = Field(default="")
#     primitive_type: PrimitiveType = Field(default=PrimitiveType.NONE)
#     object_type: ObjectType = Field(default=ObjectType.STATIC_OBJECT)

#     subpart_borders: List[int] = Field(default_factory=list)
#     ray_intersection_mode: str = Field(default="")
#     ray_intersection_argument: bool = Field(default=False)
#     random_shift: bool = Field(default=False)
#     origin: np.ndarray = Field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))
#     rotation: np.ndarray = Field(default_factory=lambda: np.eye(3))  # Assuming rotation is a 3x3 matrix
#     scale: float = Field(default=1.0)
#     centroid: np.ndarray = Field(default_factory=lambda: np.array([0.0, 0.0, 0.0]))

#     motions: List[dict] = Field(default_factory=list)
#     is_on_ground: bool = Field(default=True)
#     time_step_duration: float = Field(default=0.0)

#     @classmethod
#     def from_file(cls, filename: str, file_format: Optional[str] = None) -> 'ScenePart':
#         if not os.path.exists(filename):
#             raise FileNotFoundError(f"File {filename} not found.")

#         if file_format is None:
#             file_format = Path(filename).suffix[1:].lower()

#         # Logic to handle different file formats
#         method_name = f"from_{file_format}"
#         if hasattr(cls, method_name):
#             method = getattr(cls, method_name)
#             return method(filename)
#         else:
#             raise ValueError(f"Unsupported file format: {file_format}")

#     @staticmethod
#     def from_obj(filename: str) -> 'ScenePart':
#         # Implement OBJ file processing logic
#         pass

#     @staticmethod
#     def from_tiff(filename: str) -> 'ScenePart':
#         # Implement TIFF file processing logic
#         pass

#     @staticmethod
#     def from_xml(filename: str) -> 'ScenePart':
#         # Implement XML file processing logic
#         pass

#     @staticmethod
#     def from_json(filename: str) -> 'ScenePart':
#         # Implement JSON file processing logic
#         pass

#     @staticmethod
#     def from_csv(filename: str) -> 'ScenePart':
#         # Implement CSV file processing logic
#         pass

#     @staticmethod
#     def from_xyz(filename: str) -> 'ScenePart':
#         # Implement XYZ file processing logic
#         pass

#     @staticmethod
#     def from_las(filename: str) -> 'ScenePart':
#         # Implement LAS file processing logic
#         pass

#     def from_o3d(self, o3d_mesh: o3d.geometry.TriangleMesh):
#         # Implement processing logic for Open3D mesh
#         pass

#     def transform(
#         self,
#         translation: Optional[np.ndarray] = None,
#         scale: Optional[float] = None,
#         is_on_ground: Optional[bool] = True,
#         apply_to_axis: Optional[int] = None
#     ) -> 'ScenePart':
#         # Implement transformation logic
#         pass

#     def rotate(
#         self,
#         axis: Optional[np.ndarray] = None,
#         angle: Optional[float] = None,
#         origin: Optional[np.ndarray] = None,
#         rotation: Optional[np.ndarray] = None,
#         matrix: Optional[np.ndarray] = None,
#         euler_angles: Optional[np.ndarray] = None
#     ):
#         # Implement rotation logic
#         pass

#     def make_motion(
#         self,
#         translation: Optional[np.ndarray] = None,
#         rotation_axis: Optional[np.ndarray] = None,
#         rotation_angle: Optional[float] = None,
#         radians: Optional[bool] = True,
#         rotation_center: Optional[np.ndarray] = None,
#         loop: Optional[int] = 1,
#         rotate_around_self: Optional[bool] = False,
#         auto_crs: Optional[bool] = False
#     ):
#         # Implement motion logic
#         pass

#     def make_motion_sequence(
#         self,
#         translations: Optional[List[np.ndarray]] = None,
#         rotation_axes: Optional[List[np.ndarray]] = None,
#         rotation_angles: Optional[List[float]] = None,
#         radians: Optional[bool] = True,
#         rotation_centers: Optional[List[np.ndarray]] = None,
#         loop: Optional[int] = 1,
#         rotate_around_self: Optional[bool] = False,
#         auto_crs: Optional[bool] = False
#     ):
#         # Implement motion sequence logic
#         pass

from helios.settings import (
    ExecutionSettings,
    compose_execution_settings,
    ForceOnGroundStrategy,
)
from helios.utils import (
    classonlymethod,
    get_asset_directories,
    detect_separator,
    is_xml_loaded,
    _validate_points_array_and_get_indices,
    _as_array,
    _validate_same_shape,
    _validate_triangle_uvs,
)

from helios.validation import (
    Angle,
    AssetPath,
    CompressionLevel,
    Model,
    MultiAssetPath,
    Quaternion,
    R3Vector,
    validate_xml_file,
    UpdateableMixin,
)

from collections.abc import MutableMapping
from numpydantic import NDArray, Shape
from pydantic import (
    PositiveFloat,
    PositiveInt,
    NonNegativeFloat,
    NonNegativeInt,
    validate_call,
)
from pathlib import Path
from typing import Literal, Optional, Union, Tuple
import numpy as np

import _helios


class Material(Model, UpdateableMixin, cpp_class=_helios.Material):
    """
    Class representing a material that can be applied to primitives in a scene part. A material defines the visual properties of the primitives, such as their color and reflectance,
    but also helios-specific properties such as the classification and whether the scene part should be considered as ground.

    :param name: The name of the material. This is used to reference the material when applying it to primitives in a scene part.
    :param is_ground: Whether the material should be considered as ground. This is used to position legs with 'force_on_ground' strategy on the ground level of the scene.
    :param spectra: The name of the spectra file to use for the material. The spectra file should be located in one of the asset directories.
    :param reflectance: The reflectance of the material. This is used in the intensity calculation for returns on the respective scene parts.
    :param specularity: The specularity of the material. This is used in the intensity calculation for returns on the respective scene parts.
    :param specular_exponent: The specular exponent of the material. This is used in the intensity calculation for returns on the respective scene parts.
    :param classification: The classification of the material. This is used to assign a classification value to points that hit the respective scene parts.
    :param ambient_components: The ambient components of the material. In .mtl files, this is the Ka component. This is used in the intensity calculation for returns on the respective scene parts.
    :param diffuse_components: The diffuse components of the material. In .mtl files, this is the Kd component. This is used in the intensity calculation for returns on the respective scene parts.
    :param specular_components: The specular components of the material. In .mtl files, this is the Ks component. This is used in the intensity calculation for returns on the respective scene parts.
    :type name: str
    :type is_ground: bool
    :type spectra: str
    :type reflectance: float
    :type specularity: float
    :type specular_exponent: float
    :type classification: int
    :type ambient_components: NDArray[Shape["4"], np.float64]
    :type diffuse_components: NDArray[Shape["4"], np.float64]
    :type specular_components: NDArray[Shape["4"], np.float64]
    """
    name: str = ""
    is_ground: bool = False
    spectra: str = ""
    reflectance: float = np.nan
    specularity: float = 0
    specular_exponent: float = 10
    classification: int = 0
    ambient_components: NDArray[Shape["4"], np.float64] = np.array(
        [0, 0, 0, 0], dtype=np.float64
    )
    diffuse_components: NDArray[Shape["4"], np.float64] = np.array(
        [0, 0, 0, 0], dtype=np.float64
    )
    specular_components: NDArray[Shape["4"], np.float64] = np.array(
        [0, 0, 0, 0], dtype=np.float64
    )

    @classmethod
    @validate_call
    def from_file(cls, material_file: AssetPath, material_id: str = ""):
        _cpp_material = _helios.read_material_from_file(
            str(material_file), [str(p) for p in get_asset_directories()], material_id
        )
        material = cls._from_cpp(_cpp_material)
        material._set_constructor_provenance(
            "from_file",
            material_file=material_file,
            material_id=material_id,
        )
        return material


class MaterialDict(MutableMapping[str, Material]):
    """
    A dictionary-like interface for accessing and modifying materials via their names.
    """

    def __init__(self, owner: "ScenePart"):
        self._owner = owner

    def _snapshot(self) -> dict[str, Material]:
        pairs = _helios.get_materials_map(self._owner._cpp_object)
        return {name: Material._from_cpp(mat) for name, mat in pairs}

    def __getitem__(self, name: str) -> Material:
        return self._snapshot()[name]

    def __setitem__(self, name: str, material: Material) -> None:
        if not isinstance(material, Material):
            raise TypeError("Value must be an instance of Material.")

        snapshot = self._snapshot()
        if name not in snapshot:
            raise KeyError(
                f"Material with name '{name}' does not exist in the scene part."
            )

        _helios.change_material_instance(
            self._owner._cpp_object, name, material._cpp_object
        )

    def __delitem__(self, name: str) -> None:
        raise NotImplementedError("Deleting materials is not supported.")

    def __iter__(self):
        return iter(self._snapshot())

    def __len__(self):
        return len(self._snapshot())

    def __contains__(self, name: object) -> bool:
        if not isinstance(name, str):
            return False
        return name in self._snapshot()

    def items(self):
        return list(self._snapshot().items())

    def keys(self):
        return list(self._snapshot().keys())

    def values(self):
        return self._snapshot().values()


class BoundingBox(Model, cpp_class=_helios.AABB):
    """
    Class representing an axis-aligned bounding box. Used for representing the bounds of scene parts and scenes.
    """
    @property
    def bounds(self) -> tuple[R3Vector, R3Vector]:
        """The lower left and upper right corner of the bounding box."""
        return self._cpp_object.bounds

    @property
    def centroid(self) -> R3Vector:
        """The centroid of the bounding box."""
        lower, upper = self.bounds
        return (
            np.asarray(lower, dtype=np.float64) + np.asarray(upper, dtype=np.float64)
        ) / 2.0


class ScenePart(Model, cpp_class=_helios.ScenePart):
    """
    Class representing a part of the scene. A scene part is a collection of primitives that share the same material and that is typically loaded from one file.
    A scene can be composed of multiple scene parts, which can be transformed independently and have different materials.
    Scene parts can be loaded from .obj, .tiff, .xyz and .vox files.

    :param force_on_ground: The strategy to use for forcing the scene part on the ground. This is used to position legs with 'force_on_ground' strategy on the ground level of the scene.
    :type force_on_ground: Union[ForceOnGroundStrategy, PositiveInt]
    """
    force_on_ground: Union[ForceOnGroundStrategy, PositiveInt] = (
        ForceOnGroundStrategy.NONE
    )

    @property
    def bbox(self) -> BoundingBox:
        """The axis-aligned bounding box of the scene part."""
        return BoundingBox._from_cpp(self._cpp_object.bbox)

    @property
    def materials(self) -> MaterialDict:
        """Dictionary-like access to materials in the scene part.
        
        :return: A dictionary-like object for accessing and modifying materials in the scene part via their names.
        :rtype: MaterialDict
        """
        return MaterialDict(self)

    @validate_call
    def rotate(
        self,
        quaternion: Optional[Quaternion] = None,
        axis: Optional[R3Vector] = None,
        angle: Optional[Angle] = None,
        from_axis: Optional[R3Vector] = None,
        to_axis: Optional[R3Vector] = None,
        rotation_center: Optional[R3Vector] = None,
    ):
        """Rotate the scene part.

        The rotation can be specified as one of the following parameters:
        
        * A quaternion
        * An axis and an angle
        * An origin and an image vector

        Optionally, you may specify a rotation center. If omitted the origin
        of the coordinate system of the scene part will be used.

        :param quaternion: The quaternion to use for the rotation.
        :param axis: The axis to use for the rotation. Must be provided together with the 'angle' parameter.
        :param angle: The angle to use for the rotation. Must be provided together with the 'axis' parameter.
        :param from_axis: The origin vector to use for the rotation. Must be provided together with the 'to_axis' parameter.
        :param to_axis: The image vector to use for the rotation. Must be provided together with the 'from_axis' parameter.
        :param rotation_center: The center to use for the rotation. If omitted, the origin of the coordinate system of the scene part will be used.
        :type quaternion: Optional[Quaternion]
        """

        # The rotation object that we want to construct
        rot = None

        # Handle construction via a given quaternion
        if quaternion is not None:
            rot = _helios.Rotation(
                quaternion[0], quaternion[1], quaternion[2], quaternion[3], True
            )

        # Handle construction via an axis and angle
        if axis is not None or angle is not None:
            if rot is not None:
                raise ValueError("Too many rotation parameters specified")
            if axis is None:
                raise ValueError("Axis must be specified when angle is specified")
            if angle is None:
                raise ValueError("Angle must be specified when axis is specified")

            rot = _helios.Rotation(axis, angle)

        # Handle construction via two vectors
        if from_axis is not None or to_axis is not None:
            if rot is not None:
                raise ValueError("Too many rotation parameters specified")
            if from_axis is None:
                raise ValueError("Origin must be specified when image is specified")
            if to_axis is None:
                raise ValueError("Image must be specified when origin is specified")

            rot = _helios.Rotation(from_axis, to_axis)

        if rot is None:
            raise ValueError("No rotation parameters specified")

        # Maybe shift by the rotation center
        if rotation_center is not None:
            _helios.translate_scene_part(self._cpp_object, -rotation_center)

        # Perform the actual rotation
        _helios.rotate_scene_part(self._cpp_object, rot)

        # Undo the shift by the rotation center
        if rotation_center is not None:
            _helios.translate_scene_part(self._cpp_object, rotation_center)

        self._append_operation_provenance(
            "rotate",
            quaternion=quaternion,
            axis=axis,
            angle=angle,
            from_axis=from_axis,
            to_axis=to_axis,
            rotation_center=rotation_center,
        )

        return self

    @validate_call
    def scale(self, factor: PositiveFloat):
        """Scale the scene part by a factor.
        
        :param factor: The factor to scale the scene part by.
        :type factor: PositiveFloat
        """

        _helios.scale_scene_part(self._cpp_object, factor)
        self._append_operation_provenance("scale", factor=factor)
        return self

    @validate_call
    def translate(self, offset: R3Vector):
        """Translate the scene part by an offset.
        
        :param offset: The offset to translate the scene part by.
        :type offset: R3Vector
        """

        _helios.translate_scene_part(self._cpp_object, offset)
        self._append_operation_provenance("translate", offset=offset)
        return self

    @classonlymethod
    @validate_call
    def from_xml(cls, scene_part_file: AssetPath, id: int):
        # Validate the XML
        validate_xml_file(scene_part_file, "xsd/scene.xsd")

        _cpp_scene_part = _helios.read_scene_part_from_xml(
            str(scene_part_file), [str(p) for p in get_asset_directories()], id
        )
        scene_part = cls._from_cpp(_cpp_scene_part)
        scene_part._is_loaded_from_xml = True
        scene_part._disable_yaml_serialization_for_descendants()
        scene_part._set_constructor_provenance(
            "from_xml",
            scene_part_file=scene_part_file,
            id=id,
        )
        return scene_part

    @classonlymethod
    @validate_call
    def from_obj(cls, obj_file: AssetPath, up_axis: Literal["y", "z"] = "z"):
        """Load the scene part from an OBJ file.

        For paths (potentially) containing wildcards, use 'ScenePart.from_objs()' instead!

        :param obj_file: The path to the OBJ file to load the scene part from. Can potentially contain wildcards ('*').
        :param up_axis: The up axis to use for the loaded scene part.
        :type obj_file: AssetPath
        :type up_axis: Literal["y", "z"]

        :return: The loaded scene part.
        :rtype: ScenePart
        """

        _cpp_part = _helios.read_obj_scene_part(
            str(obj_file), [str(p) for p in get_asset_directories()], up_axis
        )

        scene_part = cls._from_cpp(_cpp_part)
        scene_part._set_constructor_provenance(
            "from_obj",
            obj_file=obj_file,
            up_axis=up_axis,
        )
        return scene_part

    @classonlymethod
    @validate_call
    def from_tiff(cls, tiff_file: AssetPath):
        """Load the scene part from a TIFF file.
        
        For paths (potentially) containing wildcards, use 'ScenePart.from_tiffs()' instead!
        
        :param tiff_file: The path to the TIFF file to load the scene part from.
        :type tiff_file: AssetPath
        
        :return: The loaded scene part.
        :rtype: ScenePart
        """

        _cpp_part = _helios.read_tiff_scene_part(
            str(tiff_file), [str(p) for p in get_asset_directories()]
        )

        scene_part = cls._from_cpp(_cpp_part)
        scene_part._set_constructor_provenance(
            "from_tiff",
            tiff_file=tiff_file,
        )
        return scene_part

    @classonlymethod
    @validate_call
    def from_objs(cls, obj_files: MultiAssetPath, up_axis: Literal["y", "z"] = "z"):
        """Load multiple scene parts from OBJ files

        Expects a single Path containing some (or none) wildcards ('*').
        Supports '**' for matching multiple directories.

        :param obj_files: The path to the OBJ files to load the scene parts from. Can potentially contain wildcards ('*').
        :param up_axis: The up axis to use for the loaded scene parts.
        :type obj_files: MultiAssetPath
        :type up_axis: Literal["y", "z"]

        :return: The loaded scene parts.
        :rtype: list[ScenePart]
        """
        return [ScenePart.from_obj(obj, up_axis) for obj in obj_files]

    @classonlymethod
    @validate_call
    def from_tiffs(cls, tiff_files: MultiAssetPath):
        """Load multiple scene parts from TIFF files

        Expects a single Path containing some (or none) wildcards ('*').
        Supports '**' for matching multiple directories.

        :param tiff_files: The path to the TIFF files to load the scene parts from. Can potentially contain wildcards ('*').
        :type tiff_files: MultiAssetPath
        :return: The loaded scene parts.
        :rtype: list[ScenePart]

        :return: The loaded scene parts.
        :rtype: list[ScenePart]
        """
        return [ScenePart.from_tiff(tiff) for tiff in tiff_files]

    @classonlymethod
    @validate_call
    def from_xyz(
        cls,
        xyz_file: AssetPath,
        voxel_size: PositiveFloat,
        separator: Optional[str] = None,
        max_color_value: NonNegativeFloat = 0.0,
        default_normal: R3Vector = np.array(
            [np.finfo(np.float64).max] * 3, dtype=np.float64
        ),
        sparse: bool = True,
        estimate_normals: bool = False,
        normals_file_columns: list[NonNegativeInt] = [3, 4, 5],
        rgb_file_columns: list[NonNegativeInt] = [6, 7, 8],
        snap_neighbor_normal: bool = False,
    ):
        """
        Load the scene part from an XYZ file.
        
        :param xyz_file: The path to the XYZ file to load the scene part from. Can potentially contain wildcards ('*').
        :param voxel_size: The voxel size to use for voxelizing the XYZ point cloud so that it can be converted to a 3D model with surfaces that can be scanned.
        :param separator: The separator used in the XYZ file. If not provided, the separator will be automatically detected.
        :param max_color_value: The maximum color value used for normalizing RGB values in the XYZ file.
        :param default_normal: The default normal to use for points in the XYZ file that do not have a normal.
        :param sparse: Whether to use a sparse voxel grid for the conversion of the XYZ point cloud to a 3D model. If false, a dense voxel grid will be used, which can lead to higher memory usage.
        :param estimate_normals: Whether to estimate normals for points in the XYZ file that do not have a normal. This can be used if the XYZ file does not contain normals or if the provided normals are not reliable.
        :param normals_file_columns: The columns in the XYZ file to use for the normal components if the XYZ file contains normals. The default is [3, 4, 5].
        :param rgb_file_columns: The columns in the XYZ file to use for the RGB components if the XYZ file contains RGB values. The default is [6, 7, 8].
        :param snap_neighbor_normal: Whether to snap the normal of points in the XYZ file that do not have a normal to the normal of their nearest neighbor that has a normal.
        :type xyz_file: AssetPath
        :type voxel_size: PositiveFloat
        :type separator: Optional[str]
        :type max_color_value: NonNegativeFloat
        :type default_normal: R3Vector
        :type sparse: bool
        :type estimate_normals: bool
        :type normals_file_columns: list[NonNegativeInt]
        :type rgb_file_columns: list[NonNegativeInt]
        :type snap_neighbor_normal: bool

        :return: The loaded scene part.
        :rtype: ScenePart
        """

        if separator is None:
            separator = detect_separator(xyz_file)

        _cpp_part = _helios.read_xyz_scene_part(
            str(xyz_file),
            [str(p) for p in get_asset_directories()],
            separator,
            voxel_size,
            max_color_value,
            default_normal,
            sparse,
            int(estimate_normals),
            normals_file_columns[0],
            normals_file_columns[1],
            normals_file_columns[2],
            rgb_file_columns[0],
            rgb_file_columns[1],
            rgb_file_columns[2],
            snap_neighbor_normal,
        )

        scene_part = cls._from_cpp(_cpp_part)
        scene_part._set_constructor_provenance(
            "from_xyz",
            xyz_file=xyz_file,
            voxel_size=voxel_size,
            separator=separator,
            max_color_value=max_color_value,
            default_normal=default_normal,
            sparse=sparse,
            estimate_normals=estimate_normals,
            normals_file_columns=normals_file_columns,
            rgb_file_columns=rgb_file_columns,
            snap_neighbor_normal=snap_neighbor_normal,
        )
        return scene_part

    @classonlymethod
    @validate_call
    def from_xyzs(
        cls,
        xyz_files: MultiAssetPath,
        voxel_size: PositiveFloat,
        separator: Optional[str] = None,
        max_color_value: NonNegativeFloat = 0.0,
        default_normal: R3Vector = np.array(
            [np.finfo(np.float64).max] * 3, dtype=np.float64
        ),
        sparse: bool = False,
        estimate_normals: bool = False,
        normals_file_columns: list[NonNegativeInt] = [3, 4, 5],
        rgb_file_columns: list[NonNegativeInt] = [6, 7, 8],
        snap_neighbor_normal: bool = False,
    ):
        """
        Load multiple scene parts from XYZ files.

        Expects a single Path containing some (or none) wildcards ('*').
        Supports '**' for matching multiple directories.

        Each parameters should be a single shared value for all files.

        :param xyz_file: The path to the XYZ file to load the scene part from. Can potentially contain wildcards ('*').
        :param voxel_size: The voxel size to use for voxelizing the XYZ point cloud so that it can be converted to a 3D model with surfaces that can be scanned.
        :param separator: The separator used in the XYZ file.
        :param max_color_value: The maximum color value used for normalizing RGB values in the XYZ file.
        :param default_normal: The default normal to use for points in the XYZ file that do not have a normal.
        :param sparse: Whether to use a sparse voxel grid for the conversion of the XYZ point cloud to a 3D model. If false, a dense voxel grid will be used, which can lead to higher memory usage.
        :param estimate_normals: Whether to estimate normals for points in the XYZ file that do not have a normal. This can be used if the XYZ file does not contain normals or if the provided normals are not reliable.
        :param normals_file_columns: The columns in the XYZ file to use for the normal components if the XYZ file contains normals. The default is [3, 4, 5].
        :param rgb_file_columns: The columns in the XYZ file to use for the RGB components if the XYZ file contains RGB values. The default is [6, 7, 8].
        :param snap_neighbor_normal: Whether to snap the normal of points in the XYZ file that do not have a normal to the normal of their nearest neighbor that has a normal.
        :type xyz_file: AssetPath
        :type voxel_size: PositiveFloat
        :type separator: Optional[str]
        :type max_color_value: NonNegativeFloat
        :type default_normal: R3Vector
        :type sparse: bool
        :type estimate_normals: bool
        :type normals_file_columns: list[NonNegativeInt]
        :type rgb_file_columns: list[NonNegativeInt]
        :type snap_neighbor_normal: bool

        :return: The loaded scene parts.
        :rtype: list[ScenePart]
        """
        return [
            ScenePart.from_xyz(
                xyz,
                voxel_size,
                separator,
                max_color_value,
                default_normal,
                sparse,
                estimate_normals,
                normals_file_columns,
                rgb_file_columns,
                snap_neighbor_normal,
            )
            for xyz in xyz_files
        ]

    @classonlymethod
    @validate_call
    def from_vox(
        cls,
        vox_file: AssetPath,
        intersection_mode: Literal["scaled", "fixed", "transmittive"] = "transmittive",
        intersection_argument: Optional[float] = None,
        random_shift: bool = False,
        ladlut_path: Optional[str] = None,
    ):
        """Load the scene part from a VOX file.
        
        :param vox_file: The path to the VOX file to load the scene part from.
        :param intersection_mode: The mode to use for calculating the intersection of rays with the voxels ['transmittive', 'fixed', 'scaled'].
        :param intersection_argument: Scaling factor; only used for intersection mode 'scaled'.
        :param random_shift: Whether to apply a random shift to the scaled cubes within the original voxel resolution; only used for intersection mode 'scaled'.
        :param ladlut_path: The path to the txt file with look-up-tables (LUTs) for custom leaf angle distributions (LADs). See `data/lut` for examples.
        :type vox_file: AssetPath
        :type intersection_mode: Literal["scaled", "fixed", "transmittive"]
        :type intersection_argument: Optional[float]
        :type random_shift: bool
        :type ladlut_path: Optional[str]

        :return: The loaded scene part.
        :rtype: ScenePart
        """

        if intersection_mode == "fixed" and intersection_argument is not None:
            raise ValueError(
                "'intersection_argument' must not be provided when 'intersection_mode' is 'fixed'."
            )

        _cpp_part = _helios.read_vox_scene_part(
            str(vox_file),
            [str(p) for p in get_asset_directories()],
            intersection_mode,
            intersection_argument if intersection_argument is not None else 0.0,
            random_shift,
            ladlut_path if ladlut_path is not None else "",
        )

        scene_part = cls._from_cpp(_cpp_part)
        scene_part._set_constructor_provenance(
            "from_vox",
            vox_file=vox_file,
            intersection_mode=intersection_mode,
            intersection_argument=intersection_argument,
            random_shift=random_shift,
            ladlut_path=ladlut_path,
        )
        return scene_part

    @classonlymethod
    @validate_call
    def from_numpy_array(
        cls,
        points: NDArray,
        voxel_size: PositiveFloat,
        *,
        normals_file_columns: Optional[list[NonNegativeInt]] = None,
        rgb_file_columns: Optional[list[NonNegativeInt]] = None,
        max_color_value: NonNegativeFloat = 0.0,
        default_normal: R3Vector = np.array(
            [np.finfo(np.float64).max] * 3, dtype=np.float64
        ),
        sparse: bool = True,
        estimate_normals: bool = False,
        snap_neighbor_normal: bool = False,
    ):
        """Load the scene part from a numpy array."""

        ncols, rcols = _validate_points_array_and_get_indices(
            points,
            normals_file_columns=normals_file_columns,
            rgb_file_columns=rgb_file_columns,
        )
        _cpp_part = _helios.read_numpy_scene_part(
            points,
            [str(p) for p in get_asset_directories()],
            voxel_size,
            max_color_value,
            default_normal,
            sparse,
            estimate_normals,
            ncols[0],
            ncols[1],
            ncols[2],
            rcols[0],
            rcols[1],
            rcols[2],
            snap_neighbor_normal,
        )

        return cls._from_cpp(_cpp_part)

    @classmethod
    def _compose_from_o3d_triangle_mesh(
        cls,
        geometry,
        *,
        up_axis: Literal["y", "z"] = "z",
    ):
        if up_axis not in ("y", "z"):
            raise ValueError("`up_axis` must be either 'y' or 'z'.")

        vertices = _as_array(
            geometry.vertices, dtype=np.float64, name="Open3D mesh vertices"
        )
        triangles = _as_array(
            geometry.triangles, dtype=np.int32, name="Open3D mesh triangles"
        )

        vertex_normals = None
        if geometry.has_vertex_normals():
            vertex_normals = _as_array(
                geometry.vertex_normals,
                dtype=np.float64,
                name="Open3D mesh vertex normals",
            )
            vertex_normals = _validate_same_shape(
                vertex_normals, "vertices", vertices, "Open3D mesh vertex normals"
            )

        triangle_normals = None
        if geometry.has_triangle_normals():
            triangle_normals = _as_array(
                geometry.triangle_normals,
                dtype=np.float64,
                name="Open3D mesh triangle normals",
            )
            triangle_normals = _validate_same_shape(
                triangle_normals, "triangles", triangles, "Open3D mesh triangle normals"
            )

        colors = None
        if geometry.has_vertex_colors():
            colors = _as_array(
                geometry.vertex_colors,
                dtype=np.float64,
                name="Open3D mesh vertex colors",
            )
            colors = _validate_same_shape(
                colors, "vertices", vertices, "Open3D mesh vertex colors"
            )

        triangle_uvs = None
        if geometry.has_triangle_uvs():
            triangle_uvs = _as_array(
                geometry.triangle_uvs,
                dtype=np.float64,
                shape_second_dim=2,
                name="Open3D mesh triangle uvs",
            )
            triangle_uvs = _validate_triangle_uvs(
                triangle_uvs, triangles, "Open3D mesh triangle uvs"
            )

        _cpp_part = _helios.read_open3d_mesh_scene_part(
            vertices,
            triangles,
            vertex_normals,
            triangle_normals,
            colors,
            triangle_uvs,
            up_axis,
        )

        return cls._from_cpp(_cpp_part)

    @classmethod
    def _compose_from_o3d_pointcloud(
        cls,
        geometry,
        *,
        voxel_size: PositiveFloat,
        max_color_value: NonNegativeFloat = 0.0,
        default_normal: R3Vector = np.array(
            [np.finfo(np.float64).max] * 3, dtype=np.float64
        ),
        sparse: bool = True,
        estimate_normals: bool = False,
        snap_neighbor_normal: bool = False,
    ):
        points = _as_array(
            geometry.points, dtype=np.float64, name="Open3D point cloud points"
        )
        normals = (
            _as_array(
                geometry.normals, dtype=np.float64, name="Open3D point cloud normals"
            )
            if geometry.has_normals()
            else None
        )
        colors = (
            _as_array(
                geometry.colors, dtype=np.float64, name="Open3D point cloud colors"
            )
            if geometry.has_colors()
            else None
        )

        combined_array = [points]
        column_count = 3
        if normals is not None:
            if normals.shape != points.shape:
                raise ValueError(
                    "The number of normals must match the number of points."
                )
            combined_array.append(normals)
            normals_indices = [column_count, column_count + 1, column_count + 2]
            column_count += 3

        if colors is not None:
            if colors.shape != points.shape:
                raise ValueError(
                    "The number of colors must match the number of points."
                )
            combined_array.append(colors)
            rgb_file_columns = [column_count, column_count + 1, column_count + 2]
            column_count += 3

        effective_max_color_value = (
            1.0 if colors is not None and max_color_value == 0.0 else max_color_value
        )

        result_array = np.hstack(combined_array)

        return cls.from_numpy_array(
            points=result_array,
            voxel_size=voxel_size,
            normals_file_columns=normals_indices if normals is not None else None,
            rgb_file_columns=rgb_file_columns if colors is not None else None,
            max_color_value=effective_max_color_value,
            default_normal=default_normal,
            sparse=sparse,
            estimate_normals=estimate_normals,
            snap_neighbor_normal=snap_neighbor_normal,
        )

    @classonlymethod
    @validate_call
    def from_open3d(cls, geometry, **kwargs):
        """
        Load the scene part from an Open3D geometry.
        The geometry can be either an open3d.geometry.TriangleMesh or an open3d.geometry.PointCloud. 
        In case of a triangle mesh, the behaviour is similar to 'ScenePart.from_obj()', in case of a point cloud, the behaviour is similar to 'ScenePart.from_xyz()' and the provided point cloud data will be voxelized.
        See below for the specific additional parameters that can be provided for each geometry type.
                
        :param geometry: The Open3D geometry to load the scene part from.
        :type geometry: open3d.geometry.TriangleMesh | open3d.geometry.PointCloud
        :param kwargs: Additional parameters to use for loading the scene part, depending on the provided geometry:
            a) for open3d.geometry.TriangleMesh: `up_axis`;
            b) for open3d.geometry.PointCloud: `voxel_size`, `max_color_value`, `default_normal`, `sparse`, `estimate_normals`, `snap_neighbor_normal`

        :returns: The loaded scene part.
        :rtype: ScenePart
        """
        try:
            import open3d
        except ImportError:
            raise ImportError(
                "Open3D is required for `ScenePart.from_open3d`, but can't be installed via conda. Install it with `pip install open3d`."
            )
        if isinstance(geometry, open3d.geometry.TriangleMesh):
            return cls._compose_from_o3d_triangle_mesh(geometry, **kwargs)
        if isinstance(geometry, open3d.geometry.PointCloud):
            return cls._compose_from_o3d_pointcloud(geometry, **kwargs)

        raise TypeError(
            "Unsupported geometry type for `ScenePart.from_o3d`. "
            f"Expected open3d.geometry.TriangleMesh or open3d.geometry.PointCloud, got {type(geometry)}."
        )

    @validate_call
    def _apply_material_to_all_primitives(self, material: Material):
        """Apply a material to all primitives in the scene part."""
        range_start = 0
        range_stop = len(self._cpp_object.primitives) - 1
        _helios.apply_material_to_primitives_range(
            self._cpp_object, material._cpp_object, range_start, range_stop
        )

    @validate_call
    def _apply_material_to_primitives_in_specific_range(
        self, material: Material, start: int, stop: int
    ):
        """Apply a material to all primitives within a specific elevation range in the scene part."""
        if start < 0 or stop < start:
            raise ValueError("Provided range is invalid")

        range_start = start
        range_stop = min(stop, len(self._cpp_object.primitives) - 1)
        _helios.apply_material_to_primitives_range(
            self._cpp_object, material._cpp_object, range_start, range_stop
        )

    @validate_call
    def _apply_material_to_indices(self, material: Material, indices: list[int]):
        """Apply a material to primitives at specific indices in the scene part."""
        valid_indices = [
            i for i in indices if 0 <= i < len(self._cpp_object.primitives)
        ]
        if not valid_indices:
            raise IndexError(
                "No valid indices provided. Indices must be within the range of existing primitives."
            )

        _helios.apply_material_to_primitives_indices(
            self._cpp_object, material._cpp_object, valid_indices
        )

    @validate_call
    def update_material(
        self,
        material: Material,
        indices: Optional[list[int]] = None,
        range_start: Optional[int] = None,
        range_stop: Optional[int] = None,
    ):
        """Update material(s) for the scene part.

        It can be provided in one of the following ways:

        * 'indices': a list of specific primitive indices to update
        * 'range_start' & 'range_stop': an index range to update

        If neither is provided, the material will be applied to all primitives.

        :param material: The material to apply to the scene part.
        :param indices: A list of specific primitive indices to update.
        :param range_start: The start of the range of indices to update.
        :param range_stop: The end of the range of indices to update.
        :type material: Material
        :type indices: Optional[list[int]]
        :type range_start: Optional[int]
        :type range_stop: Optional[int]
        """
        if indices is not None:
            self._apply_material_to_indices(material, indices)
        elif range_start is not None or range_stop is not None:
            range_start = range_start if range_start is not None else 0
            range_stop = (
                range_stop
                if range_stop is not None
                else len(self._cpp_object.primitives)
            )

            self._apply_material_to_primitives_in_specific_range(
                material, range_start, range_stop
            )
        else:
            self._apply_material_to_all_primitives(material)


class StaticScene(Model, cpp_class=_helios.StaticScene):
    """Class representing a static scene. A scene is composed of multiple scene parts, which can be transformed independently and have different materials.
    
    :param scene_parts: The scene parts that make up the scene.
    :type scene_parts: Tuple[ScenePart, ...]
    """
    scene_parts: Tuple[ScenePart, ...] = ()

    @property
    def bbox(self) -> BoundingBox:
        """The axis-aligned bounding box of the scene."""
        return BoundingBox._from_cpp(self._cpp_object.bbox)

    def add_scene_part(self, scene_part: ScenePart):
        """Add a scene part to the scene.
        
        :param scene_part: The scene part to add to the scene.
        :type scene_part: ScenePart
        """
        self.scene_parts = self.scene_parts + (scene_part,)
        _helios.add_scene_part_to_scene(self._cpp_object, scene_part._cpp_object)

    def _finalize(
        self, execution_settings: Optional[ExecutionSettings] = None, **parameters
    ):
        """Finalize the scene, making it ready for rendering."""
        if len(self._cpp_object.primitives) == 0:
            execution_settings = compose_execution_settings(
                execution_settings, parameters
            )
            self._is_finalized = True
            _helios.finalize_static_scene(
                self._cpp_object,
                execution_settings.factory_type,
                execution_settings.kdt_num_threads,
                execution_settings.kdt_geom_num_threads,
                execution_settings.sah_nodes,
            )

    def _set_reflectances(self, wavelength: float):
        """Modify the scene's primitives with correct reflectances for the given wavelength."""

        _helios.set_scene_reflectances(
            self._cpp_object, [str(p) for p in get_asset_directories()], wavelength
        )

    def _pre_set(self, field, value):
        if is_xml_loaded(self):
            raise RuntimeError("The scene loaded from XML cannot be modified.")
        if field == "scene_parts":
            self._enforce_uniqueness_across_instances(field, value)

    def _post_set(self, field):
        # When the Scene changes after initialization, we want to invalidate the KDTree etc.
        if not self._during_init:
            _helios.invalidate_static_scene(self._cpp_object)

    @classonlymethod
    @validate_call
    def from_xml(cls, scene_file: AssetPath):
        """Load the scene from an XML file.
        
        :param scene_file: The path to the XML file to load the scene from.
        :type scene_file: AssetPath
        """
        # Validate the XML
        validate_xml_file(scene_file, "xsd/scene.xsd")

        _cpp_scene = _helios.read_scene_from_xml(
            str(scene_file),
            [str(p) for p in get_asset_directories()],
            True,
        )
        scene = cls._from_cpp(_cpp_scene)
        scene._is_loaded_from_xml = True
        scene._disable_yaml_serialization_for_descendants()
        scene._set_constructor_provenance("from_xml", scene_file=scene_file)
        return scene

    @classonlymethod
    @validate_call
    def from_binary(cls, binary_file: AssetPath):
        """
        Load the scene from a binary file.

        :param binary_file: The path to the binary file to load the scene from.
        :type binary_file: AssetPath
        """
        _cpp_scene = _helios.StaticScene.from_binary(str(binary_file))
        scene = cls._from_cpp(_cpp_scene)
        scene._set_constructor_provenance("from_binary", binary_file=binary_file)
        return scene

    @validate_call
    def to_binary(self, binary_file: Path, compression_level: CompressionLevel = 6):
        """
        Save the scene to a binary file.

        :param binary_file: The path to the binary file to save the scene to.
        :param compression_level: The compression level to use for the binary file.
        :type binary_file: Path
        :type compression_level: CompressionLevel
        """
        # Ensure primitives and KD data exist before persisting.
        self._finalize()
        self._cpp_object.to_binary(
            str(binary_file.expanduser()), compression_level=compression_level
        )

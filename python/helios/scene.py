from helios.settings import ExecutionSettings, compose_execution_settings, ForceOnGroundStrategy
from helios.utils import get_asset_directories, detect_separator
from helios.validation import AssetPath, Model, MultiAssetPath, validate_xml_file

from numpydantic import NDArray, Shape
from pydantic import Field, PositiveFloat, NonNegativeFloat, NonNegativeInt, PositiveInt, validate_call, conint
from typing import Literal, Optional, Union, Annotated

import numpy as np

import _helios


class ScenePart(Model, cpp_class=_helios.ScenePart):

    force_on_ground: ForceOnGroundStrategy = ForceOnGroundStrategy.NONE   #  temporary placeholder it would be as below
    #Union[ForceOnGroundStrategy, PositiveInt] = ForceOnGroundStrategy.NONE
    is_ground: bool = False

    @validate_call
    def rotate(
        self,
        quaternion: Optional[NDArray[Shape["4"], np.float64]] = None,
        axis: Optional[NDArray[Shape["3"], np.float64]] = None,
        angle: Optional[float] = None,
        origin: Optional[NDArray[Shape["3"], np.float64]] = None,
        image: Optional[NDArray[Shape["3"], np.float64]] = None,
    ):
        """Rotate the scene part.

        The rotation can be specified as one of the following parameters:
        * A quaternion
        * An axis and an angle
        * An origin and an image vector
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
        if origin is not None or image is not None:
            if rot is not None:
                raise ValueError("Too many rotation parameters specified")
            if origin is None:
                raise ValueError("Origin must be specified when image is specified")
            if image is None:
                raise ValueError("Image must be specified when origin is specified")

            rot = _helios.Rotation(origin, image)

        if rot is None:
            raise ValueError("No rotation parameters specified")

        # Perform the actual rotation
        _helios.rotate_scene_part(self._cpp_object, rot)

        return self

    @validate_call
    def scale(self, factor: PositiveFloat):
        """Scale the scene part by a factor."""

        _helios.scale_scene_part(self._cpp_object, factor)
        return self

    @validate_call
    def translate(self, offset: NDArray[Shape["3"], np.float64]):
        """Translate the scene part by an offset."""

        _helios.translate_scene_part(self._cpp_object, offset)
        return self

    @classmethod
    @validate_call
    def from_xml(cls, scene_part_file: AssetPath, id: int):

        # Validate the XML
        validate_xml_file(scene_part_file, "xsd/scene.xsd")

        _cpp_scene_part = _helios.read_scene_part_from_xml(
            str(scene_part_file), [str(p) for p in get_asset_directories()], id
        )
        return cls._from_cpp(_cpp_scene_part)

    @classmethod
    @validate_call
    def from_obj(cls, obj_file: AssetPath, up_axis: Literal["y", "z"] = "z"):
        """Load the scene part from an OBJ file.

        For paths (potentially) containing wildcards, use 'ScenePart.from_objs()' instead!
        """

        _cpp_part = _helios.read_obj_scene_part(
            str(obj_file), [str(p) for p in get_asset_directories()], up_axis
        )

        return cls._from_cpp(_cpp_part)
    

    @classmethod
    @validate_call
    def from_tiff(cls, tiff_file: AssetPath):
        """Load the scene part from a TIFF file."""

        _cpp_part = _helios.read_tiff_scene_part(str(tiff_file))

        return cls._from_cpp(_cpp_part)


    @classmethod
    @validate_call
    def from_objs(cls, obj_files: MultiAssetPath, up_axis: Literal["y", "z"] = "z"):
        """Load multiple scene parts from OBJ files

        Expects a single Path containing some (or none) wildcards ('*').
        Supports '**' for matching multiple directories.
        """
        return [ScenePart.from_obj(obj, up_axis) for obj in obj_files]
    
    
    @classmethod
    @validate_call
    def from_tiffs(cls, tiff_files: MultiAssetPath):
        """Load multiple scene parts from TIFF files

        Expects a single Path containing some (or none) wildcards ('*').
        Supports '**' for matching multiple directories.
        """
        return [ScenePart.from_tiff(tiff) for tiff in tiff_files]
    

    @classmethod
    @validate_call
    def from_xyz(
        cls,
        xyz_file: AssetPath, 
        voxel_size: PositiveFloat,
        separator: Optional[str] = None,
        max_color_value: NonNegativeFloat = 0.0, 
        default_normal: NDArray[Shape["3"], np.float64] = np.array([np.finfo(np.float64).max] * 3, dtype=np.float64),
        sparse: bool = False,
        estimate_normals: bool = False,
        normals_file_columns: list[NonNegativeInt] = [3, 4, 5],
        rgb_file_columns: list[NonNegativeInt] = [6, 7, 8],
        snap_neighbor_normal: bool = False,
    ):
        """Load the scene part from an XYZ file."""
        
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

        return cls._from_cpp(_cpp_part)
    

    @classmethod
    @validate_call
    def from_xyzs(
        cls,
        xyz_files: MultiAssetPath,
        voxel_size: PositiveFloat,
        separator: Optional[str] = None,
        max_color_value: NonNegativeFloat = 0.0,
        default_normal: NDArray[Shape["3"], np.float64] = np.array([np.finfo(np.float64).max] * 3, dtype=np.float64),
        sparse: bool = False,
        estimate_normals: bool = False,
        normals_file_columns: list[NonNegativeInt] = [3, 4, 5],
        rgb_file_columns: list[NonNegativeInt] = [6, 7, 8],
        snap_neighbor_normal: bool = False,
    ):
        """
        Load multiple scene parts from XYZ files.

        Each parameters hould be a single shared value for all files.
        """
        return [
            ScenePart.from_xyz(xyz, voxel_size, separator, max_color_value, default_normal, 
                               sparse, estimate_normals, normals_file_columns, 
                               rgb_file_columns, snap_neighbor_normal)
            for xyz in xyz_files
            ]
    

    @classmethod
    @validate_call
    def from_vox(
        cls,
        vox_file: AssetPath,
        intersection_mode: Literal["scaled", "fixed"],
        intersection_argument: Optional[float] = None,
        random_shift: bool = False,
        ladlut_path: Optional[str] = None,
    ):
        """Load the scene part from a VOX file."""

        if intersection_mode == "fixed" and intersection_argument is not None:
            raise ValueError("'intersection_argument' must not be provided when 'intersection_mode' is 'fixed'.")

        _cpp_part = _helios.read_vox_scene_part(
            str(vox_file),
            [str(p) for p in get_asset_directories()],
            intersection_mode,
            intersection_argument if intersection_argument is not None else 0.0,
            random_shift,
            ladlut_path if ladlut_path is not None else ""
        )

        return cls._from_cpp(_cpp_part)


class StaticScene(Model, cpp_class=_helios.StaticScene):
    scene_parts: list[ScenePart] = []

    def _finalize(
        self, execution_settings: Optional[ExecutionSettings] = None, **parameters
    ):
        """Finalize the scene, making it ready for rendering."""

        if len(self._cpp_object.primitives) == 0:
            execution_settings = compose_execution_settings(
                execution_settings, parameters
            )
            _helios.finalize_static_scene(
                self._cpp_object,
                execution_settings.parallelization,
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
        if field == "scene_parts":
            self._enforce_uniqueness_across_instances(field, value)

    def _post_set(self, field):
        # When the Scene changes, we want to invalidate the KDTree etc.

        _helios.invalidate_static_scene(self._cpp_object)

    @classmethod
    @validate_call
    def from_xml(cls, scene_file: AssetPath):

        # Validate the XML
        validate_xml_file(scene_file, "xsd/scene.xsd")

        _cpp_scene = _helios.read_scene_from_xml(
            str(scene_file), [str(p) for p in get_asset_directories()], True, True
        )
        return cls._from_cpp(_cpp_scene)

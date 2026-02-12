from helios.settings import (
    ExecutionSettings,
    compose_execution_settings,
    ForceOnGroundStrategy,
)
from helios.utils import (
    get_asset_directories,
    detect_separator,
    is_xml_loaded,
    is_binary_loaded,
    is_finalized,
)

from helios.validation import (
    Angle,
    AssetPath,
    Model,
    MultiAssetPath,
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
from typing import Literal, Optional, Union, Tuple

import numpy as np

import _helios


class Material(Model, UpdateableMixin, cpp_class=_helios.Material):
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


class ScenePart(Model, cpp_class=_helios.ScenePart):
    force_on_ground: Union[ForceOnGroundStrategy, PositiveInt] = (
        ForceOnGroundStrategy.NONE
    )

    @property
    def materials(self) -> MaterialDict:
        """Dictionary-like access to materials in the scene part."""
        return MaterialDict(self)

    @validate_call
    def rotate(
        self,
        quaternion: Optional[NDArray[Shape["4"], np.float64]] = None,
        axis: Optional[NDArray[Shape["3"], np.float64]] = None,
        angle: Optional[Angle] = None,
        from_axis: Optional[NDArray[Shape["3"], np.float64]] = None,
        to_axis: Optional[NDArray[Shape["3"], np.float64]] = None,
        rotation_center: Optional[NDArray[Shape["3"], np.float64]] = None,
    ):
        """Rotate the scene part.

        The rotation can be specified as one of the following parameters:
        * A quaternion
        * An axis and an angle
        * An origin and an image vector

        Optionally, you may specify a rotation center. If omitted the origin
        of the coordinate system of the scene part will be used.
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
            self.translate(-rotation_center)

        # Perform the actual rotation
        _helios.rotate_scene_part(self._cpp_object, rot)

        # Undo the shift by the rotation center
        if rotation_center is not None:
            self.translate(rotation_center)

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
        scene_part = cls._from_cpp(_cpp_scene_part)
        scene_part._is_loaded_from_xml = True
        return scene_part

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

        _cpp_part = _helios.read_tiff_scene_part(
            str(tiff_file), [str(p) for p in get_asset_directories()]
        )

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
        default_normal: NDArray[Shape["3"], np.float64] = np.array(
            [np.finfo(np.float64).max] * 3, dtype=np.float64
        ),
        sparse: bool = True,
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
        default_normal: NDArray[Shape["3"], np.float64] = np.array(
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

        Each parameters hould be a single shared value for all files.
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

        return cls._from_cpp(_cpp_part)

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
        * 'range_start' & 'range_stop': an elevation range to update

        If neither is provided, the material will be applied to all primitives.
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
    scene_parts: Tuple[ScenePart, ...] = ()

    def add_scene_part(self, scene_part: ScenePart):
        """Add a scene part to the scene."""
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
        if is_xml_loaded(self) or is_binary_loaded(self):
            raise RuntimeError("The scene loaded from XML cannot be modified.")
        if field == "scene_parts":
            self._enforce_uniqueness_across_instances(field, value)

    def _post_set(self, field):
        # When the Scene changes after initialization, we want to invalidate the KDTree etc.
        if not self._during_init:
            _helios.invalidate_static_scene(self._cpp_object)

    @classmethod
    @validate_call
    def from_binary(cls, filename: AssetPath):
        _cpp_scene = _helios.read_scene_from_binary(str(filename))
        scene = cls._from_cpp(_cpp_scene)
        scene._is_loaded_from_binary = True
        return scene

    def to_binary(self, filename: AssetPath, is_dyn_scene: bool = False):
        if not (is_xml_loaded(self) or is_binary_loaded(self)):
            self._finalize()

        _helios.write_scene_to_binary(str(filename), self._cpp_object, is_dyn_scene)

    @classmethod
    @validate_call
    def from_xml(cls, scene_file: AssetPath, save_to_binary: bool = False):
        # Validate the XML
        validate_xml_file(scene_file, "xsd/scene.xsd")

        _cpp_scene = _helios.read_scene_from_xml(
            str(scene_file), [str(p) for p in get_asset_directories()], save_to_binary
        )
        scene = cls._from_cpp(_cpp_scene)
        scene._is_loaded_from_xml = True
        return scene

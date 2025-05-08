from helios.settings import (
    ExecutionSettings,
    compose_execution_settings,
    ForceOnGroundStrategy,
)
from helios.utils import get_asset_directories, detect_separator, strip_asset_prefix
from helios.validation import AssetPath, Model, MultiAssetPath, validate_xml_file

from numpydantic import NDArray, Shape
from pydantic import (
    PositiveFloat,
    PositiveInt,
    NonNegativeFloat,
    NonNegativeInt,
    validate_call,
)
from typing import Literal, Optional, Union

import numpy as np

import _helios


class ScenePart(Model, cpp_class=_helios.ScenePart):
    force_on_ground: Union[ForceOnGroundStrategy, PositiveInt] = (
        ForceOnGroundStrategy.NONE
    )
    is_ground: bool = False
    classification: int = 0

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

        # Record the rotation in the provenance
        self._provenance.setdefault("transformations", [])

        # The rotation object that we want to construct
        rot = None

        # Handle construction via a given quaternion
        if quaternion is not None:
            rot = _helios.Rotation(
                quaternion[0], quaternion[1], quaternion[2], quaternion[3], True
            )

            self._provenance["transformations"].append(
                {
                    "rotate": {
                        "quaternion": quaternion.tolist(),
                    },
                }
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

            self._provenance["transformations"].append(
                {
                    "rotate": {
                        "axis": axis.tolist(),
                        "angle": angle,
                    },
                }
            )

        # Handle construction via two vectors
        if origin is not None or image is not None:
            if rot is not None:
                raise ValueError("Too many rotation parameters specified")
            if origin is None:
                raise ValueError("Origin must be specified when image is specified")
            if image is None:
                raise ValueError("Image must be specified when origin is specified")

            rot = _helios.Rotation(origin, image)

            self._provenance["transformations"].append(
                {
                    "rotate": {
                        "origin": origin.tolist(),
                        "image": image.tolist(),
                    },
                }
            )

        # If no rotation was specified, raise an error
        if rot is None:
            raise ValueError("No rotation parameters specified")

        # Perform the actual rotation
        _helios.rotate_scene_part(self._cpp_object, rot)

        return self

    @validate_call
    def scale(self, factor: PositiveFloat):
        """Scale the scene part by a factor."""

        _helios.scale_scene_part(self._cpp_object, factor)

        # Record the scaling in the provenance
        self._provenance.setdefault("transformations", [])
        self._provenance["transformations"].append(
            {
                "scale": {
                    "factor": factor,
                },
            }
        )

        return self

    @validate_call
    def translate(self, offset: NDArray[Shape["3"], np.float64]):
        """Translate the scene part by an offset."""

        _helios.translate_scene_part(self._cpp_object, offset)

        # Record the translation in the provenance
        self._provenance.setdefault("transformations", [])
        self._provenance["transformations"].append(
            {
                "translate": {
                    "offset": offset.tolist(),
                },
            }
        )

        return self

    @classmethod
    @validate_call
    def from_xml(cls, scene_part_file: AssetPath, id: int):

        # Validate the XML
        validate_xml_file(scene_part_file, "xsd/scene.xsd")

        _cpp_scene_part = _helios.read_scene_part_from_xml(
            str(scene_part_file), [str(p) for p in get_asset_directories()], id
        )
        obj = cls._from_cpp(_cpp_scene_part)

        obj._provenance = {
            "from_xml": {
                "scene_part_file": str(strip_asset_prefix(scene_part_file)),
                "id": id,
            }
        }

        return obj

    @classmethod
    @validate_call
    def from_obj(cls, obj_file: AssetPath, up_axis: Literal["y", "z"] = "z"):
        """Load the scene part from an OBJ file.

        For paths (potentially) containing wildcards, use 'ScenePart.from_objs()' instead!
        """

        _cpp_part = _helios.read_obj_scene_part(
            str(obj_file), [str(p) for p in get_asset_directories()], up_axis
        )

        obj = cls._from_cpp(_cpp_part)

        obj._provenance = {
            "from_obj": {
                "obj_file": str(strip_asset_prefix(obj_file)),
                "up_axis": up_axis,
            }
        }

        return obj

    @classmethod
    @validate_call
    def from_tiff(cls, tiff_file: AssetPath):
        """Load the scene part from a TIFF file."""

        _cpp_part = _helios.read_tiff_scene_part(str(tiff_file))

        obj = cls._from_cpp(_cpp_part)

        obj._provenance = {
            "from_tiff": {
                "tiff_file": str(strip_asset_prefix(tiff_file)),
            }
        }

        return obj

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

        obj = cls._from_cpp(_cpp_part)

        obj._provenance = {
            "from_xyz": {
                "xyz_file": str(strip_asset_prefix(xyz_file)),
                "separator": separator,
                "voxel_size": voxel_size,
                "max_color_value": max_color_value,
                "default_normal": default_normal.tolist(),
                "sparse": sparse,
                "estimate_normals": estimate_normals,
                "normals_file_columns": normals_file_columns,
                "rgb_file_columns": rgb_file_columns,
                "snap_neighbor_normal": snap_neighbor_normal,
            }
        }

        return obj

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

        obj = cls._from_cpp(_cpp_part)

        obj._provenance = {
            "from_vox": {
                "vox_file": str(strip_asset_prefix(vox_file)),
                "intersection_mode": intersection_mode,
                "intersection_argument": intersection_argument,
                "random_shift": random_shift,
                "ladlut_path": ladlut_path,
            }
        }

        return obj

    @classmethod
    def _from_dict(cls, d):
        part = None
        if "from_xml" in d:
            part = cls.from_xml(**d.pop("from_xml"))
        if "from_obj" in d:
            part = cls.from_obj(**d.pop("from_obj"))
        if "from_tiff" in d:
            part = cls.from_tiff(**d.pop("from_tiff"))
        if "from_xyz" in d:
            part = cls.from_xyz(**d.pop("from_xyz"))
        if "from_vox" in d:
            part = cls.from_vox(**d.pop("from_vox"))

        if part is None:
            raise ValueError(
                "Error deserializing ScenePart: No valid source found in the dictionary."
            )

        if "transformations" in d:
            for transformation in d.pop("transformations"):
                if "rotate" in transformation:
                    part = part.rotate(**transformation["rotate"])
                    continue
                if "scale" in transformation:
                    part = part.scale(**transformation["scale"])
                    continue
                if "translate" in transformation:
                    part = part.translate(**transformation["translate"])
                    continue

                raise ValueError(
                    "Error deserializing ScenePart: Unknown transformation found in the dictionary."
                )

        for key, value in d.items():
            setattr(part, key, value)

        return part


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
    def from_binary(cls, filename: AssetPath):
        _cpp_scene = _helios.read_scene_from_binary(str(filename))
        obj = cls._from_cpp(_cpp_scene)

        obj._provenance = {
            "from_binary": {
                "filename": str(strip_asset_prefix(filename)),
            }
        }

        return obj

    def to_binary(self, filename: AssetPath, is_dyn_scene: bool = False):
        _helios.write_scene_to_binary(str(filename), self._cpp_object, is_dyn_scene)

    @classmethod
    @validate_call
    def from_xml(cls, scene_file: AssetPath):

        # Validate the XML
        validate_xml_file(scene_file, "xsd/scene.xsd")

        _cpp_scene = _helios.read_scene_from_xml(
            str(scene_file), [str(p) for p in get_asset_directories()], True
        )

        obj = cls._from_cpp(_cpp_scene)
        obj._provenance = {
            "from_xml": {
                "scene_file": str(strip_asset_prefix(scene_file)),
            }
        }
        return obj

    @classmethod
    def _from_dict(cls, d):
        scene = None
        if "from_xml" in d:
            scene = cls.from_xml(**d.pop("from_xml"))
        if "from_binary" in d:
            scene = cls.from_binary(**d.pop("from_binary"))
        if scene is None:
            scene = cls()

        for key, value in d.items():
            setattr(scene, key, value)

        return scene

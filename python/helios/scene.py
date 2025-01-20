from helios.util import get_asset_directories
from helios.validation import Validatable, ValidatedCppManagedProperty
from pathlib import Path

import _helios


class ScenePart(Validatable):
    @classmethod
    def from_xml(cls, scene_part_file: Path):
        raise NotImplementedError


class Scene(Validatable):
    scene_parts: list[ScenePart] = ValidatedCppManagedProperty(
        "scene_parts", ScenePart, iterable=True
    )

    @classmethod
    def from_xml(cls, scene_file: Path):
        _cpp_scene = _helios.read_scene_from_xml(
            scene_file, [str(p) for p in get_asset_directories()], True, True
        )
        return cls._from_cpp_object(_cpp_scene)

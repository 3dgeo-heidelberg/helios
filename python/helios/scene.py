from helios.util import get_asset_directories
from helios.validation import AssetPath, Model, Property
from pydantic import validate_call

import _helios


class ScenePart(Model, cpp_class=_helios.ScenePart):
    @classmethod
    @validate_call
    def from_xml(cls, scene_part_file: AssetPath, id: int):
        _cpp_scene_part = _helios.read_scene_part_from_xml(
            str(scene_part_file), [str(p) for p in get_asset_directories()], id
        )
        return cls.__new__(cls, _cpp_object=_cpp_scene_part)


class Scene(Model, cpp_class=_helios.Scene):
    scene_parts: list[ScenePart] = Property(
        cpp="scene_parts", wraptype=ScenePart, iterable=True, default=[]
    )

    @classmethod
    @validate_call
    def from_xml(cls, scene_file: AssetPath):
        _cpp_scene = _helios.read_scene_from_xml(
            str(scene_file), [str(p) for p in get_asset_directories()], True, True
        )
        return cls.__new__(cls, _cpp_object=_cpp_scene)

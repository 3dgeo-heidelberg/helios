from helios.util import get_asset_directories
from helios.validation import Model, Property, validate_xml_file
from pathlib import Path

import _helios


class ScenePart(Model, cpp_class=_helios.ScenePart):
    @classmethod
    def from_xml(cls, scene_part_file: Path, id: int):

        # Validate the XML
        validate_xml_file(scene_part_file, "xsd/scene.xsd")

        _cpp_scene_part = _helios.read_scene_part_from_xml(
            scene_part_file, [str(p) for p in get_asset_directories()], id
        )
        return cls.__new__(cls, _cpp_object=_cpp_scene_part)


class Scene(Model, cpp_class=_helios.Scene):
    scene_parts: list[ScenePart] = Property(
        cpp="scene_parts", wraptype=ScenePart, iterable=True, default=[]
    )

    @classmethod
    def from_xml(cls, scene_file: Path):

        # Validate the XML
        validate_xml_file(scene_file, "xsd/scene.xsd")

        _cpp_scene = _helios.read_scene_from_xml(
            scene_file, [str(p) for p in get_asset_directories()], True, True
        )
        return cls.__new__(cls, _cpp_object=_cpp_scene)

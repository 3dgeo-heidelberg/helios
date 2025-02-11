from helios.util import get_asset_directories
from helios.validation import AssetPath, Model, Property, validate_xml_file

from pydantic import validate_call

import _helios


class ScenePart(Model, cpp_class=_helios.ScenePart):
    @classmethod
    @validate_call
    def from_xml(cls, scene_part_file: AssetPath, id: int):

        # Validate the XML
        validate_xml_file(scene_part_file, "xsd/scene.xsd")

        _cpp_scene_part = _helios.read_scene_part_from_xml(
            str(scene_part_file), [str(p) for p in get_asset_directories()], id
        )
        return cls.__new__(cls, _cpp_object=_cpp_scene_part)


class StaticScene(Model, cpp_class=_helios.StaticScene):
    scene_parts: list[ScenePart] = Property(
        cpp="scene_parts", wraptype=ScenePart, iterable=True, default=[]
    )

    def finalize(self):
        """Finalize the scene, making it ready for rendering."""

        if len(self._cpp_object.primitives) == 0:
            _helios.finalize_static_scene(self._cpp_object, 4, 1, 1, 32)

    def _update_hook(self):
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
        return cls.__new__(cls, _cpp_object=_cpp_scene)

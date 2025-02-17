from helios.scene import *

import pytest


def test_construct_scene_from_xml():
    scene = StaticScene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")

    assert len(scene.scene_parts) == 5


def test_construct_scene_part_from_xml():
    part = ScenePart.from_xml("data/scenes/toyblocks/toyblocks_scene.xml", id="0")


def test_finalize_scene():
    part = ScenePart.from_xml("data/scenes/toyblocks/toyblocks_scene.xml", id="0")

    scene = StaticScene(scene_parts=[part])
    assert len(scene._cpp_object.primitives) == 0
    scene._finalize()
    assert len(scene._cpp_object.primitives) > 0


def test_scene_invalidation():
    part = ScenePart.from_xml("data/scenes/toyblocks/toyblocks_scene.xml", id="0")
    part2 = ScenePart.from_xml("data/scenes/toyblocks/toyblocks_scene.xml", id="0")
    scene = StaticScene(scene_parts=[part])
    scene._finalize()

    scene.scene_parts = [part, part2]
    assert len(scene._cpp_object.primitives) == 0
    scene._finalize()
    assert len(scene._cpp_object.primitives) > 0


def test_scenepart_from_obj():
    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene = StaticScene(scene_parts=[box])
    scene._finalize()


def test_scenepart_from_obj_yisup():
    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene = StaticScene(scene_parts=[box], up_axis="y")
    scene._finalize()


def test_scenepart_from_obj_wrong_axis_argument():
    with pytest.raises(ValueError):
        ScenePart.from_obj("data/sceneparts/basic/box/box100.obj", up_axis="x")

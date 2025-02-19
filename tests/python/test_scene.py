from helios.scene import *

import numpy as np
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


def test_scale_scenepart(box_f):
    box1 = box_f()
    box2 = box_f()

    def get_bbox(part):
        scene = StaticScene(scene_parts=[part])
        scene._finalize()
        return np.array(scene._cpp_object.bbox.bounds)

    bbox1 = get_bbox(box1)
    box2.scale(2.0)
    bbox2 = get_bbox(box2)

    assert np.allclose(bbox1 * 2, bbox2)

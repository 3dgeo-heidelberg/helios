from helios.scene import *

import math
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


def test_sceneparts_from_obj_wildcard():
    box = ScenePart.from_objs("data/sceneparts/basic/**/*.obj")
    assert len(box) == 4
    scene = StaticScene(scene_parts=box)
    scene._finalize()


def test_scenepart_from_obj_yisup():
    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene = StaticScene(scene_parts=[box], up_axis="y")
    scene._finalize()


def test_scenepart_from_obj_wrong_axis_argument():
    with pytest.raises(ValueError):
        ScenePart.from_obj("data/sceneparts/basic/box/box100.obj", up_axis="x")


def test_scenepart_from_tiff():
    scene_part = ScenePart.from_tiff("data/sceneparts/tiff/dem_hd.tif")
    assert len(scene_part._cpp_object.primitives) > 0


def get_bbox(part):
    scene = StaticScene(scene_parts=[part])
    scene._finalize()
    return np.array(scene._cpp_object.bbox_crs.bounds)


def check_rotation(box1, box2):
    bbox1 = get_bbox(box1)
    bbox1[0][1] = bbox1[0][1] * math.sqrt(2)
    bbox1[0][2] = bbox1[0][2] * math.sqrt(2)
    bbox1[1][1] = bbox1[1][1] * math.sqrt(2)
    bbox1[1][2] = bbox1[1][2] * math.sqrt(2)

    bbox2 = get_bbox(box2)
    assert np.allclose(bbox1, bbox2)


def test_rotate_scenepart_axis_angle(box_f):
    box1 = box_f()
    box2 = box_f()

    box2.rotate(axis=[1.0, 0.0, 0.0], angle=np.pi / 4)

    check_rotation(box1, box2)


def test_rotate_scenepart_quaternion(box_f):
    box1 = box_f()
    box2 = box_f()

    box2.rotate(quaternion=[0.9238795325112867, -0.3826834323650898, 0.0, 0.0])

    check_rotation(box1, box2)


def test_rotate_scenepart_two_vectors(box_f):
    box1 = box_f()
    box2 = box_f()

    box2.rotate(origin=np.array([0, 1.0, 0]), image=np.array([0.0, 1.0, 1.0]))

    check_rotation(box1, box2)


def test_rotate_scenepart_no_parameter(box):
    with pytest.raises(ValueError):
        box.rotate()


def test_rotate_scenepart_too_many_parameters(box):
    with pytest.raises(ValueError):
        box.rotate(quaternion=[1.0, 0, 0, 0], axis=[1.0, 0, 0])

    with pytest.raises(ValueError):
        box.rotate(quaternion=[1.0, 0, 0, 0], origin=[1.0, 0, 0])

    with pytest.raises(ValueError):
        box.rotate(axis=[1.0, 0, 0], angle=0.0, origin=[0, 0, 0])


def test_rotate_scenepart_missing_parameters(box):
    with pytest.raises(ValueError):
        box.rotate(axis=[1.0, 0, 0])

    with pytest.raises(ValueError):
        box.rotate(angle=0.0)

    with pytest.raises(ValueError):
        box.rotate(origin=[0.0, 0.0, 0.0])

    with pytest.raises(ValueError):
        box.rotate(image=[1.0, 0, 0])


def test_scale_scenepart(box_f):
    box1 = box_f()
    box2 = box_f()

    scale = 2.0
    box2.scale(scale)

    bbox1 = get_bbox(box1)
    bbox2 = get_bbox(box2)

    assert np.allclose(bbox1 * scale, bbox2)


def test_transform_scenepart(box_f):
    box1 = box_f()
    box2 = box_f()

    offset = np.array([10.0, 0, 0])
    box2.translate(offset)

    bbox1 = get_bbox(box1)
    bbox2 = get_bbox(box2)

    assert np.allclose(bbox1 + offset, bbox2)

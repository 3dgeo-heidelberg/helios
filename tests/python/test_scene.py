from helios.scene import *
from helios.settings import *
from helios.survey import *

import math
import numpy as np
import pytest

from helios import HeliosException

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


def test_scenepart_from_tiffs():
    scene_parts = ScenePart.from_tiffs("data/test/*.tif")
    assert len(scene_parts) == 2

def test_scenepart_from_xyz():
    scene_part1 = ScenePart.from_xyz(
        "data/sceneparts/pointclouds/sphere_dens25000.xyz", separator=" ", voxel_size=1.0,
          max_color_value=255.0, default_normal=[0.0, 0.0, 1.0])
    
    scene_part2 = ScenePart.from_xyz(
        "data/sceneparts/pointclouds/sphere_dens25000_sepcomma.xyz", separator=",", voxel_size=1.0,
          max_color_value=255.0)
    
    scene_part3 = ScenePart.from_xyz(
        "data/sceneparts/pointclouds/sphere_dens25000.xyz", separator=" ", voxel_size=1.0)
    
    scene_part4 = ScenePart.from_xyz(
        "data/sceneparts/pointclouds/sphere_dens25000.xyz", voxel_size=1.0,
          default_normal=[0.0, 0.0, 1.0], sparse=True)
    
    assert len(scene_part1._cpp_object.primitives) > 0
    assert len(scene_part2._cpp_object.primitives) > 0
    assert len(scene_part3._cpp_object.primitives) > 0
    assert len(scene_part4._cpp_object.primitives) > 0

def test_scenepart_from_xyzs():
    scene_parts1 = ScenePart.from_xyzs("data/sceneparts/pointclouds/*.xyz", voxel_size= 1.0)
                                         
    scene_parts2 = ScenePart.from_xyzs("data/sceneparts/pointclouds/*.xyz",
                            voxel_size= 1.0, max_color_value= 255.0, default_normal= [0.0, 0.0, 1.0]) 
                               
    assert len(scene_parts1) > 2
    assert len(scene_parts2) > 2

    with pytest.raises(HeliosException, match="separator mismatch"):
        ScenePart.from_xyzs(
            "data/sceneparts/pointclouds/*.xyz",
            voxel_size=1.0,
            separator=",",
        )

def test_scenepart_from_vox():
    scene_parts1 = ScenePart.from_vox("data/sceneparts/syssifoss/F_BR08_08_crown_250.vox", intersection_mode="fixed")
    scene_parts2 = ScenePart.from_vox("data/sceneparts/syssifoss/F_BR08_08_merged.vox", intersection_mode="scaled", intersection_argument=0.5)
    scene_parts3 = ScenePart.from_vox("data/sceneparts/syssifoss/F_BR08_08_merged.vox", intersection_mode="scaled")

    assert len(scene_parts1._cpp_object.primitives) > 0
    assert len(scene_parts2._cpp_object.primitives) > 0
    assert len(scene_parts3._cpp_object.primitives) > 0    

    with pytest.raises(ValueError):
        ScenePart.from_vox("data/sceneparts/syssifoss/F_BR08_08_crown_250.vox", intersection_mode="fixed", intersection_argument=0.1)
    
    
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


def test_ground_plane():
    sp1 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    sp1.rotate(axis=np.array([0, 0, 1.0]), angle=np.pi / 4)
    sp1.scale(0.5)
    sp1.translate(np.array([-45.0, 10.0, 10]))
    sp1.force_on_ground = 1 #ForceOnGroundStrategy.LEAST_COMPLEX
    
    sp2 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    sp2.scale(1)

    sp3 = ScenePart.from_obj("data/sceneparts/basic/groundplane/groundplane.obj")
    sp3.scale(70)
    sp3.translate(np.array([20.0, 0, 0]))

    sp4 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    sp4.rotate(axis=np.array([0, 0, 1.0]), angle=np.pi / 4)
    sp4.scale(0.5)
    sp4.translate(np.array([-45.0, 10.0, 10]))
    sp4.force_on_ground = 0 #ForceOnGroundStrategy.NONE

    sp5 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    sp5.rotate(axis=np.array([0, 0, 1.0]), angle=np.pi / 4)
    sp5.scale(0.5)
    sp5.translate(np.array([-45.0, 10.0, 10]))
    sp5.force_on_ground = -1
    
    scene = StaticScene(scene_parts=[sp1, sp2, sp3, sp4, sp5])
    scene._finalize()

    assert np.isclose(sp1._cpp_object.all_vertices[0].position[2], sp3._cpp_object.all_vertices[0].position[2])
    assert not np.isclose(sp1._cpp_object.all_vertices[0].position[2], sp4._cpp_object.all_vertices[0].position[2])
    assert np.isclose(sp1._cpp_object.all_vertices[0].position[2], sp5._cpp_object.all_vertices[0].position[2])
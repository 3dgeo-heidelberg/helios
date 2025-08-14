from helios.scene import *
from helios.settings import *
from helios.survey import *
from helios.settings import *
from helios.survey import *
from helios.platforms import *
from helios.scanner import *
from helios.utils import *

import os
import math
import numpy as np
import pytest
import laspy
import shutil

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
    box = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj", up_axis="y")
    scene = StaticScene(scene_parts=[box])
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
        "data/sceneparts/pointclouds/sphere_dens25000.xyz",
        separator=" ",
        voxel_size=1.0,
        max_color_value=255.0,
        default_normal=[0.0, 0.0, 1.0],
    )

    scene_part2 = ScenePart.from_xyz(
        "data/sceneparts/pointclouds/sphere_dens25000_sepcomma.xyz",
        separator=",",
        voxel_size=1.0,
        max_color_value=255.0,
    )

    scene_part3 = ScenePart.from_xyz(
        "data/sceneparts/pointclouds/sphere_dens25000.xyz",
        separator=" ",
        voxel_size=1.0,
    )

    scene_part4 = ScenePart.from_xyz(
        "data/sceneparts/pointclouds/sphere_dens25000.xyz",
        voxel_size=1.0,
        default_normal=[0.0, 0.0, 1.0],
        sparse=True,
    )

    assert len(scene_part1._cpp_object.primitives) > 0
    assert len(scene_part2._cpp_object.primitives) > 0
    assert len(scene_part3._cpp_object.primitives) > 0
    assert len(scene_part4._cpp_object.primitives) > 0


def test_scenepart_from_xyzs():
    scene_parts1 = ScenePart.from_xyzs(
        "data/sceneparts/pointclouds/*.xyz", voxel_size=1.0
    )

    scene_parts2 = ScenePart.from_xyzs(
        "data/sceneparts/pointclouds/*.xyz",
        voxel_size=1.0,
        max_color_value=255.0,
        default_normal=[0.0, 0.0, 1.0],
    )

    assert len(scene_parts1) > 2
    assert len(scene_parts2) > 2

    with pytest.raises(HeliosException, match="separator mismatch"):
        ScenePart.from_xyzs(
            "data/sceneparts/pointclouds/*.xyz",
            voxel_size=1.0,
            separator=",",
        )


def test_scenepart_from_vox():
    scene_parts1 = ScenePart.from_vox(
        "data/sceneparts/syssifoss/F_BR08_08_crown_250.vox", intersection_mode="fixed"
    )
    scene_parts2 = ScenePart.from_vox(
        "data/sceneparts/syssifoss/F_BR08_08_merged.vox",
        intersection_mode="scaled",
        intersection_argument=0.5,
    )
    scene_parts3 = ScenePart.from_vox(
        "data/sceneparts/syssifoss/F_BR08_08_merged.vox", intersection_mode="scaled"
    )

    assert len(scene_parts1._cpp_object.primitives) > 0
    assert len(scene_parts2._cpp_object.primitives) > 0
    assert len(scene_parts3._cpp_object.primitives) > 0

    with pytest.raises(ValueError):
        ScenePart.from_vox(
            "data/sceneparts/syssifoss/F_BR08_08_crown_250.vox",
            intersection_mode="fixed",
            intersection_argument=0.1,
        )


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

    box2.rotate(from_axis=np.array([0, 1.0, 0]), to_axis=np.array([0.0, 1.0, 1.0]))

    check_rotation(box1, box2)


def test_rotate_scenepart_no_parameter(box):
    with pytest.raises(ValueError):
        box.rotate()


def test_rotate_scenepart_too_many_parameters(box):
    with pytest.raises(ValueError):
        box.rotate(quaternion=[1.0, 0, 0, 0], axis=[1.0, 0, 0])

    with pytest.raises(ValueError):
        box.rotate(quaternion=[1.0, 0, 0, 0], from_axis=[1.0, 0, 0])

    with pytest.raises(ValueError):
        box.rotate(axis=[1.0, 0, 0], angle=0.0, from_axis=[0, 0, 0])


def test_rotate_scenepart_missing_parameters(box):
    with pytest.raises(ValueError):
        box.rotate(axis=[1.0, 0, 0])

    with pytest.raises(ValueError):
        box.rotate(angle=0.0)

    with pytest.raises(ValueError):
        box.rotate(from_axis=[0.0, 0.0, 0.0])

    with pytest.raises(ValueError):
        box.rotate(to_axis=[1.0, 0, 0])


def test_rotate_scenepart_rotation_center(box_f):
    box1 = box_f()
    box2 = box_f()

    box1.rotate(angle="180 deg", axis=[0, 0, 1.0], rotation_center=[100.0, 0.0, 0.0])
    box2.translate([200.0, 0.0, 0.0])
    bbox1 = get_bbox(box1)
    bbox2 = get_bbox(box2)

    assert np.allclose(bbox1, bbox2)


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


def test_scene_auto_binary():
    # We create binary while reading from xml
    survey1 = Survey.from_xml("data/surveys/toyblocks/tls_toyblocks.xml", True, True)
    points1, _ = survey1.run()

    survey2 = Survey.from_xml("data/surveys/toyblocks/tls_toyblocks.xml")
    survey2.scene = StaticScene.from_binary(
        "data/scenes/toyblocks/toyblocks_scene.scene"
    )
    points2, _ = survey2.run()

    # We read from created binary directly through .from_xml
    survey3 = Survey.from_xml("data/surveys/toyblocks/tls_toyblocks.xml", False)
    points3, _ = survey3.run()

    os.remove("data/scenes/toyblocks/toyblocks_scene.scene")
    assert len(points1) == len(points2)
    assert len(points1) == len(points3)


def test_scene_manual_binary():
    scene_part1 = ScenePart.from_obj(
        "data/sceneparts/basic/groundplane/groundplane.obj"
    )
    scene_part1.scale(70)
    scene_part1.translate([20.0, 0.0, 0.0])
    scene_part2 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    scene_part2.scale(1)
    scene_part3 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    scene_part3.rotate(angle="45 deg", axis=[0, 0, 1.0])
    scene_part3.scale(0.5)
    scene_part3.force_on_ground = ForceOnGroundStrategy.MOST_COMPLEX
    scene_part3.translate([-45.0, 10.0, 10.0])
    scene_part4 = ScenePart.from_obj("data/sceneparts/toyblocks/sphere.obj")
    scene_part4.scale(0.5)
    scene_part5 = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    scene_part5.scale(1)

    scene1 = StaticScene(
        scene_parts=[scene_part1, scene_part2, scene_part3, scene_part4, scene_part5]
    )
    scene1.to_binary("data/scenes/toyblocks/toyblocks_scene_manual1.scene")

    scene2 = StaticScene.from_binary(
        "data/scenes/toyblocks/toyblocks_scene_manual1.scene"
    )
    platform_settings1 = StaticPlatformSettings(x=-26, y=-31, z=0, force_on_ground=True)
    platform_settings2 = StaticPlatformSettings(x=-35, y=35, z=0, force_on_ground=True)
    platform_settings3 = StaticPlatformSettings(x=60, y=0, z=0, force_on_ground=True)
    scanner_settings1 = ScannerSettings(
        is_active=True,
        pulse_frequency=100000,
        scan_frequency=120,
        min_vertical_angle="-40 deg",
        max_vertical_angle="60 deg",
        head_rotation="10 deg/s",
        rotation_start_angle="245 deg",
        rotation_stop_angle="360 deg",
    )
    scanner_settings2 = ScannerSettings(
        is_active=True,
        pulse_frequency=100000,
        scan_frequency=120,
        scan_angle="50 deg",
        head_rotation="10 deg/s",
        rotation_start_angle="180 deg",
        rotation_stop_angle="320 deg",
    )
    scanner_settings3 = ScannerSettings(
        is_active=True,
        pulse_frequency=100000,
        scan_frequency=120,
        min_vertical_angle="-40 deg",
        max_vertical_angle="60 deg",
        head_rotation="10 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="180 deg",
    )
    platform1 = tripod()
    scanner1 = riegl_vz_400()
    survey1 = Survey(scanner=scanner1, platform=platform1, scene=scene2)

    survey1.add_leg(
        scanner_settings=scanner_settings1, platform_settings=platform_settings1
    )
    survey1.add_leg(
        scanner_settings=scanner_settings2, platform_settings=platform_settings2
    )
    survey1.add_leg(
        scanner_settings=scanner_settings3, platform_settings=platform_settings3
    )
    points1, _ = survey1.run()

    platform_settings2_1 = StaticPlatformSettings(
        x=-26, y=-31, z=0, force_on_ground=True
    )
    platform_settings2_2 = StaticPlatformSettings(
        x=-35, y=35, z=0, force_on_ground=True
    )
    platform_settings2_3 = StaticPlatformSettings(x=60, y=0, z=0, force_on_ground=True)
    scanner_settings2_1 = ScannerSettings(
        is_active=True,
        pulse_frequency=100000,
        scan_frequency=120,
        min_vertical_angle="-40 deg",
        max_vertical_angle="60 deg",
        head_rotation="10 deg/s",
        rotation_start_angle="245 deg",
        rotation_stop_angle="360 deg",
    )
    scanner_settings2_2 = ScannerSettings(
        is_active=True,
        pulse_frequency=100000,
        scan_frequency=120,
        scan_angle="50 deg",
        head_rotation="10 deg/s",
        rotation_start_angle="180 deg",
        rotation_stop_angle="320 deg",
    )
    scanner_settings2_3 = ScannerSettings(
        is_active=True,
        pulse_frequency=100000,
        scan_frequency=120,
        min_vertical_angle="-40 deg",
        max_vertical_angle="60 deg",
        head_rotation="10 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="180 deg",
    )
    platform2 = tripod()
    scanner2 = riegl_vz_400()
    scene2 = StaticScene.from_binary(
        "data/scenes/toyblocks/toyblocks_scene_manual1.scene"
    )
    survey2 = Survey(scanner=scanner2, platform=platform2, scene=scene2)
    survey2.add_leg(
        scanner_settings=scanner_settings2_1, platform_settings=platform_settings2_1
    )
    survey2.add_leg(
        scanner_settings=scanner_settings2_2, platform_settings=platform_settings2_2
    )
    survey2.add_leg(
        scanner_settings=scanner_settings2_3, platform_settings=platform_settings2_3
    )
    points2, _ = survey2.run()
    assert len(points1) == len(points2)
    os.remove("data/scenes/toyblocks/toyblocks_scene_manual1.scene")


def test_ground_plane():
    """
    Test the force_on_groundfunctionality of the ScenePart class.
    This test verifies that scene parts are correctly adjusted vertically to the ground plane
    depending on their force_on_ground flag.

    Here, the following is tested:
    - Scene parts with not NONE force_on_ground value will have their z-coordinate translated
      to the ground plane's z-coordinate after finalization.
    - Scene parts with force_on_ground = NONE will not be vertically adjusted.
    - Scene parts with optional positive int value for force_on_ground will specify
      the number of search steps to be performed.
    """

    sp1 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    sp1.force_on_ground = ForceOnGroundStrategy.LEAST_COMPLEX

    sp2 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    sp2.force_on_ground = ForceOnGroundStrategy.NONE

    sp3 = ScenePart.from_obj("data/sceneparts/basic/groundplane/groundplane.obj")
    sp3.scale(70)

    sp4 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    sp4.force_on_ground = 5

    scene = StaticScene(scene_parts=[sp1, sp2, sp3, sp4])
    scene._finalize()

    assert np.isclose(
        sp1._cpp_object.all_vertices[0].position[2],
        sp3._cpp_object.all_vertices[0].position[2],
    )
    assert not np.isclose(
        sp2._cpp_object.all_vertices[0].position[2],
        sp3._cpp_object.all_vertices[0].position[2],
    )
    assert np.isclose(
        sp4._cpp_object.all_vertices[0].position[2],
        sp3._cpp_object.all_vertices[0].position[2],
    )


def test_is_ground():
    """
    Test the effect of the is_ground flag on the Z-coordinate adjustment of a scene part.
    If is_ground is set to False for the ground plane, indicating that there are no valid ground scene parts in the scene,
    any other scene part should not be adjusted or aligned to the ground plane.
    """

    sp1 = ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    sp1.force_on_ground = ForceOnGroundStrategy.MOST_COMPLEX

    sp2 = ScenePart.from_obj("data/sceneparts/basic/groundplane/groundplane.obj")
    sp2.scale(70)
    sp2.is_ground = False

    scene = StaticScene(scene_parts=[sp1, sp2])
    scene._finalize()

    assert not np.isclose(
        sp1._cpp_object.all_vertices[0].position[2],
        sp2._cpp_object.all_vertices[0].position[2],
    )


def test_classification_scenepart():
    """
    Test that the classification of a scene part can be set correctly and be used during run of Survey
    """
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")
    survey.scene.scene_parts[0].classification = 1
    assert survey.scene.scene_parts[0]._cpp_object.classification == 1

    meas, _ = survey.run()

    assert np.any(meas["classification"] == 1)


def test_add_scene_part():
    """
    Test that a scene part can be added to an existing scene.
    """
    scene = StaticScene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")
    assert len(scene.scene_parts) == 5

    new_part = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene.add_scene_part(new_part)

    assert len(scene.scene_parts) == 6
    assert len(scene._cpp_object.scene_parts) == 6
    assert new_part in scene.scene_parts


def test_add_scene_part_invalid():
    """
    Test that adding a duplicate of scene_part raises an error. As well as usage of append method.
    """
    scene = StaticScene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")
    scene2 = StaticScene()
    assert len(scene.scene_parts) == 5

    new_part = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    scene.add_scene_part(new_part)

    with pytest.raises(ValueError, match="already used by another instance"):
        scene2.add_scene_part(new_part)

    with pytest.raises(AttributeError, match="object has no attribute 'append'"):
        scene.append(new_part)


def test_scenepart_flag_from_xml_set():
    from helios.utils import is_xml_loaded

    part = ScenePart.from_xml("data/scenes/toyblocks/toyblocks_scene.xml", id="0")
    assert is_xml_loaded(part)

    part2 = ScenePart.from_vox(
        "data/sceneparts/syssifoss/F_BR08_08_merged.vox",
        intersection_mode="scaled",
        intersection_argument=0.5,
    )
    assert not is_xml_loaded(part2)
    part3 = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    assert not is_xml_loaded(part3)
    part4 = ScenePart.from_xyz(
        "data/sceneparts/pointclouds/sphere_dens25000.xyz",
        separator=" ",
        voxel_size=1.0,
        max_color_value=255.0,
    )
    assert not is_xml_loaded(part4)
    part5 = ScenePart.from_tiff("data/sceneparts/tiff/dem_hd.tif")
    assert not is_xml_loaded(part5)


def test_scene_flag_from_xml_set():
    from helios.utils import is_xml_loaded

    scene = StaticScene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")
    assert is_xml_loaded(scene)


def test_create_binary_during_from_xml():
    """
    Test that a binary file is created when loading a scene from XML.
    """
    scene = StaticScene.from_xml(
        "data/scenes/toyblocks/toyblocks_scene.xml", save_to_binary=True
    )
    assert os.path.exists("data/scenes/toyblocks/toyblocks_scene.scene")
    os.remove("data/scenes/toyblocks/toyblocks_scene.scene")


def test_flags_assosiated_with_scene():
    # this creates a binary after reading from xml
    survey = Survey.from_xml("data/surveys/toyblocks/tls_toyblocks.xml", True, True)
    assert is_xml_loaded(survey)
    assert is_xml_loaded(survey.scene)
    assert not is_binary_loaded(survey.scene)
    assert not is_binary_loaded(survey)

    # this reads from created binary directly through .from_xml
    survey2 = Survey.from_xml("data/surveys/toyblocks/tls_toyblocks.xml", False)
    assert is_xml_loaded(survey2)
    assert is_xml_loaded(survey2.scene)
    assert is_binary_loaded(survey2.scene)

    scene1 = StaticScene.from_binary("data/scenes/toyblocks/toyblocks_scene.scene")
    assert is_binary_loaded(scene1)
    assert not is_xml_loaded(scene1)

    scene2 = StaticScene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")
    assert is_xml_loaded(scene2)
    assert not is_binary_loaded(scene2)


def test_manual_vs_xml_write_to_file():
    """
    Test that writing a scene to a file works correctly both manually and through XML.
    """
    test_dir = "output/test_writing/"
    scene_part1 = ScenePart.from_obj("data/sceneparts/basic/groundplane/groundplane.obj")
    scene_part1.scale(70)
    scene_part1.translate([20., 0., 0.])

    scene_part2= ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    scene_part2.scale(1)
    scene_part3= ScenePart.from_obj("data/sceneparts/toyblocks/cube.obj")
    scene_part3.rotate(angle="45 deg", axis=[0, 0, 1.0])
    scene_part3.scale(0.5)
    scene_part3.force_on_ground = ForceOnGroundStrategy.MOST_COMPLEX
    scene_part3.translate([-45.,10.,10.])
    scene_part4= ScenePart.from_obj("data/sceneparts/toyblocks/sphere.obj")
    scene_part4.scale(0.5)
    scene_part5 = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    scene_part5.scale(1)
    scene = StaticScene(scene_parts=[scene_part1, scene_part2, scene_part3, scene_part4, scene_part5])
    
    platform_settings1 = StaticPlatformSettings(x=-26, y=-31, z=0, force_on_ground=True)
    platform_settings2 = StaticPlatformSettings(x=-35, y=35, z=0, force_on_ground=True)
    platform_settings3 = StaticPlatformSettings(x=60, y=0, z=0, force_on_ground=True)
    scanner_settings1 = ScannerSettings(is_active=True, pulse_frequency=100000, scan_frequency=120, min_vertical_angle="-40 deg", max_vertical_angle="60 deg",
                                                head_rotation="10 deg/s", rotation_start_angle="245 deg", rotation_stop_angle="360 deg")
    scanner_settings2 = ScannerSettings(is_active=True, pulse_frequency=100000, scan_frequency=120, scan_angle= "50 deg",
                                                head_rotation="10 deg/s", rotation_start_angle="180 deg", rotation_stop_angle="320 deg")
    scanner_settings3 = ScannerSettings(is_active=True, pulse_frequency=100000, scan_frequency=120, min_vertical_angle="-40 deg", max_vertical_angle="60 deg",
                                                head_rotation="10 deg/s", rotation_start_angle="0 deg", rotation_stop_angle="180 deg")
    platform = tripod()
    scanner = riegl_vz_400()
    survey = Survey(scanner=scanner, platform=platform, scene=scene)
    survey.add_leg(scanner_settings=scanner_settings1, platform_settings=platform_settings1)
    survey.add_leg(scanner_settings=scanner_settings2, platform_settings=platform_settings2)
    survey.add_leg(scanner_settings=scanner_settings3, platform_settings=platform_settings3)
    survey.run(format="las", output_dir=test_dir)

    subdirectories = [d for d in os.listdir(test_dir) if os.path.isdir(os.path.join(test_dir, d))]
    if len(subdirectories) == 1:
        las_file_path = os.path.join(test_dir, subdirectories[0], "leg000_points.las")
        
        las_file1 = laspy.read(las_file_path)
        
        # Delete the subdirectory
        subdirectory_path = os.path.join(test_dir, subdirectories[0])
        shutil.rmtree(subdirectory_path)

    platform_settings11 = StaticPlatformSettings(x=-26, y=-31, z=0, force_on_ground=True)
    platform_settings12 = StaticPlatformSettings(x=-35, y=35, z=0, force_on_ground=True)
    platform_settings13 = StaticPlatformSettings(x=60, y=0, z=0, force_on_ground=True)
    scanner_settings11 = ScannerSettings(is_active=True, pulse_frequency=100000, scan_frequency=120, min_vertical_angle="-40 deg", max_vertical_angle="60 deg",
                                                head_rotation="10 deg/s", rotation_start_angle="245 deg", rotation_stop_angle="360 deg")
    scanner_settings12 = ScannerSettings(is_active=True, pulse_frequency=100000, scan_frequency=120, scan_angle= "50 deg",
                                                head_rotation="10 deg/s", rotation_start_angle="180 deg", rotation_stop_angle="320 deg")
    scanner_settings13 = ScannerSettings(is_active=True, pulse_frequency=100000, scan_frequency=120, min_vertical_angle="-40 deg", max_vertical_angle="60 deg",
                                            head_rotation="10 deg/s", rotation_start_angle="0 deg", rotation_stop_angle="180 deg")
    
    scene2 = StaticScene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")
    platform2 = tripod()
    scanner2 = riegl_vz_400()
    survey2 = Survey(scanner=scanner2, platform=platform2, scene=scene2)
    survey2.add_leg(scanner_settings=scanner_settings11, platform_settings=platform_settings11)
    survey2.add_leg(scanner_settings=scanner_settings12, platform_settings=platform_settings12)
    survey2.add_leg(scanner_settings=scanner_settings13, platform_settings=platform_settings13)
    survey2.run(format="las", output_dir=test_dir)
    subdirectories2 = [d for d in os.listdir(test_dir) if os.path.isdir(os.path.join(test_dir, d))]
    if len(subdirectories2) == 1:
        las_file_path2 = os.path.join(test_dir, subdirectories2[0], "leg000_points.las")
        las_file2 = laspy.read(las_file_path2)
        shutil.rmtree(test_dir)
    
    assert las_file1.X.shape[0] == las_file2.X.shape[0]

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
from pathlib import Path

from helios import HeliosException
import _helios


def _write_xyz_file(path: Path, separator: str = " ") -> None:
    rows = [
        [0.0, 0.0, 0.0, 255, 0, 0],
        [1.0, 0.0, 0.0, 0, 255, 0],
        [0.0, 1.0, 0.0, 0, 0, 255],
        [1.0, 1.0, 0.0, 255, 255, 0],
    ]
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(separator.join(str(value) for value in row))
            handle.write("\n")


@pytest.fixture
def xyz_file(tmp_path) -> Path:
    path = tmp_path / "small.xyz"
    _write_xyz_file(path)
    return path


@pytest.fixture
def xyz_file_comma(tmp_path) -> Path:
    path = tmp_path / "small_sepcomma.xyz"
    _write_xyz_file(path, separator=",")
    return path


@pytest.fixture
def xyz_parts_dir(tmp_path) -> Path:
    _write_xyz_file(tmp_path / "part1.xyz")
    _write_xyz_file(tmp_path / "part2.xyz")
    _write_xyz_file(tmp_path / "part3_sepcomma.xyz", separator=",")
    return tmp_path


def _configure_fast_survey_legs(survey: Survey) -> Survey:
    for leg in survey.legs:
        leg.scanner_settings.pulse_frequency = 2000
        leg.scanner_settings.scan_frequency = 20
        leg.scanner_settings.head_rotation = "30 deg/s"
        leg.scanner_settings.rotation_start_angle = "0 deg"
        leg.scanner_settings.rotation_stop_angle = "20 deg"
    return survey


def _find_single_las_file(output_dir: Path) -> Path:
    files = sorted(output_dir.rglob("leg000_points.las"))
    assert len(files) == 1, (
        f"Expected exactly one LAS file named 'leg000_points.las' in '{output_dir}', "
        f"found {len(files)}: {[str(path) for path in files]}"
    )
    return files[0]


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
    scene_part = ScenePart.from_tiff("data/test/dem_hd_sub.tif")
    assert len(scene_part._cpp_object.primitives) > 0


def test_scenepart_from_tiffs():
    scene_parts = ScenePart.from_tiffs("data/test/*.tif")
    assert len(scene_parts) == 2


def test_scenepart_from_xyz(xyz_file, xyz_file_comma):
    scene_part1 = ScenePart.from_xyz(
        str(xyz_file),
        separator=" ",
        voxel_size=1.0,
        max_color_value=255.0,
        default_normal=[0.0, 0.0, 1.0],
    )

    scene_part2 = ScenePart.from_xyz(
        str(xyz_file_comma),
        separator=",",
        voxel_size=1.0,
        max_color_value=255.0,
    )

    scene_part3 = ScenePart.from_xyz(
        str(xyz_file),
        separator=" ",
        voxel_size=1.0,
    )

    scene_part4 = ScenePart.from_xyz(
        str(xyz_file),
        voxel_size=1.0,
        default_normal=[0.0, 0.0, 1.0],
        sparse=False,
    )

    assert len(scene_part1._cpp_object.primitives) > 0
    assert len(scene_part2._cpp_object.primitives) > 0
    assert len(scene_part3._cpp_object.primitives) > 0
    assert len(scene_part4._cpp_object.primitives) > 0


def test_scenepart_from_xyzs(xyz_parts_dir):
    scene_parts1 = ScenePart.from_xyzs(
        str(xyz_parts_dir / "part[12].xyz"), voxel_size=1.0
    )

    scene_parts2 = ScenePart.from_xyzs(
        str(xyz_parts_dir / "part[12].xyz"),
        voxel_size=1.0,
        max_color_value=255.0,
        default_normal=[0.0, 0.0, 1.0],
    )

    assert len(scene_parts1) > 0
    assert len(scene_parts2) > 0

    with pytest.raises(HeliosException, match="separator mismatch"):
        ScenePart.from_xyzs(
            str(xyz_parts_dir / "*.xyz"),
            voxel_size=1.0,
            separator=",",
        )


def test_scenepart_from_vox():
    scene_parts1 = ScenePart.from_vox(
        "data/test/semitransparent_voxels.vox",
        intersection_mode="transmittive",
    )
    scene_parts2 = ScenePart.from_vox(
        "data/test/semitransparent_voxels.vox",
        intersection_mode="fixed",
    )
    scene_parts3 = ScenePart.from_vox(
        "data/test/semitransparent_voxels.vox",
        intersection_mode="scaled",
        intersection_argument=0.5,
    )
    scene_parts4 = ScenePart.from_vox(
        "data/test/semitransparent_voxels.vox",
        intersection_mode="scaled",
    )

    assert len(scene_parts1._cpp_object.primitives) > 0
    assert len(scene_parts2._cpp_object.primitives) > 0
    assert len(scene_parts3._cpp_object.primitives) > 0
    assert len(scene_parts4._cpp_object.primitives) > 0

    with pytest.raises(ValueError):
        ScenePart.from_vox(
            "data/test/semitransparent_voxels.vox",
            intersection_mode="fixed",
            intersection_argument=0.1,
        )


def get_bbox(part):
    scene = StaticScene(scene_parts=[part])
    # Avoid flaky crashes in optimized builds from concurrent KDTree construction
    # for very small test scenes.
    scene._finalize(
        execution_settings=ExecutionSettings(kdt_num_threads=1, kdt_geom_num_threads=1)
    )
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
    survey1 = Survey.from_xml(
        "data/surveys/demo/box_survey_static_puck.xml", True, True
    )
    _configure_fast_survey_legs(survey1)
    points1, _ = survey1.run()

    survey2 = Survey.from_xml("data/surveys/demo/box_survey_static_puck.xml")
    survey2.scene = StaticScene.from_binary("data/scenes/demo/box_scene.scene")
    _configure_fast_survey_legs(survey2)
    points2, _ = survey2.run()

    # We read from created binary directly through .from_xml
    survey3 = Survey.from_xml("data/surveys/demo/box_survey_static_puck.xml", False)
    _configure_fast_survey_legs(survey3)
    points3, _ = survey3.run()

    os.remove("data/scenes/demo/box_scene.scene")
    assert len(points1) == len(points2)
    assert len(points1) == len(points3)


def test_scene_manual_binary(tmp_path):
    scene1 = StaticScene(
        scene_parts=[
            ScenePart.from_obj("data/sceneparts/basic/box/box100.obj"),
            ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj").scale(0.5),
        ]
    )
    binary_path = tmp_path / "manual.scene"
    scene1.to_binary(str(binary_path))
    scene2 = StaticScene.from_binary(str(binary_path))

    scanner_settings = ScannerSettings(
        is_active=True,
        pulse_frequency=2000,
        scan_frequency=20,
        scan_angle="20 deg",
        head_rotation="30 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="20 deg",
    )
    platform_settings = StaticPlatformSettings(x=0, y=0, z=0, force_on_ground=True)

    survey1 = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=scene1)
    survey1.add_leg(
        scanner_settings=scanner_settings, platform_settings=platform_settings
    )
    points1, _ = survey1.run(format=OutputFormat.NPY)

    survey2 = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=scene2)
    survey2.add_leg(
        scanner_settings=scanner_settings, platform_settings=platform_settings
    )
    points2, _ = survey2.run(format=OutputFormat.NPY)

    assert len(points1) == len(points2)


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
    sp2.materials["None"].is_ground = False

    scene = StaticScene(scene_parts=[sp1, sp2])
    scene._finalize()

    assert not np.isclose(
        sp1._cpp_object.all_vertices[0].position[2],
        sp2._cpp_object.all_vertices[0].position[2],
    )


def test_add_scene_part():
    """
    Test that a scene part can be added to an existing scene.
    """
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

    scene = StaticScene(
        scene_parts=[scene_part1, scene_part2, scene_part3, scene_part4]
    )

    scene_part5 = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    scene_part5.scale(1)
    scene.add_scene_part(scene_part5)

    assert len(scene.scene_parts) == 5
    assert len(scene._cpp_object.scene_parts) == 5
    assert scene_part5 in scene.scene_parts


def test_add_scene_part_invalid():
    """
    Test that adding a duplicate of scene_part raises an error. As well as usage of append method.
    """
    scene = StaticScene.from_xml("data/scenes/toyblocks/toyblocks_scene.xml")
    assert len(scene.scene_parts) == 5

    new_part = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    with pytest.raises(
        RuntimeError, match="The scene loaded from XML cannot be modified."
    ):
        scene.add_scene_part(new_part)


def test_run_after_add_scene_part():
    scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )
    added_scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    added_scene_part.scale(0.5)
    scene.add_scene_part(added_scene_part)

    scanner_settings = ScannerSettings(
        is_active=True,
        pulse_frequency=2000,
        scan_frequency=20,
        scan_angle="20 deg",
        head_rotation="30 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="20 deg",
    )
    platform_settings = StaticPlatformSettings(x=0, y=0, z=0, force_on_ground=True)

    survey = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=scene)
    survey.add_leg(
        scanner_settings=scanner_settings, platform_settings=platform_settings
    )
    points, _ = survey.run(format=OutputFormat.NPY)

    scene2 = StaticScene(
        scene_parts=[
            ScenePart.from_obj("data/sceneparts/basic/box/box100.obj"),
            ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj").scale(0.5),
        ]
    )
    survey2 = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=scene2)
    survey2.add_leg(
        scanner_settings=scanner_settings, platform_settings=platform_settings
    )
    points2, _ = survey2.run(format=OutputFormat.NPY)
    assert len(points) == len(points2)


def test_scenepart_flag_from_xml_set(xyz_file):
    from helios.utils import is_xml_loaded

    part = ScenePart.from_xml("data/scenes/toyblocks/toyblocks_scene.xml", id="0")
    assert is_xml_loaded(part)

    part2 = ScenePart.from_vox(
        "data/test/semitransparent_voxels.vox",
        intersection_mode="scaled",
        intersection_argument=0.5,
    )
    assert not is_xml_loaded(part2)
    part3 = ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")
    assert not is_xml_loaded(part3)
    part4 = ScenePart.from_xyz(
        str(xyz_file),
        separator=" ",
        voxel_size=1.0,
    )
    assert not is_xml_loaded(part4)
    part5 = ScenePart.from_tiff("data/test/dem_hd_sub.tif")
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


def test_manual_vs_xml_write_to_file(tmp_path):
    """
    Test that writing a scene to a file works correctly both manually and through XML.
    """
    scanner_settings = ScannerSettings(
        is_active=True,
        pulse_frequency=2000,
        scan_frequency=20,
        scan_angle="20 deg",
        head_rotation="30 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="20 deg",
    )
    platform_settings = StaticPlatformSettings(x=0, y=0, z=0, force_on_ground=True)

    manual_scene = StaticScene(
        scene_parts=[ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")]
    )
    survey_manual = Survey(
        scanner=riegl_vz_400(), platform=tripod(), scene=manual_scene
    )
    survey_manual.add_leg(
        scanner_settings=scanner_settings, platform_settings=platform_settings
    )
    manual_dir = tmp_path / "manual"
    survey_manual.run(format=OutputFormat.LAS, output_dir=manual_dir)
    manual_las = laspy.read(_find_single_las_file(manual_dir))

    xml_scene = StaticScene.from_xml("data/scenes/demo/box_scene.xml")
    survey_xml = Survey(scanner=riegl_vz_400(), platform=tripod(), scene=xml_scene)
    survey_xml.add_leg(
        scanner_settings=scanner_settings, platform_settings=platform_settings
    )
    xml_dir = tmp_path / "xml"
    survey_xml.run(format=OutputFormat.LAS, output_dir=xml_dir)
    xml_las = laspy.read(_find_single_las_file(xml_dir))

    assert manual_las.X.shape[0] > 0
    assert manual_las.X.shape[0] == xml_las.X.shape[0]


def test_read_materials_from_file():
    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")
    assert mat1.name == "Material.008"
    assert mat1._cpp_object.name == "Material.008"
    assert mat1.reflectance == 0.2


def test_read_incorrect_file_for_material():
    with pytest.raises(FileNotFoundError):
        Material.from_file(
            "data/sceneparts/toyblocks/incorrectfile.mtl", "Material.008"
        )


def test_read_incorrect_material_name():
    with pytest.raises(RuntimeError):
        Material.from_file(
            "data/sceneparts/toyblocks/sphere.mtl", "IncorrectMaterialName"
        )


def test_apply_material_to_all_primitives():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    assert scene_part._cpp_object.primitives[4].material.name == "Material.007"
    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")
    scene_part.update_material(mat1)

    assert scene_part._cpp_object.primitives[4].material.name == "Material.008"
    mat2 = Material.from_file("data/sceneparts/toyblocks/cylinder.mtl", "Material.007")
    scene_part.materials["Material.008"] = mat2
    assert scene_part._cpp_object.primitives[4].material.name == "Material.007"


def test_apply_material_to_primitives_in_specific_range():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    assert scene_part._cpp_object.primitives[17].material.reflectance == 0.5
    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")

    scene_part.update_material(mat1, range_start=10, range_stop=20)
    assert scene_part._cpp_object.primitives[17].material.reflectance == 0.2

    mat2 = Material.from_file("data/sceneparts/toyblocks/cube_rot.mtl", "Material.002")

    scene_part.update_material(mat2, range_stop=5)
    assert scene_part._cpp_object.primitives[3].material.name == "Material.002"

    scene_part.update_material(mat2, range_start=25)
    assert scene_part._cpp_object.primitives[27].material.name == "Material.002"


def test_apply_material_to_primitives_incorrect_range():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")

    with pytest.raises(ValueError):
        scene_part.update_material(mat1, range_start=20, range_stop=10)

    with pytest.raises(ValueError):
        scene_part.update_material(mat1, range_start=-5, range_stop=10)


def test_apply_material_to_indices():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    assert scene_part._cpp_object.primitives[3].material.spectra == "shingle_red"
    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")

    scene_part.update_material(mat1, indices=[1, 3, 5])
    assert scene_part._cpp_object.primitives[3].material.spectra == "conifer"


def test_apply_material_to_incorrect_indices():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")

    with pytest.raises(IndexError):
        scene_part.update_material(mat1, indices=[-1, 3000000])


def test_update_material_attribute_for_specific_primitives(xyz_file):
    scene_part_obj = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")

    scene_part_obj.materials["Material.007"].classification = 7
    assert scene_part_obj._cpp_object.primitives[5].material.classification == 7

    scene_part_xyz = ScenePart.from_xyz(
        str(xyz_file),
        voxel_size=1.0,
    )
    scene_part_xyz.materials["default"].reflectance = 0.658
    assert scene_part_xyz._cpp_object.primitives[0].material.reflectance == 0.658

    scene_part_vox = ScenePart.from_vox(
        "data/test/semitransparent_voxels.vox",
        intersection_mode="scaled",
        intersection_argument=0.5,
    )
    scene_part_vox.materials["default"].specular_components = [0.1437, 0.2, 0.3434, 0.4]
    assert (
        round(
            scene_part_vox._cpp_object.primitives[0].material.specular_components[0], 4
        )
        == 0.1437
    )
    assert (
        round(
            scene_part_vox._cpp_object.primitives[0].material.specular_components[2], 4
        )
        == 0.3434
    )

    scene_part_tif = ScenePart.from_tiff("data/test/dem_hd_sub.tif")
    scene_part_tif.materials["default"].specular_exponent = 25.0
    assert scene_part_tif._cpp_object.primitives[0].material.specular_exponent == 25.0

    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")
    scene_part_xyz.materials["default"] = mat1
    assert scene_part_xyz._cpp_object.primitives[0].material.reflectance == 0.2


def test_update_material_attribute_incorrect_name():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")

    with pytest.raises(KeyError):
        scene_part.materials["IncorrectMaterialName"].reflectance = 0.9

    with pytest.raises(TypeError):
        not_mat = ScenePart()
        scene_part.materials["Material.007"] = not_mat

    with pytest.raises(KeyError):
        scene_part_tif = ScenePart.from_tiff("data/sceneparts/basic/plane/plane.tif")
        scene_part_tif.materials["default"].reflectance = 0.3
        # fails because there are no properties to assign to


def test_material_dict_iteration():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    material_names = [name for name in scene_part.materials.keys()]
    expected_name = "Material.007"
    assert len(material_names) == 1
    assert material_names[0] == expected_name
    scene_part.update_material(
        Material.from_file(
            "data/sceneparts/toyblocks/sphere.mtl", material_id="Material.008"
        ),
        range_start=0,
        range_stop=10,
    )
    material_names = [name for name in scene_part.materials.keys()]
    expected_names = ["Material.007", "Material.008"]
    assert set(material_names) == set(expected_names)
    assert scene_part.materials.keys().__len__() == 2


def test_material_dict_length():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    assert len(scene_part.materials) == 1
    scene_part.update_material(
        Material.from_file(
            "data/sceneparts/toyblocks/sphere.mtl", material_id="Material.008"
        ),
        range_start=0,
        range_stop=10,
    )
    assert len(scene_part.materials) == 2


def test_material_dict_contains():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    assert "Material.007" in scene_part.materials
    assert "Material.008" not in scene_part.materials
    scene_part.update_material(
        Material.from_file(
            "data/sceneparts/toyblocks/sphere.mtl", material_id="Material.008"
        ),
        range_start=0,
        range_stop=10,
    )
    assert "Material.008" in scene_part.materials


def test_material_dict_getitem():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")
    mat = scene_part.materials["Material.007"]
    assert isinstance(mat, Material)
    assert mat.name == "Material.007"
    scene_part.update_material(
        Material.from_file(
            "data/sceneparts/toyblocks/sphere.mtl", material_id="Material.008"
        ),
        range_start=0,
        range_stop=10,
    )
    mat2 = scene_part.materials["Material.008"]
    assert isinstance(mat2, Material)
    assert mat2.name == "Material.008"


def test_material_dict_cache_invalidation_on_update_material():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")

    mats1 = scene_part.materials._snapshot()
    assert "Material.007" in mats1
    assert len(mats1) == 1

    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")
    scene_part.update_material(mat1, range_start=0, range_stop=10)

    mats2 = scene_part.materials._snapshot()

    assert mats1 is not mats2
    assert "Material.008" in mats2
    assert len(mats2) == 2


def test_material_dict_cache_reflects_cpp_changes():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")

    mats1 = scene_part.materials._snapshot()

    mat1 = Material.from_file("data/sceneparts/toyblocks/sphere.mtl", "Material.008")
    scene_part.update_material(mat1)

    mats2 = scene_part.materials._snapshot()

    assert mats1 is not mats2
    assert "Material.008" in mats2


def test_material_dict_cache_mutation_propagates():
    scene_part = ScenePart.from_obj("data/sceneparts/toyblocks/cylinder.obj")

    mat = scene_part.materials["Material.007"]
    mat.reflectance = 0.123

    assert scene_part._cpp_object.primitives[5].material.reflectance == 0.123

    mats2 = scene_part.materials._snapshot()
    assert mats2["Material.007"].reflectance == 0.123

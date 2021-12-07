import pytest
import os
import shutil
import subprocess
import numpy as np
from pathlib import Path
import sys
from extra import flight_planner
import re

# HELIOS_EXE = str(Path('build') / 'helios')
# if sys.platform == "win32":
#    HELIOS_EXE += ".exe"
WORKING_DIR = str(Path(__file__).parent.parent.absolute())


def test_rotate_around_origin():
    """Test function rotate_around_point for rotation around O(0, 0)"""
    # given
    array = np.array([[0, 0],
                      [1, 2],
                      [2, 2],
                      [1, 0],
                      [2, 0],
                      [3, 2]])
    angle = 63.435

    # expected
    array_expected = np.array([[0.00000000e+00, 0.00000000e+00],
                               [2.23606798e+00, -1.99727495e-06],
                               [2.68328077e+00, -8.94429588e-01],
                               [4.47212797e-01, -8.94427590e-01],
                               [8.94425593e-01, -1.78885518e+00],
                               [3.13049357e+00, -1.78885718e+00]])

    array_rotated = flight_planner.rotate_around_point(array, angle)

    assert np.allclose(array_rotated, array_expected)


def test_flight_length():
    # given
    legs = np.array([[0, 0],
                     [0, 10],
                     [10, 10],
                     [10, 0],
                     [0, 0],
                     [0, 10],
                     [10, 10],
                     [10, 0]])

    # expected
    expected_distance = 70

    distance = flight_planner.compute_flight_length(legs)

    assert distance == expected_distance


def test_flight_lines_criss_cross():
    """Function to test creation of flight lines for criss-cross flight pattern"""
    # given
    bounding_box = [-100, -100, 100, 100]
    strip_spacing = 25

    # expected
    waypoints_expected = np.array([[100., -100.],
                                   [-100., -100.],
                                   [-100., -75.],
                                   [100., -75.],
                                   [100., -50.],
                                   [-100., -50.],
                                   [-100., -25.],
                                   [100., -25.],
                                   [100., 0.],
                                   [-100., 0.],
                                   [-100., 25.],
                                   [100., 25.],
                                   [100., 50.],
                                   [-100., 50.],
                                   [-100., 75.],
                                   [100., 75.],
                                   [100., 100.],
                                   [-100., 100.],
                                   [-100., 100.],
                                   [-100., -100.],
                                   [-75., -100.],
                                   [-75., 100.],
                                   [-50., 100.],
                                   [-50., -100.],
                                   [-25., -100.],
                                   [-25., 100.],
                                   [0., 100.],
                                   [0., -100.],
                                   [25., -100.],
                                   [25., 100.],
                                   [50., 100.],
                                   [50., -100.],
                                   [75., -100.],
                                   [75., 100.],
                                   [100., 100.],
                                   [100., -100.]])
    centre_expected = np.array([0., 0.])
    distance_expected = 4000

    waypoints, centre, distance = flight_planner.compute_flight_lines(bounding_box,
                                                                      spacing=strip_spacing,
                                                                      rotate_deg=0.0,
                                                                      flight_pattern="criss-cross")
    assert np.allclose(waypoints, waypoints_expected)
    assert np.allclose(centre, centre_expected)
    assert distance == distance_expected


def test_flight_lines_parallel():
    """Function to test creation of flight lines for parallel flight pattern"""
    # given
    bounding_box = [-100, -100, 100, 100]
    strip_spacing = 25

    # expected
    waypoints_expected = np.array([[100., -100.],
                                   [-100., -100.],
                                   [-100., -75.],
                                   [100., -75.],
                                   [100., -50.],
                                   [-100., -50.],
                                   [-100., -25.],
                                   [100., -25.],
                                   [100., 0.],
                                   [-100., 0.],
                                   [-100., 25.],
                                   [100., 25.],
                                   [100., 50.],
                                   [-100., 50.],
                                   [-100., 75.],
                                   [100., 75.],
                                   [100., 100.],
                                   [-100., 100.]])
    centre_expected = np.array([0., 0.])
    distance_expected = 2000

    waypoints, centre, distance = flight_planner.compute_flight_lines(bounding_box,
                                                                      spacing=strip_spacing,
                                                                      rotate_deg=0.0,
                                                                      flight_pattern="parallel")

    assert np.allclose(waypoints, waypoints_expected)
    assert np.allclose(centre, centre_expected)
    assert distance == distance_expected


def test_export_for_xml_not_always_active():
    """Function to test the export of legs for XML file"""
    # given
    waypoints = np.array([[0, 0],
                          [0, 10],
                          [5, 10],
                          [5, 0]])
    altitude = 200
    template_id = "template"
    speed = 50
    expected_legs_xml = f'''
        <leg>
            <platformSettings x="{waypoints[0, 0]}" y="{waypoints[0, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <leg>
            <platformSettings x="{waypoints[1, 0]}" y="{waypoints[1, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" active="false" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <leg>
            <platformSettings x="{waypoints[2, 0]}" y="{waypoints[2, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <leg>
            <platformSettings x="{waypoints[3, 0]}" y="{waypoints[3, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" active="false" trajectoryTimeInterval_s="0.05" />
        </leg>
        '''

    legs_xml = flight_planner.export_for_xml(waypoints, altitude, template_id, speed)

    assert legs_xml == expected_legs_xml


def test_add_translation_filter():
    """Function to test function add_transformation_filters for a translation"""
    # given
    tr = [10, 10, 5]

    # expected:
    trafo_filter_expected = f"""
            <filter type="translate">  
                <param type="integer" key="onGround" value="0" />
                <param type="vec3" key="offset" value="{tr[0]};{tr[1]};{tr[2]}" /> 
            </filter>\n"""

    trafo_filter = flight_planner.add_transformation_filters(translation=tr)

    assert re.sub(r"\s+", " ", trafo_filter) == re.sub(r"\s+", " ", trafo_filter_expected)


def test_add_rotation_filter():
    """Function to test function add_transformation_filters for a rotation"""
    # given
    rot = [90, 0, 180]

    # expected:
    rot_filter_expected = f"""
            <filter type="rotate">
                <param key="rotation" type="rotation">  
                    <rot angle_deg="{rot[0]}" axis="x"/>  
                    <rot angle_deg="{rot[1]}" axis="y"/>  
                    <rot angle_deg="{rot[2]}" axis="z"/>  
                </param>
            </filter>\n"""

    rot_filter = flight_planner.add_transformation_filters(rotation=rot)

    assert re.sub(r"\s+", " ", rot_filter) == re.sub(r"\s+", " ", rot_filter_expected)


def test_add_scale_filter():
    """Function to test function add_transformation_filters for a scaling"""
    # given
    sc = 0.5

    # expected:
    scale_filter_expected = f"""
            <filter type="scale">
                <param type="double" key="scale" value="{sc}" />
            </filter>\n"""

    scale_filter = flight_planner.add_transformation_filters(scale=sc)

    assert re.sub(r"\s+", " ", scale_filter) == re.sub(r"\s+", " ", scale_filter_expected)


def test_add_translation_filter_on_ground():
    """Function to test function add_transformation_filters for a translation onto ground"""
    # given
    tr = [10, 10, 5]
    on_gnd = -1

    # expected:
    trafo_filter_expected = f"""
                    <filter type="translate">  
                        <param type="integer" key="onGround" value="{on_gnd}" />
                        <param type="vec3" key="offset" value="{tr[0]};{tr[1]};{tr[2]}" /> 
                    </filter>\n"""

    trafo_filter = flight_planner.add_transformation_filters(translation=tr, on_ground=on_gnd)

    assert re.sub(r"\s+", " ", trafo_filter) == re.sub(r"\s+", " ", trafo_filter_expected)


def test_create_scenepart_obj_efilepath():
    # given
    filepath = "data/sceneparts/basic/groundplane/groundplane.obj"

    # expected
    obj_filter_expected = f'''
            <part>
                <filter type="objloader">
                    <param type="string" key="efilepath" value="{filepath} up="z" />
                </filter>

            </part>'''

    obj_filter = flight_planner.create_scenepart_obj(filepath, efilepath=True)

    # raise AssertionError
    assert re.sub(r"\s+", " ", obj_filter) == re.sub(r"\s+", " ", obj_filter_expected)


def test_create_scenepart_obj_invalid_up():
    # given
    filepath = "data/sceneparts/basic/groundplane/groundplane.obj"
    up_axis = "phi"

    # expected
    # raise AssertionError
    with pytest.raises(AssertionError) as e:
        flight_planner.create_scenepart_obj(filepath, up_axis=up_axis)
    assert e.type is AssertionError


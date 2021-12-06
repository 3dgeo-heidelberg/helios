import pytest
import os
import shutil
import subprocess
import numpy as np
from pathlib import Path
import sys
from extra import flight_planner

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

    assert expected_distance == distance


def test_flight_lines_criss_cross():
    pass


def test_flight_lines_parallel():
    pass


def test_export_for_xml_not_always_active():
    # given
    waypoints = np.array([[0, 0],
                          [0, 10],
                          [5, 10],
                          [5, 0]])
    altitude = 200
    id = "template"
    speed = 50
    expected_legs_xml = f'''
        <leg>
            <platformSettings x="{waypoints[0, 0]}" y="{waypoints[0, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{id}" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <leg>
            <platformSettings x="{waypoints[1, 0]}" y="{waypoints[1, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{id}" active="false" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <leg>
            <platformSettings x="{waypoints[2, 0]}" y="{waypoints[2, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{id}" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <leg>
            <platformSettings x="{waypoints[3, 0]}" y="{waypoints[3, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{id}" active="false" trajectoryTimeInterval_s="0.05" />
        </leg>
        '''

    legs_xml = flight_planner.export_for_xml(waypoints, altitude, id, speed)

    assert legs_xml == expected_legs_xml


def test_add_translation_filter():
    pass


def test_create_scenepart_obj():
    pass

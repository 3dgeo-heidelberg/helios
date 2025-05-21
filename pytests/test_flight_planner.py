import os
import numpy as np
from pathlib import Path
from pyhelios.util import flight_planner

WORKING_DIR = os.getcwd()


def test_rotate_around_origin():
    """Test function rotate_around_point for rotation around O(0, 0)"""
    # given
    array = np.array([[0, 0], [1, 2], [2, 2], [1, 0], [2, 0], [3, 2]])
    angle = 63.435

    # expected
    array_expected = np.array(
        [
            [0.00000000e00, 0.00000000e00],
            [2.23606798e00, -1.99727495e-06],
            [2.68328077e00, -8.94429588e-01],
            [4.47212797e-01, -8.94427590e-01],
            [8.94425593e-01, -1.78885518e00],
            [3.13049357e00, -1.78885718e00],
        ]
    )

    array_rotated = flight_planner.rotate_around_point(array, angle)

    assert np.allclose(array_rotated, array_expected)


def test_flight_length():
    """Test computation of flight length vor a given array of waypoints"""
    # given
    legs = np.array(
        [[0, 0], [0, 10], [10, 10], [10, 0], [0, 0], [0, 10], [10, 10], [10, 0]]
    )

    # expected
    expected_distance = 70

    distance = flight_planner.compute_flight_length(legs)

    assert distance == expected_distance


def test_flight_lines_criss_cross():
    """Ttest creation of flight lines for criss-cross flight pattern"""
    # given
    bounding_box = [-100, -100, 100, 100]
    strip_spacing = 25

    # expected
    waypoints_expected = np.array(
        [
            [100.0, -100.0],
            [-100.0, -100.0],
            [-100.0, -75.0],
            [100.0, -75.0],
            [100.0, -50.0],
            [-100.0, -50.0],
            [-100.0, -25.0],
            [100.0, -25.0],
            [100.0, 0.0],
            [-100.0, 0.0],
            [-100.0, 25.0],
            [100.0, 25.0],
            [100.0, 50.0],
            [-100.0, 50.0],
            [-100.0, 75.0],
            [100.0, 75.0],
            [100.0, 100.0],
            [-100.0, 100.0],
            [-100.0, 100.0],
            [-100.0, -100.0],
            [-75.0, -100.0],
            [-75.0, 100.0],
            [-50.0, 100.0],
            [-50.0, -100.0],
            [-25.0, -100.0],
            [-25.0, 100.0],
            [0.0, 100.0],
            [0.0, -100.0],
            [25.0, -100.0],
            [25.0, 100.0],
            [50.0, 100.0],
            [50.0, -100.0],
            [75.0, -100.0],
            [75.0, 100.0],
            [100.0, 100.0],
            [100.0, -100.0],
        ]
    )
    centre_expected = np.array([0.0, 0.0])
    distance_expected = 4000

    waypoints, centre, distance = flight_planner.compute_flight_lines(
        bounding_box,
        spacing=strip_spacing,
        rotate_deg=0.0,
        flight_pattern="criss-cross",
    )
    assert np.allclose(waypoints, waypoints_expected)
    assert np.allclose(centre, centre_expected)
    assert distance == distance_expected


def test_flight_lines_parallel():
    """Test creation of flight lines for parallel flight pattern"""
    # given
    bounding_box = [-100, -100, 100, 100]
    strip_spacing = 25

    # expected
    waypoints_expected = np.array(
        [
            [100.0, -100.0],
            [-100.0, -100.0],
            [-100.0, -75.0],
            [100.0, -75.0],
            [100.0, -50.0],
            [-100.0, -50.0],
            [-100.0, -25.0],
            [100.0, -25.0],
            [100.0, 0.0],
            [-100.0, 0.0],
            [-100.0, 25.0],
            [100.0, 25.0],
            [100.0, 50.0],
            [-100.0, 50.0],
            [-100.0, 75.0],
            [100.0, 75.0],
            [100.0, 100.0],
            [-100.0, 100.0],
        ]
    )
    centre_expected = np.array([0.0, 0.0])
    distance_expected = 2000

    waypoints, centre, distance = flight_planner.compute_flight_lines(
        bounding_box, spacing=strip_spacing, rotate_deg=0.0, flight_pattern="parallel"
    )

    assert np.allclose(waypoints, waypoints_expected)
    assert np.allclose(centre, centre_expected)
    assert distance == distance_expected


def test_write_legs_not_always_active():
    """Test the export of legs for XML file"""
    # given
    waypoints = np.array([[0, 0], [0, 10], [5, 10], [5, 0]])
    altitude = 200
    template_id = "template"
    speed = 50

    # expected
    expected_legs_xml = f"""
        <!-- leg000 -->
        <leg>
            <platformSettings x="{waypoints[0, 0]}" y="{waypoints[0, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <!-- leg001 -->
        <leg>
            <platformSettings x="{waypoints[1, 0]}" y="{waypoints[1, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" active="false" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <!-- leg002 -->
        <leg>
            <platformSettings x="{waypoints[2, 0]}" y="{waypoints[2, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" trajectoryTimeInterval_s="0.05" />
        </leg>
        
        <!-- leg003 -->
        <leg>
            <platformSettings x="{waypoints[3, 0]}" y="{waypoints[3, 1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" active="false" trajectoryTimeInterval_s="0.05" />
        </leg>
        """

    legs_xml = flight_planner.write_legs(waypoints, altitude, template_id, speed)

    assert legs_xml == expected_legs_xml

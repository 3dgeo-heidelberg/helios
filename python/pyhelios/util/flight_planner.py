#!/usr/bin/env python
# -- coding: utf-8 --
#
# Hannah Weiser, Heidelberg University
# December 2021
# h.weiser@uni-heidelberg.de

"""
This script contains functions to facilitate planning and configuration of HELIOS++ surveys.
"""

import numpy as np
import math
import warnings


def rotate_around_point(xy, degrees, origin=(0, 0)):
    """Rotate a point around a given point.
    by Lyle Scott  // lyle@ls3.io (https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302)

    :param xy: array of point(s) to rotate
    :param degrees: rotation (clockwise) in degrees
    :param origin: rotation origin; default: O(0, 0)
    :type xy: array or ndarray
    :type degrees: float
    :type origin: tuple or list
    :return: rotated array
    :rtype: array or ndarray
    """
    # convert degree to radians
    radians = math.radians(degrees)
    x = xy[:, 0]
    y = xy[:, 1]
    offset_x, offset_y = origin
    adjusted_x = x - offset_x
    adjusted_y = y - offset_y
    cos_rad = math.cos(radians)
    sin_rad = math.sin(radians)
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return np.array((qx, qy)).T


def compute_flight_length(waypoints):
    """This function computed the length of a flight plan.

    :param waypoints: The waypoints of the flight path e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]] (list)
    :type waypoints: array or ndarray
    :return: Summed distance of the flight lines
    :rtype: float
    """
    distance = 0
    curr_point = waypoints[-1]
    for point in waypoints[::-1]:
        distance += np.sqrt(
            (curr_point[0] - point[0]) ** 2 + (curr_point[1] - point[1]) ** 2
        )
        curr_point = point

    return distance


def compute_flight_lines(
    bounding_box, spacing, rotate_deg=0.0, flight_pattern="parallel"
):
    """This function creates a flight plan (resembling e.g. DJI flight planner) within a given bounding box
    based on the strip (flight line) spacing, the pattern and a rotation.

    :param bounding_box: Bounding box, within which to create the flight lines
    :param spacing: Spacing between flight lines
    :param rotate_deg: Angle (deg) about which to rotate the generated flight pattern about it's centre point
                    default: 0
    :param flight_pattern: Flight pattern, either of "parallel", "cross-cross"
                    default: "parallel"
    :type bounding_box: array or ndarray
    :type spacing: float
    :type rotate_deg: float
    :type flight_pattern: str
    :return: Ordered array of waypoints for the generated flight plan,
            Centre point of the bounding box (and flight plan),
            Total distance of the flight plan
    """
    centre = np.array(
        [
            (bounding_box[0] + bounding_box[2]) / 2,
            (bounding_box[1] + bounding_box[3]) / 2,
        ]
    )
    bbox_dims = np.array(
        [bounding_box[2] - bounding_box[0], bounding_box[3] - bounding_box[1]]
    )
    n_flight_lines_x = int(np.floor(bbox_dims[1] / spacing))
    n_flight_lines_y = int(np.floor(bbox_dims[0] / spacing))
    pattern_options = ["parallel", "criss-cross"]
    if flight_pattern not in pattern_options:
        warnings.warn(
            """
        Specified flight pattern is not available.
        Possible choices: 'parallel', 'criss-cross'
        Flight pattern will be set to 'parallel'
        """
        )
        flight_pattern = "parallel"

    if (
        flight_pattern == "parallel" and bbox_dims[0] >= bbox_dims[1]
    ) or flight_pattern == "criss-cross":
        y_start = centre[1] - spacing * (n_flight_lines_x / 2)
        curr_strip = np.array([[bounding_box[0], y_start], [bounding_box[2], y_start]])
        strips = curr_strip[::-1]
        for i in range(int(n_flight_lines_x)):
            next_strip = curr_strip + np.array([[0, spacing], [0, spacing]])
            strips = np.vstack((strips, next_strip))
            curr_strip = next_strip[::-1]

    if flight_pattern == "criss-cross" or (
        flight_pattern == "parallel" and bbox_dims[1] > bbox_dims[0]
    ):
        x_start = centre[0] - spacing * (n_flight_lines_y / 2)
        curr_strip = np.array([[x_start, bounding_box[1]], [x_start, bounding_box[3]]])
        if flight_pattern != "criss-cross":
            strips = curr_strip[::-1]
        else:
            strips = np.vstack((strips, curr_strip[::-1]))
        for i in range(int(n_flight_lines_y)):
            next_strip = curr_strip + np.array([[spacing, 0], [spacing, 0]])
            strips = np.vstack((strips, next_strip))
            curr_strip = next_strip[::-1]

    if rotate_deg != 0:
        strips = rotate_around_point(strips, rotate_deg, origin=centre)

    distance = compute_flight_length(strips)

    return strips, centre, distance


def flight_lines_from_shp(filename: str):
    """This function reads a shapefile with line features and returns the line coordinates for each feature

    :param filename: path to shapefile
    :type filename: str

    :return: List of arrays of points of each line geometry
    :rtype: ndarray
    """
    import fiona

    with fiona.open(filename, "r") as shapefile:
        coordinates = []
        for rec in shapefile:
            # check if type LineString
            if not rec["geometry"]["type"] == "LineString":
                raise TypeError(
                    f"Expecting geometries of type 'LineString' in {filename}."
                )
            coords = rec["geometry"]["coordinates"]
            coords = [list(ele[:2]) for ele in coords]
            coordinates.append(coords)
    coordinates = np.array(coordinates)
    coordinates_shape = coordinates.shape
    element_count = int(coordinates_shape[0] * coordinates_shape[1])

    return np.resize(coordinates, (element_count, 2))


def plot_flight_plan(waypoints):
    """This function plots the flight plan, defined by an array of waypoints.

    :param waypoints: array of waypoints e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]]
    :type: array or ndarray

    :return: the 2D-plot of the flight plan
    """
    import matplotlib.pyplot as plt

    plt.plot(waypoints[:, 0], waypoints[:, 1])
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")

    return plt


def write_legs(
    waypoints,
    altitude,
    template_id,
    speed,
    trajectory_time_interval=0.05,
    always_active=False,
):
    """This function exports a flight plan to a string to use in HELIOS++ survey XML files.

    :param waypoints: array of waypoints e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]]
    :param altitude: z-coordinate of all waypoints
    :param template_id: ID of default scanner settings (defined in survey-XML) which legs should share
    :param speed: speed of the platform in m/s
    :param trajectory_time_interval: time interval [s] in which trajectory points are written; default: 0.05
    :param always_active: flag to specify if the scanner should be always active of alternating between active and
                            inactive (boolean: True or False); default: False
    :type waypoints: tuple
    :type altitude: float
    :type template_id: str
    :type speed: float
    :type trajectory_time_interval: float
    :type always_active: bool

    :return: string with all XML leg-tags for a HELIOS++ survey file
    :rtype: str
    """
    xml_string = ""
    active = "true" if always_active else "false"
    for i, leg in enumerate(waypoints):
        if i % 2 == 0 or i == 0:
            xml_string += f"""
        <!-- leg{i:03} -->
        <leg>
            <platformSettings x="{leg[0]}" y="{leg[1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" trajectoryTimeInterval_s="{trajectory_time_interval}" />
        </leg>
        """
        elif i % 2 != 0:
            xml_string += f"""
        <!-- leg{i:03} -->
        <leg>
            <platformSettings x="{leg[0]}" y="{leg[1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{template_id}" active="{active}" trajectoryTimeInterval_s="{trajectory_time_interval}" />
        </leg>
        """
    return xml_string


if __name__ == "__main__":
    # Usage Demo
    bbox = [478250, 5473800, 478400, 5473950]
    wp, c, dist = compute_flight_lines(
        bbox, spacing=30, rotate_deg=45, flight_pattern="criss-cross"
    )
    plot = plot_flight_plan(wp)
    plot.show()
    platform_speed = 5
    print(f"Flight duration: {dist / platform_speed / 60} min")
    alt = 490
    print(
        str(
            write_legs(
                wp, altitude=alt, template_id="uls_template", speed=platform_speed
            )
        )
    )

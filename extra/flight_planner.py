#!/usr/bin/env python
# -- coding: utf-8 --
#
# Hannah Weiser, Heidelberg University
# January 2020
# h.weiser@stud.uni-heidelberg.de

"""
This script contains functions to facilitate planning and configuration of HELIOS++ surveys.
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import geopandas as gpd
import glob


def rotate_around_point(xy, degrees, origin=(0, 0)):
    """
    Rotate a point around a given point.

    by Lyle Scott  // lyle@ls3.io (https://gist.github.com/LyleScott/e36e08bfb23b1f87af68c9051f985302)

    :param xy: array of point(s) to rotate
    :param degrees: rotation (clockwise) in degrees
    :param origin: rotation origin; default: O(0, 0)
    :return:
    """
    # convert degree to radians
    radians = math.radians(degrees)
    x = xy[:, 0]
    y = xy[:, 1]
    offset_x, offset_y = origin
    adjusted_x = (x - offset_x)
    adjusted_y = (y - offset_y)
    cos_rad = math.cos(radians)
    sin_rad = math.sin(radians)
    qx = offset_x + cos_rad * adjusted_x + sin_rad * adjusted_y
    qy = offset_y + -sin_rad * adjusted_x + cos_rad * adjusted_y

    return np.array((qx, qy)).T


def compute_flight_length(waypoints):
    """
    This function computed the length of a flight plan.

    :param waypoints: The waypoints of the flight path e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]] (list)

    :return: Summed distance of the flight lines

    """
    distance = 0
    curr_point = waypoints[-1]
    for point in waypoints[::-1]:
        distance += np.sqrt((curr_point[0] - point[0])**2 + (curr_point[1] - point[1])**2)

    return distance


def compute_flight_lines(bounding_box, spacing, rotate_deg=0.0, flight_pattern="parallel"):
    """
    This function creates a flight plan (resembling e.g. DJI flight planner) within a given bounding box
    based on the strip (flight line) spacing, the pattern and a rotation.

    :param bounding_box: Bounding box, within which to create the flight lines
    :param spacing: Spacing between flight lines
    :param rotate_deg: Angle (deg) about which to rotate the generated flight pattern about it's centre point
                    default: 0
    :param flight_pattern: Flight pattern, either of "parallel", "cross-cross", "zic-zac", "spiral"
                    default: "parallel"

    :return: Ordered array of waypoints for the generated flight plan
    :return: Centre point of the bounding box (and flight plan)
    :return: Total distance of the flight plan
    """
    centre = np.array([(bounding_box[0] + bounding_box[2]) / 2, (bounding_box[1] + bounding_box[3]) / 2])
    bbox_dims = np.array([[bounding_box[2] - bounding_box[0]], [bounding_box[3] - bounding_box[1]]])
    n_flight_lines_x = int(np.floor(bbox_dims[1] / spacing))
    n_flight_lines_y = int(np.floor(bbox_dims[0] / spacing))
    pattern_options = ["parallel", "criss-cross", "zic-zac", "spiral"]
    if flight_pattern not in pattern_options:
        print("WARNING: Specified flight pattern is not available.\n"
              "Possible choices: 'parallel', 'criss-cross'\n"
              "Flight pattern will be set to 'parallel'")
        flight_pattern = "parallel"

    if (flight_pattern == "parallel" and bbox_dims[0] >= bbox_dims[1]) or flight_pattern == "criss-cross":
        y_start = centre[1] - spacing*(n_flight_lines_x/2)
        curr_strip = np.array([[bounding_box[0], y_start], [bounding_box[2], y_start]])
        strips = curr_strip[::-1]
        for i in range(int(n_flight_lines_x)):
            next_strip = curr_strip + np.array([[0, spacing], [0, spacing]])
            strips = np.vstack((strips, next_strip))
            curr_strip = next_strip[::-1]

    if flight_pattern == "criss-cross" or (flight_pattern == "parallel" and bbox_dims[1] > bbox_dims[0]):
        x_start = centre[0] - spacing*(n_flight_lines_y/2)
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


def flight_lines_from_shp(filename):
    """
    This function reads a shapefile with line features and returns the line coordinates for each feature

    :param filename: path to shapefile

    :return: List of arrays of points of each line geometry

    """
    shapefile = gpd.read_file(filename)
    lines = []
    for i in range(shapefile.shape[0]):
        coords = list(shapefile.geometry.iloc[i].coords)
        coords = [list(ele) for ele in coords]
        lines.append(coords)
    lines = np.array(lines)
    lines_shape = lines.shape
    element_count = int(lines_shape[0]*lines_shape[1])

    return np.resize(lines, (element_count, 2))


def plot_flight_plan(waypoints):
    """
    This function plots the flight plan, defined by an array of waypoints.

    :param waypoints: array of waypoints e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]]

    :return: the 2D-plot of the flight plan
    """

    plt.plot(waypoints[:, 0], waypoints[:, 1])
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')

    return plt


def export_for_xml(waypoints, altitude, template_id, speed, trajectory_time_interval=0.05, always_active=False):
    """
    This function exports a flight plan to a string to use in HELIOS++ survey XML files.

    :param waypoints: array of waypoints e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]]
    :param altitude: z-coordinate of all waypoints (float)
    :param template_id: ID of default scanner settings (defined in survey-XML) which legs should share (string)
    :param speed: velocity of the platform in m/s (float)
    :param trajectory_time_interval: time interval [s] in which trajectory points are written (float); default: 0.05
    :param always_active: flag to specify if the scanner should be always active of alternating between active and
                            inactive (boolean: True or False); default: False

    :return: string with all XML leg-tags for a HELIOS++ survey file

    """
    xml_string = ""
    for i, leg in enumerate(waypoints):
        if i % 2 == 0 or i == 0:
            xml_string += '''
        <leg>
            <platformSettings x="{x}" y="{y}" z="{z}" movePerSec_m="{v}" />
            <scannerSettings template="{id}" trajectoryTimeInterval_s="{interval}" />
        </leg>
            '''.format(x=leg[0], y=leg[1], z=altitude, v=speed, id=template_id, interval=trajectory_time_interval)
        elif i % 2 != 0 and always_active is False:
            xml_string += '''
        <leg>
            <platformSettings x="{x}" y="{y}" z="{z}" movePerSec_m="{v}" />
            <scannerSettings template="{id}" active="false" trajectoryTimeInterval_s="{interval}" />
        </leg>
            '''.format(x=leg[0], y=leg[1], z=altitude, v=speed, id=template_id, interval=trajectory_time_interval)
    return xml_string


if __name__ == "__main__":

    # Usage Demo
    bbox = [478827.922, 5471949.077, 479011.641, 5472145.427]
    #area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
    #print("area: %.2f m^2" % area)
    wp, c, dist = compute_flight_lines(bbox, spacing=30, rotate_deg=45, flight_pattern="criss-cross")
    plot = plot_flight_plan(wp)
    plot.show()
    speed = 5
    print("Flight duration: %.2f min" % (dist / speed / 60))
    alt = 490
    print(str(export_for_xml(wp, altitude=alt, template_id="uls_template", speed=speed)))

#!/usr/bin/env python
# -- coding: utf-8 --
#
# Hannah Weiser, Heidelberg University
# January 2021
# h.weiser@stud.uni-heidelberg.de

"""
This script contains functions to facilitate planning and configuration of HELIOS++ surveys.
"""

import numpy as np
import math


def rotate_around_point(xy, degrees, origin=(0, 0)):
    """Rotate a point around a given point.
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
    """This function computed the length of a flight plan.

    :param waypoints: The waypoints of the flight path e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]] (list)

    :return: Summed distance of the flight lines

    """
    distance = 0
    curr_point = waypoints[-1]
    for point in waypoints[::-1]:
        distance += np.sqrt((curr_point[0] - point[0])**2 + (curr_point[1] - point[1])**2)

    return distance


def compute_flight_lines(bounding_box, spacing, rotate_deg=0.0, flight_pattern="parallel"):
    """This function creates a flight plan (resembling e.g. DJI flight planner) within a given bounding box
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
    """This function reads a shapefile with line features and returns the line coordinates for each feature

    :param filename: path to shapefile

    :return: List of arrays of points of each line geometry

    """
    import fiona

    with fiona.open(filename, "r") as shapefile:
        coordinates = []
        for rec in shapefile:
            # check if type LineString
            if not rec["geometry"]["type"] == "LineString":
                raise TypeError(f"Expecting geometries of type 'LineString' in {filename}.")
            coords = rec["geometry"]["coordinates"]
            coords = [list(ele[:2]) for ele in coords]
            coordinates.append(coords)
    coordinates = np.array(coordinates)
    coordinates_shape = coordinates.shape
    element_count = int(coordinates_shape[0]*coordinates_shape[1])

    return np.resize(coordinates, (element_count, 2))


def plot_flight_plan(waypoints):
    """This function plots the flight plan, defined by an array of waypoints.

    :param waypoints: array of waypoints e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]]

    :return: the 2D-plot of the flight plan
    """
    import matplotlib.pyplot as plt

    plt.plot(waypoints[:, 0], waypoints[:, 1])
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')

    return plt


def export_for_xml(waypoints, altitude, id, speed,
                   trajectory_time_interval=0.05, always_active=False):
    """This function exports a flight plan to a string to use in HELIOS++ survey XML files.

    :param waypoints: array of waypoints e.g. [[50, -100], [50, 100], [-50, 100], [-50, -100]]
    :param altitude: z-coordinate of all waypoints (float)
    :param id: ID of default scanner settings (defined in survey-XML) which legs should share (string)
    :param speed: speed of the platform in m/s (float)
    :param trajectory_time_interval: time interval [s] in which trajectory points are written (float); default: 0.05
    :param always_active: flag to specify if the scanner should be always active of alternating between active and
                            inactive (boolean: True or False); default: False

    :return: string with all XML leg-tags for a HELIOS++ survey file

    """
    xml_string = ""
    active = "true" if always_active else "false"
    for i, leg in enumerate(waypoints):
        if i % 2 == 0 or i == 0:
            xml_string += f'''
        <leg>
            <platformSettings x="{leg[0]}" y="{leg[1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{id}" trajectoryTimeInterval_s="{trajectory_time_interval}" />
        </leg>
            '''
        elif i % 2 != 0:
            xml_string += f'''
        <leg>
            <platformSettings x="{leg[0]}" y="{leg[1]}" z="{altitude}" movePerSec_m="{speed}" />
            <scannerSettings template="{id}" active="{active}" trajectoryTimeInterval_s="{trajectory_time_interval}" />
        </leg>
            '''
    return xml_string


def add_transformation_filters(translation=None, rotation=None, scale=1, on_ground=0):
    """This function creates a string of transformation filters for a given translation, rotation and scale

    :param translation: list of translations in x-, y- and z-direction; [t_x, t_y, t_z] (list)
    :param rotation: list of rotations around the x-, y- and z-axes; [rot_x, rot_y, rot_z] (list)
    :param scale: value by which to scale the scenepart (float)
    :param on_ground: flag to specifiy whether the scenepart should be translated to the ground (integer)
                    0  = no ground translation
                    -1 = find optimal ground translation
                    1  = find quick ground translation
                    >1 = specify a depth for the search process
    :return: transformation filter(s) (string)
    """
    if rotation is None:
        rotation = [0, 0, 0]
    if translation is None:
        translation = [0, 0, 0]
    trafo_filter = ""
    if translation != [0, 0, 0] or on_ground != 0:
        trafo_filter += f"""
            <filter type="translate">  
                <param type="integer" key="onGround" value="{on_ground}" />
                <param type="vec3" key="offset" value="{translation[0]};{translation[1]};{translation[2]}" />  
            </filter>\n"""
    if rotation != [0, 0, 0]:
        trafo_filter += f"""
            <filter type="rotate">
                <param key="rotation" type="rotation">  
                    <rot angle_deg="{rotation[0]}" axis="x"/>  
                    <rot angle_deg="{rotation[1]}" axis="y"/>  
                    <rot angle_deg="{rotation[2]}" axis="z"/>  
                </param>
            </filter>\n"""
    if scale != 1:
        trafo_filter += f"""
            <filter type="scale">
                <param type="double" key="scale" value="{scale}" />
            </filter>\n"""

    return trafo_filter


def create_scenepart_obj(filepath, trafofilter="", efilepath=False):
    """This function creates a scenepart string to load OBJ-files

    :param filepath: path to the OBJ-file
    :param trafofilter: transformation filter, surrounded by <filter>-tags (string)
    :param efilepath: boolean, whether to use the efilepath option
    :return: scenepart (string)
    """
    if efilepath is True:
        key_opt = "efilepath"
    else:
        key_opt = "filepath"
    scenepart = f"""
        <part>
            <filter type="objloader">
                <param type="string" key="{key_opt}" value="{filepath}" />
            </filter>
            {trafofilter}
        </part>"""

    return scenepart


def create_scenepart_tiff(filepath, trafofilter="",
                          matfile="data/sceneparts/basic/groundplane/groundplane.mtl", matname="None"):
    """This function creates a scenepart string to load GeoTIFFs

    :param filepath: path to the GeoTIFF-file (string)
    :param trafofilter: transformation filter, surrounded by <filter>-tags (string)
    :param matfile: path to the material file (string)
    :param matname: name of the material to use (string)

    :return: scenepart (string)
    """
    scenepart = f"""
        <part>
            <filter type="geotiffloader">
                <param type="string" key="filepath" value="{filepath}" />
                <param type="string" key="matfile" value="{matfile}" />
                <param type="string" key="matname" value="{matname}" />
            </filter>
            {trafofilter}
        </part>"""

    return scenepart


def create_scenepart_xyz(filepath, trafofilter="", sep=" ", voxel_size=0.5):
    """This function creates a scenepart string to load ASCII point clouds in xyz-format

    :param filepath: path to the ASCII point cloud file (string)
    :param trafofilter: transformation filter, surrounded by <filter>-tags (string)
    :param sep: column separator in the ASCII point cloud file; default: " " (string)
    :param voxel_size: voxel side length for the voxelisation of the point cloud (float)

    :return: scenepart (string)
    """
    scenepart = f"""
        <part>
            <filter type="xyzloader">
                <param type="string" key="filepath" value="{filepath}" />
                <param type="string" key="separator" value="{sep}" />
                <param type="double" key="voxelSize" value="{voxel_size}" />
                <!-- Normal estimation using Singular Value Decomposition (SVD)
                MODE 1: simple mode / MODE 2: advanced mode for large files, which works in batches -->
                <param type="int" key="estimateNormals" value="1" />
                <!-- If less than three points fall into one voxel, it is discarded.
                To avoid this, a default Normal can be assigned to these voxels with:-->
                <param type="vec3" key="defaultNormal" value="0;0;1" /> 
            </filter>
            {trafofilter}
        </part>"""

    return scenepart


def create_scenepart_vox(filepath, trafofilter="", intersection_mode="transmittive", matfile=None, matname=None):
    """This function creates a scenepart string to load .vox voxel files

    :param filepath: path to the .vox-file (string)
    :param trafofilter: transformation filter, surrounded by <filter>-tags (string)
    :param intersection_mode: intersection mode for voxels (string)
                    options: "transmittive" (default), "scaled", "fixed"
    :param matfile: path to the material file (string)
    :param matname: name of the material to use (string)

    :return: scenepart (string)
    """
    if matfile or matname:
        mat_def = f"""\n<param type="string" key="matfile" value="{matfile}" />
        <param type="string" key="matname" value="{matname}" />"""
    else:
        mat_def = ""
    scenepart = f"""
        <part>
            <filter type="detailedvoxels">
                <param type="string" key="intersectionMode" value="{intersection_mode}" />
                <param type="string" key="filepath" value="{filepath}" />{mat_def}
            </filter>
            {trafofilter}
        </part>"""

    return scenepart


def build_scene(scene_id, name, sceneparts=None):
    """This function creates the content to write to the scene.xml file

    :param scene_id: ID of the scene (string)
    :param name: name of the scene (string)
    :param sceneparts: list of sceneparts to add to the scene (list)

    :return: scene XML content (string)
    """
    sceneparts = "\n".join(sceneparts)
    scene_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<document>
    <scene id="{scene_id}" name="{name}">
        {sceneparts}
    </scene>
</document>"""

    return scene_content


if __name__ == "__main__":

    # Usage Demo
    bbox = [478250, 5473800, 478400, 5473950]
    wp, c, dist = compute_flight_lines(bbox, spacing=30, rotate_deg=45, flight_pattern="criss-cross")
    plot = plot_flight_plan(wp)
    plot.show()
    speed = 5
    print("Flight duration: {} min".format(dist/speed/60))
    alt = 490
    print(str(export_for_xml(wp, altitude=alt, id="uls_template", speed=speed)))

    filters = add_transformation_filters(translation=[478335.125, 5473887.89, 0.0], rotation=[0, 0, 90], on_ground=-1)

    sp = create_scenepart_tiff(r"data\sceneparts\tiff\dem_hd.tif",
                               matfile=r"data\sceneparts\basic\groundplane\groundplane.mtl",
                               matname="None")
    sp2 = create_scenepart_obj("data/sceneparts/arbaro/black_tupelo_low.obj", trafofilter=filters)

    scene = build_scene("test", "test_scene", [sp, sp2])
    print(scene)

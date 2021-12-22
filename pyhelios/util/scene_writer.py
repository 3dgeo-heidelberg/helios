#!/usr/bin/env python
# -- coding: utf-8 --

# Hannah Weiser, Heidelberg University
# December 2021
# h.weiser@uni-heidelberg.de

"""
This script contains functions to configure and write HELIOS++ scenes.
"""


def add_transformation_filters(translation: list = None,
                               rotation: list = None,
                               scale: float = 1.,
                               on_ground: int = 0):
    """This function creates a string of transformation filters for a given translation, rotation and scale

    :param translation: list of translations in x-, y- and z-direction; [t_x, t_y, t_z]
    :param rotation: list of rotations around the x-, y- and z-axes; [rot_x, rot_y, rot_z]
    :param scale: value by which to scale the scenepart
    :param on_ground: flag to specifiy whether the scenepart should be translated to the ground
                    0  = no ground translation
                    -1 = find optimal ground translation
                    1  = find quick ground translation
                    >1 = specify a depth for the search process
    :type translation: list
    :type rotation: list
    :type scale: float
    :type on_ground: int
    :return: transformation filter(s)
    :rtype: str
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
    if scale != 1.:
        trafo_filter += f"""
            <filter type="scale">
                <param type="double" key="scale" value="{scale}" />
            </filter>\n"""

    return trafo_filter


def create_scenepart_obj(filepath: str, up_axis: str = "z", trafofilter: str = "", efilepath: bool = False):
    """This function creates a scenepart string to load OBJ-files

    :param filepath: path to the OBJ-file
    :param up_axis: axis of the OBJ-file which is pointing upwards
    :param trafofilter: transformation filter, surrounded by <filter>-tags
    :param efilepath: boolean, whether to use the efilepath option
    :type filepath: str
    :type up_axis: str
    :type trafofilter: str
    :type efilepath: bool
    :return: scenepart
    :rtype: str
    """
    filepath_mode = "efilepath" if efilepath else "filepath"
    assert up_axis in ["y", "z"]
    scenepart = f"""
        <part>
            <filter type="objloader">
                <param type="string" key="{filepath_mode}" value="{filepath} up="{up_axis}" />
            </filter>
            {trafofilter}
        </part>"""

    return scenepart


def create_scenepart_tiff(filepath: str, trafofilter: str = "",
                          matfile: str = "data/sceneparts/basic/groundplane/groundplane.mtl", matname: str = "None"):
    """This function creates a scene part string to load GeoTIFFs

    :param filepath: path to the GeoTIFF-file
    :param trafofilter: transformation filter, surrounded by <filter>-tags
    :param matfile: path to the material file
    :type filepath: str
    :type trafofilter: str
    :type matfile: str
    :param matname: name of the material to use
    :return: scenepart
    :rtype: str
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


def create_scenepart_xyz(filepath: str, trafofilter: str = "", sep: str = " ", voxel_size: float = 0.5,
                         efilepath: bool = False):
    """This function creates a scenepart string to load ASCII point clouds in xyz-format

    :param filepath: path to the ASCII point cloud file
    :param trafofilter: transformation filter, surrounded by <filter>-tags
    :param sep: column separator in the ASCII point cloud file; default: " "
    :param voxel_size: voxel side length for the voxelisation of the point cloud
    :param efilepath: boolean, whether to use the efilepath option
    :type filepath: str
    :type trafofilter: str
    :type sep: str
    :type voxel_size: float
    :type efilepath: bool
    :return: scenepart
    :rtype: str
    """
    filepath_mode = "efilepath" if efilepath else "filepath"
    scenepart = f"""
        <part>
            <filter type="xyzloader">
                <param type="string" key="{filepath_mode}" value="{filepath}" />
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


def create_scenepart_vox(filepath, trafofilter="", intersection_mode="transmittive", matfile=None, matname=None,
                         efilepath: bool = False):
    """This function creates a scenepart string to load .vox voxel files

    :param filepath: path to the .vox-file
    :param trafofilter: transformation filter, surrounded by <filter>-tags
    :param intersection_mode: intersection mode for voxels
                    options: "transmittive" (default), "scaled", "fixed"
    :param matfile: path to the material file
    :param matname: name of the material to use
    :param efilepath: boolean, whether to use the efilepath option
    :type filepath: str
    :type trafofilter: str
    :type intersection_mode: str
    :type matfile: str
    :type matname: str
    :type efilepath: bool

    :return: scenepart
    :rtype: str
    """
    filepath_mode = "efilepath" if efilepath else "filepath"
    if matfile or matname:
        mat_def = f"""\n<param type="string" key="matfile" value="{matfile}" />
        <param type="string" key="matname" value="{matname}" />"""
    else:
        mat_def = ""
    scenepart = f"""
        <part>
            <filter type="detailedvoxels">
                <param type="string" key="intersectionMode" value="{intersection_mode}" />
                <param type="string" key="{filepath_mode}" value="{filepath}" />{mat_def}
            </filter>
            {trafofilter}
        </part>"""

    return scenepart


def build_scene(scene_id, name, sceneparts=None):
    """This function creates the content to write to the scene.xml file

    :param scene_id: ID of the scene
    :param name: name of the scene
    :param sceneparts: list of sceneparts to add to the scene
    :type scene_id: str
    :type name: str
    :type sceneparts: list[*str]

    :return: scene XML content
    :return: scene XML content (string)
    :rtype: str
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
    filters = add_transformation_filters(translation=[478335.125, 5473887.89, 0.0], rotation=[0, 0, 90], on_ground=-1)

    sp = create_scenepart_tiff(r"data\sceneparts\tiff\dem_hd.tif",
                               matfile=r"data\sceneparts\basic\groundplane\groundplane.mtl",
                               matname="None")
    sp2 = create_scenepart_obj("data/sceneparts/arbaro/black_tupelo_low.obj", trafofilter=filters)

    scene = build_scene("test", "test_scene", [sp, sp2])
    print(scene)

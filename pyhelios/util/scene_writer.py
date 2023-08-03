#!/usr/bin/env python
# -- coding: utf-8 --

# Hannah Weiser, Heidelberg University
# 2023
# h.weiser@uni-heidelberg.de

"""
This script contains functions to configure and write HELIOS++ scenes.
"""

import warnings
from typing import Iterable


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
    elif len(translation) < 3:
        warnings.warn("Translation is not a list of length 3 (x, y, z). Setting missing dimensions to zero.")
        for i in range(3-len(translation)):
            translation.append(0)
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


def add_motion_rotation(id: str,
                        axis, angle: float,
                        rotation_center: Iterable[float] = None, 
                        nloops: int = 0,
                        selfmode_flag: bool = False,
                        next: str = False,
                        decimal_places: int = 8):
    """This function adds a rigid motion H++ XML string composed of a rotation
    :param id: id of the motion
    :param axis: axis of rotation
    :param angle: angle of rotation
    :param rotation_center: point around which to rotate
    :param nloops: number of times the motion should be repeated (0 = repeat endlessly)
    :param selfmode_flag: 
    :param next: id of the next motion (for building sequences of motions)
    :param decimal_places: decimal places to round coordinates/distances/angles to
    :type id: str
    :type axis: Iterable[float]
    :type angle: float
    :type rotation_center: Iterable[float]
    :type nloops: int
    :type selfmode_flag: bool
    :type next: str
    :type decimal_places: int
    
    :return: rigid motion filter
    :rtype: str
    """
    axis_string = f'{axis[0]:.3f};{axis[1]:.3f};{axis[2]:.3f}'
    if next:
        next_str = f'next="{next}"'
    else:
        next_str = ''
    if selfmode_flag is True and rotation_center is None:
        selfmode_string = ' selfMode="true"'
    else:
        selfmode_string = ''
    if rotation_center is not None:
        center_string = f' center="{rotation_center[0]:.{decimal_places}f};{rotation_center[1]:.{decimal_places}f};{rotation_center[2]:.{decimal_places}f}" autoCRS="1"'
    else:
        center_string = ''
    dmfilter = f'''
            <dmotion id="{id}" loop="{nloops}" {next_str}>
              <motion type="rotation" axis="{axis_string}" angle="{angle:.{decimal_places}f}"{selfmode_string}{center_string}/>
            </dmotion>
            '''
        
    return dmfilter


def add_motion_translation(id: str, 
                           x: float,
                           y: float,
                           z: float,
                           nloops: int = 0,
                           next: str = False,
                           decimal_places: int = 8):
    """This function returns a rigid motion H++ XML string composed of a translation
    :param id: id of the motion
    :param x: translation in x-direction
    :param y: translation in y-direction
    :param z: translation in z-direction
    :param nloops: number of times the motion should be repeated (0 = repeat endlessly)
    :param next: id of the next motion (for building sequences of motions)
    :param decimal_places: decimal places to round coordinates/distances/angles to
    :type id: str
    :type x: float
    :type y: float
    :type z: float
    :type nloops: int
    :type next: str
    :type decimal_places: int
    
    :return: rigid motion filter
    :rtype: str
    """
    if next:
        next_str = f'next="{next}"'
    else:
        next_str = ''
    dmfilter = f'''
            <dmotion id="{id}" loop="{nloops}" {next_str}>
              <motion type="translation" vec="{x:.{decimal_places}f};{y:.{decimal_places}f};{z:.{decimal_places}f}"/>
            </dmotion>
            '''
        
    return dmfilter


def add_motion_rot_tran(id: str,
                        axis, angle: float,
                        x: float, y: float, z: float,
                        rotation_center: Iterable[float] = None, 
                        nloops: int = 0,
                        selfmode_flag: bool = False,
                        next: str = False,
                        decimal_places: int = 8):
    """This function adds a rigid motion H++ XML string composed of a translation and a rotation
    :param id: id of the motion
    :param axis: axis of rotation
    :param angle: angle of rotation
    :param x: translation in x-direction
    :param y: translation in y-direction
    :param z: translation in z-direction
    :param rotation_center: point around which to rotate
    :param nloops: number of times the motion should be repeated (0 = repeat endlessly)
    :param selfmode_flag: 
    :param next: id of the next motion (for building sequences of motions)
    :param decimal_places: decimal places to round coordinates/distances/angles to
    :type id: str
    :type axis: Iterable[float]
    :type angle: float
    :type x: float
    :type y: float
    :type z: float
    :type rotation_center: Iterable[float]
    :type nloops: int
    :type selfmode_flag: bool
    :type next: str
    :type decimal_places: int
    
    :return: rigid motion filter
    :rtype: str
    """
    axis_string = f'{axis[0]:.3f};{axis[1]:.3f};{axis[2]:.3f}'
    vec_string = f'{x:.{decimal_places}f};{y:.{decimal_places}f};{z:.{decimal_places}f}'
    if next:
        next_str = f'next="{next}"'
    else:
        next_str = ''
    if selfmode_flag is True and rotation_center is None:
        selfmode_string = ' selfMode="true"'
    else:
        selfmode_string = ''
    if rotation_center is not None:
        center_string = f' center="{rotation_center[0]:.{decimal_places}f};{rotation_center[1]:.{decimal_places}f};{rotation_center[2]:.{decimal_places}f}" autoCRS="1"'
    else:
        center_string = ''
    dmfilter = f'''
            <dmotion id="{id}" loop="{nloops}" {next_str}>
              <motion type="rotation" axis="{axis_string}" angle="{angle:.{decimal_places}f}"{selfmode_string}{center_string}/>
              <motion type="translation" vec="{vec_string}"/>
            </dmotion>
            '''
    return dmfilter


def create_scenepart_obj(filepath: str, up_axis: str = "z", trafofilter: str = "",
                         efilepath: bool = False, sp_id: str = None,
                         motionfilter: str = "",
                         dyn_step: int = None, kdt_dyn_step: int = None,
                         dyn_time_step: float = None, kdt_dyn_time_step: float = None):
    """This function creates a scenepart string to load OBJ-files

    :param filepath: path to the OBJ-file
    :param up_axis: axis of the OBJ-file which is pointing upwards
    :param trafofilter: transformation filter, surrounded by <filter>-tags
    :param motionfilter:
    :param efilepath: boolean, whether to use the efilepath option
    :param sp_id:
    :param dyn_step: dynamic motions will be applied every dyn_step simulation steps
    :param kdt_dyn_step: the KDT will be updated every kdt_dyn_step dynamic object updates
    :type filepath: str
    :type up_axis: str
    :type trafofilter: str
    :type motionfilter: str
    :type efilepath: bool
    :type sp_id: str
    :type dyn_step: int
    :type kdt_dyn_step: int
    :type dyn_step: float
    :type kdt_dyn_step: float
    
    :return: scenepart
    :rtype: str
    """
    filepath_mode = "efilepath" if efilepath else "filepath"
    assert up_axis in ["y", "z"]
    if sp_id is not None:
        id_string = f'id="{sp_id}"'
    else:
        id_string = ""
    if motionfilter != "":
        if dyn_time_step and kdt_dyn_time_step:
            if kdt_dyn_time_step < dyn_time_step:
                warnings.warn("kdtDynTimeStep must be equal to or greater then kdtDynStep. Ignoring kdtDynTimeStep.")
                kdt_dyn_time_step = None
        if dyn_step and dyn_time_step:
            warnings.warn("Both dynStep and dynTimeStep were provided, but they are exclusive. Using dynStep.")
        if kdt_dyn_step and kdt_dyn_time_step:
            warnings.warn("Both kdtDynStep and kdtDynTimeStep were provided, but they are exclusive. Using kdtDynStep.")
        if dyn_step:
            dyn_step_string = f'dynStep="{dyn_step}"'
        elif dyn_time_step:
            dyn_step_string = f'dynTimeStep="{dyn_time_step}"'
        else:
            dyn_step_string = ""
        if kdt_dyn_step:
            kdt_dyn_step_string = f'kdtDynStep="{kdt_dyn_step}"'
        elif kdt_dyn_time_step:
            kdt_dyn_step_string = f'kdtDynTimeStep="{kdt_dyn_time_step}"'
        else:
            kdt_dyn_step_string = ""
    else:
        dyn_step_string = ""
        kdt_dyn_step_string = ""
    scenepart = f"""
        <part {id_string} {dyn_step_string} {kdt_dyn_step_string}>
            <filter type="objloader">
                <param type="string" key="{filepath_mode}" value="{filepath}" />
                <param type="string" key="up" value="{up_axis}" />
            </filter>
            {trafofilter}
            {motionfilter}
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


def build_scene(scene_id: str, name: str, sceneparts=None, dyn_step: int = None, kdt_dyn_step: int = None, dyn_time_step: float = None, kdt_dyn_time_step: float = None):
    """This function creates the content to write to the scene.xml file

    :param scene_id: ID of the scene
    :param name: name of the scene
    :param sceneparts: list of sceneparts to add to the scene
    :param dyn_step: dynamic motions will be applied every dyn_step simulation steps
    :type scene_id: str
    :type name: str
    :type sceneparts: list[*str]
    :type dyn_step: int
    :type kdt_dyn_step: int
    :type dyn_time_step: float
    :type kdt_dyn_time_step: float

    :return: scene XML content
    :return: scene XML content (string)
    :rtype: str
    """
    sceneparts = "\n".join(sceneparts)
    if dyn_time_step and kdt_dyn_time_step:
        if kdt_dyn_time_step < dyn_time_step:
            warnings.warn("kdtDynTimeStep must be equal to or greater then kdtDynStep. Ignoring kdtDynTimeStep.")
            kdt_dyn_time_step = None
    if dyn_step and dyn_time_step:
        warnings.warn("Both dynStep and dynTimeStep were provided, but they are exclusive. Using dynStep.")
    if kdt_dyn_step and kdt_dyn_time_step:
        warnings.warn("Both kdtDynStep and kdtDynTimeStep were provided, but they are exclusive. Using kdtDynStep.")
    if dyn_step:
        dyn_step_string = f'dynStep="{dyn_step}"'
    elif dyn_time_step:
        dyn_step_string = f'dynTimeStep="{dyn_time_step}"'
    else:
        dyn_step_string = ""
    if kdt_dyn_step:
        kdt_dyn_step_string = f'kdtDynStep="{kdt_dyn_step}"'
    elif kdt_dyn_time_step:
        kdt_dyn_step_string = f'kdtDynTimeStep="{kdt_dyn_time_step}"'
    else:
        kdt_dyn_step_string = ""
    scene_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<document>
    <scene id="{scene_id}" name="{name}" {dyn_step_string} {kdt_dyn_step_string}>
        {sceneparts}
    </scene>
</document>"""

    return scene_content


if __name__ == "__main__":
    # Usage Demo
    filters = add_transformation_filters(translation=[478335.125, 5473887.89, 0.0], rotation=[0, 0, 90], on_ground=-1)

    sp = create_scenepart_tiff("data/sceneparts/tiff/dem_hd.tif",
                               matfile="data/sceneparts/basic/groundplane/groundplane.mtl",
                               matname="None")
    sp2 = create_scenepart_obj("data/sceneparts/arbaro/black_tupelo_low.obj", trafofilter=filters)

    scene = build_scene("test", "test_scene", [sp, sp2])

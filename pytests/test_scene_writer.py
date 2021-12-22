import pytest
import numpy as np
from pathlib import Path
from pyhelios.util import scene_writer
import re

WORKING_DIR = str(Path(__file__).parent.parent.absolute())


def test_add_translation_filter():
    """Test the function add_transformation_filters for a translation"""
    # given
    tr = [10, 10, 5]

    # expected:
    trafo_filter_expected = f"""
            <filter type="translate">  
                <param type="integer" key="onGround" value="0" />
                <param type="vec3" key="offset" value="{tr[0]};{tr[1]};{tr[2]}" /> 
            </filter>\n"""

    trafo_filter = scene_writer.add_transformation_filters(translation=tr)

    assert re.sub(r"\s+", " ", trafo_filter) == re.sub(r"\s+", " ", trafo_filter_expected)


def test_add_rotation_filter():
    """Test the function add_transformation_filters for a rotation"""
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

    rot_filter = scene_writer.add_transformation_filters(rotation=rot)

    assert re.sub(r"\s+", " ", rot_filter) == re.sub(r"\s+", " ", rot_filter_expected)


def test_add_scale_filter():
    """Test the function add_transformation_filters for scaling"""
    # given
    sc = 0.5

    # expected:
    scale_filter_expected = f"""
            <filter type="scale">
                <param type="double" key="scale" value="{sc}" />
            </filter>\n"""

    scale_filter = scene_writer.add_transformation_filters(scale=sc)

    assert re.sub(r"\s+", " ", scale_filter) == re.sub(r"\s+", " ", scale_filter_expected)


def test_add_translation_filter_on_ground():
    """Test the function add_transformation_filters for a translation onto ground"""
    # given
    tr = [10, 10, 5]
    on_gnd = -1

    # expected:
    trafo_filter_expected = f"""
                    <filter type="translate">  
                        <param type="integer" key="onGround" value="{on_gnd}" />
                        <param type="vec3" key="offset" value="{tr[0]};{tr[1]};{tr[2]}" /> 
                    </filter>\n"""

    trafo_filter = scene_writer.add_transformation_filters(translation=tr, on_ground=on_gnd)

    assert re.sub(r"\s+", " ", trafo_filter) == re.sub(r"\s+", " ", trafo_filter_expected)


def test_create_scenepart_obj_efilepath():
    """Test the creation of an obj scene part with the 'efilepath' option"""
    # given
    filepath = "data/sceneparts/basic/groundplane/groundplane.obj"

    # expected
    obj_filter_expected = f'''
            <part>
                <filter type="objloader">
                    <param type="string" key="efilepath" value="{filepath} up="z" />
                </filter>

            </part>'''

    obj_filter = scene_writer.create_scenepart_obj(filepath, efilepath=True)

    # raise AssertionError
    assert re.sub(r"\s+", " ", obj_filter) == re.sub(r"\s+", " ", obj_filter_expected)


def test_create_scenepart_obj_invalid_up():
    """Test if an error is raised if an invalid 'up' axis is provided for the objloaer"""
    # given
    filepath = "data/sceneparts/basic/groundplane/groundplane.obj"
    up_axis = "phi"

    # expected
    # raise AssertionError
    with pytest.raises(AssertionError) as e:
        scene_writer.create_scenepart_obj(filepath, up_axis=up_axis)
    assert e.type is AssertionError


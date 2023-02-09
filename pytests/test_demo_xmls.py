
"""
Tests for checking that the XMLs are conform with the XSD
"""

import pytest
import xmlschema
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
import urllib

HELIOS_EXE = str(Path('run') / 'helios')
if sys.platform == "win32":
    HELIOS_EXE += ".exe"
WORKING_DIR = str(Path(__file__).parent.parent.absolute())
# find helios executable
HELIOS_EXE_NAME = "helios"
if sys.platform == "win32" or sys.platform == "win64":
    HELIOS_EXE_NAME += ".exe"
HELIOS_EXE = str(list(Path(WORKING_DIR).glob(f"**/{HELIOS_EXE_NAME}"))[0])
# get XSD files and read schemas
XSD_DIR = Path(WORKING_DIR) / 'pyhelios/util/xsd'
survey_schema = xmlschema.XMLSchema(str(XSD_DIR / 'survey.xsd'))
scene_schema = xmlschema.XMLSchema(str(XSD_DIR / 'scene.xsd'))
scanner_schema = xmlschema.XMLSchema(str(XSD_DIR / 'scanner.xsd'))
platform_schema = xmlschema.XMLSchema(str(XSD_DIR / 'platform.xsd'))


def handle_relative_path(root, *paths):
    new_paths = []
    for path in paths:
        path = Path(path)
        # if path is absolute, leave as it is
        if not path.is_absolute():
            # check if path is relative to current working directory
            try:
                path.resolve(strict=True)
            except FileNotFoundError:
                # otherwise, assume that path is relative to helios directory
                path = str(root / path)
        new_paths.append(path)

    return new_paths


def get_paths(survey_file):
    # get paths of any referenced XML files; assuming they are relative to helios root dir or absolute
    try:
        scene_file = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[0]
        scanner_file = ET.parse(survey_file).find('survey').attrib['scanner'].split('#')[0]
        platform_file = ET.parse(survey_file).find('survey').attrib['platform'].split('#')[0]
        scene_file, scanner_file, platform_file = handle_relative_path(WORKING_DIR, scene_file, scanner_file,
                                                                       platform_file)
    except KeyError as e:
        print("ERROR: Missing 'platform', 'scanner' or 'scene' key in <survey> tag.\n"
              "Please check your survey file.\n")
        raise e

    return scene_file, scanner_file, platform_file


@pytest.mark.parametrize(
    "f_survey",
    [pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'als_hd_demo_tiff.xml'),
                  id="als_hd_demo_tiff.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'als_interpolated_trajectory.xml'),
                  id="als_interpolated_trajectory.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'light_als_toyblocks_multiscanner.xml'),
                  id="light_als_toyblocks_multiscanner.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'als_hd_height_above_ground_stripid.xml'),
                  id="als_hd_stripid.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'mls_wheat_demo.xml'),
                  id="mls_wheat_demo.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'tls_arbaro_demo.xml'),
                  id="tls_arbaro_demo.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'tls_livox.xml'),
                  id="tls_livox.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'toyblocks' / 'als_toyblocks.xml'),
                  id="als_toyblocks.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'toyblocks' / 'custom_als_toyblocks.xml'),
                  id="custom_als_toyblocks.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'toyblocks' / 'mls_toyblocks.xml'),
                  id="mls_toyblocks.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'toyblocks' / 'moving_tls_toyblocks.xml'),
                  id="moving_tls_toyblocks.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'toyblocks' / 'tls_toyblocks.xml'),
                  id="tls_toyblocks.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'toyblocks' / 'uls_toyblocks_livox.xml'),
                  id="uls_toyblocks_livox.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'toyblocks' / 'uls_toyblocks_stop_and_turn.xml'),
                  id="uls_toyblocks_stop_and_turn.xml"),
     # skip because survey-scene combo not supported by XSD at the moment
     # pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'toyblocks' / 'uls_toyblocks_survey_scene_combo.xml'),
     # id="uls_toyblocks_survey_scene_combo.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'voxels' / 'als_detailedVoxels_mode_comparison.xml'),
                  id="als_detailedVoxels_mode_comparison.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'voxels' / 'tls_sphere_xyzloader.xml'),
                  id="tls_sphere_xyzloader.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'voxels' / 'tls_sphere_xyzloader_normals.xml'),
                  id="tls_sphere_xyzloader_normals.xml"),
     pytest.param((Path(WORKING_DIR) / 'data' / 'surveys' / 'voxels' / 'uls_detailedVoxels_mode_comparison.xml'),
                  id="uls_detailedVoxels_mode_comparison.xml"),
     ]
)
def test_xmls(f_survey):
    # get paths of all references XML files
    f_scene, f_scanner, f_platform = get_paths(f_survey)

    # validate XML files
    xmlschema.validate(str(f_survey), survey_schema)
    xmlschema.validate(str(f_scene), scene_schema)
    xmlschema.validate(str(f_scanner), scanner_schema)
    try:
        xmlschema.validate(str(f_platform), platform_schema)
    except urllib.error.URLError:
        assert str(Path(f_platform).name) == "interpolated"

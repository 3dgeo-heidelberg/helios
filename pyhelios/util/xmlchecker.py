# Hannah Weiser, Heidelberg University
# December 2021
# h.weiser@uni-heidelberg.de
"""
This script checks all XML files of a simulation using XML schema definitions (XSDs) and then runs the simulation.

Usage: python xmlchecker.py <path-to-survey> <additional-helios-attributes>
e.g., python xmlchecker.py data/surveys/demo/als_hd_demo_tiff.xml --lasOutput --zipOutput
"""

import xmlschema
import sys
import xml.etree.ElementTree as ET
import subprocess
from pathlib import Path


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


xsd_dir = Path(__file__).parent / 'xsd'
helios_root = Path(__file__).parent.parent.parent
survey_file = Path(sys.argv[1])
survey_file = handle_relative_path(helios_root, survey_file)[0]

# find helios executable
HELIOS_EXE_NAME = "helios"
if sys.platform == "win32" or sys.platform == "win64":
    HELIOS_EXE_NAME += ".exe"
HELIOS_EXE = str(list(helios_root.glob(f"**/{HELIOS_EXE_NAME}"))[0])
print(f"Found HELIOS++ executable: {HELIOS_EXE}")

survey_schema = xmlschema.XMLSchema(str(xsd_dir / 'survey.xsd'))
scene_schema = xmlschema.XMLSchema(str(xsd_dir / 'scene.xsd'))
scanner_schema = xmlschema.XMLSchema(str(xsd_dir / 'scanner.xsd'))
platform_schema = xmlschema.XMLSchema(str(xsd_dir / 'platform.xsd'))

# get paths of any referenced XML files; assuming they are relative to helios root dir or absolute
try:
    scene_file = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[0]
    scanner_file = ET.parse(survey_file).find('survey').attrib['scanner'].split('#')[0]
    platform_file = ET.parse(survey_file).find('survey').attrib['platform'].split('#')[0]
    scene_file, scanner_file, platform_file = handle_relative_path(helios_root, scene_file, scanner_file, platform_file)
except KeyError as e:
    print("ERROR: Missing 'platform', 'scanner' or 'scene' key in <survey> tag.\n"
          "Please check your survey file.\n")
    raise e

# validate XML files
xmlschema.validate(survey_file, survey_schema)
xmlschema.validate(scene_file, scene_schema)
xmlschema.validate(scanner_file, scanner_schema)
xmlschema.validate(platform_file, platform_schema)

# call HELIOS++
subprocess.run([HELIOS_EXE, survey_file, *sys.argv[2:]])

import xmlschema
import sys
import xml.etree.ElementTree as ET
import subprocess
from pathlib import Path
import os

# change working directory in case script was not called from helios root directory
xsd_dir = Path(__file__).parent / 'xsd'
helios_root = Path(__file__).parent.parent.parent
if Path.cwd() != helios_root:
    os.chdir(helios_root)
survey_file = Path(sys.argv[1]).resolve()
survey_file = str(survey_file.relative_to(helios_root))

# check where helios.exe is located
HELIOS_EXE_NAME = "helios"
if sys.platform == "win32" or sys.platform == "win64":
    HELIOS_EXE_NAME += ".exe"

HELIOS_EXE = str(list(Path.cwd().glob(f"**/{HELIOS_EXE_NAME}"))[0])
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

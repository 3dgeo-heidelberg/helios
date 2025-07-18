import xml.etree.ElementTree as ET
from pathlib import Path
import os


def display_xml(path, item=None):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.parse(path, parser=parser)
    tree = root.getroot()
    # ET.indent(tree)
    if item is None:
        # ET.indent(tree)
        return ET.tostring(tree, encoding="unicode")
    for e in tree:
        if "id" in e.attrib and e.attrib["id"] == item:
            # ET.indent(e)
            return ET.tostring(e, encoding="unicode")


def find_playback_dir(survey_path, helios_root=None):
    if not helios_root:
        helios_root = os.getcwd()
    if not Path(survey_path).is_absolute():
        survey_path = Path(helios_root) / survey_path
    playback = Path(helios_root) / "output"
    tree = ET.parse(survey_path)
    root = tree.getroot()
    survey_name = root.find("survey").attrib["name"]
    if not (playback / survey_name).is_dir():
        raise FileNotFoundError(
            f"Could not locate output directory: {playback / survey_name}"
        )
    last_run_dir = sorted(
        list((playback / survey_name).glob("*")),
        key=lambda f: f.stat().st_ctime,
        reverse=True,
    )[0]
    return last_run_dir

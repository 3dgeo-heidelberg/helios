#!/usr/bin/env python
# -- coding: utf-8 --
# Hannah Weiser 2021, h.weiser@stud.uni-heidelberg.de

"""
This script creates a compressed archive containing all configuration
XML-files necessary to run the specified survey(s) with the specified helios executable.

usage: python extra\archive.py <path_to_helios.exe>  <path-to-survey> <path-to-zipped_outfile>
e.g. python extra\archive.py run\helios.exe data\surveys\toyblocks\als_toyblocks.xml archive.zip

instead of a single survey XML-file, a text file can be specified which contains one survey path per line
e.g. python extra\archive.py run\helios.exe list.txt archive.zip

The archive additionally contains a text file (version.txt) with the helios version and the DOI to the zenodo release.
"""


import sys
from pathlib import Path
import zipfile
from urllib.request import urlopen
from urllib.error import URLError, HTTPError
import platform
import subprocess
from shutil import rmtree
import xml.etree.ElementTree as ET
import itertools

WORKING_DIR = str(Path(__file__).parent.parent.absolute())


class Simulation:

    def __init__(self, survey_file):
        """
        Init constructs all necessary attributes from survey file.
        """
        self.survey = survey_file
        self.scene_name = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[1]
        self.sp_files = []
        self.mat_files = []
        self.scene_file = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[0]
        self.scanner_file = ET.parse(survey_file).find('survey').attrib['scanner'].split('#')[0]
        self.platform_file = ET.parse(survey_file).find('survey').attrib['platform'].split('#')[0]
        self.scene_root = ET.parse(self.scene_file).getroot()
        with open(self.scene_file, 'r') as f:
            self.scene_content = f.read()


    def get_sceneparts(self):
        """"""

        for scene in self.scene_root.findall('scene'):
            if scene.attrib['id'] == self.scene_name:
                for scene_part in scene.findall('part'):
                    loader = scene_part.find('filter').attrib['type']
                    mat_file = None
                    sp_file = None
                    if loader == "geotiffloader":
                        sp_file = scene_part.find('filter').find('param').attrib['value']
                        self.sp_files.append(sp_file)
                    elif loader in ["xyzloader", "detailedVoxels"]:
                        for param in scene_part.findall('param'):
                            if param.attrib['key'] == 'filepath':
                                sp_file = param.attrib['value']
                                self.sp_files.append(sp_file)
                            elif param.attrib['key'] == 'matfile':
                                mat_file = param.attrib['value']
                                self.mat_files.append(mat_file)
                            elif param.attrib['key'] == 'filepath':
                                sp_file = param.attrib['value']
                                self.sp_files.append(sp_file)
                    elif loader == "objloader":
                        if scene_part.find('filter').find('param').attrib['key'] == 'efilepath':
                            pattern = sp_path.replace(".*", "*")  # will not work for other regex..
                            all_sp_files = list(Path('.').glob(pattern))
                        elif scene_part.find('filter').find('param').attrib['key'] == 'filepath':
                            all_sp_files = [scene_part.find('filter').find('param').attrib['value']]
                        self.sp_files.extend(all_sp_files)
                        for sp_file in all_sp_files:
                            with open(sp_file, "r") as f:
                                for line in f:
                                    if line.startswith('mtllib'):
                                        mat_file_name = line.split(" ")[1].strip()
                                        mat_file = Path(sp_file).parent / mat_file_name
                                        self.mat_files.append(mat_file)
                                        break


def get_version_number(helios_executable):
    try:
        helios_run = subprocess.run([helios_executable, "--version"], stdout=subprocess.PIPE, text=True)
    except Exception as e:
        print("Error. Could not execute helios:")
        print(e)
    idx1 = helios_run.stdout.find("VERSION ") + len("VERSION ")
    idx2 = helios_run.stdout.find("\n", idx1)
    return helios_run.stdout[idx1:idx2]


if __name__ == '__main__':

    HELIOS_EXE = Path(sys.argv[1])
    if sys.platform == "win32" and HELIOS_EXE.suffix == "":
        HELIOS_EXE += ".exe"
    RUN_PATH = HELIOS_EXE.parent
    sys.path.append(RUN_PATH)

    survey_path = Path(sys.argv[2])
    outfile = Path(sys.argv[3])
    allowed_suffixes = [".zip", ".7z", ".rar", ".gz", ".tar"]
    if len(set(allowed_suffixes).intersection(outfile.suffixes)) == 0:
        outfile = outfile.with_suffix(".zip")

    try:
        import pyhelios
        helios_version = str(pyhelios.getVersion())
    except Exception as e:
        helios_version = get_version_number(str(HELIOS_EXE))
    print("Your HELIOS++ version is %s" % helios_version)

    print("Writing data")
    if survey_path.suffix == ".txt":
        with open(survey_path, "r") as inf:
            surveys = inf.readlines()
            surveys = [Path(surveys[i].strip()) for i in range(len(surveys))]
    elif survey_path.suffix == ".xml":
        surveys = [Path(survey_path)]

    outzip = zipfile.ZipFile(outfile, "w")  # , compression="ZIP_DEFLATED")

    for survey in surveys:
        sim = Simulation(survey)
        sim.get_sceneparts()

        for sp_path in itertools.chain(sim.sp_files, sim.mat_files):
            if Path(sp_path) not in [Path(file) for file in outzip.namelist()]:
                outzip.write(sp_path)

        if sim.scene_file not in [Path(file) for file in outzip.namelist()]:
            if Path(sim.scene_file).is_absolute():
                scene_file_new = Path(sim.scene_file).relative_to(WORKING_DIR)
            else:
                scene_file_new = Path(sim.scene_file)
            outzip.writestr(str(scene_file_new), sim.scene_content)
            #try:
            #    scene_path_built = Path(str(scene_path_new).replace(".xml", ".scene"))
            #    outzip.write(scene_path_built)
            #except FileNotFoundError:
            #    continue

        if sim.platform_file not in [Path(file) for file in outzip.namelist()]:
            if Path(sim.platform_file).is_absolute():
                platform_file_new = Path(sim.platform_file).relative_to(WORKING_DIR)
            else:
                platform_file_new = Path(sim.platform_file)
            outzip.write(platform_file_new)

        if sim.scanner_file not in [Path(file) for file in outzip.namelist()]:
            if Path(sim.scanner_file).is_absolute():
                scanner_file_new = Path(sim.scanner_file).relative_to(WORKING_DIR)
            else:
                scanner_file_new = Path(sim.scanner_file)
            outzip.write(scanner_file_new)

        with open(survey, "r") as survey_file:
            # replace absolute with relative filepaths
            content = survey_file.read()
            content = content.replace(str(sim.scene_file), str(scene_file_new))
            content = content.replace(str(sim.platform_file), str(platform_file_new))
            content = content.replace(str(sim.scanner_file), str(scanner_file_new))
        if survey.is_absolute():
            outzip.writestr(str(survey.relative_to(WORKING_DIR)), content)
        else:
            outzip.writestr(str(survey), content)


    print("Writing assets")
    assets_path = Path("assets").glob("**/*")
    for path in assets_path:
        outzip.write(path)

    path = None
    print("Writing run")
    run_path = Path("run").glob("*")
    for path in run_path:
        if path.parts[-1] != "helios.exe":
            outzip.write(path)
    outzip.write(HELIOS_EXE)
    zipurl = None
    if not Path("run").exists():
        print("There is no run-folder in your root directory. Downloading run folder from GitHub repository")
        if platform.system() == "Windows":
            zipurl = "https://github.com/3dgeo-heidelberg/helios/releases/download/v" + helios_version \
                     + "/helios-plusplus-win.zip"
        elif platform.system() == "Linux":
            zipurl = "https://github.com/3dgeo-heidelberg/helios/releases/download/v" + helios_version \
                     + "/helios-plusplus-lin.tar.gz"

    release_url = "https://github.com/3dgeo-heidelberg/helios/releases/tag/v" + helios_version
    try:
        resp = urlopen(release_url)
        search_text = 'https://doi.org/'
        end_text = '"'
        content = str(resp.read())
        idx = content.find(search_text)
        idx_end = content.find(end_text, idx)
        doi = content[idx:idx_end]
        helios_doi_version = helios_version
    # except HTTPError or URLError:
    except Exception as e:
        doi = "https://doi.org/10.5281/zenodo.4674914"
        helios_doi_version = "1.0.6"


    if zipurl is not None:
        try:
            zipresp = urlopen(zipurl)
        except HTTPError or URLError:
            zipurl = "https://github.com/3dgeo-heidelberg/helios/releases/download/v1.0.6/helios-plusplus-win.zip"
            zipresp = urlopen(zipurl)

        temp_dir = WORKING_DIR.joinpath("tmp")
        if not temp_dir.exists():
            temp_dir.mkdir()
        with open(temp_dir.joinpath("tempfile.zip"), "wb") as tempzip:
            tempzip.write(zipresp.read())
        zf = zipfile.ZipFile(temp_dir.joinpath("tempfile.zip"))
        list_files = zf.namelist()
        for filename in list_files:
            if "run" in Path(filename).parts:
                zf.extract(filename, temp_dir)
                outzip.write(temp_dir.joinpath(filename), filename)
        zf.close()

        rmtree(temp_dir.absolute())

    print("Writing version.txt")
    outzip.writestr("version.txt", "v%s\nDOI: %s (v%s)" % (helios_version, doi, helios_doi_version))

    outzip.close()
    print("Your HELIOS++ archive has been written to %s" % outfile.absolute())

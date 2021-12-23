#!/usr/bin/env python
# -- coding: utf-8 --
# Hannah Weiser 2021, h.weiser@stud.uni-heidelberg.de

"""
This script creates a compressed archive containing all configuration
XML-files necessary to run the specified survey(s) with the specified helios executable.

usage: python pyhelios/util/archive.py <path_to_helios.exe>  <path-to-survey> <path-to-zipped_outfile>
e.g. python pyhelios/util/archive.py run/helios.exe data/surveys/toyblocks/als_toyblocks.xml archive.zip

instead of a single survey XML-file, a text file can be specified which contains one survey path per line
e.g. python pyhelios/util/archive.py run/helios.exe list.txt archive.zip

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
import warnings

WORKING_DIR = str(Path(__file__).parent.parent.absolute())
HELIOS_DOI = "https://doi.org/10.5281/zenodo.4452870"


class Simulation:
    """
    Class for a simulation.

    When initiated with a survey XML file, the survey content and paths to other referenced files (platform, scanner,
    scene) are retrieved.
    With the **get_sceneparts()** method, the file paths to all scene parts used in the referenced scene file are
    obtained and stored in lists.
    This way, the class can be used to retrieve all files necessary to reproduce a given survey.
    """

    def __init__(self, survey_file):
        """
        Init constructs all necessary attributes from survey file.

        :param survey_file: Path to survey file
        :type survey_file: str
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
        """
        Function to collect paths of scene parts from the survey file.
        These will be written to the lists sp_file and mat_files of the instance
        """

        for scene in self.scene_root.findall('scene'):
            if scene.attrib['id'] == self.scene_name:
                for scene_part in scene.findall('part'):
                    loader = scene_part.find('filter').attrib['type']
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
                        all_sp_files = []
                        if scene_part.find('filter').find('param').attrib['key'] == 'efilepath':
                            sp_file = scene_part.find('filter').find('param').attrib['value']
                            pattern = sp_file.replace(".*", "*")  # will not work for other regex..
                            all_sp_files = list(Path('.').glob(pattern))
                            self.sp_files.extend(all_sp_files)
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
    """
    Function to get the version number of HELIOS++ for a given executable
    :param helios_executable: Path to HELIOS++ executable
    :type helios_executable: str
    :return: HELIOS version number (e.g., 1.0.9)
    :rtype: str
    """
    try:
        print("Running helios")
        helios_run = subprocess.run([helios_executable, "--version"], stdout=subprocess.PIPE, text=True)
    except FileNotFoundError as exc:
        warnings.warn("Error. Could not execute helios:")
        raise exc
    idx1 = helios_run.stdout.find("VERSION ") + len("VERSION ")
    idx2 = helios_run.stdout.find("\n", idx1)

    return helios_run.stdout[idx1:idx2]


def get_latest_helios_version():
    try:
        resp = urlopen(HELIOS_DOI)
        search_text = 'Version '
        end_text = ' '
        html_content = str(resp.read())
        idx = html_content.find(search_text) + len(search_text)
        idx_end = html_content.find(end_text, idx)
        latest_version = html_content[idx:idx_end]
    except URLError:
        warnings.warn("Zenodo DOI not available.")
        latest_version = '1.1.0'
    except HTTPError:
        warnings.warn("Zenodo DOI not available.")
        latest_version = '1.1.0'

    return latest_version


if __name__ == '__main__':

    HELIOS_EXE = Path(sys.argv[1]).resolve()
    if platform.system() == 'Windows' and HELIOS_EXE.suffix == '':
        HELIOS_EXE += '.exe'
    RUN_PATH = HELIOS_EXE.parent
    sys.path.append(str(RUN_PATH))
    survey_path = Path(sys.argv[2])
    outfile = Path(sys.argv[3])
    allowed_suffixes = ['.zip', '.7z', '.rar', '.gz', '.tar']
    if len(set(allowed_suffixes).intersection(outfile.suffixes)) == 0:
        outfile = outfile.with_suffix('.zip')
    helios_version_latest = get_latest_helios_version()

    try:
        import pyhelios

        helios_version = str(pyhelios.getVersion())
    except ModuleNotFoundError as e:
        print(e)
        try:
            helios_version = get_version_number(str(HELIOS_EXE))
        except FileNotFoundError:
            print("No HELIOS++ executable found. Using the latest.")
            helios_version = helios_version_latest
    print(f'HELIOS++ version is {helios_version}')

    print('Writing data')
    if survey_path.suffix == '.txt':
        with open(survey_path, 'r') as inf:
            surveys = inf.readlines()
            surveys = [Path(surveys[i].strip()) for i in range(len(surveys))]
    elif survey_path.suffix == '.xml':
        surveys = [Path(survey_path)]
    else:
        raise ValueError(f'Survey(s) must be provided in a .txt file or as a single .xml file but '
                         f'a file with suffix "{survey_path.suffix}" was provided')

    outzip = zipfile.ZipFile(outfile, 'w')  # , compression="ZIP_DEFLATED")

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
            # try:
            #     scene_path_built = Path(str(scene_file_new).replace('.xml', '.scene'))
            #     outzip.write(scene_path_built)
            # except FileNotFoundError:
            #     continue

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

        with open(survey, "r") as f_survey:
            # replace absolute with relative file paths
            content = f_survey.read()
            content = content.replace(str(sim.scene_file), str(scene_file_new))
            content = content.replace(str(sim.platform_file), str(platform_file_new))
            content = content.replace(str(sim.scanner_file), str(scanner_file_new))
        if survey.is_absolute():
            outzip.writestr(str(survey.relative_to(WORKING_DIR)), content)
        else:
            outzip.writestr(str(survey), content)

    print('Writing assets')
    assets_path = Path('assets').glob('**/*')
    for path in assets_path:
        outzip.write(path)

    path = None
    print('Writing run')
    run_path = Path('run').glob('*')
    for path in run_path:
        if not path.parts[-1].startswith('helios'):
            outzip.write(path)
    outzip.write(HELIOS_EXE)
    zipurl = None
    if not Path('run').exists():
        warnings.warn('There is no run-folder in your root directory. Downloading run folder from GitHub repository')
        if platform.system() == 'Windows':
            zipurl = f'https://github.com/3dgeo-heidelberg/helios/releases/download/' \
                     f'v{helios_version}/helios-plusplus-win.zip'
        elif platform.system() == 'Linux':
            zipurl = f'https://github.com/3dgeo-heidelberg/helios/releases/download/' \
                     f'v{helios_version}/helios-plusplus-lin.tar.gz'

    if zipurl is not None:
        try:
            zipresp = urlopen(zipurl)
        except HTTPError or URLError:
            warnings.warn("No release available for your HELIOS++ version. Downloading latest release from GitHub.")
            if platform.system() == 'Windows':
                zipurl = f'https://github.com/3dgeo-heidelberg/helios/releases/download/v{helios_version_latest}/' \
                         f'helios-plusplus-win.zip'
            elif platform.system() == 'Linux':
                zipurl = f'https://github.com/3dgeo-heidelberg/helios/releases/download/v{helios_version_latest}/' \
                         f'helios-plusplus-lin.tar.gz'
            zipresp = urlopen(zipurl)

        temp_dir = Path(WORKING_DIR) / 'tmp'
        if not temp_dir.exists():
            temp_dir.mkdir()
        with open(temp_dir.joinpath('tempfile.zip'), 'wb') as tempzip:
            tempzip.write(zipresp.read())
        zf = zipfile.ZipFile(temp_dir.joinpath('tempfile.zip'))
        list_files = zf.namelist()
        for filename in list_files:
            if "run" in Path(filename).parts:
                zf.extract(filename, temp_dir)
                outzip.write(temp_dir.joinpath(filename), filename)
        zf.close()

        rmtree(temp_dir.absolute())

    print('Writing version.txt')
    outzip.writestr('version.txt', f'v{helios_version}\nDOI: {HELIOS_DOI}')

    outzip.close()
    print(f'Your HELIOS++ archive has been written to {outfile.absolute()}')

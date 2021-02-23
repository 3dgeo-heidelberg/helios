#!/usr/bin/env python
# -- coding: utf-8 --

"""
This script creates a compressed archive with a ready-to-use HELIOS++ distribution containing all configuration
XML-files necessary to run the specified survey(s) with the specified helios.exe.

usage: python extra\archive.py <path-to-survey> <path_to_helios.exe> <path-to-zipped_outfile>
e.g. python extra\archive.py data\surveys\toyblocks\als_toyblocks.xml run\helios.exe archive.zip

instead of a single survey XML-file, a text file can be specified which contains one survey path per line
e.g. python extra\archive.py list.txt run\helios.exe archive.zip

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


def get_version_number(path):
    try:
        helios_run = subprocess.run([path, "--version"], stdout=subprocess.PIPE, text=True)
    except Exception as e:
        print("Error. Could not execute helios:")
        print(e)
    idx1 = helios_run.stdout.find("VERSION ") + len("VERSION ")
    idx2 = helios_run.stdout.find("\n", idx1)
    return helios_run.stdout[idx1:idx2]


def remove_duplicates(a_list):
    return list(dict.fromkeys(a_list))


#def replace_abs_path(fileobj, search, replace):
#        content.replace(search, replace)
#
#    return content


def get_xml(survey, search_text, root):
    with open(survey, "r") as survey_file:
        search_text_end = "#"
        for line in survey_file.readlines():
            if search_text in line:
                start_idx = line.find(search_text) + len(search_text) + 1
                end_idx = line.find(search_text_end, start_idx)
                xml_path = Path(line[start_idx:end_idx])
                if xml_path.is_absolute():
                    xml_path_new = xml_path.relative_to(root)
                else:
                    xml_path_new = xml_path
                break

    return xml_path, xml_path_new


def get_sceneparts(scene, root):
    with open(scene, "r") as scene_file:
        content = scene_file.read()
        scene_file.seek(0, 0)
        search_text_key = '="filepath"'
        search_text_value = "value="
        scenepart_paths = []
        for line in scene_file.readlines():
            if search_text_key in line:
                start_idx = line.find(search_text_value) + len(search_text_value) + 1
                end_idx = line.find('"', start_idx)
                if end_idx is None:
                    end_idx = line.find("'", start_idx)
                sp_path = line[start_idx:end_idx]
                if Path(sp_path).is_absolute():
                    abs_path = sp_path
                    sp_path = Path(sp_path).relative_to(root)
                    content = content.replace(abs_path, str(sp_path))
                scenepart_paths.append(Path(sp_path))

    return remove_duplicates(scenepart_paths), content


root = Path.cwd()
executable = Path(sys.argv[1])
if executable.suffix == "":
    executable = executable.with_suffix(".exe")
survey_path = Path(sys.argv[2])
search_dir = executable.parent.absolute().relative_to(root)
sys.path.append(str(search_dir)+"/")
outfile = Path(sys.argv[3])
allowed_suffixes = [".zip", ".7z", ".rar", ".gz", ".tar"]
if len(set(allowed_suffixes).intersection(outfile.suffixes)) == 0:
    outfile = outfile.with_suffix(".zip")

try:
    import pyhelios
    helios_version = str(pyhelios.getVersion())
except Exception as e:
    helios_version = get_version_number(str(executable))
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
    scene_path, scene_path_new = get_xml(survey, "scene=", root)

    scenepart_paths, scene_text = get_sceneparts(scene_path, root)
    for sp_path in scenepart_paths:
        if Path(sp_path) not in [Path(file) for file in outzip.namelist()]:
            outzip.write(sp_path)

    if scene_path not in [Path(file) for file in outzip.namelist()]:
        outzip.writestr(str(scene_path_new), scene_text)

    platform_path, platform_path_new = get_xml(survey, "platform=", root)
    if platform_path not in [Path(file) for file in outzip.namelist()]:
        outzip.write(platform_path_new)

    scanner_path, scanner_path_new = get_xml(survey, "scanner=", root)
    if scanner_path not in [Path(file) for file in outzip.namelist()]:
        outzip.write(scanner_path_new)

    with open(survey, "r") as survey_file:
        # replace absolute with relative filepaths
        content = survey_file.read()
        content = content.replace(str(scene_path), str(scene_path_new))
        content = content.replace(str(platform_path), str(platform_path_new))
        content = content.replace(str(scanner_path), str(scanner_path_new))
    if survey.is_absolute():
        outzip.writestr(survey.relative_to(root), content)
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
outzip.write(executable)
zipurl = None
if not Path("run").exists():
    print("There is no run-folder in your root directory. Downloading run folder from GitHub repository")
    if platform.system() == "Windows":
        zipurl = "https://github.com/3dgeo-heidelberg/helios/releases/download/v" + helios_version +  "/helios-plusplus-win.zip"
    elif platform.system() == "Linux":
        zipurl = "https://github.com/3dgeo-heidelberg/helios/releases/download/v" + helios_version + "/helios-plusplus-lin.tar.gz"

release_url = "https://github.com/3dgeo-heidelberg/helios/releases/tag/v" + helios_version
try:
    resp = urlopen(release_url)
    search_text = 'https://doi.org/'
    end_text = '"'
    content = str(resp.read())
    idx = content.find(search_text)
    idx_end = content.find(end_text, idx)
    doi = content[idx:idx_end]
#except HTTPError or URLError:
except Exception as e:
    doi = "https://doi.org/10.5281/zenodo.4498305"
    helios_doi_version = "1.0.3"


if zipurl is not None:
    try:
        zipresp = urlopen(zipurl)
    except HTTPError or URLError:
        zipurl = "https://github.com/3dgeo-heidelberg/helios/releases/download/v1.0.3/helios-plusplus-win.zip"
        zipresp = urlopen(zipurl)

    temp_dir = root.joinpath("tmp")
    if not temp_dir.exists():
        temp_dir.mkdir()
    with open(temp_dir.joinpath("tempfile.zip"), "wb") as tempzip:
        tempzip.write(zipresp.read())
    zf = zipfile.ZipFile(temp_dir.joinpath("tempfile.zip"))
    list_files = zf.namelist()
    for filename in list_files:
        if "run" in Path(filename).parts:
            zf.extract(filename, temp_dir)
            outzip.write(temp_dir.relative_to(temp_dir).joinpath(filename))
    zf.close()

    rmtree(temp_dir.absolute())

print("Writing version.txt")
outzip.writestr("version.txt", "v%s\nDOI: %s (v%s)" % (helios_version, doi, helios_doi_version))

outzip.close()
print("Your HELIOS++ archive has been written to %s" % outfile.absolute())
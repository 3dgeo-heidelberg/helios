#!/usr/bin/env python
# -- coding: utf-8 --

"""
Tests for the Python bindings
"""

import pytest
import numpy as np
from pathlib import Path
import os
import time
import struct
import xml.etree.ElementTree as ET
import laspy

import pyhelios
from .test_demo_scenes import find_playback_dir
from . import pcloud_utils as pcu


def find_scene(survey_file):
    """helper function which returns the path to the scene XML file"""
    scene_file = ET.parse(survey_file).find("survey").attrib["scene"].split("#")[0]
    return scene_file.replace(".xml", ".scene")


def access_output(outpath, outfile_ending):
    # Attempt to rename files in output directory
    for filename in os.listdir(os.path.dirname(outpath)):
        if filename.endswith(outfile_ending):
            os.rename(
                os.path.join(os.path.dirname(outpath), filename),
                os.path.join(
                    os.path.dirname(outpath),
                    filename.replace(outfile_ending, "_accessed" + outfile_ending),
                ),
            )


def get_las_version(las_filename):
    with open(las_filename, "rb") as las_file:
        las_file.seek(24)
        header_content = las_file.read(2)
        format_str = "1s1s"
        version_minor, version_major = struct.unpack(format_str, header_content)

        return int.from_bytes(version_minor, byteorder="big"), int.from_bytes(
            version_major, byteorder="big"
        )


@pytest.fixture(scope="session")
def test_sim(output_dir):
    """
    Fixture which returns a simulation object for a given survey path
    """

    def create_test_sim(survey_path, zip_output=True, las_output=True, las10=False):
        # pyhelios.loggingSilent()
        from pyhelios import SimulationBuilder

        simB = SimulationBuilder(
            surveyPath=str(survey_path.absolute()),
            assetsDir=[str(Path("assets"))],
            outputDir=str(output_dir),
        )
        simB.setLasOutput(las_output)
        simB.setLas10(las10)
        simB.setRebuildScene(True)
        simB.setZipOutput(zip_output)

        sim = simB.build()

        return sim

    return create_test_sim


# BELOW IS COMMENTED TO PREVENT OUT OF MEMORY ERRORS
# @pytest.mark.parametrize("survey_path,las,zip",
# [(Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks.xml', True, True),
# (Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks_stripid.xml', True, True),
# (Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks.xml', False, False),
# (Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks_stripid.xml', False, False)])
# def test_open_output(test_sim, survey_path, las, zip):
# """Test accessing output files (LAS/xyz) after simulation"""
# from pyhelios import SimulationBuilder

# sim = test_sim(survey_path, las_output=las, zip_output=zip)
# sim.start()
# out = sim.join()
# ext = None

# if las is True and zip is True:
# ext = '.laz'
# elif las is True and zip is False:
# ext = '.las'
# elif las is False and zip is False:
# ext = '.xyz'
# elif las is False and zip is True:
# ext = '.bin'

# assert ext != None
# access_output(out.filepath, ext)


def test_open_output_xyz_stripid(test_sim):
    """Test accessing xyz output after simulation when using strip ID"""
    survey_path = Path("data") / "test" / "als_hd_height_above_ground_stripid_light.xml"
    sim = test_sim(survey_path, las_output=False, zip_output=False)
    sim.start()
    out = sim.join()
    access_output(out.filepath, ".xyz")


def test_open_output_xyz(test_sim):
    """Test accessing xyz output files after simulation"""
    survey_path = (
        Path("data") / "surveys" / "demo" / "light_als_toyblocks_multiscanner.xml"
    )
    sim = test_sim(survey_path, las_output=False, zip_output=False)
    sim.start()
    out = sim.join()
    access_output(out.filepath, ".xyz")


def test_open_output_laz_stripid(test_sim):
    """Test accessing LAZ output files after simulation when using strip ID"""
    survey_path = Path("data") / "test" / "als_hd_height_above_ground_stripid_light.xml"
    sim = test_sim(survey_path, las_output=True, zip_output=True, las10=True)
    sim.start()
    out = sim.join()
    v_minor, v_major = get_las_version(out.filepath)
    assert v_minor == 1
    assert v_major == 0
    access_output(out.filepath, ".laz")


def test_open_output_laz(test_sim):
    """Test accessing LAZ output files after simulation"""
    survey_path = (
        Path("data") / "surveys" / "demo" / "light_als_toyblocks_multiscanner.xml"
    )
    sim = test_sim(survey_path, las_output=True, zip_output=True)
    sim.start()
    out = sim.join()
    v_minor, v_major = get_las_version(out.filepath)
    assert v_minor == 1
    assert v_major == 4
    access_output(out.filepath, ".laz")


def test_start_stop(test_sim):
    """Test starting, pausing and stopping of simulation"""
    sim = test_sim(Path("data") / "surveys" / "toyblocks" / "als_toyblocks.xml")
    sim.start()
    assert sim.isStarted()
    time.sleep(0.05)
    sim.pause()
    assert sim.isPaused()
    sim.resume()
    assert sim.isRunning()
    while sim.isRunning():
        pass
    assert sim.isFinished()
    sim.stop()
    assert sim.isStopped()
    assert sim.isRunning() is False
    sim.join()


def test_templates(test_sim):
    """Test accessing template settings defined in a survey XML"""
    sim = test_sim(Path("data") / "surveys" / "toyblocks" / "als_toyblocks.xml")
    leg = sim.sim.getLeg(0)
    ss = leg.getScannerSettings()
    assert ss.hasTemplate()
    ss_templ = ss.getTemplate()
    ps = leg.getPlatformSettings()
    assert ps.hasTemplate()
    ps_templ = ps.getTemplate()

    assert ss_templ.id == "scanner1"
    assert ss_templ.active is True
    assert ss_templ.pulseFreq == 200_000
    assert ss_templ.trajectoryTimeInterval == 0.01
    assert ss_templ.scanFreq == 80
    assert ss_templ.scanAngle * 180 / np.pi == 20
    assert ps_templ.id == "platform1"
    assert ps_templ.movePerSec == 30


def test_survey_characteristics(test_sim):
    """Test accessing survey characteristics (name, length)"""
    path_to_survey = Path("data") / "surveys" / "toyblocks" / "als_toyblocks.xml"
    sim = test_sim(path_to_survey)
    survey = sim.sim.getSurvey()
    assert survey.name == "toyblocks_als"
    assert survey.getLength() == 0.0
    survey.calculateLength()
    assert survey.getLength() == 400.0


def test_scene():
    pass


def test_create_survey(output_dir):
    """Test creating/configuring a survey with pyhelios"""
    pyhelios.setDefaultRandomnessGeneratorSeed("7")
    test_survey_path = "data/surveys/test_survey.xml"

    # default survey (missing platform and scanner definition and not containing any legs)
    survey = """<?xml version="1.0" encoding="UTF-8"?>
    <document>
        <survey name="test_scan" scene="data/scenes/toyblocks/toyblocks_scene.xml#toyblocks_scene" platform="data/platforms.xml#sr22" scanner="data/scanners_als.xml#leica_als50">
        </survey>
    </document>
    """

    with open(test_survey_path, "w") as f:
        f.write(survey)

    # build base simulation
    simBuilder = pyhelios.SimulationBuilder(
        test_survey_path, "assets/", str(output_dir)
    )
    simBuilder.setFinalOutput(True)
    simBuilder.setLasOutput(True)
    simBuilder.setZipOutput(True)
    simBuilder.setCallbackFrequency(100)
    simBuilder.setRebuildScene(True)
    simBuilder.setNumThreads(1)
    simBuilder.setKDTJobs(1)

    simB = simBuilder.build()

    # list of waypoints
    waypoints = [
        [100.0, -100.0],
        [-100.0, -100.0],
        [-100.0, -50.0],
        [100.0, -50.0],
        [100.0, 0.0],
        [-100.0, 0.0],
        [-100.0, 50.0],
        [100.0, 50.0],
        [100.0, 100.0],
        [-100.0, 100.0],
    ]

    # settings
    altitude = 500
    speed = 150
    pulse_freq = 10_000
    scan_angle = 30 * np.pi / 180
    scan_freq = 20
    shift = simB.sim.getScene().getShift()
    for j, wp in enumerate(waypoints):
        leg = simB.sim.newLeg(j)
        leg.serialId = j
        leg.getPlatformSettings().x = wp[0] - shift.x
        leg.getPlatformSettings().y = wp[1] - shift.y
        leg.getPlatformSettings().z = altitude - shift.z
        leg.getPlatformSettings().movePerSec = speed
        leg.getScannerSettings().trajectoryTimeInterval = 0.001
        leg.getScannerSettings().pulseFreq = pulse_freq
        leg.getScannerSettings().scanAngle = scan_angle
        leg.getScannerSettings().scanFreq = scan_freq
        # scanner should only be active for legs with even ID
        if j % 2 != 0:
            leg.getScannerSettings().active = False
    survey = simB.sim.getSurvey()
    survey.calculateLength()

    # check length of survery and number of legs
    assert survey.getLength() == 1200.0
    assert simB.sim.getNumLegs() == 10

    simB.start()
    output = simB.join()
    meas, traj = pyhelios.outputToNumpy(output)
    # check length of output
    assert meas.shape == (10531, 17)
    assert traj.shape == (6670, 7)
    # compare individual points
    np.testing.assert_allclose(meas[100, :3], np.array([83.32, -66.44204, 0.03114649]))
    np.testing.assert_allclose(
        traj[0, :3], np.array([waypoints[0][0], waypoints[0][1], altitude])
    )


def test_material(test_sim):
    """Test accessing material properties of a primitive in a scene"""
    sim = test_sim(Path("data") / "surveys" / "toyblocks" / "als_toyblocks.xml")
    scene = sim.sim.getScene()
    prim0 = scene.getPrimitive(0)  # get first primitive
    mat0 = prim0.getMaterial()
    assert mat0.name == "None"
    assert mat0.isGround is True
    assert (
        Path(mat0.matFilePath)
        == Path.cwd() / "data/sceneparts/basic/groundplane/groundplane.mtl"
    )
    assert mat0.reflectance == 50.0
    assert mat0.specularity == 0.0
    assert mat0.specularExponent == 0.0
    assert mat0.classification == 0
    assert np.round(mat0.kd0, 2) == 0.20


def test_scanner(test_sim):
    """Test accessing scanner configurations with pyhelios"""
    path_to_survey = Path("data") / "test" / "als_hd_demo_tiff_min.xml"
    sim = test_sim(path_to_survey)
    scanner = sim.sim.getScanner()
    assert scanner.deviceId == "leica_als50-ii"
    assert scanner.averagePower == 4.0
    assert scanner.beamDivergence == 0.00022
    assert scanner.wavelength * 1000000000 == 1064  # has to be converted from m to nm
    assert scanner.visibility == 23.0
    assert scanner.numRays == 19  # for default beamSampleQuality of 3
    assert scanner.pulseLength_ns == 10.0
    assert list(scanner.getSupportedPulseFrequencies()) == [20000, 60000, 150000]
    assert (
        scanner.toString()
        == """Scanner: leica_als50-ii
Device[0]: leica_als50-ii
	Average Power: 4 W
	Beam Divergence: 0.22 mrad
	Wavelength: 1064 nm
	Visibility: 23 km
"""
    )


def test_detector(test_sim):
    """Test accessing detector settings with pyhelios"""
    path_to_survey = Path("data") / "test" / "als_hd_demo_tiff_min.xml"
    sim = test_sim(path_to_survey)
    scanner = sim.sim.getScanner()
    detector = scanner.getDetector()

    assert detector.accuracy == 0.05
    assert detector.rangeMin == 200
    assert detector.rangeMax == 1700

    scene_file = find_scene(path_to_survey)
    if os.path.isfile(scene_file):
        os.remove(scene_file)


@pytest.mark.parametrize(
    "export_to_file",
    [
        pytest.param(True, id="setExportToFile(True)"),
        pytest.param(False, id="setExportToFile(False)"),
    ],
)
def test_output(output_dir, export_to_file):
    """Validating the output of a survey started with pyhelios"""
    from pyhelios import SimulationBuilder

    survey_path = Path("data") / "test" / "als_hd_demo_tiff_min.xml"
    pyhelios.setDefaultRandomnessGeneratorSeed("43")
    simB = SimulationBuilder(
        surveyPath=str(survey_path.absolute()),
        assetsDir=[str(Path("assets"))],
        outputDir=str(output_dir),
    )
    simB.setFinalOutput(True)
    simB.setExportToFile(export_to_file)
    simB.setRebuildScene(True)
    simB.setCallbackFrequency(100)
    simB.setNumThreads(1)
    simB.setKDTJobs(1)
    simB.setFixedGpsTimeStart("2022-01-01 00:00:00")

    sim = simB.build()

    sim.start()
    output = sim.join()
    measurements_array, trajectory_array = pyhelios.outputToNumpy(output)

    if export_to_file:
        # check if output files exist
        dirname = find_playback_dir(survey_path, output_dir)
        assert (Path(dirname) / "leg000_points.xyz").exists()
        assert (Path(dirname) / "leg001_points.xyz").exists()
        assert (Path(dirname) / "leg002_points.xyz").exists()
        # check if output files contain the same data as the internal arrays
        leg0 = np.loadtxt(Path(dirname) / "leg000_points.xyz")
        leg1 = np.loadtxt(Path(dirname) / "leg001_points.xyz")
        leg2 = np.loadtxt(Path(dirname) / "leg002_points.xyz")
        measurements_from_file = np.vstack((leg0, leg1, leg2))
        # account for different (order of) fields in internal array and file
        assert np.allclose(
            measurements_array[:, (0, 1, 2, 9, 10, 12, 11, 13, 14, 15, 16)],
            measurements_from_file,
            atol=1e-6,
        )

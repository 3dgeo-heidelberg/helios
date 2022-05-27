#!/usr/bin/env python
# -- coding: utf-8 --

"""
Tests for the Python bindings
"""

import pytest
import numpy as np
from pathlib import Path
import sys
import os
import time
import xml.etree.ElementTree as ET
# import shutil


DELETE_FILES_AFTER = False
HELIOS_EXE = str(Path('run') / 'helios')
if sys.platform == "win32":
    HELIOS_EXE += ".exe"
WORKING_DIR = str(Path(__file__).parent.parent.absolute())
sys.path.append(WORKING_DIR)
import pyhelios


def find_scene(survey_file):
    scene_file = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[0]
    return scene_file.replace('.xml', '.scene')


@pytest.fixture(scope="session")
def test_sim():
    def create_test_sim(survey_path):
        # pyhelios.loggingSilent()
        from pyhelios import SimulationBuilder
        simB = SimulationBuilder(
            surveyPath=str(survey_path.absolute()),
            assetsDir=WORKING_DIR + os.sep + 'assets' + os.sep,
            outputDir=WORKING_DIR + os.sep + 'output' + os.sep,
        )
        simB.setLasOutput(True)
        simB.setRebuildScene(True)
        simB.setZipOutput(True)

        sim = simB.build()

        return sim

    return create_test_sim


def test_start_stop(test_sim):
    sim = test_sim(Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks.xml')
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
    sim = test_sim(Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks.xml')
    leg = sim.sim.getLeg(0)
    ss = leg.getScannerSettings()
    assert ss.hasTemplate()
    ss_templ = ss.getTemplate()
    ps = leg.getPlatformSettings()
    assert ps.hasTemplate()
    ps_templ = ps.getTemplate()

    assert ss_templ.id == 'scanner1'
    assert ss_templ.active is True
    assert ss_templ.pulseFreq == 300_000
    assert ss_templ.trajectoryTimeInterval == 0.01
    assert ss_templ.scanFreq == 200
    assert ss_templ.scanAngle * 180 / np.pi == 20
    assert ps_templ.id == 'platform1'
    assert ps_templ.movePerSec == 30


def test_survey_characteristics(test_sim):
    path_to_survey = Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks.xml'
    sim = test_sim(path_to_survey)
    assert Path(sim.sim.getSurveyPath()) == Path(WORKING_DIR) / path_to_survey
    survey = sim.sim.getSurvey()
    assert survey.name == 'toyblocks_als'
    assert survey.getLength() == 0.0
    survey.calculateLength()
    assert survey.getLength() == 400.0


def test_scene():
    pass


def test_create_survey():
    pyhelios.setDefaultRandomnessGeneratorSeed("7")
    test_survey_path = 'data/surveys/test_survey.xml'

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
        test_survey_path,
        'assets/',
        'output/'
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
        [100., -100.],
        [-100., -100.],
        [-100., -50.],
        [100., -50.],
        [100., 0.],
        [-100., 0.],
        [-100., 50.],
        [100., 50.],
        [100., 100.],
        [-100., 100.]]

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
    assert meas.shape == (10410, 17)
    assert traj.shape == (6670, 7)
    np.testing.assert_allclose(meas[100, :3], np.array([83.32, -66.43508, -0.07260715]))
    np.testing.assert_allclose(traj[0, :3], np.array([waypoints[0][0], waypoints[0][1], altitude]))

    # cleanup
    os.remove(test_survey_path)
    # shutil.rmtree(Path(output.outpath).parent)  # Fails with permission error (for the last trajectory file)


def test_material(test_sim):
    sim = test_sim(Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks.xml')
    scene = sim.sim.getScene()
    prim0 = scene.getPrimitive(0)  # get first primitive
    mat0 = prim0.getMaterial()
    assert mat0.name == 'None'
    assert mat0.isGround is True
    assert mat0.matFilePath == 'data/sceneparts/basic/groundplane/groundplane.mtl'
    assert mat0.reflectance == 50.0
    assert mat0.specularity == 0.0
    assert mat0.specularExponent == 0.0
    assert mat0.classification == 0
    assert np.round(mat0.kd0, 2) == 0.20


def test_scanner(test_sim):
    path_to_survey = Path('data') / 'test' / 'als_hd_demo_tiff_min.xml'
    sim = test_sim(path_to_survey)
    scanner = sim.sim.getScanner()
    assert scanner.deviceId == 'leica_als50-ii'
    assert scanner.averagePower == 4.0
    assert scanner.beamDivergence == 0.00022
    assert scanner.wavelength * 1000000000 == 1064  # has to be converted from m to nm
    assert scanner.visibility == 23.0
    assert scanner.numRays == 19  # for default beamSampleQuality of 3
    assert scanner.pulseLength_ns == 10.0
    assert list(scanner.getSupportedPulseFrequencies()) == [20000, 60000, 150000]
    assert scanner.toString() == 'Scanner: leica_als50-ii Power: 4.000000 W Divergence: 0.220000 mrad ' \
                                 'Wavelength: 1064 nm Visibility: 23.000000 km'


def test_detector(test_sim):
    path_to_survey = Path('data') / 'test' / 'als_hd_demo_tiff_min.xml'
    sim = test_sim(path_to_survey)
    scanner = sim.sim.getScanner()
    detector = scanner.getDetector()

    assert detector.accuracy == 0.05
    assert detector.rangeMin == 200
    assert detector.rangeMax == 1700

    scene_file = find_scene(path_to_survey)
    if os.path.isfile(scene_file):
        os.remove(scene_file)


def test_output():
    from pyhelios import SimulationBuilder
    survey_path = Path('data') / 'test' / 'als_hd_demo_tiff_min.xml'
    pyhelios.setDefaultRandomnessGeneratorSeed("43")
    simB = SimulationBuilder(
        surveyPath=str(survey_path.absolute()),
        assetsDir=WORKING_DIR + os.sep + 'assets' + os.sep,
        outputDir=WORKING_DIR + os.sep + 'output' + os.sep,
    )
    simB.setFinalOutput(True)
    simB.setRebuildScene(True)
    simB.setCallbackFrequency(100)
    simB.setNumThreads(1)
    simB.setKDTJobs(1)

    sim = simB.build()

    sim.start()
    output = sim.join()
    measurements_array, trajectory_array = pyhelios.outputToNumpy(output)
    np.testing.assert_allclose(measurements_array[0, :3], np.array([474500.3, 5473529.0, 106.0196]), rtol=0.000001)
    assert measurements_array.shape == (2433, 17)
    assert trajectory_array.shape == (9, 7)
    assert Path(output.outpath).parent.parent == Path(WORKING_DIR) / "output" / "als_hd_demo"

    # cleanup
    # shutil.rmtree(Path(output.outpath).parent)  # Fails with permission error (for the last trajectory file)

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


DELETE_FILES_AFTER = False
HELIOS_EXE = str(Path('run') / 'helios')
if sys.platform == "win32":
    HELIOS_EXE += ".exe"
WORKING_DIR = str(Path(__file__).parent.parent.absolute())


def find_scene(survey_file):
    scene_file = ET.parse(survey_file).find('survey').attrib['scene'].split('#')[0]
    return scene_file.replace('.xml', '.scene')


@pytest.fixture(scope="session")
def test_sim():
    def create_test_sim(survey_path):
        sys.path.append(WORKING_DIR)
        import pyhelios
        pyhelios.loggingSilent()
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

    assert ss_templ.id == "scanner1"
    assert ss_templ.active is True
    assert ss_templ.pulseFreq == 300_000
    assert ss_templ.trajectoryTimeInterval == 0.01
    assert ss_templ.scanFreq == 200
    assert ss_templ.scanAngle * 180 / np.pi == 20
    assert ps_templ.id == "platform1"
    assert ps_templ.movePerSec == 30


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

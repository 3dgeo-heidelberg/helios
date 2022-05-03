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

MAX_DIFFERENCE_BYTES = 1024
DELETE_FILES_AFTER = False
HELIOS_EXE = str(Path('run') / 'helios')
if sys.platform == "win32":
    HELIOS_EXE += ".exe"
WORKING_DIR = str(Path(__file__).parent.parent.absolute())


@pytest.fixture(scope="session")
def toyblocks_sim(survey_path=Path('data') / 'surveys' / 'toyblocks' / 'als_toyblocks.xml'):
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


def test_start_stop(toyblocks_sim):
    toyblocks_sim.start()
    assert toyblocks_sim.isStarted()
    time.sleep(0.05)
    toyblocks_sim.pause()
    assert toyblocks_sim.isPaused()
    toyblocks_sim.resume()
    assert toyblocks_sim.isRunning()
    while toyblocks_sim.isRunning():
        pass
    assert toyblocks_sim.isFinished()
    toyblocks_sim.stop()
    assert toyblocks_sim.isStopped()
    assert toyblocks_sim.isRunning() is False

    yield toyblocks_sim


def test_templates(toyblocks_sim):
    leg = toyblocks_sim.sim.getLeg(0)
    ss = leg.getScannerSettings()
    assert ss.hasTemplate()
    ss_templ = ss.getTemplate()
    ps = leg.getPlatformSettings()
    assert ps.hasTemplate()
    ps_templ = ps.getTemplate()

    assert ss_templ.id == "scanner1"
    assert ss_templ.active is True
    assert ss_templ.pulseFreq == 1_200_000
    assert ss_templ.trajectoryTimeInterval == 0.01
    assert ss_templ.scanFreq == 200
    assert ss_templ.scanAngle * 180 / np.pi == 20
    assert ps_templ.id == "platform1"
    assert ps_templ.movePerSec == 30

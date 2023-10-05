import pytest
import os, shutil
from test_demo_scenes import run_helios_executable, find_playback_dir
from pathlib import Path
import sys
import datetime, time
import hashlib

MAX_DIFFERENCE_BYTES = 1024
DELETE_FILES_AFTER = True
HELIOS_EXE = str(Path('run') / 'helios')
if sys.platform == "win32":
    HELIOS_EXE += ".exe"
WORKING_DIR = str(Path(__file__).parent.parent.absolute())


def sha256sum(filename):
    h  = hashlib.sha256()
    b  = bytearray(128*1024)
    mv = memoryview(b)
    with open(filename, 'rb', buffering=0) as f:
        for n in iter(lambda : f.readinto(mv), 0):
            h.update(mv[:n])
    return h.hexdigest()


def run_helios_pyhelios(survey_path: Path, options=None) -> Path:
    sys.path.append(WORKING_DIR)
    import pyhelios
    pyhelios.setDefaultRandomnessGeneratorSeed("43")
    from pyhelios import SimulationBuilder
    simB = SimulationBuilder(
        surveyPath=str(survey_path.absolute()),
        assetsDir=WORKING_DIR + os.sep + 'assets' + os.sep,
        outputDir=WORKING_DIR + os.sep + 'output' + os.sep,
    )
    simB.setLasOutput(False)
    simB.setRebuildScene(True)
    simB.setNumThreads(1)
    simB.setKDTJobs(1)
    simB.setFixedGpsTimeStart(options['gpsStartTime'])
    sim = simB.build()

    sim.start()
    output = sim.join()
    sim = None
    return find_playback_dir(survey_path)

def test_gpsStartTimeFlag_exe():
    now = datetime.datetime(year=1994, month=1, day=18, hour=1, minute=45, second=22)
    unixtime = datetime.datetime.timestamp(now)
    stringtime = now.strftime('%Y-%m-%d %H:%M:%S')
    # first, run 'as is':
    r1 = run_helios_executable(Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                                        options=['--gpsStartTime', ''])
    # run with posix ts:
    r2 = run_helios_executable(Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                                        options=['--gpsStartTime', f'{unixtime:.3f}'])
    # run with string ts:
    r3 = run_helios_executable(Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                                        options=['--gpsStartTime', stringtime])

    assert (r1 / 'leg000_points.xyz').exists()
    r1_sum = sha256sum(r1 / 'leg000_points.xyz')
    r2_sum = sha256sum(r2 / 'leg000_points.xyz')
    r3_sum = sha256sum(r3 / 'leg000_points.xyz')
    assert r2_sum == r3_sum
    assert r2_sum == 'e1daca137b066eca4bd29b62f3cb5ecde4bcc41650f2cdfc44110c87b97f23dc' or \
           r2_sum == 'e5ff320c318bd8cb87865ff8a3fd5d8835ca7a66736cc7469e59ad2c365a3a31'  # linux checksum
    assert r1_sum != r2_sum

    if DELETE_FILES_AFTER:
        shutil.rmtree(r1)
        shutil.rmtree(r2)
        shutil.rmtree(r3)


def test_gpsStartTimeFlag_pyh():
    now = datetime.datetime(year=1994, month=1, day=18, hour=1, minute=45, second=22)
    unixtime = datetime.datetime.timestamp(now)
    stringtime = now.strftime('%Y-%m-%d %H:%M:%S')
    # first, run 'as is':
    r1 = run_helios_pyhelios(Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                               options={'gpsStartTime': ''})
    # run with posix ts:
    r2 = run_helios_pyhelios(Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                               options={'gpsStartTime': f'{unixtime:.0f}'})
    # run with string ts:
    r3 = run_helios_pyhelios(Path(WORKING_DIR) / 'data' / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                               options={'gpsStartTime': stringtime})

    assert (r1 / 'leg000_points.xyz').exists()
    r1_sum = sha256sum(r1 / 'leg000_points.xyz')
    r2_sum = sha256sum(r2 / 'leg000_points.xyz')
    r3_sum = sha256sum(r3 / 'leg000_points.xyz')
    assert r2_sum == r3_sum
    assert r2_sum == 'e1daca137b066eca4bd29b62f3cb5ecde4bcc41650f2cdfc44110c87b97f23dc' or \
           r2_sum == 'e5ff320c318bd8cb87865ff8a3fd5d8835ca7a66736cc7469e59ad2c365a3a31'  # linux checksum
    assert r1_sum != r2_sum

    if DELETE_FILES_AFTER:
        try:
            shutil.rmtree(r1)
            shutil.rmtree(r2)
            shutil.rmtree(r3)
        except Exception as e:
            print(f"Error cleaning up: {e}")

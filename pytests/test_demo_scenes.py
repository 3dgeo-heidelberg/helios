import pytest
import os, shutil
import subprocess
from pathlib import Path
import sys

HELIOS_EXE = str(Path('build') / 'helios')
if sys.platform == "win32":
    HELIOS_EXE += ".exe"
WORKING_DIR = str(Path(__file__).parent.parent.absolute())

def find_playback_dir(survey_path):
    playback = Path(WORKING_DIR) / 'output' / 'Survey Playback'
    with open(Path(WORKING_DIR) / survey_path, 'r') as sf:
        for line in sf:
            if '<survey name' in line:
                survey_name = line.split('name="')[1].split('"')[0]
    if not (playback / survey_name).is_dir():
        raise Exception('Could not locate output directory')
    last_run_dir = sorted(list((playback / survey_name).glob('*')), key=lambda f: f.stat().st_ctime, reverse=True)[0]
    return last_run_dir / 'points'

def run_helios_executable(survey_path: Path, options=None) -> Path:
    if options is None:
        options = list()
    command = [HELIOS_EXE, str(survey_path)] + options
    print(command)
    p = subprocess.Popen(command, cwd=WORKING_DIR, shell=True)
    p.wait()
    assert p.returncode == 0
    return find_playback_dir(survey_path)

def run_helios_pyhelios(survey_path: Path, options=None) -> Path:
    sys.path.append(str((Path(WORKING_DIR) / 'build').absolute()))
    print(sys.path)
    import pyhelios
    sim = pyhelios.Simulation(
        str(survey_path.absolute()),
        WORKING_DIR + os.sep + 'assets' + os.sep,
        WORKING_DIR + os.sep + 'output' + os.sep,
        0,  # Num Threads
        1,  # LAS v1.4 output
        0,  # LAS v1.0 output
        0,  # ZIP output
    )
    # Load the survey file.
    sim.loadSurvey(
        1,  # Leg Noise Disabled FLAG
        1,  # Rebuild Scene FLAG
        0,  # Write Waveform FLAG
        0,  # Calculate Echowidth FLAG
        0,  # Full Wave Noise FLAG
        1  # Platform Noise Disabled FLAG
    )
    sim.start()
    output = sim.join()
    return find_playback_dir(survey_path)


def test_arbaro_tls_exe():
    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                                        options=['--rebuildScene', '--lasOutput', '--seed', '43', '-vt'])
    eval_arbaro_tls(dirname_exe)

def test_arbaro_tls_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml')
    eval_arbaro_tls(dirname_pyh)

def eval_arbaro_tls(dirname):
    assert (dirname / 'leg000_points.las').exists()
    assert abs((dirname / 'leg000_points.las').stat().st_size - 18_628_011) < 1_024
    assert (dirname / 'leg001_points.las').exists()
    assert abs((dirname / 'leg001_points.las').stat().st_size - 12_152_115) < 1_024
    with open(dirname / 'leg000_trajectory.txt', 'r') as f:
        line = f.readline()
        assert line.startswith('1.0000 25.5000 0.0000')
    # clean up
    shutil.rmtree(dirname)

def test_tiffloader_als_exe():
    dirname_exe = run_helios_executable(Path('data') / 'test' / 'als_hd_demo_tiff_min.xml',
                                        options=['--rebuildScene', '--lasOutput', '--seed', '43', '-vt'])
    eval_tiffloader_als(dirname_exe)

def test_tiffloader_als_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'test' / 'als_hd_demo_tiff_min.xml')
    eval_tiffloader_als(dirname_pyh)

def eval_tiffloader_als(dirname):
    assert (dirname / 'leg000_points.las').exists()
    assert abs((dirname / 'leg000_points.las').stat().st_size - 109_197) < 1_024
    assert (dirname / 'leg001_points.las').exists()
    assert abs((dirname / 'leg001_points.las').stat().st_size - 109_197) < 1_024
    with open(dirname / 'leg000_trajectory.txt', 'r') as f:
        line = f.readline()
        line = f.readline()
        assert line.startswith('474500.7510 5474500.0000 1500.0000')
    # clean up
    shutil.rmtree(dirname)

def test_detailedVoxels_uls_exe():
    dirname_exe = run_helios_executable(Path('data') / 'test' / 'uls_detailedVoxels_mode_comparison_min.xml',
                                        options=['--rebuildScene', '--lasOutput', '--seed', '43', '-vt'])
    eval_detailedVoxels_uls(dirname_exe)

def test_detailedVoxels_uls_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'test' / 'uls_detailedVoxels_mode_comparison_min.xml')
    eval_detailedVoxels_uls(dirname_pyh)

def eval_detailedVoxels_uls(dirname):
    assert (dirname / 'leg000_points.las').exists()
    assert abs((dirname / 'leg000_points.las').stat().st_size - 418_509) < 1_024 * 3  # for some reason, the exe result is smaller (by 2268 bytes)
    assert (dirname / 'leg000_trajectory.txt').exists()
    assert abs((dirname / 'leg000_trajectory.txt').stat().st_size - 1_197) < 1_024
    with open(dirname / 'leg000_trajectory.txt', 'r') as f:
        line = f.readline()
        line = f.readline()
        line = f.readline()
        line = f.readline()
        line = f.readline()
        line = f.readline()
        line = f.readline()
        assert line.startswith('-3.0000 -1.2741 50.0000')
    # clean up
    shutil.rmtree(dirname)

def test_xyzVoxels_tls_exe():
    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'voxels' / 'tls_sphere_xyzloader_rgb_normals.xml',
                                        options=['--rebuildScene', '--lasOutput', '--seed', '43', '-vt'])
    eval_xyzVoxels_tls(dirname_exe)

def test_xyzVoxels_tls_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'voxels' / 'tls_sphere_xyzloader_rgb_normals.xml')
    eval_xyzVoxels_tls(dirname_pyh)

def eval_xyzVoxels_tls(dirname):
    assert (dirname / 'leg000_points.las').exists()
    assert abs((dirname / 'leg000_points.las').stat().st_size - 16_937_811) < 1_024
    # clean up
    shutil.rmtree(dirname)

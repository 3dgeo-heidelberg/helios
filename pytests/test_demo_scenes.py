import os
import shutil
import subprocess
import sys
from pathlib import Path
import numpy as np
import pytest
import fnmatch

try:
    import laspy
except ImportError:
    pass

MAX_DIFFERENCE_BYTES = 1024
DELETE_FILES_AFTER = False
HELIOS_EXE = str(Path('run') / 'helios')
if sys.platform == "win32":
    HELIOS_EXE += ".exe"
WORKING_DIR = str(Path(__file__).parent.parent.absolute())


def find_playback_dir(survey_path):
    playback = Path(WORKING_DIR) / 'output'
    with open(Path(WORKING_DIR) / survey_path, 'r') as sf:
        for line in sf:
            if '<survey name' in line:
                survey_name = line.split('name="')[1].split('"')[0]
    if not (playback / survey_name).is_dir():
        raise FileNotFoundError('Could not locate output directory')
    last_run_dir = sorted(list((playback / survey_name).glob('*')), key=lambda f: f.stat().st_ctime, reverse=True)[0]
    return last_run_dir


def run_helios_executable(survey_path: Path, options=None) -> Path:
    if options is None:
        options = list()
    command = [HELIOS_EXE, str(survey_path)] + options + ['--rebuildScene',
                                                          '--seed', '43',
                                                          '-vt',
                                                          '-j', '1']
    print(command)
    # shell must be false for linux (but true for windows(?))
    p = subprocess.Popen(command, cwd=WORKING_DIR, shell=(sys.platform == "win32"))
    p.wait()
    assert p.returncode == 0
    return find_playback_dir(survey_path)


def run_helios_pyhelios(survey_path: Path, las_output: bool = True, zip_output: bool = False,
                        start_time: str = None, split_by_channel: bool = False, las10: bool = False) -> Path:
    sys.path.append(WORKING_DIR)
    import pyhelios
    pyhelios.setDefaultRandomnessGeneratorSeed("43")
    from pyhelios import SimulationBuilder
    simB = SimulationBuilder(
        surveyPath=str(survey_path.absolute()),
        assetsDir=WORKING_DIR + os.sep + 'assets' + os.sep,
        outputDir=WORKING_DIR + os.sep + 'output' + os.sep,
    )
    simB.setLasOutput(las_output)
    simB.setLas10(las10)
    simB.setRebuildScene(True)
    simB.setZipOutput(zip_output)
    simB.setSplitByChannel(split_by_channel)
    simB.setNumThreads(1)
    simB.setKDTJobs(1)
    if start_time:
        simB.setFixedGpsTimeStart(start_time)
    sim = simB.build()

    sim.start()
    sim.join()
    return find_playback_dir(survey_path)


def speed_from_traj(trajectory_file):

    def mode(arr):
        vals, counts = np.unique(arr, return_counts=True)
        mode_idx = np.argwhere(counts == np.max(counts))
        mode_val = vals[mode_idx].flatten().tolist()[0]

        # round to 4 decimal places
        return np.round(mode_val, 3)

    # load trajectory file
    traj_data = np.loadtxt(trajectory_file, delimiter=" ", usecols=(0, 1, 2, 3))

    # get distance between all points
    points = traj_data[:, :2]
    d = np.diff(points, axis=0)
    segdists = np.sqrt((d ** 2).sum(axis=1))
    # get most frequent distance (ignore smaller distances from speedup/slowdown)
    dist = mode(segdists)

    # get time diff. between points (should always be the same, but we're nevertheless computing the mode to be safe)
    times = traj_data[:, 3]
    dt = np.diff(times, axis=0)
    time_diff = mode(dt)

    speed = dist / time_diff

    return speed


@pytest.mark.exe
def test_arbaro_tls_exe():
    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                                        options=['--lasOutput',
                                                 '--gpsStartTime', '2022-01-01 00:00:00'])
    eval_arbaro_tls(dirname_exe)


@pytest.mark.pyh
def test_arbaro_tls_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'tls_arbaro_demo.xml',
                                      start_time='2022-01-01 00:00:00')
    eval_arbaro_tls(dirname_pyh)


def eval_arbaro_tls(dirname):
    assert (dirname / 'leg000_points.las').exists()
    assert abs((dirname / 'leg000_points.las').stat().st_size - 15_803_973) < MAX_DIFFERENCE_BYTES
    assert (dirname / 'leg001_points.las').exists()
    assert abs((dirname / 'leg001_points.las').stat().st_size - 10_149_957) < MAX_DIFFERENCE_BYTES
    with open(dirname / 'leg000_trajectory.txt', 'r') as f:
        line = f.readline()
        assert line.startswith('1.0000 25.5000 0.0000')
    # clean up
    if DELETE_FILES_AFTER:
        shutil.rmtree(dirname)


@pytest.mark.exe
def test_tiffloader_als_exe():
    dirname_exe = run_helios_executable(Path('data') / 'test' / 'als_hd_demo_tiff_min.xml',
                                        options=['--lasOutput',
                                                 '--gpsStartTime', '2022-01-01 00:00:00'])
    eval_tiffloader_als(dirname_exe)


@pytest.mark.pyh
def test_tiffloader_als_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'test' / 'als_hd_demo_tiff_min.xml',
                                      start_time='2022-01-01 00:00:00')
    eval_tiffloader_als(dirname_pyh)


def eval_tiffloader_als(dirname):
    assert (dirname / 'leg000_points.las').exists()
    assert abs((dirname / 'leg000_points.las').stat().st_size - 40_239) < MAX_DIFFERENCE_BYTES
    assert (dirname / 'leg001_points.las').exists()
    assert abs((dirname / 'leg001_points.las').stat().st_size - 62_757) < MAX_DIFFERENCE_BYTES
    with open(dirname / 'leg000_trajectory.txt', 'r') as f:
        next(f)
        line = f.readline()
        assert line.startswith('474500.7500 5474500.0000 1500.0000')
    # clean up
    if DELETE_FILES_AFTER:
        shutil.rmtree(dirname)


@pytest.mark.exe
def test_detailedVoxels_uls_exe():
    dirname_exe = run_helios_executable(Path('data') / 'test' / 'uls_detailedVoxels_mode_comparison_min.xml',
                                        options=['--lasOutput',
                                                 '--gpsStartTime', '2022-01-01 00:00:00'])
    eval_detailedVoxels_uls(dirname_exe)


@pytest.mark.pyh
def test_detailedVoxels_uls_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'test' / 'uls_detailedVoxels_mode_comparison_min.xml',
                                      start_time='2022-01-01 00:00:00')
    eval_detailedVoxels_uls(dirname_pyh)


def eval_detailedVoxels_uls(dirname):
    assert (dirname / 'leg000_points.las').exists()
    assert abs((dirname / 'leg000_points.las').stat().st_size - 420_291) < MAX_DIFFERENCE_BYTES
    assert (dirname / 'leg000_trajectory.txt').exists()
    assert abs((dirname / 'leg000_trajectory.txt').stat().st_size - 2_541) < MAX_DIFFERENCE_BYTES
    with open(dirname / 'leg000_trajectory.txt', 'r') as f:
        for _ in range(6):
            next(f)
        line = f.readline()
        assert line.startswith('-3.0000 -2.1000 50.0000')
    # clean up
    if DELETE_FILES_AFTER:
        shutil.rmtree(dirname)


@pytest.mark.exe
def test_xyzVoxels_tls_exe():
    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'voxels' / 'tls_sphere_xyzloader_normals.xml',
                                        options=['--lasOutput'])
    eval_xyzVoxels_tls(dirname_exe)


@pytest.mark.pyh
def test_xyzVoxels_tls_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'voxels' / 'tls_sphere_xyzloader_normals.xml')
    eval_xyzVoxels_tls(dirname_pyh)


def eval_xyzVoxels_tls(dirname):
    assert (dirname / 'leg000_points.las').exists()
    assert abs((dirname / 'leg000_points.las').stat().st_size - 14_115_393) < MAX_DIFFERENCE_BYTES
    # clean up
    if DELETE_FILES_AFTER:
        shutil.rmtree(dirname)


@pytest.mark.exe
def test_interpolated_traj_exe():
    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'als_interpolated_trajectory.xml',
                                        options=['--lasOutput',
                                                 '--zipOutput',
                                                 '--gpsStartTime', '2022-01-01 00:00:00'])
    eval_interpolated_traj(dirname_exe)


@pytest.mark.pyh
def test_interpolated_traj_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'als_interpolated_trajectory.xml',
                                      zip_output=True,
                                      start_time='2022-01-01 00:00:00')
    eval_interpolated_traj(dirname_pyh)


def eval_interpolated_traj(dirname):
    assert (dirname / 'leg000_points.laz').exists()
    assert (dirname / 'leg000_trajectory.txt').exists()
    assert abs((dirname / 'leg000_points.laz').stat().st_size - 862_526) < MAX_DIFFERENCE_BYTES
    with open(dirname / 'leg000_trajectory.txt', 'r') as f:
        for _ in range(3):
            next(f)
        line = f.readline()
        assert line.startswith('13.4766 1.7424 400.0000')
    # clean up
    if DELETE_FILES_AFTER:
        shutil.rmtree(dirname)


@pytest.mark.exe
def test_quadcopter_exe():
    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'toyblocks' / 'uls_toyblocks_survey_scene_combo.xml',
                                        options=['--lasOutput',
                                                 '--zipOutput'])
    eval_quadcopter(dirname_exe)


@pytest.mark.pyh
def test_quadcopter_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'toyblocks' / 'uls_toyblocks_survey_scene_combo.xml',
                                      zip_output=True)
    eval_quadcopter(dirname_pyh)


def eval_quadcopter(dirname):
    assert (dirname / 'leg000_points.laz').exists()
    assert (dirname / 'leg000_trajectory.txt').exists()
    assert abs((dirname / 'leg000_points.laz').stat().st_size - 1_980_849) < MAX_DIFFERENCE_BYTES
    assert abs((dirname / 'leg002_points.laz').stat().st_size - 2_166_606) < MAX_DIFFERENCE_BYTES
    assert abs((dirname / 'leg004_points.laz').stat().st_size - 3_841_155) < MAX_DIFFERENCE_BYTES
    assert speed_from_traj(dirname / 'leg000_trajectory.txt') == pytest.approx(10.0, 0.001)
    assert speed_from_traj(dirname / 'leg002_trajectory.txt') == pytest.approx(7.0, 0.001)
    assert speed_from_traj(dirname / 'leg004_trajectory.txt') == pytest.approx(4.0, 0.001)
    with open(dirname / 'leg000_trajectory.txt', 'r') as f:
        for _ in range(3):
            next(f)
        line = f.readline()
        assert line.startswith('-69.9983 -60.0000 80.0002')
    # clean up
    if DELETE_FILES_AFTER:
        shutil.rmtree(dirname)


@pytest.mark.pyh
def test_als_multichannel_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'light_als_toyblocks_multiscanner.xml',
                                      zip_output=True)
    eval_als_multichannel(dirname_pyh)


@pytest.mark.pyh
def test_als_multichannel_split_pyh():
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'light_als_toyblocks_multiscanner.xml',
                                      zip_output=True, split_by_channel=True)
    eval_als_multichannel_split(dirname_pyh)


@pytest.mark.exe
def test_als_multichannel():
    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'light_als_toyblocks_multiscanner.xml',
                                        options=['--lasOutput',
                                                 '--zipOutput'])
    eval_als_multichannel(dirname_exe)


@pytest.mark.exe
def test_als_multichannel_split():
    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'light_als_toyblocks_multiscanner.xml',
                                        options=['--lasOutput',
                                                 '--zipOutput',
                                                 '--splitByChannel'])
    eval_als_multichannel_split(dirname_exe)


def eval_als_multichannel(dirname):
    assert len(fnmatch.filter(os.listdir(dirname), '*.laz')) == 2


def eval_als_multichannel_split(dirname):
    # 2 legs, Livox Mid-100 has 3 channels, so we expect 6 point clouds
    assert len(fnmatch.filter(os.listdir(dirname), '*.laz')) == 6


@pytest.mark.skipif("laspy" not in sys.modules,
                    reason="requires the laspy library")
@pytest.mark.pyh
@pytest.mark.parametrize(
    "zip_flag, las10_flag",
    [
        pytest.param(False, False, id="LAS v1.4"),
        pytest.param(False, True, id="LAS v1.0"),
        pytest.param(True, False, id="LAZ v1.4"),
        pytest.param(True, True, id="LAZ v1.0"),
    ]
)
def test_las_pyh(zip_flag: bool, las10_flag: bool):
    """"""
    dirname_pyh = run_helios_pyhelios(Path('data') / 'surveys' / 'demo' / 'light_als_toyblocks_multiscanner.xml',
                                      las_output=True, zip_output=zip_flag, las10=las10_flag)
    las_version = "1.0" if las10_flag else "1.4"
    eval_las(dirname_pyh, las_version)


@pytest.mark.skipif("laspy" not in sys.modules,
                    reason="requires the laspy library")
@pytest.mark.exe
@pytest.mark.parametrize(
    "zip_flag, las10_flag",
    [
        pytest.param(False, False, id="LAS v1.4"),
        pytest.param(False, True, id="LAS v1.0"),
        pytest.param(True, False, id="LAZ v1.4"),
        pytest.param(True, True, id="LAZ v1.0"),
    ]
)
def test_las_exe(zip_flag: bool, las10_flag: bool):
    options = ["--lasOutput"]
    if zip_flag:
        options.append("--zipOutput")
    if las10_flag:
        options.append("--las10")

    dirname_exe = run_helios_executable(Path('data') / 'surveys' / 'demo' / 'light_als_toyblocks_multiscanner.xml',
                                        options=options)
    las_version = "1.0" if las10_flag else "1.4"
    eval_las(dirname_exe, las_version)


def eval_las(dirname, las_version):
    path = Path(dirname) / fnmatch.filter(os.listdir(dirname), '*.la?')[0]
    las = laspy.read(path)
    dimensions = [d.name for d in las.point_format.dimensions]
    expected_dimensions = [
        "X", "Y", "Z",
        "intensity",
        "classification",
        "gps_time",
        "echo_width",
        "fullwaveIndex",
        "hitObjectId",
        "heliosAmplitude"
    ]
    for dim in expected_dimensions:
        assert dim in dimensions
    assert las.header.version == las_version

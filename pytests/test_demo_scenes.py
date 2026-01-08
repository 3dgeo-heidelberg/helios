from pyhelios.__main__ import helios_exec

import os
import sys
from pathlib import Path
import numpy as np
import pytest
import fnmatch
import xml.etree.ElementTree as ET

import pyhelios
from . import pcloud_utils as pcu

try:
    import laspy
except ImportError:
    pass


def find_playback_dir(survey_path, playback):
    tree = ET.parse(Path(survey_path))
    root = tree.getroot()
    survey_name = root.find("survey").attrib["name"]
    if not (playback / survey_name).is_dir():
        raise FileNotFoundError(
            f"Could not locate output directory: {playback / survey_name}"
        )
    last_run_dir = sorted(
        list((playback / survey_name).glob("*")),
        key=lambda f: f.stat().st_ctime,
        reverse=True,
    )[0]
    return last_run_dir


def run_helios_executable(survey_path: Path, output_dir: Path, options=None) -> Path:
    if options is None:
        options = list()

    helios_exec(
        [str(survey_path)]
        + options
        + [
            "--output",
            str(output_dir),
            "--rebuildScene",
            "--seed",
            "43",
            "-vt",
            "-j",
            "1",
        ]
    )

    return find_playback_dir(survey_path, output_dir)


def run_helios_pyhelios(
    survey_path: Path,
    output: Path,
    las_output: bool = True,
    zip_output: bool = False,
    start_time: str = None,
    split_by_channel: bool = False,
    las10: bool = False,
) -> Path:
    pyhelios.setDefaultRandomnessGeneratorSeed("43")
    simB = pyhelios.SimulationBuilder(
        surveyPath=str(survey_path.absolute()),
        assetsDir=[str(Path("assets"))],
        outputDir=str(output),
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
    return find_playback_dir(survey_path, output)


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
    segdists = np.sqrt((d**2).sum(axis=1))
    # get most frequent distance (ignore smaller distances from speedup/slowdown)
    dist = mode(segdists)

    # get time diff. between points (should always be the same, but we're nevertheless computing the mode to be safe)
    times = traj_data[:, 3]
    dt = np.diff(times, axis=0)
    time_diff = mode(dt)

    speed = dist / time_diff

    return speed


@pytest.mark.exe
def test_arbaro_tls_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "surveys" / "demo" / "tls_arbaro_demo.xml",
        output_dir,
        options=[
            "--lasOutput",
            "--gpsStartTime",
            "2022-01-01 00:00:00",
        ],
    )
    eval_arbaro_tls(regression_data, dirname_exe)


@pytest.mark.pyh
def test_arbaro_tls_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "surveys" / "demo" / "tls_arbaro_demo.xml",
        output=output_dir,
        start_time="2022-01-01 00:00:00",
    )
    eval_arbaro_tls(regression_data, dirname_pyh)


def eval_arbaro_tls(regression_data, dirname):
    assert (dirname / "leg000_points.las").exists()
    assert (dirname / "leg001_points.las").exists()

    # ToDo: same approach for trajectory?
    with open(dirname / "leg000_trajectory.txt", "r") as f:
        line = f.readline()
        assert line.startswith("1.0000 25.5000 0.0000")

    if regression_data:
        pcloud0_ref = pcu.PointCloud.from_las_file(
            regression_data / "arbaro_tls_leg000_points.las"
        )
        pcloud0 = pcu.PointCloud.from_las_file(dirname / "leg000_points.las")
        pcloud0.assert_equals(pcloud0_ref)

        pcloud1_ref = pcu.PointCloud.from_las_file(
            regression_data / "arbaro_tls_leg001_points.las"
        )
        pcloud1 = pcu.PointCloud.from_las_file(dirname / "leg001_points.las")
        pcloud1.assert_equals(pcloud1_ref)


@pytest.mark.exe
def test_tiffloader_als_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "test" / "als_hd_demo_tiff_min.xml",
        output_dir,
        options=[
            "--lasOutput",
            "--gpsStartTime",
            "2022-01-01 00:00:00",
        ],
    )
    eval_tiffloader_als(regression_data, dirname_exe)


@pytest.mark.pyh
def test_tiffloader_als_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "test" / "als_hd_demo_tiff_min.xml",
        output=output_dir,
        start_time="2022-01-01 00:00:00",
    )
    eval_tiffloader_als(regression_data, dirname_pyh)


def eval_tiffloader_als(regression_data, dirname):
    assert (dirname / "leg000_points.las").exists()
    assert (dirname / "leg001_points.las").exists()
    assert (dirname / "leg002_points.las").exists()

    if regression_data:
        pcloud0_ref = pcu.PointCloud.from_las_file(
            regression_data / "tiffloader_als_leg000_points.las"
        )
        pcloud0 = pcu.PointCloud.from_las_file(dirname / "leg000_points.las")
        pcloud0.assert_equals(pcloud0_ref)
        pcloud1_ref = pcu.PointCloud.from_las_file(
            regression_data / "tiffloader_als_leg001_points.las"
        )
        pcloud1 = pcu.PointCloud.from_las_file(dirname / "leg001_points.las")
        pcloud1.assert_equals(pcloud1_ref)
        pcloud2_ref = pcu.PointCloud.from_las_file(
            regression_data / "tiffloader_als_leg002_points.las"
        )
        pcloud2 = pcu.PointCloud.from_las_file(dirname / "leg002_points.las")
        pcloud2.assert_equals(pcloud2_ref)

    with open(dirname / "leg000_trajectory.txt", "r") as f:
        next(f)
        line = f.readline()
        assert line.startswith("474500.7500 5474500.0000 1500.0000")


@pytest.mark.exe
@pytest.mark.xfail(
    condition=sys.platform == "win32",
    reason="This test behaves flaky (+-1 points difference) and we currently do not understand why",
)
def test_detailedVoxels_uls_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "test" / "uls_detailedVoxels_mode_comparison_min.xml",
        output_dir,
        options=["--lasOutput", "--gpsStartTime", "2022-01-01 00:00:00"],
    )
    eval_detailedVoxels_uls(regression_data, dirname_exe)


@pytest.mark.pyh
@pytest.mark.xfail(
    condition=sys.platform == "win32",
    reason="This test behaves flaky (+-1 points difference) and we currently do not understand why",
)
def test_detailedVoxels_uls_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "test" / "uls_detailedVoxels_mode_comparison_min.xml",
        output=output_dir,
        start_time="2022-01-01 00:00:00",
    )
    eval_detailedVoxels_uls(regression_data, dirname_pyh)


def eval_detailedVoxels_uls(regression_data, dirname):
    assert (dirname / "leg000_points.las").exists()
    with open(dirname / "leg000_trajectory.txt", "r") as f:
        for _ in range(6):
            next(f)
        line = f.readline()
        assert line.startswith("-3.0000 -2.1000 50.0000")

    if regression_data:
        pcloud0_ref = pcu.PointCloud.from_las_file(
            regression_data / "detailedVoxels_uls_leg000_points.las"
        )
        pcloud0 = pcu.PointCloud.from_las_file(dirname / "leg000_points.las")
        pcloud0.assert_equals(pcloud0_ref)


@pytest.mark.exe
def test_xyzVoxels_tls_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "surveys" / "voxels" / "tls_sphere_xyzloader_normals.xml",
        output_dir,
        options=["--lasOutput", "--gpsStartTime", "2022-01-01 00:00:00"],
    )
    eval_xyzVoxels_tls(regression_data, dirname_exe)


@pytest.mark.pyh
def test_xyzVoxels_tls_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "surveys" / "voxels" / "tls_sphere_xyzloader_normals.xml",
        output=output_dir,
        start_time="2022-01-01 00:00:00",
    )
    eval_xyzVoxels_tls(regression_data, dirname_pyh)


def eval_xyzVoxels_tls(regression_data, dirname):
    assert (dirname / "leg000_points.las").exists()

    if regression_data:
        pcloud_ref = pcu.PointCloud.from_las_file(
            regression_data / "xyzVoxels_tls_leg000_points.las"
        )
        pcloud = pcu.PointCloud.from_las_file(dirname / "leg000_points.las")
        pcloud.assert_equals(pcloud_ref)


@pytest.mark.exe
def test_interpolated_traj_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "surveys" / "demo" / "als_interpolated_trajectory.xml",
        output_dir,
        options=["--lasOutput", "--zipOutput", "--gpsStartTime", "2022-01-01 00:00:00"],
    )
    eval_interpolated_traj(regression_data, dirname_exe)


@pytest.mark.pyh
def test_interpolated_traj_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "surveys" / "demo" / "als_interpolated_trajectory.xml",
        output=output_dir,
        zip_output=True,
        start_time="2022-01-01 00:00:00",
    )
    eval_interpolated_traj(regression_data, dirname_pyh)


def eval_interpolated_traj(regression_data, dirname):
    assert (dirname / "leg000_points.laz").exists()
    assert (dirname / "leg001_points.laz").exists()
    assert (dirname / "leg002_points.laz").exists()
    assert (dirname / "leg003_points.laz").exists()

    with open(dirname / "leg000_trajectory.txt", "r") as f:
        for _ in range(3):
            next(f)
        line = f.readline()
        assert line.startswith("13.4766 1.7424 400.0000")

    if regression_data:
        pcloud0_ref = pcu.PointCloud.from_las_file(
            regression_data / "interpolated_traj_leg000_points.laz", fnames=["gps_time"]
        )
        pcloud0 = pcu.PointCloud.from_las_file(dirname / "leg000_points.laz", fnames=["gps_time"])
        pcloud0.assert_equals(pcloud0_ref)
        pcloud1_ref = pcu.PointCloud.from_las_file(
            regression_data / "interpolated_traj_leg001_points.laz"
        )
        pcloud1 = pcu.PointCloud.from_las_file(dirname / "leg001_points.laz")
        pcloud2_ref = pcu.PointCloud.from_las_file(
            regression_data / "interpolated_traj_leg002_points.laz"
        )
        pcloud2 = pcu.PointCloud.from_las_file(dirname / "leg002_points.laz")
        pcloud2.assert_equals(pcloud2_ref)
        pcloud1.assert_equals(pcloud1_ref)
        pcloud3_ref = pcu.PointCloud.from_las_file(
            regression_data / "interpolated_traj_leg003_points.laz"
        )
        pcloud3 = pcu.PointCloud.from_las_file(dirname / "leg003_points.laz")
        pcloud3.assert_equals(pcloud3_ref)


@pytest.mark.skipif("laspy" not in sys.modules, reason="requires the laspy library")
@pytest.mark.exe
def test_quadcopter_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "surveys" / "toyblocks" / "uls_toyblocks_survey_scene_combo.xml",
        output_dir,
        options=[
            "--lasOutput",
            "--zipOutput",
            "--gpsStartTime",
            "2022-01-01 00:00:00",
        ],
    )
    eval_quadcopter(regression_data, dirname_exe)


@pytest.mark.skipif("laspy" not in sys.modules, reason="requires the laspy library")
@pytest.mark.pyh
def test_quadcopter_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "surveys" / "toyblocks" / "uls_toyblocks_survey_scene_combo.xml",
        output=output_dir,
        zip_output=True,
        start_time="2022-01-01 00:00:00",
    )
    eval_quadcopter(regression_data, dirname_pyh)


def eval_quadcopter(regression_data, dirname):
    assert (dirname / "leg000_points.laz").exists()
    assert (dirname / "leg001_points.laz").exists()
    assert (dirname / "leg002_points.laz").exists()
    assert (dirname / "leg004_points.laz").exists()
    assert speed_from_traj(dirname / "leg000_trajectory.txt") == pytest.approx(
        10.0, 0.001
    )
    assert speed_from_traj(dirname / "leg002_trajectory.txt") == pytest.approx(
        7.0, 0.001
    )
    assert speed_from_traj(dirname / "leg004_trajectory.txt") == pytest.approx(
        4.0, 0.001
    )
    with open(dirname / "leg000_trajectory.txt", "r") as f:
        for _ in range(3):
            next(f)
        line = f.readline()
        assert line.startswith("-69.9983 -60.0000 80.0002")

    if regression_data:
        pcloud0_ref = pcu.PointCloud.from_las_file(
            regression_data / "quadcopter_leg000_points.laz"
        )
        pcloud0 = pcu.PointCloud.from_las_file(dirname / "leg000_points.laz")
        pcloud0.assert_equals(pcloud0_ref)
        pcloud1_ref = pcu.PointCloud.from_las_file(
            regression_data / "quadcopter_leg001_points.laz"
        )
        pcloud1 = pcu.PointCloud.from_las_file(dirname / "leg001_points.laz")
        pcloud1.assert_equals(pcloud1_ref)
        pcloud2_ref = pcu.PointCloud.from_las_file(
            regression_data / "quadcopter_leg002_points.laz"
        )
        pcloud2 = pcu.PointCloud.from_las_file(dirname / "leg002_points.laz")
        pcloud2.assert_equals(pcloud2_ref)
        pcloud3_ref = pcu.PointCloud.from_las_file(
            regression_data / "quadcopter_leg004_points.laz"
        )
        pcloud3 = pcu.PointCloud.from_las_file(dirname / "leg004_points.laz")
        pcloud3.assert_equals(pcloud3_ref)


@pytest.mark.pyh
def test_als_multichannel_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "surveys" / "demo" / "light_als_toyblocks_multiscanner.xml",
        output=output_dir,
        zip_output=True,
        start_time="2022-01-01 00:00:00",
    )
    eval_als_multichannel(regression_data, dirname_pyh)


@pytest.mark.pyh
def test_als_multichannel_split_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "surveys" / "demo" / "light_als_toyblocks_multiscanner.xml",
        output=output_dir,
        zip_output=True,
        split_by_channel=True,
        start_time="2022-01-01 00:00:00",
    )
    eval_als_multichannel_split(regression_data, dirname_pyh)


@pytest.mark.exe
def test_als_multichannel_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "surveys" / "demo" / "light_als_toyblocks_multiscanner.xml",
        output_dir,
        options=[
            "--lasOutput",
            "--zipOutput",
            "--gpsStartTime",
            "2022-01-01 00:00:00",
        ],
    )
    eval_als_multichannel(regression_data, dirname_exe)


@pytest.mark.exe
def test_als_multichannel_split_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "surveys" / "demo" / "light_als_toyblocks_multiscanner.xml",
        output_dir,
        options=[
            "--lasOutput",
            "--zipOutput",
            "--gpsStartTime",
            "2022-01-01 00:00:00",
            "--splitByChannel",
        ],
    )
    eval_als_multichannel_split(regression_data, dirname_exe)


def eval_als_multichannel(regression_data, dirname):
    assert len(fnmatch.filter(os.listdir(dirname), "*.laz")) == 2
    assert (dirname / "leg000_points.laz").exists()
    assert (dirname / "leg002_points.laz").exists()

    if regression_data:
        pcloud0_ref = pcu.PointCloud.from_las_file(
            regression_data / "als_multichannel_leg000_points.laz"
        )
        pcloud0 = pcu.PointCloud.from_las_file(dirname / "leg000_points.laz")
        pcloud0.assert_equals(pcloud0_ref)
        pcloud1_ref = pcu.PointCloud.from_las_file(
            regression_data / "als_multichannel_leg002_points.laz"
        )
        pcloud1 = pcu.PointCloud.from_las_file(dirname / "leg002_points.laz")
        pcloud1.assert_equals(pcloud1_ref)


def eval_als_multichannel_split(regression_data, dirname):
    # 2 legs, Livox Mid-100 has 3 channels, so we expect 6 point clouds
    assert len(fnmatch.filter(os.listdir(dirname), "*.laz")) == 6
    assert (dirname / "leg000_points_dev0.laz").exists()
    assert (dirname / "leg000_points_dev1.laz").exists()
    assert (dirname / "leg000_points_dev2.laz").exists()
    assert (dirname / "leg002_points_dev0.laz").exists()
    assert (dirname / "leg002_points_dev1.laz").exists()
    assert (dirname / "leg002_points_dev2.laz").exists()

    if regression_data:
        pcloud0_0_ref = pcu.PointCloud.from_las_file(
            regression_data / "als_multichannel_split_leg000_points_dev0.laz"
        )
        pcloud0_0 = pcu.PointCloud.from_las_file(dirname / "leg000_points_dev0.laz")
        pcloud0_0.assert_equals(pcloud0_0_ref)
        pcloud0_1_ref = pcu.PointCloud.from_las_file(
            regression_data / "als_multichannel_split_leg000_points_dev1.laz"
        )
        pcloud0_1 = pcu.PointCloud.from_las_file(dirname / "leg000_points_dev1.laz")
        pcloud0_1.assert_equals(pcloud0_1_ref)
        pcloud0_2_ref = pcu.PointCloud.from_las_file(
            regression_data / "als_multichannel_split_leg000_points_dev2.laz"
        )
        pcloud0_2 = pcu.PointCloud.from_las_file(dirname / "leg000_points_dev2.laz")
        pcloud0_2.assert_equals(pcloud0_2_ref)
        pcloud1_0_ref = pcu.PointCloud.from_las_file(
            regression_data / "als_multichannel_split_leg002_points_dev0.laz"
        )
        pcloud1_0 = pcu.PointCloud.from_las_file(dirname / "leg002_points_dev0.laz")
        pcloud1_0.assert_equals(pcloud1_0_ref)
        pcloud1_1_ref = pcu.PointCloud.from_las_file(
            regression_data / "als_multichannel_split_leg002_points_dev1.laz"
        )
        pcloud1_1 = pcu.PointCloud.from_las_file(dirname / "leg002_points_dev1.laz")
        pcloud1_1.assert_equals(pcloud1_1_ref)
        pcloud1_2_ref = pcu.PointCloud.from_las_file(
            regression_data / "als_multichannel_split_leg002_points_dev2.laz"
        )
        pcloud1_2 = pcu.PointCloud.from_las_file(dirname / "leg002_points_dev2.laz")
        pcloud1_2.assert_equals(pcloud1_2_ref)


@pytest.mark.skipif("laspy" not in sys.modules, reason="requires the laspy library")
@pytest.mark.pyh
@pytest.mark.parametrize(
    "zip_flag, las10_flag",
    [
        pytest.param(False, False, id="LAS v1.4"),
        pytest.param(False, True, id="LAS v1.0"),
        pytest.param(True, False, id="LAZ v1.4"),
        pytest.param(True, True, id="LAZ v1.0"),
    ],
)
def test_las_pyh(zip_flag: bool, las10_flag: bool, output_dir):
    """"""
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "surveys" / "demo" / "light_als_toyblocks_multiscanner.xml",
        output=output_dir,
        las_output=True,
        zip_output=zip_flag,
        las10=las10_flag,
    )
    las_version = "1.0" if las10_flag else "1.4"
    eval_las(dirname_pyh, las_version)


@pytest.mark.skipif("laspy" not in sys.modules, reason="requires the laspy library")
@pytest.mark.exe
@pytest.mark.parametrize(
    "zip_flag, las10_flag",
    [
        pytest.param(False, False, id="LAS v1.4"),
        pytest.param(False, True, id="LAS v1.0"),
        pytest.param(True, False, id="LAZ v1.4"),
        pytest.param(True, True, id="LAZ v1.0"),
    ],
)
def test_las_exe(zip_flag: bool, las10_flag: bool, output_dir):
    options = ["--lasOutput"]
    if zip_flag:
        options.append("--zipOutput")
    if las10_flag:
        options.append("--las10")

    dirname_exe = run_helios_executable(
        Path("data") / "surveys" / "demo" / "light_als_toyblocks_multiscanner.xml",
        output_dir,
        options=options,
    )
    las_version = "1.0" if las10_flag else "1.4"
    eval_las(dirname_exe, las_version)


def eval_las(dirname, las_version, check_empty=False):
    path = Path(dirname) / fnmatch.filter(os.listdir(dirname), "*.la?")[0]
    las = laspy.read(path)
    dimensions = [d.name for d in las.point_format.dimensions]
    expected_dimensions = [
        "X",
        "Y",
        "Z",
        "intensity",
        "classification",
        "gps_time",
        "echo_width",
        "fullwaveIndex",
        "hitObjectId",
        "heliosAmplitude",
    ]
    for dim in expected_dimensions:
        assert dim in dimensions
    assert las.header.version == las_version
    if check_empty is True:
        assert las.header.point_count > 0


@pytest.mark.skipif("laspy" not in sys.modules, reason="requires the laspy library")
@pytest.mark.pyh
def test_strip_id_pyh(output_dir):
    """"""
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "test" / "als_hd_height_above_ground_stripid_light.xml",
        output=output_dir,
        las_output=True,
        zip_output=True,
    )
    las_version = "1.4"
    eval_las(dirname_pyh, las_version, check_empty=False)


@pytest.mark.skipif("laspy" not in sys.modules, reason="requires the laspy library")
@pytest.mark.exe
def test_strip_id_exe(output_dir):

    dirname_exe = run_helios_executable(
        Path("data") / "test" / "als_hd_height_above_ground_stripid_light.xml",
        output_dir,
        options=["--lasOutput", "--zipOutput"],
    )
    las_version = "1.4"
    eval_las(dirname_exe, las_version, check_empty=True)


@pytest.mark.pyh
def test_dyn_pyh(regression_data, output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "surveys" / "dyn" / "tls_dyn_cube.xml",
        output=output_dir,
        las_output=True,
        zip_output=True,
        start_time="2022-01-01 00:00:00",
    )
    eval_dyn(regression_data, dirname_pyh)


@pytest.mark.exe
def test_dyn_exe(regression_data, output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "surveys" / "dyn" / "tls_dyn_cube.xml",
        output_dir,
        options=["--lasOutput", "--zipOutput", "--gpsStartTime", "2022-01-01 00:00:00"],
    )
    eval_dyn(regression_data, dirname_exe)


def eval_dyn(regression_data, dirname):
    assert (dirname / "leg000_points.laz").exists()

    if regression_data:
        pcloud0_ref = pcu.PointCloud.from_las_file(
            regression_data / "dyn_leg000_points.laz"
        )
        pcloud0 = pcu.PointCloud.from_las_file(dirname / "leg000_points.laz")
        pcloud0.assert_equals(pcloud0_ref)


@pytest.mark.pyh
def test_maxduration_pyh(output_dir):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "test" / "test_max_duration.xml",
        output=output_dir,
        las_output=False,
    )
    eval_maxduration(dirname_pyh)


@pytest.mark.exe
def test_maxduration_exe(output_dir):
    dirname_exe = run_helios_executable(
        Path("data") / "test" / "test_max_duration.xml", output_dir
    )
    eval_maxduration(dirname_exe)


def eval_maxduration(dirname):
    # max time is set to 0.1 seconds
    delta_t_expected = 0.1
    gps_time = np.loadtxt(dirname / "leg000_points.xyz", usecols=10)
    delta_t = gps_time.max() - gps_time.min()
    assert delta_t <= delta_t_expected + 1e-4


# @pytest.mark.pyh
# def test_no_infinite_run_pyh(output_dir):
#    with pytest.raises(Exception):
#        dirname_pyh = run_helios_pyhelios(
#            Path("data") / "test" / "test_infinite_run.xml",
#            output=output_dir
#        )
# The HeliosException leads to a fatal Python error that is not caught by pytest ...


@pytest.mark.exe
def test_no_infinite_run_exe(output_dir, capfd):
    dirname_exe = run_helios_executable(
        Path("data") / "test" / "test_infinite_run.xml", output_dir
    )
    captured = capfd.readouterr()
    eval_no_infinite_run(dirname_exe, captured)


def eval_no_infinite_run(dirname, captured):
    path = dirname / "leg000_points.xyz"
    # assert that the file is empty
    assert os.path.getsize(path) == 0
    # Not working as expected unless we globally set --capture=tee-sys in pyproject.toml
    # see https://github.com/pytest-dev/pytest/issues/5997
    # assert "ERROR: No platform movement, scanner head rotation or maximum duration set." in captured.out

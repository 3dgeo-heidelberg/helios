from pyhelios.__main__ import helios_exec

import os
import sys
from pathlib import Path
import numpy as np
import pytest
import time
import xml.etree.ElementTree as ET

import pyhelios
from scipy.spatial import KDTree


def find_playback_dir(survey_path, playback):
    tree = ET.parse(Path(survey_path))
    root = tree.getroot()
    survey_name = root.find("survey").attrib["name"]
    if not (playback / survey_name).is_dir():
        raise FileNotFoundError(
            f"Could not locate output directory: {playback / survey_name}"
        )
    dirs = list((playback / survey_name).glob("*"))
    sorted_dirs = sorted(dirs, key=lambda f: f.stat().st_ctime, reverse=True)
    last_run_dir = sorted_dirs[0]

    # check if there is another with the same time stamp
    for d in sorted_dirs[1:]:
        if d.stat().st_ctime == last_run_dir.stat().st_ctime:
            # compare suffixes
            d_suffix = d.name.split("_")[-1]
            last_run_suffix = last_run_dir.name.split("_")[-1]
            if len(d_suffix) > 1 and int(d_suffix) > int(last_run_suffix):
                last_run_dir = d

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


@pytest.mark.pyh
@pytest.mark.parametrize(
    "survey_xml, survey_xml_nowarmup, check_time",
    [
        # not checking time for rotating since due to the limited effective scan angles, the time difference between simulated points does not match the elapsed simulated time 
        pytest.param("test_warmup_rotating.xml", "test_warmup_rotating_nowarmup.xml", 0),
        pytest.param("test_warmup_oscillating.xml", "test_warmup_oscillating_nowarmup.xml", 1),
        pytest.param("test_warmup_conic.xml", "test_warmup_conic_nowarmup.xml", 1),
        pytest.param("test_warmup_risley.xml", "test_warmup_risley_nowarmup.xml", 1)
    ],
)
def test_optics_warmup_phase_pyh(output_dir, survey_xml, survey_xml_nowarmup, check_time):
    dirname_pyh = run_helios_pyhelios(
        Path("data") / "test" / survey_xml,
        output=output_dir,
        las_output=False,
        start_time="2022-01-01 00:00:00"
    )
    dirname_pyh_nowarmup = run_helios_pyhelios(
        Path("data") / "test" / survey_xml_nowarmup,
        output=output_dir,
        las_output=False,
        start_time="2022-01-01 00:00:00"
    )
    eval_optics_warmup_phase(dirname_pyh, dirname_pyh_nowarmup, check_time)


@pytest.mark.exe
@pytest.mark.parametrize(
    "survey_xml, survey_xml_nowarmup, check_time",
    [
        # not checking time for rotating since due to the limited effective scan angles, the time difference between simulated points does not match the elapsed simulated time 
        pytest.param("test_warmup_rotating.xml", "test_warmup_rotating_nowarmup.xml", 0),
        pytest.param("test_warmup_oscillating.xml", "test_warmup_oscillating_nowarmup.xml", 1),
        pytest.param("test_warmup_conic.xml", "test_warmup_conic_nowarmup.xml", 1),
        pytest.param("test_warmup_risley.xml", "test_warmup_risley_nowarmup.xml", 1)
    ],
)
def test_optics_warmup_phase_exe(output_dir, survey_xml, survey_xml_nowarmup, check_time):
    dirname_exe = run_helios_executable(
        Path("data") / "test" / survey_xml, output_dir,
        options=["--gpsStartTime", "2022-01-01 00:00:00"]
    )
    dirname_exe_nowarmup = run_helios_executable(
        Path("data") / "test" / survey_xml_nowarmup, output_dir,
        options=["--gpsStartTime", "2022-01-01 00:00:00"]
    )
    eval_optics_warmup_phase(dirname_exe, dirname_exe_nowarmup, check_time)


def eval_optics_warmup_phase(dirname, dirname_nowarmup, check_time):
    assert dirname != dirname_nowarmup, "Output directories should be different for warmup and no warmup runs"
    # read simulated point clouds and their GPS time
    points_warmup = np.loadtxt(dirname / "leg000_points.xyz")
    points_nowarmup = np.loadtxt(dirname_nowarmup / "leg000_points.xyz")
    gps_time_warmup = points_warmup[:, 10]
    gps_time_nowarmup = points_nowarmup[:, 10]

    # criterion 1: the one with warmup should be a subset of the one without warmup
    points_nowarmup_clipped = points_nowarmup[gps_time_nowarmup >= gps_time_nowarmup.min() + 0.002]
    # Allow difference in number of points by up to 1
    assert abs(len(points_warmup) - len(points_nowarmup_clipped)) <= 1, \
        f"Point count difference too large: {len(points_warmup)} vs {len(points_nowarmup_clipped)}"
    
    # Use KDTree to find nearest neighbors (use the larger set as the tree)
    if len(points_warmup) <= len(points_nowarmup_clipped):
        tree = KDTree(points_nowarmup_clipped[:, :3])
        distances, _ = tree.query(points_warmup[:, :3])
    else:
        tree = KDTree(points_warmup[:, :3])
        distances, _ = tree.query(points_nowarmup_clipped[:, :3])
    
    # Make sure that 90 percent of points are within tolerance
    tol = 0.05
    assert np.sum(distances <= tol) >= 0.9 * min(len(points_warmup), len(points_nowarmup_clipped)), \
        f"Too many points differ by more than {tol} meters: {np.sum(distances > tol)} out of {len(distances)}"

    # criterion 2: Both should start at the same GPS time
    assert np.isclose(gps_time_warmup.min(), gps_time_nowarmup.min())
    
    # criterion 3: delta t should be 0.01 seconds for warmup and 0.012 seconds for no warmup
    if check_time:
        delta_t_warmup = (gps_time_warmup.max() - gps_time_warmup.min())
        delta_t_nowarmup = (gps_time_nowarmup.max() - gps_time_nowarmup.min())
        # max_duration is not as expected?
        assert np.isclose(delta_t_warmup, 0.01, atol=1e-4)
        assert np.isclose(delta_t_nowarmup, 0.012, atol=1e-4)

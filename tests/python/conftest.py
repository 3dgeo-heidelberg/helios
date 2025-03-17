from pathlib import Path

from helios.platforms import tripod as tripod_platform, sr22
from helios.scanner import (
    leica_als50,
    riegl_vq_1560i,
    riegl_vz_400,
    vlp16,
)
from helios.scene import ScenePart, StaticScene
from helios.settings import (
    ExecutionSettings,
    OutputSettings,
    set_execution_settings,
    set_output_settings,
)
from helios.survey import Survey
from helios.utils import add_asset_directory, set_rng_seed

import math
import os
import pathlib
import pooch
import pytest


# Regression data archives to download
TEST_DATA_ARCHIVE = "https://github.com/3dgeo-heidelberg/helios-test-data/releases/download/2025-02-05/data.tar.gz"
TEST_DATA_CHECKSUM = "581b9f13ab3dcaf0422a8ab6069ba4534db276c75c098da23dd8ed264a5e7cee"


@pytest.fixture
def regression_data(request):
    """A fixture that ensures the existence of regression data

    Returns the Path to where that data is located after (cached) download
    from GitHub.
    """

    if not request.config.getoption("--regression-tests"):
        return None

    # Define the cache location
    cache = pooch.os_cache("helios")

    # Trigger the download
    pooch.retrieve(
        TEST_DATA_ARCHIVE,
        TEST_DATA_CHECKSUM,
        path=cache,
        downloader=pooch.HTTPDownloader(timeout=(3, None)),
        processor=pooch.Untar(extract_dir="."),
    )

    return cache


@pytest.fixture(scope="session")
def persisting_output_dir(request):
    if request.config.getoption("--keep-output"):
        return pathlib.Path(os.getcwd()) / "pytest-output"
    else:
        return request.getfixturevalue("tmp_path_factory").mktemp("output")


@pytest.fixture(autouse=True)
def rng_seed():
    """Reset the RNG before each test"""

    set_rng_seed(42)


@pytest.fixture
def reset_global_state():
    """Reset global state after a test alters it"""

    yield

    set_execution_settings(ExecutionSettings())
    set_output_settings(OutputSettings())


@pytest.fixture
def single_tls_scanner():
    return riegl_vz_400()


@pytest.fixture
def single_als_scanner():
    return leica_als50()


@pytest.fixture
def multi_tls_scanner():
    return vlp16()


@pytest.fixture
def multi_als_scanner():
    return riegl_vq_1560i()


# A few more generalized fixture names to be used
# if the more detailed specs do not matter
tls_scanner = single_tls_scanner
als_scanner = single_als_scanner
scanner = tls_scanner


@pytest.fixture
def tripod():
    return tripod_platform()


@pytest.fixture
def airplane():
    return sr22()


@pytest.fixture
def box_f():
    return lambda: ScenePart.from_obj("data/sceneparts/basic/box/box100.obj")


@pytest.fixture
def box(box_f):
    return box_f()


@pytest.fixture
def scene(box):
    return StaticScene(scene_parts=[box])


@pytest.fixture
def tls_survey(tls_scanner, tripod, scene):
    survey = Survey(scanner=tls_scanner, platform=tripod, scene=scene)
    survey.add_leg(
        x=0,
        y=0,
        z=0,
        pulse_frequency=2000,
        head_rotation="10 deg/s",
        rotation_start_angle="0 deg",
        rotation_stop_angle="10 deg",
    )
    return survey


survey = tls_survey


@pytest.fixture()
def assetdir(tmp_path):
    add_asset_directory(tmp_path)
    tmp_path = tmp_path / "root"
    tmp_path.mkdir()

    a = tmp_path / "a"
    b = tmp_path / "b" / "bb"
    c = tmp_path / "c"
    a.mkdir()
    b.mkdir(parents=True)
    c.mkdir()

    a1 = a / "some.obj"
    a2 = a / "second.obj"
    a3 = a / "notobj.smth"
    bb1 = b / "other.obj"
    c1 = c / "notobj.smt"
    a1.touch()
    a2.touch()
    a3.touch()
    bb1.touch()
    c1.touch()

    return tmp_path

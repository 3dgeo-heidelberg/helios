from helios.platform import tripod as tripod_platform, sr22
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
from helios.util import set_rng_seed

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


@pytest.fixture(autouse=True)
def reset_global_state():
    """Reset global state after a test alters it"""

    # NB: This fixture being autouse=True is a stop-gap measure until we
    #     implemented clone behaviour for all model objects, so that we
    #     avoid accidentally editing the global state.

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
def box():
    return ScenePart.from_xml("data/scenes/demo/box_scene.xml", id="0")


@pytest.fixture
def scene():
    return StaticScene.from_xml("data/scenes/demo/box_scene.xml")


@pytest.fixture
def tls_survey(tls_scanner, tripod, scene):
    def torad(x):
        return (x / 180.0) * math.pi

    survey = Survey(scanner=tls_scanner, platform=tripod, scene=scene)
    survey.add_leg(
        x=0,
        y=0,
        z=0,
        pulse_frequency=2000,
        head_rotation=torad(10),
        rotation_start_angle=torad(0),
        rotation_stop_angle=torad(10),
    )
    return survey


survey = tls_survey


def pytest_addoption(parser):
    # Add an option to also run slow tests which are omitted by default
    parser.addoption(
        "--slow", action="store_true", default=False, help="run slow tests"
    )

    # Add an option for running regression tests. This is useful to e.g.
    # restrict the regression testing bit to the platform on which we have
    # regression data available.
    parser.addoption(
        "--regression-tests",
        action="store_true",
        default=False,
        help="run regression tests",
    )

    # Add an option to keep the output of the demo regression tests in
    # the pytest-output folder
    parser.addoption(
        "--keep-output",
        action="store_true",
        default=False,
        help="keep output of demo tests",
    )


def pytest_runtest_setup(item):
    if "slow" in item.keywords and not (
        item.config.getoption("--slow") or item.config.getoption("--regression-tests")
    ):
        pytest.skip("need --slow option to run")

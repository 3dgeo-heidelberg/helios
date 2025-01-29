import os
import pathlib
import pooch
import pytest
import shutil


TEST_DATA_ARCHIVE = "https://github.com/3dgeo-heidelberg/helios-test-data/releases/download/2025-01-29/data.tar.gz"
TEST_DATA_CHECKSUM = "d15873ddc06d6dd15d9db31277feefa8064082788c69a80dc403525f2e07dbe5"


@pytest.fixture
def regression_data(request):
    """ A fixture that ensures the existence of regression data

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
def output_dir(request):
    dir = pathlib.Path(os.getcwd()) / "pytest-output"

    yield dir

    if request.config.getoption("--delete-output"):
        shutil.rmtree(dir)


def pytest_addoption(parser):
    parser.addoption(
        "--regression-tests",
        action="store_true",
        default=False,
        help="run regression tests",
    )

    parser.addoption(
        "--delete-output",
        action="store_true",
        default=False,
        help="run regression tests",
    )

import pooch
import pytest


TEST_DATA_ARCHIVE = "https://github.com/dokempf/helios-test-data/releases/download/2024-08-29/data.tar.gz"
TEST_DATA_CHECKSUM = "df4490f41cd5f9e17fd794429ea7b2fa1f0ad58848b5df44199d24c820cb324b"


@pytest.fixture
def regression_data():
    """ A fixture that ensures the existence of regression data

    Returns the Path to where that data is located after (cached) download
    from GitHub.
    """
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

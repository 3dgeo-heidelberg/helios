from helios.util import *

from pathlib import Path

import pytest


def test_get_asset_directories():
    dirs = get_asset_directories()

    assert len(dirs) == 3
    for dir in dirs:
        assert isinstance(dir, Path)
        assert dir.exists()


def test_find_file_unknown():
    with pytest.raises(FileNotFoundError):
        find_file("unknown_file", fatal=True)

    assert find_file("unknown_file", fatal=False) is None


def test_find_file_absolute():
    assert find_file(__file__) == Path(__file__)


def test_find_file_relative():
    found = find_file("__init__.py")
    assert isinstance(found, Path)
    assert found.exists()


def test_custom_asset_directory(tmp_path):
    add_asset_directory(tmp_path)

    dirs = get_asset_directories()
    assert len(dirs) == 4
    assert dirs[0] == tmp_path

    file = tmp_path / "test_file"
    file.touch()

    assert find_file("test_file") == file

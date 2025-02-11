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


def test_combine_empty():
    # Passing no parameters yields correct result

    assert combine_parameters() == [{}]


def test_combine_zip():
    params = combine_parameters(a=[1, 2], b=[3, 4], groups=[["a", "b"]])
    assert len(params) == 2
    assert params == [{"a": 1, "b": 3}, {"a": 2, "b": 4}]


def test_combine_groups():
    params = combine_parameters(a=[1, 2], b=[3, 4])
    assert len(params) == 4
    assert {"a": 1, "b": 3} in params
    assert {"a": 1, "b": 4} in params
    assert {"a": 2, "b": 3} in params
    assert {"a": 2, "b": 4} in params


def test_combine_both_modes():
    params = combine_parameters(
        a=[1, 2], b=[3, 4], c=[5, 6], groups=[["a", "b"], ["c"]]
    )
    assert len(params) == 4
    assert {"a": 1, "b": 3, "c": 5} in params
    assert {"a": 2, "b": 4, "c": 5} in params
    assert {"a": 1, "b": 3, "c": 6} in params
    assert {"a": 2, "b": 4, "c": 6} in params


def test_combine_interpolation():
    params = combine_parameters(a=[1, 2], b="{a}{a}")
    assert len(params) == 2
    assert {"a": 1, "b": "11"} in params
    assert {"a": 2, "b": "22"} in params


def test_ungrouped_parameter():
    params = combine_parameters(a=[1, 2], b=3)
    assert len(params) == 2
    assert {"a": 1, "b": 3} in params
    assert {"a": 2, "b": 3} in params


def test_ungrouped_list():
    params = combine_parameters(a=[1, 2], b=[3, 4], groups=[["a"]])
    assert len(params) == 2
    assert {"a": 1, "b": [3, 4]} in params
    assert {"a": 2, "b": [3, 4]} in params


def test_combine_size_mismatch():
    with pytest.raises(ValueError):
        combine_parameters(a=[1, 2], b=[3, 4, 5], groups=[["a", "b"]])


def test_combine_uniterable_type():
    with pytest.raises(ValueError):
        combine_parameters(a=1, b=[3, 4], groups=[["a", "b"]])


def test_string_not_considered_iterable():
    params = combine_parameters(a="test", b=[1])
    assert len(params) == 1
    assert params[0] == {"a": "test", "b": 1}


def test_combine_tuple_okay():
    params = combine_parameters(a=(1, 2), b=[3, 4], groups=[["a", "b"]])
    assert len(params) == 2
    assert {"a": 1, "b": 3} in params
    assert {"a": 2, "b": 4} in params

    # Also works with groups=None
    params = combine_parameters(a=(1, 2), b=(3, 4))
    assert len(params) == 4
    assert {"a": 1, "b": 3} in params
    assert {"a": 1, "b": 4} in params
    assert {"a": 2, "b": 3} in params
    assert {"a": 2, "b": 4} in params


def test_set_rng_seed():
    set_rng_seed(43)
    set_rng_seed(datetime.now(timezone.utc))

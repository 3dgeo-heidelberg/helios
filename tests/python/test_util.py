from helios.utils import *
from helios.utils import _prepare_trajectory_array

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


def test_find_files(assetdir):
    found = find_files("root/b/bb/other.obj")
    assert len(found) == 1

    found = find_files("root/b/*/other.obj")
    assert len(found) == 1

    found = find_files("root/a/*.obj")
    assert len(found) == 2

    found = find_files("root/**/*.obj")
    assert len(found) == 3

    found = find_files("root/*/*.obj")
    assert len(found) == 2

    found = find_files(assetdir / "*/*.obj")
    assert len(found) == 2


def test_classonlymethod():
    class SomeClass:
        @classonlymethod
        def my_method(cls):
            return "Hello"

    class SubClass:
        @classonlymethod
        def reading_from_smth(cls):
            return SubClass()

    assert SomeClass.my_method() == "Hello"
    with pytest.raises(TypeError):
        s = SomeClass()
        s.my_method()

    instance_obj = SubClass.reading_from_smth()
    with pytest.raises(TypeError):
        instance_obj.reading_from_smth()


def test_prepare_trajectory_array_wrong_arr():
    required_fields = ("t", "roll", "pitch", "yaw", "x", "y", "z")
    test_arr1 = np.zeros(
        2,
        dtype={
            "names": ("t", "x", "y", "z", "roll", "pitch", "yaw"),
            "formats": ("f8", "i4", "f8", "f8", "i4", "f8", "f8"),
        },
    )

    test_arr2 = np.ones(
        2,
        dtype={
            "names": ("t", "roll", "pitch", "yaw", "x", "y", "z"),
            "formats": ("f8", "f8", "f8", "f8", "f8", "f8", "f8"),
        },
    )

    test_arr3 = np.zeros(
        2,
        dtype={
            "names": ("t", "x", "y", "z", "roll", "pitch", "yaw"),
            "formats": ("f8", "f8", "f8", "f8", "f8", "f8", "f8"),
        },
    )
    test_arr3 = test_arr3[["t", "yaw", "x", "z", "roll", "pitch", "y"]]

    res_test_arr1 = _prepare_trajectory_array(test_arr1)
    res_test_arr2 = _prepare_trajectory_array(test_arr2)
    res_test_arr3 = _prepare_trajectory_array(test_arr3)

    assert res_test_arr1.dtype.names == required_fields
    assert all(res_test_arr1.dtype[f] == np.dtype("f8") for f in required_fields)
    assert res_test_arr2.dtype.names == required_fields
    assert all(res_test_arr2.dtype[f] == np.dtype("f8") for f in required_fields)
    assert res_test_arr3.dtype.names == required_fields
    assert all(res_test_arr3.dtype[f] == np.dtype("f8") for f in required_fields)


def test_prepare_trajectory_array_wrong_dtype():
    with pytest.raises(ValueError):
        _prepare_trajectory_array(np.zeros(2, dtype=[("t", "f8"), ("b", "f8")]))

    with pytest.raises(ValueError):
        _prepare_trajectory_array(
            np.zeros(
                1,
                dtype={
                    "names": ("t", "x", "y", "z", "roll", "pitch"),
                    "formats": ("f8", "f8", "f8", "f8", "f8", "f8"),
                },
            )
        )

    with pytest.raises(ValueError):
        _prepare_trajectory_array(
            np.zeros(
                1,
                dtype={
                    "names": ("t", "x", "y", "z", "roll", "pitch", "yaw", "extra"),
                    "formats": ("f8", "f8", "f8", "f8", "f8", "f8", "f8", "f8"),
                },
            )
        )

    with pytest.raises(ValueError):
        _prepare_trajectory_array(
            np.zeros(
                1,
                dtype={
                    "names": ("t", "x", "y", "z", "rokl", "pitch", "yaw"),
                    "formats": ("f8", "f8", "f8", "f8", "f8", "f8", "f8"),
                },
            )
        )

    with pytest.raises(ValueError):
        _prepare_trajectory_array(np.zeros(10))

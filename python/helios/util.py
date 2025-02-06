import importlib_resources as resources
import numpy as np
import os

from collections.abc import Iterable
from pathlib import Path
from pydantic import validate_call
from typing import Union


# The list of user provided directories that will be searched for assets.
_custom_asset_directories = []

# The list of built-in directories that will be searched for assets.
_builtin_asset_directories = [
    Path(os.getcwd()),
    resources.files("helios"),
    resources.files("helios") / "data",
]


def add_asset_directory(directory: Path) -> None:
    """
    Add a directory to the list of directories that will be searched for assets.

    :param directory: The directory to add.
    :type directory: Path
    """

    directory = Path(directory)

    if directory not in _custom_asset_directories:
        _custom_asset_directories.append(directory)


def get_asset_directories() -> list[Path]:
    """
    Get the list of directories that will be searched for assets.

    :return: The list of directories.
    :rtype: list[Path]
    """

    return _custom_asset_directories + _builtin_asset_directories


def find_file(filename: str, fatal: bool = True) -> Union[Path, None]:
    """
    Find a file in the list of directories that have been added as asset directories
    or in some default search locations.

    :param filename: The name of the file to find.
    :type filename: str
    :param fatal: Whether to raise an exception if the file is not found.
    :type fatal: bool
    :return: The path to the file, or None if the file was not found.
    :rtype: Path
    """

    # Check if the given filename is an absolute path.
    if Path(filename).is_absolute():
        file_path = Path(filename)

        # If it exists, return it.
        if file_path.exists():
            return file_path

    # Iterate all the given asset directories
    for directory in get_asset_directories():
        file_path = directory / filename

        if file_path.exists():
            return file_path

    if fatal:
        raise FileNotFoundError(
            f"Could not find file '{filename}' in any of the given asset directories."
        )

    return None


@validate_call
def combine_parameters(groups: Union[None, list[list[str]]] = None, **parameters):
    """
    Combine parameters for parameter studies by building combinations of the given parameters.
    This function allows for the combination of parameters, especially useful for parameter studies
    where different combinations of parameters need to be evaluated. It supports grouping of parameters
    and ensures that all parameters are expanded together (using Python's zip function), where as
    across groups, the cartesian product is built.

    :param groups: A list of groups where each group is a list of parameter names that should be combined together.
    :type groups: Union[None, list[list[str]]], optional
    :param parameters: Keyword arguments representing the parameters and their possible values. Each parameter can be
    :type parameters: dict
    :return: A list of dictionaries where each dictionary represents a unique combination of the parameters.
    :rtype: list[dict]
    :raises ValueError: If any parameter within a group is not iterable or if the lengths of the iterables within a group varies
    """

    # Define default for groups: Each parameter that was passed an iterable
    # of possible values forms its own group.
    if groups is None:
        groups = [[key] for key, value in parameters.items() if isinstance(value, list)]

    # Ensure that all parameters within a group are lists of the same length
    for group in groups:
        for key in group:
            if isinstance(parameters[key], str) or not isinstance(
                parameters[key], Iterable
            ):
                raise ValueError("All parameters within a group must be iterable.")
        if len(set(len(parameters[key]) for key in group)) > 1:
            raise ValueError("All parameters within a group must have the same length.")

    # Isolate the parameters that are not part of any group
    ungrouped = {
        key: value for key, value in parameters.items() if key not in sum(groups, [])
    }

    # Incrementally build the result dictionary
    result = [ungrouped]

    # Iterate over all groups to add their respective parameters
    for group in groups:
        # Build a dictionary of all parameters within this group
        group_result = []

        # We zip across the list of values of the contained parameters
        for values in zip(*[parameters[key] for key in group]):
            group_result.append({key: value for key, value in zip(group, values)})

        # Update the result by building the cartesian product with the
        # result for the current group
        result = [{**a, **b} for a in result for b in group_result]

    # Use Python string formatting on any string values, allowing complex
    # replacement syntax at very low implementaion cost
    for entry in result:
        for key, value in entry.items():
            if isinstance(value, str):
                entry[key] = value.format(**entry)

    return result


meas_dtype = np.dtype(
    [
        ("dev_id", "U50"),
        ("dev_idx", "u8"),
        ("hit_object_id", "U50"),
        ("position", "3f8"),
        ("beam_direction", "3f8"),
        ("beam_origin", "3f8"),
        ("distance", "f8"),
        ("intensity", "f8"),
        ("echo_width", "f8"),
        ("return_number", "i4"),
        ("pulse_return_number", "i4"),
        ("fullwave_index", "i4"),
        ("classification", "i4"),
        ("gps_time", "f8"),
    ]
)

traj_dtype = np.dtype(
    [
        ("gps_time", "f8"),
        ("position", "3f8"),
        ("roll", "f8"),
        ("pitch", "f8"),
        ("yaw", "f8"),
    ]
)

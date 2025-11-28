from collections.abc import Iterable
from datetime import datetime, timezone
from pathlib import Path
from pydantic import validate_call
from typing import Union, Sequence, TypeVar, List, TYPE_CHECKING
from numpydantic import NDArray, Shape


import importlib_resources as resources
import numpy as np
import os

if TYPE_CHECKING:
    # only for static type checkers, never at runtime
    from helios.survey import Survey
    from helios.scene import StaticScene
    from helios.settings import ExecutionSettings

import _helios

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


def find_file(filename: Union[Path, str], fatal: bool = True) -> Union[Path, None]:
    """
    Find a file in the list of directories that have been added as asset directories
    or in some default search locations.

    :param filename: The name of the file to find.
    :type filename: Union[Path, str]
    :param fatal: Whether to raise an exception if the file is not found.
    :type fatal: bool
    :return: The path to the file, or None if the file was not found.
    :rtype: Path
    """

    # Convert the filename to a Path object if it is a string.
    filename = Path(filename).expanduser()

    # Check if the given filename is an absolute path.
    if filename.is_absolute():
        # If it exists, return it.
        if filename.exists():
            return filename

    # Iterate all the given asset directories
    for directory in get_asset_directories():
        file_path = directory / filename

        if file_path.exists():
            return file_path

    if fatal:
        m = f"Could not find file '{filename}' in any of the given asset directories."
        if "*" in str(filename):
            m += " For resolving wildcards, use 'find_files' instead of 'find_file'!"
        raise FileNotFoundError(m)

    return None


def find_files(filename: Union[Path, str], fatal: bool = True) -> list[Path]:
    """
    Find files in the list of directories that have been added as asset directories
    or in some default search locations. Supports wildcards in the search pattern.

    :param filename: The name of the file to find.
    :type filename: Union[Path, str]
    :param fatal: Whether to raise an exception if no file is found.
    :type fatal: bool
    :return: A list of found filepaths.
    :rtype: list[Path]
    """

    filename = Path(filename).expanduser()
    assets = get_asset_directories()
    files = []

    if filename.is_absolute():
        if filename.exists():
            return [filename]
        else:
            # resolve wildcards in absolute paths
            parts = filename.parts
            assets = [Path(parts[0])]
            filename = Path().joinpath(*parts[1:])

    for asset in assets:
        files.extend(asset.glob(str(filename)))

    if len(files) == 0 and fatal:
        raise FileNotFoundError(
            f"Could not find file '{filename}' in any of the given asset directories."
        )
    return files


@validate_call
def set_rng_seed(seed: Union[int, datetime] = datetime.now(timezone.utc)) -> None:
    """Set the seed of the random number generator.

    :param seed: The seed to set.
        Can be an integer or a datetime object.
    :type seed: Union[int, datetime]
    """

    # The backend takes the seed as a string. We keep this for compatibility
    # of our regression tests, but ask the user to provide an integer. Only
    # integers are allowed in those strings anyway.
    _helios.default_rand_generator_seed(str(seed))


def is_real_iterable(value):
    return isinstance(value, Iterable) and not isinstance(value, str)


@validate_call
def combine_parameters(groups: Union[None, list[list[str]]] = None, **parameters):
    """Combine parameter spaces for parameter studies.

    This function allows to define lists of possible values for different parameters
    and to combine them in complex ways to generate a list of parameter dicionaries.
    Arbitrary parameters can be passed as keyword arguments. Parameter values may be
    lists to define a set of possible values for that parameter, but they don't have
    to.

    There is two ways of combining two parameter ranges: Either the cartesian product
    of possibilities can be generated or the two ranges can be zipped. The function
    allows both methods and arbirtrary combinations there of. The combination behaviour
    is controlled by the groups parameter, which accepts a list of lists that define
    groups that should be zipped together.

    Parameter values of type string may contain Python string formatting syntax. This
    way, they can reference other parameters in the resulting parameter dictionary
    and interpolate their values.

    :param groups:
        The list of groups that should be zipped instead of generating the cartesian
        product. Each group is a list of parameter names. Passing None will result in
        each parameter forming its own group and cartesian products being generated.
        This is the default behaviour.
    :type groups: Union[None, list[list[str]]]
    :param parameters:
        The actual parameters given as keyword arguments. Values can either be single
        values or lists of possible values.
    :type parameters: dict
    :return:
        A list of dictionaries where each dictionary represents a unique combination
        of the parameters.
    :rtype: list[dict]
    :raises ValueError:
        If any parameter within a group is not iterable or if the lengths of the
        iterables within a group varies.
    """

    # Define default for groups: Each parameter that was passed an iterable
    # of possible values forms its own group.
    if groups is None:
        groups = [[key] for key, value in parameters.items() if is_real_iterable(value)]

    # Ensure that all parameters within a group are lists of the same length
    for group in groups:
        for key in group:
            if not is_real_iterable(parameters[key]):
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


@validate_call
def detect_separator(file_path: Path) -> str:
    """Detect the separator used in an XYZ file."""
    possible_separators = [" ", ",", "\t", ";"]
    if not file_path.exists() or not file_path.is_file():
        raise ValueError(f"File not found: {file_path}")

    separator_counts = {sep: 0 for sep in possible_separators}
    with file_path.open("r", encoding="utf-8") as f:
        for line in f:
            stripped_line = line.strip()

            if (
                stripped_line
                and not stripped_line.startswith("//")
                and not stripped_line.startswith("#")
            ):
                return next(
                    (sep for sep in possible_separators if sep in stripped_line), " "
                )

    raise ValueError(f"Could not detect separator in file: {file_path}")


def is_finalized(obj) -> bool:
    """Return True if the Scene was finalized."""
    return getattr(obj, "_is_finalized", False)


def is_binary_loaded(obj) -> bool:
    """Return True if the Scene was constructed via from_binary."""
    return getattr(obj, "_is_loaded_from_binary", False)


def is_xml_loaded(obj) -> bool:
    """Return True if the object was constructed via from_xml or _from_cpp."""
    return getattr(obj, "_is_loaded_from_xml", False)


def apply_scene_shift(
    survey: "Survey", execution_settings: "ExecutionSettings"
) -> None:
    """
    If this survey was not loaded from XML, apply `make_scene_shift` once.
    Subsequent calls are no-ops.
    """

    if getattr(survey, "_scene_shift_done", False):
        return

    _helios.make_scene_shift(
        survey._cpp_object,
    )
    setattr(survey, "_scene_shift_done", True)


meas_dtype = np.dtype(
    [
        ("channel_id", "u8"),
        ("hit_object_id", "i4"),
        ("position", "3f8"),
        ("beam_direction", "3f8"),
        ("beam_origin", "3f8"),
        ("distance", "f8"),
        ("intensity", "f8"),
        ("echo_width", "f8"),
        ("return_number", "i4"),
        ("number_of_returns", "i4"),
        ("fullwave_index", "i4"),
        ("classification", "i4"),
        ("gps_time", "f8"),
        ("point_source_id", "u2"),
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

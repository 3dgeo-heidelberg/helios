import importlib_resources as resources
import os

from pathlib import Path


# The list of user provided directories that will be searched for assets.
_custom_asset_directories = []

# The list of built-in directories that will be searched for assets.
_builtin_asset_directories = [
    Path(os.getcwd()),
    resources.files("helios"),
    resources.files("helios") / "data",
]


def add_asset_directory(directory: Path | str) -> None:
    """
    Add a directory to the list of directories that will be searched for assets.

    :param directory: The directory to add.
    :type directory: Path | str
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


def find_file(filename: str, fatal: bool) -> Path | None:
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

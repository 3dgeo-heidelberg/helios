from helios.validation import Model, Property, ThreadCount, UpdateableMixin

from enum import IntEnum
from pathlib import Path
from pydantic import Field
from typing import Annotated, Union


class ParallelizationStrategy(IntEnum):
    CHUNK = 0
    WAREHOUSE = 1


class LogVerbosity(IntEnum):
    SILENT = 0b000000
    QUIET = 0b100000
    TIME = 0b101000
    DEFAULT = 0b111000
    VERBOSE = 0b111100
    VERY_VERBOSE = 0b111111


class ExecutionSettings(Model, UpdateableMixin):
    parallelization: ParallelizationStrategy = Property(
        default=ParallelizationStrategy.CHUNK
    )
    num_threads: ThreadCount = Property(default=None)
    chunk_size: Annotated[int, Field(strict=True, gt=1)] = Property(default=32)
    warehouse_factor: Annotated[int, Field(strict=True, gt=1)] = Property(default=4)
    log_file: Union[Path, None] = Property(default="helios.log")
    log_file_only: bool = Property(default=False)
    verbosity: LogVerbosity = Property(default=LogVerbosity.DEFAULT)


# Storage for global execution settings
_global_execution_settings: ExecutionSettings = ExecutionSettings()


def set_execution_settings(
    execution_settings: Union[ExecutionSettings, None] = None, **parameters
):
    """Set the global execution settings for the Helios++ library

    :param execution_settings:
        An instance of execution settings to use as the global settings.
        Alternatively, None can be passed to only allow manipulation of
        individual parameters passed as keyword arguments.
    :type execution_settings: Union[ExecutionSettings, None]
    :param parameters:
        Individual parameters to set on the global execution settings.
    """

    global _global_execution_settings

    # Update the global settings with the provided instance
    if execution_settings is not None:
        _global_execution_settings = execution_settings

    # Update the global settings with the provided parameters
    _global_execution_settings.update_from_dict(parameters)


def compose_execution_settings(
    local_settings: Union[ExecutionSettings, None] = None, parameters: dict = {}
) -> ExecutionSettings:
    """Compose execution settings from global, local and manual settings

    :param local_settings:
        The local execution settings to use. If None, the global settings
        will be used as the base.
    :type local_settings: Union[ExecutionSettings, None]
    :param parameters:
        Individual parameters to set on the execution settings.
    """

    # Determine base settings
    if local_settings is None:
        result = _global_execution_settings
    else:
        result = local_settings

    # Update the base settings with the provided parameters
    result.update_from_dict(parameters, skip_exceptions=True)

    return result

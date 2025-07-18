from helios.validation import (
    CreatedDirectory,
    Length,
    Model,
    ThreadCount,
    TimeInterval,
    UpdateableMixin,
    units,
)

from enum import IntEnum
from pydantic import PositiveInt
from typing import Optional
from logging import ERROR, DEBUG, INFO, WARNING
from datetime import datetime
import os

import _helios
import sys

# StrEnum is Python >= 3.11, so we use a conda-forge packaged backport
if sys.version_info >= (3, 11):
    from enum import StrEnum
else:
    from backports.strenum import StrEnum


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

    def apply(self) -> None:
        """Call the corresponding C++ logging-level setter."""
        match self:
            case LogVerbosity.SILENT:
                _helios.logging_silent()
            case LogVerbosity.QUIET:
                _helios.logging_quiet()
            case LogVerbosity.TIME:
                _helios.logging_time()
            case LogVerbosity.DEFAULT:
                _helios.logging_default()
            case LogVerbosity.VERBOSE:
                _helios.logging_verbose()
            case LogVerbosity.VERY_VERBOSE:
                _helios.logging_very_verbose()


class KDTreeFactoryType(IntEnum):
    SIMPLE = 1
    SAH = 2
    SAH_BEST_AXIS = 3
    SAH_APPROXIMATION = 4


class OutputFormat(StrEnum):
    LAS = "las"
    LAZ = "laz"
    XYZ = "xyz"
    NPY = "npy"
    LASPY = "laspy"

    # TODO: Determine whether we want formats las10 and laz10 or whether this
    #       is exactly the time to abolish them.


class ForceOnGroundStrategy(IntEnum):
    NONE = 0
    LEAST_COMPLEX = 1
    MOST_COMPLEX = -1


class ExecutionSettings(Model, UpdateableMixin):
    parallelization: ParallelizationStrategy = ParallelizationStrategy.CHUNK
    num_threads: ThreadCount = None
    chunk_size: PositiveInt = 32
    warehouse_factor: PositiveInt = 4
    log_file: bool = False
    log_file_only: bool = False
    verbosity: LogVerbosity = LogVerbosity.SILENT
    factory_type: KDTreeFactoryType = KDTreeFactoryType.SAH_APPROXIMATION
    kdt_num_threads: ThreadCount = None
    kdt_geom_num_threads: ThreadCount = None
    sah_nodes: PositiveInt = 32
    discard_shutdown: bool = True


class OutputSettings(Model, UpdateableMixin):
    format: OutputFormat = OutputFormat.NPY
    split_by_channel: bool = False
    output_dir: CreatedDirectory = "output"
    write_waveform: bool = False
    write_pulse: bool = False
    las_scale: Length = 0.0001


class FullWaveformSettings(Model, cpp_class=_helios.FWFSettings):
    bin_size: TimeInterval = 0.25 * units.ns
    beam_sample_quality: PositiveInt = 3
    win_size: TimeInterval = 1.0 * units.ns
    max_fullwave_range: TimeInterval = 0.0 * units.ns

    def _to_cpp(self):
        # Convert to the underlying C++ structure, undoing SI unit conversion

        fwf = _helios.FWFSettings()
        fwf.bin_size = self.bin_size * 1e9
        fwf.beam_sample_quality = self.beam_sample_quality
        fwf.win_size = self.win_size * 1e9
        fwf.max_fullwave_range = self.max_fullwave_range * 1e9

        return fwf


# Storage for global settings
_global_execution_settings: ExecutionSettings = ExecutionSettings()
_global_output_settings: OutputSettings = OutputSettings()


def set_execution_settings(
    execution_settings: Optional[ExecutionSettings] = None, **parameters
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

    _global_execution_settings.verbosity.apply()
    apply_log_writing(_global_execution_settings)


def set_output_settings(output_settings: Optional[OutputSettings] = None, **parameters):
    """Set the global output settings for the Helios++ library

    :param output_settings:
        An instance of output settings to use as the global settings.
        Alternatively, None can be passed to only allow manipulation of
        individual parameters passed as keyword arguments.
    :type output_settings: Optional[OutputSettings]
    :param parameters:
        Individual parameters to set on the global output settings.
    """

    global _global_output_settings

    # Update the global settings with the provided instance
    if output_settings is not None:
        _global_output_settings = output_settings

    # Update the global settings with the provided parameters
    _global_output_settings.update_from_dict(parameters)


def _compose_settings(settings, parameters):
    """Compose settings from multiple sources

    :param settings:
        The settings to compose. These should be instances of the
        same Model class.
    :param parameters:
        Individual parameters to set on the composed settings.
    """

    result = None

    # Find the most specialized base settings class
    for base in settings:
        if base is not None:
            result = base.clone()
            break

    # There should always be one base settings class, because the
    # global one defaults to a default instance of the settings class
    assert result is not None

    # Update the base settings with the provided parameters
    result.update_from_dict(parameters, skip_exceptions=True)

    return result


def compose_execution_settings(
    local_settings: Optional[ExecutionSettings] = None, parameters: dict = {}
) -> ExecutionSettings:
    """Compose execution settings from global, local and manual settings

    :param local_settings:
        The local execution settings to use. If None, the global settings
        will be used as the base.
    :type local_settings: Union[ExecutionSettings, None]
    :param parameters:
        Individual parameters to set on the execution settings.
    """

    return _compose_settings((local_settings, _global_execution_settings), parameters)


def compose_output_settings(
    local_settings: Optional[OutputSettings] = None, parameters: dict = {}
) -> OutputSettings:
    """Compose output settings from global, local and manual settings

    :param local_settings:
        The local output settings to use. If None, the global settings
        will be used as the base.
    :type local_settings: Union[OutputSettings, None]
    :param parameters:
        Individual parameters to set on the output settings.
    """

    return _compose_settings((local_settings, _global_output_settings), parameters)


def apply_log_writing(execution_settings: ExecutionSettings):
    """Apply the chosen log writing mode to c++ part"""
    config: dict[str, str] = {}

    if execution_settings.log_file_only or execution_settings.log_file:
        log_dir = "output/logs"
        os.makedirs(log_dir, exist_ok=True)
        file_log = os.path.join(
            log_dir, f"helios_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log"
        )
        config["file_name"] = file_log

    if execution_settings.log_file_only:
        config["type"] = "file"
    elif execution_settings.log_file:
        config["type"] = "full"
    else:
        config["type"] = "std_out"

    if config["type"] in {"file", "full"} and "file_name" not in config:
        raise ValueError(f"Logger type '{config['type']}' requires a file_name")

    _helios.configure_logging(config)

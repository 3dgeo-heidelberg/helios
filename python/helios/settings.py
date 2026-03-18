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
from pydantic import PositiveInt, BaseModel
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
    """Enum representing the parallelization strategy to use for processing the survey.
    
    See also: https://doi.org/10.1109/ACCESS.2022.3211072
    """
    CHUNK = 0
    """chunk"""
    
    WAREHOUSE = 1
    """warehouse"""


class LogVerbosity(IntEnum):
    """
    Enum representing the logging verbosity level to use for processing the survey.
    """
    SILENT = 0b000000
    "No logging output will be produced."
    
    QUIET = 0b100000
    "Only error messages will be logged."
    
    TIME = 0b101000
    "Log timing information."
    
    DEFAULT = 0b111000
    "Default logging level."
    
    VERBOSE = 0b111100
    "Verbose logging output."
    
    VERY_VERBOSE = 0b111111
    "Very verbose logging output."

    def apply(self) -> None:
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
    """Enum representing the type of KDTree factory to use for building the KDTree.
    
    See also: https://doi.org/10.1109/ACCESS.2022.3211072
    """
    
    SIMPLE = 1
    """Simple KDTree based on median balancing"""
    
    SAH = 2
    """Surface Area Heuristic (SAH) based KDTree"""

    SAH_BEST_AXIS = 3
    """SAH based KDTree with best axis criteria"""
    
    SAH_APPROXIMATION = 4
    """fast approximation of SAH based KDTree (default)"""


class OutputFormat(StrEnum):
    """
    Enum representing the output format to use for writing the point clouds.
    """
    LAS = "las"
    """LAS format (v1.4)."""
    LAZ = "laz"
    """LAZ format (v1.4, compressed)."""

    XYZ = "xyz"
    """Plain text format with x, y, z coordinates and additional attributes as columns."""

    NPY = "npy"
    """Structured numpy array with fields for x, y, z coordinates and additional attributes."""

    LASPY = "laspy"
    """laspy format"""

    # TODO: Determine whether we want formats las10 and laz10 or whether this
    #       is exactly the time to abolish them.


class ProgressBarStrategy(StrEnum):
    NONE = "none"
    LEGS = "legs"
    TIME = "time"
    LEGS_TIME = "legs+time"
    PER_LEG_TIME = "per_leg_time"


class ForceOnGroundStrategy(IntEnum):
    """Enum representing the strategy to use for forcing points on the ground."""

    NONE = 0
    """Do not force points on the ground."""

    LEAST_COMPLEX = 1
    """Use the minimum z vertex to calculate ground translation. Least computationally accurate."""

    MOST_COMPLEX = -1
    """Force on ground with optimum vertex (the one which is closest to the ground plane). Optimum solution guaranteed, but computationally expensive."""


class ExecutionSettings(Model, UpdateableMixin):
    """Class representing the execution settings for a HELIOS++ survey.

    :param parallelization: The parallelization strategy to use for processing the survey.
    :param num_threads: The number of threads to use for processing the survey. If None, the number of hardware threads will be used.
    :param chunk_size: The chunk size to use for processing the survey. Only used if the parallelization strategy is set to CHUNK.
    :param warehouse_factor: The warehouse factor to use for processing the survey. Only used if the parallelization strategy is set to WAREHOUSE.
    :param log_file: Whether to write a log file for the survey. If true, a log file will be written to the output directory.
    :param log_file_only: Whether to write only a log file for the survey. If true, no logging output will be written to the console.
    :param verbosity: The logging verbosity level to use for processing the survey.
    :param factory_type: The type of KDTree factory to use for building the KDTree.
    :param kdt_num_threads: The number of threads to use for building the KDTree. If None, the number of hardware threads will be used.
    :param kdt_geom_num_threads: The number of threads to use for building the geometry of the KDTree. If None, the number of hardware threads will be used.
    :param sah_nodes: The number of nodes to use for the SAH approximation. Only used if the factory type is set to SAH_APPROXIMATION.
    :param discard_shutdown: Whether to discard the shutdown message when the survey is finished.
    :type parallelization: ParallelizationStrategy
    :type num_threads: Optional[ThreadCount]
    :type chunk_size: PositiveInt
    :type warehouse_factor: PositiveInt
    :type log_file: bool
    :type log_file_only: bool
    :type verbosity: LogVerbosity
    :type factory_type: KDTreeFactoryType
    :type kdt_num_threads: Optional[ThreadCount]
    :type kdt_geom_num_threads: Optional[ThreadCount]
    :type sah_nodes: PositiveInt
    :type discard_shutdown: bool
    """
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
    progressbar: ProgressBarStrategy = ProgressBarStrategy.PER_LEG_TIME


class OutputSettings(Model, UpdateableMixin):
    """
    Class representing the output settings for a HELIOS++ survey.

    :param format: The output format to use for writing the point clouds.
    :param split_by_channel: Whether to split the output point clouds by channel.
    :param output_dir: The output directory to write the point clouds to.
    :param write_waveform: Whether to write the waveform data for the points.
    :param write_pulse: Whether to write the pulse data for the points.
    :param las_scale: The scale to use for writing LAS files. Only used if the output format is set to LAS or LAZ.
    :type format: OutputFormat
    :type split_by_channel: bool
    :type output_dir: CreatedDirectory
    :type write_waveform: bool
    :type write_pulse: bool
    :type las_scale: Length
    """
    format: OutputFormat = OutputFormat.NPY
    split_by_channel: bool = False
    output_dir: CreatedDirectory = "output"
    write_waveform: bool = False
    write_pulse: bool = False
    las_scale: Length = 0.0001


class FullWaveformSettings(Model, cpp_class=_helios.FWFSettings):
    """Class representing the settings for full waveform processing.
    
    :param bin_size: The size of the bins to use for full waveform processing in seconds. Default is 0.25 ns.
    :param beam_sample_quality: The beam sample quality to use for full waveform processing. Default is 3.
    :param win_size: The size of the window to use for full waveform processing in seconds. Default is 1.0 ns.
    :param max_fullwave_range: The maximum range to use for full waveform processing in seconds. Default is 0.0 ns (no maximum range).
    :type bin_size: TimeInterval
    :type beam_sample_quality: PositiveInt
    :type win_size: TimeInterval
    :type max_fullwave_range: TimeInterval
    """
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
    # Apply the chosen log writing mode to c++ part
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

# Export the version given in project metadata
from importlib import metadata

__version__ = metadata.version(__package__)
del metadata

from helios.leg import Leg
from helios.platforms import (
    Platform,
    PlatformSettings,
    DynamicPlatformSettings,
    load_traj_csv,
    TrajectorySettings,
    StaticPlatformSettings,
    list_platforms,
    platform_from_name,
)
from helios.scanner import Scanner, ScannerSettings, list_scanners, scanner_from_name
from helios.scene import StaticScene, ScenePart
from helios.settings import (
    ExecutionSettings,
    ForceOnGroundStrategy,
    FullWaveformSettings,
    KDTreeFactoryType,
    LogVerbosity,
    OutputFormat,
    OutputSettings,
    ParallelizationStrategy,
    set_execution_settings,
    set_output_settings,
)
from helios.survey import Survey
from helios.utils import add_asset_directory, combine_parameters
from helios.validation import units
from _helios import HeliosException

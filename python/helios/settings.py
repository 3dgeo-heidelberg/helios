from helios.validation import Model, Property, ThreadCount

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


class ExecutionSettings(Model):
    parallelization: ParallelizationStrategy = Property(
        default=ParallelizationStrategy.CHUNK
    )
    num_threads: ThreadCount = Property(default=None)
    chunk_size: Annotated[int, Field(strict=True, gt=1)] = Property(default=32)
    warehouse_factor: Annotated[int, Field(strict=True, gt=1)] = Property(default=4)
    log_file: Union[Path, None] = Property(default="helios.log")
    log_file_only: bool = Property(default=False)
    verbosity: LogVerbosity = Property(default=LogVerbosity.DEFAULT)

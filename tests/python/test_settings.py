from helios.settings import *


def test_execution_settings_defaults():
    settings = ExecutionSettings()

    assert settings.parallelization == ParallelizationStrategy.CHUNK
    assert isinstance(settings.num_threads, int)
    assert settings.num_threads >= 1
    assert settings.chunk_size == 32
    assert settings.warehouse_factor == 4
    assert settings.log_file
    assert not settings.log_file_only
    assert settings.verbosity == LogVerbosity.DEFAULT

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


def test_compose_execution_settings():
    # Use local settings if provided
    local = ExecutionSettings(chunk_size=64)
    settings = compose_execution_settings(local)
    assert settings.chunk_size == 64

    # Use manually provided parameters
    settings = compose_execution_settings(local, {"log_file": "test.log"})
    assert settings.chunk_size == 64
    assert settings.log_file == Path("test.log")


def test_set_execution_settings(reset_global_state):
    settings = ExecutionSettings(chunk_size=64)
    set_execution_settings(settings)
    assert compose_execution_settings().chunk_size == 64

    set_execution_settings(log_file="test.log")
    assert compose_execution_settings().chunk_size == 64
    assert compose_execution_settings().log_file == Path("test.log")

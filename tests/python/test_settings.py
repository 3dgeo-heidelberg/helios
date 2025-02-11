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
    assert settings.factory_type == KDTreeFactoryType.SAH_APPROXIMATION
    assert isinstance(settings.kdt_num_threads, int)
    assert settings.kdt_num_threads >= 1
    assert isinstance(settings.kdt_geom_num_threads, int)
    assert settings.kdt_geom_num_threads >= 1
    assert settings.sah_nodes == 32


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


def test_output_settings_defaults():
    settings = OutputSettings()

    assert settings.format == OutputFormat.NPY
    assert not settings.split_by_channel
    assert isinstance(settings.output_dir, Path)
    assert not settings.write_waveform
    assert not settings.write_pulse
    assert settings.las_scale == 0.0001


def test_compose_output_settings():
    # Use local settings if provided
    local = OutputSettings(format=OutputFormat.XYZ)
    settings = compose_output_settings(local)
    assert settings.format == OutputFormat.XYZ

    # Use manually provided parameters
    settings = compose_output_settings(local, {"write_pulse": True})
    assert settings.format == OutputFormat.XYZ
    assert settings.write_pulse == True


def test_set_output_settings(reset_global_state):
    settings = OutputSettings(format=OutputFormat.XYZ)
    set_output_settings(settings)
    assert compose_output_settings().format == OutputFormat.XYZ

    set_output_settings(write_pulse=True)
    assert compose_output_settings().format == OutputFormat.XYZ
    assert compose_output_settings().write_pulse == True

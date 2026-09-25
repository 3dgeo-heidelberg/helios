from helios.settings import *
from helios.survey import OutputFormat, Survey

import copy
from pathlib import Path
import pytest


def test_execution_settings_defaults():
    settings = ExecutionSettings()

    assert settings.parallelization == ParallelizationStrategy.CHUNK
    assert isinstance(settings.num_threads, int)
    assert settings.num_threads >= 1
    assert settings.chunk_size == 32
    assert settings.warehouse_factor == 4
    assert not settings.log_file
    assert not settings.log_file_only
    assert settings.verbosity == LogVerbosity.DEFAULT
    assert settings.factory_type == KDTreeFactoryType.SAH_APPROXIMATION
    assert isinstance(settings.kdt_num_threads, int)
    assert settings.kdt_num_threads >= 1
    assert isinstance(settings.kdt_geom_num_threads, int)
    assert settings.kdt_geom_num_threads >= 1
    assert settings.sah_nodes == 32
    assert settings.progressbar == ProgressBarStrategy.PER_LEG_TIME


def test_execution_settings_progressbar_accepts_strategy_strings():
    settings = ExecutionSettings(progressbar="legs+time")
    assert settings.progressbar == ProgressBarStrategy.LEGS_TIME


def test_compose_execution_settings():
    # Use local settings if provided
    local = ExecutionSettings(chunk_size=64)
    settings = compose_execution_settings(local)
    assert settings.chunk_size == 64

    # Use manually provided parameters
    settings = compose_execution_settings(local, {"log_file": True})
    assert settings.chunk_size == 64
    assert settings.log_file


def test_set_execution_settings(reset_global_state):
    settings = ExecutionSettings(chunk_size=64)
    set_execution_settings(settings)
    assert compose_execution_settings().chunk_size == 64

    set_execution_settings(log_file=True)
    assert compose_execution_settings().chunk_size == 64
    assert compose_execution_settings().log_file


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


def test_full_waveform_settings_defaults():
    settings = FullWaveformSettings()

    assert settings.bin_size == 2.5e-10
    assert settings.beam_sample_quality == 3
    assert settings.win_size == 1e-9
    assert settings.max_fullwave_range == 0.0


def test_convert_full_waveform_settings_to_cpp():
    settings = FullWaveformSettings(
        bin_size=1.0 * units.ns,
        beam_sample_quality=4,
        win_size=2.0 * units.ns,
        max_fullwave_range=10.0 * units.ns,
    )
    cpp_settings = settings._to_cpp()

    assert cpp_settings.bin_size == 1.0
    assert cpp_settings.beam_sample_quality == 4
    assert cpp_settings.win_size == 2.0
    assert cpp_settings.max_fullwave_range == 10.0


def test_execution_settings_clone_and_deepcopy_run_survey(survey):
    settings = ExecutionSettings(num_threads=1, chunk_size=16)
    settings.runtime_note = "temporary"

    for copied in (settings.clone(), copy.deepcopy(settings)):
        assert copied is not settings
        assert copied.chunk_size == 16
        assert copied.num_threads == 1
        assert not hasattr(copied, "runtime_note")

        copied.chunk_size = 24
        assert settings.chunk_size == 16

        points, trajectory = survey.clone().run(
            format=OutputFormat.NPY, execution_settings=copied
        )
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


def test_output_settings_clone_and_deepcopy_run_survey(survey):
    settings = OutputSettings(format=OutputFormat.NPY, split_by_channel=False)
    settings.runtime_note = "temporary"

    for copied in (settings.clone(), copy.deepcopy(settings)):
        assert copied is not settings
        assert copied.format == OutputFormat.NPY
        assert not hasattr(copied, "runtime_note")

        points, trajectory = survey.clone().run(output_settings=copied)
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


def test_full_waveform_settings_clone_and_deepcopy_run_survey(survey):
    settings = FullWaveformSettings(
        bin_size=1.0 * units.ns,
        beam_sample_quality=4,
        win_size=2.0 * units.ns,
        max_fullwave_range=10.0 * units.ns,
    )
    settings.runtime_note = "temporary"

    for copied in (settings.clone(), copy.deepcopy(settings)):
        assert copied is not settings
        assert copied.beam_sample_quality == 4
        assert not hasattr(copied, "runtime_note")

        cloned_survey = survey.clone()
        cloned_survey.full_waveform_settings = copied
        points, trajectory = cloned_survey.run(
            format=OutputFormat.NPY, execution_settings=ExecutionSettings(num_threads=1)
        )
        assert points.shape[0] > 0
        assert trajectory.shape[0] > 0


def _write_invalid_survey_xml(path: Path) -> None:
    path.write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<document>
    <survey name="arbaro_demo_tls"
            scene="data/scenes/demo/arbaro_demo.xml#arbaro_demo"
            platform="data/platforms.xml#tripod"
            scanner="data/scanners_tls.xml#riegl_vz400">
        <FWFSettings binSize_ns="0.2" beamSampleQuality="3" winSize_ns="0.3" />
        <leg>
            <platformSettings x="1.0" y="25.5" onGround="true" />
            <scannerSettings template="profile1"
                             verticalAngleMin_deg="-40.0"
                             verticalAngleMax_deg="60"
                             headRotateStart_deg="100"
                             headRotateStop_deg="225"
                             trajectoryTimeInterval_s="1.0"/>
        </leg>
    </survey>
</document>
""",
        encoding="utf-8",
    )


def test_fullwaveform_settings_rejects_invalid_window_in_python():
    with pytest.raises(ValueError, match="win_size must satisfy"):
        FullWaveformSettings(
            bin_size=0.2 * units.ns,
            win_size=0.3 * units.ns,
        )


def test_fullwaveform_settings_rejects_invalid_window_from_xml(tmp_path):
    xml_file = tmp_path / "invalid_survey.xml"
    _write_invalid_survey_xml(xml_file)

    with pytest.raises(ValueError, match="winSize_ns must be >="):
        Survey.from_xml(xml_file)

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
    assert settings.verbosity == LogVerbosity.SILENT
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

    assert settings.sample_spacing == 0.03747405725
    assert settings.beam_sample_quality == 3
    assert settings.beam_sampling_factor == 2.0
    assert settings.peak_window == 0.149896229
    assert settings.max_recording_span == 0.0


def test_convert_full_waveform_settings_to_cpp():
    settings = FullWaveformSettings(
        bin_size=1.0 * units.ns,
        beam_sample_quality=4,
        beam_sampling_factor=1.5,
        win_size=2.0 * units.ns,
        max_fullwave_range=10.0 * units.ns,
    )
    cpp_settings = settings._to_cpp()

    assert cpp_settings.sample_spacing == pytest.approx(0.149896229)
    assert cpp_settings.beam_sample_quality == 4
    assert cpp_settings.beam_sampling_factor == 1.5
    assert cpp_settings.peak_window == pytest.approx(0.299792458)
    assert cpp_settings.max_recording_span == pytest.approx(1.49896229)


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
    with pytest.raises(ValueError, match="peakWindow_m must be >="):
        FullWaveformSettings(
            bin_size=0.2 * units.ns,
            win_size=0.3 * units.ns,
        )


@pytest.mark.parametrize("factor", [0, -1, float("inf"), float("nan")])
def test_fullwaveform_settings_rejects_invalid_sampling_factor(factor):
    with pytest.raises(ValueError):
        FullWaveformSettings(beam_sampling_factor=factor)


def test_fullwaveform_sampling_settings_roundtrip(tmp_path):
    settings = FullWaveformSettings(beam_sample_quality=8, beam_sampling_factor=1.5)
    path = settings.to_yaml(tmp_path / "sampling.yaml", shallow=False)
    restored = FullWaveformSettings.from_yaml(path)
    assert restored.beam_sample_quality == 8
    assert restored.beam_sampling_factor == 1.5
    assert restored._to_cpp().beam_sampling_factor == 1.5


def test_fullwaveform_settings_rejects_invalid_window_from_xml(tmp_path):
    xml_file = tmp_path / "invalid_survey.xml"
    _write_invalid_survey_xml(xml_file)

    with pytest.raises(ValueError, match="peakWindow_m must be >="):
        Survey.from_xml(xml_file)


@pytest.mark.parametrize("shape", ["gamma", "gaussian"])
def test_analytic_waveform_settings_roundtrip(tmp_path, shape):
    settings = FullWaveformSettings(
        sample_spacing="2 cm",
        peak_window="10 cm",
        max_recording_span="25 m",
        pulse_shape=shape,
        correct_peak_offset=False,
        recording_tail_tolerance=1e-6,
    )
    path = settings.to_yaml(tmp_path / "waveform.yaml")
    text = path.read_text()
    assert "bin_size" not in text
    restored = FullWaveformSettings.from_yaml(path)
    assert restored.sample_spacing == 0.02
    assert restored.peak_window == 0.1
    assert restored.max_recording_span == 25
    assert restored.pulse_shape == shape
    assert restored.correct_peak_offset is False
    assert restored.recording_tail_tolerance == 1e-6


def test_legacy_waveform_constructor_and_yaml(tmp_path):
    import yaml

    with pytest.warns(DeprecationWarning):
        settings = FullWaveformSettings(bin_size="0.25 ns", win_size="1 ns")
    assert settings.sample_spacing == pytest.approx(0.03747405725)
    assert settings.peak_window == pytest.approx(0.149896229)
    with pytest.raises(ValueError, match="Specify only one"):
        FullWaveformSettings(bin_size="0.25 ns", sample_spacing=0.02)
    path = settings.to_yaml(tmp_path / "legacy.yaml")
    doc = yaml.safe_load(path.read_text())
    doc["serialization_minor_version"] = 0
    doc["fields"] = {
        "bin_size": 0.25e-9,
        "win_size": 1e-9,
        "max_fullwave_range": 100e-9,
    }
    path.write_text(yaml.safe_dump(doc))
    with pytest.warns(DeprecationWarning):
        restored = FullWaveformSettings.from_yaml(path)
    assert restored.max_recording_span == pytest.approx(14.9896229)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"sample_spacing": 0},
        {"sample_spacing": float("inf")},
        {"peak_window": 0.01},
        {"recording_tail_tolerance": 0},
        {"recording_tail_tolerance": 1},
        {"pulse_shape": "triangle"},
        {"max_recording_span": -1},
    ],
)
def test_analytic_waveform_invalid_settings(kwargs):
    with pytest.raises(ValueError):
        FullWaveformSettings(**kwargs)


def test_range_setting_assignment_validates_before_mutation():
    settings = FullWaveformSettings()
    with pytest.raises(ValueError):
        settings.sample_spacing = 1
    assert settings.sample_spacing == 0.03747405725

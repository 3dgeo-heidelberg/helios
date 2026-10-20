"""Pulse timing, sampled definitions, and persistence through scanner preparation."""

import copy

import numpy as np
import pytest
import yaml

import _helios
from helios import FullWaveformSettings, SampledPulse, Scanner, Survey


def test_sampled_definition_units_ownership_and_immutability():
    offsets = np.array([-2e-9, 0, 3e-9])
    amplitudes = np.array([0.0, 4, 0])
    pulse = SampledPulse(offsets=offsets, amplitudes=amplitudes)
    offsets[:] = 0
    amplitudes[:] = 0
    assert pulse.offsets == pytest.approx((-2e-9, 0, 3e-9))
    assert pulse.amplitudes == (0, 1, 0)
    assert pulse.peak_offset is None
    assert pulse._cpp_object.evaluate(0) == 1
    with pytest.raises(AttributeError, match="immutable"):
        pulse.amplitudes = (0, 2, 0)
    with pytest.raises(TypeError):
        pulse.amplitudes[1] = 2
    assert copy.deepcopy(pulse) is pulse
    plateau = SampledPulse(
        offsets=["-2 ns", "0 ns", "3 ns", "7 ns"],
        amplitudes=[0, 2, 2, 0],
        peak_offset="1 ns",
    )
    assert plateau.peak_offset == pytest.approx(1e-9)
    assert plateau._cpp_object.peak_offset == pytest.approx(0.149896229)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"offsets": [0], "amplitudes": [1]},
        {"offsets": [0, 1], "amplitudes": [1]},
        {"offsets": [0, 0], "amplitudes": [0, 1]},
        {"offsets": [1, 0], "amplitudes": [0, 1]},
        {"offsets": [0, float("inf")], "amplitudes": [0, 1]},
        {"offsets": [0, 1], "amplitudes": [float("nan"), 1]},
        {"offsets": [0, 1], "amplitudes": [-1, 1]},
        {"offsets": [0, 1], "amplitudes": [0, 0]},
        {"offsets": [0, 1], "amplitudes": [1, 1]},
        {"offsets": [0, 1, 2], "amplitudes": [1, 0, 1]},
        {"offsets": [0, 1], "amplitudes": [0, 1], "peak_offset": 0.5},
        {"offsets": [0, 1], "amplitudes": [0, 1], "peak_offset": float("inf")},
    ],
)
def test_invalid_sampled_definitions(kwargs):
    with pytest.raises(ValueError):
        SampledPulse(**kwargs)


@pytest.mark.parametrize("shallow", [False, True])
@pytest.mark.parametrize("explicit", [False, True])
def test_pulse_settings_yaml_and_cpp_roundtrip(tmp_path, shallow, explicit):
    pulse = SampledPulse(
        offsets=["-2 ns", "1 ns", "3 ns"],
        amplitudes=[0, 5, 0],
        peak_offset="1 ns" if explicit else None,
    )
    settings = FullWaveformSettings(
        pulse_shape="sampled",
        sampled_pulse=pulse,
        gaussian_mean="-3 ns",
    )
    path = settings.to_yaml(tmp_path / "settings.yaml", shallow=shallow)
    restored = FullWaveformSettings.from_yaml(path)
    for result in [
        restored,
        copy.deepcopy(settings),
        FullWaveformSettings._from_cpp(settings._to_cpp()),
    ]:
        assert result.gaussian_mean == pytest.approx(-3e-9)
        assert result.sampled_pulse.offsets == pytest.approx(pulse.offsets)
        assert result.sampled_pulse.amplitudes == pulse.amplitudes
        assert result.sampled_pulse.peak_offset == pulse.peak_offset
        assert result._to_cpp().sampled_pulse.peak_offset == pytest.approx(0.149896229)
    settings.pulse_shape = "gamma"
    assert settings.sampled_pulse is pulse
    settings.pulse_shape = "sampled"
    with pytest.raises(ValueError, match="requires sampled_pulse"):
        settings.sampled_pulse = None
    assert settings.sampled_pulse is pulse


def test_gaussian_mean_and_old_yaml_default(tmp_path):
    assert _helios.GaussianPulse(4).peak_offset == 0
    settings = FullWaveformSettings(pulse_shape="gaussian", gaussian_mean="-3 ns")
    assert settings._to_cpp().gaussian_mean == pytest.approx(-3e-9)
    assert _helios.PulseShape.create(
        "gaussian", 4, 1e-4, -3
    ).peak_offset == pytest.approx(-0.449688687)
    with pytest.raises(ValueError):
        FullWaveformSettings(gaussian_mean=float("inf"))
    with pytest.raises(ValueError):
        FullWaveformSettings(pulse_shape="sampled")
    path = settings.to_yaml(tmp_path / "legacy.yaml")
    data = yaml.safe_load(path.read_text())
    data["serialization_minor_version"] = 0
    del data["fields"]["gaussian_mean"]
    del data["fields"]["sampled_pulse"]
    path.write_text(yaml.safe_dump(data))
    assert FullWaveformSettings.from_yaml(path).gaussian_mean == 0


def test_multichannel_preparation_and_switching():
    scanner = Scanner.from_xml("data/scanners_als.xml", "riegl_vq-1560i")._cpp_object
    pulse = SampledPulse(offsets=[-2e-9, 1e-9, 4e-9], amplitudes=[0, 1, 0])
    settings = FullWaveformSettings(
        pulse_shape="sampled", sampled_pulse=pulse
    )._to_cpp()
    scanner.set_fwf_settings(settings, 0)
    scanner.prepare_pulse(0)
    first = scanner.get_pulse_model(0)
    assert isinstance(first, _helios.SampledPulse)
    scanner.prepare_pulse(0)
    assert scanner.get_pulse_model(0).evaluate(first.peak_offset) == 1
    scanner.prepare_pulse(1)
    assert not isinstance(scanner.get_pulse_model(1), _helios.SampledPulse)
    settings.pulse_shape = "gaussian"
    settings.gaussian_mean = -2e-9
    scanner.set_fwf_settings(settings, 0)
    scanner.prepare_pulse(0)
    assert scanner.get_pulse_model(0).peak_offset == pytest.approx(-0.299792458)
    assert first.evaluate(first.peak_offset) == 1
    settings.pulse_shape = "sampled"
    scanner.set_fwf_settings(settings, 0)
    scanner.prepare_pulse(0)
    assert isinstance(scanner.get_pulse_model(0), _helios.SampledPulse)


def test_sampled_survey_copy_and_yaml(tmp_path):
    survey = Survey.from_xml("data/test/serialization_survey.xml")
    survey.full_waveform_settings = FullWaveformSettings(
        pulse_shape="sampled",
        sampled_pulse=SampledPulse(offsets=[-2e-9, 0, 4e-9], amplitudes=[0, 1, 0]),
    )
    copied = copy.deepcopy(survey)
    path = copied.to_yaml(tmp_path / "survey.yaml", shallow=True)
    restored = Survey.from_yaml(path)
    for item in [survey, copied, restored]:
        assert item.full_waveform_settings.sampled_pulse.amplitudes == (0, 1, 0)
        assert item.full_waveform_settings._to_cpp().sampled_pulse.evaluate(0) == 1


@pytest.mark.parametrize("mean", ["-2 ns", "0 ns", "3 ns"])
def test_sampled_shape_survives_survey_run(survey, mean):
    survey.full_waveform_settings = FullWaveformSettings(
        pulse_shape="sampled",
        gaussian_mean=mean,
        sampled_pulse=SampledPulse(offsets=[-4e-9, 1e-9, 8e-9], amplitudes=[0, 1, 0]),
    )
    for item in [survey, copy.deepcopy(survey)]:
        points, _ = item.run(
            num_threads=1, kdt_num_threads=1, kdt_geom_num_threads=1, progressbar="none"
        )
        assert len(points) > 0
        model = item.scanner._cpp_object.get_pulse_model()
        assert isinstance(model, _helios.SampledPulse)
        assert model.peak_offset == pytest.approx(0.149896229)

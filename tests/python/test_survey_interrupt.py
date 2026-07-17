import threading

import pytest

from helios import survey as survey_module
from helios.settings import ExecutionSettings, OutputFormat, ProgressBarStrategy
from helios.survey import Survey, _start_playback_interruptible


def test_start_playback_interruptible_completes():
    class FakePlayback:
        def __init__(self):
            self.started = False
            self.stopped = False

        def start(self):
            self.started = True

        def stop(self):
            self.stopped = True

    playback = FakePlayback()
    _start_playback_interruptible(playback, poll_interval=0.001)

    assert playback.started is True
    assert playback.stopped is False


def test_start_playback_interruptible_stops_on_keyboard_interrupt(monkeypatch):
    class FakePlayback:
        def __init__(self):
            self._stop_event = threading.Event()
            self.stop_calls = 0

        def start(self):
            self._stop_event.wait(timeout=5)

        def stop(self):
            self.stop_calls += 1
            self._stop_event.set()

    playback = FakePlayback()
    original_join = threading.Thread.join
    join_calls = {"count": 0}

    def interrupt_once(self, timeout=None):
        join_calls["count"] += 1
        if join_calls["count"] == 1:
            raise KeyboardInterrupt
        return original_join(self, timeout)

    monkeypatch.setattr(threading.Thread, "join", interrupt_once)

    with pytest.raises(KeyboardInterrupt):
        _start_playback_interruptible(playback, poll_interval=0.001)

    assert playback.stop_calls == 1


def test_interrupted_dynamic_survey_is_consumed(monkeypatch, xml_dynamic_test_survey):
    dynamic_survey = xml_dynamic_test_survey
    starts = []

    def interrupt_start(playback):
        starts.append(playback)
        raise KeyboardInterrupt

    monkeypatch.setattr(survey_module, "_start_playback_interruptible", interrupt_start)
    execution_settings = ExecutionSettings(
        num_threads=1, progressbar=ProgressBarStrategy.NONE
    )

    with pytest.raises(KeyboardInterrupt):
        dynamic_survey.run(
            format=OutputFormat.NPY, execution_settings=execution_settings
        )

    with pytest.raises(RuntimeError, match="reset support is planned"):
        dynamic_survey.run(
            format=OutputFormat.NPY, execution_settings=execution_settings
        )
    assert len(starts) == 1

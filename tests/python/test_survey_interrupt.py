import threading

import pytest

from helios.survey import _start_playback_interruptible


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

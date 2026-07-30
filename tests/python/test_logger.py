import json
import logging
import sys
from types import SimpleNamespace
import pytest
from helios.settings import ExecutionSettings, OutputSettings
from helios.survey import Survey
from helios.scene import ScenePart

from importlib import import_module

logger = import_module("helios.logger")


@pytest.fixture(autouse=True)
def _reset_logger():
    logger.shutdown_logging()
    yield
    logger.shutdown_logging()


def _make_record(level=logging.INFO, msg="test message", logger_name="helios"):
    record = logging.LogRecord(
        name=logger_name,
        level=level,
        pathname=__file__,
        lineno=123,
        msg=msg,
        args=(),
        exc_info=None,
        func="test_fn",
    )
    return record


def test_logging_context_roundtrip():
    token = logger.set_logging_context(scan_id="scan-001", worker="main")
    try:
        record = _make_record()
        assert logger._ContextFilter().filter(record) is True
        assert record.scan_id == "scan-001"
        assert record.worker == "main"
    finally:
        logger.reset_logging_context(token)

    clean_record = _make_record()
    logger._ContextFilter().filter(clean_record)
    assert not hasattr(clean_record, "scan_id")
    assert not hasattr(clean_record, "worker")


def test_timezone_formatter():
    formatter = logger.TimezoneFormatter(
        datefmt="%Y-%m-%d %H:%M:%S %Z",
        utc=True,
    )

    record = _make_record()
    formatted_time = formatter.formatTime(record)

    assert formatted_time.endswith("UTC") or formatted_time.endswith("UTC+00:00")


def test_color_formatter(monkeypatch):
    monkeypatch.delenv("NO_COLOR", raising=False)
    monkeypatch.setattr(sys.stdout, "isatty", lambda: True)

    formatter = logger.ColorFormatter(force_color=False)
    text = formatter.format(_make_record(level=logging.WARNING, msg="warning message"))
    assert "\033[93m" in text
    assert "warning message" in text


def test_color_formatter_no_color(monkeypatch):
    monkeypatch.setenv("NO_COLOR", "1")
    monkeypatch.setattr(sys.stdout, "isatty", lambda: True)

    formatter = logger.ColorFormatter(force_color=False)
    text = formatter.format(_make_record(level=logging.WARNING, msg="warning message"))
    assert "\033[93m" not in text
    assert "warning message" in text


def test_json_formatter():
    formatter = logger.JsonFormatter()
    record = _make_record(level=logging.ERROR, msg="error message")
    record.category = "bootstrap"
    record.thread_id_hash = 42
    json_output = formatter.format(record)
    data = json.loads(json_output)

    assert data["level"] == "ERROR"
    assert data["message"] == "error message"
    assert data["logger"] == "helios"
    assert data["category"] == "bootstrap"
    assert data["thread_id_hash"] == 42
    assert "timestamp" in data


def test_logging_config_normalized_logfile(tmp_path):
    cfg = logger.LoggingConfig(logfile=tmp_path / "logs" / "helios.log")
    resolved = cfg.normalized_logfile()
    assert resolved == (tmp_path / "logs" / "helios.log").expanduser().resolve()
    assert resolved.is_absolute()


def test_install_default_excepthook_uncaught_exception(tmp_path):
    logfile = tmp_path / "errors.log"

    service = logger.configure_logging(
        logger.LoggingConfig(
            stdout=False,
            logfile=logfile,
            use_cpp_bridge=False,
        ),
        force=True,
    )

    logger.install_default_excepthook(service)
    sys.excepthook(
        ValueError,
        ValueError("Test uncaught exception"),
        None,
    )

    for h in service._handlers:
        h.flush()

    assert "ValueError: Test uncaught exception" in logfile.read_text()


def test_get_log_returns_child():
    logger.configure_logging(
        logger.LoggingConfig(
            level=logging.INFO,
            use_cpp_bridge=False,
            stdout=False,
            logfile=None,
        ),
        force=True,
    )
    child = logger.get_logger("scene")
    assert isinstance(child, logging.Logger)
    assert child.name == "helios.scene"


def test_python_lvl_to_cpp_lvl_mapping():
    assert (
        logger._py_level_to_cpp_min_level(logging.DEBUG)
        == logger._helios.LogLevel.DEBUG
    )
    assert (
        logger._py_level_to_cpp_min_level(logging.INFO) == logger._helios.LogLevel.INFO
    )
    assert (
        logger._py_level_to_cpp_min_level(logging.WARNING)
        == logger._helios.LogLevel.WARN
    )
    assert (
        logger._py_level_to_cpp_min_level(logging.ERROR) == logger._helios.LogLevel.ERR
    )


def test_cpp_backend_adapter_native_api(monkeypatch):
    calls: list[tuple[str, Any]] = []

    class FakeLogLevel:
        TRACE = "TRACE"
        DEBUG = "DEBUG"
        INFO = "INFO"
        TIME = "TIME"
        WARN = "WARN"
        ERR = "ERR"
        OFF = "OFF"

    class FakeLoggingConfig:
        def __init__(self):
            self.capacity = None
            self.min_level = None
            self.clear_queue = None

    event = SimpleNamespace(
        level=FakeLogLevel.WARN,
        message="test message from cpp",
        unix_time=1_700_000_000_123_000,
        thread_id_hash=236,
        file="tmp_scanner.cpp",
        line=42,
        function="test_fn",
    )

    dropped_counts = SimpleNamespace(
        overflow=3,
        shutdown=4,
    )

    fake_helios = SimpleNamespace(
        LogLevel=FakeLogLevel,
        LoggingConfig=FakeLoggingConfig,
        logging_configure=lambda cfg: calls.append(("configure", cfg)),
        logging_configure_silent=lambda: calls.append(("silent", None)),
        logging_shutdown=lambda: calls.append(("shutdown", None)),
        logging_set_min_level=lambda level: calls.append(("set_min_level", level)),
        logging_is_stopped=lambda: False,
        logging_drain=lambda max_items: [event],
        logging_wait_pop=lambda timeout_ms: event,
        logging_consume_dropped_counts=lambda: dropped_counts,
    )

    monkeypatch.setattr(
        logger,
        "_helios",
        fake_helios,
        raising=True,
    )

    backend = logger._CppLoggingBackend()

    cfg = logger.LoggingConfig(
        level=logging.WARNING,
        use_cpp_bridge=True,
        stdout=False,
        logfile=None,
    )

    backend.configure(cfg)

    assert calls[0][0] == "configure"

    cpp_cfg = calls[0][1]
    assert isinstance(cpp_cfg, FakeLoggingConfig)
    assert cpp_cfg.capacity == cfg.queue_capacity
    assert cpp_cfg.min_level == FakeLogLevel.WARN
    assert cpp_cfg.clear_queue is True

    backend.configure_silent()
    assert calls[1] == ("silent", None)

    backend.set_min_level(logging.ERROR)
    assert calls[2] == ("set_min_level", FakeLogLevel.ERR)

    assert backend.is_stopped() is False

    drained = backend.drain(max_items=10)

    assert len(drained) == 1
    assert isinstance(drained[0], logging.LogRecord)
    assert drained[0].getMessage() == "test message from cpp"
    assert drained[0].pathname == "tmp_scanner.cpp"
    assert drained[0].lineno == 42
    assert drained[0].funcName == "test_fn"
    assert drained[0].thread == 236

    popped = backend.wait_pop(100)

    assert isinstance(popped, logging.LogRecord)
    assert popped.getMessage() == "test message from cpp"

    dropped = backend.consume_dropped_counts()

    assert dropped.overflow == 3
    assert dropped.shutdown == 4

    backend.shutdown()
    assert calls[-1] == ("shutdown", None)


def test_logging_info_from_cpp(tmp_path):
    logfile = tmp_path / "tmp_helios.log"
    service = logger.configure_logging(
        logger.LoggingConfig(
            level=logging.DEBUG,
            use_cpp_bridge=True,
            logfile=logfile,
            file_format=logger.LogFormat.JSON,
        ),
        force=True,
    )
    try:
        scene_parts1 = ScenePart.from_vox(
            "data/test/semitransparent_voxels.vox",
            intersection_mode="transmittive",
        )
    finally:
        logger.shutdown_logging()

    log_text = logfile.read_text()
    assert "Reading detailed voxels from" in log_text


def test_run_w_ex_set(tmp_path):
    survey = Survey.from_xml("data/surveys/toyblocks/als_toyblocks.xml")
    ex_settings = ExecutionSettings()
    ex_settings.log_file_only = True
    outp_settings = OutputSettings(output_dir=tmp_path / "output")
    try:
        survey.run(execution_settings=ex_settings, output_settings=outp_settings)
    finally:
        logger.shutdown_logging()

    logs_dir = tmp_path / "output" / "logs"
    log_files = list(logs_dir.glob("helios*"))
    assert len(log_files) == 1
    log_file = log_files[0]
    text_data = log_file.read_text()
    assert "It was not possible to" in text_data

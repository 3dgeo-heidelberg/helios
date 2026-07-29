import os
import json
import logging
import logging.handlers
import sys
from datetime import datetime, timezone
import time
import threading
import contextvars
from enum import Enum
from pathlib import Path
from typing import Any, Optional
import dataclasses
import _helios

TRACE_LEVEL_NUM = 5
TIME_LEVEL_NUM = 25
logging.addLevelName(TRACE_LEVEL_NUM, "TRACE")
logging.addLevelName(TIME_LEVEL_NUM, "TIME")

_LOG_CONTEXT: contextvars.ContextVar[dict[str, Any]] = contextvars.ContextVar(
    "project_log_context",
    default={},
)


def _cpp_level_to_py_level(level: Any) -> int:
    try:
        raw = int(level)
    except Exception:
        raw = 2

    mapping = {
        0: TRACE_LEVEL_NUM,
        1: logging.DEBUG,
        2: logging.INFO,
        3: TIME_LEVEL_NUM,
        4: logging.WARNING,
        5: logging.ERROR,
        255: logging.CRITICAL + 10,
    }
    return mapping.get(raw, logging.INFO)


def _py_level_to_cpp_min_level(level: int) -> Any:
    if level <= TRACE_LEVEL_NUM:
        return _helios.LogLevel.TRACE
    if level <= logging.DEBUG:
        return _helios.LogLevel.DEBUG
    if level <= logging.INFO:
        return _helios.LogLevel.INFO
    if level <= TIME_LEVEL_NUM:
        return _helios.LogLevel.TIME
    if level <= logging.WARNING:
        return _helios.LogLevel.WARN
    if level <= logging.ERROR:
        return _helios.LogLevel.ERR
    return _helios.LogLevel.OFF


def set_logging_context(**fields: Any) -> contextvars.Token[dict[str, Any]]:
    """Merge fields into the current logging context."""

    current = dict(_LOG_CONTEXT.get())
    current.update(fields)
    return _LOG_CONTEXT.set(current)


def reset_logging_context(token: contextvars.Token[dict[str, Any]]) -> None:
    _LOG_CONTEXT.reset(token)


def clear_logging_context() -> None:
    _LOG_CONTEXT.set({})


class _ContextFilter(logging.Filter):
    """Inject context variables into each LogRecord."""

    def filter(self, record: logging.LogRecord) -> bool:
        for key, value in _LOG_CONTEXT.get().items():
            if not hasattr(record, key):
                setattr(record, key, value)
        return True


class LogFormat(str, Enum):
    TEXT = "text"
    JSON = "json"


_DEFAULT_COLORS = {
    "DEBUG": "\033[94m",
    "INFO": "\033[92m",
    "WARNING": "\033[93m",
    "ERROR": "\033[91m",
    "CRITICAL": "\033[41m\033[37m",
    "TRACE": "\033[37m",
    "TIME": "\033[36m",
}
_DEFAULT_RESET_COLOR = "\033[0m"

_DEFAULT_DATEFMT = "%Y-%m-%d %H:%M:%S %z"
_DEFAULT_FORMAT = (
    "%(asctime)s | %(levelname)s | %(thread)d |"
    "%(pathname)s:%(lineno)d | %(name)s | %(message)s"
)


class _SkipFileOnlyFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        return not getattr(record, "_helios_file_only", False)


class TimezoneFormatter(logging.Formatter):
    def __init__(
        self,
        fmt: str = _DEFAULT_FORMAT,
        datefmt: str = _DEFAULT_DATEFMT,
        *,
        utc: bool = False,
    ) -> None:
        super().__init__(fmt=fmt, datefmt=datefmt)
        self._utc = utc

    def formatTime(self, record: logging.LogRecord, datefmt: str | None = None) -> str:
        dt = (
            datetime.fromtimestamp(record.created, tz=timezone.utc)
            if self._utc
            else datetime.fromtimestamp(record.created).astimezone()
        )
        return dt.strftime(datefmt or self.datefmt)


class ColorFormatter(TimezoneFormatter):
    def __init__(
        self,
        fmt: str = _DEFAULT_FORMAT,
        datefmt: str = _DEFAULT_DATEFMT,
        *,
        force_color: bool = False,
        utc: bool = False,
    ) -> None:
        super().__init__(fmt=fmt, datefmt=datefmt, utc=utc)
        self._force_color = force_color

    def format(self, record: logging.LogRecord) -> str:
        text = super().format(record)
        if not self._should_color():
            return text
        color = _DEFAULT_COLORS.get(record.levelname)
        if not color:
            return text
        return f"{color}{text}{_DEFAULT_RESET_COLOR}"

    def _should_color(self) -> bool:
        if self._force_color:
            return True
        if os.environ.get("NO_COLOR") is not None:
            return False
        stream = getattr(sys, "stdout", None)
        if stream and type(stream).__name__ == "OutStream":
            return True
        return bool(stream and hasattr(stream, "isatty") and stream.isatty())


class JsonFormatter(logging.Formatter):
    _SKIP_KEYS = frozenset(
        {
            "args",
            "asctime",
            "created",
            "exc_info",
            "exc_text",
            "filename",
            "funcName",
            "levelno",
            "lineno",
            "module",
            "msecs",
            "message",
            "msg",
            "name",
            "pathname",
            "process",
            "processName",
            "relativeCreated",
            "stack_info",
            "thread",
            "threadName",
            "taskName",
        }
    )

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "timestamp": self._format_time(record),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "source_file": record.pathname,
            "source_line": record.lineno,
            "source_function": record.funcName,
            "module": record.module,
            "process": record.process,
            "function": record.funcName,
            "thread": record.thread,
            "line": record.lineno,
        }

        for key, value in record.__dict__.items():
            if key.startswith("_") or key in self._SKIP_KEYS:
                continue
            payload[key] = value

        if record.exc_info:
            payload["exception"] = self.formatException(record.exc_info)
        if record.stack_info:
            payload["stack"] = record.stack_info

        return json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"), default=repr
        )

    def _format_time(self, record: logging.LogRecord) -> str:
        dt = datetime.fromtimestamp(record.created, tz=timezone.utc)
        return dt.isoformat(timespec="milliseconds").replace("+00:00", "Z")


@dataclasses.dataclass(slots=True)
class LoggingConfig:
    level: int = logging.WARNING
    logger_name: str = "helios"
    use_cpp_bridge: bool = True
    queue_capacity: int = 4096
    batch_size: int = 256
    stdout: bool = True
    logfile: Optional[Path] = None
    max_bytes: int = 25 * 1024 * 1024
    backup_count: int = 5
    encoding: str = "utf-8"
    stdout_format: LogFormat = LogFormat.TEXT
    file_format: LogFormat = LogFormat.JSON
    datefmt: str = "%Y-%m-%d %H:%M:%S %z"
    stdout_utc: bool = False
    file_utc: bool = True
    capture_warnings: bool = True
    poll_interval: float = 0.1
    force_color: bool = False

    def normalized_logfile(self) -> Optional[Path]:
        if self.logfile is None:
            return None
        return Path(self.logfile).expanduser().resolve()


class JupyterActiveCellHandler(logging.StreamHandler):
    def __init__(self, stream: Any = None) -> None:
        super().__init__(stream)
        self._kernel: Any = None
        self._checked_ipython = False

    def _get_kernel(self) -> Any:
        if not self._checked_ipython:
            self._checked_ipython = True
            ipython = sys.modules.get("IPython")
            if ipython is not None:
                ip = ipython.get_ipython()
                if ip is not None and hasattr(ip, "kernel"):
                    self._kernel = ip.kernel
        return self._kernel

    def emit(self, record: logging.LogRecord) -> None:
        try:
            kernel = self._get_kernel()
            if kernel is not None and hasattr(self.stream, "set_parent"):
                parent = kernel.get_parent()
                if parent:
                    self.stream.set_parent(parent)
        except Exception:
            pass

        super().emit(record)


def _cpp_event_to_log_record(event: Any) -> logging.LogRecord:
    levelno = _cpp_level_to_py_level(event.level)
    created = (event.unix_time / 1_000_000.0) if event.unix_time else time.time()

    record = logging.LogRecord(
        name="helios",
        level=levelno,
        pathname=getattr(event, "file", ""),
        lineno=int(getattr(event, "line", 0) or 0),
        msg=getattr(event, "message", ""),
        args=(),
        exc_info=None,
        func=getattr(event, "function", "") or None,
    )

    record.created = created
    record.msecs = (created - int(created)) * 1000.0
    record.thread = event.thread_id_hash or threading.get_ident()
    record.process = os.getpid()
    return record


class _CppLoggingBackend:
    def configure(self, cfg: LoggingConfig) -> None:
        cpp_cfg = _helios.LoggingConfig()
        cpp_cfg.capacity = int(cfg.queue_capacity)
        cpp_cfg.min_level = _py_level_to_cpp_min_level(cfg.level)
        cpp_cfg.clear_queue = True
        _helios.logging_configure(cpp_cfg)

    def configure_silent(self) -> None:
        _helios.logging_configure_silent()

    def shutdown(self) -> None:
        _helios.logging_shutdown()

    def set_min_level(self, level: int) -> None:
        _helios.logging_set_min_level(_py_level_to_cpp_min_level(level))

    def is_stopped(self) -> bool:
        return bool(_helios.logging_is_stopped())

    def consume_dropped_counts(self) -> Any:
        return _helios.logging_consume_dropped_counts()

    def drain(self, max_items: int) -> list[logging.LogRecord]:
        events = _helios.logging_drain(int(max_items))
        return [_cpp_event_to_log_record(ev) for ev in events]

    def wait_pop(self, timeout_ms: int) -> logging.LogRecord | None:
        ev = _helios.logging_wait_pop(int(timeout_ms))
        if ev is None:
            return None
        return _cpp_event_to_log_record(ev)


class LoggingService:
    def __init__(self, config: LoggingConfig) -> None:
        self.config = config
        self.logger = logging.getLogger(config.logger_name)

        self._backend: _CppLoggingBackend = _CppLoggingBackend()
        self._consumer_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._started = False
        self._lock = threading.RLock()

        self._handlers: list[logging.Handler] = []
        self._error: BaseException | None = None

        self._install_logger_level_helpers()

    def __getattr__(self, name: str) -> Any:
        return getattr(self.logger, name)

    def __enter__(self) -> "LoggingService":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def start(self) -> "LoggingService":
        with self._lock:
            if self._started:
                return self

            self._configure_python_logging()
            self._configure_cpp_bridge()

            self._started = True
            _state["service"] = self
            _state["configured"] = True
            return self

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return

            self._stop_cpp_bridge()
            self._teardown_python_logging()

            self._started = False
            if _state.get("service") is self:
                _state.update({"service": None, "configured": False})

    def set_level(self, level: int) -> None:
        with self._lock:
            self.config.level = level

            logging.getLogger().setLevel(level)
            self.logger.setLevel(level)

            for handler in self._handlers:
                handler.setLevel(level)

            if self._backend is not None:
                self._backend.set_min_level(level)

    def _teardown_python_logging(self) -> None:
        if self.config.capture_warnings:
            logging.captureWarnings(False)

        root = logging.getLogger()

        for handler in self._handlers:
            try:
                handler.flush()
            except Exception:
                pass
            try:
                root.removeHandler(handler)
            except Exception:
                pass

            try:
                handler.close()
            except Exception:
                pass

        self._handlers = []

    def _configure_python_logging(self) -> None:
        root = logging.getLogger()
        root.setLevel(self.config.level)

        self._handlers = _build_downstream_handlers(self.config)
        for handler in self._handlers:
            root.addHandler(handler)

        self.logger.setLevel(self.config.level)
        self.logger.propagate = True

        if self.config.capture_warnings:
            logging.captureWarnings(True)

    def _configure_cpp_bridge(self) -> None:

        if self.config.use_cpp_bridge:
            self._backend.configure(self.config)
            self._consumer_thread = threading.Thread(
                target=self._run_cpp_consumer,
                name="logging-consumer",
                daemon=True,
            )
            self._consumer_thread.start()
        else:
            self._backend.configure_silent()

    def _stop_cpp_bridge(self) -> None:
        self._stop_event.set()

        if self._backend is not None:
            try:
                self._backend.shutdown()
            except Exception:
                pass

        if self._consumer_thread is not None:
            try:
                self._consumer_thread.join(timeout=2.0)
            except Exception:
                pass
            self._consumer_thread = None

        if self._error is not None:
            error = self._error
            self._error = None
            raise error

        self._stop_event.clear()

    def _emit_file_only_error(
        self, message: str, *, overflow: int, shutdown: int
    ) -> None:
        record = logging.LogRecord(
            name="helios",
            level=logging.ERROR,
            pathname=__file__,
            lineno=0,
            msg=message,
            args=(),
            exc_info=None,
            func="_run_cpp_consumer",
        )
        record._helios_file_only = True
        record.overflow_dropped = overflow
        record.shutdown_dropped = shutdown
        self.logger.handle(record)

    def _report_dropped_counts(self) -> None:
        if self._backend is None:
            return

        dropped = self._backend.consume_dropped_counts()
        overflow = int(getattr(dropped, "overflow", 0) or 0)
        shutdown = int(getattr(dropped, "shutdown", 0) or 0)

        if overflow or shutdown:
            self._emit_file_only_error(
                f"Dropped C++ log messages: overflow={overflow}, shutdown={shutdown}",
                overflow=overflow,
                shutdown=shutdown,
            )

    def _run_cpp_consumer(self) -> None:
        timeout_ms = max(1, int(self.config.poll_interval * 1000))
        batch_size = max(1, int(self.config.batch_size))

        try:
            while not self._stop_event.is_set():
                event = self._backend.wait_pop(timeout_ms)

                if event is None:
                    if self._backend.is_stopped():
                        self._drain_remaining_cpp_events(batch_size)
                        self._report_dropped_counts()
                        break

                    self._report_dropped_counts()
                    continue

                self._emit_cpp_event(event)
                self._drain_remaining_cpp_events(batch_size - 1)
                self._report_dropped_counts()

        except Exception as exc:
            self._error = exc
            self._stop_event.set()
            return

    def _drain_remaining_cpp_events(self, max_items: int) -> None:
        if self._backend is None or max_items <= 0:
            return

        try:
            for event in self._backend.drain(max_items):
                self._emit_cpp_event(event)
        except Exception:
            pass

    def _emit_cpp_event(self, record: logging.LogRecord) -> None:
        logging.getLogger(record.name).handle(record)

    def _install_logger_level_helpers(self) -> None:
        def trace(self_logger: logging.Logger, msg, *args, **kwargs):
            self_logger.log(TRACE_LEVEL_NUM, msg, *args, **kwargs)

        def time_log(self_logger: logging.Logger, msg, *args, **kwargs):
            self_logger.log(TIME_LEVEL_NUM, msg, *args, **kwargs)

        if not hasattr(logging.Logger, "trace"):
            logging.Logger.trace = trace

        if not hasattr(logging.Logger, "time"):
            logging.Logger.time = time_log


_state_lock = threading.RLock()

_state: dict[str, Any] = {
    "configured": False,
    "service": None,
}


def configure_logging(
    config: LoggingConfig | None = None, *, force: bool = False
) -> LoggingService:
    cfg = config or _DEFAULT_LOGGING_CONFIG

    with _state_lock:
        service = _state.get("service")
        if service is not None and not force:
            if config is None or cfg == service.config:
                return service

        if service is not None:
            shutdown_logging()

        service = LoggingService(cfg)
        service.start()
        _state["service"] = service
        _state["configured"] = True
        return service


def get_logger(name: str | None = None) -> logging.Logger:
    service = _state.get("service")
    if isinstance(service, LoggingService):
        base = service.logger
    else:
        base = logging.getLogger("helios")
    return base if not name else base.getChild(name)


def shutdown_logging() -> None:
    with _state_lock:
        service = _state.get("service")
        if isinstance(service, LoggingService):
            try:
                service.stop()
            except Exception:
                pass

        _state["service"] = None
        _state["configured"] = False


def install_default_excepthook(
    logger: logging.Logger | LoggingService | None = None,
) -> None:
    if isinstance(logger, LoggingService):
        logger = logger.logger
    if logger is None:
        logger = get_logger()

    original_hook = sys.excepthook

    def _hook(exc_type, exc_value, exc_traceback):
        if issubclass(exc_type, KeyboardInterrupt):
            original_hook(exc_type, exc_value, exc_traceback)
            return
        logger.exception(
            "Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback)
        )

    sys.excepthook = _hook


def _build_downstream_handlers(cfg: LoggingConfig) -> list[logging.Handler]:
    handlers: list[logging.Handler] = []

    if cfg.stdout:
        stream_handler = JupyterActiveCellHandler(sys.stdout)
        stream_handler.setLevel(cfg.level)

        if cfg.stdout_format == LogFormat.JSON:
            stream_handler.setFormatter(JsonFormatter())
        else:
            stream_handler.setFormatter(
                ColorFormatter(
                    _DEFAULT_FORMAT,
                    datefmt=cfg.datefmt,
                    force_color=cfg.force_color,
                    utc=cfg.stdout_utc,
                )
            )

        stream_handler.addFilter(_ContextFilter())
        stream_handler.addFilter(_SkipFileOnlyFilter())
        handlers.append(stream_handler)

    logfile = cfg.normalized_logfile()
    if logfile is not None:
        logfile.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.handlers.RotatingFileHandler(
            filename=logfile,
            maxBytes=cfg.max_bytes,
            backupCount=cfg.backup_count,
            encoding=cfg.encoding,
            delay=True,
        )
        file_handler.setLevel(cfg.level)

        if cfg.file_format == LogFormat.JSON:
            file_handler.setFormatter(JsonFormatter())
        else:
            file_handler.setFormatter(
                ColorFormatter(
                    _DEFAULT_FORMAT,
                    datefmt=cfg.datefmt,
                    force_color=False,
                    utc=cfg.file_utc,
                )
            )

        file_handler.addFilter(_ContextFilter())
        handlers.append(file_handler)

    if not handlers:
        handlers.append(logging.NullHandler())

    return handlers


_DEFAULT_LOGGING_CONFIG = LoggingConfig(
    level=logging.DEBUG,
    logger_name="helios",
    use_cpp_bridge=True,
    stdout=True,
    logfile=None,
    stdout_format=LogFormat.TEXT,
    file_format=LogFormat.JSON,
    stdout_utc=False,
    file_utc=True,
)
_service = configure_logging(_DEFAULT_LOGGING_CONFIG, force=True)
logger = _service.logger

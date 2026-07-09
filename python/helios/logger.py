import os
import json
import logging
import logging.handlers
import sys
from datetime import datetime, timezone
import time
import threading
import queue
import contextvars
from enum import Enum
from pathlib import Path
from typing import Any, Optional
import dataclasses


__all__ = [
    "LogFormat",
    "LoggingConfig",
    "QueueRecordBridge",
    "clear_logging_context",
    "configure_logging",
    "get_logger",
    "install_default_excepthook",
    "reset_logging_context",
    "set_logging_context",
    "shutdown_logging",
]


_LOG_CONTEXT: contextvars.ContextVar[dict[str, Any]] = contextvars.ContextVar(
    "project_log_context",
    default={},
)


def set_logging_context(**fields: Any) -> contextvars.Token[dict[str, Any]]:
    """Merge fields into the current logging context."""

    current = dict(_LOG_CONTEXT.get())
    current.update(fields)
    return _LOG_CONTEXT.set(current)


def reset_logging_context(token: contextvars.Token[dict[str, Any]]) -> None:
    """Restore the logging context returned by :func:`set_logging_context`."""

    _LOG_CONTEXT.reset(token)


def clear_logging_context() -> None:
    """Remove all contextual fields for the current execution context."""

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


class _UTCFormatter(logging.Formatter):
    converter = staticmethod(time.gmtime)


_DEFAULT_COLORS = {
    "DEBUG": "\033[94m",
    "INFO": "\033[92m",
    "WARNING": "\033[93m",
    "ERROR": "\033[91m",
    "CRITICAL": "\033[41m\033[37m",
}
_DEFAULT_RESET_COLOR = "\033[0m"

_DEFAULT_DATEFMT = "%Y-%m-%d %H:%M:%S"

_DEFAULT_FORMAT = (
    "%(asctime)s | %(levelname)s | %(process)d | %(threadName)s | "
    "%(name)s | %(message)s"
)


class ColorFormatter(logging.Formatter):
    """Colorize text output by log level for terminal stdout."""

    def __init__(
        self,
        fmt: str = _DEFAULT_FORMAT,
        datefmt: str = _DEFAULT_DATEFMT,
        *,
        force_color: bool = False,
    ) -> None:
        super().__init__(fmt=fmt, datefmt=datefmt)
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
        return bool(stream and hasattr(stream, "isatty") and stream.isatty())


class JsonFormatter(logging.Formatter):
    """Structured JSON formatter for machine-oriented log ingestion."""

    def __init__(self, *, utc: bool = False, datefmt: str = _DEFAULT_DATEFMT) -> None:
        super().__init__(datefmt=datefmt)
        self._utc = utc

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "timestamp": self._format_time(record),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
            "module": record.module,
            "process": record.process,
            "function": record.funcName,
            "thread": record.threadName,
            "line": record.lineno,
        }

        for key, value in vars(record).items():
            if key.startswith("_"):
                continue
            if key in payload:
                continue
            if key in {
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
            }:
                continue
            try:
                json.dumps(value)
                payload[key] = value
            except TypeError:
                payload[key] = repr(value)

        if record.exc_info:
            payload["exception"] = self.formatException(record.exc_info)
        if record.stack_info:
            payload["stack"] = record.stack_info

        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

    def _format_time(self, record: logging.LogRecord) -> str:
        dt = datetime.fromtimestamp(
            record.created, tz=timezone.utc if self._utc else None
        )
        return dt.isoformat(timespec="milliseconds")


@dataclasses.dataclass(slots=True)
class LoggingConfig:
    level: int = logging.INFO
    logger_name: str = "helios"
    use_queue: bool = True
    propagate: bool = False
    stdout: bool = True
    logfile: Optional[Path] = None
    max_bytes: int = 25 * 1024 * 1024
    backup_count: int = 5
    encoding: str = "utf-8"
    format: LogFormat = LogFormat.TEXT
    datefmt: str = _DEFAULT_DATEFMT
    utc: bool = False
    capture_warnings: bool = True
    poll_interval: float = 0.1
    force_color: bool = False

    def normalized_logfile(self) -> Optional[Path]:
        if self.logfile is None:
            return None
        return Path(self.logfile).expanduser().resolve()


_state_lock = threading.RLock()
_state: dict[str, Any] = {
    "configured": False,
    "queue": None,
    "listener": None,
    "logger": None,
    "handlers": [],
    "root_handlers": [],
    "config": None,
}


class _QueueConsumer:  # TODO this class should be changed after implementing the C++ bridge for logging queue
    def __init__(
        self,
        queue: "queue.Queue[logging.LogRecord]",
        handlers: list[logging.Handler],
        poll_interval: float = 0.1,
    ) -> None:
        self._queue = queue
        self._handlers = handlers
        self._poll_interval = poll_interval
        self._stop_event = threading.Event()
        self._error: list[BaseException] = []
        self._worker = threading.Thread(
            target=self._run, name="logging-consumer", daemon=True
        )

    def start(self) -> None:
        self._worker.start()

    def stop(self) -> None:
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass
        self._worker.join()

    def join(self, timeout: Optional[float] = None) -> None:
        self._worker.join(timeout)

    @property
    def error(self) -> BaseException | None:
        return self._error[0] if self._error else None

    def _run(self) -> None:
        try:
            while True:
                try:
                    record = self._queue.get(timeout=self._poll_interval)
                except queue.Empty:
                    continue
                if record is None:
                    break
                self._dispatch(record)
        finally:
            self._flush_handlers()

    def _dispatch(self, record: logging.LogRecord) -> None:
        for handler in self._handlers:
            if record.levelno < handler.level:
                continue
            if not handler.filter(record):
                continue
            try:
                handler.handle(record)
            except Exception:
                self._report_handler_error(record)

    def _flush_handlers(self) -> None:
        for handler in self._handlers:
            try:
                handler.flush()
            except Exception as e:
                self._error.append(e)

    def _report_handler_error(self, record: logging.LogRecord) -> None:
        try:
            sys.stderr.write(f"Logging handler failed for {record.name}")
        except Exception:
            pass


def configure_logging(
    config: LoggingConfig | None = None, *, force: bool = False
) -> logging.Logger:
    """Configure the logging system based on the provided configuration."""
    cfg = config or LoggingConfig()

    with _state_lock:
        if _state["configured"] and not force:
            logger = _state["logger"]
            assert isinstance(logger, logging.Logger)
            return logger

        shutdown_logging()

        root = logging.getLogger()
        root.setLevel(cfg.level)
        root.handlers.clear()
        root.filters.clear()
        root.addFilter(_ContextFilter())

        project_logger = logging.getLogger(cfg.logger_name)
        project_logger.setLevel(cfg.level)
        project_logger.propagate = True
        project_logger.handlers.clear()
        project_logger.filters.clear()

        handlers = _build_downstream_handlers(cfg)
        _state["handlers"] = handlers

        if cfg.use_queue:
            q: "queue.Queue[logging.LogRecord | None]" = queue.Queue(-1)
            queue_handler = logging.handlers.QueueHandler(q)
            queue_handler.setLevel(cfg.level)
            queue_handler.addFilter(_ContextFilter())
            root.addHandler(queue_handler)
            _state["root_handlers"] = [queue_handler]

            q_consumer = _QueueConsumer(q, handlers, cfg.poll_interval)
            q_consumer.start()
            _state["queue"] = q
            _state["listener"] = q_consumer
        else:
            for handler in handlers:
                root.addHandler(handler)
            _state["root_handlers"] = list(handlers)

        if cfg.capture_warnings:
            logging.captureWarnings(True)

        _state["logger"] = project_logger
        _state["configured"] = True
        _state["config"] = cfg
        return project_logger


def get_logger(name: str | None = None) -> logging.Logger:
    """Get the project logger or a child logger."""

    base_name = _state["logger"].name if _state["logger"] else "laser_scan"
    base = logging.getLogger(base_name)
    return base if not name else base.getChild(name)


def shutdown_logging() -> None:
    """Shutdown the logging system and clean up resources."""
    with _state_lock:
        if not _state["configured"]:
            return

        listener = _state.get("listener")
        if listener is not None:
            try:
                listener.stop()
            except Exception:
                pass

        root = logging.getLogger()
        for handler in list(_state.get("root_handlers") or []):
            try:
                root.removeHandler(handler)
            except Exception:
                pass
            try:
                handler.flush()
            except Exception:
                pass
            try:
                handler.close()
            except Exception:
                pass

        for handler in list(_state.get("handlers") or []):
            try:
                handler.flush()
            except Exception:
                pass
            try:
                handler.close()
            except Exception:
                pass

        logging.captureWarnings(False)

        _state.update(
            {
                "configured": False,
                "queue": None,
                "listener": None,
                "logger": None,
                "handlers": [],
                "root_handlers": [],
                "config": None,
            }
        )


def _build_downstream_handlers(cfg: LoggingConfig) -> list[logging.Handler]:
    handlers: list[logging.Handler] = []

    if cfg.stdout:
        stream_handler = logging.StreamHandler(stream=sys.stdout)
        stream_handler.setLevel(cfg.level)
        if cfg.format == LogFormat.JSON:
            stream_handler.setFormatter(JsonFormatter(utc=cfg.utc, datefmt=cfg.datefmt))
        else:
            stream_handler.setFormatter(
                ColorFormatter(
                    _DEFAULT_FORMAT, datefmt=cfg.datefmt, force_color=cfg.force_color
                )
            )
        stream_handler.addFilter(_ContextFilter())
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
        if cfg.format == LogFormat.JSON:
            file_handler.setFormatter(JsonFormatter(utc=cfg.utc, datefmt=cfg.datefmt))
        else:
            if cfg.utc:
                file_handler.setFormatter(
                    _UTCFormatter(_DEFAULT_FORMAT, datefmt=cfg.datefmt)
                )
            else:
                file_handler.setFormatter(
                    logging.Formatter(_DEFAULT_FORMAT, datefmt=cfg.datefmt)
                )
        file_handler.addFilter(_ContextFilter())
        handlers.append(file_handler)

    if not handlers:
        handlers.append(logging.NullHandler())

    return handlers


def install_default_excepthook(logger_name: str | None = None) -> None:
    """Install a default exception hook that logs uncaught exceptions."""
    logger = logging.getLogger(
        logger_name or (_state["logger"].name if _state["logger"] else "laser_scan")
    )
    original_hook = sys.excepthook

    def _hook(exc_type, exc_value, exc_traceback):
        if issubclass(exc_type, KeyboardInterrupt):
            original_hook(exc_type, exc_value, exc_traceback)
            return
        logger.exception(
            "Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback)
        )

    sys.excepthook = _hook

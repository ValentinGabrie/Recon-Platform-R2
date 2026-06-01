"""recon_webui.logging_config — Centralized logging configuration for the project.

This module provides:
- Structured logging with consistent formatting across all components
- Dual output: console (INFO+) and file (DEBUG+)
- Component-based log filtering
- Structured JSON-like format for log aggregation tools
"""

import logging
import logging.handlers
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

# Logging handlers are acquired from BOTH the eventlet hub (Flask emit loop,
# request handlers) AND the real OS thread that runs rclpy.spin() in
# RosBridge. After eventlet.monkey_patch(), a handler's default
# threading.RLock is a greenlet semaphore — acquiring it from the real rclpy
# thread deadlocks the hub ("N RLock(s) were not greened" / silent stall on
# the first cross-thread log). Same hazard DataChannel already dodges. Pull
# the UN-greened threading module so handler locks are native and safe from
# any thread; fall back to stdlib when eventlet isn't present (pytest).
try:
    import eventlet.patcher as _evp  # type: ignore
    _native_threading = _evp.original("threading")
except ImportError:
    import threading as _native_threading


class ComponentFilter(logging.Filter):
    """Add component name to log records."""

    def filter(self, record: logging.LogRecord) -> bool:
        record.component = record.name.split(".")[-1] if "." in record.name else record.name
        return True


class StructuredFormatter(logging.Formatter):
    """Structured log formatter with ISO8601 timestamps and context."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record with structured output."""
        timestamp = datetime.utcnow().isoformat(timespec="milliseconds") + "Z"
        component = getattr(record, "component", record.name)
        level = record.levelname

        # Build context fields
        context = []
        if record.funcName and record.funcName != "<module>":
            context.append(f"func={record.funcName}")
        if record.lineno:
            context.append(f"line={record.lineno}")
        context_str = f" [{', '.join(context)}]" if context else ""

        return (
            f"{timestamp} {level:8s} [{component:20s}] "
            f"{record.getMessage()}{context_str}"
        )


def setup_logging(
    log_dir: Optional[str] = None,
    level: int = logging.DEBUG,
    console_level: int = logging.INFO,
) -> Path:
    """Configure logging for the entire Recon-Platform-R2 project.

    Args:
        log_dir: Directory for log files. Defaults to /tmp/recon_logs/
        level: Logging level for file output (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        console_level: Logging level for console output

    Returns:
        Path to the log directory

    Example:
        >>> log_dir = setup_logging()
        >>> logger = logging.getLogger(__name__)
        >>> logger.info("Application started")
    """
    if log_dir is None:
        log_dir = "/tmp/recon_logs"

    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)

    # Create filename with date
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = log_path / f"recon_{timestamp}.log"

    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    # Remove any existing handlers to avoid duplicates
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)

    # Console handler (higher level, user-facing)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(console_level)
    console_formatter = StructuredFormatter()
    console_handler.setFormatter(console_formatter)
    console_filter = ComponentFilter()
    console_handler.addFilter(console_filter)
    # Native (un-greened) lock — see module header.
    console_handler.lock = _native_threading.RLock()

    # File handler (debug level, captures everything)
    try:
        file_handler = logging.handlers.RotatingFileHandler(
            log_file,
            maxBytes=10 * 1024 * 1024,  # 10 MB
            backupCount=5,  # Keep 5 rotated backups
        )
        file_handler.setLevel(level)
        file_formatter = StructuredFormatter()
        file_handler.setFormatter(file_formatter)
        file_filter = ComponentFilter()
        file_handler.addFilter(file_filter)
        # Native (un-greened) lock — see module header.
        file_handler.lock = _native_threading.RLock()
        root_logger.addHandler(file_handler)
    except (IOError, OSError) as e:
        console_handler.emit(
            logging.LogRecord(
                name="logging_config",
                level=logging.WARNING,
                pathname="",
                lineno=0,
                msg=f"Could not create file handler at {log_file}: {e}",
                args=(),
                exc_info=None,
            )
        )

    root_logger.addHandler(console_handler)

    # Log startup message
    logger = logging.getLogger("recon_webui.logging_config")
    logger.info("Logging initialized \u2014 file: %s", log_file)

    # Suppress noisy third-party loggers
    logging.getLogger("urllib3").setLevel(logging.WARNING)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    logging.getLogger("eventlet").setLevel(logging.WARNING)

    return log_path


def get_logger(name: str) -> logging.Logger:
    """Get a logger instance for a specific module.

    Args:
        name: Module name (typically __name__)

    Returns:
        Configured logger instance

    Example:
        >>> logger = get_logger(__name__)
        >>> logger.debug("Debug message")
        >>> logger.info("Info message")
        >>> logger.warning("Warning message")
        >>> logger.error("Error message", extra={"user_id": 123})
    """
    return logging.getLogger(name)

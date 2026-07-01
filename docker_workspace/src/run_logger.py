"""
Merkezi Run Logger - CELEBILER USV Observability Katmani
========================================================
Her calisma icin benzersiz run_id uretir, yapilandirilmis JSONL event logging saglar,
log rotasyonu ile disk korumasi yapar, startup/shutdown snapshot uretir ve
mevcut runtime_debug_log.py ile yan yana (onun yerini almadan) calisir.

Kullanim:
    from run_logger import RunLogger, get_run_logger
    rl = RunLogger(component="usv_main")
    rl.info("mission_start", mission_id="M001", waypoints=12)
    rl.shutdown(reason="operator_stop")

NOT: Bu modul runtime_debug_log.py'yi DEGISTIRMEZ; onu genisletir.
"""
from __future__ import annotations

import hashlib
import json
import logging
import os
import platform
import random
import shutil
import socket
import string
import subprocess
import sys
import threading
import time
from collections import defaultdict, deque
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Dict, List, Optional

# ---------------------------------------------------------------------------
# compliance_profile sabitlerini import et
# ---------------------------------------------------------------------------
try:
    from compliance_profile import (
        CONTROL_DIR,
        LOG_DIR,
        USV_MODE,
        USV_MODE_RACE,
        USV_MODE_TEST,
    )
except ImportError:
    CONTROL_DIR = os.environ.get("CONTROL_DIR", "")
    LOG_DIR = os.environ.get("LOG_DIR", "")
    USV_MODE = os.environ.get("USV_MODE", "test")
    USV_MODE_RACE = "race"
    USV_MODE_TEST = "test"

# ---------------------------------------------------------------------------
# Sabitler
# ---------------------------------------------------------------------------
_RUN_LOGGER_VERSION = "1.0.0"
_DEFAULT_MAX_BYTES = int(os.environ.get("USV_RUN_LOG_MAX_BYTES", str(50 * 1024 * 1024)))
_DEFAULT_BACKUP_COUNT = int(os.environ.get("USV_RUN_LOG_BACKUPS", "10"))
_DISK_WARN_THRESHOLD_PCT = float(os.environ.get("USV_DISK_WARN_PCT", "85"))
_DISK_ERROR_THRESHOLD_PCT = float(os.environ.get("USV_DISK_ERROR_PCT", "95"))
_RATE_LIMIT_WINDOW_S = float(os.environ.get("USV_RATE_LIMIT_WINDOW_S", "1.0"))
_RATE_LIMIT_MAX_PER_WINDOW = int(os.environ.get("USV_RATE_LIMIT_MAX", "200"))

_GIT_SHA: Optional[str] = None
_GIT_BRANCH: Optional[str] = None
_GIT_DIRTY: Optional[bool] = None

_global_run_logger: Optional["RunLogger"] = None
_global_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Helper Functions
# ---------------------------------------------------------------------------
def _generate_run_id() -> str:
    """Generate a unique run ID with timestamp and random suffix."""
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    rand = ''.join(random.choices(string.ascii_lowercase + string.digits, k=6))
    return f"{ts}-{rand}"


def _get_git_info() -> tuple[Optional[str], Optional[str], Optional[bool]]:
    """Get git SHA, branch, and dirty status."""
    try:
        sha = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL,
            timeout=2
        ).decode().strip()
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            stderr=subprocess.DEVNULL,
            timeout=2
        ).decode().strip()
        dirty = bool(subprocess.check_output(
            ["git", "status", "--porcelain"],
            stderr=subprocess.DEVNULL,
            timeout=2
        ).decode().strip())
        return sha, branch, dirty
    except Exception:
        return None, None, None


def _get_resource_usage() -> Dict[str, Any]:
    """Get current process resource usage."""
    try:
        import resource
        usage = resource.getrusage(resource.RUSAGE_SELF)
        cpu_user = usage.ru_utime
        cpu_sys = usage.ru_stime
        rss_kb = usage.ru_maxrss
    except Exception:
        cpu_user = cpu_sys = 0.0
        rss_kb = 0

    try:
        with open("/proc/meminfo") as f:
            meminfo = {}
            for line in f:
                parts = line.split()
                if len(parts) >= 2:
                    meminfo[parts[0].rstrip(':')] = int(parts[1])
            total_kb = meminfo.get('MemTotal', 0)
            avail_kb = meminfo.get('MemAvailable', 0)
    except Exception:
        total_kb = avail_kb = 0

    try:
        disk = shutil.disk_usage('/')
        disk_total = disk.total
        disk_used = disk.used
        disk_free = disk.free
    except Exception:
        disk_total = disk_used = disk_free = 0

    try:
        pid = os.getpid()
        thread_count = threading.active_count()
        fd_count = len(os.listdir(f'/proc/{pid}/fd'))
    except Exception:
        pid = os.getpid()
        thread_count = threading.active_count()
        fd_count = 0

    return {
        "cpu_user_s": round(cpu_user, 3),
        "cpu_sys_s": round(cpu_sys, 3),
        "rss_kb": rss_kb,
        "mem_total_kb": total_kb,
        "mem_avail_kb": avail_kb,
        "disk_total_b": disk_total,
        "disk_used_b": disk_used,
        "disk_free_b": disk_free,
        "pid": pid,
        "thread_count": thread_count,
        "fd_count": fd_count,
    }



class RunLogger:
    """
    Merkezi run logger: JSONL event logging, metrics, resource sampling.
    Her component icin ayri log dosyasi olusturur.
    """

    def __init__(
        self,
        component: str,
        run_id: Optional[str] = None,
        log_dir: Optional[str] = None,
        mode: Optional[str] = None,
        mission_state: str = "idle",
        parkur: str = "unknown",
    ):
        self.component = component
        self.run_id = run_id or _generate_run_id()
        self.log_dir = log_dir or LOG_DIR or "logs"
        self.mode = mode or USV_MODE
        self.mission_state = mission_state
        self.parkur = parkur

        self._lock = threading.Lock()
        self._degraded = False
        self._start_time = time.time()
        self._start_monotonic = time.monotonic()
        self._event_count = 0
        self._rate_limiter = defaultdict(lambda: deque(maxlen=_RATE_LIMIT_MAX_PER_WINDOW))

        # Trace mode
        self._trace_enabled = os.environ.get("USV_LOG_TRACE", "0").strip().lower() in ("1", "true", "yes")

        # Create log directory
        Path(self.log_dir).mkdir(parents=True, exist_ok=True)

        # Setup JSONL logger
        self._jsonl_path = os.path.join(self.log_dir, f"{component}.jsonl")
        self._jsonl_handler = RotatingFileHandler(
            self._jsonl_path,
            maxBytes=_DEFAULT_MAX_BYTES,
            backupCount=_DEFAULT_BACKUP_COUNT,
        )
        self._jsonl_handler.setFormatter(logging.Formatter('%(message)s'))
        self._jsonl_logger = logging.getLogger(f"run_logger.{component}")
        self._jsonl_logger.setLevel(logging.INFO)
        self._jsonl_logger.addHandler(self._jsonl_handler)
        self._jsonl_logger.propagate = False

        # Setup metrics logger
        self._metrics_path = os.path.join(self.log_dir, f"{component}.metrics.jsonl")
        self._metrics_handler = RotatingFileHandler(
            self._metrics_path,
            maxBytes=_DEFAULT_MAX_BYTES,
            backupCount=_DEFAULT_BACKUP_COUNT,
        )
        self._metrics_handler.setFormatter(logging.Formatter('%(message)s'))
        self._metrics_logger = logging.getLogger(f"run_logger.{component}.metrics")
        self._metrics_logger.setLevel(logging.INFO)
        self._metrics_logger.addHandler(self._metrics_handler)
        self._metrics_logger.propagate = False

        # Resource sampling thread
        self._resource_thread: Optional[threading.Thread] = None
        self._resource_stop_event = threading.Event()
        self._start_resource_sampling()

        # Log startup
        self.info("run_logger_init", version=_RUN_LOGGER_VERSION, run_id=self.run_id)


    def _check_disk_space(self) -> bool:
        """Check disk space and return True if OK."""
        try:
            disk = shutil.disk_usage(self.log_dir)
            used_pct = (disk.used / disk.total) * 100 if disk.total > 0 else 0
            if used_pct >= _DISK_ERROR_THRESHOLD_PCT:
                self._log_event("disk_space_critical", level="ERROR", used_pct=used_pct)
                return False
            elif used_pct >= _DISK_WARN_THRESHOLD_PCT:
                self._log_event("disk_space_warning", level="WARN", used_pct=used_pct)
        except Exception:
            pass
        return True

    def _check_rate_limit(self, event: str) -> bool:
        """Check if event should be rate-limited. Returns True if allowed."""
        # Import security-critical events check
        try:
            from common.event_schema import is_security_critical
            if is_security_critical(event):
                return True
        except ImportError:
            pass
        
        now = time.time()
        window = self._rate_limiter[event]

        # Remove old entries
        while window and (now - window[0]) > _RATE_LIMIT_WINDOW_S:
            window.popleft()

        # Check limit
        if len(window) >= _RATE_LIMIT_MAX_PER_WINDOW:
            return False

        window.append(now)
        return True

    def _log_event(self, event: str, level: str = "INFO", **data) -> None:
        """Internal: write a single JSONL event."""
        if not self._check_disk_space():
            if not self._degraded:
                self._degraded = True
                # Try to log degradation, but don't recurse
                try:
                    entry = self._build_event_dict("logging_degraded", "ERROR", reason="disk_full")
                    self._jsonl_logger.info(json.dumps(entry, ensure_ascii=False))
                except Exception:
                    pass
            return

        if not self._check_rate_limit(event):
            return

        try:
            entry = self._build_event_dict(event, level, **data)
            self._jsonl_logger.info(json.dumps(entry, ensure_ascii=False))
            self._event_count += 1
        except Exception as e:
            if not self._degraded:
                self._degraded = True
                try:
                    entry = self._build_event_dict("logging_degraded", "ERROR", error=str(e))
                    self._jsonl_logger.info(json.dumps(entry, ensure_ascii=False))
                except Exception:
                    pass

    def _build_event_dict(self, event: str, level: str, **data) -> Dict[str, Any]:
        """Build a complete event dictionary."""
        now = time.time()
        monotonic_ms = int((time.monotonic() - self._start_monotonic) * 1000)

        entry = {
            "ts": datetime.fromtimestamp(now, tz=timezone.utc).isoformat(),
            "monotonic_ms": monotonic_ms,
            "run_id": self.run_id,
            "component": self.component,
            "level": level,
            "event": event,
            "mode": self.mode,
            "mission_state": self.mission_state,
            "parkur": self.parkur,
            "message": data.pop("message", ""),
        }
        if data:
            entry["data"] = data
        return entry


    def _start_resource_sampling(self) -> None:
        """Start periodic resource sampling thread."""
        interval = float(os.environ.get("USV_RESOURCE_SAMPLE_INTERVAL_S", "5.0"))

        def sampler():
            while not self._resource_stop_event.wait(interval):
                try:
                    usage = _get_resource_usage()
                    usage["uptime_s"] = round(time.time() - self._start_time, 1)
                    usage["event_count"] = self._event_count
                    entry = self._build_event_dict("resource_usage_sample", "DEBUG", **usage)
                    self._metrics_logger.info(json.dumps(entry, ensure_ascii=False))
                except Exception:
                    pass

        self._resource_thread = threading.Thread(target=sampler, daemon=True, name=f"resource-sampler-{self.component}")
        self._resource_thread.start()

    def update_state(self, mission_state: Optional[str] = None, parkur: Optional[str] = None) -> None:
        """Update mission state and/or parkur."""
        if mission_state is not None:
            old_state = self.mission_state
            self.mission_state = mission_state
            if old_state != mission_state:
                self.info("mission_state_changed", old_state=old_state, new_state=mission_state)
        if parkur is not None:
            old_parkur = self.parkur
            self.parkur = parkur
            if old_parkur != parkur:
                self.info("parkur_changed", old_parkur=old_parkur, new_parkur=parkur)

    def debug(self, event: str, **data) -> None:
        """Log DEBUG level event."""
        self._log_event(event, level="DEBUG", **data)

    def info(self, event: str, **data) -> None:
        """Log INFO level event."""
        self._log_event(event, level="INFO", **data)

    def warn(self, event: str, **data) -> None:
        """Log WARN level event."""
        self._log_event(event, level="WARN", **data)

    def error(self, event: str, **data) -> None:
        """Log ERROR level event."""
        self._log_event(event, level="ERROR", **data)

    def trace_enter(self, func_name: str, **kwargs) -> float:
        """Log function entry in trace mode. Returns start time."""
        if not self._trace_enabled:
            return 0.0
        start = time.monotonic()
        self.debug("trace_enter", func=func_name, **kwargs)
        return start

    def trace_exit(self, func_name: str, start_time: float, **kwargs) -> None:
        """Log function exit in trace mode with duration."""
        if not self._trace_enabled or start_time == 0.0:
            return
        duration_ms = (time.monotonic() - start_time) * 1000
        self.debug("trace_exit", func=func_name, duration_ms=round(duration_ms, 2), **kwargs)

    def shutdown(self, reason: str = "normal", **data) -> None:
        """Log shutdown and cleanup resources."""
        self.info("run_logger_shutdown", reason=reason, **data)
        self._resource_stop_event.set()
        if self._resource_thread and self._resource_thread.is_alive():
            self._resource_thread.join(timeout=2.0)
        self._jsonl_handler.close()
        self._metrics_handler.close()

    def get_stats(self) -> Dict[str, Any]:
        """Get logger statistics."""
        return {
            "run_id": self.run_id,
            "component": self.component,
            "event_count": self._event_count,
            "uptime_s": round(time.time() - self._start_time, 1),
            "degraded": self._degraded,
            "trace_enabled": self._trace_enabled,
        }


def get_run_logger(
    component: str,
    run_id: Optional[str] = None,
    log_dir: Optional[str] = None,
    mode: Optional[str] = None,
) -> RunLogger:
    """Get or create a global RunLogger instance for a component."""
    global _global_run_logger
    with _global_lock:
        if _global_run_logger is None:
            _global_run_logger = RunLogger(
                component=component,
                run_id=run_id,
                log_dir=log_dir,
                mode=mode,
            )
        return _global_run_logger


def reset_run_logger() -> None:
    """Reset global run logger (for testing)."""
    global _global_run_logger
    with _global_lock:
        if _global_run_logger is not None:
            _global_run_logger.shutdown(reason="reset")
        _global_run_logger = None

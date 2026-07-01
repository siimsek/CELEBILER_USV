"""
Logging utilities for CELEBILER USV structured logging.

Provides rate limiting, safe logging wrappers, and helper functions.
"""
from __future__ import annotations

import hashlib
import logging
import os
import time
from collections import defaultdict, deque
from typing import Any, Dict, Optional

from .event_schema import SECURITY_CRITICAL_EVENTS, is_security_critical


# ---------------------------------------------------------------------------
# Rate Limiter
# ---------------------------------------------------------------------------
class RateLimiter:
    """
    Sliding window rate limiter for log events.
    
    Prevents log spam by limiting the number of events per time window.
    Security-critical events bypass rate limiting.
    """
    
    def __init__(self, window_s: float = 1.0, max_per_window: int = 200):
        """
        Initialize rate limiter.
        
        Args:
            window_s: Time window in seconds
            max_per_window: Maximum events allowed per window
        """
        self.window_s = window_s
        self.max_per_window = max_per_window
        self._events: Dict[str, deque] = defaultdict(lambda: deque(maxlen=max_per_window))
    
    def should_log(self, event_name: str) -> bool:
        """
        Check if an event should be logged.
        
        Args:
            event_name: Name of the event
            
        Returns:
            True if event should be logged, False if rate-limited
        """
        # Security-critical events bypass rate limiting
        if is_security_critical(event_name):
            return True
        
        now = time.time()
        window = self._events[event_name]
        
        # Remove old entries outside the window
        while window and (now - window[0]) > self.window_s:
            window.popleft()
        
        # Check if we're at the limit
        if len(window) >= self.max_per_window:
            return False
        
        # Record this event
        window.append(now)
        return True
    
    def reset(self, event_name: Optional[str] = None) -> None:
        """
        Reset rate limiter for a specific event or all events.
        
        Args:
            event_name: Event to reset, or None to reset all
        """
        if event_name is None:
            self._events.clear()
        else:
            self._events.pop(event_name, None)


# ---------------------------------------------------------------------------
# Safe Logging Wrapper
# ---------------------------------------------------------------------------
def safe_log(
    logger: logging.Logger,
    level: str,
    event: str,
    rate_limiter: Optional[RateLimiter] = None,
    **data: Any,
) -> None:
    """
    Safely log an event with rate limiting and exception handling.
    
    Args:
        logger: Python logger instance
        level: Log level (DEBUG, INFO, WARN, ERROR)
        event: Event name
        rate_limiter: Optional rate limiter instance
        **data: Additional event data
    """
    try:
        # Check rate limit
        if rate_limiter and not rate_limiter.should_log(event):
            return
        
        # Map level string to logging level
        level_map = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARN": logging.WARNING,
            "ERROR": logging.ERROR,
        }
        log_level = level_map.get(level.upper(), logging.INFO)
        
        # Build message
        message = data.get("message", "")
        extra_data = {k: v for k, v in data.items() if k != "message"}
        
        if extra_data:
            extra_str = " ".join(f"{k}={v}" for k, v in extra_data.items())
            full_message = f"{message} {extra_str}".strip()
        else:
            full_message = message
        
        # Log the event
        logger.log(log_level, f"[{event}] {full_message}")
        
    except Exception as e:
        # Never let logging errors crash the application
        try:
            logger.error(f"Logging error for event {event}: {e}")
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Mode-aware Log Level
# ---------------------------------------------------------------------------
def mode_log_level(mode: str) -> int:
    """
    Get the appropriate log level for a given mode.
    
    Args:
        mode: Operating mode (test, sim, race)
        
    Returns:
        Python logging level constant
    """
    mode_lower = mode.lower()
    
    if mode_lower in ("test", "sim"):
        # Test and sim modes: verbose logging
        return logging.DEBUG
    elif mode_lower == "race":
        # Race mode: INFO for most, but keep critical DEBUG events
        return logging.INFO
    else:
        # Default to INFO
        return logging.INFO


# ---------------------------------------------------------------------------
# Run ID Management
# ---------------------------------------------------------------------------
_current_run_id: Optional[str] = None


def get_run_id() -> str:
    """
    Get the current run ID, or generate a new one if none exists.
    
    Returns:
        Current run ID string
    """
    global _current_run_id
    
    if _current_run_id is None:
        # Generate a new run ID
        import random
        import string
        from datetime import datetime, timezone
        
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        rand = ''.join(random.choices(string.ascii_lowercase + string.digits, k=6))
        _current_run_id = f"{ts}-{rand}"
    
    return _current_run_id


def set_run_id(run_id: str) -> None:
    """
    Set the current run ID.
    
    Args:
        run_id: Run ID to set
    """
    global _current_run_id
    _current_run_id = run_id


def reset_run_id() -> None:
    """Reset the current run ID (for testing)."""
    global _current_run_id
    _current_run_id = None


# ---------------------------------------------------------------------------
# File Hashing
# ---------------------------------------------------------------------------
def compute_file_hash(path: str, algorithm: str = "sha256") -> Optional[str]:
    """
    Compute hash of a file.
    
    Args:
        path: Path to the file
        algorithm: Hash algorithm (sha256, md5, sha1)
        
    Returns:
        Hex digest string, or None if file doesn't exist
    """
    try:
        if not os.path.exists(path):
            return None
        
        hash_func = hashlib.new(algorithm)
        
        with open(path, "rb") as f:
            # Read in chunks to handle large files
            while chunk := f.read(8192):
                hash_func.update(chunk)
        
        return hash_func.hexdigest()
        
    except Exception:
        return None

"""
Common utilities for CELEBILER USV observability and logging.
"""
from .event_schema import (
    validate_event,
    EventLevel,
    # Event name constants
    EVT_SYSTEM_BOOT_BEGIN,
    EVT_RUN_ID_CREATED,
    EVT_SERVICE_INIT_BEGIN,
    EVT_SERVICE_INIT_COMPLETE,
    EVT_MISSION_START_REQUESTED,
    EVT_MISSION_STARTED,
    EVT_MISSION_COMPLETED,
    EVT_ESTOP_TRIGGERED,
    EVT_ESTOP_CLEARED,
    EVT_RESOURCE_USAGE_SAMPLE,
    EVT_LOGGING_DEGRADED,
)
from .logging_utils import (
    RateLimiter,
    safe_log,
    mode_log_level,
    get_run_id,
    compute_file_hash,
)

__all__ = [
    "validate_event",
    "EventLevel",
    "RateLimiter",
    "safe_log",
    "mode_log_level",
    "get_run_id",
    "compute_file_hash",
]

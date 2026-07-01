"""
Event schema definitions for CELEBILER USV structured logging.

All event names, required fields, and validation logic.
"""
from __future__ import annotations

from enum import Enum
from typing import Any, Dict, List, Optional


class EventLevel(str, Enum):
    """Event severity levels."""
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"


# Required fields in every event
REQUIRED_FIELDS = [
    "ts",
    "monotonic_ms",
    "run_id",
    "component",
    "level",
    "event",
    "mode",
    "mission_state",
    "parkur",
    "message",
]


# ---------------------------------------------------------------------------
# System Boot Events
# ---------------------------------------------------------------------------
EVT_SYSTEM_BOOT_BEGIN = "system_boot_begin"
EVT_RUN_ID_CREATED = "run_id_created"
EVT_GIT_SNAPSHOT = "git_snapshot"
EVT_ENV_SNAPSHOT = "env_snapshot"
EVT_CONFIG_FILES_LOADED = "config_files_loaded"
EVT_CONFIG_HASHES = "config_hashes"
EVT_MODE_SELECTED = "mode_selected"
EVT_RACE_MODE_HARDENING_ENABLED = "race_mode_hardening_enabled"
EVT_DOCKER_CONTAINER_STATUS = "docker_container_status"
EVT_HOST_PREFLIGHT_BEGIN = "host_preflight_begin"
EVT_HOST_PREFLIGHT_RESULT = "host_preflight_result"
EVT_SERIAL_DEVICES_DETECTED = "serial_devices_detected"
EVT_USB_DEVICES_DETECTED = "usb_devices_detected"
EVT_NETWORK_INTERFACES_DETECTED = "network_interfaces_detected"
EVT_FORBIDDEN_NETWORK_CHECK_RESULT = "forbidden_network_check_result"
EVT_WIFI_DISABLED_CHECK_RESULT = "wifi_disabled_check_result"
EVT_PROCESS_START_REQUESTED = "process_start_requested"
EVT_PROCESS_STARTED = "process_started"
EVT_PROCESS_START_FAILED = "process_start_failed"
EVT_PORT_BIND_ATTEMPT = "port_bind_attempt"
EVT_PORT_BIND_SUCCESS = "port_bind_success"
EVT_PORT_BIND_FAILURE = "port_bind_failure"
EVT_DEPENDENCY_READY = "dependency_ready"
EVT_DEPENDENCY_TIMEOUT = "dependency_timeout"
EVT_STARTUP_READINESS_GATE_RESULT = "startup_readiness_gate_result"


# ---------------------------------------------------------------------------
# Service Lifecycle Events
# ---------------------------------------------------------------------------
EVT_SERVICE_INIT_BEGIN = "service_init_begin"
EVT_SERVICE_INIT_COMPLETE = "service_init_complete"
EVT_SERVICE_CONFIG_LOADED = "service_config_loaded"
EVT_SERVICE_DEPENDENCY_WAIT_BEGIN = "service_dependency_wait_begin"
EVT_SERVICE_DEPENDENCY_READY = "service_dependency_ready"
EVT_SERVICE_HEARTBEAT = "service_heartbeat"
EVT_SERVICE_LOOP_TICK_SLOW = "service_loop_tick_slow"
EVT_SERVICE_EXCEPTION = "service_exception"
EVT_SERVICE_RECOVERABLE_ERROR = "service_recoverable_error"
EVT_SERVICE_FATAL_ERROR = "service_fatal_error"
EVT_SERVICE_SHUTDOWN_REQUESTED = "service_shutdown_requested"
EVT_SERVICE_SHUTDOWN_COMPLETE = "service_shutdown_complete"
EVT_SIGNAL_RECEIVED = "signal_received"
EVT_RESOURCE_USAGE_SAMPLE = "resource_usage_sample"


# ---------------------------------------------------------------------------
# Mission Events
# ---------------------------------------------------------------------------
EVT_MISSION_FILE_DETECTED = "mission_file_detected"
EVT_MISSION_FILE_HASH = "mission_file_hash"
EVT_MISSION_PARSE_BEGIN = "mission_parse_begin"
EVT_MISSION_PARSE_SUCCESS = "mission_parse_success"
EVT_MISSION_PARSE_ERROR = "mission_parse_error"
EVT_MISSION_SCHEMA_VERSION = "mission_schema_version"
EVT_MISSION_PROFILE_LOADED = "mission_profile_loaded"
EVT_MISSION_PROFILE_LOCKED = "mission_profile_locked"
EVT_TARGET_COLOR_LOCKED = "target_color_locked"
EVT_MISSION_UPLOAD_SOURCE_DETECTED = "mission_upload_source_detected"
EVT_PIXHAWK_MISSION_MIRROR_BEGIN = "pixhawk_mission_mirror_begin"
EVT_PIXHAWK_MISSION_MIRROR_SUCCESS = "pixhawk_mission_mirror_success"
EVT_PIXHAWK_MISSION_MIRROR_FAILED = "pixhawk_mission_mirror_failed"
EVT_READY_STATE_CHANGED = "ready_state_changed"
EVT_START_GATE_CHECK_BEGIN = "start_gate_check_begin"
EVT_START_GATE_CHECK_PASSED = "start_gate_check_passed"
EVT_START_GATE_CHECK_FAILED = "start_gate_check_failed"
EVT_MISSION_START_REQUESTED = "mission_start_requested"
EVT_MISSION_STARTED = "mission_started"
EVT_MISSION_START_REJECTED = "mission_start_rejected"
EVT_POST_START_LOCK_ENABLED = "post_start_lock_enabled"
EVT_POST_START_COMMAND_REJECTED = "post_start_command_rejected"
EVT_PARKUR_TRANSITION_REQUESTED = "parkur_transition_requested"
EVT_PARKUR_TRANSITION_COMPLETED = "parkur_transition_completed"
EVT_WAYPOINT_REACHED = "waypoint_reached"
EVT_GATE_PASSED = "gate_passed"
EVT_MISSION_COMPLETED = "mission_completed"
EVT_MISSION_ABORTED = "mission_aborted"
EVT_MISSION_TIMEOUT = "mission_timeout"
EVT_MISSION_FINAL_STATE_WRITTEN = "mission_final_state_written"


# ---------------------------------------------------------------------------
# RC / E-Stop Events
# ---------------------------------------------------------------------------
EVT_RC_FRAME_RECEIVED = "rc_frame_received"
EVT_RC_CHANNEL_SNAPSHOT = "rc_channel_snapshot"
EVT_RC_LINK_LOST = "rc_link_lost"
EVT_RC_LINK_RESTORED = "rc_link_restored"
EVT_MANUAL_OVERRIDE_DETECTED = "manual_override_detected"
EVT_MANUAL_OVERRIDE_RELEASED = "manual_override_released"
EVT_ESTOP_STATE_CHANGED = "estop_state_changed"
EVT_ESTOP_TRIGGERED = "estop_triggered"
EVT_ESTOP_CLEARED = "estop_cleared"
EVT_PHYSICAL_KILL_CHAIN_STATUS = "physical_kill_chain_status"
EVT_FAILSAFE_CONDITION_DETECTED = "failsafe_condition_detected"
EVT_FAILSAFE_ENTERED = "failsafe_entered"
EVT_FAILSAFE_RECOVERED = "failsafe_recovered"
EVT_HOLD_COMMAND_SENT = "hold_command_sent"
EVT_GUIDED_SETPOINT_BLOCKED_BY_SAFETY = "guided_setpoint_blocked_by_safety"
EVT_UNSAFE_OUTPUT_REJECTED = "unsafe_output_rejected"


# ---------------------------------------------------------------------------
# MAVLink Events
# ---------------------------------------------------------------------------
EVT_MAVLINK_CONNECTION_OPENED = "mavlink_connection_opened"
EVT_MAVLINK_CONNECTION_CLOSED = "mavlink_connection_closed"
EVT_MAVLINK_HEARTBEAT_RECEIVED = "mavlink_heartbeat_received"
EVT_MAVLINK_HEARTBEAT_TIMEOUT = "mavlink_heartbeat_timeout"
EVT_MAVLINK_MODE_CHANGE_REQUESTED = "mavlink_mode_change_requested"
EVT_MAVLINK_MODE_CHANGE_RESULT = "mavlink_mode_change_result"
EVT_MAVLINK_ARMING_RESULT = "mavlink_arming_result"
EVT_MAVLINK_SETPOINT_SENT = "mavlink_setpoint_sent"
EVT_MAVLINK_SETPOINT_REJECTED = "mavlink_setpoint_rejected"
EVT_MAVLINK_SETPOINT_TIMEOUT = "mavlink_setpoint_timeout"
EVT_MAVLINK_GPS_FIX_STATUS = "mavlink_gps_fix_status"
EVT_MAVLINK_EKF_STATUS = "mavlink_ekf_status"
EVT_MAVLINK_BATTERY_STATUS = "mavlink_battery_status"
EVT_MAVLINK_SERVO_SNAPSHOT = "mavlink_servo_snapshot"
EVT_MAVLINK_PARAM_BASELINE_HASH = "mavlink_param_baseline_hash"
EVT_MAVLINK_MISSION_ITEM_COUNT = "mavlink_mission_item_count"
EVT_MAVLINK_CURRENT_WAYPOINT = "mavlink_current_waypoint"


# ---------------------------------------------------------------------------
# Camera Events
# ---------------------------------------------------------------------------
EVT_CAMERA_OPENED = "camera_opened"
EVT_CAMERA_CLOSED = "camera_closed"
EVT_CAMERA_FRAME_RATE = "camera_frame_rate"
EVT_CAMERA_FRAME_DROP = "camera_frame_drop"
EVT_CAMERA_FRAME_LATENCY = "camera_frame_latency"
EVT_CAMERA_TIMEOUT = "camera_timeout"
EVT_YOLO_MODEL_LOAD_TIME = "yolo_model_load_time"
EVT_YOLO_MODEL_INFO = "yolo_model_info"
EVT_DETECTION_COUNT = "detection_count"
EVT_DETECTION_CLASS = "detection_class"
EVT_DETECTION_CONFIDENCE = "detection_confidence"
EVT_DETECTION_BBOX = "detection_bbox"
EVT_TARGET_COLOR_LOCKED = "target_color_locked"
EVT_TARGET_COLOR_DETECTION_RESULT = "target_color_detection_result"
EVT_WRONG_TARGET_DETECTION_RESULT = "wrong_target_detection_result"
EVT_GATE_BEARING = "gate_bearing"
EVT_VISION_PIPELINE_LATENCY = "vision_pipeline_latency"
EVT_OVERLAY_RECORD_PATH = "overlay_record_path"
EVT_VIDEO_RECORD_START = "video_record_start"
EVT_VIDEO_RECORD_STOP = "video_record_stop"


# ---------------------------------------------------------------------------
# Lidar Events
# ---------------------------------------------------------------------------
EVT_LIDAR_CONNECTION_STATUS = "lidar_connection_status"
EVT_LIDAR_SCAN_RATE = "lidar_scan_rate"
EVT_LIDAR_VALID_SCAN_COUNT = "lidar_valid_scan_count"
EVT_LIDAR_INVALID_SCAN_COUNT = "lidar_invalid_scan_count"
EVT_LIDAR_RANGE_STATS = "lidar_range_stats"
EVT_LIDAR_SECTOR_DISTANCES = "lidar_sector_distances"
EVT_LIDAR_OBSTACLE_CLUSTER_COUNT = "lidar_obstacle_cluster_count"
EVT_COSTMAP_UPDATE_TIME = "costmap_update_time"
EVT_COSTMAP_GRID_INFO = "costmap_grid_info"
EVT_COSTMAP_CELL_RATIOS = "costmap_cell_ratios"
EVT_COSTMAP_ORIGIN = "costmap_origin"
EVT_COSTMAP_FRESHNESS = "costmap_freshness"
EVT_LIDAR_SCAN_DROP = "lidar_scan_drop"
EVT_LIDAR_TIMEOUT = "lidar_timeout"
EVT_OBSTACLE_PROXIMITY_ALARM = "obstacle_proximity_alarm"
EVT_AVOIDANCE_PRIORITY_REASON = "avoidance_priority_reason"
EVT_COSTMAP_ARTIFACT_PATH = "costmap_artifact_path"


# ---------------------------------------------------------------------------
# Planner / Controller Events
# ---------------------------------------------------------------------------
EVT_PLANNER_INPUT_SNAPSHOT = "planner_input_snapshot"
EVT_PLANNER_CURRENT_POSE = "planner_current_pose"
EVT_PLANNER_CURRENT_GOAL = "planner_current_goal"
EVT_PLANNER_LOCAL_GOAL = "planner_local_goal"
EVT_PLANNER_GLOBAL_WAYPOINT = "planner_global_waypoint"
EVT_PLANNER_PATH_INFO = "planner_path_info"
EVT_PLANNER_PATH_BLOCKED = "planner_path_blocked"
EVT_PLANNER_REPLAN_REQUESTED = "planner_replan_requested"
EVT_PLANNER_REPLAN_COMPLETED = "planner_replan_completed"
EVT_PLANNER_REPLAN_FAILED = "planner_replan_failed"
EVT_AVOIDANCE_ACTIVATED = "avoidance_activated"
EVT_AVOIDANCE_CLEARED = "avoidance_cleared"
EVT_AVOIDANCE_SIDE_SELECTED = "avoidance_side_selected"
EVT_CONTROLLER_TARGET_SPEED = "controller_target_speed"
EVT_CONTROLLER_TARGET_HEADING = "controller_target_heading"
EVT_CONTROLLER_HEADING_ERROR = "controller_heading_error"
EVT_CONTROLLER_CROSS_TRACK_ERROR = "controller_cross_track_error"
EVT_CONTROLLER_SPEED_ERROR = "controller_speed_error"
EVT_SETPOINT_CLAMP_REASON = "setpoint_clamp_reason"
EVT_SETPOINT_SENT = "setpoint_sent"
EVT_SETPOINT_REJECTED = "setpoint_rejected"
EVT_GUIDANCE_SOURCE = "guidance_source"


# ---------------------------------------------------------------------------
# Shutdown Events
# ---------------------------------------------------------------------------
EVT_SHUTDOWN_REQUESTED = "shutdown_requested"
EVT_SHUTDOWN_SOURCE = "shutdown_source"
EVT_ACTIVE_MISSION_STATE = "active_mission_state"
EVT_LAST_KNOWN_POSE = "last_known_pose"
EVT_LAST_KNOWN_VELOCITY = "last_known_velocity"
EVT_LAST_MODE = "last_mode"
EVT_LAST_WAYPOINT = "last_waypoint"
EVT_LAST_TARGET_COLOR = "last_target_color"
EVT_ACTIVE_FAILSAFE_REASON = "active_failsafe_reason"
EVT_OPEN_FILES_CLOSED = "open_files_closed"
EVT_CHILD_PROCESSES_TERMINATED = "child_processes_terminated"
EVT_DOCKER_STOP_STATUS = "docker_stop_status"
EVT_FINAL_TELEMETRY_SNAPSHOT = "final_telemetry_snapshot"
EVT_FINAL_MISSION_STATE = "final_mission_state"
EVT_ARTIFACT_MANIFEST_WRITTEN = "artifact_manifest_written"
EVT_LOG_FLUSH_RESULT = "log_flush_result"
EVT_SHUTDOWN_COMPLETE = "shutdown_complete"


# ---------------------------------------------------------------------------
# Observability / Logging Events
# ---------------------------------------------------------------------------
EVT_LOGGING_DEGRADED = "logging_degraded"
EVT_TRACE_ENTER = "trace_enter"
EVT_TRACE_EXIT = "trace_exit"
EVT_RUN_LOGGER_INIT = "run_logger_init"
EVT_RUN_LOGGER_SHUTDOWN = "run_logger_shutdown"
EVT_MISSION_STATE_CHANGED = "mission_state_changed"
EVT_PARKUR_CHANGED = "parkur_changed"
EVT_DISK_SPACE_WARNING = "disk_space_warning"
EVT_DISK_SPACE_CRITICAL = "disk_space_critical"


# ---------------------------------------------------------------------------
# Security-critical events (never rate-limited)
# ---------------------------------------------------------------------------
SECURITY_CRITICAL_EVENTS = frozenset([
    EVT_ESTOP_TRIGGERED,
    EVT_ESTOP_CLEARED,
    EVT_ESTOP_STATE_CHANGED,
    EVT_MANUAL_OVERRIDE_DETECTED,
    EVT_MANUAL_OVERRIDE_RELEASED,
    EVT_MISSION_STARTED,
    EVT_MISSION_COMPLETED,
    EVT_MISSION_ABORTED,
    EVT_FAILSAFE_ENTERED,
    EVT_FAILSAFE_RECOVERED,
    EVT_PHYSICAL_KILL_CHAIN_STATUS,
    EVT_LOGGING_DEGRADED,
    EVT_POST_START_COMMAND_REJECTED,
    EVT_UNSAFE_OUTPUT_REJECTED,
    EVT_GUIDED_SETPOINT_BLOCKED_BY_SAFETY,
])


def is_security_critical(event_name: str) -> bool:
    """Check if an event is security-critical (never rate-limited)."""
    return event_name in SECURITY_CRITICAL_EVENTS


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
def validate_event(event: Dict[str, Any]) -> List[str]:
    """
    Validate an event dict against the schema.
    Returns a list of error messages (empty = valid).
    """
    errors = []
    for field in REQUIRED_FIELDS:
        if field not in event:
            errors.append(f"missing required field: {field}")

    level = event.get("level")
    if level is not None and level not in ("DEBUG", "INFO", "WARN", "ERROR"):
        errors.append(f"invalid level: {level}")

    ts = event.get("ts")
    if ts is not None and not isinstance(ts, str):
        errors.append(f"ts must be string, got {type(ts).__name__}")

    monotonic_ms = event.get("monotonic_ms")
    if monotonic_ms is not None and not isinstance(monotonic_ms, (int, float)):
        errors.append(f"monotonic_ms must be numeric, got {type(monotonic_ms).__name__}")

    return errors

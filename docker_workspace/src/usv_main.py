"""
CELEBILER USV - Kritik Rapor Uyum Durum Makinesi
"""

import csv
import glob
import hashlib
import json
import math
import os
from collections import deque
from pathlib import Path
import random
import signal
import sys
import threading
import time

# Enable unbuffered output for proper log capture
sys.stdout = open(sys.stdout.fileno(), 'w', buffering=1)
sys.stderr = open(sys.stderr.fileno(), 'w', buffering=1)

from console_utils import make_console_printer
from compliance_profile import (
    CAMERA_FRAME_TIMEOUT_S,
    CAM_WRONG_TARGET_BEARING_MAX_DEG,
    CAM_WRONG_TARGET_MIN_AREA_NORM,
    CAM_WRONG_TARGET_STRONG_AREA_NORM,
    classify_lidar_map_sector,
    classify_lidar_scan_sector,
    COMMS_POLICY,
    CONTROL_HZ,
    CONTROL_DIR,
    D_MIN_M,
    DYN_SPEED_BAND_HARD_DEG,
    DYN_SPEED_BAND_MEDIUM_DEG,
    DYN_SPEED_BAND_SOFT_DEG,
    DYN_SPEED_ENABLED,
    DYN_SPEED_FACTOR_HARD,
    DYN_SPEED_FACTOR_MEDIUM,
    DYN_SPEED_FACTOR_SOFT,
    DYN_SPEED_FACTOR_STRAIGHT,
    DYN_SPEED_LOG_PERIOD_S,
    DYN_SPEED_MIN_MPS_NAV,
    DYN_SPEED_MIN_MPS_P1,
    DYN_SPEED_MIN_MPS_P2,
    DYN_SPEED_SCOPE,
    FAILSAFE_SLOW_MPS,
    FUSION_BEARING_WINDOW_DEG,
    FUSION_CONFIRM_HOLD_S,
    FUSION_CAMERA_ONLY_TIMEOUT_S,
    FUSION_ENABLED,
    FUSION_GATE_EVENT_CONFIRM_WINDOW_S,
    FUSION_LIDAR_CONFIRM_MAX_M,
    FUSION_LIDAR_MIN_VALID_M,
    FUSION_LOG_PERIOD_S,
    GEOFENCE_ANCHOR_COOLDOWN_S,
    GEOFENCE_ANCHOR_PULSE_S,
    GEOFENCE_ANCHOR_SPEED_MPS,
    GEOFENCE_DRIFT_TRIGGER_M,
    GEOFENCE_ENABLED,
    GEOFENCE_FAILSAFE_ONLY,
    GEOFENCE_LOG_PERIOD_S,
    GEOFENCE_RADIUS_M,
    HEARTBEAT_FAIL_S,
    HEARTBEAT_WARN_S,
    HORIZON_LOCK_ENABLED,
    HORIZON_LOCK_LOG_PERIOD_S,
    HORIZON_LOCK_MAX_CORRECTION_DEG,
    HORIZON_LOCK_MIN_TILT_DEG,
    HORIZON_LOCK_PITCH_GAIN,
    HORIZON_LOCK_ROLL_GAIN,
    HORIZON_LOCK_SCOPE,
    LIDAR_OA_SECTOR_CENTER_HALF_DEG,
    LIDAR_OA_SECTOR_SIDE_MAX_DEG,
    LIDAR_OA_SECTOR_SIDE_MIN_DEG,
    LIDAR_READY_TIMEOUT_S,
    LIDAR_STATUS_JSON_ENABLED,
    LINK_TOPOLOGY,
    normalize_lidar_bearing_deg,
    LOG_DIR,
    MISSION_FILE_DEFAULT,
    MISSION_INPUT_FORMAT,
    P1_AUTO_WAYPOINT_TIMEOUT_S,
    P1_SPEED_APPROACH_MPS,
    P1_SPEED_CRUISE_MPS,
    P2_CAM_YELLOW_FUSION_MULT,
    P2_CAM_YELLOW_WEIGHT,
    P2_ORANGE_BOUNDARY_WEIGHT,
    P2_CRUISE_MPS,
    P2_ESCAPE_MAX_DEG,
    P2_ESCAPE_MAX_DEG_LOCAL_MINIMA,
    P2_GATE_CONFIRM_S,
    P2_LIDAR_WARN_EXIT_MARGIN_M,
    P2_LIDAR_WARN_M,
    NAV_AVOID_BIAS_SLEW_DEG_PER_S,
    NAV_CROSS_TRACK_CORR_CAP_DEG,
    NAV_CROSS_TRACK_ALIGN_MIN_M,
    NAV_CROSS_TRACK_ALIGN_CAP_DEG,
    NAV_CROSS_TRACK_K,
    NAV_CROSS_TRACK_L1_MAX_M,
    NAV_CROSS_TRACK_L1_MIN_M,
    NAV_CROSS_TRACK_MIN_LEG_M,
    NAV_ALIGN_ACQUIRE_DEG,
    NAV_ALIGN_ENTER_ADVANCE_DEG,
    NAV_ALIGN_HEADING_DONE_DEG,
    NAV_ALIGN_PIVOT_UNTIL_DEG,
    NAV_ALIGN_STABLE_S,
    NAV_STRICT_HEADING_FIRST,
    NAV_WP_ARRIVAL_EXIT_MARGIN_M,
    NAV_ALIGN_REACQUIRE_DEG,
    NAV_ALIGN_TIMEOUT_LARGE_ERR_DEG,
    NAV_ALIGN_TURN_IMMUNITY_S,
    NAV_HEADING_MAX_ERROR_DEG,
    NAV_HEADING_CMD_SLEW_DEG_PER_S,
    NAV_HEADING_DAMPING_ENABLED,
    NAV_HEADING_DAMPING_LPF_ALPHA,
    NAV_HEADING_DAMPING_YAWRATE_GAIN,
    NAV_ALIGN_LIDAR_SUSPEND_MAX_DIST_M,
    NAV_ALIGN_TIGHT_OBSTACLE_M,
    NAV_ALIGN_MAX_SPEED_MPS,
    NAV_ALIGN_CREEP_SPEED_MPS,
    NAV_MIN_PROGRESS_SPEED_MPS,
    NAV_ALIGN_REVERT_ALIGN_DEG,
    NAV_ALIGN_TIMEOUT_S,
    NAV_ALIGN_SUSPEND_NEAR_M,
    NAV_WP_APPROACH_SPEED_CAP_DIST_M,
    NAV_WP_APPROACH_SPEED_CAP_MPS,
    NAV_NEAR_WP_TURN_SPEED_CAP_DIST_M,
    NAV_NEAR_WP_TURN_ERR_DEG,
    NAV_NEAR_WP_TURN_SPEED_CAP_MPS,
    NAV_TURN_SPEED_CAP_HARD_ERR_DEG,
    NAV_TURN_SPEED_CAP_HARD_MPS,
    NAV_TURN_SPEED_CAP_MEDIUM_ERR_DEG,
    NAV_TURN_SPEED_CAP_MEDIUM_MPS,
    NAV_TURN_SPEED_CAP_SOFT_ERR_DEG,
    NAV_TURN_SPEED_CAP_SOFT_MPS,
    P2_LOCAL_MINIMA_TIMEOUT_S,
    P2_STABLE_S,
    P2_WAIT_SPEED_MPS,
    P3_MAX_SPEED_MPS,
    P3_TARGET_BEARING_GAIN,
    P3_TARGET_HEADING_CLAMP_DEG,
    P3_RETRY_COUNT,
    P3_RETRY_S,
    P3_REVERSE_DISTANCE_M,
    P3_REVERSE_SPEED_MPS,
    P3_REVERSE_TIMEOUT_S,
    P3_TIMEOUT_S,
    R_WP_M,
    RC7_ESTOP_FORCE_PWM,
    RC7_ESTOP_PWM,
    RC7_SAFE_PWM,
    RC_RACE_START_PWM,
    REPORT_TELEMETRY_GROUPS,
    MODE_NEUTRAL_DWELL_S,
    HEADING_DEADZONE_DEG,
    HEADING_HYST_ENTER_DEG,
    HEADING_HYST_EXIT_DEG,
    HEADING_WRONG_TURN_ERR_DEG,
    HEADING_WRONG_TURN_YAWRATE_DPS,
    HEADING_WRONG_TURN_TRIGGER_S,
    HEADING_WRONG_TURN_GAIN,
    HEADING_WRONG_TURN_SPEED_CAP_MPS,
    PWM_NEUTRAL_US,
    PWM_MIN_US,
    PWM_MAX_US,
    PWM_DEADBAND_US,
    PWM_MIN_EFFECTIVE_US,
    PWM_SLEW_RATE_US_PER_S,
    PWM_JERK_LIMIT_US_PER_S2,
    PWM_TRIM_MAX_US,
    PWM_TRIM_LEARN_GAIN,
    PWM_TRIM_ERR_BAND_DEG,
    PWM_TRIM_YAW_RATE_BAND_DPS,
    SIM_ALLOW_RC_OVERRIDE,
    T_HOLD_S,
    TRUST_BAR_ENABLED,
    TRUST_GOOD_THRESHOLD,
    TRUST_LIDAR_POINTS_FULL,
    TRUST_LOG_PERIOD_S,
    TRUST_WARN_THRESHOLD,
    TRUST_WEIGHT_CAMERA,
    TRUST_WEIGHT_GPS,
    TRUST_WEIGHT_LIDAR,
    TRUST_WEIGHT_RC,
    USV_MODE,
    USV_MODE_RACE,
    UNIFIED_EXECUTION_PATH,
    WIND_ASSIST_ACTIVE_ERR_MAX_DEG,
    WIND_ASSIST_BIAS_MAX_DEG,
    WIND_ASSIST_DECAY_PER_S,
    WIND_ASSIST_ENABLED,
    WIND_ASSIST_I_GAIN,
    WIND_ASSIST_LOG_PERIOD_S,
    WIND_ASSIST_SCOPE,
    evaluate_readiness_flags,
    evaluate_storage_health,
)

print = make_console_printer("USV")

from mission_adapter import adapt_mission_to_structured
from mission_config import (
    MISSION_PROFILE_ALLOWED_P3_ENGAGEMENT_MODES,
    MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT,
    MISSION_PROFILE_DEFAULT_P3_ENGAGEMENT_MODE,
    MISSION_PROFILE_DEFAULT_TRANSITION_POLICY,
    TARGET_STATE_FILE,
    default_sim_mission,
    get_mission_split_profile,
    load_target_state,
    parse_mission_profile_payload,
    split_nav_engage,
    validate_coordinate_mission,
    validate_target_color as validate_mission_target_color,
)
from motor_controller import MotorController as CoreMotorController
from navigation import clamp_heading_error, compass_heading_to_ne_velocity
from nav_guidance import (
    GuidanceDecision,
    allocate_twin_thrusters,
    compute_nav_decision,
    compute_p2_decision,  # kept as shim — use compute_nav_decision in new code
    schedule_waypoint_speed,
)
from obstacle_avoidance import decide_three_sector_avoidance
from json_atomic import atomic_write_json
from runtime_debug_log import log_jsonl, redirect_std_streams, setup_component_logger
from sim_nav_state import heading_rad_to_nav_deg, load_sim_nav_state, mavlink_heading_cdeg_valid, parse_sim_home
from spatial_frame import (
    current_spatial_pose_enu,
    lidar_local_to_world_enu_compass,
    lidar_local_to_world_enu_gazebo,
    world_enu_to_lidar_local_compass,
)

_usv_dbg = setup_component_logger("usv_main")
redirect_std_streams(_usv_dbg)
_usv_dbg.info(
    "boot LOG_DIR=%s USV_SIM=%s USV_MODE=%s",
    LOG_DIR,
    os.environ.get("USV_SIM"),
    os.environ.get("USV_MODE"),
)


def _gps_week_tow_ms_from_unix(now_s: float) -> tuple:
    """GPS week and ms-within-week for MAVLink GPS_INPUT (ArduPilot AP_GPS_MAV uses time_week > 0)."""
    from datetime import datetime, timezone

    gps0 = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    t = datetime.fromtimestamp(float(now_s), tz=timezone.utc)
    delta_s = max(0.0, (t - gps0).total_seconds())
    week = int(delta_s // 604800)
    ms = int((delta_s % 604800) * 1000)
    return max(1, week), ms % 604_800_000


BAUD_RATE = 115200
CONTROL_LOOP_HZ = max(5.0, min(10.0, float(CONTROL_HZ)))
LOOP_DT = 1.0 / CONTROL_LOOP_HZ
P2_READY_TIMEOUT_S = float(os.environ.get("USV_P2_READY_TIMEOUT_S", "30.0"))
P2_GATE_MIN_WAIT_TIMEOUT_S = float(os.environ.get("USV_P2_GATE_MIN_WAIT_TIMEOUT_S", "60.0"))

# Mission file default to profile-resolved path
MISSION_FILE = os.environ.get("MISSION_FILE", MISSION_FILE_DEFAULT)

FLAG_START = f"{CONTROL_DIR}/start_mission.flag"
FLAG_STOP = f"{CONTROL_DIR}/emergency_stop.flag"
STATE_FILE = f"{CONTROL_DIR}/mission_state.json"
LINK_STATE_FILE = f"{CONTROL_DIR}/telemetry_link_state.json"
CAMERA_STATUS_FILE = f"{CONTROL_DIR}/camera_status.json"
MOTOR_COMMAND_FILE = f"{CONTROL_DIR}/motor_command.json"
CAMERA_STATUS_TIMEOUT_S = float(os.environ.get("CAMERA_STATUS_TIMEOUT_S", "2.5"))
CAMERA_STATUS_MTIME_TIMEOUT_S = float(os.environ.get("CAMERA_STATUS_MTIME_TIMEOUT_S", "3.0"))
GUIDED_COMMAND_HZ = max(5.0, min(10.0, float(os.environ.get("USV_GUIDED_COMMAND_HZ", "10"))))
GUIDED_COMMAND_MIN_PERIOD_S = 1.0 / GUIDED_COMMAND_HZ
USE_RC_OVERRIDE_FOR_AUTONOMY = (
    os.environ.get("USV_USE_RC_OVERRIDE", "").strip() == "1"
    and USV_MODE != USV_MODE_RACE
)
AUTONOMY_ACTUATOR_MODE = "rc_override_bench_test" if USE_RC_OVERRIDE_FOR_AUTONOMY else "guided_velocity_yaw"
PIXHAWK_MISSION_SYNC_TIMEOUT_S = float(os.environ.get("PIXHAWK_MISSION_SYNC_TIMEOUT_S", "4.0"))
# Startup local mission upload must never freeze the companion state writer.
# If SITL is still warming up, keep sensors/MAVLink/state fresh while waiting,
# then fall back to the local mission reference if the FC is not ready yet.
PIXHAWK_MISSION_READY_WAIT_S = float(os.environ.get("PIXHAWK_MISSION_READY_WAIT_S", "20.0"))
PIXHAWK_MISSION_READY_SERVICE_DT_S = float(os.environ.get("PIXHAWK_MISSION_READY_SERVICE_DT_S", "0.2"))
# Idle loop: Pixhawk mission mirror cadence (separate from per-download item retries).
PIXHAWK_MISSION_IDLE_MIRROR_S = float(os.environ.get("PIXHAWK_MISSION_IDLE_MIRROR_S", "4.0"))
FC_EKF_VARIANCE_BLOCK_S = float(os.environ.get("FC_EKF_VARIANCE_BLOCK_S", "4.0"))
FC_START_STABLE_S = float(os.environ.get("FC_START_STABLE_S", "4.0"))
FC_NAV_READY_STABLE_S = float(os.environ.get("FC_NAV_READY_STABLE_S", "1.5"))
FC_NAV_READY_STABLE_S_EFFECTIVE = float(
    os.environ.get(
        "USV_SIM_FC_NAV_READY_STABLE_S" if os.environ.get("USV_SIM") == "1" else "FC_NAV_READY_STABLE_S",
        "0.4" if os.environ.get("USV_SIM") == "1" else str(FC_NAV_READY_STABLE_S),
    )
    or ("0.4" if os.environ.get("USV_SIM") == "1" else str(FC_NAV_READY_STABLE_S))
)
FC_EKF_VARIANCE_MAX = float(os.environ.get("FC_EKF_VARIANCE_MAX", "1.0"))
FC_GPS_FRESH_S = float(os.environ.get("FC_GPS_FRESH_S", "3.0"))
FC_GLOBAL_POSITION_FRESH_S = float(os.environ.get("FC_GLOBAL_POSITION_FRESH_S", "3.0"))
FC_EKF_FRESH_S = float(os.environ.get("FC_EKF_FRESH_S", "5.0"))
FC_START_PREREQ_WAIT_S = float(os.environ.get("FC_START_PREREQ_WAIT_S", "5.0"))
SIM_EKF_GPS_GLITCH_GRACE_S = float(os.environ.get("SIM_EKF_GPS_GLITCH_GRACE_S", "30.0"))
_ARM_STATUSTEXT_BLOCKERS = frozenset({
    "accel_inconsistent_recent",
    "gyro_inconsistent_recent",
    "bad_position_recent",
    "ekf_variance_recent",
})
ARM_STATUSTEXT_BLOCK_S = float(os.environ.get(
    "ARM_STATUSTEXT_BLOCK_S",
    "15.0" if os.environ.get("USV_SIM") == "1" else "8.0",
))
FC_ARM_READY_WAIT_S = float(os.environ.get(
    "FC_ARM_READY_WAIT_S",
    "12.0" if os.environ.get("USV_SIM") == "1" else "8.0",
))
SIM_EKF_ORIGIN_ARM_DELAY_S = float(os.environ.get("SIM_EKF_ORIGIN_ARM_DELAY_S", "1.5"))
# Sim + yerel dosya: idle Pixhawk indirmesi MAVLink'i kirletir; mission_item_*_timeout gösterebilir.
_SIM_SKIP_IDLE_PIXHAWK_SOURCES = frozenset({
    "local_flat_file",
    "structured_legacy",
    "default_sim_mission",
    "sim_default_flat_file",
})
GUIDED_GLOBAL_VEL_YAW_TYPE_MASK = 0x9C7
GUIDED_EXPECTED_MODE_ID = 15
SIM_MANUAL_RC_FALLBACK_ENABLED = (
    os.environ.get("SIM_FALLBACK_MANUAL_RC_ACTUATION", "0").strip() == "1"
)
PIXHAWK_OBSTACLE_DISTANCE_ENABLED = (
    os.environ.get("USV_SEND_OBSTACLE_DISTANCE", "1").strip() != "0"
)
OBSTACLE_DISTANCE_PERIOD_S = float(os.environ.get("USV_OBSTACLE_DISTANCE_PERIOD_S", "0.20"))
NAV_STUCK_TIMEOUT_S = float(os.environ.get("USV_NAV_STUCK_TIMEOUT_S", "10.0"))
NAV_STUCK_MIN_COMMAND_MPS = float(os.environ.get("USV_NAV_STUCK_MIN_COMMAND_MPS", "0.20"))
NAV_STUCK_PROGRESS_EPS_M = float(os.environ.get("USV_NAV_STUCK_PROGRESS_EPS_M", "0.20"))
NAV_STUCK_SPEED_EPS_MPS = float(os.environ.get("USV_NAV_STUCK_SPEED_EPS_MPS", "0.08"))

FILE3_MAP_MP4 = f"{LOG_DIR}/file3_local_map.mp4"
FILE3_INDEX_CSV = f"{LOG_DIR}/file3_local_map_index.csv"
FILE3_MAP_JPG = f"{LOG_DIR}/file3_local_map_latest.jpg"

# Lidar stability + persistent occupancy settings.
LIDAR_FRAME_MAX_AGE_S = 0.15
LIDAR_VALID_MIN_M = 0.20
LIDAR_VALID_MAX_M = 18.0
LIDAR_SELF_FILTER_MIN_M = float(os.environ.get(
    "LIDAR_SELF_FILTER_MIN_M",
    "0.55" if os.environ.get("USV_SIM") == "1" else "0.20",
))
LIDAR_VALID_RATIO_MIN = float(os.environ.get(
    "LIDAR_VALID_RATIO_MIN",
    "0.10" if os.environ.get("USV_SIM") == "1" else "0.15",
))
LIDAR_STALE_WARN_CONSEC = 20
LIDAR_SIM_CLOCK_FUTURE_TOL_S = 0.50
LIDAR_ANGULAR_RESIDUAL_MAX_M = 0.35
LIDAR_JUMP_ISOLATION_M = 1.2
LIDAR_BIN_SIZE_DEG = 1.5
LIDAR_TEMPORAL_WINDOW = 6
LIDAR_TEMPORAL_MIN_SEEN = 3
LIDAR_CONF_TENTATIVE = 0.35
LIDAR_CONF_STABLE = 0.60
LIDAR_CONF_DROP = 0.35
LIDAR_GRID_RES_M = 0.25
LIDAR_GRID_SPAN_M = 60.0
LIDAR_MAP_OCC_LOG_ODDS = 1.2
LIDAR_MAP_FREE_LOG_ODDS = -1.0
LIDAR_MAP_OCC_STABLE_COUNT = 3
_is_sim_lidar = os.environ.get("USV_SIM") == "1"
LIDAR_MAP_DISPLAY_STABLE_COUNT = int(os.environ.get("LIDAR_MAP_DISPLAY_STABLE_COUNT", "3" if _is_sim_lidar else "5"))
LIDAR_MAP_PERSIST_STABLE_COUNT = int(os.environ.get("LIDAR_MAP_PERSIST_STABLE_COUNT", "8"))
LIDAR_MAP_PERSIST_MIN_HITS = int(os.environ.get("LIDAR_MAP_PERSIST_MIN_HITS", "12"))
LIDAR_MAP_MAX_LOG_ODDS = 4.0
LIDAR_MAP_MIN_LOG_ODDS = -4.0
LIDAR_TAU_STABLE_S = float(os.environ.get("LIDAR_TAU_STABLE_S", "45.0" if _is_sim_lidar else "22.0"))
LIDAR_TAU_TENTATIVE_S = float(os.environ.get("LIDAR_TAU_TENTATIVE_S", "8.0" if _is_sim_lidar else "3.0"))
LIDAR_BLOCKED_ENTER = 0.65
LIDAR_BLOCKED_EXIT = 0.45
LIDAR_SIDE_LOCK_S = 1.0
LIDAR_SIDE_SWITCH_PENALTY = 0.12
LIDAR_UNRELIABLE_DECAY_GAIN_MAX = float(os.environ.get("LIDAR_UNRELIABLE_DECAY_GAIN_MAX", "1.5" if _is_sim_lidar else "4.0"))
LIDAR_UNRELIABLE_STABLE_DROP = 2
LIDAR_UNRELIABLE_RISK_DECAY = 0.72
LIDAR_UNRELIABLE_CLEAR_BLOCK_CONSEC = 5
LIDAR_DEGRADED_LIMIT_TIMEOUT_S = 4.0
LIDAR_DEGRADED_HOLD_TIMEOUT_S = 8.0
SIM_AVOID_RECOVERY_WINDOW_S = 6.0
SIM_AVOID_RECOVERY_DIST_GROWTH_M = 0.90
SIM_AVOID_RECOVERY_BLOCKED_MIN = 0.95
SIM_AVOID_RECOVERY_COOLDOWN_S = 4.0
SIM_AVOID_RECOVERY_HOLD_S = 0.40


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def haversine_distance(lat1, lon1, lat2, lon2):
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (
        math.sin(d_lat / 2) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(d_lon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return 6371000 * c


def calculate_bearing(lat1, lon1, lat2, lon2):
    d_lon = math.radians(lon2 - lon1)
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    y = math.sin(d_lon) * math.cos(lat2_r)
    x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def normalize_heading_error(error):
    try:
        err_rad = math.radians(float(error))
    except (TypeError, ValueError):
        return 0.0
    return math.degrees(math.atan2(math.sin(err_rad), math.cos(err_rad)))


def _smooth_lidar_sector_m(prev, raw):
    """EMA on lidar sector range to reduce BEST_EFFORT drop / UI flicker (open range stays 99)."""
    if raw >= 98.0:
        return raw
    if prev is None:
        return raw
    alpha = 0.30
    return alpha * float(raw) + (1.0 - alpha) * float(prev)


ARDUROVER_MODE_NAMES = {
    0: "MANUAL",
    1: "ACRO",
    3: "STEERING",
    4: "HOLD",
    5: "LOITER",
    6: "FOLLOW",
    7: "SIMPLE",
    10: "AUTO",
    11: "RTL",
    12: "SMART_RTL",
    15: "GUIDED",
    16: "INITIALISING",
}

MAV_RESULT_NAMES = {
    0: "ACCEPTED",
    1: "TEMPORARILY_REJECTED",
    2: "DENIED",
    3: "UNSUPPORTED",
    4: "FAILED",
    5: "IN_PROGRESS",
    6: "CANCELLED",
    7: "COMMAND_LONG_ONLY",
    8: "COMMAND_INT_ONLY",
    9: "COMMAND_UNSUPPORTED_MAV_FRAME",
}


class USVStateMachine:
    STATE_IDLE = 0
    STATE_NAV = 1        # Tüm nav waypoint'leri: GPS + lidar kaçınma + kamera kapı hizalama
    STATE_ENGAGE = 2     # Son aşama: HSV hedef tespiti + kamikaze angajman
    STATE_COMPLETED = 4
    STATE_HOLD = 5

    def __init__(self):
        print("=" * 60)
        print("  CELEBILER USV - KRITIK UYUM MODU")
        print("=" * 60)
        print(f"[INFO] MOD: {'[RACE] YARISMA' if USV_MODE == USV_MODE_RACE else '[TEST] TEST'}")

        # CRITICAL: Check if running in simulation
        self.simulation_mode = bool(os.environ.get("USV_SIM") == "1")
        print(f"[INIT] Simulation mode: {self.simulation_mode} (USV_SIM={os.environ.get('USV_SIM')})")
        self.state = self.STATE_IDLE
        self.mission_active = False
        self.mission_start_time = 0.0
        self.start_phase = "idle"
        self._boot_mono = time.monotonic()
        self.command_lock = False

        # Initialize GPS position from SIM_HOME if in simulation mode
        # Robust parse: SIM_HOME = "lat,lon,alt,heading" (e.g. "-35.363262,149.165237,584,0")
        sim_home_lat, sim_home_lon = 0.0, 0.0
        _sim_home_raw = os.environ.get("SIM_HOME", "").strip()
        if _sim_home_raw:
            try:
                _home_tokens = [t.strip() for t in _sim_home_raw.split(",")]
                if len(_home_tokens) >= 2:
                    _lat_candidate = float(_home_tokens[0])
                    _lon_candidate = float(_home_tokens[1])
                    if -90.0 <= _lat_candidate <= 90.0 and -180.0 <= _lon_candidate <= 180.0:
                        if abs(_lat_candidate) > 1e-6 or abs(_lon_candidate) > 1e-6:
                            sim_home_lat = _lat_candidate
                            sim_home_lon = _lon_candidate
            except (ValueError, IndexError):
                pass
        
        # Fallback hard-coded home if SIM_HOME is missing/invalid in simulation mode
        if os.environ.get("USV_SIM") == "1" and abs(sim_home_lat) < 1e-6 and abs(sim_home_lon) < 1e-6:
            sim_home_lat, sim_home_lon = -35.363262, 149.165237
            print(f"[WARN] [GPS_INIT] SIM_HOME gecersiz/eksik, varsayilan kullaniliyor: {sim_home_lat},{sim_home_lon}")
        
        self.current_lat = sim_home_lat if os.environ.get("USV_SIM") == "1" else 0.0
        self.current_lon = sim_home_lon if os.environ.get("USV_SIM") == "1" else 0.0
        self._sim_home_lat = sim_home_lat  # Saklayarak SIM modunda fallback olarak kullanilir
        self._sim_home_lon = sim_home_lon
        if os.environ.get("USV_SIM") == "1":
            print(f"[GPS_INIT] Baslangic GPS: lat={self.current_lat:.7f} lon={self.current_lon:.7f} (SIM_HOME)")
        self.current_heading = 0.0
        # Heading smoothing filter (exponential moving average) to reduce compass drift oscillation
        self._heading_ema_alpha = 0.15  # Smoothing factor (0-1, higher = more responsive)
        self._heading_ema = None  # Initialize on first update
        self.current_roll_deg = 0.0
        self.current_pitch_deg = 0.0
        self.current_yaw_rate_dps = 0.0
        self.current_speed_mps = 0.0
        self.gps_satellites_visible = 0
        self.gps_fix_type = 0
        self.gps_global_position_int_received = False  # Track if GLOBAL_POSITION_INT arrived (simulation GPS_RAW_INT fallback)
        self.battery_voltage = 0.0  # Volts (from SYS_STATUS MAVLink message)
        self.rc_channels = {f"ch{i}": 1500 for i in range(1, 9)}  # Initialize to neutral PWM
        self._last_sent_rc_override = {"ch1": 1500, "ch3": 1500, "ts": 0.0}
        self._last_rc_override_diag = {
            "semantics": "left_right_pwm",
            "desired_left_pwm": 1500,
            "desired_right_pwm": 1500,
            "ch1": 1500,
            "ch3": 1500,
        }
        self._both_reverse_count = 0
        self._single_side_reverse_count = 0
        self._reverse_while_heading_small_count = 0
        self._sim_mavlink_actuation_rc = False  # SIM test: MANUAL + MAVLink RC when GUIDED/EKF rejects
        self.sim_actuation_fallback = "none"
        self.sim_actuation_fallback_reason = "none"
        self.vehicle_armed = False
        self.vehicle_mode_custom = 0
        self.vehicle_mode_name = "UNKNOWN"
        self.last_mode_command = {
            "requested": "",
            "custom_mode": None,
            "target_system": None,
            "target_component": None,
            "confirmed": False,
            "actual_mode": "UNKNOWN",
            "actual_custom_mode": None,
            "ack_result": "",
            "statustext": "",
            "ts_monotonic": 0.0,
        }
        self.last_command_ack = {
            "command": None,
            "result": None,
            "result_name": "",
            "progress": None,
            "target_system": None,
            "target_component": None,
            "ts_monotonic": 0.0,
        }
        self.last_statustext = {
            "severity": None,
            "text": "",
            "ts_monotonic": 0.0,
        }
        self.last_servo_output_raw = {
            "servo1_raw": int(PWM_NEUTRAL_US),
            "servo3_raw": int(PWM_NEUTRAL_US),
            "servo2_raw": int(PWM_NEUTRAL_US),
            "servo4_raw": int(PWM_NEUTRAL_US),
            "ts_monotonic": 0.0,
        }
        self.last_nav_controller_output = {
            "nav_roll": 0.0,
            "nav_bearing": 0.0,
            "target_bearing": 0.0,
            "wp_dist": 0.0,
            "xtrack_error": 0.0,
            "ts_monotonic": 0.0,
        }
        self.mission_current_seq = -1
        self.mission_current_ts = 0.0
        self.mission_reached_seq = -1
        self.mission_reached_ts = 0.0
        self.race_p1_completion_source = "none"
        self.fc_ekf_flags = 0
        self.fc_ekf_variances = {}
        self.fc_ekf_report_seen = False
        self.fc_ekf_report_ts = 0.0
        self._last_gps_raw_int_ts = 0.0
        self._last_global_position_int_ts = 0.0
        self._last_mavlink_message_ts = {}
        self.fc_nav_mode_ready = False
        self.fc_nav_mode_ready_since = None
        self.fc_guided_block_reason = "boot"
        self._last_ekf_variance_statustext_ts = 0.0
        self._fc_start_blocker_ts = {}
        self._sim_ekf_origin_set_mono = 0.0
        self.fc_start_ready = False
        self.fc_start_block_reason = "boot"
        self._mode_state_last_signature = None
        self._mode_state_last_log_ts = 0.0
        self._manual_override_active = False
        self._rc_neutral_since = time.monotonic()
        self.mode_state = {
            "canonical_mode": "SAFE_HOLD",
            "source": "boot",
            "reason": "boot",
            "armed": False,
            "vehicle_mode": "UNKNOWN",
            "mission_active": False,
            "command_lock": False,
            "ready_state": False,
            "estop_state": False,
            "ts_monotonic": round(time.monotonic(), 3),
        }
        self._heading_zero_latched = False
        self._wrong_turn_since = None
        self._wrong_turn_active = False
        self._front_min_smooth = 99.0
        self._wrong_turn_count = 0
        self._wrong_turn_last_log_ts = 0.0
        self.heading_control_diagnostic = "nominal"
        self._heading_damping_hold_active = False
        self._heading_damping_hold_count = 0
        self._heading_damping_last_log_ts = 0.0
        self._mixer_prev_left_pwm = int(PWM_NEUTRAL_US)
        self._mixer_prev_right_pwm = int(PWM_NEUTRAL_US)
        self._mixer_prev_left_rate = 0.0
        self._mixer_prev_right_rate = 0.0
        self._mixer_last_ts = time.monotonic()
        self._trim_port_us = 0.0
        self._trim_stbd_us = 0.0
        self._pwm_trim_last_ts = time.monotonic()
        self.motor_controller = CoreMotorController()

        self.v_target = 0.0
        self.heading_target = 0.0
        self.guidance_detail_source = "idle"
        self.guidance_mode = "idle"
        self.guidance_reason = "idle"
        self.avoidance_bias_deg = 0.0
        self.cross_track_error_m = 0.0
        self.cross_track_corr_deg = 0.0
        self.cross_track_align_active = False
        self.nominal_heading_deg = 0.0
        self.gate_assist_bias_deg = 0.0
        self.progress_along_leg_m = 0.0
        self.waypoint_passed_gate = False
        self.waypoint_accept_reason = "idle"
        self.motor_limit_reason = "neutral"
        self.nav_position_source = "boot"
        self.nav_heading_source = "boot"
        self.nav_source_detail = "boot"
        self.nav_state_age_s = 999.0
        self.nav_fix_valid = False
        self.nav_target_bearing_deg = 0.0
        self.nav_target_distance_m = 9999.0
        self.nav_target_distance_delta_m = 0.0
        self.nav_target_distance_increase_count = 0
        self.nav_heading_error_delta_deg = 0.0
        self.nav_heading_error_deg = 0.0
        self._prev_nav_target_distance_m = None
        self._prev_nav_heading_error_deg = None
        self.nav_target_lat = 0.0
        self.nav_target_lon = 0.0
        self.nav_solution_source = "boot"
        self.guided_position_source = "invalid"
        self.closest_waypoint_distance_m = 9999.0
        self._closest_waypoint_distance_seen = 9999.0
        self.nav_arrival_phase = "idle"
        self._last_motor_command_source = "boot"
        self._last_motor_command_write_time = 0.0
        self._last_written_motor_command = None
        self._last_obstacle_distance_send_ts = 0.0
        self._last_obstacle_distance_log_ts = 0.0
        self._nav_no_motion_since = None
        self._nav_no_motion_last_dist_m = None
        self._nav_no_motion_last_progress_m = 0.0
        self._nav_no_motion_last_log_ts = 0.0
        self._wp_leg_start_lat = 0.0
        self._wp_leg_start_lon = 0.0
        self._wp_leg_target_lat = 0.0
        self._wp_leg_target_lon = 0.0
        self._wp_leg_length_m = 0.0
        self._wp_leg_valid = False
        self._nav_align_mode = "advance"
        self._nav_align_t0 = None
        self._wp_transition_until = 0.0
        self._nav_align_lock_until = 0.0
        self._nav_align_stable_since = None
        self._nav_align_phase_last = "idle"
        self._last_guided_diag_ts = 0.0
        self._last_guided_velocity_ne = (0.0, 0.0)
        self._heading_cmd_filtered_deg = 0.0
        self._heading_cmd_last_ts = time.monotonic()
        self._avoid_bias_filtered_deg = 0.0
        self._avoid_bias_last_ts = time.monotonic()
        self._wp_accept_hold_start = None
        self._nav_snapshot = {
            "lat": float(self.current_lat),
            "lon": float(self.current_lon),
            "heading": float(self.current_heading),
            "valid": False,
            "source": "boot",
            "heading_source": "boot",
            "source_detail": "boot",
            "age_s": 999.0,
        }
        self.avoidance_active = False
        self.avoidance_source = "none"
        self.escape_side = "none"
        self.lidar_sector_ages = {"left": None, "center": None, "right": None}

        self.camera_status = {}
        self.camera_status_age_s = 999.0
        self.camera_status_mtime_age_s = 999.0
        self.camera_status_stale = True
        self.camera_adaptation = {
            "enabled": False,
            "mode": "normal",
            "luma_mean": 0.0,
            "exposure_gain": 1.0,
            "exposure_beta": 0.0,
            "hsv_s_shift": 0,
            "hsv_v_shift": 0,
            "hsv_profile": "base",
        }
        self.camera_ready = False
        self.trust_bar = 100.0  # Autonomy health trust score (%)
        self.lidar_available = False
        self.lidar_ready = False
        self.last_lidar_time = 0.0
        self.lidar_left_dist = 99.0
        self.lidar_center_dist = 99.0
        self.lidar_right_dist = 99.0
        self.min_obstacle_distance = 99.0
        self.obstacle_detected = False
        # Collision detection: track if vessel moving with command but stuck
        self._last_collision_ts = 0.0
        self._last_collision_check_pos = (0.0, 0.0)  # (x, y) from vehicle_position.json
        self._collision_cooldown_s = 2.0
        self._collision_position_thresh_m = 0.15  # Stuck if <15cm advance
        self._collision_speed_thresh_mps = 0.2    # With >0.2 m/s command
        self._map_points = []
        self._stable_map_points = []
        self._lidar_front_samples = []
        self._lidar_sector_hold = {"left": None, "center": None, "right": None}
        self._lidar_sector_ts = {"left": 0.0, "center": 0.0, "right": 0.0}
        self._lidar_ema_left = None
        self._lidar_ema_center = None
        self._lidar_ema_right = None
        self._lidar_raw_sector_min_m = 99.0
        self._lidar_current_left_m = 99.0
        self._lidar_current_center_m = 99.0
        self._lidar_current_right_m = 99.0
        self._lidar_frame_status = "init"
        self._lidar_frame_valid_ratio = 0.0
        self._lidar_quality_ready = False
        self._lidar_clock_source = "none"
        self._lidar_stamp_age_s = None
        self._lidar_drop_reason = "init"
        self._lidar_local_obstacle_valid = False
        self._lidar_degraded_since = None
        self._lidar_degraded_mode = "normal"
        self._lidar_degraded_age_s = 0.0
        self._lidar_stale_consecutive = 0
        self._lidar_stale_warned = False
        self._lidar_stale_blocked = False
        self._lidar_unreliable_clock_consecutive = 0
        self._lidar_clock_reject_count = 0
        self._lidar_clock_reject_last_log_ts = 0.0
        self._lidar_last_log_ts = 0.0
        self._lidar_stable_point_count = 0
        self._lidar_tentative_point_count = 0
        self._lidar_temporal_bins = {}
        self._lidar_frame_seq = 0
        self._lidar_last_frame_monotonic = 0.0
        self._lidar_grid = {}
        self._lidar_grid_last_ts = time.monotonic()
        self._lidar_grid_max_cells = 22000
        self._lidar_occ_cells_count = 0
        self._lidar_front_risk = 0.0
        self._lidar_left_risk = 0.0
        self._lidar_right_risk = 0.0
        self._lidar_map_metrics = {
            "front_min_m": 99.0,
            "left_min_m": 99.0,
            "right_min_m": 99.0,
            "front_risk": 0.0,
            "left_risk": 0.0,
            "right_risk": 0.0,
            "occupied_cells": 0,
        }
        self._blocked_hysteresis = False
        self._blocked_level = 0.0
        self._obstacle_threat_active = False
        self._obstacle_threat_source = "none"
        self._avoid_side_lock_until = 0.0
        self._avoid_switch_count = 0
        self._final_speed_limiter = "init"
        self._thrust_boost_until = 0.0
        self._p1_avoid_smooth = 0.0
        self._p2_avoid_smooth = 0.0
        self._escape_lock_side = None
        self._escape_clear_since = None
        self._p2_warn_latch = False
        self._front_min_smooth = 99.0
        self._yellow_seen_since = None
        self._orange_seen_since = None
        self._tracking_loss_since = None
        self._avoid_recovery_samples = deque(maxlen=20)
        self._avoid_recovery_count = 0
        self._avoid_recovery_last_ts = 0.0

        self.fusion_policy = "concurrent_lidar_camera"
        self.ghost_gate_count = 0
        self.ghost_target_count = 0
        self.ghost_yellow_count = 0
        self.ghost_orange_count = 0
        self.last_confirmed_gate_dist_m = None
        self.last_confirmed_target_dist_m = None
        self.last_confirmed_yellow_dist_m = None
        self.last_confirmed_orange_dist_m = None
        self._last_gate_confirm_ts = 0.0
        self._last_target_confirm_ts = 0.0
        self._last_yellow_confirm_ts = 0.0
        self._last_orange_confirm_ts = 0.0
        self._gate_fusion_hold_since = None
        self._target_fusion_hold_since = None
        self._yellow_fusion_hold_since = None
        self._orange_fusion_hold_since = None
        self._gate_ghost_latched = False
        self._target_ghost_latched = False
        self._yellow_ghost_latched = False
        self._orange_ghost_latched = False
        self._gate_camera_only_since = None
        self._target_camera_only_since = None
        self._yellow_camera_only_since = None
        self._orange_camera_only_since = None
        self._p2_local_minima_start_ts = None
        self._p2_local_minima_active = False
        self.dynamic_speed_profile = {
            "enabled": bool(DYN_SPEED_ENABLED),
            "mode": "banded",
            "scope": list(DYN_SPEED_SCOPE),
            "active": False,
            "band": "straight",
            "factor": float(DYN_SPEED_FACTOR_STRAIGHT),
            "heading_error_abs_deg": 0.0,
            "base_speed_mps": 0.0,
            "output_speed_mps": 0.0,
        }
        self._wind_i_state_deg_s = 0.0
        self._wind_bias_deg = 0.0
        self._wind_last_ts = time.monotonic()
        self.wind_assist = {
            "enabled": bool(WIND_ASSIST_ENABLED),
            "mode": "integral_crab_bias",
            "scope": list(WIND_ASSIST_SCOPE),
            "active": False,
            "reason": "idle",
            "i_gain": float(WIND_ASSIST_I_GAIN),
            "bias_deg": 0.0,
            "bias_max_deg": float(WIND_ASSIST_BIAS_MAX_DEG),
            "heading_error_abs_deg": 0.0,
            "corrected_heading_error_deg": 0.0,
        }
        self.hold_reason = "NONE"
        self.mission_end_reason = "none"
        self._anchor_center_lat = None
        self._anchor_center_lon = None
        self._anchor_pulse_until_ts = 0.0
        self._anchor_last_pulse_ts = 0.0
        self._anchor_last_heading_err_deg = 0.0
        self._anchor_breach_count = 0
        self._anchor_pulse_count = 0
        self._anchor_was_inside_fence = True
        self.virtual_anchor = {
            "enabled": bool(GEOFENCE_ENABLED),
            "mode": "geofence_virtual_anchor",
            "active": False,
            "reason": "idle",
            "failsafe_only": bool(GEOFENCE_FAILSAFE_ONLY),
            "fence_radius_m": float(GEOFENCE_RADIUS_M),
            "drift_trigger_m": float(GEOFENCE_DRIFT_TRIGGER_M),
            "drift_from_center_m": 0.0,
            "inside_fence": True,
            "anchor_set": False,
            "anchor_lat": None,
            "anchor_lon": None,
            "pulse_speed_mps": 0.0,
            "pulse_heading_error_deg": 0.0,
            "pulse_count": 0,
            "breach_count": 0,
        }
        self.horizon_lock = {
            "enabled": bool(HORIZON_LOCK_ENABLED),
            "mode": "imu_horizon_lock",
            "scope": list(HORIZON_LOCK_SCOPE),
            "active": False,
            "reason": "idle",
            "channel": "none",
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "raw_bearing_deg": 0.0,
            "correction_deg": 0.0,
            "corrected_bearing_deg": 0.0,
            "roll_gain": float(HORIZON_LOCK_ROLL_GAIN),
            "pitch_gain": float(HORIZON_LOCK_PITCH_GAIN),
            "max_correction_deg": float(HORIZON_LOCK_MAX_CORRECTION_DEG),
        }
        self.autonomy_health = {
            "enabled": bool(TRUST_BAR_ENABLED),
            "trust_score": 0.0,
            "level": "low",
            "color": "red",
            "label": "DUSUK",
            "advisory": "Otonomi guveni dusuk, RC kumandayi ele alin.",
            "failsafe_state": "normal",
            "gps_satellites_visible": 0,
            "gps_fix_type": 0,
            "camera_mode": "normal",
            "lidar_point_count": 0,
            "rc_link_active": False,
            "component_scores": {
                "gps": 0.0,
                "camera": 0.0,
                "lidar": 0.0,
                "rc": 0.0,
            },
            "weights": {
                "gps": float(TRUST_WEIGHT_GPS),
                "camera": float(TRUST_WEIGHT_CAMERA),
                "lidar": float(TRUST_WEIGHT_LIDAR),
                "rc": float(TRUST_WEIGHT_RC),
            },
        }

        self.last_heartbeat_time = time.monotonic()
        self.last_gcs_heartbeat_mono = None
        self.heartbeat_age_s = 0.0
        self.link_heartbeat_age_s = 0.0
        self.link_heartbeat_source = "onboard_fallback"
        self._last_idle_pixhawk_sync_mono = time.monotonic()
        self.failsafe_state = "normal"
        self.timeout_count = 0
        self.error_counters = {
            "state_write_error": 0,
            "mav_read_error": 0,
            "camera_state_read_error": 0,
            "link_state_read_error": 0,
        }
        self._warn_last = {}
        self._last_race_flag_purge_log = 0.0

        self.estop_latched = False
        self.estop_source = ""

        self.nav_waypoints = []    # Tüm ara nav waypoint'leri (GPS + lidar + kamera)
        self.engage_wp = None       # Son hedef waypoint (angajman noktası)
        # Legacy compat attributes — kept so telemetry/dashboard reads don't error
        self.waypoints_p1 = self.nav_waypoints
        self.waypoints_p2 = []
        self.waypoints_p3 = []
        self.target_color = ""
        self.mission_input_format = MISSION_INPUT_FORMAT
        self.mission_upload_source = "local_flat_file"
        self.mission_synced = False
        self.pixhawk_mission_count = 0
        self.pixhawk_mission_synced_at = None
        self.pixhawk_mission_last_hash = ""
        self._pixhawk_mission_download_hash = ""
        self._pixhawk_mission_upload_hash = ""
        self._pixhawk_mission_upload_attempt_hash = ""
        self.pixhawk_mission_sync_error = ""
        self.pixhawk_mirror_status = "unknown"
        self.pixhawk_mirror_error = ""
        self.post_start_mission_change_rejected = False
        self.mission_validated_at_timestamp = None
        self.mission_split_profile = get_mission_split_profile(0)
        self.mission_profile = {}
        self.mission_profile_valid = False
        self.mission_profile_race_ready = False
        self.mission_profile_error = "not_loaded"
        self.p2_min_gate_count = int(MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT)
        self.p3_engagement_mode = str(MISSION_PROFILE_DEFAULT_P3_ENGAGEMENT_MODE)
        self.phase_transition_policy = dict(MISSION_PROFILE_DEFAULT_TRANSITION_POLICY)
        self.gate_count = 0
        self._gate_event_start = None
        self._gate_event_latched = False
        self._gate_bearing_prev = None  # Track bearing sign for spec-compliant gate passage
        self.p3_wrong_target_avoidance = False
        self.p3_wrong_target_class = ""
        self.p3_contact_confirmation_source = "none"
        self._p3_wrong_target_last_log_ts = 0.0
        self._wp_target = "--"
        self._wp_info = "-- / --"
        self._active_waypoint_index = -1

        self.health_flags = {}
        self.health_missing = []
        self.health_storage = {}
        self.health_ready = False
        self.health_checked_at = 0.0

        self._cv2 = None
        self._file3_writer = None
        self._file3_index_file = None
        self._file3_index_writer = None
        self._file3_last_ts = 0.0

        # IO Caching state
        self._last_written_state = None
        self._last_state_write_time = 0.0

        self.master = None
        self._last_guided_command_time = 0.0
        self._last_guided_command = None
        self._last_companion_heartbeat_time = 0.0
        self._last_post_start_mission_check = 0.0
        self._race_p1_auto_completed_count = 0
        self._connect_pixhawk()
        self._try_connect_lidar()
        self._init_file3_recorder()
        self._refresh_health_check()
        self._update_mode_state(source="boot", reason="init")
        self._write_state()

    def _active_parkur_label(self):
        mapping = {
            self.STATE_IDLE: "IDLE",
            self.STATE_NAV: "NAV",
            self.STATE_ENGAGE: "ENGAGE",
            self.STATE_COMPLETED: "COMPLETED",
            self.STATE_HOLD: "HOLD",
        }
        return mapping.get(self.state, "UNKNOWN")

    def _get_objective_phase(self):
        mapping = {
            self.STATE_IDLE: "IDLE",
            self.STATE_NAV: "NAV_CORRIDOR",
            self.STATE_ENGAGE: "ENGAGE_TARGET",
            self.STATE_COMPLETED: "COMPLETED",
            self.STATE_HOLD: "HOLD",
        }
        return mapping.get(self.state, "UNKNOWN")

    def _get_perception_policy(self):
        active_label = self._active_parkur_label()
        return {
            "gate": active_label == "NAV",            # Kapi hizalama tum nav waypointlerde aktif
            "yellow_obstacle": active_label == "NAV", # Sari engel kacınma tum nav waypointlerde aktif
            "orange_boundary": active_label == "NAV", # Turuncu sinir dubasi — temas yasak
            "target": active_label == "ENGAGE",       # HSV hedef tespiti sadece angajman fazinda
        }

    def _get_guidance_source(self):
        """
        Return explicit guidance source identifying execution mode.

        Returns:
        - "nav_pixhawk_auto": Race mode NAV (Pixhawk autonomous)
        - "nav_pi_guided":    Test mode NAV (Pi-based guidance)
        - "engage_pi_guided": ENGAGE guidance (Pi in both modes)
        - "idle": Not in active mission
        """
        if self.state == self.STATE_NAV:
            if USV_MODE == USV_MODE_RACE and self.vehicle_mode_name == "AUTO":
                return "nav_pixhawk_auto"
            return "nav_pi_guided"
        elif self.state == self.STATE_ENGAGE:
            return "engage_pi_guided"
        else:
            return "idle"

    def _set_guidance_source(self, source):
        self.guidance_detail_source = str(source or "idle")

    def _set_guidance_idle(self, reason="idle"):
        self.guidance_mode = "idle"
        self.guidance_reason = str(reason or "idle")
        self.guidance_detail_source = "idle"
        self.avoidance_bias_deg = 0.0
        self.cross_track_error_m = 0.0
        self.cross_track_corr_deg = 0.0
        self.cross_track_align_active = False
        self.nominal_heading_deg = 0.0
        self.gate_assist_bias_deg = 0.0
        self.progress_along_leg_m = 0.0
        self.waypoint_passed_gate = False
        self.waypoint_accept_reason = "idle"
        self.motor_limit_reason = "neutral"
        self._heading_cmd_filtered_deg = 0.0
        self._heading_cmd_last_ts = time.monotonic()
        self._avoid_bias_filtered_deg = 0.0
        self._avoid_bias_last_ts = time.monotonic()
        self._blocked_hysteresis = False
        self._blocked_level = 0.0
        self._final_speed_limiter = "idle"
        self._avoid_recovery_samples.clear()

    def _guidance_reason_with_lidar_block(self):
        reason = str(self.guidance_reason or "idle")
        if self.mission_active and self._lidar_stale_blocked:
            return f"{reason};lidar_stale_blocked"
        return reason

    def _record_guidance_decision(self, decision: GuidanceDecision):
        self.guidance_mode = str(decision.mode)
        self.guidance_reason = str(decision.reason)
        self.avoidance_bias_deg = round(float(decision.avoidance_bias_deg), 3)
        self.cross_track_error_m = round(float(decision.cross_track_error_m), 3)
        self.nominal_heading_deg = round(float(getattr(decision, "nominal_heading_deg", 0.0)), 3)
        self.gate_assist_bias_deg = round(float(getattr(decision, "gate_assist_bias_deg", 0.0)), 3)
        self.motor_limit_reason = str(decision.limit_reason)
        self._set_guidance_source(decision.mode)

    def _log_nav_invalid(self, target_lat, target_lon):
        self._warn_throttled(
            "nav_invalid",
            (
                f"[WARN] [NAV] invalid_fix source={self.nav_position_source} "
                f"target=({float(target_lat):.7f},{float(target_lon):.7f})"
            ),
            period_s=1.5,
        )

    def _resolve_nav_solution(self):
        lat = float(self.current_lat)
        lon = float(self.current_lon)
        heading = float(self.current_heading)
        source = "mavlink"
        heading_source = "mavlink"
        source_detail = "mavlink_gps"
        nav_state_age_s = 0.0
        fix_valid = bool(
            (self.gps_fix_type >= 2 or self._gps_position_valid())
            and -90.0 <= lat <= 90.0
            and -180.0 <= lon <= 180.0
            and (abs(lat) > 1e-6 or abs(lon) > 1e-6)
        )

        if self.simulation_mode or os.environ.get("USV_SIM") == "1":
            nav = load_sim_nav_state(control_dir=CONTROL_DIR)
            nav_state_age = nav.get("state_age_s")
            try:
                nav_state_age_s = float(nav_state_age) if nav_state_age is not None else None
            except (TypeError, ValueError):
                nav_state_age_s = None
            # ts_epoch==0 in vehicle_position.json yields age=None; 999s blocks
            # _wait_for_stable_nav_solution (needs age<=0.35) despite valid lat/lon.
            if nav_state_age_s is None:
                nav_state_age_s = 0.0 if nav.get("valid") else 999.0
            source_detail = str(nav.get("source", "sim_nav_state") or "sim_nav_state")
            heading_source = str(nav.get("heading_source", "sim_nav_state") or "sim_nav_state")
            if nav.get("valid"):
                lat = float(nav.get("lat", lat))
                lon = float(nav.get("lon", lon))
                heading = float(nav.get("heading_deg", heading))
                try:
                    self.current_yaw_rate_dps = float(nav.get("observed_yaw_rate_dps", self.current_yaw_rate_dps) or 0.0)
                except (TypeError, ValueError):
                    self.current_yaw_rate_dps = 0.0
                source = "sim_nav_state"
                fix_valid = True

        self.nav_position_source = source
        self.nav_heading_source = heading_source
        self.nav_source_detail = source_detail
        self.nav_state_age_s = round(float(nav_state_age_s), 3)
        self.nav_fix_valid = bool(fix_valid)
        self.nav_solution_source = source
        self._nav_snapshot = {
            "lat": float(lat),
            "lon": float(lon),
            "heading": float(heading),
            "valid": bool(fix_valid),
            "source": str(source),
            "heading_source": str(heading_source),
            "source_detail": str(source_detail),
            "age_s": float(nav_state_age_s),
        }
        if source == "sim_nav_state":
            self.current_lat = lat
            self.current_lon = lon
            self.current_heading = heading
        return lat, lon, heading

    def _nav_snapshot_copy(self):
        snap = self._nav_snapshot if isinstance(self._nav_snapshot, dict) else {}
        return {
            "lat": float(snap.get("lat", self.current_lat) or 0.0),
            "lon": float(snap.get("lon", self.current_lon) or 0.0),
            "heading": float(snap.get("heading", self.current_heading) or 0.0),
            "valid": bool(snap.get("valid", False)),
            "source": str(snap.get("source", self.nav_position_source) or self.nav_position_source),
            "heading_source": str(snap.get("heading_source", self.nav_heading_source) or self.nav_heading_source),
            "source_detail": str(snap.get("source_detail", self.nav_source_detail) or self.nav_source_detail),
            "age_s": (
                float(snap["age_s"])
                if "age_s" in snap
                else float(self.nav_state_age_s)
            ),
        }

    def _wait_for_stable_nav_solution(self, samples_required=5, timeout_s=3.0):
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        stable = 0
        last_good = None
        while time.monotonic() < deadline and self.mission_active:
            self._drain_mav_messages()
            self._resolve_nav_solution()
            snap = self._nav_snapshot_copy()
            if snap["valid"] and snap["age_s"] <= 0.35:
                stable += 1
                last_good = snap
                if stable >= int(samples_required):
                    return snap
            else:
                stable = 0
            time.sleep(0.05)
        return last_good

    def _rc_link_active(self):
        if self.simulation_mode or os.environ.get("USV_SIM") == "1":
            return True
        return any(900 <= self.rc_channels.get(f"ch{i}", 0) <= 2100 for i in range(1, 5))

    def _is_rc_stick_active(self):
        """
        Detect if RC sticks are being actively manipulated (manual override).
        
        Returns True if:
        - CH1 (steering) is deflected ±50 from neutral (1450-1550)
        - CH3 (throttle) is deflected ±50 from neutral (1450-1550)
        
        Deadband of ±50 PWM allows for slight drift without triggering override.
        This implements AGENTS.md Section 2.1: RC override ALWAYS preempts autonomy.
        """
        RC_NEUTRAL = 1500
        RC_DEADBAND = 50  # ±50 PWM from neutral
        
        ch1_pwm = int(self.rc_channels.get("ch1", 0) or 0)
        ch3_pwm = int(self.rc_channels.get("ch3", 0) or 0)

        # In SITL, RC_CHANNELS often reflect the last override we injected
        # ourselves. Treating those echoed values as a human stick movement
        # preempts autonomy continuously and masks the real command path.
        if self.simulation_mode or os.environ.get("USV_SIM") == "1":
            if not SIM_ALLOW_RC_OVERRIDE:
                return False
            last = self._last_sent_rc_override if isinstance(self._last_sent_rc_override, dict) else {}
            last_ts = float(last.get("ts", 0.0) or 0.0)
            if last_ts > 0.0:
                echoed = (
                    abs(ch1_pwm - int(last.get("ch1", RC_NEUTRAL) or RC_NEUTRAL)) <= 12
                    and abs(ch3_pwm - int(last.get("ch3", RC_NEUTRAL) or RC_NEUTRAL)) <= 12
                    and (time.monotonic() - last_ts) <= 1.0
                )
                if echoed:
                    return False

        ch1 = abs(ch1_pwm - RC_NEUTRAL)
        ch3 = abs(ch3_pwm - RC_NEUTRAL)
        
        stick_active = (ch1 > RC_DEADBAND) or (ch3 > RC_DEADBAND)
        return stick_active

    def _is_rc_neutral(self):
        try:
            ch1 = int(self.rc_channels.get("ch1", 1500) or 1500)
            ch3 = int(self.rc_channels.get("ch3", 1500) or 1500)
        except Exception:
            return True
        return abs(ch1 - 1500) <= 50 and abs(ch3 - 1500) <= 50

    def _update_mode_state(self, source="runtime", reason=""):
        now = time.monotonic()
        rc_override = self._is_rc_stick_active()
        if rc_override:
            self._manual_override_active = True
            self._rc_neutral_since = None
        elif self._rc_neutral_since is None:
            self._rc_neutral_since = now

        neutral_dwell_s = 0.0
        if self._rc_neutral_since is not None:
            neutral_dwell_s = max(0.0, now - float(self._rc_neutral_since))

        auto_request = bool(self.mission_active and not self.command_lock)
        failsafe_hard = str(getattr(self, "failsafe_state", "normal") or "normal") in ("triggered", "hold")
        actual_vehicle_mode = str(self.vehicle_mode_name or "UNKNOWN")
        fc_autonomous_mode = actual_vehicle_mode in ("AUTO", "GUIDED")
        fc_mode_blocked = bool(
            auto_request
            and self.master is not None
            and not self._autonomy_pwm_via_rc_override()
            and not fc_autonomous_mode
        )
        auto_resume_allowed = bool(
            auto_request
            and self.health_ready
            and not self.estop_latched
            and not failsafe_hard
            and not fc_mode_blocked
        )
        auto_degraded_allowed = bool(
            auto_request
            and not self.estop_latched
            and not failsafe_hard
            and not self.command_lock
            and not fc_mode_blocked
        )
        wait_neutral_dwell = bool(
            self._manual_override_active
            and (not rc_override)
            and neutral_dwell_s < float(MODE_NEUTRAL_DWELL_S)
        )

        canonical_mode = "SAFE_HOLD"
        canonical_source = "safety"
        canonical_reason = "safe_hold"
        if self.estop_latched:
            canonical_mode = "ESTOP"
            canonical_source = "estop"
            canonical_reason = "estop_latched"
        elif rc_override:
            canonical_mode = "RC_OVERRIDE"
            canonical_source = "rc"
            canonical_reason = "rc_stick_active"
        elif auto_resume_allowed and not wait_neutral_dwell:
            canonical_mode = "AUTO"
            canonical_source = "autonomy"
            canonical_reason = "autonomy_control"
            self._manual_override_active = False
        elif auto_degraded_allowed and not wait_neutral_dwell:
            canonical_mode = "AUTO"
            canonical_source = "autonomy"
            canonical_reason = "degraded:not_ready"
        elif wait_neutral_dwell:
            canonical_mode = "SAFE_HOLD"
            canonical_source = "safety"
            canonical_reason = "rc_neutral_dwell"
        elif not auto_request:
            canonical_reason = "mission_inactive"
        elif self.command_lock:
            canonical_reason = "command_lock"
        elif fc_mode_blocked:
            canonical_reason = f"fc_mode_not_autonomous:{actual_vehicle_mode}"
        elif not self.health_ready:
            canonical_reason = "not_ready"

        reason_text = str(reason or canonical_reason)
        self.mode_state = {
            "canonical_mode": str(canonical_mode),
            "source": str(canonical_source),
            "reason": str(reason_text),
            "armed": bool(self.vehicle_armed),
            "vehicle_mode": str(self.vehicle_mode_name),
            "mission_active": bool(self.mission_active),
            "command_lock": bool(self.command_lock),
            "ready_state": bool(self.health_ready),
            "estop_state": bool(self.estop_latched),
            "auto_request": bool(auto_request),
            "auto_resume_allowed": bool(auto_resume_allowed),
            "degraded": bool(canonical_mode == "AUTO" and not self.health_ready),
            "neutral_dwell_s": round(float(neutral_dwell_s), 3),
            "ts_monotonic": round(float(now), 3),
        }
        signature = (
            self.mode_state["canonical_mode"],
            self.mode_state["source"],
            self.mode_state["reason"],
            self.mode_state["armed"],
            self.mode_state["vehicle_mode"],
        )
        if signature != self._mode_state_last_signature:
            self._mode_state_last_signature = signature
            if (now - float(self._mode_state_last_log_ts or 0.0)) >= 0.1:
                print(
                    "[MODE_STATE] mode={} src={} reason={} armed={} vehicle_mode={}".format(
                        self.mode_state["canonical_mode"],
                        self.mode_state["source"],
                        self.mode_state["reason"],
                        self.mode_state["armed"],
                        self.mode_state["vehicle_mode"],
                    )
                )
                self._mode_state_last_log_ts = now
        return dict(self.mode_state)

    def _apply_heading_deadzone_hysteresis(self, heading_error_deg):
        err = float(heading_error_deg)
        err_abs = abs(err)
        if self._heading_zero_latched:
            if err_abs <= float(HEADING_HYST_ENTER_DEG):
                return 0.0
            self._heading_zero_latched = False
        if err_abs <= max(float(HEADING_DEADZONE_DEG), float(HEADING_HYST_EXIT_DEG)):
            self._heading_zero_latched = True
            return 0.0
        return float(err)

    def _apply_wrong_turn_guard(self, heading_error_deg, speed_mps):
        err = float(heading_error_deg)
        speed = max(0.0, float(speed_mps))
        yaw_rate = float(self.current_yaw_rate_dps or 0.0)
        err_abs = abs(err)
        yaw_abs = abs(yaw_rate)
        if speed <= 0.01:
            self._wrong_turn_since = None
            self._wrong_turn_active = False
            return err, speed, False
        if bool(NAV_STRICT_HEADING_FIRST) and (
            str(getattr(self, "_nav_align_mode", "align")) == "align"
            or err_abs > float(NAV_ALIGN_HEADING_DONE_DEG)
        ):
            self._wrong_turn_since = None
            self._wrong_turn_active = False
            return err, speed, False
        # Large post-waypoint turns can briefly look like a wrong turn while yaw catches up.
        if err_abs >= 28.0:
            self._wrong_turn_since = None
            self._wrong_turn_active = False
            return err, speed, False
        mismatch = False
        if err_abs >= float(HEADING_WRONG_TURN_ERR_DEG) and yaw_abs >= float(HEADING_WRONG_TURN_YAWRATE_DPS):
            mismatch = (err > 0.0 and yaw_rate < 0.0) or (err < 0.0 and yaw_rate > 0.0)

        now = time.monotonic()
        if mismatch:
            if self._wrong_turn_since is None:
                self._wrong_turn_since = now
        else:
            self._wrong_turn_since = None
            self._wrong_turn_active = False

        guard_active = False
        if self._wrong_turn_since is not None:
            guard_active = (now - float(self._wrong_turn_since)) >= float(HEADING_WRONG_TURN_TRIGGER_S)

        if guard_active:
            if not self._wrong_turn_active:
                self._wrong_turn_count += 1
            self._wrong_turn_active = True
            if not self._autonomy_pwm_via_rc_override():
                hold_speed = 0.0 if os.environ.get("USV_SIM") == "1" else float(speed)
                diagnostic = (
                    "wrong_turn_hold:sim_guided_yaw_only"
                    if os.environ.get("USV_SIM") == "1"
                    else "wrong_turn_observed:guided_autopilot_control"
                )
                self.heading_control_diagnostic = diagnostic
                if (now - float(self._wrong_turn_last_log_ts or 0.0)) >= 0.5:
                    mode_label = "sim_guided_yaw_hold" if hold_speed <= 0.0 else "guided diagnostic_only"
                    print(
                        "[WRONG_TURN] observed err={:.1f} yaw_rate={:.2f} speed={:.2f}->{:.2f} count={} mode={}".format(
                            err,
                            yaw_rate,
                            speed,
                            hold_speed,
                            int(self._wrong_turn_count),
                            mode_label,
                        )
                    )
                    self._wrong_turn_last_log_ts = now
                if hold_speed <= 0.0:
                    return float(err), 0.0, True
                return float(err), float(speed), False
            guarded_err = 0.0
            guarded_speed = 0.0
            self.heading_control_diagnostic = "wrong_turn_hold:calibration_needed"
            if (now - float(self._wrong_turn_last_log_ts or 0.0)) >= 0.5:
                print(
                    "[WRONG_TURN] hold active err={:.1f} yaw_rate={:.2f} speed={:.2f}->{:.2f} count={} diagnostic=calibration_needed".format(
                        err,
                        yaw_rate,
                        speed,
                        guarded_speed,
                        int(self._wrong_turn_count),
                    )
                )
                self._wrong_turn_last_log_ts = now
            return float(guarded_err), float(guarded_speed), True

        if str(self.heading_control_diagnostic).startswith("wrong_turn_hold") or str(self.heading_control_diagnostic).startswith("wrong_turn_observed"):
            self.heading_control_diagnostic = "nominal"
        return float(err), float(speed), False

    def _sync_motor_controller_state(self):
        if not hasattr(self, "motor_controller"):
            return
        snap = self.motor_controller.snapshot()
        self._mixer_prev_left_pwm = float(snap["prev_left_pwm"])
        self._mixer_prev_right_pwm = float(snap["prev_right_pwm"])
        self._mixer_prev_left_rate = float(snap["prev_left_rate"])
        self._mixer_prev_right_rate = float(snap["prev_right_rate"])
        self._mixer_last_ts = float(snap["last_ts"])
        self._trim_port_us = float(snap["trim_port_us"])
        self._trim_stbd_us = float(snap["trim_stbd_us"])
        self._pwm_trim_last_ts = float(snap["trim_last_ts"])

    def _reset_mixer_transient(self, keep_trim=True):
        if hasattr(self, "motor_controller"):
            self.motor_controller.reset(keep_trim=bool(keep_trim))
            self._sync_motor_controller_state()
            return
        neutral = int(PWM_NEUTRAL_US)
        self._mixer_prev_left_pwm = neutral
        self._mixer_prev_right_pwm = neutral
        self._mixer_prev_left_rate = 0.0
        self._mixer_prev_right_rate = 0.0
        self._mixer_last_ts = time.monotonic()
        if not keep_trim:
            self._trim_port_us = 0.0
            self._trim_stbd_us = 0.0

    def _apply_pwm_deadband_min_effective(self, pwm_us):
        if hasattr(self, "motor_controller"):
            return self.motor_controller.apply_deadband_min_effective(pwm_us)
        neutral = int(PWM_NEUTRAL_US)
        pwm = int(round(float(pwm_us)))
        delta = int(pwm - neutral)
        if abs(delta) <= int(PWM_DEADBAND_US):
            return neutral
        if abs(delta) < int(PWM_MIN_EFFECTIVE_US):
            delta = int(PWM_MIN_EFFECTIVE_US if delta > 0 else -PWM_MIN_EFFECTIVE_US)
        return int(clamp(neutral + delta, int(PWM_MIN_US), int(PWM_MAX_US)))

    def _update_pwm_trim(self, heading_error_deg, yaw_norm):
        now = time.monotonic()
        dt = max(0.01, min(0.30, now - float(self._pwm_trim_last_ts or now)))
        self._pwm_trim_last_ts = now
        steady_heading = abs(float(heading_error_deg)) <= float(PWM_TRIM_ERR_BAND_DEG)
        light_turn = abs(float(yaw_norm)) <= 0.15
        speed_ok = float(self.current_speed_mps) >= 0.2
        if self.mission_active and steady_heading and light_turn and speed_ok:
            step = float(self.current_yaw_rate_dps) * float(PWM_TRIM_LEARN_GAIN) * dt
            step = clamp(step, -1.2, 1.2)
            self._trim_port_us = float(clamp(
                self._trim_port_us + step,
                -float(PWM_TRIM_MAX_US),
                float(PWM_TRIM_MAX_US),
            ))
            self._trim_stbd_us = float(clamp(
                self._trim_stbd_us - step,
                -float(PWM_TRIM_MAX_US),
                float(PWM_TRIM_MAX_US),
            ))
        else:
            decay = max(0.0, min(1.0, dt * 0.35))
            keep = 1.0 - decay
            self._trim_port_us *= keep
            self._trim_stbd_us *= keep

    def _apply_pwm_slew_and_jerk(self, target_left_pwm, target_right_pwm):
        if hasattr(self, "motor_controller"):
            result = self.motor_controller.apply_slew_and_jerk(target_left_pwm, target_right_pwm)
            self._sync_motor_controller_state()
            return result
        now = time.monotonic()
        dt = max(0.01, min(0.30, now - float(self._mixer_last_ts or now)))
        self._mixer_last_ts = now
        max_step = float(PWM_SLEW_RATE_US_PER_S) * dt
        max_rate_delta = float(PWM_JERK_LIMIT_US_PER_S2) * dt

        left_step = clamp(float(target_left_pwm) - float(self._mixer_prev_left_pwm), -max_step, max_step)
        right_step = clamp(float(target_right_pwm) - float(self._mixer_prev_right_pwm), -max_step, max_step)

        left_rate_cmd = left_step / dt
        right_rate_cmd = right_step / dt
        left_rate = float(self._mixer_prev_left_rate) + clamp(
            left_rate_cmd - float(self._mixer_prev_left_rate),
            -max_rate_delta,
            max_rate_delta,
        )
        right_rate = float(self._mixer_prev_right_rate) + clamp(
            right_rate_cmd - float(self._mixer_prev_right_rate),
            -max_rate_delta,
            max_rate_delta,
        )

        left_pwm = float(self._mixer_prev_left_pwm) + (left_rate * dt)
        right_pwm = float(self._mixer_prev_right_pwm) + (right_rate * dt)

        if (target_left_pwm - self._mixer_prev_left_pwm) >= 0:
            left_pwm = min(left_pwm, float(target_left_pwm))
        else:
            left_pwm = max(left_pwm, float(target_left_pwm))
        if (target_right_pwm - self._mixer_prev_right_pwm) >= 0:
            right_pwm = min(right_pwm, float(target_right_pwm))
        else:
            right_pwm = max(right_pwm, float(target_right_pwm))

        left_pwm = float(clamp(left_pwm, float(PWM_MIN_US), float(PWM_MAX_US)))
        right_pwm = float(clamp(right_pwm, float(PWM_MIN_US), float(PWM_MAX_US)))

        self._mixer_prev_left_pwm = float(left_pwm)
        self._mixer_prev_right_pwm = float(right_pwm)
        self._mixer_prev_left_rate = float(left_rate)
        self._mixer_prev_right_rate = float(right_rate)
        return int(round(left_pwm)), int(round(right_pwm))

    def _harden_motor_allocation(self, allocation, heading_error_deg, speed_mps):
        if hasattr(self, "motor_controller"):
            left, right = self.motor_controller.harden_allocation(
                allocation=allocation,
                heading_error_deg=heading_error_deg,
                speed_mps=speed_mps,
                current_speed_mps=float(self.current_speed_mps or 0.0),
                current_yaw_rate_dps=float(self.current_yaw_rate_dps or 0.0),
                mission_active=bool(self.mission_active),
            )
            self._sync_motor_controller_state()
            return int(left), int(right)
        if abs(float(speed_mps)) <= 1e-3 and abs(float(heading_error_deg)) <= 1e-3:
            self._reset_mixer_transient(keep_trim=True)
            return int(PWM_NEUTRAL_US), int(PWM_NEUTRAL_US)

        self._update_pwm_trim(heading_error_deg=heading_error_deg, yaw_norm=getattr(allocation, "yaw_norm", 0.0))
        target_left = float(getattr(allocation, "left_pwm", PWM_NEUTRAL_US)) + float(self._trim_port_us)
        target_right = float(getattr(allocation, "right_pwm", PWM_NEUTRAL_US)) + float(self._trim_stbd_us)
        target_left = clamp(target_left, float(PWM_MIN_US), float(PWM_MAX_US))
        target_right = clamp(target_right, float(PWM_MIN_US), float(PWM_MAX_US))
        target_left = self._apply_pwm_deadband_min_effective(target_left)
        target_right = self._apply_pwm_deadband_min_effective(target_right)
        smooth_left, smooth_right = self._apply_pwm_slew_and_jerk(target_left, target_right)
        smooth_left = int(clamp(smooth_left, int(PWM_MIN_US), int(PWM_MAX_US)))
        smooth_right = int(clamp(smooth_right, int(PWM_MIN_US), int(PWM_MAX_US)))
        return smooth_left, smooth_right

    def _compute_trust_bar(self):
        """
        Return the current autonomy health trust score (0-100%).

        The detailed computation lives in `_update_autonomy_health()`. Keeping a
        second simplified trust implementation here caused simulation-only false
        negatives because sparse-but-healthy LiDAR scans were treated as a
        sensor fault and autonomy speed was capped continuously.
        """
        self._update_autonomy_health()
        trust_score = float(self.autonomy_health.get("trust_score", 0.0) or 0.0)
        self.trust_bar = round(trust_score, 1)
        return self.trust_bar

    def _warn_throttled(self, key, message, period_s=5.0):
        now = time.monotonic()
        last = self._warn_last.get(key, 0.0)
        if now - last >= period_s:
            print(message)
            self._warn_last[key] = now

    def _bump_error(self, key, message=None, period_s=5.0):
        self.error_counters[key] = int(self.error_counters.get(key, 0)) + 1
        if message:
            self._warn_throttled(f"err_{key}", f"{message} (count={self.error_counters[key]})", period_s=period_s)

    def _camera_bearing_to_lidar_angle(self, camera_bearing_deg):
        # Camera uses +right, lidar callback uses +left.
        return -float(camera_bearing_deg)

    def _nearest_lidar_distance_for_camera_bearing(self, camera_bearing_deg):
        if not self._lidar_front_samples:
            return None
        lidar_angle_deg = self._camera_bearing_to_lidar_angle(camera_bearing_deg)
        half_window = max(0.1, float(FUSION_BEARING_WINDOW_DEG))
        nearest = None
        for angle_deg, distance_m in self._lidar_front_samples:
            if abs(angle_deg - lidar_angle_deg) > half_window:
                continue
            if not (FUSION_LIDAR_MIN_VALID_M <= distance_m <= FUSION_LIDAR_CONFIRM_MAX_M):
                continue
            if nearest is None or distance_m < nearest:
                nearest = distance_m
        return nearest

    def _camera_lidar_obstacle_concurrent(self, *, raw_detected, bearing_deg):
        """True when camera and lidar both contribute for the same obstacle bearing."""
        if not bool(raw_detected):
            return False
        if not (bool(self.camera_ready) and bool(self.lidar_ready)):
            return False
        try:
            bearing = float(bearing_deg)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(bearing):
            return False
        return self._nearest_lidar_distance_for_camera_bearing(bearing) is not None

    def _camera_obstacle_confirmed(self, bearing_deg, area_norm, seen_since, *, min_dwell_s, strong_area_norm):
        try:
            bearing = float(bearing_deg)
            area = float(area_norm)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(bearing) or not math.isfinite(area) or area <= 0.0:
            return False
        nearest_lidar = self._nearest_lidar_distance_for_camera_bearing(bearing)
        if nearest_lidar is not None:
            return True
        if seen_since is None:
            return False
        dwell_s = time.monotonic() - float(seen_since)
        return bool(area >= float(strong_area_norm) and dwell_s >= float(min_dwell_s))

    def _fusion_log(self, key, message):
        self._warn_throttled(f"fusion_{key}", message, period_s=FUSION_LOG_PERIOD_S)

    def _dyn_speed_log(self, key, message):
        self._warn_throttled(f"dyn_speed_{key}", message, period_s=DYN_SPEED_LOG_PERIOD_S)

    def _wind_log(self, key, message):
        self._warn_throttled(f"wind_{key}", message, period_s=WIND_ASSIST_LOG_PERIOD_S)

    def _geo_log(self, key, message):
        self._warn_throttled(f"geo_{key}", message, period_s=GEOFENCE_LOG_PERIOD_S)

    def _horizon_log(self, key, message):
        self._warn_throttled(f"horizon_{key}", message, period_s=HORIZON_LOCK_LOG_PERIOD_S)

    def _trust_log(self, key, message):
        self._warn_throttled(f"trust_{key}", message, period_s=TRUST_LOG_PERIOD_S)

    def _update_autonomy_health(self):
        if not TRUST_BAR_ENABLED:
            self.autonomy_health = {
                "enabled": False,
                "trust_score": 0.0,
                "level": "off",
                "color": "gray",
                "label": "KAPALI",
                "advisory": "Trust bar devre disi.",
                "failsafe_state": self.failsafe_state,
                "gps_satellites_visible": int(self.gps_satellites_visible),
                "gps_fix_type": int(self.gps_fix_type),
                "camera_mode": str(self.camera_adaptation.get("mode", "normal")),
                "lidar_point_count": int(len(self._lidar_front_samples)),
                "rc_link_active": bool(self._rc_link_active()),
                "component_scores": {"gps": 0.0, "camera": 0.0, "lidar": 0.0, "rc": 0.0},
                "weights": {
                    "gps": float(TRUST_WEIGHT_GPS),
                    "camera": float(TRUST_WEIGHT_CAMERA),
                    "lidar": float(TRUST_WEIGHT_LIDAR),
                    "rc": float(TRUST_WEIGHT_RC),
                },
            }
            return

        sat = max(0, int(self.gps_satellites_visible or 0))
        fix = max(0, int(self.gps_fix_type or 0))
        rc_active = bool(self._rc_link_active())
        lidar_points = int(len(self._lidar_front_samples))
        cam_mode = str(self.camera_adaptation.get("mode", "normal"))

        if self.simulation_mode:
            sat = max(sat, 12)
            fix = max(fix, 4)
            gps_q = 1.0
            camera_q = 1.0 if bool(self.camera_ready) else 0.35
            if bool(self.lidar_ready) and lidar_points > 0:
                points_score = clamp(float(lidar_points) / float(max(1, TRUST_LIDAR_POINTS_FULL)), 0.0, 1.0)
                lidar_q = clamp(0.35 + (0.65 * points_score), 0.0, 1.0)
            else:
                lidar_q = 0.2
            rc_q = 1.0 if rc_active else 0.5
        else:
            sat_norm = clamp((float(sat) - 4.0) / 8.0, 0.0, 1.0)
            if fix >= 4:
                fix_norm = 1.0
            elif fix == 3:
                fix_norm = 0.8
            elif fix == 2:
                fix_norm = 0.45
            else:
                fix_norm = 0.1
            pos_valid = 1.0 if self._gps_position_valid() else 0.0
            gps_q = clamp((0.55 * sat_norm) + (0.35 * fix_norm) + (0.10 * pos_valid), 0.0, 1.0)

            if not self.camera_ready:
                camera_q = 0.0
            else:
                if cam_mode == "normal":
                    mode_score = 1.0
                elif cam_mode in ("dark", "bright"):
                    mode_score = 0.72
                else:
                    mode_score = 0.85
                luma = float(self.camera_adaptation.get("luma_mean", 0.0) or 0.0)
                if luma <= 0.0:
                    luma_score = 0.4
                else:
                    luma_score = clamp(1.0 - abs(luma - 128.0) / 128.0, 0.0, 1.0)
                camera_q = clamp((0.60 * mode_score) + (0.40 * luma_score), 0.0, 1.0)

            if not self.lidar_ready or lidar_points <= 0:
                lidar_q = 0.0
            else:
                points_score = clamp(float(lidar_points) / float(max(1, TRUST_LIDAR_POINTS_FULL)), 0.0, 1.0)
                lidar_q = clamp(0.55 + (0.45 * points_score), 0.0, 1.0)

            if not rc_active:
                rc_q = 0.0
            else:
                age_score = clamp(1.0 - (float(self.link_heartbeat_age_s) / float(max(1.0, HEARTBEAT_WARN_S))), 0.0, 1.0)
                rc_q = clamp(0.40 + (0.60 * age_score), 0.0, 1.0)

        base_score = (
            float(TRUST_WEIGHT_GPS) * gps_q
            + float(TRUST_WEIGHT_CAMERA) * camera_q
            + float(TRUST_WEIGHT_LIDAR) * lidar_q
            + float(TRUST_WEIGHT_RC) * rc_q
        )
        safety_factor = 1.0
        if self.failsafe_state == "warning":
            safety_factor = 0.75
        elif self.failsafe_state in ("triggered", "hold"):
            safety_factor = 0.40
        trust_score = clamp(base_score * safety_factor * 100.0, 0.0, 100.0)

        if trust_score >= float(TRUST_GOOD_THRESHOLD):
            level, color, label = "high", "green", "YUKSEK"
            advisory = "Otonomi guveni yuksek."
        elif trust_score >= float(TRUST_WARN_THRESHOLD):
            level, color, label = "medium", "yellow", "ORTA"
            advisory = "Dikkat: Otonomi guveni orta, RC kumandayi hazir tutun."
        else:
            level, color, label = "low", "red", "DUSUK"
            advisory = "Otonomi guveni dusuk, RC kumandayi ele alin."

        self.autonomy_health = {
            "enabled": True,
            "trust_score": round(float(trust_score), 2),
            "level": level,
            "color": color,
            "label": label,
            "advisory": advisory,
            "failsafe_state": self.failsafe_state,
            "gps_satellites_visible": int(sat),
            "gps_fix_type": int(fix),
            "camera_mode": cam_mode,
            "lidar_point_count": int(lidar_points),
            "rc_link_active": bool(rc_active),
            "component_scores": {
                "gps": round(float(gps_q * 100.0), 2),
                "camera": round(float(camera_q * 100.0), 2),
                "lidar": round(float(lidar_q * 100.0), 2),
                "rc": round(float(rc_q * 100.0), 2),
            },
            "weights": {
                "gps": float(TRUST_WEIGHT_GPS),
                "camera": float(TRUST_WEIGHT_CAMERA),
                "lidar": float(TRUST_WEIGHT_LIDAR),
                "rc": float(TRUST_WEIGHT_RC),
            },
        }

        self._trust_log(
            level,
            (
                f"[TRUST] score={trust_score:.1f} level={label} "
                f"gps={gps_q*100:.0f} cam={camera_q*100:.0f} lidar={lidar_q*100:.0f} rc={rc_q*100:.0f}"
            ),
        )

    def _set_horizon_lock_idle(self, reason="idle", channel="none"):
        self.horizon_lock = {
            "enabled": bool(HORIZON_LOCK_ENABLED),
            "mode": "imu_horizon_lock",
            "scope": list(HORIZON_LOCK_SCOPE),
            "active": False,
            "reason": str(reason),
            "channel": str(channel),
            "roll_deg": round(float(self.current_roll_deg), 3),
            "pitch_deg": round(float(self.current_pitch_deg), 3),
            "raw_bearing_deg": 0.0,
            "correction_deg": 0.0,
            "corrected_bearing_deg": 0.0,
            "roll_gain": float(HORIZON_LOCK_ROLL_GAIN),
            "pitch_gain": float(HORIZON_LOCK_PITCH_GAIN),
            "max_correction_deg": float(HORIZON_LOCK_MAX_CORRECTION_DEG),
        }

    def _apply_horizon_lock_to_bearing(self, raw_bearing_deg, parkur_label, channel, detected, update_state=True):
        try:
            raw_bearing = float(raw_bearing_deg)
        except (TypeError, ValueError):
            raw_bearing = 0.0
        roll_deg = float(self.current_roll_deg)
        pitch_deg = float(self.current_pitch_deg)

        active = False
        reason = "active"
        correction_deg = 0.0
        corrected_bearing = raw_bearing

        if not HORIZON_LOCK_ENABLED:
            reason = "disabled"
        elif parkur_label not in HORIZON_LOCK_SCOPE:
            reason = "out_of_scope"
        elif not self.mission_active:
            reason = "mission_inactive"
        elif not bool(detected):
            reason = "no_detection"
        elif max(abs(roll_deg), abs(pitch_deg)) < float(HORIZON_LOCK_MIN_TILT_DEG):
            reason = "tilt_small"
        else:
            raw_correction = (-(roll_deg * float(HORIZON_LOCK_ROLL_GAIN))) - (
                pitch_deg * float(HORIZON_LOCK_PITCH_GAIN)
            )
            correction_deg = float(
                clamp(raw_correction, -float(HORIZON_LOCK_MAX_CORRECTION_DEG), float(HORIZON_LOCK_MAX_CORRECTION_DEG))
            )
            corrected_bearing = raw_bearing + correction_deg
            active = True
            clamped = abs(raw_correction - correction_deg) > 1e-6
            if clamped:
                reason = "clamped"
                if update_state:
                    self._horizon_log(
                        f"{parkur_label}_{channel}_clamped",
                        (
                            f"[HORIZON] {parkur_label}/{channel} clamped "
                            f"roll={roll_deg:.1f} pitch={pitch_deg:.1f} raw={raw_bearing:.1f} corr={correction_deg:.1f}"
                        ),
                    )
            else:
                if update_state:
                    self._horizon_log(
                        f"{parkur_label}_{channel}_active",
                        (
                            f"[HORIZON] {parkur_label}/{channel} active "
                            f"roll={roll_deg:.1f} pitch={pitch_deg:.1f} raw={raw_bearing:.1f} corrected={corrected_bearing:.1f}"
                        ),
                    )

        if update_state:
            self.horizon_lock = {
                "enabled": bool(HORIZON_LOCK_ENABLED),
                "mode": "imu_horizon_lock",
                "scope": list(HORIZON_LOCK_SCOPE),
                "active": bool(active),
                "reason": reason,
                "channel": str(channel),
                "roll_deg": round(float(roll_deg), 3),
                "pitch_deg": round(float(pitch_deg), 3),
                "raw_bearing_deg": round(float(raw_bearing), 3),
                "correction_deg": round(float(correction_deg), 3),
                "corrected_bearing_deg": round(float(corrected_bearing), 3),
                "roll_gain": float(HORIZON_LOCK_ROLL_GAIN),
                "pitch_gain": float(HORIZON_LOCK_PITCH_GAIN),
                "max_correction_deg": float(HORIZON_LOCK_MAX_CORRECTION_DEG),
            }
        return float(corrected_bearing)

    @staticmethod
    def _geo_position_valid_values(lat, lon):
        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except (TypeError, ValueError):
            return False
        if not (-90.0 <= lat_f <= 90.0 and -180.0 <= lon_f <= 180.0):
            return False
        if abs(lat_f) < 1e-6 and abs(lon_f) < 1e-6:
            return False
        return True

    def _gps_position_valid(self):
        try:
            lat = float(self.current_lat)
            lon = float(self.current_lon)
        except (TypeError, ValueError):
            return False
        return self._geo_position_valid_values(lat, lon)

    def _recent_ekf_variance_warning(self):
        ts = float(getattr(self, "_last_ekf_variance_statustext_ts", 0.0) or 0.0)
        if ts <= 0.0:
            return False
        age_s = time.monotonic() - ts
        if age_s >= float(FC_EKF_VARIANCE_BLOCK_S):
            return False
        # ArduPilot "EKF variance" STATUSTEXT can arrive transiently during arm/NAV
        # while EKF_STATUS_REPORT variances are already healthy.
        if self.fc_ekf_report_seen and self._ekf_variances_within_limit() and not self._ekf_hard_fault_reason():
            return False
        return True

    @staticmethod
    def _classify_fc_start_blocker(text):
        low_text = str(text or "").lower()
        if "smartrtl" in low_text:
            return ""
        if "gyros inconsistent" in low_text and ("arm:" in low_text or "prearm:" in low_text):
            return "gyro_inconsistent_recent"
        if "accels inconsistent" in low_text and ("arm:" in low_text or "prearm:" in low_text):
            return "accel_inconsistent_recent"
        if "ekf variance" in low_text:
            return "ekf_variance_recent"
        if "bad position" in low_text and ("arm:" in low_text or "prearm:" in low_text):
            return "bad_position_recent"
        return ""

    @staticmethod
    def _fc_arm_blocker_ttl_s(reason):
        if str(reason or "") in _ARM_STATUSTEXT_BLOCKERS:
            return float(ARM_STATUSTEXT_BLOCK_S)
        return float(FC_START_STABLE_S)

    def _mark_fc_start_blocker(self, reason):
        reason = str(reason or "").strip()
        if not reason:
            return
        if not isinstance(getattr(self, "_fc_start_blocker_ts", None), dict):
            self._fc_start_blocker_ts = {}
        self._fc_start_blocker_ts[reason] = time.monotonic()
        self.fc_start_ready = False
        self.fc_start_block_reason = reason
        if reason in ("ekf_variance_recent", "bad_position_recent", "accel_inconsistent_recent", "gyro_inconsistent_recent"):
            self._last_ekf_variance_statustext_ts = time.monotonic()
            if reason in ("ekf_variance_recent", "bad_position_recent"):
                self.fc_nav_mode_ready = False
                self.fc_nav_mode_ready_since = None
                self.fc_guided_block_reason = (
                    "ekf_variance_recent" if reason == "ekf_variance_recent" else "bad_position_recent"
                )

    def _recent_fc_start_block_reason(self, now=None):
        now = float(now if now is not None else time.monotonic())
        ts_map = getattr(self, "_fc_start_blocker_ts", {})
        if not isinstance(ts_map, dict):
            return "none"
        recent = []
        for reason, ts in ts_map.items():
            try:
                age = now - float(ts or 0.0)
            except (TypeError, ValueError):
                continue
            if 0.0 <= age < self._fc_arm_blocker_ttl_s(reason):
                recent.append((age, str(reason)))
        if not recent:
            return "none"
        recent.sort(key=lambda item: item[0])
        return recent[0][1]

    def _fc_start_status(self, guided_prereq=None):
        if not isinstance(guided_prereq, dict):
            guided_prereq = self._guided_prereq_status()
        recent_block = self._recent_fc_start_block_reason()
        ready = bool(guided_prereq.get("ready", False) and recent_block == "none")
        if recent_block != "none":
            reason = recent_block
        elif not guided_prereq.get("ready", False):
            reason = str(guided_prereq.get("fc_guided_block_reason", "guided_prereq_missing") or "guided_prereq_missing")
        else:
            reason = "none"
        self.fc_start_ready = bool(ready)
        self.fc_start_block_reason = reason
        return {"ready": bool(ready), "block_reason": reason}

    def _message_age_s(self, mtype, now=None):
        ts_map = getattr(self, "_last_mavlink_message_ts", {})
        if not isinstance(ts_map, dict):
            return None
        try:
            ts = float(ts_map.get(str(mtype), 0.0) or 0.0)
        except (TypeError, ValueError):
            return None
        if ts <= 0.0:
            return None
        return max(0.0, float(now if now is not None else time.monotonic()) - ts)

    def _mavlink_message_ages(self, now=None):
        now = float(now if now is not None else time.monotonic())
        out = {}
        for mtype in ("HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "EKF_STATUS_REPORT", "SYS_STATUS", "RC_CHANNELS"):
            age = self._message_age_s(mtype, now=now)
            out[mtype] = round(age, 3) if age is not None else None
        return out

    def _ekf_variances_within_limit(self):
        values = self.fc_ekf_variances if isinstance(self.fc_ekf_variances, dict) else {}
        checked = []
        for key in ("velocity", "pos_horiz", "pos_vert", "compass"):
            try:
                val = float(values.get(key, 0.0) or 0.0)
            except (TypeError, ValueError):
                continue
            if math.isfinite(val):
                checked.append(val)
        return not checked or max(checked) <= float(FC_EKF_VARIANCE_MAX)

    def _ekf_hard_fault_reason(self):
        flags = int(self.fc_ekf_flags or 0)
        if flags <= 0:
            return ""
        # MAV_ESTIMATOR_TYPE_FLAGS hard blockers. POS_HORIZ_ABS absence and
        # CONST_POS_MODE remain diagnostics because ArduPilot can report them
        # transiently while GPS/GPI and variances are already healthy.
        gps_glitch = bool(flags & 1024)
        accel_error = bool(flags & 4096)
        if gps_glitch:
            if (
                os.environ.get("USV_SIM") == "1"
                and (time.monotonic() - float(getattr(self, "_boot_mono", 0.0) or 0.0))
                < float(SIM_EKF_GPS_GLITCH_GRACE_S)
            ):
                return ""
            return "gps_glitch"
        if accel_error:
            return "accel_error"
        return ""

    def _ekf_flag_diagnostics(self):
        flags = int(self.fc_ekf_flags or 0)
        return {
            "attitude": bool(flags & 1),
            "velocity_horiz": bool(flags & 2),
            "pos_horiz_rel": bool(flags & 8),
            "pos_horiz_abs": bool(flags & 16),
            "const_pos_mode": bool(flags & 128),
            "gps_glitch": bool(flags & 1024),
            "accel_error": bool(flags & 4096),
        }

    def _update_fc_nav_mode_ready(self, fc_position_ready=None):
        now = time.monotonic()
        gpi_age = self._message_age_s("GLOBAL_POSITION_INT", now=now)
        gps_age = self._message_age_s("GPS_RAW_INT", now=now)
        ekf_age = self._message_age_s("EKF_STATUS_REPORT", now=now)
        if fc_position_ready is None:
            fc_position_ready = bool(
                int(self.gps_fix_type or 0) >= 3
                and gpi_age is not None
                and gpi_age <= float(FC_GLOBAL_POSITION_FRESH_S)
                and gps_age is not None
                and gps_age <= float(FC_GPS_FRESH_S)
            )
        reason = "none"
        ready_instant = True
        if not fc_position_ready:
            ready_instant = False
            if gps_age is None or gps_age > float(FC_GPS_FRESH_S):
                reason = "gps_fix_stale"
            elif gpi_age is None or gpi_age > float(FC_GLOBAL_POSITION_FRESH_S):
                reason = "global_position_stale"
            else:
                reason = "fc_position_not_ready"
        elif self._recent_ekf_variance_warning():
            ready_instant = False
            reason = "ekf_variance_recent"
        elif self.fc_ekf_report_seen and ekf_age is not None and ekf_age > float(FC_EKF_FRESH_S):
            ready_instant = False
            reason = "ekf_report_stale"
        elif os.environ.get("USV_SIM") == "1" and not self.fc_ekf_report_seen:
            ready_instant = False
            reason = "ekf_status_report_missing"
        elif self.fc_ekf_report_seen and self._ekf_hard_fault_reason():
            ready_instant = False
            reason = "ekf_hard_fault"
        elif self.fc_ekf_report_seen and not self._ekf_variances_within_limit():
            ready_instant = False
            reason = "ekf_variance_high"

        if ready_instant:
            if self.fc_nav_mode_ready_since is None:
                self.fc_nav_mode_ready_since = now
            stable_for = now - float(self.fc_nav_mode_ready_since)
            if stable_for >= float(FC_NAV_READY_STABLE_S_EFFECTIVE):
                self.fc_nav_mode_ready = True
                self.fc_guided_block_reason = "none"
            else:
                self.fc_nav_mode_ready = False
                self.fc_guided_block_reason = "none"
        else:
            self.fc_nav_mode_ready_since = None
            self.fc_nav_mode_ready = False
            self.fc_guided_block_reason = reason
        return bool(self.fc_nav_mode_ready)

    def _guided_prereq_status(self):
        sim_mode = bool(self.simulation_mode or os.environ.get("USV_SIM") == "1")
        sim_nav = {}
        sim_nav_ok = False
        if sim_mode:
            sim_nav = load_sim_nav_state(control_dir=CONTROL_DIR)
            if sim_nav.get("valid"):
                sim_nav_ok = True
                self.current_lat = float(sim_nav.get("lat"))
                self.current_lon = float(sim_nav.get("lon"))
                heading = sim_nav.get("heading_deg")
                if heading is not None:
                    self.current_heading = float(heading)
        now = time.monotonic()
        message_ages = self._mavlink_message_ages(now=now)
        gpi_age = message_ages.get("GLOBAL_POSITION_INT")
        gps_age = message_ages.get("GPS_RAW_INT")
        heartbeat_age = float(self.link_heartbeat_age_s or 0.0)
        position_valid = bool(self._gps_position_valid())
        gps_fix_fresh = bool(gps_age is not None and gps_age <= float(FC_GPS_FRESH_S))
        gpi_fresh = bool(gpi_age is not None and gpi_age <= float(FC_GLOBAL_POSITION_FRESH_S))
        gps_fix_ok = bool(int(self.gps_fix_type or 0) >= 3 and gps_fix_fresh and position_valid)
        sim_global_ok = bool(
            sim_mode
            and self.gps_global_position_int_received
            and gpi_fresh
            and gps_fix_ok
        )
        # GUIDED/AUTO readiness must describe the flight controller, not only
        # the companion-computer sim_nav_state. sim_nav_state remains useful as
        # a navigation observation, but ArduPilot will not produce servo/PWM
        # output from GUIDED/AUTO unless its own global position path is valid.
        fc_position_ready = bool(sim_global_ok if sim_mode else (gps_fix_ok and gpi_fresh))
        fc_nav_ready = self._update_fc_nav_mode_ready(fc_position_ready=fc_position_ready)
        position_ready = bool(fc_position_ready and fc_nav_ready)
        position_source = (
            "mavlink_global_position" if fc_position_ready
            else "sim_nav_state_only" if sim_nav_ok
            else "invalid"
        )
        self.guided_position_source = position_source
        mavlink_ok = bool(self.master is not None)
        heartbeat_ok = bool(mavlink_ok and heartbeat_age < HEARTBEAT_WARN_S)
        ready = bool(mavlink_ok and heartbeat_ok and position_ready and not self.estop_latched)
        missing = []
        if not mavlink_ok:
            missing.append("mavlink")
        if not heartbeat_ok:
            missing.append("heartbeat_stale")
        if not position_ready:
            if sim_mode and not sim_nav_ok:
                missing.append("sim_nav_state")
            if not self.gps_global_position_int_received or not gpi_fresh:
                missing.append("global_position_stale")
            if int(self.gps_fix_type or 0) < 3 or not gps_fix_fresh:
                missing.append("gps_fix_stale")
            if not position_valid and not sim_nav_ok:
                missing.append("gps_position")
            if sim_mode and sim_nav_ok and (
                not self.gps_global_position_int_received or not gpi_fresh or int(self.gps_fix_type or 0) < 3 or not gps_fix_fresh
            ):
                missing.append("fc_gps_ekf_not_ready")
            if fc_position_ready and not fc_nav_ready:
                block_reason = str(self.fc_guided_block_reason or "none")
                if block_reason != "none":
                    missing.append(block_reason)
        if self.estop_latched:
            missing.append("estop")
        deduped_missing = []
        for item in missing:
            if item not in deduped_missing:
                deduped_missing.append(item)
        block_reason = str(self.fc_guided_block_reason or "none")
        if not heartbeat_ok:
            block_reason = "heartbeat_stale"
        fc_start_block_reason = self._recent_fc_start_block_reason(now=now)
        fc_start_ready = bool(ready and fc_start_block_reason == "none")
        if not fc_start_ready and fc_start_block_reason == "none":
            fc_start_block_reason = block_reason if block_reason != "none" else "guided_prereq_missing"
        self.fc_start_ready = bool(fc_start_ready)
        self.fc_start_block_reason = str(fc_start_block_reason)
        return {
            "ready": ready,
            "missing": deduped_missing,
            "gps_fix_type": int(self.gps_fix_type or 0),
            "gps_satellites_visible": int(self.gps_satellites_visible or 0),
            "global_position_int_received": bool(self.gps_global_position_int_received),
            "fc_gps_ekf_ready": bool(fc_position_ready and fc_nav_ready),
            "fc_position_ready": bool(fc_position_ready),
            "fc_position_valid": bool(sim_global_ok if sim_mode else gps_fix_ok),
            "fc_nav_mode_ready": bool(fc_nav_ready),
            "fc_guided_block_reason": block_reason,
            "fc_ekf_flags": int(self.fc_ekf_flags or 0),
            "fc_ekf_variances": dict(self.fc_ekf_variances),
            "fc_ekf_report_seen": bool(self.fc_ekf_report_seen),
            "fc_ekf": {
                "flags": int(self.fc_ekf_flags or 0),
                "flag_diagnostics": self._ekf_flag_diagnostics(),
                "variances": dict(self.fc_ekf_variances),
                "report_seen": bool(self.fc_ekf_report_seen),
                "report_age_s": message_ages.get("EKF_STATUS_REPORT"),
                "hard_fault": str(self._ekf_hard_fault_reason() or ""),
            },
            "fc_gps": {
                "fix_type": int(self.gps_fix_type or 0),
                "satellites_visible": int(self.gps_satellites_visible or 0),
                "gps_raw_int_age_s": gps_age,
                "global_position_int_age_s": gpi_age,
                "position_valid": bool(position_valid),
            },
            "fc_guided_ready": bool(ready),
            "fc_start_ready": bool(fc_start_ready),
            "fc_start_block_reason": str(fc_start_block_reason),
            "last_mavlink_message_ages": dict(message_ages),
            "position_valid": position_valid,
            "position_source": position_source,
            "sim_nav_valid": bool(sim_nav_ok),
            "sim_nav_reason": str(sim_nav.get("reason", "")) if isinstance(sim_nav, dict) else "",
            "sim_nav_age_s": (
                round(float(sim_nav.get("state_age_s")), 3)
                if isinstance(sim_nav, dict) and sim_nav.get("state_age_s") is not None
                else None
            ),
            "heartbeat_age_s": round(float(self.link_heartbeat_age_s or 0.0), 3),
            "vehicle_mode": str(self.vehicle_mode_name),
        }

    def _autonomy_pwm_via_rc_override(self):
        """True when twin-PWM should be sent via MAVLink RC_OVERRIDE (not GUIDED setpoints)."""
        return bool(USE_RC_OVERRIDE_FOR_AUTONOMY or getattr(self, "_sim_mavlink_actuation_rc", False))

    def _mark_sim_manual_rc_fallback(self, reason, prereq=None):
        """Test-simulation only: use MANUAL + MAVLink RC override when FC EKF/GPS blocks NAV modes."""
        if os.environ.get("USV_SIM") != "1" or USV_MODE == USV_MODE_RACE:
            return False
        self._sim_mavlink_actuation_rc = True
        self.sim_actuation_fallback = "manual_rc_override"
        self.sim_actuation_fallback_reason = str(reason or "fc_nav_not_ready")
        if not isinstance(prereq, dict):
            prereq = self._guided_prereq_status()
        log_jsonl(
            "usv_main",
            False,
            event="sim_actuation_fallback",
            fallback=self.sim_actuation_fallback,
            reason=self.sim_actuation_fallback_reason,
            fc_gps_fix_type=int(prereq.get("gps_fix_type", self.gps_fix_type) or 0),
            gps_sats=int(prereq.get("gps_satellites_visible", self.gps_satellites_visible) or 0),
            gpi_received=bool(prereq.get("global_position_int_received", self.gps_global_position_int_received)),
            sim_nav_valid=bool(prereq.get("sim_nav_valid", False)),
            sim_nav_reason=str(prereq.get("sim_nav_reason", "")),
        )
        print(
            "[SIM] [NAV] Test fallback selected: MANUAL + MAVLink RC override "
            f"reason={self.sim_actuation_fallback_reason} "
            f"fc_fix={int(prereq.get('gps_fix_type', self.gps_fix_type) or 0)} "
            f"sats={int(prereq.get('gps_satellites_visible', self.gps_satellites_visible) or 0)} "
            f"gpi={bool(prereq.get('global_position_int_received', self.gps_global_position_int_received))} "
            f"sim_nav_valid={bool(prereq.get('sim_nav_valid', False))}"
        )
        return True

    def _sitl_seed_global_origin_if_needed(self, force=False):
        """JSON SITL: without an EKF/global origin ArduPilot often keeps GPS fix at 0 and rejects GUIDED."""
        if os.environ.get("USV_SIM") != "1" or not self.master:
            return
        if not force:
            self._drain_mav_messages()
            lat0 = float(self.current_lat)
            lon0 = float(self.current_lon)
            if int(self.gps_fix_type or 0) >= 2 and self._geo_position_valid_values(lat0, lon0):
                return
            if self.gps_global_position_int_received and self._geo_position_valid_values(lat0, lon0):
                return
        try:
            from pymavlink import mavutil
            from sim_nav_state import parse_sim_home

            hlat, hlon, halt, _ = parse_sim_home()
            ts = int(getattr(self.master, "target_system", 0) or 0) or 1
            tc = int(getattr(self.master, "target_component", 0) or 0) or 1
            lat_i = int(round(float(hlat) * 1.0e7))
            lon_i = int(round(float(hlon) * 1.0e7))
            alt_mm = int(round(float(halt) * 1000.0))
            self.master.mav.set_gps_global_origin_send(ts, lat_i, lon_i, alt_mm)
            self.master.mav.command_long_send(
                ts,
                tc,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,
                0.0,
                0.0,
                0.0,
                0.0,
                float(hlat),
                float(hlon),
                float(halt),
            )
            if not force:
                log_jsonl(
                    "usv_main",
                    False,
                    event="sim_seed_global_origin",
                    lat=float(hlat),
                    lon=float(hlon),
                    alt_m=float(halt),
                )
                print(
                    f"[SIM] EKF/home seed: GPS_GLOBAL_ORIGIN + DO_SET_HOME "
                    f"lat={hlat:.6f} lon={hlon:.6f} alt_m={halt:.1f}"
                )
        except Exception as exc:
            print(f"[WARN] [SIM] Global origin seed failed: {exc}")

    def _sitl_gps_input_from_sim(self):
        """Optional bench-only GPS_INPUT injector.

        Normal simulation uses ArduPilot's JSON/SITL GPS path (GPS1_TYPE=100).
        Keep this disabled by default so FC readiness has one authoritative
        GPS source.
        """
        if os.environ.get("USV_SIM") != "1" or not self.master:
            return
        if os.environ.get("SIM_SITL_GPS_INPUT_PUMP", "0").strip().lower() not in ("1", "true", "yes"):
            return
        try:
            from sim_nav_state import load_sim_nav_state, parse_sim_home

            home_lat, home_lon, halt, _ = parse_sim_home()
            nav = load_sim_nav_state(control_dir=CONTROL_DIR)
            if nav.get("valid"):
                lat = float(nav.get("lat", home_lat))
                lon = float(nav.get("lon", home_lon))
            else:
                lat = float(home_lat)
                lon = float(home_lon)
            usec = int(time.time() * 1.0e6)
            tw, tw_ms = _gps_week_tow_ms_from_unix(time.time())
            yaw_cdeg = 0
            if nav.get("valid") and nav.get("heading_deg") is not None:
                try:
                    hd = float(nav.get("heading_deg")) % 360.0
                    yi = int(round(hd * 100.0)) % 36000
                    # MAVLink: 0 = yaw not available; 36000 cdeg = north
                    yaw_cdeg = 36000 if yi == 0 else yi
                except (TypeError, ValueError):
                    yaw_cdeg = 0
            self.master.mav.gps_input_send(
                usec,
                0,
                0,
                tw_ms,
                tw,
                3,
                int(round(lat * 1.0e7)),
                int(round(lon * 1.0e7)),
                float(halt),
                0.8,
                0.8,
                0.0,
                0.0,
                0.0,
                0.05,
                0.35,
                0.45,
                12,
                yaw_cdeg,
            )
            _now = time.monotonic()
            if not hasattr(self, "_last_gps_input_log_ts") or (_now - self._last_gps_input_log_ts) >= 1.0:
                self._last_gps_input_log_ts = _now
                log_jsonl(
                    "usv_main",
                    False,
                    event="gps_input_send",
                    lat=round(float(lat), 7),
                    lon=round(float(lon), 7),
                    alt_m=round(float(halt), 2),
                    yaw_cdeg=int(yaw_cdeg),
                    fix_type=3,
                    sats=12,
                    time_week=int(tw),
                    time_week_ms=int(tw_ms),
                    nav_valid=bool(nav.get("valid")),
                )
        except Exception as exc:
            self._warn_throttled("gps_input_inj", f"[WARN] [SIM] gps_input_send: {exc}", period_s=2.0)

    def _sitl_prepare_autopilot_for_guided(self, timeout_s=None):
        """Wait until ArduPilot reports usable GPS/global position so AUTO/GUIDED mode changes succeed."""
        if os.environ.get("USV_SIM") != "1":
            return True
        if timeout_s is None:
            sim_nav_now = load_sim_nav_state(control_dir=CONTROL_DIR)
            if self._recent_fc_start_block_reason() != "none":
                timeout_s = float(os.environ.get("SIM_SITL_PREP_TIMEOUT_S", "55") or "55")
            elif USV_MODE != USV_MODE_RACE and sim_nav_now.get("valid"):
                timeout_s = float(os.environ.get("SIM_SITL_PREP_TIMEOUT_S", "55") or "55")
            else:
                timeout_s = float(os.environ.get("SIM_SITL_PREP_TIMEOUT_S", "55") or "55")
        self._sitl_seed_global_origin_if_needed()
        _last_gps_inject = 0.0
        _last_state_write = 0.0
        _last_reseed = 0.0
        _fallback_logged = False
        deadline = time.monotonic() + float(timeout_s)
        while self.mission_active and time.monotonic() < deadline:
            _now = time.monotonic()
            if (_now - _last_reseed) >= 8.0:
                _last_reseed = _now
                if int(self.gps_fix_type or 0) < 2:
                    self._sitl_seed_global_origin_if_needed(force=True)
            if (
                os.environ.get("SIM_SITL_GPS_INPUT_PUMP", "0").strip().lower() in ("1", "true", "yes")
                and (_now - _last_gps_inject) >= 0.05
            ):
                _last_gps_inject = _now
                self._sitl_gps_input_from_sim()
            self._drain_mav_messages()
            self._resolve_nav_solution()
            if (_now - _last_state_write) >= 0.5:
                _last_state_write = _now
                try:
                    self._write_state()
                except Exception:
                    pass
            lat = float(self.current_lat)
            lon = float(self.current_lon)
            fix = int(self.gps_fix_type or 0)
            prereq_now = self._guided_prereq_status()
            if bool(prereq_now.get("ready", False)):
                guided_ready = True
                if USV_MODE != USV_MODE_RACE:
                    guided_ready = str(self.vehicle_mode_name or "").upper() == "GUIDED"
                    if not guided_ready:
                        guided_ready = bool(self._set_mode("GUIDED"))
                if guided_ready:
                    print(
                        f"[SIM] ArduPilot ready for NAV modes "
                        f"(fix={fix} sats={self.gps_satellites_visible} "
                        f"gpi={self.gps_global_position_int_received} "
                        f"ekf_flags={int(self.fc_ekf_flags or 0)} "
                        f"mode={self.vehicle_mode_name} "
                        f"lat={lat:.6f} lon={lon:.6f})"
                    )
                    return True
            if USV_MODE != USV_MODE_RACE and not _fallback_logged:
                prereq = prereq_now
                if prereq.get("sim_nav_valid") and not (
                    int(prereq.get("gps_fix_type", 0) or 0) >= 3
                    or bool(prereq.get("global_position_int_received", False))
                ):
                    _fallback_logged = True
                    print(
                        "[SIM] [NAV] FC GPS/EKF bekleniyor; sim_nav_state gecerli, "
                        f"grace={float(timeout_s):.1f}s sonra test fallback kullanilabilir"
                    )
            time.sleep(0.05)
        # Timeout: FC never produced fix/GPI. Test sim can still NAV using Pi sim_nav + MANUAL+RC fallback.
        if USV_MODE != USV_MODE_RACE:
            self._resolve_nav_solution()
            prereq = self._guided_prereq_status()
            lat_t = float(self.current_lat)
            lon_t = float(self.current_lon)
            if (
                SIM_MANUAL_RC_FALLBACK_ENABLED
                and
                self.nav_fix_valid
                and self._geo_position_valid_values(lat_t, lon_t)
                and bool(prereq.get("sim_nav_valid", False))
            ):
                self._mark_sim_manual_rc_fallback("fc_gps_ekf_not_ready", prereq=prereq)
                return True
            if self.nav_fix_valid and bool(prereq.get("sim_nav_valid", False)):
                print(
                    "[ERR] [SIM] FC GPS/EKF hazir degil; MANUAL+RC fallback varsayilan kapali "
                    "(SIM_FALLBACK_MANUAL_RC_ACTUATION=1 yalniz bench icin)."
                )
        print("[ERR] [SIM] Timeout: ArduPilot GPS/EKF not ready (AUTO/GUIDED prerequisite)")
        log_jsonl(
            "usv_main",
            False,
            event="FC_NAV_PREREQ_TIMEOUT",
            prereq=self._guided_prereq_status(),
            fc_start_block_reason=str(self.fc_start_block_reason),
        )
        return False

    def _resolve_dynamic_speed_band(self, heading_error_abs_deg):
        err = max(0.0, float(heading_error_abs_deg))
        if err <= DYN_SPEED_BAND_SOFT_DEG:
            return "straight", float(DYN_SPEED_FACTOR_STRAIGHT)
        if err <= DYN_SPEED_BAND_MEDIUM_DEG:
            return "soft", float(DYN_SPEED_FACTOR_SOFT)
        if err <= DYN_SPEED_BAND_HARD_DEG:
            return "medium", float(DYN_SPEED_FACTOR_MEDIUM)
        return "hard", float(DYN_SPEED_FACTOR_HARD)

    def _apply_dynamic_speed(self, base_speed_mps, heading_error_deg, parkur_label):
        try:
            base_speed = max(0.0, float(base_speed_mps))
        except (TypeError, ValueError):
            base_speed = 0.0
        try:
            heading_abs = abs(float(heading_error_deg))
        except (TypeError, ValueError):
            heading_abs = 0.0

        band, factor = self._resolve_dynamic_speed_band(heading_abs)
        active = bool(DYN_SPEED_ENABLED and parkur_label in DYN_SPEED_SCOPE and base_speed > 0.0)
        output_speed = base_speed

        if active:
            candidate_speed = base_speed * factor
            turn_floor = 0.0
            if band != "straight":
                if parkur_label in ("NAV", "P1", "P2"):  # NAV = unified, P1/P2 = legacy compat
                    turn_floor = DYN_SPEED_MIN_MPS_NAV
            output_speed = min(base_speed, max(candidate_speed, turn_floor))
            self._dyn_speed_log(
                f"{parkur_label}_{band}",
                (
                    f"[DYN_SPEED] {parkur_label} band={band} "
                    f"heading_error={heading_abs:.1f}deg base={base_speed:.2f}mps output={output_speed:.2f}mps"
                ),
            )
        else:
            band = "straight"
            factor = float(DYN_SPEED_FACTOR_STRAIGHT)

        self.dynamic_speed_profile = {
            "enabled": bool(DYN_SPEED_ENABLED),
            "mode": "banded",
            "scope": list(DYN_SPEED_SCOPE),
            "active": bool(active),
            "band": band,
            "factor": round(float(factor), 3),
            "heading_error_abs_deg": round(float(heading_abs), 3),
            "base_speed_mps": round(float(base_speed), 3),
            "output_speed_mps": round(float(output_speed), 3),
        }
        return float(output_speed)

    def _set_dynamic_speed_idle(self):
        self.dynamic_speed_profile = {
            "enabled": bool(DYN_SPEED_ENABLED),
            "mode": "banded",
            "scope": list(DYN_SPEED_SCOPE),
            "active": False,
            "band": "straight",
            "factor": float(DYN_SPEED_FACTOR_STRAIGHT),
            "heading_error_abs_deg": 0.0,
            "base_speed_mps": 0.0,
            "output_speed_mps": 0.0,
        }

    def _set_wind_assist_idle(self):
        self._wind_i_state_deg_s = 0.0
        self._wind_bias_deg = 0.0
        self._wind_last_ts = time.monotonic()
        self.wind_assist = {
            "enabled": bool(WIND_ASSIST_ENABLED),
            "mode": "integral_crab_bias",
            "scope": list(WIND_ASSIST_SCOPE),
            "active": False,
            "reason": "idle",
            "i_gain": float(WIND_ASSIST_I_GAIN),
            "bias_deg": 0.0,
            "bias_max_deg": float(WIND_ASSIST_BIAS_MAX_DEG),
            "heading_error_abs_deg": 0.0,
            "corrected_heading_error_deg": 0.0,
        }

    def _apply_wind_assist(self, heading_err_deg, parkur_label, speed_mps, center_obstacle=False):
        try:
            heading_err = float(heading_err_deg)
        except (TypeError, ValueError):
            heading_err = 0.0
        try:
            speed = max(0.0, float(speed_mps))
        except (TypeError, ValueError):
            speed = 0.0

        now = time.monotonic()
        dt = max(0.0, min(1.0, now - self._wind_last_ts))
        self._wind_last_ts = now

        heading_abs = abs(heading_err)
        active = False
        reason = "active"
        if not WIND_ASSIST_ENABLED:
            reason = "disabled"
        elif parkur_label not in WIND_ASSIST_SCOPE:
            reason = "out_of_scope"
        elif not self.mission_active:
            reason = "mission_inactive"
        elif speed <= 0.0:
            reason = "zero_speed"
        elif self.failsafe_state != "normal":
            reason = "failsafe"
        elif center_obstacle:
            reason = "center_obstacle"
        elif heading_abs > WIND_ASSIST_ACTIVE_ERR_MAX_DEG:
            reason = "err_too_large"
        else:
            active = True

        if active:
            self._wind_i_state_deg_s += heading_err * dt
            raw_bias = float(WIND_ASSIST_I_GAIN) * self._wind_i_state_deg_s
            clamped_bias = clamp(raw_bias, -WIND_ASSIST_BIAS_MAX_DEG, WIND_ASSIST_BIAS_MAX_DEG)
            clamped = abs(raw_bias - clamped_bias) > 1e-6
            self._wind_bias_deg = float(clamped_bias)
            if abs(WIND_ASSIST_I_GAIN) > 1e-9:
                self._wind_i_state_deg_s = self._wind_bias_deg / float(WIND_ASSIST_I_GAIN)
            corrected_heading_err = heading_err + self._wind_bias_deg
            if clamped:
                self._wind_log(
                    f"{parkur_label}_clamped",
                    (
                        f"[WIND] clamped {parkur_label} heading_error={heading_err:.1f}deg "
                        f"bias={self._wind_bias_deg:.2f}deg limit={WIND_ASSIST_BIAS_MAX_DEG:.1f}deg"
                    ),
                )
                reason = "clamped"
            else:
                self._wind_log(
                    f"{parkur_label}_active",
                    (
                        f"[WIND] active {parkur_label} heading_error={heading_err:.1f}deg "
                        f"bias={self._wind_bias_deg:.2f}deg corrected={corrected_heading_err:.1f}deg"
                    ),
                )
        else:
            previous_bias = self._wind_bias_deg
            decay_step = float(WIND_ASSIST_DECAY_PER_S) * dt
            if self._wind_bias_deg > 0.0:
                self._wind_bias_deg = max(0.0, self._wind_bias_deg - decay_step)
            elif self._wind_bias_deg < 0.0:
                self._wind_bias_deg = min(0.0, self._wind_bias_deg + decay_step)
            self._wind_bias_deg = float(clamp(self._wind_bias_deg, -WIND_ASSIST_BIAS_MAX_DEG, WIND_ASSIST_BIAS_MAX_DEG))
            if abs(WIND_ASSIST_I_GAIN) > 1e-9:
                self._wind_i_state_deg_s = self._wind_bias_deg / float(WIND_ASSIST_I_GAIN)
            else:
                self._wind_i_state_deg_s = 0.0
            corrected_heading_err = heading_err + self._wind_bias_deg
            if abs(previous_bias - self._wind_bias_deg) > 1e-6:
                self._wind_log(
                    f"{parkur_label}_decay",
                    (
                        f"[WIND] decay {parkur_label} reason={reason} "
                        f"bias={previous_bias:.2f}->{self._wind_bias_deg:.2f}deg"
                    ),
                )

        self.wind_assist = {
            "enabled": bool(WIND_ASSIST_ENABLED),
            "mode": "integral_crab_bias",
            "scope": list(WIND_ASSIST_SCOPE),
            "active": bool(active),
            "reason": reason,
            "i_gain": float(WIND_ASSIST_I_GAIN),
            "bias_deg": round(float(self._wind_bias_deg), 3),
            "bias_max_deg": float(WIND_ASSIST_BIAS_MAX_DEG),
            "heading_error_abs_deg": round(float(heading_abs), 3),
            "corrected_heading_error_deg": round(float(corrected_heading_err), 3),
        }
        return float(corrected_heading_err)

    def _set_virtual_anchor_idle(self, reason="idle", clear_center=False):
        if clear_center:
            self._anchor_center_lat = None
            self._anchor_center_lon = None
        self._anchor_pulse_until_ts = 0.0
        self._anchor_last_heading_err_deg = 0.0
        self._anchor_was_inside_fence = True
        self.virtual_anchor = {
            "enabled": bool(GEOFENCE_ENABLED),
            "mode": "geofence_virtual_anchor",
            "active": False,
            "reason": str(reason),
            "failsafe_only": bool(GEOFENCE_FAILSAFE_ONLY),
            "fence_radius_m": float(GEOFENCE_RADIUS_M),
            "drift_trigger_m": float(GEOFENCE_DRIFT_TRIGGER_M),
            "drift_from_center_m": 0.0,
            "inside_fence": True,
            "anchor_set": bool(self._anchor_center_lat is not None and self._anchor_center_lon is not None),
            "anchor_lat": self._anchor_center_lat,
            "anchor_lon": self._anchor_center_lon,
            "pulse_speed_mps": 0.0,
            "pulse_heading_error_deg": 0.0,
            "pulse_count": int(self._anchor_pulse_count),
            "breach_count": int(self._anchor_breach_count),
        }

    def _arm_virtual_anchor_center(self):
        if not self._gps_position_valid():
            return False
        self._anchor_center_lat = float(self.current_lat)
        self._anchor_center_lon = float(self.current_lon)
        self._anchor_pulse_until_ts = 0.0
        self._anchor_last_pulse_ts = 0.0
        self._anchor_last_heading_err_deg = 0.0
        self._anchor_was_inside_fence = True
        self._geo_log(
            "armed",
            (
                f"[GEOFENCE] anchor_armed lat={self._anchor_center_lat:.7f} "
                f"lon={self._anchor_center_lon:.7f} radius={GEOFENCE_RADIUS_M:.1f}m"
            ),
        )
        return True

    def _run_virtual_anchor_step(self):
        if not GEOFENCE_ENABLED:
            self._set_virtual_anchor_idle(reason="disabled", clear_center=True)
            self._command_speed_heading(0.0, 0.0)
            return

        if self.state != self.STATE_HOLD or self.mission_active:
            self._set_virtual_anchor_idle(reason="hold_inactive", clear_center=False)
            self._command_speed_heading(0.0, 0.0)
            return
        if self.estop_latched:
            self._set_virtual_anchor_idle(reason="estop_latched", clear_center=False)
            self._command_speed_heading(0.0, 0.0)
            return
        if GEOFENCE_FAILSAFE_ONLY and not str(self.hold_reason).startswith("FAILSAFE"):
            self._set_virtual_anchor_idle(reason="not_failsafe_hold", clear_center=False)
            self._command_speed_heading(0.0, 0.0)
            return
        if self.failsafe_state != "hold":
            self._set_virtual_anchor_idle(reason="failsafe_recovered", clear_center=False)
            self._command_speed_heading(0.0, 0.0)
            return
        if not self._gps_position_valid():
            self._set_virtual_anchor_idle(reason="gps_invalid", clear_center=False)
            self._geo_log("gps_invalid", "[GEOFENCE] gps_invalid virtual_anchor_idle")
            self._command_speed_heading(0.0, 0.0)
            return

        if self._anchor_center_lat is None or self._anchor_center_lon is None:
            if not self._arm_virtual_anchor_center():
                self._set_virtual_anchor_idle(reason="anchor_arm_failed", clear_center=True)
                self._command_speed_heading(0.0, 0.0)
                return

        center_lat = float(self._anchor_center_lat)
        center_lon = float(self._anchor_center_lon)
        drift_m = haversine_distance(self.current_lat, self.current_lon, center_lat, center_lon)
        inside_fence = bool(drift_m <= float(GEOFENCE_RADIUS_M))
        needs_correction = bool(drift_m >= float(GEOFENCE_DRIFT_TRIGGER_M))
        now = time.monotonic()

        if (not inside_fence) and self._anchor_was_inside_fence:
            self._anchor_breach_count += 1
            self._geo_log(
                "fence_breach",
                f"[GEOFENCE] fence_breach drift={drift_m:.2f}m radius={GEOFENCE_RADIUS_M:.1f}m",
            )
        self._anchor_was_inside_fence = inside_fence

        pulse_speed = 0.0
        heading_err = 0.0
        reason = "holding"
        active = False

        if needs_correction:
            reason = "correcting"
            target_bearing = calculate_bearing(self.current_lat, self.current_lon, center_lat, center_lon)
            heading_err = normalize_heading_error(target_bearing - self.current_heading)
            self._anchor_last_heading_err_deg = float(heading_err)
            pulse_ready = (now - self._anchor_last_pulse_ts) >= float(GEOFENCE_ANCHOR_COOLDOWN_S)
            pulse_active = now <= self._anchor_pulse_until_ts
            if pulse_ready and not pulse_active:
                self._anchor_last_pulse_ts = now
                self._anchor_pulse_until_ts = now + float(GEOFENCE_ANCHOR_PULSE_S)
                self._anchor_pulse_count += 1
                pulse_active = True
                self._geo_log(
                    "correcting",
                    (
                        f"[GEOFENCE] correcting drift={drift_m:.2f}m "
                        f"heading_error={heading_err:.1f}deg speed={GEOFENCE_ANCHOR_SPEED_MPS:.2f}mps"
                    ),
                )
            if pulse_active:
                pulse_speed = float(GEOFENCE_ANCHOR_SPEED_MPS)
                active = True
        else:
            self._anchor_pulse_until_ts = 0.0
            self._anchor_last_heading_err_deg = 0.0

        self._command_speed_heading(pulse_speed, heading_err if pulse_speed > 0.0 else 0.0)
        self.virtual_anchor = {
            "enabled": bool(GEOFENCE_ENABLED),
            "mode": "geofence_virtual_anchor",
            "active": bool(active),
            "reason": reason,
            "failsafe_only": bool(GEOFENCE_FAILSAFE_ONLY),
            "fence_radius_m": float(GEOFENCE_RADIUS_M),
            "drift_trigger_m": float(GEOFENCE_DRIFT_TRIGGER_M),
            "drift_from_center_m": round(float(drift_m), 3),
            "inside_fence": bool(inside_fence),
            "anchor_set": True,
            "anchor_lat": round(center_lat, 7),
            "anchor_lon": round(center_lon, 7),
            "pulse_speed_mps": round(float(pulse_speed), 3),
            "pulse_heading_error_deg": round(float(self._anchor_last_heading_err_deg if pulse_speed > 0.0 else 0.0), 3),
            "pulse_count": int(self._anchor_pulse_count),
            "breach_count": int(self._anchor_breach_count),
        }

    def _fuse_visual_detection(
        self,
        *,
        label,
        raw_detected,
        raw_bearing_deg,
        hold_attr,
        ghost_latch_attr,
        ghost_count_attr,
        confirm_ts_attr,
        last_dist_attr,
    ):
        now = time.monotonic()
        if not bool(raw_detected):
            setattr(self, hold_attr, None)
            setattr(self, ghost_latch_attr, False)
            return False, None

        if self.simulation_mode:
            setattr(self, hold_attr, now)
            setattr(self, ghost_latch_attr, False)
            setattr(self, confirm_ts_attr, now)
            setattr(self, last_dist_attr, 1.0)
            return True, 1.0

        if not self.lidar_ready:
            camera_only_attr = "_gate_camera_only_since" if "gate" in label else (
                "_target_camera_only_since" if "target" in label else (
                    "_yellow_camera_only_since" if "yellow" in label else "_orange_camera_only_since"
                )
            )
            camera_only_since = getattr(self, camera_only_attr)
            now = time.monotonic()
            if camera_only_since is None:
                camera_only_since = now
                setattr(self, camera_only_attr, camera_only_since)
            if (now - camera_only_since) >= FUSION_CAMERA_ONLY_TIMEOUT_S:
                setattr(self, hold_attr, now)
                setattr(self, confirm_ts_attr, now)
                setattr(self, last_dist_attr, 10.0)
                self._fusion_log(
                    f"{label}_camera_only_fallback",
                    f"[FUSION] {label} fallback camera_only after {FUSION_CAMERA_ONLY_TIMEOUT_S}s timeout",
                )
                return True, 10.0
            setattr(self, hold_attr, None)
            self._fusion_log(
                f"{label}_lidar_not_ready",
                f"[FUSION] {label} lidar_not_ready waiting_camera_only ({FUSION_CAMERA_ONLY_TIMEOUT_S - (now - camera_only_since):.1f}s)",
            )
            return False, None

        nearest = self._nearest_lidar_distance_for_camera_bearing(raw_bearing_deg)
        if nearest is None:
            setattr(self, hold_attr, None)
            if not bool(getattr(self, ghost_latch_attr)):
                setattr(self, ghost_latch_attr, True)
                current_count = int(getattr(self, ghost_count_attr))
                setattr(self, ghost_count_attr, current_count + 1)
            self._fusion_log(
                f"{label}_ghost_rejected",
                f"[FUSION] {label} ghost_rejected cam_bearing={raw_bearing_deg:.1f}",
            )
            return False, None

        setattr(self, ghost_latch_attr, False)
        hold_since = getattr(self, hold_attr)
        if hold_since is None:
            hold_since = now
            setattr(self, hold_attr, hold_since)
        if (now - hold_since) < FUSION_CONFIRM_HOLD_S:
            return False, nearest

        setattr(self, confirm_ts_attr, now)
        setattr(self, last_dist_attr, float(nearest))
        self._fusion_log(
            f"{label}_confirmed",
            f"[FUSION] {label} confirmed lidar={nearest:.2f}m cam_bearing={raw_bearing_deg:.1f}",
        )
        return True, nearest

    def _apply_camera_lidar_fusion(self, camera_data):
        fused = dict(camera_data)

        gate_actionable = bool(camera_data.get("gate_actionable", True))
        target_actionable = bool(camera_data.get("target_actionable", True))
        gate_raw_detected = bool(
            camera_data.get("gate_detected_raw", camera_data.get("gate_detected", False))
        ) and gate_actionable
        target_raw_detected = bool(
            camera_data.get("target_detected_raw", camera_data.get("target_detected", False))
        ) and target_actionable
        try:
            gate_bearing_raw = float(
                camera_data.get(
                    "gate_center_bearing_deg_imu_corrected",
                    camera_data.get("gate_center_bearing_deg", 0.0),
                )
            )
        except (TypeError, ValueError):
            gate_bearing_raw = 0.0
        try:
            target_bearing_raw = float(
                camera_data.get(
                    "target_bearing_error_deg_imu_corrected",
                    camera_data.get("target_bearing_error_deg", 0.0),
                )
            )
        except (TypeError, ValueError):
            target_bearing_raw = 0.0

        gate_detected, _ = self._fuse_visual_detection(
            label="gate",
            raw_detected=gate_raw_detected,
            raw_bearing_deg=gate_bearing_raw,
            hold_attr="_gate_fusion_hold_since",
            ghost_latch_attr="_gate_ghost_latched",
            ghost_count_attr="ghost_gate_count",
            confirm_ts_attr="_last_gate_confirm_ts",
            last_dist_attr="last_confirmed_gate_dist_m",
        )
        target_detected, _ = self._fuse_visual_detection(
            label="target",
            raw_detected=target_raw_detected,
            raw_bearing_deg=target_bearing_raw,
            hold_attr="_target_fusion_hold_since",
            ghost_latch_attr="_target_ghost_latched",
            ghost_count_attr="ghost_target_count",
            confirm_ts_attr="_last_target_confirm_ts",
            last_dist_attr="last_confirmed_target_dist_m",
        )

        fused["gate_detected"] = gate_detected
        if not gate_detected:
            fused["gate_stable_s"] = 0.0
            fused["gate_center_bearing_deg"] = 0.0
            fused["gate_passed_event"] = False

        fused["target_detected"] = target_detected
        if not target_detected:
            fused["target_bearing_error_deg"] = 0.0
            fused["target_area_norm"] = 0.0

        yellow_actionable = bool(camera_data.get("yellow_actionable", True))
        orange_actionable = bool(camera_data.get("orange_actionable", True))
        yellow_raw_detected = bool(camera_data.get("yellow_obstacle_detected_raw", False)) and yellow_actionable
        orange_raw_detected = bool(camera_data.get("orange_boundary_detected_raw", False)) and orange_actionable
        try:
            yellow_bearing_raw = float(camera_data.get("yellow_obstacle_bearing_deg_raw", 0.0) or 0.0)
        except (TypeError, ValueError):
            yellow_bearing_raw = 0.0
        try:
            orange_bearing_raw = float(camera_data.get("orange_boundary_bearing_deg_raw", 0.0) or 0.0)
        except (TypeError, ValueError):
            orange_bearing_raw = 0.0

        yellow_detected, yellow_dist = self._fuse_visual_detection(
            label="yellow_obstacle",
            raw_detected=yellow_raw_detected,
            raw_bearing_deg=yellow_bearing_raw,
            hold_attr="_yellow_fusion_hold_since",
            ghost_latch_attr="_yellow_ghost_latched",
            ghost_count_attr="ghost_yellow_count",
            confirm_ts_attr="_last_yellow_confirm_ts",
            last_dist_attr="last_confirmed_yellow_dist_m",
        )
        orange_detected, orange_dist = self._fuse_visual_detection(
            label="orange_boundary",
            raw_detected=orange_raw_detected,
            raw_bearing_deg=orange_bearing_raw,
            hold_attr="_orange_fusion_hold_since",
            ghost_latch_attr="_orange_ghost_latched",
            ghost_count_attr="ghost_orange_count",
            confirm_ts_attr="_last_orange_confirm_ts",
            last_dist_attr="last_confirmed_orange_dist_m",
        )

        if yellow_actionable:
            fused["yellow_obstacle_detected"] = bool(yellow_detected)
            if yellow_detected:
                fused["yellow_obstacle_bearing_deg"] = round(float(yellow_bearing_raw), 3)
                fused["yellow_obstacle_area_norm"] = round(
                    float(camera_data.get("yellow_obstacle_area_norm_raw", 0.0) or 0.0),
                    4,
                )
            elif not yellow_raw_detected:
                fused["yellow_obstacle_bearing_deg"] = 0.0
                fused["yellow_obstacle_area_norm"] = 0.0

        if orange_actionable:
            fused["orange_boundary_detected"] = bool(orange_detected)
            if orange_detected:
                fused["orange_boundary_bearing_deg"] = round(float(orange_bearing_raw), 3)
                fused["orange_boundary_area_norm"] = round(
                    float(camera_data.get("orange_boundary_area_norm_raw", 0.0) or 0.0),
                    4,
                )
            elif not orange_raw_detected:
                fused["orange_boundary_bearing_deg"] = 0.0
                fused["orange_boundary_area_norm"] = 0.0

        return fused

    def _refresh_link_heartbeat(self):
        onboard_age = max(0.0, time.monotonic() - self.last_heartbeat_time)
        if not self.master:
            self.link_heartbeat_age_s = 0.0
            self.link_heartbeat_source = "offline_simulation" if self.simulation_mode else "no_master"
            return

        link_age = None
        try:
            if os.path.exists(LINK_STATE_FILE):
                with open(LINK_STATE_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    link_age = float(data.get("link_heartbeat_age_s", onboard_age))
                    ts = float(data.get("ts_monotonic", 0.0))
                    if ts > 0.0:
                        link_age += max(0.0, time.monotonic() - ts)
                    link_age = max(0.0, link_age)
                    self.link_heartbeat_source = "telemetry"
                else:
                    self.link_heartbeat_source = "onboard_fallback"
            else:
                self.link_heartbeat_source = "onboard_fallback"
        except Exception as exc:
            self._bump_error("link_state_read_error", f"[WARN] [LINK] Link state okuma hatasi: {exc}")
            self.link_heartbeat_source = "onboard_fallback"
            link_age = None

        if link_age is None:
            link_age = onboard_age
            if self.simulation_mode or os.environ.get("USV_SIM") == "1":
                self.link_heartbeat_source = "simulation_mavlink"

        old_age = self.link_heartbeat_age_s
        self.link_heartbeat_age_s = float(link_age)
        
        # LOG: Heartbeat age monitoring (log every 2+ second change or every 10s)
        now = time.monotonic()
        if not hasattr(self, '_last_hb_log') or (now - self._last_hb_log) >= 10.0 or abs(self.link_heartbeat_age_s - old_age) >= 2.0:
            print(f"[LINK_HB] Age={self.link_heartbeat_age_s:.1f}s Src={self.link_heartbeat_source} OnboardAge={onboard_age:.1f}s")
            self._last_hb_log = now

    def _refresh_health_check(self):
        sim_runtime = bool(self.simulation_mode or os.environ.get("USV_SIM") == "1")

        if sim_runtime:
            mavlink_ok = bool(self.master is not None)
            heartbeat_ok = bool(self.master is not None and self.link_heartbeat_age_s < HEARTBEAT_WARN_S)
            rc_ok = True
        else:
            mavlink_ok = bool(self.master is not None)
            heartbeat_ok = bool(self.link_heartbeat_age_s < HEARTBEAT_WARN_S)
            rc_ok = self._rc_link_active()

        if sim_runtime:
            estop_safe = bool((not self.estop_latched) and not os.path.exists(FLAG_STOP))
        else:
            estop_safe = bool((not self.estop_latched) and self.rc_channels.get("ch7", 0) <= RC7_SAFE_PWM and not os.path.exists(FLAG_STOP))
        lidar_point_count = int(len(self._lidar_front_samples))
        if sim_runtime:
            storage_health = {
                "local_writable": True,
                "usb_required": False,
                "usb_present": False,
                "usb_writable": True,
                "usb_candidates": [],
            }
            camera_fresh = bool(self.camera_ready)
            lidar_fresh = bool(self.lidar_ready and self._lidar_quality_ready)
        else:
            storage_health = evaluate_storage_health(USV_MODE, local_dir=LOG_DIR)
            camera_fresh = self.camera_ready
            lidar_fresh = bool(self.lidar_ready and self._lidar_quality_ready and lidar_point_count > 0)

        if self.lidar_ready and lidar_point_count <= 0 and not sim_runtime:
            self._warn_throttled(
                "lidar_points_missing",
                "[WARN] [LIDAR] Ready flag true ama nokta sayisi 0 - startup readiness bekleniyor",
            )
        flags, missing = evaluate_readiness_flags(
            mode=USV_MODE,
            mavlink_vehicle_link=mavlink_ok,
            telemetry_heartbeat_ok=heartbeat_ok,
            rc_link_active=rc_ok,
            estop_safe=estop_safe,
            camera_fresh=camera_fresh,
            lidar_fresh=lidar_fresh,
            storage_health=storage_health,
        )
        self.health_flags = flags
        self.health_missing = missing
        self.health_storage = storage_health
        self.health_ready = not missing
        self.health_checked_at = round(time.monotonic(), 3)
        self._update_autonomy_health()

    def _write_state(self):
        try:
            os.makedirs(CONTROL_DIR, exist_ok=True)
            active_parkur = self._active_parkur_label()
            objective_phase = self._get_objective_phase()
            perception_policy = self._get_perception_policy()
            guided_prereq = self._guided_prereq_status()
            dyn_speed_state = self.dynamic_speed_profile if isinstance(self.dynamic_speed_profile, dict) else {}
            dyn_speed_payload = {
                "enabled": bool(dyn_speed_state.get("enabled", DYN_SPEED_ENABLED)),
                "mode": str(dyn_speed_state.get("mode", "banded")),
                "scope": list(dyn_speed_state.get("scope", list(DYN_SPEED_SCOPE))),
                "active": bool(dyn_speed_state.get("active", False)),
                "band": str(dyn_speed_state.get("band", "straight")),
                "factor": round(float(dyn_speed_state.get("factor", DYN_SPEED_FACTOR_STRAIGHT)), 3),
                "heading_error_abs_deg": round(float(dyn_speed_state.get("heading_error_abs_deg", 0.0)), 3),
                "base_speed_mps": round(float(dyn_speed_state.get("base_speed_mps", 0.0)), 3),
                "output_speed_mps": round(float(dyn_speed_state.get("output_speed_mps", 0.0)), 3),
            }
            wind_state = self.wind_assist if isinstance(self.wind_assist, dict) else {}
            wind_payload = {
                "enabled": bool(wind_state.get("enabled", WIND_ASSIST_ENABLED)),
                "mode": str(wind_state.get("mode", "integral_crab_bias")),
                "scope": list(wind_state.get("scope", list(WIND_ASSIST_SCOPE))),
                "active": bool(wind_state.get("active", False)),
                "reason": str(wind_state.get("reason", "idle")),
                "i_gain": float(wind_state.get("i_gain", WIND_ASSIST_I_GAIN)),
                "bias_deg": round(float(wind_state.get("bias_deg", 0.0)), 3),
                "bias_max_deg": float(wind_state.get("bias_max_deg", WIND_ASSIST_BIAS_MAX_DEG)),
                "heading_error_abs_deg": round(float(wind_state.get("heading_error_abs_deg", 0.0)), 3),
                "corrected_heading_error_deg": round(float(wind_state.get("corrected_heading_error_deg", 0.0)), 3),
            }
            horizon_state = self.horizon_lock if isinstance(self.horizon_lock, dict) else {}
            horizon_payload = {
                "enabled": bool(horizon_state.get("enabled", HORIZON_LOCK_ENABLED)),
                "mode": str(horizon_state.get("mode", "imu_horizon_lock")),
                "scope": list(horizon_state.get("scope", list(HORIZON_LOCK_SCOPE))),
                "active": bool(horizon_state.get("active", False)),
                "reason": str(horizon_state.get("reason", "idle")),
                "channel": str(horizon_state.get("channel", "none")),
                "roll_deg": round(float(horizon_state.get("roll_deg", self.current_roll_deg)), 3),
                "pitch_deg": round(float(horizon_state.get("pitch_deg", self.current_pitch_deg)), 3),
                "raw_bearing_deg": round(float(horizon_state.get("raw_bearing_deg", 0.0)), 3),
                "correction_deg": round(float(horizon_state.get("correction_deg", 0.0)), 3),
                "corrected_bearing_deg": round(float(horizon_state.get("corrected_bearing_deg", 0.0)), 3),
                "roll_gain": float(horizon_state.get("roll_gain", HORIZON_LOCK_ROLL_GAIN)),
                "pitch_gain": float(horizon_state.get("pitch_gain", HORIZON_LOCK_PITCH_GAIN)),
                "max_correction_deg": float(horizon_state.get("max_correction_deg", HORIZON_LOCK_MAX_CORRECTION_DEG)),
            }
            camera_adapt_state = self.camera_adaptation if isinstance(self.camera_adaptation, dict) else {}
            camera_adapt_payload = {
                "enabled": bool(camera_adapt_state.get("enabled", False)),
                "mode": str(camera_adapt_state.get("mode", "normal")),
                "luma_mean": round(float(camera_adapt_state.get("luma_mean", 0.0) or 0.0), 2),
                "exposure_gain": round(float(camera_adapt_state.get("exposure_gain", 1.0) or 1.0), 3),
                "exposure_beta": round(float(camera_adapt_state.get("exposure_beta", 0.0) or 0.0), 3),
                "hsv_s_shift": int(camera_adapt_state.get("hsv_s_shift", 0) or 0),
                "hsv_v_shift": int(camera_adapt_state.get("hsv_v_shift", 0) or 0),
                "hsv_profile": str(camera_adapt_state.get("hsv_profile", "base")),
            }
            autonomy_health_state = self.autonomy_health if isinstance(self.autonomy_health, dict) else {}
            autonomy_health_payload = {
                "enabled": bool(autonomy_health_state.get("enabled", TRUST_BAR_ENABLED)),
                "trust_score": round(float(autonomy_health_state.get("trust_score", 0.0) or 0.0), 2),
                "level": str(autonomy_health_state.get("level", "low")),
                "color": str(autonomy_health_state.get("color", "red")),
                "label": str(autonomy_health_state.get("label", "DUSUK")),
                "advisory": str(autonomy_health_state.get("advisory", "--")),
                "failsafe_state": str(autonomy_health_state.get("failsafe_state", self.failsafe_state)),
                "gps_satellites_visible": int(autonomy_health_state.get("gps_satellites_visible", self.gps_satellites_visible) or 0),
                "gps_fix_type": int(autonomy_health_state.get("gps_fix_type", self.gps_fix_type) or 0),
                "camera_mode": str(autonomy_health_state.get("camera_mode", self.camera_adaptation.get("mode", "normal"))),
                "lidar_point_count": int(autonomy_health_state.get("lidar_point_count", len(self._lidar_front_samples)) or 0),
                "rc_link_active": bool(autonomy_health_state.get("rc_link_active", self._rc_link_active())),
                "component_scores": dict(autonomy_health_state.get("component_scores", {})),
                "weights": dict(autonomy_health_state.get("weights", {})),
            }
            virtual_anchor_state = self.virtual_anchor if isinstance(self.virtual_anchor, dict) else {}
            virtual_anchor_payload = {
                "enabled": bool(virtual_anchor_state.get("enabled", GEOFENCE_ENABLED)),
                "mode": str(virtual_anchor_state.get("mode", "geofence_virtual_anchor")),
                "active": bool(virtual_anchor_state.get("active", False)),
                "reason": str(virtual_anchor_state.get("reason", "idle")),
                "failsafe_only": bool(virtual_anchor_state.get("failsafe_only", GEOFENCE_FAILSAFE_ONLY)),
                "fence_radius_m": round(float(virtual_anchor_state.get("fence_radius_m", GEOFENCE_RADIUS_M)), 3),
                "drift_trigger_m": round(float(virtual_anchor_state.get("drift_trigger_m", GEOFENCE_DRIFT_TRIGGER_M)), 3),
                "drift_from_center_m": round(float(virtual_anchor_state.get("drift_from_center_m", 0.0)), 3),
                "inside_fence": bool(virtual_anchor_state.get("inside_fence", True)),
                "anchor_set": bool(virtual_anchor_state.get("anchor_set", False)),
                "anchor_lat": virtual_anchor_state.get("anchor_lat"),
                "anchor_lon": virtual_anchor_state.get("anchor_lon"),
                "pulse_speed_mps": round(float(virtual_anchor_state.get("pulse_speed_mps", 0.0)), 3),
                "pulse_heading_error_deg": round(float(virtual_anchor_state.get("pulse_heading_error_deg", 0.0)), 3),
                "pulse_count": int(virtual_anchor_state.get("pulse_count", 0) or 0),
                "breach_count": int(virtual_anchor_state.get("breach_count", 0) or 0),
            }
            fusion_payload = {
                "enabled": bool(FUSION_ENABLED),
                "policy": self.fusion_policy,
                "ghost_gate_count": int(self.ghost_gate_count),
                "ghost_target_count": int(self.ghost_target_count),
                "ghost_yellow_count": int(self.ghost_yellow_count),
                "ghost_orange_count": int(self.ghost_orange_count),
                "last_confirmed_gate_dist_m": (
                    round(float(self.last_confirmed_gate_dist_m), 3)
                    if self.last_confirmed_gate_dist_m is not None
                    else None
                ),
                "last_confirmed_target_dist_m": (
                    round(float(self.last_confirmed_target_dist_m), 3)
                    if self.last_confirmed_target_dist_m is not None
                    else None
                ),
                "last_confirmed_yellow_dist_m": (
                    round(float(self.last_confirmed_yellow_dist_m), 3)
                    if self.last_confirmed_yellow_dist_m is not None
                    else None
                ),
                "last_confirmed_orange_dist_m": (
                    round(float(self.last_confirmed_orange_dist_m), 3)
                    if self.last_confirmed_orange_dist_m is not None
                    else None
                ),
                "lidar_ready": bool(self.lidar_ready),
                "camera_ready": bool(self.camera_ready),
            }
            active_waypoint_index = int(self._active_waypoint_index) if self._active_waypoint_index >= 0 else -1
            camera_pipeline = self.camera_status.get("pipeline", {})
            if not isinstance(camera_pipeline, dict):
                camera_pipeline = {}
            lidar_points_local_for_state = []
            lidar_points_world_for_state = []
            lidar_frame_ok = bool(self._lidar_quality_ready) and str(self._lidar_frame_status).startswith("ok")
            display_stable = int(LIDAR_MAP_DISPLAY_STABLE_COUNT)
            if not lidar_frame_ok:
                display_stable = max(int(LIDAR_MAP_OCC_STABLE_COUNT), display_stable - 1)
            seen_world = set()
            for wx_m, wy_m in self._occupied_lidar_map_points_world(
                limit=8192,
                min_stable_count=display_stable,
            ):
                key = (round(float(wx_m), 2), round(float(wy_m), 2))
                if key in seen_world:
                    continue
                seen_world.add(key)
                lidar_points_world_for_state.append([key[0], key[1]])
            if lidar_frame_ok:
                for item in list(self._stable_map_points)[:640]:
                    if not isinstance(item, (list, tuple)) or len(item) < 2:
                        continue
                    try:
                        x_m = float(item[0])
                        y_m = float(item[1])
                    except (TypeError, ValueError):
                        continue
                    if not (math.isfinite(x_m) and math.isfinite(y_m)):
                        continue
                    lidar_points_local_for_state.append([round(x_m, 2), round(y_m, 2)])
            lidar_points_frame = "enu_world"
            _gcs_mono = getattr(self, "last_gcs_heartbeat_mono", None)
            mission_planner_gcs_age_s = (
                round(time.monotonic() - float(_gcs_mono), 3) if _gcs_mono is not None else None
            )
            rc_diag = self._last_rc_override_diag if isinstance(self._last_rc_override_diag, dict) else {}
            nav_diverging = bool(
                float(self.progress_along_leg_m) < 0.0
                and float(self.nav_target_distance_delta_m) > 0.05
                and int(self._wrong_turn_count) > 0
            )
            nav_diagnostic_reason = str(self.heading_control_diagnostic or "nominal")
            if nav_diverging:
                nav_diagnostic_reason = "diverging_from_waypoint:negative_progress_distance_increase_wrong_turn"
            servo_output = self.last_servo_output_raw if isinstance(self.last_servo_output_raw, dict) else {}
            nav_controller = self.last_nav_controller_output if isinstance(self.last_nav_controller_output, dict) else {}
            servo1_pwm = int(servo_output.get("servo1_raw", PWM_NEUTRAL_US) or PWM_NEUTRAL_US)
            servo3_pwm = int(servo_output.get("servo3_raw", PWM_NEUTRAL_US) or PWM_NEUTRAL_US)
            servo_yaw_delta_pwm = int(servo1_pwm - servo3_pwm)
            servo_yaw_sign_mismatch = bool(
                abs(float(self.nav_heading_error_deg)) >= float(HEADING_WRONG_TURN_ERR_DEG)
                and abs(float(self.current_yaw_rate_dps)) >= float(HEADING_WRONG_TURN_YAWRATE_DPS)
                and (
                    (float(self.nav_heading_error_deg) > 0.0 and float(self.current_yaw_rate_dps) < 0.0)
                    or (float(self.nav_heading_error_deg) < 0.0 and float(self.current_yaw_rate_dps) > 0.0)
                )
            )
            payload = {
                "ts_monotonic": round(time.monotonic(), 3),
                "mode": "race" if USV_MODE == USV_MODE_RACE else "test",
                "state": self.state,
                "active": self.mission_active,
                "mode_state": dict(self.mode_state),
                "guided_ready": bool(guided_prereq.get("ready", False)),
                "fc_guided_ready": bool(guided_prereq.get("fc_guided_ready", guided_prereq.get("ready", False))),
                "fc_start_ready": bool(guided_prereq.get("fc_start_ready", False)),
                "fc_start_block_reason": str(guided_prereq.get("fc_start_block_reason", "unknown")),
                "guided_ready_missing": list(guided_prereq.get("missing", [])),
                "guided_position_source": str(guided_prereq.get("position_source", self.guided_position_source)),
                "guided_prereq": dict(guided_prereq),
                "fc_gps": dict(guided_prereq.get("fc_gps", {})),
                "fc_ekf": dict(guided_prereq.get("fc_ekf", {})),
                "last_mavlink_message_ages": dict(guided_prereq.get("last_mavlink_message_ages", {})),
                "fc_gps_ekf_ready": bool(guided_prereq.get("fc_gps_ekf_ready", False)),
                "fc_position_valid": bool(guided_prereq.get("fc_position_valid", False)),
                "fc_nav_mode_ready": bool(guided_prereq.get("fc_nav_mode_ready", False)),
                "fc_guided_block_reason": str(guided_prereq.get("fc_guided_block_reason", "none")),
                "fc_ekf_flags": int(guided_prereq.get("fc_ekf_flags", 0) or 0),
                "fc_ekf_variances": dict(guided_prereq.get("fc_ekf_variances", {})),
                "fc_ekf_report_seen": bool(guided_prereq.get("fc_ekf_report_seen", False)),
                "start_phase": str(self.start_phase or "idle"),
                "sim_actuation_fallback": str(self.sim_actuation_fallback),
                "sim_actuation_fallback_reason": str(self.sim_actuation_fallback_reason),
                "fc_gps_fix_type": int(self.gps_fix_type),
                "gps_sats": int(self.gps_satellites_visible),
                "gpi_received": bool(self.gps_global_position_int_received),
                "sim_nav_valid": bool(guided_prereq.get("sim_nav_valid", False)),
                "last_mode_command": dict(self.last_mode_command),
                "last_command_ack": dict(self.last_command_ack),
                "last_statustext": dict(self.last_statustext),
                "mission_current_seq": int(self.mission_current_seq),
                "mission_reached_seq": int(self.mission_reached_seq),
                "race_p1_completion_source": str(self.race_p1_completion_source),
                "start_time": self.mission_start_time,
                "start_requested": bool(os.path.exists(FLAG_START)),
                "hold_reason": self.hold_reason,
                "mission_end_reason": str(self.mission_end_reason or "none"),
                "target": self._wp_target,
                "wp_info": self._wp_info,
                "active_parkur": active_parkur,
                "active_waypoint_index": active_waypoint_index,
                "objective_phase": objective_phase,
                "guidance_source": self._get_guidance_source(),
                "guidance_detail_source": self.guidance_detail_source,
                "guidance_mode": self.guidance_mode,
                "guidance_reason": self._guidance_reason_with_lidar_block(),
                "avoidance_bias_deg": round(float(self.avoidance_bias_deg), 3),
                "cross_track_error_m": round(float(self.cross_track_error_m), 3),
                "cross_track_corr_deg": round(float(getattr(self, "cross_track_corr_deg", 0.0)), 3),
                "cross_track_align_active": bool(getattr(self, "cross_track_align_active", False)),
                "nominal_heading_deg": round(float(self.nominal_heading_deg), 3),
                "gate_assist_bias_deg": round(float(self.gate_assist_bias_deg), 3),
                "progress_along_leg_m": round(float(self.progress_along_leg_m), 3),
                "nav_target_distance_delta_m": round(float(self.nav_target_distance_delta_m), 3),
                "nav_target_distance_increase_count": int(self.nav_target_distance_increase_count),
                "nav_heading_error_delta_deg": round(float(self.nav_heading_error_delta_deg), 3),
                "nav_diagnostic_reason": nav_diagnostic_reason,
                "waypoint_passed_gate": bool(self.waypoint_passed_gate),
                "waypoint_accept_reason": self.waypoint_accept_reason,
                "motor_limit_reason": self.motor_limit_reason,
                "target_color": self.target_color or "--",
                "mission_input_format": self.mission_input_format,
                "perception_policy": perception_policy,
                "gate_count": self.gate_count,
                "command_lock": self.command_lock,
                "failsafe_state": self.failsafe_state,
                "mission_lifecycle": {
                    "schema_version": 1,
                    "upload_source": self.mission_upload_source,
                    "mission_synced": bool(self.mission_synced),
                    "mission_profile_valid": bool(self.mission_profile_valid),
                    "mission_profile_race_ready": bool(self.mission_profile_race_ready),
                    "mission_profile_error": str(self.mission_profile_error or ""),
                    "p2_min_gate_count": int(self.p2_min_gate_count),
                    "p3_engagement_mode": str(self.p3_engagement_mode),
                    "pixhawk_mission_count": int(self.pixhawk_mission_count),
                    "pixhawk_mission_synced_at": self.pixhawk_mission_synced_at,
                    "pixhawk_mission_sync_error": str(self.pixhawk_mission_sync_error or ""),
                    "pixhawk_mirror_status": str(self.pixhawk_mirror_status or "unknown"),
                    "pixhawk_mirror_error": str(self.pixhawk_mirror_error or ""),
                    "post_start_mission_change_rejected": bool(self.post_start_mission_change_rejected),
                    "validated_at_timestamp": self.mission_validated_at_timestamp,
                },
                "mission_upload_source": self.mission_upload_source,
                "mission_synced": bool(self.mission_synced),
                "pixhawk_mission_count": int(self.pixhawk_mission_count),
                "pixhawk_mission_synced_at": self.pixhawk_mission_synced_at,
                "pixhawk_mission_sync_error": str(self.pixhawk_mission_sync_error or ""),
                "pixhawk_mirror_status": str(self.pixhawk_mirror_status or "unknown"),
                "pixhawk_mirror_error": str(self.pixhawk_mirror_error or ""),
                "mission_validated_at_timestamp": self.mission_validated_at_timestamp,
                "post_start_mission_change_rejected": bool(self.post_start_mission_change_rejected),
                "mission_profile": dict(self.mission_profile),
                "mission_profile_valid": bool(self.mission_profile_valid),
                "mission_profile_race_ready": bool(self.mission_profile_race_ready),
                "mission_profile_error": str(self.mission_profile_error or ""),
                "phase_transition_policy": dict(self.phase_transition_policy),
                "p2_min_gate_count": int(self.p2_min_gate_count),
                "p3_engagement_mode": str(self.p3_engagement_mode),
                "waypoint_counts": {
                    "nav_total": len(self.nav_waypoints) if hasattr(self, 'nav_waypoints') else 0,
                    "has_engage": self.engage_wp is not None if hasattr(self, 'engage_wp') else False,
                    # Legacy backward-compat keys
                    "parkur1": len(self.nav_waypoints) if hasattr(self, 'nav_waypoints') else 0,
                    "parkur2": 0,
                    "parkur3": 1 if (hasattr(self, 'engage_wp') and self.engage_wp is not None) else 0,
                },
                "mission_split_profile": dict(self.mission_split_profile),
                "gate_gecildi": bool(self.gate_count > 0),  # Event flag: gate passed
                "angajman_tamam": bool(self.state == self.STATE_COMPLETED),  # Event flag: engagement complete
                "timeout": bool(self.timeout_count > 0),  # Event flag: timeout occurred
                "p3_wrong_target_avoidance": bool(self.p3_wrong_target_avoidance),
                "p3_wrong_target_class": str(self.p3_wrong_target_class or ""),
                "p3_contact_confirmation_source": str(self.p3_contact_confirmation_source or "none"),
                "estop_state": self.estop_latched,
                "estop_source": self.estop_source or "--",
                "camera_ready": self.camera_ready,
                "lidar_ready": self.lidar_ready,
                "camera_pipeline": camera_pipeline,
                "camera_status_age_s": round(float(self.camera_status_age_s), 3),
                "camera_status_mtime_age_s": round(float(self.camera_status_mtime_age_s), 3),
                "camera_status_stale": bool(self.camera_status_stale),
                "roll_deg": round(self.current_roll_deg, 3),
                "pitch_deg": round(self.current_pitch_deg, 3),
                "gps_satellites_visible": int(self.gps_satellites_visible),
                "gps_fix_type": int(self.gps_fix_type),
                "battery_voltage": round(float(self.battery_voltage), 3),
                "heartbeat_age_s": round(self.heartbeat_age_s, 3),
                "mission_planner_gcs_age_s": mission_planner_gcs_age_s,
                "gps_ok": bool(
                    self.gps_fix_type >= 2
                    or (self.gps_global_position_int_received and os.environ.get("USV_SIM") == "1")
                    or bool(guided_prereq.get("sim_nav_valid", False))
                ),
                "ekf_ok": bool(self.health_ready),  # EKF health represented by overall health readiness
                "imu_ok": bool(abs(self.current_roll_deg) < 180 and abs(self.current_pitch_deg) < 180),  # IMU health: valid roll/pitch
                "link_heartbeat_age_s": round(self.link_heartbeat_age_s, 3),
                "link_heartbeat_source": self.link_heartbeat_source,
                "current_heading": round(self.current_heading, 3),
                "nav_position_source": self.nav_position_source,
                "nav_heading_source": self.nav_heading_source,
                "nav_solution_source": self.nav_solution_source,
                "nav_source_detail": self.nav_source_detail,
                "nav_state_age_s": round(float(self.nav_state_age_s), 3),
                "nav_fix_valid": bool(self.nav_fix_valid),
                "nav_target_bearing_deg": round(float(self.nav_target_bearing_deg), 3),
                "nav_target_distance_m": round(float(self.nav_target_distance_m), 3),
                "nav_heading_error_deg": round(float(self.nav_heading_error_deg), 3),
                "nav_target_lat": round(float(self.nav_target_lat), 7),
                "nav_target_lon": round(float(self.nav_target_lon), 7),
                "nav_leg_start_lat": round(float(self._wp_leg_start_lat), 7),
                "nav_leg_start_lon": round(float(self._wp_leg_start_lon), 7),
                "closest_waypoint_distance_m": round(float(self._closest_waypoint_distance_seen), 3),
                "nav_arrival_phase": self.nav_arrival_phase,
                "nav_align_mode": str(self._nav_align_mode),
                "nav_align_phase": "ACQUIRE_HEADING" if self._nav_align_mode == "align" else "TRACK_LEG",
                "nav_strict_heading_first": bool(NAV_STRICT_HEADING_FIRST),
                "nav_align_stable_s": round(float(NAV_ALIGN_STABLE_S), 3),
                "surge_allowed": bool(self._nav_surge_allowed(self.nav_heading_error_deg)),
                "advance_stable_elapsed_s": round(float(self._nav_advance_stable_elapsed_s()), 3),
                "guided_vx_north_mps": round(float(self._last_guided_velocity_ne[0]), 4),
                "guided_vy_east_mps": round(float(self._last_guided_velocity_ne[1]), 4),
                "avoidance_active": bool(self.avoidance_active),
                "avoidance_source": self.avoidance_source,
                "escape_side": self.escape_side,
                "avoid_switch_count": int(self._avoid_switch_count),
                "blocked_level": round(float(self._blocked_level), 3),
                "obstacle_threat_active": bool(self._obstacle_threat_active),
                "obstacle_threat_source": str(self._obstacle_threat_source),
                "avoidance_recovery_count": int(self._avoid_recovery_count),
                "avoidance_recovery_active": bool(
                    (time.monotonic() - float(self._avoid_recovery_last_ts)) <= float(SIM_AVOID_RECOVERY_COOLDOWN_S)
                ),
                "final_speed_limiter": str(self._final_speed_limiter),
                "lidar_sector_ages": dict(self.lidar_sector_ages),
                "lidar_left_m": round(float(self.lidar_left_dist), 3),
                "lidar_center_m": round(float(self.lidar_center_dist), 3),
                "lidar_right_m": round(float(self.lidar_right_dist), 3),
                "min_obstacle_m": round(float(self.min_obstacle_distance), 3),
                "lidar_frame_status": str(self._lidar_frame_status),
                "lidar_clock_source": str(self._lidar_clock_source),
                "lidar_stamp_age_s": (
                    round(float(self._lidar_stamp_age_s), 3)
                    if self._lidar_stamp_age_s is not None
                    else None
                ),
                "lidar_drop_reason": str(self._lidar_drop_reason),
                "lidar_local_obstacle_valid": bool(self._lidar_local_obstacle_valid),
                "lidar_degraded_mode": str(self._lidar_degraded_mode),
                "lidar_degraded_age_s": round(float(self._lidar_degraded_age_s), 3),
                "lidar_stale_consecutive": int(self._lidar_stale_consecutive),
                "lidar_unreliable_clock_consecutive": int(self._lidar_unreliable_clock_consecutive),
                "lidar_clock_reject_count": int(self._lidar_clock_reject_count),
                "lidar_frame_valid_ratio": round(float(self._lidar_frame_valid_ratio), 3),
                "lidar_quality_ready": bool(self._lidar_quality_ready),
                "stable_points": int(self._lidar_stable_point_count),
                "tentative_points": int(self._lidar_tentative_point_count),
                "map_occupied_cells": int(self._lidar_occ_cells_count),
                "lidar_points_frame": "enu_world",
                "lidar_points_world": lidar_points_world_for_state,
                "lidar_points_local": lidar_points_local_for_state,
                "v_target": round(self.v_target, 3),
                "current_speed_mps": round(float(self.current_speed_mps), 3),
                "current_yaw_rate_dps": round(float(self.current_yaw_rate_dps), 3),
                "heading_target": round(self.heading_target, 3),
                "cmd_port_pwm": int(rc_diag.get("desired_left_pwm", PWM_NEUTRAL_US) or PWM_NEUTRAL_US),
                "cmd_stbd_pwm": int(rc_diag.get("desired_right_pwm", PWM_NEUTRAL_US) or PWM_NEUTRAL_US),
                "sim_rc_override_semantics": str(rc_diag.get("semantics", "left_right_pwm")),
                "rc_override_ch1": int(rc_diag.get("ch1", PWM_NEUTRAL_US) or PWM_NEUTRAL_US),
                "rc_override_ch3": int(rc_diag.get("ch3", PWM_NEUTRAL_US) or PWM_NEUTRAL_US),
                "desired_left_pwm": int(rc_diag.get("desired_left_pwm", PWM_NEUTRAL_US) or PWM_NEUTRAL_US),
                "desired_right_pwm": int(rc_diag.get("desired_right_pwm", PWM_NEUTRAL_US) or PWM_NEUTRAL_US),
                "last_servo_output_raw": dict(servo_output),
                "last_nav_controller_output": dict(nav_controller),
                "servo_yaw_delta_pwm": int(servo_yaw_delta_pwm),
                "servo_yaw_sign_mismatch": bool(servo_yaw_sign_mismatch),
                "motor_command_source": str(self._last_motor_command_source or "unknown"),
                "both_reverse_count": int(self._both_reverse_count),
                "single_side_reverse_count": int(self._single_side_reverse_count),
                "reverse_while_heading_small_count": int(self._reverse_while_heading_small_count),
                "pwm_trim_port_us": round(float(self._trim_port_us), 3),
                "pwm_trim_stbd_us": round(float(self._trim_stbd_us), 3),
                "wrong_turn_count": int(self._wrong_turn_count),
                "wrong_turn_active": bool(self._wrong_turn_active),
                "heading_damping_hold_active": bool(self._heading_damping_hold_active),
                "heading_damping_hold_count": int(self._heading_damping_hold_count),
                "heading_control_diagnostic": str(self.heading_control_diagnostic or "nominal"),
                "timeout_count": self.timeout_count,
                "error_counters": dict(self.error_counters),
                "health_check": {
                    "ready": self.health_ready,
                    "missing": self.health_missing,
                    "flags": self.health_flags,
                    "storage": self.health_storage,
                    "checked_at_monotonic": self.health_checked_at,
                },
                "ready_state": self.health_ready,
                "ready_missing": self.health_missing,
                "telemetry_groups": REPORT_TELEMETRY_GROUPS,
                "link_topology": LINK_TOPOLOGY,
                "comms_policy": COMMS_POLICY,
                "sensor_fusion": fusion_payload,
                "dynamic_speed_profile": dyn_speed_payload,
                "wind_assist": wind_payload,
                "horizon_lock": horizon_payload,
                "camera_adaptation": camera_adapt_payload,
                "autonomy_health": autonomy_health_payload,
                "virtual_anchor": virtual_anchor_payload,
            }

            now = time.monotonic()
            
            # Caching Mechanism: Determine if meaningful state has changed (ignoring age/time fluctuations)
            logical_state_changed = True
            if self._last_written_state is not None:
                old = self._last_written_state
                # Check critical functional elements
                if (old.get('state') == payload['state'] and
                    old.get('active') == payload['active'] and
                    old.get('hold_reason') == payload['hold_reason'] and
                    old.get('mission_end_reason') == payload['mission_end_reason'] and
                    old.get('guided_ready') == payload['guided_ready'] and
                    old.get('guided_ready_missing') == payload['guided_ready_missing'] and
                    old.get('guided_position_source') == payload['guided_position_source'] and
                    old.get('fc_gps_ekf_ready') == payload['fc_gps_ekf_ready'] and
                    old.get('fc_position_valid') == payload['fc_position_valid'] and
                    old.get('fc_nav_mode_ready') == payload['fc_nav_mode_ready'] and
                    old.get('fc_guided_block_reason') == payload['fc_guided_block_reason'] and
                    old.get('fc_ekf_flags') == payload['fc_ekf_flags'] and
                    old.get('fc_ekf_variances') == payload['fc_ekf_variances'] and
                    old.get('fc_ekf_report_seen') == payload['fc_ekf_report_seen'] and
                    old.get('start_phase') == payload['start_phase'] and
                    old.get('last_mode_command') == payload['last_mode_command'] and
                    old.get('sim_actuation_fallback') == payload['sim_actuation_fallback'] and
                    old.get('sim_actuation_fallback_reason') == payload['sim_actuation_fallback_reason'] and
                    old.get('fc_gps_fix_type') == payload['fc_gps_fix_type'] and
                    old.get('gps_sats') == payload['gps_sats'] and
                    old.get('gpi_received') == payload['gpi_received'] and
                    old.get('sim_nav_valid') == payload['sim_nav_valid'] and
                    old.get('last_command_ack') == payload['last_command_ack'] and
                    old.get('last_statustext') == payload['last_statustext'] and
                    old.get('mission_current_seq') == payload['mission_current_seq'] and
                    old.get('mission_reached_seq') == payload['mission_reached_seq'] and
                    old.get('race_p1_completion_source') == payload['race_p1_completion_source'] and
                    old.get('target') == payload['target'] and
                    old.get('wp_info') == payload['wp_info'] and
                    old.get('active_waypoint_index') == payload['active_waypoint_index'] and
                    old.get('objective_phase') == payload['objective_phase'] and
                    old.get('guidance_detail_source') == payload['guidance_detail_source'] and
                    old.get('guidance_mode') == payload['guidance_mode'] and
                    old.get('guidance_reason') == payload['guidance_reason'] and
                    old.get('avoidance_bias_deg') == payload['avoidance_bias_deg'] and
                    old.get('cross_track_error_m') == payload['cross_track_error_m'] and
                    old.get('nominal_heading_deg') == payload['nominal_heading_deg'] and
                    old.get('gate_assist_bias_deg') == payload['gate_assist_bias_deg'] and
                    old.get('progress_along_leg_m') == payload['progress_along_leg_m'] and
                    old.get('nav_target_distance_delta_m') == payload['nav_target_distance_delta_m'] and
                    old.get('nav_target_distance_increase_count') == payload['nav_target_distance_increase_count'] and
                    old.get('nav_heading_error_delta_deg') == payload['nav_heading_error_delta_deg'] and
                    old.get('nav_diagnostic_reason') == payload['nav_diagnostic_reason'] and
                    old.get('waypoint_passed_gate') == payload['waypoint_passed_gate'] and
                    old.get('waypoint_accept_reason') == payload['waypoint_accept_reason'] and
                    old.get('motor_limit_reason') == payload['motor_limit_reason'] and
                    old.get('mission_lifecycle') == payload['mission_lifecycle'] and
                    old.get('mission_upload_source') == payload['mission_upload_source'] and
                    old.get('mission_synced') == payload['mission_synced'] and
                    old.get('mission_profile_valid') == payload['mission_profile_valid'] and
                    old.get('mission_profile_race_ready') == payload['mission_profile_race_ready'] and
                    old.get('mission_profile_error') == payload['mission_profile_error'] and
                    old.get('phase_transition_policy') == payload['phase_transition_policy'] and
                    old.get('p2_min_gate_count') == payload['p2_min_gate_count'] and
                    old.get('p3_engagement_mode') == payload['p3_engagement_mode'] and
                    old.get('pixhawk_mission_count') == payload['pixhawk_mission_count'] and
                    old.get('target_color') == payload['target_color'] and
                    old.get('perception_policy') == payload['perception_policy'] and
                    old.get('gate_count') == payload['gate_count'] and
                    old.get('command_lock') == payload['command_lock'] and
                    old.get('failsafe_state') == payload['failsafe_state'] and
                    old.get('estop_state') == payload['estop_state'] and
                    old.get('estop_source') == payload['estop_source'] and
                    old.get('nav_position_source') == payload['nav_position_source'] and
                    old.get('nav_heading_source') == payload['nav_heading_source'] and
                    old.get('nav_source_detail') == payload['nav_source_detail'] and
                    old.get('nav_fix_valid') == payload['nav_fix_valid'] and
                    old.get('nav_target_bearing_deg') == payload['nav_target_bearing_deg'] and
                    old.get('nav_target_distance_m') == payload['nav_target_distance_m'] and
                    old.get('nav_heading_error_deg') == payload['nav_heading_error_deg'] and
                    old.get('nav_align_mode') == payload['nav_align_mode'] and
                    old.get('nav_align_phase') == payload['nav_align_phase'] and
                    old.get('guided_vx_north_mps') == payload['guided_vx_north_mps'] and
                    old.get('guided_vy_east_mps') == payload['guided_vy_east_mps'] and
                    old.get('camera_pipeline') == payload['camera_pipeline'] and
                    old.get('v_target') == payload['v_target'] and
                    old.get('current_speed_mps') == payload['current_speed_mps'] and
                    old.get('current_yaw_rate_dps') == payload['current_yaw_rate_dps'] and
                    old.get('heading_target') == payload['heading_target'] and
                    old.get('cmd_port_pwm') == payload['cmd_port_pwm'] and
                    old.get('cmd_stbd_pwm') == payload['cmd_stbd_pwm'] and
                    old.get('sim_rc_override_semantics') == payload['sim_rc_override_semantics'] and
                    old.get('rc_override_ch1') == payload['rc_override_ch1'] and
                    old.get('rc_override_ch3') == payload['rc_override_ch3'] and
                    old.get('last_servo_output_raw') == payload['last_servo_output_raw'] and
                    old.get('last_nav_controller_output') == payload['last_nav_controller_output'] and
                    old.get('servo_yaw_delta_pwm') == payload['servo_yaw_delta_pwm'] and
                    old.get('servo_yaw_sign_mismatch') == payload['servo_yaw_sign_mismatch'] and
                    old.get('motor_command_source') == payload['motor_command_source'] and
                    old.get('pwm_trim_port_us') == payload['pwm_trim_port_us'] and
                    old.get('pwm_trim_stbd_us') == payload['pwm_trim_stbd_us'] and
                    old.get('wrong_turn_count') == payload['wrong_turn_count'] and
                    old.get('wrong_turn_active') == payload['wrong_turn_active'] and
                    old.get('heading_damping_hold_active') == payload['heading_damping_hold_active'] and
                    old.get('heading_damping_hold_count') == payload['heading_damping_hold_count'] and
                    old.get('heading_control_diagnostic') == payload['heading_control_diagnostic'] and
                    old.get('timeout_count') == payload['timeout_count'] and
                    old.get('camera_ready') == payload['camera_ready'] and
                    old.get('camera_status_stale') == payload['camera_status_stale'] and
                    old.get('lidar_ready') == payload['lidar_ready'] and
                    old.get('sensor_fusion') == payload['sensor_fusion'] and
                    old.get('dynamic_speed_profile') == payload['dynamic_speed_profile'] and
                    old.get('wind_assist') == payload['wind_assist'] and
                    old.get('horizon_lock') == payload['horizon_lock'] and
                    old.get('camera_adaptation') == payload['camera_adaptation'] and
                    old.get('autonomy_health') == payload['autonomy_health'] and
                    old.get('virtual_anchor') == payload['virtual_anchor'] and
                    round(float(old.get('lidar_left_m', -1.0) or -1.0), 2) == round(float(payload['lidar_left_m']), 2) and
                    round(float(old.get('lidar_center_m', -1.0) or -1.0), 2) == round(float(payload['lidar_center_m']), 2) and
                    round(float(old.get('lidar_right_m', -1.0) or -1.0), 2) == round(float(payload['lidar_right_m']), 2) and
                    round(float(old.get('min_obstacle_m', -1.0) or -1.0), 2) == round(float(payload['min_obstacle_m']), 2)):
                    logical_state_changed = False
            
            # Write only if logical state changed or 1 second heartbeat timeout exceeded
            if logical_state_changed or (now - self._last_state_write_time >= 1.0):
                if not atomic_write_json(STATE_FILE, payload):
                    self._bump_error(
                        "state_write_error",
                        "[WARN] [STATE] Atomic yazma basarisiz; onceki gecerli state korunuyor",
                    )
                    return

                self._last_state_write_time = now
                self._last_written_state = payload
                
                # LOG: State file write frequency
                if not hasattr(self, '_last_state_log') or (now - self._last_state_log) >= 3.0:
                    print(f"[STATE_W] Wrote mission_state.json (state={self.state} active={self.mission_active} hb_age={self.link_heartbeat_age_s:.1f}s)")
                    self._last_state_log = now
                
        except Exception as exc:
            self._bump_error("state_write_error", f"[WARN] [STATE] Yazma hatasi: {exc}")

    def _init_file3_recorder(self):
        try:
            import cv2

            self._cv2 = cv2
            os.makedirs(LOG_DIR, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._file3_writer = cv2.VideoWriter(FILE3_MAP_MP4, fourcc, 1.0, (600, 600))
            if not self._file3_writer or not self._file3_writer.isOpened():
                print("[WARN] [FILE3] VideoWriter acilamadi.")
                self._file3_writer = None
            self._file3_index_file = open(FILE3_INDEX_CSV, "w", newline="", encoding="utf-8")
            self._file3_index_writer = csv.writer(self._file3_index_file)
            self._file3_index_writer.writerow(
                ["ts_monotonic", "lat", "lon", "heading", "left_m", "center_m", "right_m"]
            )
            self._file3_index_file.flush()
            print(f"[REC] [FILE3] Harita kaydi: {FILE3_MAP_MP4}")
            print(f"[IDX] [FILE3] Indeks: {FILE3_INDEX_CSV}")
        except Exception as exc:
            print(f"[WARN] [FILE3] Baslatma hatasi: {exc}")
            self._file3_writer = None
            self._file3_index_file = None
            self._file3_index_writer = None

    def _close_file3_recorder(self):
        try:
            if self._file3_writer:
                self._file3_writer.release()
        except Exception as exc:
            self._warn_throttled("file3_close_writer", f"[WARN] [FILE3] Writer kapatma hatasi: {exc}")
        try:
            if self._file3_index_file:
                self._file3_index_file.close()
        except Exception as exc:
            self._warn_throttled("file3_close_index", f"[WARN] [FILE3] Index kapatma hatasi: {exc}")

    def _record_file3_if_due(self):
        now = time.monotonic()
        if now - self._file3_last_ts < 1.0:
            return
        self._file3_last_ts = now

        if self._file3_index_writer:
            self._file3_index_writer.writerow(
                [
                    round(now, 3),
                    round(self.current_lat, 7),
                    round(self.current_lon, 7),
                    round(self.current_heading, 2),
                    round(self.lidar_left_dist, 2),
                    round(self.lidar_center_dist, 2),
                    round(self.lidar_right_dist, 2),
                ]
            )
            self._file3_index_file.flush()

        if not self._cv2:
            return

        cv2 = self._cv2
        img = cv2.cvtColor(cv2.UMat(600, 600, cv2.CV_8UC1).get(), cv2.COLOR_GRAY2BGR)
        cx, cy = 300, 300
        cv2.arrowedLine(img, (cx, cy + 16), (cx, cy - 26), (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)

        scale = 20.0
        for x_m, y_m in self._map_points[:2500]:
            px = int(cx - y_m * scale)
            py = int(cy - x_m * scale)
            if 0 <= px < 600 and 0 <= py < 600:
                img[py, px] = (255, 255, 255)

        cv2.putText(
            img,
            f"L:{self.lidar_left_dist:.1f} C:{self.lidar_center_dist:.1f} R:{self.lidar_right_dist:.1f}",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
        )
        cv2.putText(
            img,
            f"TS:{time.strftime('%H:%M:%S')}",
            (12, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (180, 180, 180),
            1,
        )
        if self._file3_writer:
            self._file3_writer.write(img)
        try:
            cv2.imwrite(FILE3_MAP_JPG, img)
        except Exception as exc:
            self._warn_throttled("file3_jpg", f"[WARN] [FILE3] JPG yazma hatasi: {exc}")

    def _current_sim_pose_gazebo(self):
        try:
            import json
            from pathlib import Path

            with open(Path(CONTROL_DIR) / "vehicle_position.json", "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            return (
                float(payload.get("pos_x", 0.0) or 0.0),
                float(payload.get("pos_y", 0.0) or 0.0),
                float(payload.get("heading_rad", 0.0) or 0.0),
            )
        except Exception:
            return 0.0, 0.0, 0.0

    def _get_spatial_origin_latlon(self):
        cached_lat = getattr(self, "_spatial_origin_lat", None)
        cached_lon = getattr(self, "_spatial_origin_lon", None)
        if cached_lat is not None and cached_lon is not None:
            return float(cached_lat), float(cached_lon)
        if self.nav_waypoints:
            try:
                return float(self.nav_waypoints[0][0]), float(self.nav_waypoints[0][1])
            except (TypeError, ValueError, IndexError):
                pass
        home_lat, home_lon, _, _ = parse_sim_home()
        return float(home_lat), float(home_lon)

    def _current_spatial_pose(self):
        """Return boat pose as (east_m, north_m, heading_rad) in unified ENU."""
        origin_lat, origin_lon = self._get_spatial_origin_latlon()
        return current_spatial_pose_enu(
            simulation_mode=bool(getattr(self, "simulation_mode", False)),
            origin_lat=origin_lat,
            origin_lon=origin_lon,
            current_lat=float(getattr(self, "current_lat", 0.0) or 0.0),
            current_lon=float(getattr(self, "current_lon", 0.0) or 0.0),
            current_heading_deg=float(getattr(self, "current_heading", 0.0) or 0.0),
        )

    def _lidar_local_to_world_enu(self, x_local, y_local, pose=None):
        if self.simulation_mode or os.environ.get("USV_SIM") == "1":
            pos_x, pos_y, heading_rad = self._current_sim_pose_gazebo()
            return lidar_local_to_world_enu_gazebo(x_local, y_local, pos_x, pos_y, heading_rad)
        east_m, north_m, heading_rad = pose if pose is not None else self._current_spatial_pose()
        heading_deg = math.degrees(float(heading_rad)) % 360.0
        return lidar_local_to_world_enu_compass(x_local, y_local, east_m, north_m, heading_deg)

    def _world_grid_key(self, wx_m, wy_m):
        ix = int(math.floor(float(wx_m) / float(LIDAR_GRID_RES_M)))
        iy = int(math.floor(float(wy_m) / float(LIDAR_GRID_RES_M)))
        return ix, iy

    @staticmethod
    def _world_grid_cell_center(ix, iy):
        return (float(ix) + 0.5) * float(LIDAR_GRID_RES_M), (float(iy) + 0.5) * float(LIDAR_GRID_RES_M)

    def _world_enu_to_lidar_local(self, wx_m, wy_m, pose=None):
        if self.simulation_mode or os.environ.get("USV_SIM") == "1":
            pos_x, pos_y, heading_rad = self._current_sim_pose_gazebo()
            y_gazebo = -float(wx_m)
            x_gazebo = float(wy_m)
            dx = x_gazebo - float(pos_x)
            dy = y_gazebo - float(pos_y)
            c = math.cos(float(heading_rad))
            s = math.sin(float(heading_rad))
            x_local = (dx * c) + (dy * s)
            y_local = (-dx * s) + (dy * c)
            return float(x_local), float(y_local)
        east_m, north_m, heading_rad = pose if pose is not None else self._current_spatial_pose()
        heading_deg = math.degrees(float(heading_rad)) % 360.0
        return world_enu_to_lidar_local_compass(wx_m, wy_m, east_m, north_m, heading_deg)

    def _find_pixhawk_port(self):
        try:
            from pymavlink import mavutil

            m = mavutil.mavlink_connection("udpin:0.0.0.0:14551", baud=BAUD_RATE)
            if m.wait_heartbeat(timeout=3):
                print("[OK] [MAV] Pixhawk UDP:14551")
                return m
            m.close()
        except Exception as exc:
            self._bump_error("mav_read_error", f"[WARN] [MAV] Mesaj drain hatasi: {exc}")

        for port in glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"):
            try:
                from pymavlink import mavutil

                m = mavutil.mavlink_connection(port, baud=BAUD_RATE)
                if m.wait_heartbeat(timeout=2):
                    print(f"[OK] [MAV] Pixhawk seri: {port}")
                    return m
                m.close()
            except Exception:
                pass
        return None

    def _connect_pixhawk(self):
        print("[MAV] Pixhawk aranıyor...")
        try:
            from pymavlink import mavutil

            if os.environ.get("USV_SIM") == "1":
                print("[SIM] Otonomi MAVLink Simulasyon modunda calisiyor...")
                sim_endpoints = [
                    "tcp:127.0.0.1:5760",
                    "udpin:127.0.0.1:14550",
                ]
                self.master = None
                for endpoint in sim_endpoints:
                    candidate = None
                    try:
                        candidate = mavutil.mavlink_connection(endpoint, baud=BAUD_RATE)
                        heartbeat = candidate.wait_heartbeat(timeout=5)
                        if not heartbeat:
                            raise RuntimeError("heartbeat timeout")
                        candidate.target_system = int(heartbeat.get_srcSystem() or candidate.target_system or 1)
                        candidate.target_component = int(heartbeat.get_srcComponent() or candidate.target_component or 1)
                        self.vehicle_mode_custom = int(getattr(heartbeat, "custom_mode", 0) or 0)
                        self.vehicle_mode_name = ARDUROVER_MODE_NAMES.get(
                            self.vehicle_mode_custom,
                            f"UNKNOWN({self.vehicle_mode_custom})",
                        )
                        self.master = candidate
                        print(
                            f"[OK] [MAV] SIM endpoint baglandi: {endpoint} "
                            f"target={self.master.target_system}/{self.master.target_component}"
                        )
                        break
                    except Exception as exc:
                        print(f"[WARN] [MAV] SIM endpoint hatasi ({endpoint}): {exc}")
                        try:
                            candidate.close()
                        except Exception:
                            pass
                if not self.master:
                    raise RuntimeError("SIM MAVLink endpoint bulunamadi")
                self.simulation_mode = True  # ← FIX: Simülasyonda True olmalı, watchdog heartbeat check'i skip yapılsın
            else:
                self.master = self._find_pixhawk_port()
                self.simulation_mode = False  # Real hardware için False
            
            if not self.master:
                raise RuntimeError("Pixhawk bulunamadi")
            if int(getattr(self.master, "target_system", 0) or 0) <= 0:
                self.master.target_system = 1
            if int(getattr(self.master, "target_component", 0) or 0) <= 0:
                self.master.target_component = 1
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,
                1,
            )
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                10,
                1,
            )
            try:
                intervals = (
                    (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000.0),
                    (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 200000.0),
                    (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 200000.0),
                    (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 500000.0),
                    (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 200000.0),
                )
                for msg_id, interval_us in intervals:
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,
                        float(msg_id),
                        float(interval_us),
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    )
                print("[MAV] SET_MESSAGE_INTERVAL requested for GPI/GPS/EKF/SYS_STATUS/RC")
            except Exception as _interval_exc:
                print(f"[WARN] [MAV] SET_MESSAGE_INTERVAL basarisiz: {_interval_exc}")
            print(f"[MAV] Target system/component: {self.master.target_system}/{self.master.target_component}")
            self.last_heartbeat_time = time.monotonic()
            self._send_companion_heartbeat_if_due(force=True)
            print("[OK] [MAV] Baglanti hazir")
        except Exception as exc:
            print(f"[WARN] [MAV] Baglanti yok: {exc}")
            if os.environ.get("USV_SIM") == "1":
                print("[HATA] [SIM] SITL baglantisi kurulamadi. Simulasyon stack'ini kontrol edin.")
                print("[HATA] [SIM] run_sim_stack.sh uzerinden baslatin veya SITL calistigini dogrulayin.")
                self.master = None
                self.simulation_mode = False
            else:
                print("[WARN] DONANIM BULUNAMADI - OFFLINE TEST MODU AKTIF")
                self.master = None
                self.simulation_mode = True

    def _send_companion_heartbeat_if_due(self, force=False):
        if not self.master:
            return
        now = time.monotonic()
        if not force and (now - float(self._last_companion_heartbeat_time or 0.0)) < 1.0:
            return
        try:
            from pymavlink import mavutil

            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self._last_companion_heartbeat_time = now
        except Exception as exc:
            self._warn_throttled(
                "companion_heartbeat",
                f"[WARN] [MAV] companion heartbeat gonderilemedi: {exc}",
                period_s=5.0,
            )

    def _try_connect_lidar(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
            from sensor_msgs.msg import LaserScan

            try:
                rclpy.init()
            except RuntimeError:
                pass

            class _LidarNode(Node):
                def __init__(self, parent):
                    super().__init__("usv_lidar_listener")
                    self.parent = parent
                    # BEST_EFFORT + depth=1: en guncel tarama (depth=10 eski mesafe/kaçınma titremesi)
                    lidar_qos = QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=1,
                    )
                    self.create_subscription(
                        LaserScan, "/scan", self.cb, lidar_qos
                    )

                def cb(self, msg):
                    self.parent.last_lidar_time = time.monotonic()
                    frame = self.parent._validate_lidar_frame(msg)
                    frame_valid = bool(frame.get("valid"))
                    points = []
                    front_samples = []
                    stable_points = []
                    tentative_points = []
                    left = 99.0
                    center = 99.0
                    right = 99.0
                    if frame_valid:
                        filtered = frame.get("points", [])
                        front_samples = list(frame.get("front_samples", []))
                        stable_points, tentative_points = self.parent._update_lidar_confidence_points(filtered)
                        quality_ready = bool(len(stable_points) > 0)
                        self.parent._lidar_quality_ready = bool(quality_ready)
                        self.parent._stable_map_points = [(p[2], p[3]) for p in stable_points]
                        self.parent._update_lidar_persistent_map(stable_points, tentative_points, frame_valid=True)
                        metrics = self.parent._compute_map_sector_metrics()
                        map_front = float(metrics.get("front_min_m", 99.0))
                        map_left = float(metrics.get("left_min_m", 99.0))
                        map_right = float(metrics.get("right_min_m", 99.0))

                        for _, _, x_m, y_m in filtered:
                            points.append((x_m, y_m))
                        if quality_ready:
                            for deg, r in front_samples:
                                sector = classify_lidar_scan_sector(deg)
                                if sector == "left":
                                    left = min(left, r)
                                elif sector == "right":
                                    right = min(right, r)
                                elif sector == "center":
                                    center = min(center, r)
                            current_left = float(left)
                            current_center = float(center)
                            current_right = float(right)
                            # Prefer persistent-map sectors only after the current frame has stable hits.
                            if map_left < 98.0:
                                left = min(left, map_left)
                            if map_right < 98.0:
                                right = min(right, map_right)
                            if map_front < 98.0:
                                center = min(center, map_front)
                        else:
                            front_samples = []
                            current_left = 99.0
                            current_center = 99.0
                            current_right = 99.0
                        self.parent._lidar_frame_status = str(frame.get("reason", "ok"))
                        self.parent._lidar_drop_reason = str(frame.get("drop_reason", "ok"))
                        self.parent._lidar_stale_consecutive = 0
                        self.parent._lidar_stale_warned = False
                        self.parent._lidar_stale_blocked = False
                        self.parent._lidar_unreliable_clock_consecutive = 0
                        self.parent._lidar_local_obstacle_valid = bool(quality_ready)
                    else:
                        current_left = 99.0
                        current_center = 99.0
                        current_right = 99.0
                        self.parent._stable_map_points = []
                        self.parent._lidar_quality_ready = False
                        self.parent._lidar_frame_status = f"degraded:{frame.get('reason', 'invalid')}"
                        self.parent._lidar_drop_reason = str(frame.get("drop_reason", frame.get("reason", "invalid")))
                        drop_reason = str(self.parent._lidar_drop_reason)
                        metrics = self.parent._lidar_map_metrics if isinstance(self.parent._lidar_map_metrics, dict) else {}
                        if drop_reason == "stale_unreliable_clock":
                            self.parent._lidar_unreliable_clock_consecutive += 1
                            self.parent._lidar_clock_reject_count += 1
                            self.parent._lidar_stale_consecutive = 0
                            self.parent._lidar_stale_warned = False
                            self.parent._lidar_stale_blocked = False
                            decay_gain = min(
                                float(LIDAR_UNRELIABLE_DECAY_GAIN_MAX),
                                1.0 + (0.6 * float(self.parent._lidar_unreliable_clock_consecutive)),
                            )
                            self.parent._update_lidar_persistent_map(
                                [],
                                [],
                                frame_valid=False,
                                decay_gain=decay_gain,
                                stable_drop=int(LIDAR_UNRELIABLE_STABLE_DROP),
                            )
                            metrics = self.parent._compute_map_sector_metrics()
                            self.parent._apply_unreliable_clock_risk_decay()
                            raw_left, raw_center, raw_right, raw_valid_hits = self.parent._extract_lidar_sector_mins_raw(msg)
                            if raw_valid_hits > 0:
                                left = min(left, float(raw_left))
                                center = min(center, float(raw_center))
                                right = min(right, float(raw_right))
                                self.parent._lidar_local_obstacle_valid = True
                            else:
                                self.parent._lidar_local_obstacle_valid = False
                            now_lidar = time.monotonic()
                            if (now_lidar - float(self.parent._lidar_clock_reject_last_log_ts)) >= 1.0:
                                self.parent._lidar_clock_reject_last_log_ts = now_lidar
                                log_jsonl(
                                    "usv_main",
                                    False,
                                    event="lidar_clock_reject",
                                    drop_reason="stale_unreliable_clock",
                                    clock_source=str(frame.get("clock_source", "unknown")),
                                    stamp_age_s=frame.get("stamp_age_s"),
                                    consecutive=int(self.parent._lidar_unreliable_clock_consecutive),
                                    reject_count=int(self.parent._lidar_clock_reject_count),
                                    raw_valid_hits=int(raw_valid_hits),
                                    map_occupied_cells=int(self.parent._lidar_occ_cells_count),
                                    front_risk=round(float(self.parent._lidar_front_risk), 3),
                                )
                        else:
                            self.parent._lidar_unreliable_clock_consecutive = 0
                            self.parent._lidar_local_obstacle_valid = False
                        if str(frame.get("reason", "")) == "stale":
                            self.parent._lidar_stale_consecutive += 1
                            self.parent._lidar_stale_blocked = self.parent._lidar_stale_consecutive >= int(LIDAR_STALE_WARN_CONSEC)
                            if (
                                self.parent._lidar_stale_consecutive >= int(LIDAR_STALE_WARN_CONSEC)
                                and not self.parent._lidar_stale_warned
                            ):
                                self.parent._lidar_stale_warned = True
                                stale_age = frame.get("stamp_age_s")
                                stale_age_disp = f"{float(stale_age):.3f}" if stale_age is not None else "None"
                                print(
                                    f"[WARN] [LIDAR] degraded:stale ardışık={self.parent._lidar_stale_consecutive} "
                                    f"clock={frame.get('clock_source', 'unknown')} age_s={stale_age_disp}"
                                )
                                log_jsonl(
                                    "usv_main",
                                    False,
                                    event="lidar_stale_warning",
                                    consecutive=int(self.parent._lidar_stale_consecutive),
                                    status=str(self.parent._lidar_frame_status),
                                    clock_source=str(frame.get("clock_source", "unknown")),
                                    stamp_age_s=frame.get("stamp_age_s"),
                                )
                        else:
                            self.parent._lidar_stale_consecutive = 0
                            self.parent._lidar_stale_warned = False
                            self.parent._lidar_stale_blocked = False
                        if drop_reason != "stale_unreliable_clock":
                            decay_gain = 1.0
                            stable_drop = 0
                            if drop_reason == "low_valid_ratio":
                                decay_gain = float(LIDAR_UNRELIABLE_DECAY_GAIN_MAX)
                                stable_drop = max(1, int(LIDAR_UNRELIABLE_STABLE_DROP))
                            self.parent._update_lidar_persistent_map(
                                [],
                                [],
                                frame_valid=False,
                                decay_gain=decay_gain,
                                stable_drop=stable_drop,
                            )
                            metrics = self.parent._compute_map_sector_metrics()
                        map_front = float(metrics.get("front_min_m", 99.0))
                        map_left = float(metrics.get("left_min_m", 99.0))
                        map_right = float(metrics.get("right_min_m", 99.0))
                        if drop_reason not in ("low_valid_ratio", "empty") and map_left < 98.0:
                            left = map_left
                        if drop_reason not in ("low_valid_ratio", "empty") and map_right < 98.0:
                            right = map_right
                        if drop_reason not in ("low_valid_ratio", "empty") and map_front < 98.0:
                            center = map_front
                    if frame_valid and (left >= 98.5 and center >= 98.5 and right >= 98.5):
                        self.parent._lidar_local_obstacle_valid = False
                    self.parent._lidar_frame_valid_ratio = float(frame.get("valid_ratio", 0.0) or 0.0)
                    self.parent._lidar_current_left_m = float(current_left)
                    self.parent._lidar_current_center_m = float(current_center)
                    self.parent._lidar_current_right_m = float(current_right)
                    self.parent._lidar_clock_source = str(frame.get("clock_source", "none"))
                    stamp_age_val = frame.get("stamp_age_s")
                    self.parent._lidar_stamp_age_s = float(stamp_age_val) if isinstance(stamp_age_val, (int, float)) else None
                    self.parent._lidar_raw_sector_min_m = float(min(left, center, right))
                    hold = self.parent._lidar_sector_hold
                    sector_ts = self.parent._lidar_sector_ts
                    now_mono = time.monotonic()
                    hold_ttl_s = 0.58
                    if left < 98.5:
                        hold["left"] = left
                        sector_ts["left"] = now_mono
                    elif hold["left"] is not None and (now_mono - sector_ts.get("left", 0.0)) <= hold_ttl_s:
                        left = hold["left"]
                    else:
                        hold["left"] = None
                    if center < 98.5:
                        hold["center"] = center
                        sector_ts["center"] = now_mono
                    elif hold["center"] is not None and (now_mono - sector_ts.get("center", 0.0)) <= hold_ttl_s:
                        center = hold["center"]
                    else:
                        hold["center"] = None
                    if right < 98.5:
                        hold["right"] = right
                        sector_ts["right"] = now_mono
                    elif hold["right"] is not None and (now_mono - sector_ts.get("right", 0.0)) <= hold_ttl_s:
                        right = hold["right"]
                    else:
                        hold["right"] = None

                    p = self.parent
                    sl = _smooth_lidar_sector_m(p._lidar_ema_left, left)
                    sc = _smooth_lidar_sector_m(p._lidar_ema_center, center)
                    sr = _smooth_lidar_sector_m(p._lidar_ema_right, right)
                    p._lidar_ema_left = sl
                    p._lidar_ema_center = sc
                    p._lidar_ema_right = sr
                    self.parent.lidar_left_dist = sl
                    self.parent.lidar_center_dist = sc
                    self.parent.lidar_right_dist = sr
                    self.parent.min_obstacle_distance = min(sl, sc, sr)
                    self.parent.obstacle_detected = center < D_MIN_M
                    self.parent._map_points = points
                    self.parent._lidar_front_samples = front_samples
                    self.parent._lidar_last_frame_monotonic = now_mono
                    self.parent.lidar_ready = bool(
                        self.parent.lidar_available
                        and (now_mono - float(self.parent.last_lidar_time)) < float(LIDAR_READY_TIMEOUT_S)
                        and self.parent._lidar_quality_ready
                    )
                    if (now_mono - float(self.parent._lidar_last_log_ts)) >= 1.0:
                        self.parent._lidar_last_log_ts = now_mono
                        log_jsonl(
                            "usv_main",
                            False,
                            event="lidar_frame",
                            status=str(self.parent._lidar_frame_status),
                            drop_reason=str(self.parent._lidar_drop_reason),
                            clock_source=str(self.parent._lidar_clock_source),
                            stamp_age_s=self.parent._lidar_stamp_age_s,
                            valid_ratio=round(float(self.parent._lidar_frame_valid_ratio), 3),
                            stable_points=int(self.parent._lidar_stable_point_count),
                            tentative_points=int(self.parent._lidar_tentative_point_count),
                            map_occupied_cells=int(self.parent._lidar_occ_cells_count),
                            front_risk=round(float(self.parent._lidar_front_risk), 3),
                            lidar_local_obstacle_valid=bool(self.parent._lidar_local_obstacle_valid),
                            lidar_degraded_mode=str(self.parent._lidar_degraded_mode),
                            stale_consecutive=int(self.parent._lidar_stale_consecutive),
                            guidance_reason=str(self.parent._guidance_reason_with_lidar_block()),
                        )

            node = _LidarNode(self)

            def spin():
                try:
                    rclpy.spin(node)
                except Exception:
                    pass

            t = threading.Thread(target=spin, daemon=True)
            t.start()
            self.lidar_available = True
            print("[OK] [LIDAR] Dinleniyor")
        except ImportError:
            print("[WARN] [LIDAR] rclpy yok - lidar devre disi")
            self.lidar_available = False
        except Exception as exc:
            print(f"[WARN] [LIDAR] Hata: {exc}")
            self.lidar_available = False

    def _set_mode(self, mode_name):
        if not self.master:
            return True
        try:
            from pymavlink import mavutil

            mapping = self.master.mode_mapping() or {}
            custom_mode = mapping.get(mode_name)
            if custom_mode is None:
                print(f"[WARN] [MODE] {mode_name} bulunamadi")
                self.last_mode_command.update({
                    "requested": str(mode_name),
                    "confirmed": False,
                    "actual_mode": str(self.vehicle_mode_name),
                    "actual_custom_mode": self.vehicle_mode_custom,
                    "statustext": "mode_mapping_missing",
                    "ts_monotonic": round(time.monotonic(), 3),
                })
                if mode_name == "GUIDED":
                    self.mission_end_reason = "mode_change_failed:GUIDED"
                return False
            if mode_name == "GUIDED" and int(custom_mode) != GUIDED_EXPECTED_MODE_ID:
                print(
                    f"[ERROR] [MODE] GUIDED mapping beklenen {GUIDED_EXPECTED_MODE_ID}, "
                    f"gelen {custom_mode}; komut reddedildi"
                )
                self.last_mode_command.update({
                    "requested": str(mode_name),
                    "custom_mode": int(custom_mode),
                    "confirmed": False,
                    "actual_mode": str(self.vehicle_mode_name),
                    "actual_custom_mode": self.vehicle_mode_custom,
                    "statustext": "guided_mode_mapping_error",
                    "ts_monotonic": round(time.monotonic(), 3),
                })
                self.mission_end_reason = "mode_change_failed:GUIDED"
                log_jsonl(
                    "usv_main",
                    False,
                    event="guided_mode_mapping_error",
                    expected=GUIDED_EXPECTED_MODE_ID,
                    actual=int(custom_mode),
                )
                return False
            target_system = int(getattr(self.master, "target_system", 0) or 0)
            target_component = int(getattr(self.master, "target_component", 0) or 0)
            if target_system <= 0:
                target_system = 1
                self.master.target_system = target_system
            if target_component <= 0:
                target_component = 1
                self.master.target_component = target_component
            command_ts = time.monotonic()
            self.last_mode_command = {
                "requested": str(mode_name),
                "custom_mode": int(custom_mode),
                "target_system": target_system,
                "target_component": target_component,
                "confirmed": False,
                "actual_mode": str(self.vehicle_mode_name),
                "actual_custom_mode": self.vehicle_mode_custom,
                "ack_result": "",
                "statustext": "",
                "ts_monotonic": round(command_ts, 3),
            }
            try:
                self.master.set_mode(int(custom_mode))
            except Exception:
                pass
            self.master.mav.set_mode_send(
                target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                int(custom_mode),
            )
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                self._drain_mav_messages()
                actual_mode = self.vehicle_mode_custom
                if actual_mode is not None and int(actual_mode) == int(custom_mode):
                    self.vehicle_mode_name = str(mode_name)
                    self.last_mode_command.update({
                        "confirmed": True,
                        "actual_mode": str(self.vehicle_mode_name),
                        "actual_custom_mode": int(actual_mode),
                        "ack_result": str(self.last_command_ack.get("result_name", "")),
                        "statustext": str(self.last_statustext.get("text", "")),
                        "ts_monotonic": round(time.monotonic(), 3),
                    })
                    self._update_mode_state(source="mode_cmd", reason=f"set_mode:{mode_name}")
                    print(f"[MODE] [MODE] {mode_name}")
                    return True
                time.sleep(0.05)
            statustext = ""
            try:
                if float(self.last_statustext.get("ts_monotonic", 0.0) or 0.0) >= command_ts:
                    statustext = str(self.last_statustext.get("text", "") or "")
            except Exception:
                statustext = ""
            ack_result = str(self.last_command_ack.get("result_name", "") or "")
            self.last_mode_command.update({
                "confirmed": False,
                "actual_mode": str(self.vehicle_mode_name),
                "actual_custom_mode": self.vehicle_mode_custom,
                "ack_result": ack_result,
                "statustext": statustext,
                "ts_monotonic": round(time.monotonic(), 3),
            })
            if mode_name == "GUIDED":
                self.mission_end_reason = "mode_change_failed:GUIDED"
                log_jsonl(
                    "usv_main",
                    False,
                    event="mode_change_failed",
                    requested=str(mode_name),
                    custom_mode=int(custom_mode),
                    actual_mode=str(self.vehicle_mode_name),
                    actual_custom_mode=self.vehicle_mode_custom,
                    ack_result=ack_result,
                    statustext=statustext,
                )
            self._warn_throttled(
                f"mode_confirm_{mode_name}",
                (
                    f"[WARN] [MODE] {mode_name} onaylanmadi; "
                    f"actual={self.vehicle_mode_name}/{self.vehicle_mode_custom} "
                    f"ack={ack_result or '--'} text={statustext or '--'}"
                ),
                period_s=1.0,
            )
            return False
        except Exception as exc:
            print(f"[WARN] [MODE] Hata ({mode_name}): {exc}")
            return False

    def _set_mode_sim_retry(self, mode_name, extra_attempts=24):
        """SIM: retry mode while FC GPS/EKF converges."""
        if self._set_mode(mode_name):
            return True
        if os.environ.get("USV_SIM") != "1" or not self.master:
            return False
        _attempts = max(1, int(extra_attempts))
        for attempt in range(1, _attempts + 1):
            print(f"[SIM] {mode_name} yeniden deneniyor ({attempt}/{_attempts})")
            if attempt == 1 or attempt % 5 == 0:
                try:
                    self._sitl_seed_global_origin_if_needed(force=True)
                except Exception:
                    pass
            for _inj in range(4):
                self._sitl_gps_input_from_sim()
                self._drain_mav_messages()
                time.sleep(0.04)
            time.sleep(0.12)
            if self._set_mode(mode_name):
                return True
        return False

    def _arm(self):
        if not self.master:
            return True
        try:
            def wait_until_armed(timeout_s):
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline:
                    self._drain_mav_messages()
                    if self.master.motors_armed():
                        self.vehicle_armed = True
                        self._update_mode_state(source="arm", reason="armed")
                        print("[OK] [ARM] Arac armed")
                        return True
                    time.sleep(0.1)
                return False

            self._drain_mav_messages()
            start_status = self._fc_start_status()
            if not start_status.get("ready", False):
                print(f"[WARN] [ARM] FC start hazir degil: {start_status.get('block_reason')}")
                return False

            self.master.arducopter_arm()
            if wait_until_armed(6.0):
                return True

            if os.environ.get("USV_SIM") == "1":
                self._drain_mav_messages()
                start_status = self._fc_start_status()
                if not start_status.get("ready", False):
                    print(f"[WARN] [ARM] Normal arm reddedildi; blocker={start_status.get('block_reason')}")
                    return False
                print("[WARN] [ARM] Normal arm timeout, SITL stabilize beklenip normal retry deneniyor")
                retry_deadline = time.monotonic() + max(8.0, float(ARM_STATUSTEXT_BLOCK_S))
                stable_since = None
                while time.monotonic() < retry_deadline:
                    self._service_startup_wait_once()
                    try:
                        self._sitl_gps_input_from_sim()
                    except Exception:
                        pass
                    arm_ready, _, start_status = self._arm_ready_now()
                    if arm_ready:
                        if stable_since is None:
                            stable_since = time.monotonic()
                        elif time.monotonic() - float(stable_since) >= float(FC_START_STABLE_S):
                            break
                    else:
                        stable_since = None
                    time.sleep(0.1)
                else:
                    print(f"[WARN] [ARM] Retry oncesi FC start hazir degil: {self.fc_start_block_reason}")
                    return False
                self.master.arducopter_arm()
                if wait_until_armed(4.0):
                    return True

            print("[WARN] [ARM] Arming timeout")
            return False
        except Exception as exc:
            print(f"[WARN] [ARM] Hata: {exc}")
            return False

    def _disarm(self):
        if not self.master:
            return True
        try:
            self.master.arducopter_disarm()
            self.vehicle_armed = False
            self._update_mode_state(source="disarm", reason="disarm_cmd")
            return True
        except Exception:
            return False

    def _set_rc7_estop(self, enabled):
        if not self.master:
            return True
        pwm = RC7_ESTOP_FORCE_PWM if enabled else RC7_SAFE_PWM
        try:
            for _ in range(5):
                rc = [65535] * 8
                rc[6] = pwm
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *rc,
                )
                time.sleep(0.04)
            return True
        except Exception as exc:
            print(f"[WARN] [RC7] E-stop komutu hatasi: {exc}")
            return False

    def _send_obstacle_distance_from_lidar_if_due(self, left_m, center_m, right_m):
        """Publish coarse onboard lidar sectors to Pixhawk OA as MAVLink OBSTACLE_DISTANCE."""
        if not PIXHAWK_OBSTACLE_DISTANCE_ENABLED or not self.master or not self.lidar_ready:
            return
        now = time.monotonic()
        if (now - float(self._last_obstacle_distance_send_ts or 0.0)) < max(0.05, OBSTACLE_DISTANCE_PERIOD_S):
            return
        self._last_obstacle_distance_send_ts = now
        try:
            from pymavlink import mavutil

            min_cm = 20
            max_cm = int(round(float(LIDAR_VALID_MAX_M) * 100.0))

            def range_cm(value):
                try:
                    meters = float(value)
                except (TypeError, ValueError):
                    return 65535
                if not math.isfinite(meters) or meters >= 98.5:
                    return 65535
                return int(clamp(round(meters * 100.0), min_cm, max_cm))

            distances = [65535] * 72
            sector_values = (
                (-LIDAR_OA_SECTOR_SIDE_MAX_DEG, -LIDAR_OA_SECTOR_CENTER_HALF_DEG, range_cm(left_m)),
                (-LIDAR_OA_SECTOR_CENTER_HALF_DEG, LIDAR_OA_SECTOR_CENTER_HALF_DEG, range_cm(center_m)),
                (LIDAR_OA_SECTOR_SIDE_MIN_DEG, LIDAR_OA_SECTOR_SIDE_MAX_DEG, range_cm(right_m)),
            )
            angle_offset = -90.0
            increment_f = 2.5
            for start_deg, end_deg, cm in sector_values:
                if cm == 65535:
                    continue
                start_i = max(0, int(math.floor((start_deg - angle_offset) / increment_f)))
                end_i = min(71, int(math.ceil((end_deg - angle_offset) / increment_f)))
                for idx in range(start_i, end_i + 1):
                    distances[idx] = min(int(distances[idx]), int(cm))

            sensor_type = int(getattr(mavutil.mavlink, "MAV_DISTANCE_SENSOR_LASER", 0))
            frame = int(getattr(mavutil.mavlink, "MAV_FRAME_BODY_FRD", 12))
            try:
                self.master.mav.obstacle_distance_send(
                    int(time.time() * 1_000_000),
                    sensor_type,
                    distances,
                    0,
                    min_cm,
                    max_cm,
                    float(increment_f),
                    float(angle_offset),
                    frame,
                )
            except TypeError:
                self.master.mav.obstacle_distance_send(
                    int(time.time() * 1_000_000),
                    sensor_type,
                    distances,
                    int(round(increment_f)),
                    min_cm,
                    max_cm,
                )
            if (now - float(self._last_obstacle_distance_log_ts or 0.0)) >= 2.0:
                self._last_obstacle_distance_log_ts = now
                log_jsonl(
                    "usv_main",
                    False,
                    event="pixhawk_obstacle_distance",
                    left_m=round(float(left_m), 3),
                    center_m=round(float(center_m), 3),
                    right_m=round(float(right_m), 3),
                    min_cm=min_cm,
                    max_cm=max_cm,
                )
        except Exception as exc:
            self._warn_throttled(
                "obstacle_distance_send",
                f"[WARN] [MAV] OBSTACLE_DISTANCE gonderilemedi: {exc}",
                period_s=2.0,
            )

    def _reset_nav_no_motion_monitor(self, dist_m=None):
        self._nav_no_motion_since = None
        self._nav_no_motion_last_dist_m = None if dist_m is None else float(dist_m)
        self._nav_no_motion_last_progress_m = float(self.progress_along_leg_m)

    def _check_nav_no_motion_hold(self, dist_m, command_speed_mps, obstacle_threat=False):
        """Detect commanded NAV motion with no position progress before it becomes a silent stall."""
        if (
            not self.mission_active
            or self.state != self.STATE_NAV
            or self.command_lock
            or bool(obstacle_threat)
            or float(command_speed_mps) < float(NAV_STUCK_MIN_COMMAND_MPS)
            or float(dist_m) <= max(float(R_WP_M) * 1.5, 1.0)
        ):
            self._reset_nav_no_motion_monitor(dist_m)
            return False

        now = time.monotonic()
        current_dist = float(dist_m)
        last_dist = self._nav_no_motion_last_dist_m
        last_progress = float(self._nav_no_motion_last_progress_m)
        progress_gain = float(self.progress_along_leg_m) - last_progress
        dist_gain = (float(last_dist) - current_dist) if last_dist is not None else 0.0
        moving = bool(
            dist_gain >= float(NAV_STUCK_PROGRESS_EPS_M)
            or progress_gain >= float(NAV_STUCK_PROGRESS_EPS_M)
            or abs(float(self.current_speed_mps or 0.0)) >= float(NAV_STUCK_SPEED_EPS_MPS)
        )
        if moving or last_dist is None:
            self._nav_no_motion_since = now
            self._nav_no_motion_last_dist_m = current_dist
            self._nav_no_motion_last_progress_m = float(self.progress_along_leg_m)
            return False

        stalled_s = now - float(self._nav_no_motion_since or now)
        if stalled_s < float(NAV_STUCK_TIMEOUT_S):
            if (now - float(self._nav_no_motion_last_log_ts or 0.0)) >= 2.0:
                self._nav_no_motion_last_log_ts = now
                log_jsonl(
                    "usv_main",
                    False,
                    event="nav_no_motion_warning",
                    stalled_s=round(float(stalled_s), 3),
                    dist_m=round(current_dist, 3),
                    command_speed_mps=round(float(command_speed_mps), 3),
                    current_speed_mps=round(float(self.current_speed_mps or 0.0), 3),
                    vehicle_mode=str(self.vehicle_mode_name),
                    actuator_mode=str(AUTONOMY_ACTUATOR_MODE),
                )
            return False

        print(
            f"[ERR] [NAV] Komut var ama ilerleme yok: stalled={stalled_s:.1f}s "
            f"dist={current_dist:.2f}m speed_cmd={float(command_speed_mps):.2f} "
            f"vehicle_mode={self.vehicle_mode_name}"
        )
        log_jsonl(
            "usv_main",
            False,
            event="nav_stuck_pwm_neutral",
            stalled_s=round(float(stalled_s), 3),
            dist_m=round(current_dist, 3),
            command_speed_mps=round(float(command_speed_mps), 3),
            current_speed_mps=round(float(self.current_speed_mps or 0.0), 3),
            vehicle_mode=str(self.vehicle_mode_name),
            guidance_mode=str(self.guidance_mode),
            motor_command_source=str(self._last_motor_command_source),
            sim_actuation_fallback=str(self.sim_actuation_fallback),
        )
        self.mission_end_reason = "nav_stuck_pwm_neutral"
        self._enter_hold("NAV_STUCK_PWM_NEUTRAL")
        return True

    def _write_motor_command(self, left_pwm, right_pwm):
        rc_diag = self._last_rc_override_diag if isinstance(self._last_rc_override_diag, dict) else {}
        payload = {
            "ts": round(time.time(), 3),
            "ts_monotonic": round(time.monotonic(), 3),
            "left_pwm": int(left_pwm),
            "right_pwm": int(right_pwm),
            "cmd_port_pwm": int(left_pwm),
            "cmd_stbd_pwm": int(right_pwm),
            "guided_speed_mps": round(float(self.v_target), 3),
            "guided_heading_target_deg": round(float(self.heading_target), 3),
            "actuator_control_mode": AUTONOMY_ACTUATOR_MODE,
            "sim_rc_override_semantics": str(rc_diag.get("semantics", "left_right_pwm")),
            "rc_override_ch1": int(rc_diag.get("ch1", PWM_NEUTRAL_US) or PWM_NEUTRAL_US),
            "rc_override_ch3": int(rc_diag.get("ch3", PWM_NEUTRAL_US) or PWM_NEUTRAL_US),
            "desired_left_pwm": int(rc_diag.get("desired_left_pwm", left_pwm) or left_pwm),
            "desired_right_pwm": int(rc_diag.get("desired_right_pwm", right_pwm) or right_pwm),
            "source": str(self._last_motor_command_source or "unknown"),
            "mission_active": bool(self.mission_active),
            "mode_state": {
                "canonical_mode": str(self.mode_state.get("canonical_mode", "SAFE_HOLD")),
                "source": str(self.mode_state.get("source", "unknown")),
                "reason": str(self.mode_state.get("reason", "unknown")),
                "armed": bool(self.mode_state.get("armed", False)),
            },
        }
        now = time.monotonic()
        should_write = (
            self._last_written_motor_command != payload
            or (now - float(self._last_motor_command_write_time or 0.0)) >= 0.1
        )
        if not should_write:
            return
        if atomic_write_json(MOTOR_COMMAND_FILE, payload):
            self._last_motor_command_write_time = now
            self._last_written_motor_command = payload
        else:
            self._warn_throttled(
                "motor_command_write",
                "[WARN] [MOTOR] motor_command atomic yazma basarisiz; onceki dosya korunuyor",
                period_s=2.0,
            )

    def _send_guided_speed_heading(self, speed_mps, heading_deg, reason="autonomy", display_pwm=None):
        """Primary autonomy actuator path: Rover GUIDED velocity + yaw setpoint."""
        speed_mps = max(0.0, float(speed_mps))
        heading_deg = float(heading_deg) % 360.0
        heading_err = normalize_heading_error(float(heading_deg) - float(self.current_heading))
        turn_phase = abs(float(heading_err)) > float(NAV_ALIGN_HEADING_DONE_DEG)
        body_forward_only = False
        if turn_phase and speed_mps > 0.0:
            creep_cap = max(0.0, float(NAV_ALIGN_CREEP_SPEED_MPS))
            if creep_cap > 0.0:
                speed_mps = min(float(speed_mps), creep_cap)
                body_forward_only = True
            else:
                speed_mps = 0.0
                body_forward_only = True
        self.v_target = speed_mps
        self.heading_target = heading_deg
        self._last_sent_rc_override = {
            "ch1": int(PWM_NEUTRAL_US),
            "ch3": int(PWM_NEUTRAL_US),
            "ts": time.monotonic(),
        }
        if isinstance(display_pwm, (list, tuple)) and len(display_pwm) >= 2:
            display_left = int(display_pwm[0])
            display_right = int(display_pwm[1])
        else:
            display_left = int(PWM_NEUTRAL_US)
            display_right = int(PWM_NEUTRAL_US)
        servo_output = self.last_servo_output_raw if isinstance(self.last_servo_output_raw, dict) else {}
        nav_controller = self.last_nav_controller_output if isinstance(self.last_nav_controller_output, dict) else {}
        self._last_rc_override_diag = {
            "semantics": "guided_velocity_yaw",
            "desired_left_pwm": int(display_left),
            "desired_right_pwm": int(display_right),
            "ch1": int(PWM_NEUTRAL_US),
            "ch3": int(PWM_NEUTRAL_US),
        }
        self._write_motor_command(display_left, display_right)
        if not self.master:
            return False

        now = time.monotonic()
        signature = (round(speed_mps, 3), round(heading_deg, 2), str(reason))
        if (
            self._last_guided_command == signature
            and (now - float(self._last_guided_command_time or 0.0)) < GUIDED_COMMAND_MIN_PERIOD_S
        ):
            return True

        try:
            from pymavlink import mavutil

            yaw_rad = math.radians(heading_deg)
            velocity_heading_deg = float(self.current_heading) if body_forward_only else heading_deg
            vx, vy = compass_heading_to_ne_velocity(speed_mps, velocity_heading_deg)
            self._last_guided_velocity_ne = (float(vx), float(vy))
            self.master.mav.set_position_target_global_int_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                GUIDED_GLOBAL_VEL_YAW_TYPE_MASK,
                int(round(self.current_lat * 1e7)),
                int(round(self.current_lon * 1e7)),
                0.0,
                float(vx),
                float(vy),
                0.0,
                0.0,
                0.0,
                0.0,
                float(yaw_rad),
                0.0,
            )
            self._last_guided_command_time = now
            self._last_guided_command = signature
            if (now - float(getattr(self, "_last_guided_diag_ts", 0.0) or 0.0)) >= 1.0:
                self._last_guided_diag_ts = now
                log_jsonl(
                    "usv_main",
                    False,
                    event="guided_setpoint",
                    speed_mps=round(float(speed_mps), 3),
                    heading_target_deg=round(float(heading_deg), 3),
                    current_heading_deg=round(float(self.current_heading), 3),
                    heading_error_deg=round(float(heading_err), 3),
                    turn_phase=bool(turn_phase),
                    body_forward_only=bool(body_forward_only),
                    vx_north_mps=round(float(vx), 4),
                    vy_east_mps=round(float(vy), 4),
                    yaw_rad=round(float(yaw_rad), 5),
                    reason=str(reason),
                    type_mask=int(GUIDED_GLOBAL_VEL_YAW_TYPE_MASK),
                    frame="MAV_FRAME_GLOBAL_INT",
                    vehicle_mode=str(self.vehicle_mode_name),
                    desired_left_pwm=int(display_left),
                    desired_right_pwm=int(display_right),
                    servo_output_raw=dict(servo_output),
                    nav_controller_output=dict(nav_controller),
                    wrong_turn_active=bool(self._wrong_turn_active),
                    wrong_turn_count=int(self._wrong_turn_count),
                )
            if now % 2.0 < 0.1:
                print(f"[GUIDED] speed={speed_mps:.2f}m/s heading={heading_deg:.1f} reason={reason}")
            return True
        except Exception as exc:
            self._warn_throttled("guided_command", f"[WARN] [GUIDED] setpoint gonderilemedi: {exc}", period_s=2.0)
            return False

    def _track_reverse_pwm_stats(self, left_pwm: int, right_pwm: int) -> None:
        if str(self._last_motor_command_source) != "autonomy":
            return
        reverse_thr = int(PWM_NEUTRAL_US) - int(max(4, PWM_DEADBAND_US))
        left_rev = int(left_pwm) < reverse_thr
        right_rev = int(right_pwm) < reverse_thr
        if left_rev and right_rev:
            self._both_reverse_count += 1
            return
        if left_rev or right_rev:
            self._single_side_reverse_count += 1
            if abs(float(self.nav_heading_error_deg)) <= 10.0:
                self._reverse_while_heading_small_count += 1

    def _sim_twin_pwm_to_rover_rc(self, left_pwm: int, right_pwm: int) -> tuple:
        neutral = float(PWM_NEUTRAL_US)
        span = max(1.0, float(PWM_MAX_US - PWM_NEUTRAL_US))
        left_norm = clamp((float(left_pwm) - neutral) / span, -1.0, 1.0)
        right_norm = clamp((float(right_pwm) - neutral) / span, -1.0, 1.0)
        throttle_norm = clamp((left_norm + right_norm) * 0.5, -1.0, 1.0)
        steer_norm = clamp((right_norm - left_norm) * 0.5, -1.0, 1.0)
        steer_pwm = int(round(neutral + (steer_norm * span)))
        throttle_pwm = int(round(neutral + (throttle_norm * span)))
        return (
            int(clamp(steer_pwm, PWM_MIN_US, PWM_MAX_US)),
            int(clamp(throttle_pwm, PWM_MIN_US, PWM_MAX_US)),
        )

    def _set_rc_override(self, left_pwm, right_pwm):
        """Fallback/manual path only. Autonomy normally uses GUIDED velocity+yaw."""
        left_pwm = int(clamp(left_pwm, 1100, 1900))
        right_pwm = int(clamp(right_pwm, 1100, 1900))
        self._track_reverse_pwm_stats(left_pwm, right_pwm)
        send_ch1 = int(left_pwm)
        send_ch3 = int(right_pwm)
        semantics = "left_right_pwm"
        if os.environ.get("USV_SIM") == "1" and getattr(self, "_sim_mavlink_actuation_rc", False):
            send_ch1, send_ch3 = self._sim_twin_pwm_to_rover_rc(left_pwm, right_pwm)
            semantics = "steer_throttle"
        self._last_rc_override_diag = {
            "semantics": semantics,
            "desired_left_pwm": int(left_pwm),
            "desired_right_pwm": int(right_pwm),
            "ch1": int(send_ch1),
            "ch3": int(send_ch3),
        }
        if not self.master:
            self._write_motor_command(left_pwm, right_pwm)
            if time.monotonic() % 5.0 < 0.1:
                print(f"[WARN] [MOTOR] master is None, cannot send motor command")
            return
        try:
            # RC CHANNELS OVERRIDE (stable path for motorboat)
            # Real/bench: CH1/CH3 can mirror left/right PWM.
            # Sim fallback: CH1=steer, CH3=throttle so ArduRover owns the motor mix.
            # Channels 0-7 (8 total), unused channels = 0
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                int(send_ch1),    # CH1
                0,                # CH2 (unused)
                int(send_ch3),    # CH3
                0,                # CH4–CH8 (unused)
                0,
                0,
                0,
                0
            )
            self._last_sent_rc_override = {
                "ch1": int(send_ch1),
                "ch3": int(send_ch3),
                "ts": time.monotonic(),
            }

            now = time.monotonic()
            if now % 2.0 < 0.1:  # Log every 2 sec to avoid spam
                print(
                    f"[DEBUG] [MOTOR] RC_OVERRIDE semantics={semantics} "
                    f"CH1={send_ch1} CH3={send_ch3} desired_left={left_pwm} desired_right={right_pwm}"
                )
            last_left = int(getattr(self, "_last_motor_log_left_pwm", left_pwm))
            last_right = int(getattr(self, "_last_motor_log_right_pwm", right_pwm))
            last_ts = float(getattr(self, "_last_motor_log_ts", 0.0) or 0.0)
            if abs(left_pwm - last_left) >= 8 or abs(right_pwm - last_right) >= 8 or (now - last_ts) >= 1.0:
                print(
                    f"[MOTOR] RC override: desired_left={left_pwm} desired_right={right_pwm} "
                    f"semantics={semantics} ch1={send_ch1} ch3={send_ch3}"
                )
                self._last_motor_log_left_pwm = left_pwm
                self._last_motor_log_right_pwm = right_pwm
                self._last_motor_log_ts = now
        except AttributeError as exc:
            print(f"[ERROR] [MOTOR] master attribute missing: {exc}")
        except Exception as exc:
            print(f"[WARN] [MOTOR] Motor command error: {exc}")
        self._write_motor_command(left_pwm, right_pwm)

    def _command_speed_heading(self, speed_mps, heading_error_deg):
        """
        Command motor speed and heading. 
        
        COMMAND LOCK: If command_lock=True (mission safety), reject autonomy commands.
        RC OVERRIDE: If RC sticks manually deflected (ch1/ch3 away from neutral), use RC values directly.
        
        Per AGENTS.md Section 2.1: RC override ALWAYS preempts autonomy.
        E-stop (RC7) always works and bypasses all checks.
        """
        self._update_mode_state(source="control")
        # CHECK 1: Command lock (mission safety gate)
        if self.command_lock:
            self._last_motor_command_source = "command_lock"
            self._reset_mixer_transient(keep_trim=True)
            if self._autonomy_pwm_via_rc_override():
                self._set_rc_override(int(PWM_NEUTRAL_US), int(PWM_NEUTRAL_US))
            else:
                self._send_guided_speed_heading(0.0, self.current_heading, reason="command_lock")
            if not hasattr(self, '_last_motor_lock_log'):
                self._last_motor_lock_log = 0.0
            now = time.monotonic()
            if (now - self._last_motor_lock_log) >= 5.0:  # Log every 5 sec
                print(f"[MOTOR_LOCK] LOCKED - commanded(speed={speed_mps:.2f}m/s, hdg_err={heading_error_deg:.1f}°) → neutral. Reason: {self.hold_reason}")
                self._last_motor_lock_log = now
            return
        
        # CHECK 2: RC manual override (absolute preemption per AGENTS.md 2.1)
        if self._is_rc_stick_active():
            # RC sticks are being manually moved → map steer/throttle to twin-thruster PWM
            rc_ch1 = int(self.rc_channels.get("ch1", 1500) or 1500)  # Steering
            rc_ch3 = int(self.rc_channels.get("ch3", 1500) or 1500)  # Throttle
            left_pwm, right_pwm = self._mix_rc_steer_throttle_to_twin_pwm(rc_ch1, rc_ch3)
            self._manual_override_active = True
            self._rc_neutral_since = None
            self._last_motor_command_source = "manual_rc"
            self._set_rc_override(left_pwm, right_pwm)
            
            # Log RC override (throttled to avoid spam)
            if not hasattr(self, '_last_rc_override_log'):
                self._last_rc_override_log = 0.0
            now = time.monotonic()
            if (now - self._last_rc_override_log) >= 2.0:
                print(
                    f"[RC_OVERRIDE] steer={rc_ch1} throttle={rc_ch3} -> left={left_pwm} right={right_pwm} "
                    f"(ignoring autonomy speed={speed_mps:.2f}m/s)"
                )
                self._last_rc_override_log = now
            return

        # RC override birakildiktan sonra AUTO'ya donus neutral dwell ile yapilir.
        now = time.monotonic()
        if self._manual_override_active:
            if self._rc_neutral_since is None:
                self._rc_neutral_since = now
            neutral_dwell_s = max(0.0, now - float(self._rc_neutral_since))
            if neutral_dwell_s < float(MODE_NEUTRAL_DWELL_S):
                self._last_motor_command_source = "rc_neutral_dwell"
                self._reset_mixer_transient(keep_trim=True)
                if self._autonomy_pwm_via_rc_override():
                    self._set_rc_override(int(PWM_NEUTRAL_US), int(PWM_NEUTRAL_US))
                else:
                    self._send_guided_speed_heading(0.0, self.current_heading, reason="rc_neutral_dwell")
                self._update_mode_state(source="control", reason="rc_neutral_dwell")
                return
            self._manual_override_active = False
            self._rc_neutral_since = now
        
        # CHECK 3: Sensor fusion policy (AGENTS.md derating on low trust)
        trust_bar = self._compute_trust_bar()
        if trust_bar < TRUST_WARN_THRESHOLD:
            # Autonomy health degraded → cap speed to FAILSAFE_SLOW_MPS (0.3 m/s)
            speed_mps = min(float(speed_mps), FAILSAFE_SLOW_MPS)
            if not hasattr(self, '_last_failsafe_log'):
                self._last_failsafe_log = 0.0
            if (now - self._last_failsafe_log) >= 3.0:
                print(f"[FAILSAFE] Autonomy health trust={trust_bar:.1f}% < {TRUST_WARN_THRESHOLD}% → speed capped to {FAILSAFE_SLOW_MPS} m/s")
                self._last_failsafe_log = now
        
        # NORMAL AUTONOMY: No command lock, no RC override, trust OK → execute commanded track
        # Final commanded values (after all checks/caps applied)
        shaped_heading_err = self._apply_heading_deadzone_hysteresis(clamp_heading_error(heading_error_deg))
        shaped_heading_err, shaped_speed_mps, wrong_turn_guard = self._apply_wrong_turn_guard(
            shaped_heading_err,
            speed_mps,
        )
        self.v_target = float(shaped_speed_mps)
        self.heading_target = (self.current_heading + shaped_heading_err) % 360

        allocation = allocate_twin_thrusters(
            speed_mps=float(shaped_speed_mps),
            heading_error_deg=float(shaped_heading_err),
            max_speed_mps=max(P2_CRUISE_MPS, P3_MAX_SPEED_MPS, P1_SPEED_CRUISE_MPS),
        )
        self.motor_limit_reason = str(allocation.limit_reason)
        if wrong_turn_guard:
            self.motor_limit_reason = "wrong_turn_guard"
        left_pwm_cmd, right_pwm_cmd = self._harden_motor_allocation(
            allocation=allocation,
            heading_error_deg=float(shaped_heading_err),
            speed_mps=float(shaped_speed_mps),
        )
        now = time.monotonic()
        need_boost = bool(
            shaped_speed_mps > 0.45
            and float(self.current_speed_mps or 0.0) < 0.15
            and self._blocked_level < 0.80
        )
        if need_boost and now >= float(self._thrust_boost_until):
            self._thrust_boost_until = now + 0.6
        if now < float(self._thrust_boost_until):
            left_delta = int(round((left_pwm_cmd - int(PWM_NEUTRAL_US)) * 0.08))
            right_delta = int(round((right_pwm_cmd - int(PWM_NEUTRAL_US)) * 0.08))
            left_pwm_cmd = int(clamp(left_pwm_cmd + left_delta, int(PWM_MIN_US), int(PWM_MAX_US)))
            right_pwm_cmd = int(clamp(right_pwm_cmd + right_delta, int(PWM_MIN_US), int(PWM_MAX_US)))
            if self.motor_limit_reason == "nominal":
                self.motor_limit_reason = "short_thrust_boost"
        if os.environ.get("USV_SIM") == "1" and time.monotonic() % 1.0 < 0.1:
            print(
                f"[DEBUG] [MOTOR] SIM mode: left={left_pwm_cmd} right={right_pwm_cmd} "
                f"(speed={shaped_speed_mps:.2f}, hdg_err={shaped_heading_err:.1f})"
            )
        self._last_motor_command_source = "autonomy"
        if self._autonomy_pwm_via_rc_override():
            self._set_rc_override(left_pwm_cmd, right_pwm_cmd)
        else:
            sent = self._send_guided_speed_heading(
                shaped_speed_mps,
                self.heading_target,
                reason=self.guidance_mode or "autonomy",
                display_pwm=(left_pwm_cmd, right_pwm_cmd),
            )
            if not sent:
                self._warn_throttled(
                    "guided_command_failed_hold",
                    "[WARN] [GUIDED] Birincil komut basarisiz; RC override fallback kapali, HOLD uygulanıyor",
                    period_s=2.0,
                )
                self._enter_hold("GUIDED_SETPOINT_SEND_FAILED")
                return
        self._update_mode_state(source="control")

    def stop_motors(self):
        # Stop motors - always works (emergency handler)
        self.v_target = 0.0
        self.motor_limit_reason = "stopped"
        self._last_motor_command_source = "stop"
        self._reset_mixer_transient(keep_trim=True)
        use_rc = self._autonomy_pwm_via_rc_override()
        if use_rc:
            self._set_rc_override(int(PWM_NEUTRAL_US), int(PWM_NEUTRAL_US))
        else:
            self._send_guided_speed_heading(0.0, self.current_heading, reason="stop")
        self._sim_mavlink_actuation_rc = False
        self.sim_actuation_fallback = "none"
        self.sim_actuation_fallback_reason = "none"
        self._disarm()
        self._update_mode_state(source="control", reason="stop_motors")

    def _hold_waypoint_neutral(self, reason="waypoint_hold"):
        """Send neutral setpoint between waypoints without disarming an active mission."""
        hold_reason = str(reason or "waypoint_hold")
        self.v_target = 0.0
        self.motor_limit_reason = hold_reason
        self._last_motor_command_source = hold_reason
        self._reset_mixer_transient(keep_trim=True)
        if self._autonomy_pwm_via_rc_override():
            self._set_rc_override(int(PWM_NEUTRAL_US), int(PWM_NEUTRAL_US))
        else:
            self._send_guided_speed_heading(0.0, self.current_heading, reason=hold_reason)
        self._update_mode_state(source="control", reason=hold_reason)

    def _force_safe_outputs(self, include_estop=False, repeat=6):
        if not self.master:
            return
        self._reset_mixer_transient(keep_trim=False)
        for _ in range(max(1, int(repeat))):
            self._set_rc_override(1500, 1500)
            time.sleep(0.04)
        if include_estop:
            self._set_rc7_estop(True)
        self._disarm()

    def _consume_flag(self, path):
        if not os.path.exists(path):
            return False
        try:
            os.remove(path)
            return True
        except Exception as exc:
            print(f"[WARN] [FLAG] Silme hatasi ({path}): {exc}")
            return False

    def _drain_mav_messages(self):
        # In simulation mode, still need to process SITL messages for state updates
        if not self.master:
            return
        self._send_companion_heartbeat_if_due()
        
        if not hasattr(self, '_mav_message_counts'):
            self._mav_message_counts = {}
        if not hasattr(self, '_last_mav_debug_log'):
            self._last_mav_debug_log = 0.0
        
        try:
            message_count = 0
            # Sim: fresh vehicle_position.json is authoritative; do not mix MAVLink hdg (often 0/invalid) into EMA.
            sim_heading_authority = (
                os.environ.get("USV_SIM") == "1"
                and bool(load_sim_nav_state(control_dir=CONTROL_DIR).get("valid"))
            )
            trust_zero_hdg = os.environ.get("USV_SIM") != "1"
            while True:
                msg = self.master.recv_match(blocking=False)
                if not msg:
                    break
                message_count += 1
                mtype = msg.get_type()
                log_jsonl("usv_main", False, event="drain_rx", mtype=mtype)
                try:
                    self._last_mavlink_message_ts[str(mtype)] = time.monotonic()
                except Exception:
                    pass
                self._mav_message_counts[mtype] = self._mav_message_counts.get(mtype, 0) + 1
                if mtype == "COMMAND_ACK":
                    try:
                        command = int(getattr(msg, "command", -1) or -1)
                        result = int(getattr(msg, "result", -1) or -1)
                        progress = getattr(msg, "progress", None)
                        result_param2 = getattr(msg, "result_param2", None)
                        target_system = getattr(msg, "target_system", None)
                        target_component = getattr(msg, "target_component", None)
                        result_name = MAV_RESULT_NAMES.get(result, f"UNKNOWN({result})")
                        self.last_command_ack = {
                            "command": command,
                            "result": result,
                            "result_name": result_name,
                            "progress": progress,
                            "result_param2": result_param2,
                            "target_system": target_system,
                            "target_component": target_component,
                            "ts_monotonic": round(time.monotonic(), 3),
                        }
                        print(f"[MAV_ACK] command={command} result={result_name} progress={progress}")
                        log_jsonl(
                            "usv_main",
                            False,
                            event="command_ack",
                            command=command,
                            result=result,
                            result_name=result_name,
                            progress=progress,
                            result_param2=result_param2,
                        )
                    except Exception as ack_exc:
                        self._warn_throttled("command_ack_parse", f"[WARN] [MAV_ACK] parse hatasi: {ack_exc}")
                elif mtype == "STATUSTEXT":
                    try:
                        text = getattr(msg, "text", "")
                        if isinstance(text, bytes):
                            text = text.decode("utf-8", errors="replace")
                        text = str(text).rstrip("\x00").strip()
                        severity = int(getattr(msg, "severity", -1) or -1)
                        self.last_statustext = {
                            "severity": severity,
                            "text": text,
                            "ts_monotonic": round(time.monotonic(), 3),
                        }
                        if text:
                            low_text = str(text).lower()
                            if "ekf failsafe cleared" in low_text:
                                self._last_ekf_variance_statustext_ts = 0.0
                                if isinstance(getattr(self, "_fc_start_blocker_ts", None), dict):
                                    for _block_key in (
                                        "ekf_variance_recent",
                                        "bad_position_recent",
                                        "accel_inconsistent_recent",
                                        "gyro_inconsistent_recent",
                                    ):
                                        self._fc_start_blocker_ts.pop(_block_key, None)
                            if "origin set" in low_text and os.environ.get("USV_SIM") == "1":
                                self._sim_ekf_origin_set_mono = time.monotonic()
                            blocker = self._classify_fc_start_blocker(text)
                            if blocker:
                                self._mark_fc_start_blocker(blocker)
                            print(f"[MAV_TEXT] sev={severity} {text}")
                            log_jsonl(
                                "usv_main",
                                False,
                                event="statustext",
                                severity=severity,
                                text=text,
                            )
                    except Exception as text_exc:
                        self._warn_throttled("statustext_parse", f"[WARN] [MAV_TEXT] parse hatasi: {text_exc}")
                elif mtype == "GLOBAL_POSITION_INT":
                    lat = float(getattr(msg, "lat", 0) or 0) / 1e7
                    lon = float(getattr(msg, "lon", 0) or 0) / 1e7
                    if self._geo_position_valid_values(lat, lon):
                        self.current_lat = lat
                        self.current_lon = lon
                        self.gps_global_position_int_received = True  # Mark GPS valid for simulation mode (JSON backend)
                        self._last_global_position_int_ts = time.monotonic()
                        if os.environ.get("USV_SIM") == "1" and self.gps_fix_type < 3:
                            self.gps_fix_type = 3  # 3D fix equivalent (from GLOBAL_POSITION_INT)
                            self.gps_satellites_visible = 12  # Simulate 12 satellites visible
                    else:
                        self._warn_throttled(
                            "invalid_global_position_int",
                            (
                                "[WARN] [MAV] GLOBAL_POSITION_INT gecersiz, "
                                f"current_lat/lon korunuyor lat={lat:.7f} lon={lon:.7f} "
                                f"source={self.guided_position_source}"
                            ),
                            period_s=2.0,
                        )
                    # Heading with exponential moving average filter (reduces compass drift oscillation)
                    if not sim_heading_authority:
                        raw_cdeg = float(getattr(msg, "hdg", 65535) or 65535)
                        if mavlink_heading_cdeg_valid(raw_cdeg, trust_zero=trust_zero_hdg):
                            raw_heading = raw_cdeg / 100.0
                            if self._heading_ema is None:
                                self._heading_ema = raw_heading
                            else:
                                delta_deg = normalize_heading_error(raw_heading - self._heading_ema)
                                self._heading_ema = (self._heading_ema + self._heading_ema_alpha * delta_deg) % 360.0
                            self.current_heading = self._heading_ema % 360.0
                elif mtype == "GPS_RAW_INT":
                    self._last_gps_raw_int_ts = time.monotonic()
                    self.gps_satellites_visible = int(getattr(msg, "satellites_visible", 0) or 0)
                    raw_fix = int(getattr(msg, "fix_type", 0) or 0)
                    if (
                        os.environ.get("USV_SIM") == "1"
                        and self.gps_global_position_int_received
                        and raw_fix < 3
                    ):
                        pass
                    else:
                        self.gps_fix_type = raw_fix
                elif mtype == "EKF_STATUS_REPORT":
                    try:
                        self.fc_ekf_flags = int(getattr(msg, "flags", 0) or 0)
                        self.fc_ekf_variances = {
                            "velocity": round(float(getattr(msg, "velocity_variance", 0.0) or 0.0), 4),
                            "pos_horiz": round(float(getattr(msg, "pos_horiz_variance", 0.0) or 0.0), 4),
                            "pos_vert": round(float(getattr(msg, "pos_vert_variance", 0.0) or 0.0), 4),
                            "compass": round(float(getattr(msg, "compass_variance", 0.0) or 0.0), 4),
                            "terrain_alt": round(float(getattr(msg, "terrain_alt_variance", 0.0) or 0.0), 4),
                            "airspeed": round(float(getattr(msg, "airspeed_variance", 0.0) or 0.0), 4),
                        }
                        self.fc_ekf_report_seen = True
                        self.fc_ekf_report_ts = time.monotonic()
                        self._update_fc_nav_mode_ready()
                    except Exception as ekf_exc:
                        self._warn_throttled("ekf_status_parse", f"[WARN] [MAV] EKF_STATUS_REPORT parse hatasi: {ekf_exc}")
                elif mtype == "ATTITUDE":
                    self.current_roll_deg = math.degrees(getattr(msg, "roll", 0.0))
                    self.current_pitch_deg = math.degrees(getattr(msg, "pitch", 0.0))
                    try:
                        self.current_yaw_rate_dps = math.degrees(float(getattr(msg, "yawspeed", 0.0) or 0.0))
                    except (TypeError, ValueError):
                        self.current_yaw_rate_dps = 0.0
                    # Yaw from ATTITUDE - DISABLED in SIM mode (use Gazebo heading instead)
                    # In SIM: vehicle_position.json heading_rad is updated every loop with Gazebo data
                    # In HW: uncomment below to use MAVLink ATTITUDE yaw
                    if not self.simulation_mode:  # Hardware only
                        yaw_rad = getattr(msg, "yaw", 0.0)
                        yaw_deg = math.degrees(yaw_rad)
                        # Convert to 0-360 range for consistency with GPS bearing
                        yaw_deg = (yaw_deg + 360) % 360
                        if self._heading_ema is None:
                            self._heading_ema = yaw_deg
                        else:
                            delta_deg = normalize_heading_error(yaw_deg - self._heading_ema)
                            self._heading_ema = (self._heading_ema + self._heading_ema_alpha * delta_deg) % 360.0
                        self.current_heading = self._heading_ema % 360.0
                elif mtype == "HEARTBEAT":
                    try:
                        from pymavlink import mavutil

                        heartbeat_type = int(getattr(msg, "type", -1) or -1)
                        is_gcs = heartbeat_type == int(mavutil.mavlink.MAV_TYPE_GCS)
                    except Exception:
                        is_gcs = False
                    src_system = int(msg.get_srcSystem() or 0)
                    src_component = int(msg.get_srcComponent() or 0)
                    try:
                        from pymavlink import mavutil as _m_util

                        mp_cid = int(getattr(_m_util.mavlink, "MAV_COMP_ID_MISSIONPLANNER", 190))
                        if is_gcs or src_component == mp_cid:
                            self.last_gcs_heartbeat_mono = time.monotonic()
                    except Exception:
                        if is_gcs:
                            self.last_gcs_heartbeat_mono = time.monotonic()
                    target_system = int(getattr(self.master, "target_system", 0) or 0)
                    accept_heartbeat = bool(
                        src_system > 0
                        and not is_gcs
                        and (target_system <= 0 or src_system == target_system)
                    )
                    if accept_heartbeat:
                        if target_system <= 0:
                            self.master.target_system = src_system
                        if int(getattr(self.master, "target_component", 0) or 0) <= 0 and src_component > 0:
                            self.master.target_component = src_component
                        self.last_heartbeat_time = time.monotonic()
                        try:
                            self.vehicle_armed = bool(
                                int(getattr(msg, "base_mode", 0) or 0)
                                & int(mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                            )
                        except Exception:
                            self.vehicle_armed = False
                        self.vehicle_mode_custom = int(getattr(msg, "custom_mode", 0) or 0)
                        self.vehicle_mode_name = ARDUROVER_MODE_NAMES.get(
                            self.vehicle_mode_custom,
                            f"UNKNOWN({self.vehicle_mode_custom})",
                        )
                elif mtype == "SERVO_OUTPUT_RAW":
                    try:
                        self.last_servo_output_raw = {
                            "servo1_raw": int(getattr(msg, "servo1_raw", int(PWM_NEUTRAL_US)) or int(PWM_NEUTRAL_US)),
                            "servo2_raw": int(getattr(msg, "servo2_raw", int(PWM_NEUTRAL_US)) or int(PWM_NEUTRAL_US)),
                            "servo3_raw": int(getattr(msg, "servo3_raw", int(PWM_NEUTRAL_US)) or int(PWM_NEUTRAL_US)),
                            "servo4_raw": int(getattr(msg, "servo4_raw", int(PWM_NEUTRAL_US)) or int(PWM_NEUTRAL_US)),
                            "ts_monotonic": round(time.monotonic(), 3),
                        }
                    except Exception as servo_exc:
                        self._warn_throttled("servo_output_parse", f"[WARN] [MAV] SERVO_OUTPUT_RAW parse hatasi: {servo_exc}")
                elif mtype == "NAV_CONTROLLER_OUTPUT":
                    try:
                        self.last_nav_controller_output = {
                            "nav_roll": round(float(getattr(msg, "nav_roll", 0.0) or 0.0), 3),
                            "nav_bearing": round(float(getattr(msg, "nav_bearing", 0.0) or 0.0), 3),
                            "target_bearing": round(float(getattr(msg, "target_bearing", 0.0) or 0.0), 3),
                            "wp_dist": round(float(getattr(msg, "wp_dist", 0.0) or 0.0), 3),
                            "xtrack_error": round(float(getattr(msg, "xtrack_error", 0.0) or 0.0), 3),
                            "ts_monotonic": round(time.monotonic(), 3),
                        }
                    except Exception as nav_ctl_exc:
                        self._warn_throttled("nav_controller_parse", f"[WARN] [MAV] NAV_CONTROLLER_OUTPUT parse hatasi: {nav_ctl_exc}")
                elif mtype == "MISSION_CURRENT":
                    try:
                        self.mission_current_seq = int(getattr(msg, "seq", -1) or -1)
                        self.mission_current_ts = time.monotonic()
                    except Exception as mission_current_exc:
                        self._warn_throttled("mission_current_parse", f"[WARN] [MAV] MISSION_CURRENT parse hatasi: {mission_current_exc}")
                elif mtype in ("MISSION_ITEM_REACHED", "MISSION_REACHED"):
                    try:
                        self.mission_reached_seq = int(getattr(msg, "seq", -1) or -1)
                        self.mission_reached_ts = time.monotonic()
                    except Exception as mission_reached_exc:
                        self._warn_throttled("mission_reached_parse", f"[WARN] [MAV] MISSION_REACHED parse hatasi: {mission_reached_exc}")
                elif mtype == "VFR_HUD":
                    try:
                        self.current_speed_mps = float(getattr(msg, "groundspeed", 0.0) or 0.0)
                    except (TypeError, ValueError):
                        self.current_speed_mps = 0.0
                elif mtype == "RC_CHANNELS":
                    self.rc_channels["ch1"] = getattr(msg, "chan1_raw", 0)
                    self.rc_channels["ch2"] = getattr(msg, "chan2_raw", 0)
                    self.rc_channels["ch3"] = getattr(msg, "chan3_raw", 0)
                    self.rc_channels["ch4"] = getattr(msg, "chan4_raw", 0)
                    self.rc_channels["ch5"] = getattr(msg, "chan5_raw", 0)
                    self.rc_channels["ch7"] = getattr(msg, "chan7_raw", 0)
                    self.rc_channels["ch8"] = getattr(msg, "chan8_raw", 0)
                    if self.rc_channels["ch7"] >= RC7_ESTOP_PWM:
                        self._trigger_estop("RC7", force_rc7=False)
                elif mtype == "SYS_STATUS":
                    # Battery voltage from SYS_STATUS (in mV)
                    self.battery_voltage = float(getattr(msg, "voltage_battery", 0) or 0) / 1000.0  # Convert mV to V
            
            # Debug logging every 5 seconds
            now = time.monotonic()
            if (now - self._last_mav_debug_log) >= 5.0:
                if message_count > 0:
                    print(f"[MAV_DEBUG] Drained {message_count} messages. Types: {dict(list(self._mav_message_counts.items())[:5])}")
                    print(f"[MAV_STATE] Heartbeat_age={self.heartbeat_age_s:.1f}s GPS_sats={self.gps_satellites_visible} Batt={self.battery_voltage:.2f}V")
                self._last_mav_debug_log = now
            self._update_mode_state(source="mav")
        except Exception as exc:
            self._bump_error("mav_read_error", f"[WARN] [MAV] Mesaj drain hatasi: {exc}")
            print(f"[MAV_ERROR] Exception: {exc}")
            _usv_dbg.exception("mav drain: %s", exc)

    def _read_camera_status(self):
        default = {
            "ts_monotonic": 0.0,
            "frame_age_s": 999.0,
            "objective_phase": self._get_objective_phase(),
            "perception_policy": self._get_perception_policy(),
            "gate_actionable": False,
            "yellow_actionable": False,
            "target_actionable": False,
            "gate_detected_raw": False,
            "gate_stable_s_raw": 0.0,
            "gate_center_bearing_deg_raw": 0.0,
            "gate_passed_event_raw": False,
            "gate_detected": False,
            "gate_stable_s": 0.0,
            "gate_center_bearing_deg": 0.0,
            "gate_passed_event": False,
            "yellow_obstacle_detected_raw": False,
            "yellow_obstacle_bearing_deg_raw": 0.0,
            "yellow_obstacle_area_norm_raw": 0.0,
            "yellow_obstacle_detected": False,
            "yellow_obstacle_bearing_deg": 0.0,
            "yellow_obstacle_area_norm": 0.0,
            "orange_actionable": False,
            "orange_boundary_detected_raw": False,
            "orange_boundary_bearing_deg_raw": 0.0,
            "orange_boundary_area_norm_raw": 0.0,
            "orange_boundary_detected": False,
            "orange_boundary_bearing_deg": 0.0,
            "orange_boundary_area_norm": 0.0,
            "target_detected_raw": False,
            "target_bearing_error_deg_raw": 0.0,
            "target_area_norm_raw": 0.0,
            "target_detected": False,
            "target_bearing_error_deg": 0.0,
            "target_area_norm": 0.0,
            "camera_adaptation": dict(self.camera_adaptation),
        }
        data = default
        now_mono = time.monotonic()
        now_wall = time.time()
        status_mtime_age_s = 999.0
        try:
            if os.path.exists(CAMERA_STATUS_FILE):
                try:
                    status_mtime_age_s = max(0.0, now_wall - os.path.getmtime(CAMERA_STATUS_FILE))
                except Exception:
                    status_mtime_age_s = 999.0
                with open(CAMERA_STATUS_FILE, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
                data = {**default, **loaded}
        except Exception as exc:
            self._bump_error("camera_state_read_error", f"[WARN] [CAM] Status okuma hatasi: {exc}")
            data = default
            status_mtime_age_s = 999.0

        try:
            status_ts = float(data.get("ts_monotonic", 0.0) or 0.0)
        except (TypeError, ValueError):
            status_ts = 0.0
        status_age_s = 999.0 if status_ts <= 0.0 else max(0.0, now_mono - status_ts)
        status_stale = bool(
            status_age_s > float(CAMERA_STATUS_TIMEOUT_S)
            or status_mtime_age_s > float(CAMERA_STATUS_MTIME_TIMEOUT_S)
        )
        self.camera_status_age_s = float(status_age_s)
        self.camera_status_mtime_age_s = float(status_mtime_age_s)
        self.camera_status_stale = bool(status_stale)
        data["status_age_s"] = round(float(status_age_s), 3)
        data["status_mtime_age_s"] = round(float(status_mtime_age_s), 3)
        data["status_stale"] = bool(status_stale)

        for key in (
            "gate_detected",
            "gate_stable_s",
            "gate_center_bearing_deg",
            "gate_passed_event",
            "yellow_obstacle_detected",
            "yellow_obstacle_bearing_deg",
            "yellow_obstacle_area_norm",
            "orange_boundary_detected",
            "orange_boundary_bearing_deg",
            "orange_boundary_area_norm",
            "target_detected",
            "target_bearing_error_deg",
            "target_area_norm",
        ):
            data[f"{key}_raw"] = data.get(f"{key}_raw", data.get(key, default.get(key)))

        default_policy = default["perception_policy"]
        perception_policy = data.get("perception_policy", default_policy)
        if not isinstance(perception_policy, dict):
            perception_policy = dict(default_policy)
        perception_policy = {
            "gate": bool(perception_policy.get("gate", default_policy["gate"])),
            "yellow_obstacle": bool(perception_policy.get("yellow_obstacle", default_policy["yellow_obstacle"])),
            "orange_boundary": bool(
                perception_policy.get("orange_boundary", default_policy.get("orange_boundary", False))
            ),
            "target": bool(perception_policy.get("target", default_policy["target"])),
        }
        data["objective_phase"] = str(data.get("objective_phase", default["objective_phase"]) or default["objective_phase"])
        data["perception_policy"] = perception_policy
        gate_actionable = bool(data.get("gate_actionable", perception_policy["gate"]))
        yellow_actionable = bool(data.get("yellow_actionable", perception_policy["yellow_obstacle"]))
        orange_actionable = bool(data.get("orange_actionable", perception_policy.get("orange_boundary", False)))
        target_actionable = bool(data.get("target_actionable", perception_policy["target"]))
        data["gate_actionable"] = gate_actionable
        data["yellow_actionable"] = yellow_actionable
        data["orange_actionable"] = orange_actionable
        data["target_actionable"] = target_actionable
        if not gate_actionable:
            data["gate_detected"] = False
            data["gate_stable_s"] = 0.0
            data["gate_center_bearing_deg"] = 0.0
            data["gate_passed_event"] = False
        if not yellow_actionable:
            data["yellow_obstacle_detected"] = False
            data["yellow_obstacle_bearing_deg"] = 0.0
            data["yellow_obstacle_area_norm"] = 0.0
        if not orange_actionable:
            data["orange_boundary_detected"] = False
            data["orange_boundary_bearing_deg"] = 0.0
            data["orange_boundary_area_norm"] = 0.0
        if not target_actionable:
            data["target_detected"] = False
            data["target_bearing_error_deg"] = 0.0
            data["target_area_norm"] = 0.0

        if status_stale:
            for key in (
                "gate_detected_raw",
                "gate_passed_event_raw",
                "gate_detected",
                "gate_passed_event",
                "yellow_obstacle_detected_raw",
                "yellow_obstacle_detected",
                "orange_boundary_detected_raw",
                "orange_boundary_detected",
                "target_detected_raw",
                "target_detected",
                "wrong_target_detected_raw",
                "wrong_target_detected",
            ):
                data[key] = False
            for key in (
                "gate_stable_s_raw",
                "gate_stable_s",
                "gate_center_bearing_deg_raw",
                "gate_center_bearing_deg",
                "yellow_obstacle_bearing_deg_raw",
                "yellow_obstacle_bearing_deg",
                "yellow_obstacle_area_norm_raw",
                "yellow_obstacle_area_norm",
                "orange_boundary_bearing_deg_raw",
                "orange_boundary_bearing_deg",
                "orange_boundary_area_norm_raw",
                "orange_boundary_area_norm",
                "target_bearing_error_deg_raw",
                "target_bearing_error_deg",
                "target_area_norm_raw",
                "target_area_norm",
                "wrong_target_bearing_deg_raw",
                "wrong_target_bearing_deg",
                "wrong_target_area_norm_raw",
                "wrong_target_area_norm",
            ):
                data[key] = 0.0
            data["wrong_target_class_raw"] = ""
            data["wrong_target_class"] = ""

        parkur_label = self._active_parkur_label()
        gate_detected_raw = bool(data.get("gate_detected_raw", False))
        target_detected_raw = bool(data.get("target_detected_raw", False))
        gate_signal_active = gate_detected_raw and gate_actionable
        target_signal_active = target_detected_raw and target_actionable
        try:
            gate_bearing_raw = float(data.get("gate_center_bearing_deg_raw", 0.0))
        except (TypeError, ValueError):
            gate_bearing_raw = 0.0
        try:
            target_bearing_raw = float(data.get("target_bearing_error_deg_raw", 0.0))
        except (TypeError, ValueError):
            target_bearing_raw = 0.0
        gate_bearing_corrected = self._apply_horizon_lock_to_bearing(
            gate_bearing_raw,
            parkur_label,
            "gate",
            gate_detected_raw,
            update_state=False,
        )
        target_bearing_corrected = self._apply_horizon_lock_to_bearing(
            target_bearing_raw,
            parkur_label,
            "target",
            target_detected_raw,
            update_state=False,
        )
        data["gate_center_bearing_deg_imu_corrected"] = gate_bearing_corrected
        data["target_bearing_error_deg_imu_corrected"] = target_bearing_corrected
        data["gate_center_bearing_deg"] = gate_bearing_corrected
        data["target_bearing_error_deg"] = target_bearing_corrected
        if target_signal_active:
            self._apply_horizon_lock_to_bearing(
                target_bearing_raw,
                parkur_label,
                "target",
                True,
                update_state=True,
            )
        elif gate_signal_active:
            self._apply_horizon_lock_to_bearing(
                gate_bearing_raw,
                parkur_label,
                "gate",
                True,
                update_state=True,
            )
        else:
            self._set_horizon_lock_idle(reason="no_detection", channel="none")

        try:
            frame_age_s = float(data.get("frame_age_s", 999.0))
        except (TypeError, ValueError):
            frame_age_s = 999.0

        self.camera_ready = bool(frame_age_s < CAMERA_FRAME_TIMEOUT_S and not status_stale)
        self.lidar_ready = bool(
            self.lidar_available and (time.monotonic() - self.last_lidar_time) < LIDAR_READY_TIMEOUT_S
            and self._lidar_quality_ready
        )

        fusion_scope_active = bool(FUSION_ENABLED and self.state in (self.STATE_NAV, self.STATE_ENGAGE))
        if fusion_scope_active:
            data = self._apply_camera_lidar_fusion(data)
        else:
            self._gate_fusion_hold_since = None
            self._target_fusion_hold_since = None
            self._yellow_fusion_hold_since = None
            self._orange_fusion_hold_since = None
            self._gate_ghost_latched = False
            self._target_ghost_latched = False
            self._yellow_ghost_latched = False
            self._orange_ghost_latched = False
            self._gate_camera_only_since = None
            self._target_camera_only_since = None
            self._yellow_camera_only_since = None
            self._orange_camera_only_since = None

        adapt = data.get("camera_adaptation", {})
        if not isinstance(adapt, dict):
            adapt = {}
        self.camera_adaptation = {
            "enabled": bool(adapt.get("enabled", False)),
            "mode": str(adapt.get("mode", "normal")),
            "luma_mean": round(float(adapt.get("luma_mean", 0.0) or 0.0), 2),
            "exposure_gain": round(float(adapt.get("exposure_gain", 1.0) or 1.0), 3),
            "exposure_beta": round(float(adapt.get("exposure_beta", 0.0) or 0.0), 3),
            "hsv_s_shift": int(adapt.get("hsv_s_shift", 0) or 0),
            "hsv_v_shift": int(adapt.get("hsv_v_shift", 0) or 0),
            "hsv_profile": str(adapt.get("hsv_profile", "base")),
        }
        data["camera_adaptation"] = dict(self.camera_adaptation)
        self.camera_status = data

    def _update_watchdog(self):
        if not self.master:
            self.heartbeat_age_s = 0.0
            self.link_heartbeat_age_s = 0.0
            self.link_heartbeat_source = "offline_simulation" if self.simulation_mode else "no_master"
            self.failsafe_state = "normal"
            if not hasattr(self, '_sim_watchdog_logged'):
                print(f"[WATCHDOG] MAVLink master yok; watchdog pasif source={self.link_heartbeat_source}")
                self._sim_watchdog_logged = True
            return

        self.heartbeat_age_s = time.monotonic() - self.last_heartbeat_time
        self._refresh_link_heartbeat()

        link_fail = self.link_heartbeat_age_s >= HEARTBEAT_FAIL_S
        onboard_fail = self.heartbeat_age_s >= HEARTBEAT_FAIL_S
        link_warn = self.link_heartbeat_age_s >= HEARTBEAT_WARN_S
        onboard_warn = self.heartbeat_age_s >= HEARTBEAT_WARN_S

        # DETAILED LOGGING for watchdog
        if not hasattr(self, '_last_watchdog_log'):
            self._last_watchdog_log = 0.0
        now = time.monotonic()
        if (now - self._last_watchdog_log) >= 2.0:  # MORE FREQUENT logging (was 10s)
            print(f"[WATCHDOG] link_hb={self.link_heartbeat_age_s:.1f}s [fail={link_fail} warn={link_warn}] onboard_hb={self.heartbeat_age_s:.1f}s [fail={onboard_fail} warn={onboard_warn}] src={self.link_heartbeat_source} failsafe={self.failsafe_state}")
            self._last_watchdog_log = now

        if link_fail and self.failsafe_state != "hold":
            print(f"[ESTOP] [FAILSAFE] Telemetri link heartbeat timeout >=30s (age={self.link_heartbeat_age_s:.1f}s)")
            self.failsafe_state = "triggered"
            if self.mission_active:
                self._command_speed_heading(FAILSAFE_SLOW_MPS, 0.0)
                time.sleep(1.0)
            self.failsafe_state = "hold"
            self._enter_hold("FAILSAFE_LINK_HEARTBEAT")
        elif onboard_fail and self.failsafe_state != "hold":
            print(f"[ESTOP] [FAILSAFE] Onboard heartbeat timeout >=30s (age={self.heartbeat_age_s:.1f}s)")
            self.failsafe_state = "triggered"
            if self.mission_active:
                self._command_speed_heading(FAILSAFE_SLOW_MPS, 0.0)
                time.sleep(1.0)
            self.failsafe_state = "hold"
            self._enter_hold("FAILSAFE_ONBOARD_HEARTBEAT")
        elif link_warn or onboard_warn:
            self.failsafe_state = "warning"
            if self.mission_active:
                self._warn_throttled(
                    "heartbeat_warn_safe_speed",
                    (
                        f"[WARN] [FAILSAFE] heartbeat warn: link={self.link_heartbeat_age_s:.1f}s "
                        f"onboard={self.heartbeat_age_s:.1f}s; speed_cap={FAILSAFE_SLOW_MPS:.2f}m/s"
                    ),
                    period_s=2.0,
                )
                self._command_speed_heading(min(float(self.v_target or 0.0), FAILSAFE_SLOW_MPS), 0.0)
        else:
            self.failsafe_state = "normal"

    def _trigger_estop(self, source, force_rc7=False):
        if self.estop_latched:
            return
        print(f"ESTOP_TRIGGERED source={source} force_rc7={force_rc7}")
        log_jsonl(
            "usv_main",
            False,
            event="ESTOP_TRIGGERED",
            source=str(source),
            force_rc7=bool(force_rc7),
            mission_active=bool(self.mission_active),
            state=int(self.state),
        )
        self.estop_latched = True
        self.estop_source = source
        self.command_lock = True
        self._set_mode("HOLD")
        self._force_safe_outputs(include_estop=force_rc7, repeat=8)
        self._enter_hold("ESTOP")

    def _clear_estop_if_safe(self):
        if not self.estop_latched:
            return True
        if self.rc_channels.get("ch7", 0) <= RC7_SAFE_PWM and not os.path.exists(FLAG_STOP):
            print("[OK] [ESTOP] Latch temizlendi (RC7 safe)")
            self.estop_latched = False
            self.estop_source = ""
            self.command_lock = False
            self._set_rc7_estop(False)
            self._refresh_health_check()
            self._write_state()
            return True
        print("[WARN] [ESTOP] RC7 hala kesme konumunda, görev baslatilamiyor")
        return False

    def _enter_hold(self, reason):
        self.state = self.STATE_HOLD
        self.mission_active = False
        self.hold_reason = str(reason or "UNKNOWN")
        if str(self.mission_end_reason or "none") == "none":
            self.mission_end_reason = f"hold:{self.hold_reason.lower()}"
        self.v_target = 0.0
        self.heading_target = self.current_heading
        self._set_guidance_idle(reason=f"hold:{self.hold_reason.lower()}")
        self._set_dynamic_speed_idle()
        self._set_wind_assist_idle()
        self._set_horizon_lock_idle(reason="hold", channel="none")
        self.stop_motors()
        self._set_mode("HOLD")
        if reason.startswith("FAILSAFE"):
            self.failsafe_state = "hold"
            if self._arm_virtual_anchor_center():
                self._set_virtual_anchor_idle(reason="armed", clear_center=False)
            else:
                self._set_virtual_anchor_idle(reason="anchor_arm_failed", clear_center=True)
        else:
            self._set_virtual_anchor_idle(reason="hold_non_failsafe", clear_center=True)
        self._refresh_health_check()
        self._update_mode_state(source="safety", reason=f"hold:{reason}")
        self._write_state()

    def _distance_and_heading_error(self, target_lat, target_lon):
        lat, lon, heading = self._resolve_nav_solution()
        snap = self._nav_snapshot_copy()
        self.nav_target_lat = float(target_lat)
        self.nav_target_lon = float(target_lon)
        if not self.nav_fix_valid:
            self._log_nav_invalid(target_lat, target_lon)
            self.nav_target_distance_m = 9999.0
            self.nav_target_distance_delta_m = 0.0
            self.nav_heading_error_delta_deg = 0.0
            self.nav_target_bearing_deg = 0.0
            self.nav_heading_error_deg = 0.0
            self.closest_waypoint_distance_m = 9999.0
            return 9999.0, 0.0, snap
        dist = haversine_distance(lat, lon, target_lat, target_lon)
        target_bearing = calculate_bearing(lat, lon, target_lat, target_lon)
        heading_error = normalize_heading_error(target_bearing - heading)
        if self._prev_nav_target_distance_m is None:
            self.nav_target_distance_delta_m = 0.0
            self.nav_target_distance_increase_count = 0
        else:
            self.nav_target_distance_delta_m = float(dist) - float(self._prev_nav_target_distance_m)
            if self.nav_target_distance_delta_m > 0.05:
                self.nav_target_distance_increase_count += 1
            elif self.nav_target_distance_delta_m < -0.05:
                self.nav_target_distance_increase_count = 0
        if self._prev_nav_heading_error_deg is None:
            self.nav_heading_error_delta_deg = 0.0
        else:
            self.nav_heading_error_delta_deg = normalize_heading_error(
                float(heading_error) - float(self._prev_nav_heading_error_deg)
            )
        self._prev_nav_target_distance_m = float(dist)
        self._prev_nav_heading_error_deg = float(heading_error)
        self.nav_target_distance_m = float(dist)
        self.nav_target_bearing_deg = float(target_bearing)
        self.nav_heading_error_deg = float(heading_error)
        self.closest_waypoint_distance_m = float(dist)
        self._closest_waypoint_distance_seen = min(float(self._closest_waypoint_distance_seen), float(dist))
        return dist, heading_error, snap

    def _configure_waypoint_leg(self, target_lat, target_lon, leg_start=None, start_snapshot=None):
        """
        Freeze the active leg geometry once per waypoint.

        The acceptance logic uses this fixed leg frame to reject side-clips of
        the waypoint radius. Without a stable leg frame, any loop-to-loop drift
        in the start reference can reintroduce early acceptance.
        """
        if leg_start and len(leg_start) >= 2:
            start_lat = float(leg_start[0])
            start_lon = float(leg_start[1])
        elif isinstance(start_snapshot, dict) and start_snapshot.get("valid"):
            start_lat = float(start_snapshot.get("lat", self.current_lat))
            start_lon = float(start_snapshot.get("lon", self.current_lon))
        else:
            start_lat = float(self.current_lat)
            start_lon = float(self.current_lon)

        sim_now = bool(self.simulation_mode or os.environ.get("USV_SIM") == "1")
        if sim_now and not self._geo_position_valid_values(start_lat, start_lon):
            nav_fb = load_sim_nav_state(control_dir=CONTROL_DIR)
            if nav_fb.get("valid"):
                start_lat = float(nav_fb.get("lat", start_lat) or start_lat)
                start_lon = float(nav_fb.get("lon", start_lon) or start_lon)

        self._wp_leg_start_lat = start_lat
        self._wp_leg_start_lon = start_lon
        self._wp_leg_target_lat = float(target_lat)
        self._wp_leg_target_lon = float(target_lon)
        self._wp_leg_length_m = float(
            haversine_distance(start_lat, start_lon, float(target_lat), float(target_lon))
        )
        self._wp_leg_valid = bool(self._wp_leg_length_m > 0.25)
        self._wp_accept_hold_start = None
        self.progress_along_leg_m = 0.0
        self.waypoint_passed_gate = False
        self.waypoint_accept_reason = "tracking"
        self.nav_arrival_phase = "tracking"
        self.closest_waypoint_distance_m = 9999.0
        self._closest_waypoint_distance_seen = 9999.0
        self._prev_nav_target_distance_m = None
        self._prev_nav_heading_error_deg = None
        self.nav_target_distance_delta_m = 0.0
        self.nav_target_distance_increase_count = 0
        self.nav_heading_error_delta_deg = 0.0
        self._nav_align_mode = "align"
        self._nav_align_t0 = time.monotonic()
        self._nav_align_stable_since = None
        self._wp_transition_until = time.monotonic() + float(NAV_ALIGN_TURN_IMMUNITY_S)
        self._nav_align_lock_until = time.monotonic() + float(NAV_ALIGN_TURN_IMMUNITY_S)
        self._nav_align_phase_last = "ACQUIRE_HEADING"

    def _latlon_offset_m(self, origin_lat, origin_lon, lat, lon):
        lat_scale = 111320.0
        lon_scale = 111320.0 * math.cos(math.radians(float(origin_lat)))
        east_m = (float(lon) - float(origin_lon)) * lon_scale
        north_m = (float(lat) - float(origin_lat)) * lat_scale
        return east_m, north_m

    def _waypoint_leg_progress(self, nav_snapshot):
        leg_length = max(float(self._wp_leg_length_m or 0.0), 0.001)
        min_meaningful_leg_m = max(float(R_WP_M) * 1.5, 3.0)
        if (not self._wp_leg_valid) or leg_length < min_meaningful_leg_m:
            # Very short first legs make the "passed waypoint" test meaningless
            # because even tiny pose noise looks like a complete fly-through.
            # In that case we fall back to true proximity (`inner_radius`) only.
            return 0.0, False
        leg_east, leg_north = self._latlon_offset_m(
            self._wp_leg_start_lat,
            self._wp_leg_start_lon,
            self._wp_leg_target_lat,
            self._wp_leg_target_lon,
        )
        cur_east, cur_north = self._latlon_offset_m(
            self._wp_leg_start_lat,
            self._wp_leg_start_lon,
            nav_snapshot.get("lat", self.current_lat),
            nav_snapshot.get("lon", self.current_lon),
        )
        progress_m = ((cur_east * leg_east) + (cur_north * leg_north)) / leg_length
        if progress_m < -2.0 or progress_m > (leg_length + 5.0):
            self._wp_leg_valid = False
            return 0.0, False
        passed_gate = progress_m >= leg_length
        return float(progress_m), bool(passed_gate)

    def _evaluate_waypoint_acceptance(self, distance_m, nav_snapshot):
        """
        Accept a waypoint only when it is actually reached, not merely grazed.

        The normal case uses the shared compliance acceptance radius and hold
        time. `radius+passed` handles the fly-through case, but only after the
        craft has crossed the waypoint along the current leg.
        """
        progress_m, passed_gate = self._waypoint_leg_progress(nav_snapshot)
        self.progress_along_leg_m = round(float(progress_m), 3)
        self.waypoint_passed_gate = bool(passed_gate)

        dist_m = float(distance_m)
        arrival_radius = float(R_WP_M)
        hold_required_s = float(T_HOLD_S)
        pass_radius = max(arrival_radius + 0.4, min(float(R_WP_M), 2.8))
        exit_margin = float(NAV_WP_ARRIVAL_EXIT_MARGIN_M)
        within_arrival = dist_m <= arrival_radius
        within_arrival_latch = dist_m <= (arrival_radius + exit_margin)

        result = False
        hold_active = False

        if within_arrival:
            if self._wp_accept_hold_start is None:
                self._wp_accept_hold_start = time.monotonic()
            hold_s = time.monotonic() - self._wp_accept_hold_start
            self.waypoint_accept_reason = "arrival_radius"
            self.nav_arrival_phase = "holding" if hold_s < hold_required_s else "accepted"
            result = bool(hold_s >= hold_required_s)
            hold_active = True
        elif within_arrival_latch and self._wp_accept_hold_start is not None:
            hold_s = time.monotonic() - float(self._wp_accept_hold_start)
            self.waypoint_accept_reason = "arrival_radius"
            self.nav_arrival_phase = "holding" if hold_s < hold_required_s else "accepted"
            result = bool(hold_s >= hold_required_s)
            hold_active = True
        else:
            # Fly-through acceptance: if craft clearly crossed waypoint along leg
            # and is now moving away while still in pass radius, accept without
            # requiring a long stop that can induce circling.
            passed_margin = max(0.25, min(1.2, float(self._wp_leg_length_m or 0.0) * 0.08))
            passed_stably = bool(passed_gate and progress_m >= (float(self._wp_leg_length_m or 0.0) + passed_margin))
            moving_away = dist_m >= (float(self._closest_waypoint_distance_seen or dist_m) + 0.12)
            if passed_stably and dist_m <= pass_radius and moving_away:
                self._wp_accept_hold_start = None
                self.waypoint_accept_reason = "leg_pass_trend"
                self.nav_arrival_phase = "accepted"
                result = True
                hold_active = True
            else:
                self._wp_accept_hold_start = None
                self.waypoint_accept_reason = "tracking"
                self.nav_arrival_phase = "tracking"
                result = False
                hold_active = False

        # Log on state transition (once per reason change)
        new_reason = str(self.waypoint_accept_reason)
        if getattr(self, "_last_logged_accept_reason", None) != new_reason:
            self._last_logged_accept_reason = new_reason
            print(
                f"[NAV] WP accept state: reason={new_reason} phase={self.nav_arrival_phase} "
                f"dist={dist_m:.2f}m arrival_r={arrival_radius:.2f}m pass_r={pass_radius:.2f}m "
                f"speed={abs(float(self.current_speed_mps or 0.0)):.2f}m/s progress={self.progress_along_leg_m:.2f}m"
            )
        return result, hold_active

    def _check_abort(self):
        self._drain_mav_messages()
        if self._consume_flag(FLAG_STOP):
            self._trigger_estop("YKI", force_rc7=True)
            return True
        self._read_camera_status()
        self._update_watchdog()
        self._refresh_health_check()
        self._record_file3_if_due()
        if self.estop_latched:
            return True
        return self.state == self.STATE_HOLD

    def _check_rc_connected(self, timeout_sec=3):
        if self.simulation_mode or not self.master:
            return True
        end_t = time.time() + timeout_sec
        while time.time() < end_t:
            self._drain_mav_messages()
            valid = self._rc_link_active()
            if valid:
                return True
            time.sleep(0.1)
        return False

    def _mission_hash(self, coords):
        try:
            payload = json.dumps(coords, sort_keys=True, separators=(",", ":")).encode("utf-8")
            return hashlib.sha256(payload).hexdigest()
        except Exception:
            return ""

    def _extract_mavlink_waypoint(self, msg):
        try:
            from pymavlink import mavutil

            command = int(getattr(msg, "command", 0) or 0)
            nav_cmds = (
                int(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT),
                int(mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT),
            )
            if command not in nav_cmds:
                return None
            msg_type = msg.get_type()
            x = float(getattr(msg, "x", 0) or 0)
            y = float(getattr(msg, "y", 0) or 0)
            if msg_type == "MISSION_ITEM_INT":
                lat, lon = x / 1e7, y / 1e7
            elif msg_type == "MISSION_ITEM":
                # MAVLink mission lat/lon use degE7; tiny values treated as degrees
                if max(abs(x), abs(y)) > 1.0e4:
                    lat, lon = x / 1e7, y / 1e7
                else:
                    lat, lon = x, y
            else:
                return None
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                return None
            if abs(lat) < 1e-7 and abs(lon) < 1e-7:
                return None
            return [round(lat, 7), round(lon, 7)]
        except Exception:
            return None

    def _request_mission_item_int(self, seq):
        try:
            from pymavlink import mavutil

            mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
            self.master.mav.mission_request_int_send(
                self.master.target_system,
                self.master.target_component,
                int(seq),
                mission_type,
            )
            return True
        except TypeError:
            self.master.mav.mission_request_int_send(
                self.master.target_system,
                self.master.target_component,
                int(seq),
            )
            return True
        except AttributeError:
            self.pixhawk_mission_sync_error = "mission_request_int_unavailable"
            print("[ERROR] [MISSION_SYNC] MISSION_REQUEST_INT kullanilamiyor; deprecated MISSION_REQUEST reddedildi")
            log_jsonl(
                "usv_main",
                False,
                event="pixhawk_mission_sync_error",
                reason="mission_request_int_unavailable",
                seq=int(seq),
            )
            return False

    def _send_mission_ack_accepted(self):
        if not self.master:
            return
        try:
            from pymavlink import mavutil

            try:
                self.master.mav.mission_ack_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self.master.mav.mission_ack_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_ACCEPTED,
                )
        except Exception as exc:
            self._warn_throttled("mission_ack", f"[WARN] [MISSION_SYNC] MISSION_ACK gonderilemedi: {exc}", period_s=2.0)

    def _sim_test_should_upload_local_mission(self):
        return bool(os.environ.get("USV_SIM") == "1" and USV_MODE != USV_MODE_RACE and self.master is not None)

    def _service_startup_wait_once(self):
        """Keep readiness inputs fresh while a pre-loop startup operation waits."""
        try:
            self._drain_mav_messages()
        except Exception as exc:
            self._warn_throttled(
                "startup_wait_mav",
                f"[WARN] [STARTUP] MAVLink servis hatasi: {exc}",
                period_s=2.0,
            )

        if self.master and os.environ.get("USV_SIM") == "1":
            now_mono = time.monotonic()
            last_gps_pump = float(getattr(self, "_startup_wait_gps_pump_last_ts", 0.0) or 0.0)
            if (now_mono - last_gps_pump) >= 0.1:
                self._startup_wait_gps_pump_last_ts = now_mono
                try:
                    self._sitl_gps_input_from_sim()
                except Exception:
                    pass
            last_origin_seed = float(getattr(self, "_startup_wait_origin_last_ts", 0.0) or 0.0)
            if (now_mono - last_origin_seed) >= 5.0:
                self._startup_wait_origin_last_ts = now_mono
                try:
                    self._sitl_seed_global_origin_if_needed()
                except Exception:
                    pass

        try:
            self._read_camera_status()
        except Exception as exc:
            self._bump_error("camera_state_read_error", f"[WARN] [CAM] Startup okuma hatasi: {exc}")
        try:
            self._compute_trust_bar()
        except Exception:
            pass
        self._refresh_health_check()
        self._update_mode_state(source="startup_wait")
        self._write_state()

    def _wait_for_mission_upload_prereq(self):
        deadline = time.monotonic() + max(0.0, float(PIXHAWK_MISSION_READY_WAIT_S))
        last_log = 0.0
        while True:
            self._service_startup_wait_once()
            prereq = self._guided_prereq_status()
            if prereq.get("ready", False):
                return True, prereq

            now = time.monotonic()
            if now - last_log >= 3.0:
                last_log = now
                missing = ",".join(prereq.get("missing", [])) or "unknown"
                print(f"[MISSION_SYNC] SITL GPS/EKF hazırlanıyor, mission upload erteleniyor (missing={missing})")

            if now >= deadline:
                missing = ",".join(prereq.get("missing", [])) or "unknown"
                self.pixhawk_mission_sync_error = f"mission_upload_prereq_timeout:{missing}"
                log_jsonl(
                    "usv_main",
                    False,
                    event="pixhawk_mission_upload_skipped",
                    reason=self.pixhawk_mission_sync_error,
                    prereq=dict(prereq),
                )
                self._write_state()
                return False, prereq

            time.sleep(max(0.05, float(PIXHAWK_MISSION_READY_SERVICE_DT_S)))

    def _arm_ready_now(self):
        guided_prereq = self._guided_prereq_status()
        fc_start = self._fc_start_status(guided_prereq=guided_prereq)
        if not guided_prereq.get("ready", False) or not fc_start.get("ready", False):
            return False, guided_prereq, fc_start
        if os.environ.get("USV_SIM") == "1":
            if not bool(self.fc_ekf_report_seen) or not bool(self.fc_nav_mode_ready):
                return False, guided_prereq, fc_start
            origin_mono = float(getattr(self, "_sim_ekf_origin_set_mono", 0.0) or 0.0)
            if origin_mono > 0.0:
                since_origin = time.monotonic() - origin_mono
                if since_origin < float(SIM_EKF_ORIGIN_ARM_DELAY_S):
                    return False, guided_prereq, fc_start
        return True, guided_prereq, fc_start

    def _wait_for_arm_ready(self, deadline_s=None):
        deadline = time.monotonic() + max(0.0, float(
            deadline_s if deadline_s is not None else FC_ARM_READY_WAIT_S
        ))
        last_log = 0.0
        ready, guided_prereq, fc_start = self._arm_ready_now()
        while True:
            if ready:
                return True, guided_prereq, fc_start

            now = time.monotonic()
            if now - last_log >= 2.0:
                last_log = now
                block_reason = str(fc_start.get("block_reason", "unknown"))
                print(f"[START] Arm hazirlik bekleniyor (block={block_reason})")

            if now >= deadline:
                return False, guided_prereq, fc_start

            self._service_startup_wait_once()
            ready, guided_prereq, fc_start = self._arm_ready_now()
            time.sleep(max(0.05, float(PIXHAWK_MISSION_READY_SERVICE_DT_S)))

    def _wait_for_fc_start_ready(self, deadline_s=None):
        deadline = time.monotonic() + max(0.0, float(
            deadline_s if deadline_s is not None else FC_START_PREREQ_WAIT_S
        ))
        last_log = 0.0
        guided_prereq = self._guided_prereq_status()
        fc_start = self._fc_start_status(guided_prereq=guided_prereq)
        while True:
            if guided_prereq.get("ready", False) and fc_start.get("ready", False):
                return True, guided_prereq, fc_start

            now = time.monotonic()
            if now - last_log >= 2.0:
                last_log = now
                missing = ",".join(guided_prereq.get("missing", [])) or "unknown"
                block_reason = str(fc_start.get("block_reason", "unknown"))
                print(
                    f"[START] FC hazirlik bekleniyor (missing={missing}, block={block_reason})"
                )

            if now >= deadline:
                return False, guided_prereq, fc_start

            self._service_startup_wait_once()
            guided_prereq = self._guided_prereq_status()
            fc_start = self._fc_start_status(guided_prereq=guided_prereq)
            time.sleep(max(0.05, float(PIXHAWK_MISSION_READY_SERVICE_DT_S)))

    def _start_request_pending(self):
        if os.path.exists(FLAG_START):
            return True
        return str(getattr(self, "start_phase", "idle") or "idle") in (
            "requested",
            "preflight",
            "fc_prereq",
            "arming",
        )

    def _mission_upload_home(self, coords):
        try:
            from sim_nav_state import parse_sim_home

            hlat, hlon, halt, _ = parse_sim_home()
            if self._geo_position_valid_values(hlat, hlon):
                return float(hlat), float(hlon), float(halt)
        except Exception:
            pass
        if self._geo_position_valid_values(self.current_lat, self.current_lon):
            return float(self.current_lat), float(self.current_lon), 0.0
        if coords:
            return float(coords[0][0]), float(coords[0][1]), 0.0
        return -35.363262, 149.165237, 0.0

    def _send_mission_item_int(self, seq, lat, lon, alt_m, current=0):
        from pymavlink import mavutil

        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        args = [
            self.master.target_system,
            self.master.target_component,
            int(seq),
            frame,
            command,
            int(current),
            1,
            0.0,
            0.0,
            0.0,
            0.0,
            int(round(float(lat) * 1.0e7)),
            int(round(float(lon) * 1.0e7)),
            float(alt_m),
        ]
        try:
            self.master.mav.mission_item_int_send(
                *args,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
        except TypeError:
            self.master.mav.mission_item_int_send(*args)

    def _upload_local_mission_to_pixhawk(self, coords, source):
        if not self._sim_test_should_upload_local_mission():
            return False
        if self._start_request_pending():
            return False
        try:
            from pymavlink import mavutil

            coords = validate_coordinate_mission(coords)
            if not coords:
                self.pixhawk_mission_sync_error = "local_mission_empty"
                self.pixhawk_mirror_status = "failed"
                self.pixhawk_mirror_error = self.pixhawk_mission_sync_error
                return False
            mission_hash = self._mission_hash(coords)
            if mission_hash and mission_hash == self._pixhawk_mission_upload_hash:
                self.pixhawk_mirror_status = "synced"
                self.pixhawk_mirror_error = ""
                return True
            if mission_hash and mission_hash == self._pixhawk_mission_upload_attempt_hash:
                if str(self.pixhawk_mission_sync_error or ""):
                    if not (
                        str(self.pixhawk_mission_sync_error).startswith("mission_upload_prereq_not_ready:")
                        and bool(self._guided_prereq_status().get("ready", False))
                    ):
                        return False
            self._pixhawk_mission_upload_attempt_hash = mission_hash

            # SITL can reject mission upload while GPS/EKF is still warming up.
            # Keep the startup service loop alive while waiting so readiness and
            # mission_state.json do not go stale before the main loop starts.
            prereq = self._guided_prereq_status()
            prereq_ready = bool(prereq.get("ready", False))
            if not prereq_ready and (
                USV_MODE == USV_MODE_RACE or self._sim_test_should_upload_local_mission()
            ):
                prereq_ready, prereq = self._wait_for_mission_upload_prereq()
            if not prereq_ready:
                missing = ",".join(prereq.get("missing", [])) or "unknown"
                self.pixhawk_mirror_status = "pending_fc_ready"
                self.pixhawk_mirror_error = f"mission_upload_prereq_not_ready:{missing}"
                self.pixhawk_mission_sync_error = ""
                log_jsonl(
                    "usv_main",
                    False,
                    event="pixhawk_mission_upload_skipped",
                    reason=self.pixhawk_mirror_error,
                    source=str(source),
                    mission_hash=str(mission_hash),
                    prereq=dict(prereq),
                )
                print(f"[WARN] [MISSION_SYNC] Local mission Pixhawk upload atlandi: {missing}")
                return False

            while self.master.recv_match(blocking=False) is not None:
                pass

            home_lat, home_lon, home_alt = self._mission_upload_home(coords)
            items = [(home_lat, home_lon, home_alt, 1)]
            items.extend((float(lat), float(lon), 0.0, 0) for lat, lon in coords)
            count = len(items)

            try:
                self.master.mav.mission_clear_all_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self.master.mav.mission_clear_all_send(
                    self.master.target_system,
                    self.master.target_component,
                )
            self._service_startup_wait_once()

            try:
                self.master.mav.mission_count_send(
                    self.master.target_system,
                    self.master.target_component,
                    int(count),
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self.master.mav.mission_count_send(
                    self.master.target_system,
                    self.master.target_component,
                    int(count),
                )

            deadline = time.monotonic() + max(5.0, float(count) * 2.0)
            sent = set()
            ack_result = None
            while time.monotonic() < deadline:
                self._service_startup_wait_once()
                msg = self.master.recv_match(
                    type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                    blocking=True,
                    timeout=0.15,
                )
                if msg is None:
                    continue
                mtype = msg.get_type()
                if mtype == "MISSION_ACK":
                    ack_result = int(getattr(msg, "type", -1) or -1)
                    break
                seq = int(getattr(msg, "seq", -1) or -1)
                if 0 <= seq < count:
                    lat, lon, alt_m, current = items[seq]
                    self._send_mission_item_int(seq, lat, lon, alt_m, current=current)
                    sent.add(seq)

            if ack_result is None:
                ack_deadline = time.monotonic() + 1.0
                while time.monotonic() < ack_deadline:
                    self._service_startup_wait_once()
                    ack = self.master.recv_match(type=["MISSION_ACK"], blocking=True, timeout=0.15)
                    if ack is not None:
                        ack_result = int(getattr(ack, "type", -1) or -1)
                        break

            if ack_result not in (None, int(mavutil.mavlink.MAV_MISSION_ACCEPTED)):
                ack_names = {
                    1: "ERROR",
                    2: "UNSUPPORTED_FRAME",
                    3: "UNSUPPORTED",
                    4: "NO_SPACE",
                    5: "INVALID",
                    6: "INVALID_PARAM1",
                    7: "INVALID_PARAM2",
                    8: "INVALID_PARAM3",
                    9: "INVALID_PARAM4",
                    10: "INVALID_PARAM5_X",
                    11: "INVALID_PARAM6_Y",
                    12: "INVALID_PARAM7",
                    13: "INVALID_SEQUENCE",
                    14: "DENIED",
                    15: "OPERATION_CANCELLED",
                }
                self.pixhawk_mission_sync_error = f"mission_upload_ack_{ack_result}"
                self.pixhawk_mirror_status = "failed"
                self.pixhawk_mirror_error = self.pixhawk_mission_sync_error
                self.pixhawk_mission_count = 0
                log_jsonl(
                    "usv_main",
                    False,
                    event="pixhawk_mission_upload_failed",
                    reason=self.pixhawk_mission_sync_error,
                    ack_result=int(ack_result),
                    ack_name=ack_names.get(int(ack_result), "UNKNOWN"),
                    sent_sequences=sorted(int(seq) for seq in sent),
                    expected_count=int(count),
                )
                print(
                    "[ERROR] [MISSION_SYNC] Local mission upload rejected "
                    f"ack={ack_result}({ack_names.get(int(ack_result), 'UNKNOWN')}) "
                    f"sent={len(sent)}/{count}"
                )
                if not self._start_request_pending():
                    self._pixhawk_mission_upload_attempt_hash = ""
                return False
            if len(sent) < count:
                self.pixhawk_mission_sync_error = f"mission_upload_incomplete_{len(sent)}_of_{count}"
                self.pixhawk_mirror_status = "failed"
                self.pixhawk_mirror_error = self.pixhawk_mission_sync_error
                self.pixhawk_mission_count = 0
                missing = [seq for seq in range(count) if seq not in sent]
                log_jsonl(
                    "usv_main",
                    False,
                    event="pixhawk_mission_upload_failed",
                    reason=self.pixhawk_mission_sync_error,
                    missing_sequences=missing,
                    sent_sequences=sorted(int(seq) for seq in sent),
                    expected_count=int(count),
                )
                return False

            self.pixhawk_mission_count = int(count)
            self.pixhawk_mission_synced_at = time.time()
            self.pixhawk_mission_sync_error = ""
            self.pixhawk_mirror_status = "synced"
            self.pixhawk_mirror_error = ""
            self._pixhawk_mission_download_hash = self._mission_hash(coords)
            self._pixhawk_mission_upload_hash = mission_hash
            log_jsonl(
                "usv_main",
                False,
                event="pixhawk_mission_uploaded",
                source=str(source),
                pixhawk_mission_count=int(count),
                nav_waypoints=len(coords),
            )
            print(f"[MISSION_SYNC] Local mission uploaded to SITL Pixhawk source={source} count={count}")
            return True
        except Exception as exc:
            self.pixhawk_mission_sync_error = f"mission_upload_failed:{exc}"
            self.pixhawk_mirror_status = "failed"
            self.pixhawk_mirror_error = self.pixhawk_mission_sync_error
            self._warn_throttled(
                "pixhawk_mission_upload",
                f"[WARN] [MISSION_SYNC] Local mission Pixhawk upload basarisiz: {exc}",
                period_s=2.0,
            )
            return False

    def _download_pixhawk_mission(self, timeout_s=PIXHAWK_MISSION_SYNC_TIMEOUT_S):
        if not self.master:
            self.pixhawk_mission_sync_error = "no_mavlink_master"
            return None
        try:
            from pymavlink import mavutil

            while self.master.recv_match(blocking=False) is not None:
                pass
            deadline = time.monotonic() + max(1.0, float(timeout_s))
            try:
                self.master.mav.mission_request_list_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
            except TypeError:
                self.master.mav.mission_request_list_send(
                    self.master.target_system,
                    self.master.target_component,
                )
            count_msg = None
            while time.monotonic() < deadline:
                msg = self.master.recv_match(type=["MISSION_COUNT"], blocking=True, timeout=0.35)
                if msg is not None:
                    count_msg = msg
                    break
            if count_msg is None:
                self.pixhawk_mission_sync_error = "mission_count_timeout"
                return None

            count = int(getattr(count_msg, "count", 0) or 0)
            self.pixhawk_mission_count = count
            if count <= 0:
                self.pixhawk_mission_sync_error = "empty_pixhawk_mission"
                return None
            deadline = time.monotonic() + max(float(timeout_s), (float(count) * 6.0) + 1.0)

            coords = []
            for seq in range(count):
                item = None
                for attempt in range(1, 4):
                    if not self._request_mission_item_int(seq):
                        return None
                    item_deadline = time.monotonic() + 2.0
                    while time.monotonic() < item_deadline and time.monotonic() < deadline:
                        msg = self.master.recv_match(
                            type=["MISSION_ITEM_INT", "MISSION_ITEM"],
                            blocking=True,
                            timeout=0.25,
                        )
                        if msg is None:
                            continue
                        if int(getattr(msg, "seq", -1) or -1) == seq:
                            item = msg
                            break
                    if item is not None:
                        break
                    self._warn_throttled(
                        f"mission_item_{seq}_retry",
                        f"[WARN] [MISSION_SYNC] seq={seq} timeout attempt={attempt}/3",
                        period_s=0.5,
                    )
                if item is None:
                    self.pixhawk_mission_sync_error = f"mission_item_{seq}_timeout"
                    log_jsonl(
                        "usv_main",
                        False,
                        event="pixhawk_mission_sync_error",
                        reason=self.pixhawk_mission_sync_error,
                        seq=int(seq),
                        retries=3,
                        timeout_s=2.0,
                    )
                    return None
                if seq == 0:
                    continue
                coord = self._extract_mavlink_waypoint(item)
                if coord is not None:
                    coords.append(coord)

            if len(coords) == 0:
                self.pixhawk_mission_sync_error = "no_nav_waypoints_extracted"
                print("[ERROR] [MISSION_SYNC] no_nav_waypoints_extracted (NAV_WAYPOINT/NAV_SPLINE yok)")
                log_jsonl(
                    "usv_main",
                    False,
                    event="pixhawk_mission_sync_error",
                    reason="no_nav_waypoints_extracted",
                    pixhawk_mission_count=int(count),
                )
                return None

            coords = validate_coordinate_mission(coords)
            self.pixhawk_mission_synced_at = time.time()
            self._pixhawk_mission_download_hash = self._mission_hash(coords)
            self.pixhawk_mission_sync_error = ""
            self._send_mission_ack_accepted()
            print(f"[MISSION_SYNC] Pixhawk mission mirrored: raw_count={count} nav_waypoints={len(coords)}")
            log_jsonl(
                "usv_main",
                False,
                event="pixhawk_mission_synced",
                pixhawk_mission_count=count,
                nav_waypoints=len(coords),
            )
            return coords
        except Exception as exc:
            self.pixhawk_mission_sync_error = str(exc)
            self._warn_throttled("pixhawk_mission_sync", f"[WARN] [MISSION_SYNC] Pixhawk mission alinamadi: {exc}", period_s=2.0)
            return None

    def _apply_mission_coords(self, coords, source, input_format=MISSION_INPUT_FORMAT):
        coords = validate_coordinate_mission(coords)
        self.nav_waypoints, self.engage_wp = split_nav_engage(coords)
        self.waypoints_p1 = self.nav_waypoints
        self.waypoints_p2 = []
        self.waypoints_p3 = [self.engage_wp] if self.engage_wp is not None else []
        self.mission_input_format = str(input_format or MISSION_INPUT_FORMAT)
        self.mission_upload_source = str(source or "unknown")
        self.mission_split_profile = get_mission_split_profile(len(coords))
        target_state = load_target_state(TARGET_STATE_FILE)
        self.target_color = str(target_state.get("target_color") or self.target_color or "RED").strip().upper() or "RED"
        self.mission_validated_at_timestamp = time.time()
        self.mission_synced = True

    def _apply_mission_profile_parse(self, parsed, *, target_color_override=""):
        """Apply mission_profile metadata without assuming static P1/P2/P3 splits."""
        parsed = parsed if isinstance(parsed, dict) else {}
        profile = parsed.get("mission_profile") if isinstance(parsed.get("mission_profile"), dict) else {}
        self.mission_profile = dict(profile or {})
        self.mission_profile_valid = bool(parsed.get("mission_profile_valid", bool(profile)))
        self.mission_profile_race_ready = bool(parsed.get("race_ready", False) and self.mission_profile_valid)
        self.mission_profile_error = str(parsed.get("profile_error", "") or "")
        self.p2_min_gate_count = int(
            self.mission_profile.get("p2_min_gate_count", MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT)
            or MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT
        )
        self.p3_engagement_mode = str(
            self.mission_profile.get("p3_engagement_mode", MISSION_PROFILE_DEFAULT_P3_ENGAGEMENT_MODE)
            or MISSION_PROFILE_DEFAULT_P3_ENGAGEMENT_MODE
        )
        policy = self.mission_profile.get("phase_transition_policy")
        if isinstance(policy, dict):
            merged_policy = dict(MISSION_PROFILE_DEFAULT_TRANSITION_POLICY)
            merged_policy.update(policy)
            self.phase_transition_policy = merged_policy
        else:
            self.phase_transition_policy = dict(MISSION_PROFILE_DEFAULT_TRANSITION_POLICY)
        profile_target = str(self.mission_profile.get("target_color") or parsed.get("target_color") or "").strip().upper()
        if target_color_override:
            profile_target = str(target_color_override).strip().upper()
        if profile_target:
            self.target_color = profile_target
        if self.mission_profile_valid:
            self.mission_split_profile = dict(self.mission_split_profile or {})
            self.mission_split_profile.update({
                "mission_profile_valid": True,
                "mission_profile_race_ready": bool(self.mission_profile_race_ready),
                "p2_min_gate_count": int(self.p2_min_gate_count),
                "p3_engagement_mode": str(self.p3_engagement_mode),
                "p1_waypoint_count_hint": self.mission_profile.get("p1_waypoint_count"),
                "parkur_ranges_hint": self.mission_profile.get("parkur_ranges"),
            })

    def _load_mission_profile_metadata_from_file(self, filepath):
        """Load optional profile metadata while preserving Pixhawk mission waypoints."""
        if not filepath or not os.path.exists(filepath):
            return False
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                raw_data = json.load(f)
            if not isinstance(raw_data, dict) or "mission_profile" not in raw_data:
                return False
            parsed = parse_mission_profile_payload(
                raw_data,
                upload_source=self.mission_upload_source,
                validation_timestamp=time.time(),
                race_required=False,
            )
            self._apply_mission_profile_parse(parsed)
            print(
                "[MISSION_PROFILE] Loaded profile metadata "
                f"target={self.target_color} p3_mode={self.p3_engagement_mode}"
            )
            log_jsonl(
                "usv_main",
                False,
                event="mission_profile_loaded",
                source="mission_file_metadata",
                race_ready=bool(self.mission_profile_race_ready),
                target_color=str(self.target_color),
                p2_min_gate_count=int(self.p2_min_gate_count),
                p3_engagement_mode=str(self.p3_engagement_mode),
            )
            return True
        except Exception as exc:
            self.mission_profile_valid = False
            self.mission_profile_race_ready = False
            self.mission_profile_error = str(exc)
            self._warn_throttled("mission_profile_load", f"[WARN] [MISSION_PROFILE] yuklenemedi: {exc}", period_s=2.0)
            return False

    def _sync_pixhawk_mission_if_available(self):
        coords = self._download_pixhawk_mission()
        if not coords:
            return False
        new_hash = self._mission_hash(coords)
        if self.mission_active:
            if new_hash and new_hash != self.pixhawk_mission_last_hash:
                self.post_start_mission_change_rejected = True
                print(
                    "[WARN] [MISSION_SYNC] post_start_mission_change_rejected: "
                    f"new_hash={new_hash[:12]} old_hash={self.pixhawk_mission_last_hash[:12]}"
                )
                log_jsonl(
                    "usv_main",
                    False,
                    event="post_start_mission_change_rejected",
                    pixhawk_mission_count=self.pixhawk_mission_count,
                    new_hash=new_hash,
                    active_hash=self.pixhawk_mission_last_hash,
                )
            return False
        self._apply_mission_coords(coords, source="pixhawk_mission", input_format="mavlink_mission_items")
        self.pixhawk_mission_last_hash = new_hash
        self.pixhawk_mirror_status = "synced"
        self.pixhawk_mirror_error = ""
        return True

    def _current_local_mission_coords(self):
        coords = list(getattr(self, "nav_waypoints", []) or [])
        engage = getattr(self, "engage_wp", None)
        if engage is not None:
            coords.append(engage)
        return coords

    def _mark_pixhawk_mirror_pending(self, reason="pending_fc_ready"):
        self.pixhawk_mirror_status = str(reason or "pending_fc_ready")
        self.pixhawk_mirror_error = ""
        self.pixhawk_mission_sync_error = ""

    def load_mission(self, filepath=None):
        """
        Load and validate mission file.

        Operational contract:
        - Flat ordered waypoint list: [[lat, lon], ...]

        Legacy compatibility:
        - Structured mission is accepted only if the compatibility switch is enabled.

        Pi keeps the full ordered mission as its local reference and derives P1/P2/P3
        slices from the onboard split profile. Target color is loaded separately from
        target_state.json, not from the mission waypoint file.
        """
        fp = filepath or MISSION_FILE
        skip_initial_pixhawk_download = bool(
            os.environ.get("USV_SIM") == "1"
            and USV_MODE != USV_MODE_RACE
            and os.path.exists(fp)
        )
        if skip_initial_pixhawk_download:
            self._mark_pixhawk_mirror_pending("pending_fc_ready")
            log_jsonl(
                "usv_main",
                False,
                event="pixhawk_mission_download_skipped",
                reason="local_mission_active_pixhawk_mirror_pending",
                source="sim_test_local_mission",
            )
        else:
            if self._sync_pixhawk_mission_if_available():
                self._load_mission_profile_metadata_from_file(fp)
                print(
                    f"[OK] [GOREV] Pixhawk mission kullaniliyor nav={len(self.nav_waypoints)} "
                    f"count={self.pixhawk_mission_count}"
                )
                return True

        if str(self.pixhawk_mission_sync_error or "") == "no_nav_waypoints_extracted":
            print(
                "[ERR] [MISSION] Pixhawk misyonundan NAV_WAYPOINT/NAV_SPLINE ayiklanamadi; "
                "dosya veya sim varsayilanina dusulmez."
            )
            self.mission_synced = False
            return False

        if os.path.exists(fp):
            try:
                with open(fp, "r", encoding="utf-8") as f:
                    raw_data = json.load(f)

                mission_target_color = ""
                coords_for_pixhawk_upload = []
                if isinstance(raw_data, list):
                    parsed = parse_mission_profile_payload(raw_data, race_required=False)
                    coords = validate_coordinate_mission(parsed.get("flat_waypoints", raw_data))
                    coords_for_pixhawk_upload = list(coords)
                    self.nav_waypoints = list(parsed.get("nav_waypoints", []))
                    self.engage_wp = parsed.get("engage_wp")
                    # Legacy compat
                    self.waypoints_p1 = self.nav_waypoints
                    self.waypoints_p2 = []
                    self.waypoints_p3 = [self.engage_wp] if self.engage_wp is not None else []
                    self.mission_input_format = MISSION_INPUT_FORMAT
                    self.mission_upload_source = "local_flat_file"
                    self.mission_split_profile = get_mission_split_profile(len(coords))
                    self._apply_mission_profile_parse(parsed)
                    self.pixhawk_mission_last_hash = self._mission_hash(coords)
                elif isinstance(raw_data, dict):
                    if "mission_profile" in raw_data:
                        parsed = parse_mission_profile_payload(
                            raw_data,
                            upload_source="structured_profile",
                            validation_timestamp=time.time(),
                            race_required=False,
                        )
                        self.nav_waypoints = list(parsed.get("nav_waypoints", []))
                        self.engage_wp = parsed.get("engage_wp")
                        self.waypoints_p1 = list(parsed.get("parkur1", [])) or self.nav_waypoints
                        self.waypoints_p2 = list(parsed.get("parkur2", []))
                        self.waypoints_p3 = list(parsed.get("p3_search_waypoints", parsed.get("parkur3", [])))
                        mission_target_color = str(parsed.get("target_color") or "").strip().upper()
                        self.mission_input_format = parsed.get("input_format", "structured_profile")
                        self.mission_upload_source = "structured_profile"
                        coords_for_pixhawk_upload = list(parsed.get("flat_waypoints", []))
                        self.mission_split_profile = get_mission_split_profile(len(coords_for_pixhawk_upload))
                        self._apply_mission_profile_parse(parsed, target_color_override=mission_target_color)
                        self.pixhawk_mission_last_hash = self._mission_hash(coords_for_pixhawk_upload)
                    else:
                        mission = adapt_mission_to_structured(raw_data, strict=True)
                        parsed = parse_mission_profile_payload(
                            raw_data,
                            upload_source="structured_legacy",
                            validation_timestamp=time.time(),
                            race_required=False,
                        )
                        self.nav_waypoints = parsed.get("nav_waypoints", mission.get("nav_waypoints", mission.get("parkur1", [])))
                        self.engage_wp = parsed.get("engage_wp", mission.get("engage_wp", None))
                        # Legacy compat
                        self.waypoints_p1 = list(parsed.get("parkur1", self.nav_waypoints))
                        self.waypoints_p2 = list(parsed.get("parkur2", []))
                        self.waypoints_p3 = list(parsed.get("p3_search_waypoints", parsed.get("parkur3", [])))
                        mission_target_color = str(parsed.get("target_color") or mission.get("target_color") or "").strip().upper()
                        self.mission_input_format = parsed.get("input_format", mission.get("_mission_input_format", "structured_legacy"))
                        self.mission_upload_source = mission.get("_adapter_source", "structured_legacy")
                        coords_for_pixhawk_upload = list(parsed.get("flat_waypoints", []))
                        if not coords_for_pixhawk_upload:
                            coords_for_pixhawk_upload = list(self.nav_waypoints)
                            if self.engage_wp is not None:
                                coords_for_pixhawk_upload.append(self.engage_wp)
                        self.mission_split_profile = dict(mission.get("_split_profile") or get_mission_split_profile(len(coords_for_pixhawk_upload)))
                        self._apply_mission_profile_parse(parsed, target_color_override=mission_target_color)
                        self.pixhawk_mission_last_hash = self._mission_hash(coords_for_pixhawk_upload)
                else:
                    raise ValueError(f"Mission must be list or object, got {type(raw_data).__name__}")

                if not self.nav_waypoints and self.engage_wp is None:
                    raise ValueError("Mission file contains profile metadata but no local navigation waypoints")

                total_count = len(self.nav_waypoints) + (1 if self.engage_wp is not None else 0)
                if not self.mission_split_profile:
                    self.mission_split_profile = get_mission_split_profile(total_count)
                target_state = load_target_state(TARGET_STATE_FILE)
                target_from_state = str(target_state.get("target_color") or "").strip().upper()
                self.target_color = target_from_state or mission_target_color or self.target_color or "RED"
                self.mission_validated_at_timestamp = time.time()
                self.mission_synced = True
                if self._sim_test_should_upload_local_mission() and self.mission_upload_source in (
                    "local_flat_file",
                    "structured_legacy",
                ):
                    self._mark_pixhawk_mirror_pending("pending_fc_ready")

                print(
                    f"[OK] [GOREV] VALIDATED nav={len(self.nav_waypoints)} engage={'yes' if self.engage_wp else 'no'} "
                    f"Hedef={self.target_color} format={self.mission_input_format}"
                )
                return True
            except Exception as exc:
                print(f"[ERR] [MISSION] Validation failed: {exc}")
                return False
        
        # Fallback: generate default mission for simulation
        if self.simulation_mode or os.environ.get("USV_SIM") == "1":
            try:
                home_tokens = [token.strip() for token in os.environ.get("SIM_HOME", "").split(",")]
                base_lat = float(home_tokens[0])
                base_lon = float(home_tokens[1])
            except Exception:
                base_lat, base_lon = -35.363262, 149.165237
            coords = default_sim_mission(base_lat, base_lon)
            self.nav_waypoints, self.engage_wp = split_nav_engage(coords)
            # Legacy compat
            self.waypoints_p1 = self.nav_waypoints
            self.waypoints_p2 = []
            self.waypoints_p3 = [self.engage_wp] if self.engage_wp is not None else []
            self.mission_input_format = MISSION_INPUT_FORMAT
            self.mission_upload_source = "sim_default_flat_file"
            self.mission_split_profile = get_mission_split_profile(len(coords))
            self.pixhawk_mission_last_hash = self._mission_hash(coords)
            target_state = load_target_state(TARGET_STATE_FILE)
            self.target_color = str(target_state.get("target_color") or self.target_color or "RED").strip().upper() or "RED"
            self.mission_validated_at_timestamp = time.time()
            self.mission_synced = True
            if self._sim_test_should_upload_local_mission():
                self._mark_pixhawk_mirror_pending("pending_fc_ready")
            else:
                self._upload_local_mission_to_pixhawk(coords, self.mission_upload_source)
            print(
                f"[SIM] [GOREV] Varsayilan gorev yuklendi "
                f"nav={len(self.nav_waypoints)} engage={'yes' if self.engage_wp else 'no'} "
                f"Hedef={self.target_color}"
            )
            return True
        print(f"[WARN] [GOREV] Dosya bulunamadi: {fp}")
        return False

    def validate_mission_schema(self, data):
        """Validate operational flat mission schema."""
        validate_coordinate_mission(data)
        return True

    def validate_parkur_waypoints(self, parkur_name, parkur_data, max_waypoints=None):
        """Validate waypoints for a parkur."""
        if not isinstance(parkur_data, list):
            raise ValueError(f"{parkur_name} must be array, got {type(parkur_data)}")
        if len(parkur_data) == 0:
            raise ValueError(f"{parkur_name} cannot be empty")
        if max_waypoints and len(parkur_data) > max_waypoints:
            raise ValueError(f"{parkur_name} can have max {max_waypoints} waypoints, got {len(parkur_data)}")
        
        for i, wp in enumerate(parkur_data):
            if not isinstance(wp, list) or len(wp) != 2:
                raise ValueError(f"{parkur_name}[{i}] must be [lat, lon], got {wp}")
            lat, lon = wp
            if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
                raise ValueError(f"{parkur_name}[{i}] coordinates must be numbers")
            if not (-90 <= lat <= 90):
                raise ValueError(f"{parkur_name}[{i}] latitude {lat} out of range [-90,90]")
            if not (-180 <= lon <= 180):
                raise ValueError(f"{parkur_name}[{i}] longitude {lon} out of range [-180,180]")
        
        return True

    def validate_target_color(self, target_color):
        """Validate target color value."""
        valid_colors = ["RED", "GREEN", "BLACK", "KIRMIZI_SANCAK", "YESIL_SANCAK", "SIYAH_HEDEF"]
        if target_color not in valid_colors:
            raise ValueError(f"target_color '{target_color}' must be one of {valid_colors}")
        return True

    def _refresh_mission_progress_ui_at_start(self):
        """Dashboard: ilk NAV waypoint + sayaç (SITL hazırlık / mode beklerken bos kalmasın)."""
        n = len(self.nav_waypoints) if self.nav_waypoints else 0
        if n <= 0:
            self._wp_target = "--"
            self._wp_info = "-- / --"
            self._active_waypoint_index = -1
            return
        try:
            wp0 = self.nav_waypoints[0]
            lat, lon = float(wp0[0]), float(wp0[1])
            self._wp_target = f"{lat:.7f}, {lon:.7f}"
            self._wp_info = f"1 / {n}"
            self._active_waypoint_index = 0
        except (TypeError, ValueError, IndexError):
            self._wp_target = "--"
            self._wp_info = f"0 / {n}"
            self._active_waypoint_index = -1

    def start_mission(self):
        if self.mission_active:
            print("[WARN] [GOREV] Zaten aktif")
            return False
        self.start_phase = "requested"
        if os.path.exists(FLAG_START):
            self._consume_flag(FLAG_START)
        if not self.load_mission(MISSION_FILE):
            err = str(self.pixhawk_mission_sync_error or "")
            if err == "no_nav_waypoints_extracted":
                self.mission_end_reason = "start_rejected:no_nav_waypoints_extracted"
            else:
                self.mission_end_reason = "start_rejected:mission_load_failed"
            print("❌ [MISSION] Guncel mission yuklenemedi")
            fail_reason = "no_nav_waypoints_extracted" if err == "no_nav_waypoints_extracted" else "mission_load_failed"
            log_jsonl(
                "usv_main",
                False,
                event="START_REJECTED",
                reason=fail_reason,
                pixhawk_mission_sync_error=err or None,
                mission_end_reason=self.mission_end_reason,
            )
            self.start_phase = "rejected"
            self._write_state()
            return False
        self.mission_end_reason = "none"

        if not self.mission_synced:
            self.mission_end_reason = "start_rejected:mission_not_synced"
            print("START_REJECTED: mission_not_synced")
            log_jsonl("usv_main", False, event="START_REJECTED", reason="mission_not_synced")
            self.start_phase = "rejected"
            self._write_state()
            return False
        if USV_MODE == USV_MODE_RACE and self.mission_upload_source != "pixhawk_mission":
            self.mission_end_reason = "start_rejected:race_requires_pixhawk_mission_sync"
            print("START_REJECTED: race_requires_pixhawk_mission_sync")
            log_jsonl(
                "usv_main",
                False,
                event="START_REJECTED",
                reason="race_requires_pixhawk_mission_sync",
                mission_upload_source=self.mission_upload_source,
            )
            self.start_phase = "rejected"
            self._write_state()
            return False
        if USV_MODE == USV_MODE_RACE:
            race_profile_errors = []
            if not self.mission_profile_valid or not self.mission_profile_race_ready:
                race_profile_errors.append("mission_profile_not_race_ready")
            if not str(self.target_color or "").strip() or str(self.target_color).strip() == "--":
                race_profile_errors.append("target_color_not_locked")
            if str(self.p3_engagement_mode or "") not in (
                "vision_color_track",
                "vision_color_track_with_gps_fallback",
            ):
                race_profile_errors.append("invalid_p3_engagement_mode")
            if race_profile_errors:
                reason = "+".join(race_profile_errors)
                self.mission_end_reason = f"start_rejected:{reason}"
                print(f"START_REJECTED: {reason}")
                log_jsonl(
                    "usv_main",
                    False,
                    event="START_REJECTED",
                    reason=reason,
                    mission_profile_valid=bool(self.mission_profile_valid),
                    mission_profile_race_ready=bool(self.mission_profile_race_ready),
                    mission_profile_error=str(self.mission_profile_error or ""),
                    target_color=str(self.target_color or ""),
                    p3_engagement_mode=str(self.p3_engagement_mode or ""),
                )
                self.start_phase = "rejected"
                self._write_state()
                return False
        self.start_phase = "preflight"
        self._drain_mav_messages()
        self._read_camera_status()
        self._update_watchdog()
        self._refresh_health_check()

        if not self._clear_estop_if_safe():
            self.mission_end_reason = "start_rejected:estop_latched"
            log_jsonl("usv_main", False, event="START_REJECTED", reason="estop_latched")
            self.start_phase = "rejected"
            self._write_state()
            return False

        if not self._check_rc_connected():
            print("❌ [GOREV] RC bagli degil")
            self.mission_end_reason = "start_rejected:rc_not_connected"
            log_jsonl("usv_main", False, event="START_REJECTED", reason="rc_not_connected")
            self.start_phase = "rejected"
            self._write_state()
            return False

        self._refresh_health_check()
        if not self.health_ready:
            missing = ", ".join(self.health_missing) if self.health_missing else "unknown"
            print(f"❌ [READY] HEALTH_CHECK gecmedi: {missing}")
            self.mission_end_reason = f"start_rejected:health_check:{missing}"
            log_jsonl(
                "usv_main",
                False,
                event="START_REJECTED",
                reason="health_check_failed",
                missing=self.health_missing,
            )
            self.start_phase = "rejected"
            self._write_state()
            return False

        self.start_phase = "fc_prereq"
        self._drain_mav_messages()
        fc_ready, guided_prereq, fc_start = self._wait_for_fc_start_ready()
        if not fc_ready:
            missing = ",".join(guided_prereq.get("missing", [])) or "unknown"
            block_reason = str(fc_start.get("block_reason", "unknown"))
            if not guided_prereq.get("ready", False):
                self.mission_end_reason = f"start_rejected:guided_prereq_missing:{missing}"
            else:
                self.mission_end_reason = f"start_rejected:fc_start_blocked:{block_reason}"
            print(f"START_REJECTED: {self.mission_end_reason}")
            log_jsonl(
                "usv_main",
                False,
                event="START_REJECTED",
                reason="fc_start_blocked" if guided_prereq.get("ready", False) else "guided_prereq_missing",
                missing=list(guided_prereq.get("missing", [])),
                fc_start_block_reason=block_reason,
                gps_fix_type=int(guided_prereq.get("gps_fix_type", 0) or 0),
                global_position_int_received=bool(guided_prereq.get("global_position_int_received", False)),
                position_valid=bool(guided_prereq.get("position_valid", False)),
                position_source=str(guided_prereq.get("position_source", "invalid")),
                sim_nav_valid=bool(guided_prereq.get("sim_nav_valid", False)),
                sim_nav_reason=str(guided_prereq.get("sim_nav_reason", "")),
            )
            self.start_phase = "rejected"
            self._write_state()
            return False

        self.start_phase = "arming"
        self.command_lock = True
        self._sim_mavlink_actuation_rc = False
        self.sim_actuation_fallback = "none"
        self.sim_actuation_fallback_reason = "none"
        self._manual_override_active = False
        self._rc_neutral_since = time.monotonic()
        self.state = self.STATE_IDLE
        self.hold_reason = "NONE"
        self.mission_end_reason = "none"
        self.failsafe_state = "normal"
        self._lidar_degraded_since = None
        self._lidar_degraded_mode = "normal"
        self._lidar_degraded_age_s = 0.0
        self._obstacle_threat_active = False
        self._obstacle_threat_source = "none"
        self._set_dynamic_speed_idle()
        self._set_wind_assist_idle()
        self._set_horizon_lock_idle(reason="mission_active", channel="none")
        self._set_virtual_anchor_idle(reason="mission_active", clear_center=True)
        self.gate_count = 0
        self.timeout_count = 0
        self._gate_event_start = None
        self._gate_event_latched = False
        self._refresh_mission_progress_ui_at_start()
        self._set_guidance_idle(reason="mission_start")
        self._lidar_stale_consecutive = 0
        self._lidar_stale_warned = False
        self._lidar_stale_blocked = False
        arm_ready, guided_prereq, fc_start = self._wait_for_arm_ready()
        if not arm_ready:
            block_reason = str(fc_start.get("block_reason", "unknown"))
            self.mission_end_reason = f"start_rejected:arm_prereq_timeout:{block_reason}"
            print(f"START_REJECTED: {self.mission_end_reason}")
            log_jsonl(
                "usv_main",
                False,
                event="START_REJECTED",
                reason="arm_prereq_timeout",
                fc_start_block_reason=block_reason,
                missing=list(guided_prereq.get("missing", [])),
            )
            self.command_lock = False
            self.state = self.STATE_IDLE
            self.stop_motors()
            self.start_phase = "rejected"
            self._write_state()
            return False
        if not self._arm():
            print("❌ [START] Arming basarisiz")
            arm_block = str(self.fc_start_block_reason or "unknown")
            self.mission_end_reason = f"start_rejected:arm_failed:{arm_block}"
            log_jsonl("usv_main", False, event="START_REJECTED", reason="arm_failed", fc_start_block_reason=arm_block)
            self.command_lock = False
            self.state = self.STATE_IDLE
            self.stop_motors()
            self.start_phase = "rejected"
            self._write_state()
            return False
        # Arming basarili → command_lock kutulayalim
        self.mission_active = True
        self.mission_start_time = time.time()
        self.state = self.STATE_NAV
        self.command_lock = False
        self._update_mode_state(source="mission", reason="mission_started")
        self.start_phase = "started"
        self._write_state()
        print("[START] [GOREV] Baslatildi")
        log_jsonl(
            "usv_main",
            False,
            event="mission_start_allowed",
            guidance_path="native",
            health_ready=bool(self.health_ready),
        )
        # GUARD: Wait for heading synchronization before first navigation command
        # Gazebo pose update delays 2-5s; skip early navigation to prevent ±180° heading wrap
        print("[WAIT] Heading stabilization...")
        heading_stable_count = 0
        for attempt in range(30):  # Max 3 seconds at 10Hz
            if 0.0 <= self.current_heading < 360.0 and self.nav_fix_valid:
                heading_stable_count += 1
                if heading_stable_count >= 5:  # 5 stable readings = 500ms stable
                    print(f"[OK] Heading stable: {self.current_heading:.1f}°")
                    break
            else:
                heading_stable_count = 0
            time.sleep(0.1)
        return True

    # _navigate_p1_waypoint removed — dead code after unified NAV refactor.
    # run_nav() uses _navigate_p2_waypoint (compute_nav_decision) for all waypoints.
    # Gate assist + lidar avoidance are active on every nav waypoint uniformly.

    def _wait_p2_ready(self):
        print("[CHECK] [P1->P2] Kamera/Lidar hazirlik kontrolu")
        next_log = 0.0
        started = time.monotonic()
        while self.mission_active:
            if self._check_abort():
                return False
            if self.camera_ready and self.lidar_ready:
                print("[OK] [P1->P2] Hazirlik tamam")
                return True
            elapsed = time.monotonic() - started
            if elapsed >= float(P2_READY_TIMEOUT_S):
                self.mission_end_reason = "p2_ready_timeout"
                print(
                    f"[ERR] [P1->P2] READY_TIMEOUT {elapsed:.1f}s "
                    f"camera_ready={self.camera_ready} lidar_ready={self.lidar_ready}"
                )
                log_jsonl(
                    "usv_main",
                    False,
                    event="p2_ready_timeout",
                    elapsed_s=round(float(elapsed), 3),
                    timeout_s=round(float(P2_READY_TIMEOUT_S), 3),
                    camera_ready=bool(self.camera_ready),
                    lidar_ready=bool(self.lidar_ready),
                )
                self._enter_hold("P2_READY_TIMEOUT")
                return False
            self._command_speed_heading(P2_WAIT_SPEED_MPS, 0.0)
            if time.monotonic() >= next_log:
                print(
                    f"[WAIT] [P1->P2] Bekleniyor camera_ready={self.camera_ready} "
                    f"lidar_ready={self.lidar_ready} mode=AUTO speed_cap<={P2_WAIT_SPEED_MPS:.1f}m/s"
                )
                next_log = time.monotonic() + 1.5
            self._write_state()
            time.sleep(LOOP_DT)
        return False

    def _track_gate_event(self):
        """Track P2 gate passage with bearing sign-change verification (spec compliant)."""
        event_active_raw = bool(self.camera_status.get("gate_passed_event", False))
        event_active = event_active_raw
        if FUSION_ENABLED:
            gate_confirm_recent = (time.monotonic() - self._last_gate_confirm_ts) <= FUSION_GATE_EVENT_CONFIRM_WINDOW_S
            if event_active_raw and not gate_confirm_recent:
                self._fusion_log("gate_event_ghost_rejected", "[FUSION] gate_event ghost_rejected no_recent_confirm")
            event_active = event_active_raw and gate_confirm_recent
        
        # Extract bearing for sign-change verification (Spec: bearing işareti değişimi)
        bearing_center = float(self.camera_status.get("gate_center_bearing_deg", 0.0))
        
        if event_active:
            if self._gate_event_start is None:
                self._gate_event_start = time.monotonic()
            
            # Check bearing sign change: if previous bearing exists, verify sign transition
            bearing_sign_changed = False
            if self._gate_bearing_prev is not None:
                prev_sign = 1 if self._gate_bearing_prev >= 0 else -1
                curr_sign = 1 if bearing_center >= 0 else -1
                bearing_sign_changed = (prev_sign != curr_sign)
            
            if (time.monotonic() - self._gate_event_start) >= P2_GATE_CONFIRM_S and not self._gate_event_latched:
                if bearing_sign_changed or self._gate_bearing_prev is None:
                    # Bearing verified or first confirmation (fallback)
                    self.gate_count += 1
                    self._gate_event_latched = True
                    print(f"[GATE] [P2] gate_gecildi -> {self.gate_count} (bearing={bearing_center:.1f}°, sign_change={bearing_sign_changed})")
            
            self._gate_bearing_prev = bearing_center
        else:
            self._gate_event_start = None
            self._gate_event_latched = False
            self._gate_bearing_prev = None

    def _p2_entry_evidence(self):
        """Return whether passive AUTO monitoring has reached P2-like perception."""
        self._read_camera_status()
        gate_detected = bool(
            self.camera_status.get("gate_detected", False)
            or self.camera_status.get("gate_detected_raw", False)
        )
        try:
            gate_stable_s = float(self.camera_status.get("gate_stable_s", 0.0) or 0.0)
        except (TypeError, ValueError):
            gate_stable_s = 0.0
        gate_event = bool(
            self.camera_status.get("gate_passed_event", False)
            or self.camera_status.get("gate_passed_event_raw", False)
        )
        yellow_seen = bool(
            self.camera_status.get("yellow_obstacle_detected", False)
            or self.camera_status.get("yellow_obstacle_detected_raw", False)
        )
        orange_seen = bool(
            self.camera_status.get("orange_boundary_detected", False)
            or self.camera_status.get("orange_boundary_detected_raw", False)
        )
        sensors_ready = bool(self.camera_ready and self.lidar_ready)
        visual_evidence = bool(
            gate_event
            or (gate_detected and gate_stable_s >= float(P2_STABLE_S))
            or yellow_seen
            or orange_seen
        )
        if sensors_ready and visual_evidence:
            reason_parts = []
            if gate_event:
                reason_parts.append("gate_event")
            if gate_detected:
                reason_parts.append(f"gate_stable:{gate_stable_s:.1f}s")
            if yellow_seen:
                reason_parts.append("yellow_obstacle")
            if orange_seen:
                reason_parts.append("orange_boundary")
            return True, "+".join(reason_parts) or "p2_visual_evidence"
        if not sensors_ready:
            return False, f"waiting_sensors:camera={self.camera_ready}:lidar={self.lidar_ready}"
        return False, "waiting_p2_visual_evidence"

    def _p2_remaining_start_index_from_fc(self):
        """Best-effort local waypoint index for switching from Pixhawk AUTO to Pi GUIDED."""
        candidates = []
        if int(self.mission_current_seq) >= 0:
            candidates.append(int(self.mission_current_seq))
        if int(self.mission_reached_seq) >= 0:
            candidates.append(int(self.mission_reached_seq) + 1)
        if not candidates:
            return 0
        return max(0, min(len(self.nav_waypoints), max(candidates)))

    def _wait_for_p2_gate_minimum(self):
        required = max(0, int(getattr(self, "p2_min_gate_count", MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT)))
        if required <= 0 or self.gate_count >= required:
            return True
        print(f"[CHECK] [P2] gate_count={self.gate_count}/{required} ek kapi kaniti bekleniyor")
        deadline = time.monotonic() + float(P2_GATE_MIN_WAIT_TIMEOUT_S)
        next_log = 0.0
        while self.mission_active and self.gate_count < required:
            if self._check_abort():
                return False
            if time.monotonic() >= deadline:
                self.mission_end_reason = "p2_gate_min_timeout"
                print(f"[ERR] [P2] gate_min_timeout gate={self.gate_count}/{required}")
                log_jsonl(
                    "usv_main",
                    False,
                    event="p2_gate_min_timeout",
                    gate_count=int(self.gate_count),
                    required=int(required),
                    timeout_s=round(float(P2_GATE_MIN_WAIT_TIMEOUT_S), 3),
                )
                return False
            self._read_camera_status()
            self._track_gate_event()
            front = float(getattr(self, "lidar_center_dist", 99.0) or 99.0)
            if front < float(D_MIN_M):
                self._command_speed_heading(0.0, 0.0)
            else:
                gate_detected = bool(self.camera_status.get("gate_detected", False))
                gate_bearing = float(self.camera_status.get("gate_center_bearing_deg", 0.0) or 0.0)
                speed = min(float(P2_WAIT_SPEED_MPS), 0.6) if gate_detected else 0.0
                self._command_speed_heading(speed, gate_bearing if gate_detected else 0.0)
            if time.monotonic() >= next_log:
                print(f"[WAIT] [P2] gate_count={self.gate_count}/{required}")
                next_log = time.monotonic() + 1.5
            self._write_state()
            time.sleep(LOOP_DT)
        return self.gate_count >= required

    def _validate_lidar_frame(self, msg):
        now_wall = time.time()
        n_total = max(0, len(getattr(msg, "ranges", []) or []))
        if n_total <= 0:
            return {
                "valid": False,
                "reason": "empty",
                "drop_reason": "empty",
                "clock_source": "none",
                "stamp_age_s": None,
                "valid_ratio": 0.0,
                "points": [],
                "front_samples": [],
            }

        stamp_age_s = None
        clock_source = "none"
        stamp_reliable = False
        try:
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            if stamp is not None:
                ts = float(getattr(stamp, "sec", 0) or 0) + (float(getattr(stamp, "nanosec", 0) or 0) * 1e-9)
                if ts > 0.0:
                    raw_age_s = now_wall - ts
                    if abs(raw_age_s) <= float(LIDAR_SIM_CLOCK_FUTURE_TOL_S):
                        stamp_age_s = max(0.0, raw_age_s)
                        clock_source = "header_wall_delta"
                        stamp_reliable = True
                    else:
                        stamp_age_s = raw_age_s
                        clock_source = "header_unreliable"
                        stamp_reliable = False
        except Exception:
            stamp_age_s = None
            clock_source = "header_parse_error"
            stamp_reliable = False
        if stamp_age_s is None and clock_source == "none":
            clock_source = "no_stamp"

        reject_by_stamp = False
        if self.simulation_mode:
            reject_by_stamp = bool(stamp_reliable and stamp_age_s is not None and stamp_age_s > float(LIDAR_FRAME_MAX_AGE_S))
            if clock_source in ("header_unreliable", "no_stamp", "header_parse_error"):
                # Sim clock is often not wall-synced; fall back to content-valid frame gating.
                clock_source = f"{clock_source}_sim_fallback"
                stamp_age_s = None
                reject_by_stamp = False
        else:
            reject_by_stamp = bool(stamp_age_s is not None and stamp_age_s > float(LIDAR_FRAME_MAX_AGE_S))
        if reject_by_stamp:
            return {
                "valid": False,
                "reason": "stale",
                "drop_reason": "stale",
                "clock_source": clock_source,
                "stamp_age_s": stamp_age_s,
                "valid_ratio": 0.0,
                "points": [],
                "front_samples": [],
            }

        raw_ranges = []
        for val in msg.ranges:
            try:
                raw_ranges.append(float(val))
            except (TypeError, ValueError):
                raw_ranges.append(float("nan"))

        valid_raw = [r for r in raw_ranges if math.isfinite(r) and LIDAR_SELF_FILTER_MIN_M < r < LIDAR_VALID_MAX_M]
        valid_ratio = float(len(valid_raw)) / float(max(1, n_total))
        if valid_ratio < float(LIDAR_VALID_RATIO_MIN):
            return {
                "valid": False,
                "reason": "low_valid_ratio",
                "drop_reason": "low_valid_ratio",
                "clock_source": clock_source,
                "stamp_age_s": stamp_age_s,
                "valid_ratio": valid_ratio,
                "points": [],
                "front_samples": [],
            }

        filtered = []
        front_samples = []
        angle = float(msg.angle_min)
        for idx, r in enumerate(raw_ranges):
            if not (math.isfinite(r) and LIDAR_SELF_FILTER_MIN_M < r < LIDAR_VALID_MAX_M):
                angle += float(msg.angle_increment)
                continue
            left = raw_ranges[idx - 1] if idx > 0 else float("nan")
            right = raw_ranges[idx + 1] if idx + 1 < n_total else float("nan")
            neigh = [v for v in (left, r, right) if math.isfinite(v) and LIDAR_SELF_FILTER_MIN_M < v < LIDAR_VALID_MAX_M]
            if len(neigh) < 2:
                angle += float(msg.angle_increment)
                continue
            med = sorted(neigh)[len(neigh) // 2]
            if abs(r - med) > float(LIDAR_ANGULAR_RESIDUAL_MAX_M):
                angle += float(msg.angle_increment)
                continue
            if (
                math.isfinite(left)
                and math.isfinite(right)
                and abs(r - left) > float(LIDAR_JUMP_ISOLATION_M)
                and abs(r - right) > float(LIDAR_JUMP_ISOLATION_M)
            ):
                angle += float(msg.angle_increment)
                continue
            deg = math.degrees(angle)
            if -90.0 <= deg <= 90.0:
                x_m = r * math.cos(angle)
                y_m = r * math.sin(angle)
                filtered.append((deg, r, x_m, y_m))
                front_samples.append((deg, r))
            angle += float(msg.angle_increment)

        return {
            "valid": True,
            "reason": "ok",
            "drop_reason": "ok",
            "clock_source": clock_source,
            "stamp_age_s": stamp_age_s,
            "valid_ratio": valid_ratio,
            "points": filtered,
            "front_samples": front_samples,
        }

    def _update_lidar_confidence_points(self, filtered_points):
        self._lidar_frame_seq += 1
        frame_seq = int(self._lidar_frame_seq)
        current_bins = {}
        for deg, r, x_m, y_m in filtered_points:
            bin_id = int(round(float(deg) / float(LIDAR_BIN_SIZE_DEG)))
            old = current_bins.get(bin_id)
            if old is None or r < old[1]:
                current_bins[bin_id] = (deg, r, x_m, y_m)

        for bin_id, sample in current_bins.items():
            state = self._lidar_temporal_bins.get(bin_id)
            if state is None:
                state = {"seen": deque(maxlen=LIDAR_TEMPORAL_WINDOW), "last_seen": 0.0, "last_range": 99.0}
                self._lidar_temporal_bins[bin_id] = state
            state["seen"].append(frame_seq)
            state["last_seen"] = time.monotonic()
            state["last_range"] = float(sample[1])

        stale_bins = []
        min_alive_seq = frame_seq - (LIDAR_TEMPORAL_WINDOW * 8)
        for bin_id, state in self._lidar_temporal_bins.items():
            seen = deque([s for s in state.get("seen", deque()) if s >= min_alive_seq], maxlen=LIDAR_TEMPORAL_WINDOW)
            state["seen"] = seen
            if not seen:
                stale_bins.append(bin_id)
        for bin_id in stale_bins:
            self._lidar_temporal_bins.pop(bin_id, None)

        stable_points = []
        tentative_points = []
        for bin_id, (deg, r, x_m, y_m) in current_bins.items():
            state = self._lidar_temporal_bins.get(bin_id, {})
            seen = [s for s in state.get("seen", []) if s >= frame_seq - (LIDAR_TEMPORAL_WINDOW - 1)]
            temporal = clamp(float(len(seen)) / float(max(1, LIDAR_TEMPORAL_WINDOW)), 0.0, 1.0)
            neighbor_seen = 0
            for off in (-1, 1):
                n_state = self._lidar_temporal_bins.get(bin_id + off)
                if not n_state:
                    continue
                n_hits = [s for s in n_state.get("seen", []) if s >= frame_seq - 1]
                if n_hits:
                    neighbor_seen += 1
            spatial = 1.0 if neighbor_seen >= 1 else 0.25
            range_q = clamp((float(LIDAR_VALID_MAX_M) - float(r)) / float(LIDAR_VALID_MAX_M), 0.0, 1.0)
            conf = (0.45 * temporal) + (0.35 * spatial) + (0.20 * range_q)
            sample = (deg, r, x_m, y_m, conf)
            if conf < float(LIDAR_CONF_DROP):
                continue
            if conf >= float(LIDAR_CONF_STABLE) and len(seen) >= int(LIDAR_TEMPORAL_MIN_SEEN):
                stable_points.append(sample)
            elif conf >= float(LIDAR_CONF_TENTATIVE):
                tentative_points.append(sample)

        self._lidar_stable_point_count = int(len(stable_points))
        self._lidar_tentative_point_count = int(len(tentative_points))
        return stable_points, tentative_points

    def _grid_key(self, x_m, y_m):
        half = float(LIDAR_GRID_SPAN_M) / 2.0
        x_clamped = clamp(float(x_m), -half, half)
        y_clamped = clamp(float(y_m), -half, half)
        ix = int(math.floor((x_clamped + half) / float(LIDAR_GRID_RES_M)))
        iy = int(math.floor((y_clamped + half) / float(LIDAR_GRID_RES_M)))
        return ix, iy

    def _update_lidar_persistent_map(self, stable_points, tentative_points, frame_valid, decay_gain=1.0, stable_drop=0):
        now = time.monotonic()
        dt = max(0.01, min(0.5, now - float(self._lidar_grid_last_ts or now)))
        dt = dt * clamp(float(decay_gain), 1.0, float(LIDAR_UNRELIABLE_DECAY_GAIN_MAX))
        self._lidar_grid_last_ts = now

        # Decay old cells even when frame is degraded.
        to_delete = []
        occ_cells = 0
        items = list(self._lidar_grid.items())
        for key, cell in items:
            stable_n = int(cell.get("stable_count", 0))
            hit_n = int(cell.get("hit_count", 0))
            last_seen = float(cell.get("last_seen_ts", 0.0) or 0.0)
            if (
                stable_n >= int(LIDAR_MAP_PERSIST_STABLE_COUNT)
                and hit_n >= int(LIDAR_MAP_PERSIST_MIN_HITS)
                and (now - last_seen) < float(LIDAR_TAU_STABLE_S)
            ):
                occupied = (
                    float(cell.get("log_odds", 0.0)) > float(LIDAR_MAP_OCC_LOG_ODDS)
                    and stable_n >= int(LIDAR_MAP_OCC_STABLE_COUNT)
                )
                if occupied:
                    occ_cells += 1
                continue
            if stable_n > 0:
                tau = float(LIDAR_TAU_STABLE_S)
                if stable_n >= int(LIDAR_MAP_PERSIST_STABLE_COUNT):
                    tau = float(LIDAR_TAU_STABLE_S) * 3.5
                elif stable_n >= int(LIDAR_MAP_DISPLAY_STABLE_COUNT):
                    tau = float(LIDAR_TAU_STABLE_S) * 2.0
                if hit_n >= int(LIDAR_MAP_PERSIST_MIN_HITS):
                    tau = max(tau, float(LIDAR_TAU_STABLE_S) * 4.0)
            else:
                tau = float(LIDAR_TAU_TENTATIVE_S)
            cell["log_odds"] = float(cell.get("log_odds", 0.0)) * math.exp(-dt / max(tau, 0.1))
            if int(stable_drop) > 0:
                cell["stable_count"] = max(0, stable_n - int(stable_drop))
            stale_s = 10.0
            if stable_n >= int(LIDAR_MAP_PERSIST_STABLE_COUNT) and hit_n >= int(LIDAR_MAP_PERSIST_MIN_HITS):
                stale_s = 45.0
            elif stable_n >= int(LIDAR_MAP_DISPLAY_STABLE_COUNT):
                stale_s = 25.0
            if abs(float(cell["log_odds"])) < 0.05 and (now - float(cell.get("last_seen_ts", 0.0) or 0.0)) > stale_s:
                to_delete.append(key)
                continue
            occupied = (
                float(cell.get("log_odds", 0.0)) > float(LIDAR_MAP_OCC_LOG_ODDS)
                and int(cell.get("stable_count", 0)) >= int(LIDAR_MAP_OCC_STABLE_COUNT)
            )
            if occupied:
                occ_cells += 1
        for key in to_delete:
            self._lidar_grid.pop(key, None)

        if not frame_valid:
            self._lidar_occ_cells_count = int(occ_cells)
            return

        pose = self._current_spatial_pose()
        boat_wx, boat_wy = float(pose[0]), float(pose[1])

        def _touch_world(wx_m, wy_m, delta, stable_hit=False):
            ix, iy = self._world_grid_key(wx_m, wy_m)
            key = (int(ix), int(iy))
            cell = self._lidar_grid.get(key)
            if cell is None:
                cell = {"log_odds": 0.0, "last_seen_ts": 0.0, "hit_count": 0, "stable_count": 0}
                self._lidar_grid[key] = cell
            cell["log_odds"] = clamp(
                float(cell.get("log_odds", 0.0)) + float(delta),
                float(LIDAR_MAP_MIN_LOG_ODDS),
                float(LIDAR_MAP_MAX_LOG_ODDS),
            )
            cell["last_seen_ts"] = now
            if delta > 0.0:
                cell["hit_count"] = int(cell.get("hit_count", 0)) + 1
                if stable_hit:
                    cell["stable_count"] = int(cell.get("stable_count", 0)) + 1

        for deg, r, x_m, y_m, _ in stable_points:
            wx_end, wy_end = self._lidar_local_to_world_enu(x_m, y_m, pose=pose)
            steps = max(1, int(min(float(r) / float(LIDAR_GRID_RES_M), 40)))
            for step in range(1, steps):
                t = float(step) / float(steps)
                wx = boat_wx + (t * (wx_end - boat_wx))
                wy = boat_wy + (t * (wy_end - boat_wy))
                _touch_world(wx, wy, -0.35, stable_hit=False)
            _touch_world(wx_end, wy_end, +0.85, stable_hit=True)

        for _, _, x_m, y_m, _ in tentative_points:
            wx_end, wy_end = self._lidar_local_to_world_enu(x_m, y_m, pose=pose)
            _touch_world(wx_end, wy_end, +0.25, stable_hit=False)

        if len(self._lidar_grid) > int(self._lidar_grid_max_cells):
            # Drop oldest/weakest cells first.
            ordered = sorted(
                list(self._lidar_grid.items()),
                key=lambda item: (abs(float(item[1].get("log_odds", 0.0))), float(item[1].get("last_seen_ts", 0.0))),
            )
            for key, _ in ordered[: max(1, len(self._lidar_grid) - int(self._lidar_grid_max_cells))]:
                self._lidar_grid.pop(key, None)

        occ_cells = 0
        vals = list(self._lidar_grid.values())
        for cell in vals:
            if (
                float(cell.get("log_odds", 0.0)) > float(LIDAR_MAP_OCC_LOG_ODDS)
                and int(cell.get("stable_count", 0)) >= int(LIDAR_MAP_OCC_STABLE_COUNT)
            ):
                occ_cells += 1
        self._lidar_occ_cells_count = int(occ_cells)

    def _compute_map_sector_metrics(self):
        front_risk = 0.0
        left_risk = 0.0
        right_risk = 0.0
        front_min = 99.0
        left_min = 99.0
        right_min = 99.0
        occupied_cells = 0
        pose = self._current_spatial_pose()
        boat_wx, boat_wy = float(pose[0]), float(pose[1])
        max_range_m = float(P2_LIDAR_WARN_M) + 2.5
        items = list(self._lidar_grid.items())
        for (ix, iy), cell in items:
            log_odds = float(cell.get("log_odds", 0.0))
            stable_count = int(cell.get("stable_count", 0))
            if log_odds <= float(LIDAR_MAP_OCC_LOG_ODDS) or stable_count < int(LIDAR_MAP_OCC_STABLE_COUNT):
                continue
            wx, wy = self._world_grid_cell_center(ix, iy)
            if math.hypot(wx - boat_wx, wy - boat_wy) > max_range_m:
                continue
            x_m, y_m = self._world_enu_to_lidar_local(wx, wy, pose=pose)
            if x_m <= 0.0:
                continue
            d_m = math.hypot(x_m, y_m)
            if d_m > max_range_m:
                continue
            bearing = normalize_lidar_bearing_deg(math.degrees(math.atan2(y_m, x_m)))
            weight = clamp(log_odds / max(float(LIDAR_MAP_OCC_LOG_ODDS), 0.1), 0.0, 1.8)
            occupied_cells += 1
            map_sector = classify_lidar_map_sector(bearing)
            if map_sector == "front":
                front_risk += weight / max(0.6, d_m)
                front_min = min(front_min, d_m)
            elif map_sector == "left":
                left_risk += weight / max(0.6, d_m)
                left_min = min(left_min, d_m)
            elif map_sector == "right":
                right_risk += weight / max(0.6, d_m)
                right_min = min(right_min, d_m)

        metrics = {
            "front_min_m": float(front_min),
            "left_min_m": float(left_min),
            "right_min_m": float(right_min),
            "front_risk": clamp(front_risk / 2.4, 0.0, 1.0),
            "left_risk": clamp(left_risk / 1.8, 0.0, 1.0),
            "right_risk": clamp(right_risk / 1.8, 0.0, 1.0),
            "occupied_cells": int(occupied_cells),
        }
        self._lidar_map_metrics = metrics
        self._lidar_front_risk = float(metrics["front_risk"])
        self._lidar_left_risk = float(metrics["left_risk"])
        self._lidar_right_risk = float(metrics["right_risk"])
        return metrics

    def _occupied_lidar_map_points_world(self, limit=4096, min_stable_count=None):
        """Return occupied world-grid points in fixed ENU coordinates (highest confidence first)."""
        min_stable = int(LIDAR_MAP_OCC_STABLE_COUNT if min_stable_count is None else min_stable_count)
        scored = []
        for (ix, iy), cell in list(self._lidar_grid.items()):
            log_odds = float(cell.get("log_odds", 0.0))
            stable_count = int(cell.get("stable_count", 0))
            hit_count = int(cell.get("hit_count", 0))
            if log_odds <= float(LIDAR_MAP_OCC_LOG_ODDS) or stable_count < min_stable:
                continue
            wx, wy = self._world_grid_cell_center(ix, iy)
            score = (stable_count * 100) + (hit_count * 10) + log_odds
            scored.append((score, wx, wy))
        scored.sort(key=lambda item: item[0], reverse=True)
        return [(wx, wy) for _, wx, wy in scored[: int(limit)]]

    def _update_blocked_hysteresis(self, front_risk):
        risk = clamp(float(front_risk), 0.0, 1.0)
        if self._blocked_hysteresis:
            if risk <= float(LIDAR_BLOCKED_EXIT):
                self._blocked_hysteresis = False
        else:
            if risk >= float(LIDAR_BLOCKED_ENTER):
                self._blocked_hysteresis = True
        self._blocked_level = risk
        return bool(self._blocked_hysteresis)

    def _apply_unreliable_clock_risk_decay(self):
        if not self.simulation_mode:
            return
        if int(self._lidar_unreliable_clock_consecutive) <= 0:
            return
        decay = float(LIDAR_UNRELIABLE_RISK_DECAY) ** float(min(6, int(self._lidar_unreliable_clock_consecutive)))
        decay = clamp(decay, 0.10, 1.0)
        if isinstance(self._lidar_map_metrics, dict):
            for key in ("front_risk", "left_risk", "right_risk"):
                self._lidar_map_metrics[key] = clamp(float(self._lidar_map_metrics.get(key, 0.0)) * decay, 0.0, 1.0)
        self._lidar_front_risk = clamp(float(self._lidar_front_risk) * decay, 0.0, 1.0)
        self._lidar_left_risk = clamp(float(self._lidar_left_risk) * decay, 0.0, 1.0)
        self._lidar_right_risk = clamp(float(self._lidar_right_risk) * decay, 0.0, 1.0)
        self._blocked_level = clamp(float(self._blocked_level) * decay, 0.0, 1.0)
        if int(self._lidar_unreliable_clock_consecutive) >= int(LIDAR_UNRELIABLE_CLEAR_BLOCK_CONSEC):
            self._blocked_hysteresis = False

    def _maybe_trigger_sim_avoidance_recovery(self, dist_m, decision_mode):
        if not self.simulation_mode:
            self._avoid_recovery_samples.clear()
            return False
        mode = str(decision_mode or "")
        now = time.monotonic()
        if mode != "nav_avoid" or float(self._blocked_level) < float(SIM_AVOID_RECOVERY_BLOCKED_MIN):
            self._avoid_recovery_samples.clear()
            return False
        if (now - float(self._avoid_recovery_last_ts)) < float(SIM_AVOID_RECOVERY_COOLDOWN_S):
            return False
        self._avoid_recovery_samples.append((now, float(dist_m)))
        if len(self._avoid_recovery_samples) < 6:
            return False
        first_ts = float(self._avoid_recovery_samples[0][0])
        if (now - first_ts) < float(SIM_AVOID_RECOVERY_WINDOW_S):
            return False
        dist_start = min(float(item[1]) for item in list(self._avoid_recovery_samples)[:3])
        dist_end = max(float(item[1]) for item in list(self._avoid_recovery_samples)[-3:])
        dist_growth = float(dist_end - dist_start)
        if dist_growth < float(SIM_AVOID_RECOVERY_DIST_GROWTH_M):
            return False
        self._avoid_recovery_count += 1
        self._avoid_recovery_last_ts = now
        self._avoid_recovery_samples.clear()
        self._escape_lock_side = None
        self._avoid_side_lock_until = 0.0
        self._p2_avoid_smooth = 0.0
        self._avoid_bias_filtered_deg = 0.0
        self._blocked_hysteresis = False
        self._blocked_level = min(float(self._blocked_level), 0.35)
        self._update_lidar_persistent_map(
            [],
            [],
            frame_valid=False,
            decay_gain=float(LIDAR_UNRELIABLE_DECAY_GAIN_MAX),
            stable_drop=max(1, int(LIDAR_UNRELIABLE_STABLE_DROP)),
        )
        self._compute_map_sector_metrics()
        self._apply_unreliable_clock_risk_decay()
        self._set_wind_assist_idle()
        self._command_speed_heading(0.0, 0.0)
        log_jsonl(
            "usv_main",
            False,
            event="avoidance_recovery_reset",
            dist_start_m=round(float(dist_start), 3),
            dist_end_m=round(float(dist_end), 3),
            dist_growth_m=round(float(dist_growth), 3),
            blocked_level=round(float(self._blocked_level), 3),
            map_occupied_cells=int(self._lidar_occ_cells_count),
            front_risk=round(float(self._lidar_front_risk), 3),
            recovery_count=int(self._avoid_recovery_count),
        )
        print(
            f"[WARN] [NAV] avoidance recovery reset dist_growth={dist_growth:.2f}m "
            f"blocked={self._blocked_level:.2f} occ={self._lidar_occ_cells_count}"
        )
        self._write_state()
        time.sleep(float(SIM_AVOID_RECOVERY_HOLD_S))
        return True

    def _choose_avoid_side_from_risk(self, left_risk, right_risk):
        now = time.monotonic()
        side = str(self._escape_lock_side or "none")
        if now < float(self._avoid_side_lock_until) and side in ("left", "right"):
            return side
        left_score = float(left_risk)
        right_score = float(right_risk)
        if side == "left":
            left_score -= float(LIDAR_SIDE_SWITCH_PENALTY)
        elif side == "right":
            right_score -= float(LIDAR_SIDE_SWITCH_PENALTY)
        chosen = "left" if left_score <= right_score else "right"
        if side in ("left", "right") and chosen != side:
            self._avoid_switch_count += 1
        self._escape_lock_side = chosen
        self._avoid_side_lock_until = now + float(LIDAR_SIDE_LOCK_S)
        return chosen

    def _nav_surge_allowed(self, heading_error_deg):
        err_abs = abs(float(heading_error_deg))
        if str(getattr(self, "_nav_align_mode", "align")) != "advance":
            return False
        if err_abs > float(NAV_ALIGN_ENTER_ADVANCE_DEG):
            return False
        if bool(NAV_STRICT_HEADING_FIRST) and err_abs > float(NAV_ALIGN_HEADING_DONE_DEG):
            return False
        stable_since = getattr(self, "_nav_align_stable_since", None)
        if stable_since is not None:
            return False
        return True

    def _nav_advance_stable_elapsed_s(self):
        stable_since = getattr(self, "_nav_align_stable_since", None)
        if stable_since is None:
            return 0.0
        return max(0.0, time.monotonic() - float(stable_since))

    def _apply_nav_speed_governor(
        self,
        speed_cmd,
        heading_err_deg,
        blocked_level,
        dist_m,
        align_follow=False,
        lidar_emergency=False,
    ):
        speed = max(0.0, float(speed_cmd))
        err_abs = abs(float(heading_err_deg))
        blocked = clamp(float(blocked_level), 0.0, 1.0)
        reason = "nominal"

        if bool(NAV_STRICT_HEADING_FIRST) and str(getattr(self, "_nav_align_mode", "align")) == "align":
            self._final_speed_limiter = "heading_align_hold"
            return 0.0, "heading_align_hold"

        if bool(NAV_STRICT_HEADING_FIRST) and err_abs > float(NAV_ALIGN_HEADING_DONE_DEG):
            self._final_speed_limiter = "heading_align_hold"
            return 0.0, "heading_align_hold"

        if bool(lidar_emergency) and blocked >= 0.85:
            self._final_speed_limiter = "critical_block_stop"
            return 0.0, "critical_block_stop"

        if err_abs <= 8.0:
            heading_factor = 1.0
            reason = "heading_full"
        elif err_abs <= 20.0:
            heading_factor = 0.7
            reason = "heading_reduced"
        else:
            heading_factor = 0.45
            reason = "turn_priority"
        speed = speed * heading_factor
        speed = speed * (1.0 - (0.5 * blocked))

        if float(dist_m) < 1.5:
            speed = min(speed, 0.45)
            reason = "near_wp_cap"
        if align_follow and err_abs > 20.0:
            speed = min(speed, 0.35)
            reason = "align_turn_cap"

        allow_min_progress = (
            not bool(align_follow)
            and err_abs <= float(NAV_ALIGN_HEADING_DONE_DEG)
            and str(getattr(self, "_nav_align_mode", "align")) == "advance"
        )
        if (
            allow_min_progress
            and speed_cmd > 0.0
            and not lidar_emergency
            and blocked < 0.85
            and dist_m > 1.2
            and float(NAV_MIN_PROGRESS_SPEED_MPS) > 0.0
        ):
            speed = max(speed, float(NAV_MIN_PROGRESS_SPEED_MPS))
            if reason in ("turn_priority", "heading_reduced", "heading_full"):
                reason = f"{reason}_min_progress"

        speed = max(0.0, speed)
        self._final_speed_limiter = str(reason)
        return float(speed), str(reason)

    def _lidar_sector_snapshot(self):
        now = time.monotonic()
        snapshot = {}
        ages = {}
        for name, dist in (
            ("left", self.lidar_left_dist),
            ("center", self.lidar_center_dist),
            ("right", self.lidar_right_dist),
        ):
            ts = float(self._lidar_sector_ts.get(name, 0.0) or 0.0)
            age_s = None if ts <= 0.0 else max(0.0, now - ts)
            ages[name] = None if age_s is None else round(age_s, 3)
            known = bool(age_s is not None and age_s <= 0.62 and float(dist) < 98.5)
            snapshot[name] = {"distance_m": float(dist), "age_s": age_s, "known": known}
        self.lidar_sector_ages = ages
        return snapshot

    def _extract_lidar_sector_mins_raw(self, msg):
        left = 99.0
        center = 99.0
        right = 99.0
        valid_hits = 0
        try:
            angle = float(msg.angle_min)
            inc = float(msg.angle_increment)
            raw_ranges = getattr(msg, "ranges", []) or []
        except Exception:
            return left, center, right, 0
        for raw in raw_ranges:
            try:
                r = float(raw)
            except (TypeError, ValueError):
                angle += inc
                continue
            if not (math.isfinite(r) and LIDAR_SELF_FILTER_MIN_M < r < LIDAR_VALID_MAX_M):
                angle += inc
                continue
            deg = math.degrees(angle)
            angle += inc
            if deg < -90.0 or deg > 90.0:
                continue
            valid_hits += 1
            sector = classify_lidar_scan_sector(deg)
            if sector == "left":
                left = min(left, r)
            elif sector == "right":
                right = min(right, r)
            elif sector == "center":
                center = min(center, r)
        return left, center, right, int(valid_hits)

    def _update_lidar_degraded_guard(self):
        if str(self._lidar_drop_reason) != "stale_unreliable_clock":
            self._lidar_degraded_since = None
            self._lidar_degraded_mode = "normal"
            self._lidar_degraded_age_s = 0.0
            return "normal"
        if self.simulation_mode:
            # In sim, unreliable header clock alone must not force HOLD.
            self._lidar_degraded_since = None
            self._lidar_degraded_mode = "limited"
            self._lidar_degraded_age_s = 0.0
            return "limited"
        now = time.monotonic()
        if self._lidar_degraded_since is None:
            self._lidar_degraded_since = now
        age_s = max(0.0, now - float(self._lidar_degraded_since))
        self._lidar_degraded_age_s = age_s
        if age_s >= float(LIDAR_DEGRADED_HOLD_TIMEOUT_S):
            self._lidar_degraded_mode = "hold"
            return "hold"
        self._lidar_degraded_mode = "limited"
        return "limited"

    def _select_escape_side(self, left_known, left_d, right_known, right_d):
        if self._escape_lock_side in ("left", "right"):
            return self._escape_lock_side
        if left_known and right_known:
            side = "right" if right_d >= left_d else "left"
        elif left_known:
            side = "right"
        elif right_known:
            side = "left"
        else:
            side = "none"
        if side in ("left", "right"):
            self._escape_lock_side = side
        return side

    def _p2_lidar_escape_heading_sectors_fallback(self):
        """Scan ornegi yoksa sol/orta/sag sektor min mesafeleriyle kacinma."""
        sector = self._lidar_sector_snapshot()
        ld = sector["left"]["distance_m"] if sector["left"]["known"] else 99.0
        cd = sector["center"]["distance_m"] if sector["center"]["known"] else 99.0
        rd = sector["right"]["distance_m"] if sector["right"]["known"] else 99.0
        fm = min(ld, cd, rd)
        if fm >= P2_LIDAR_WARN_M:
            return 0.0
        urgency = clamp(
            (P2_LIDAR_WARN_M - fm) / max(P2_LIDAR_WARN_M - 0.15, 0.01),
            0.0,
            1.0,
        )
        side = self._select_escape_side(sector["left"]["known"], ld, sector["right"]["known"], rd)
        if side == "left":
            return -P2_ESCAPE_MAX_DEG * urgency
        return P2_ESCAPE_MAX_DEG * urgency

    @staticmethod
    def _allow_simple_avoidance_bias(nav_turn_priority, lidar_emergency, camera_lidar_fused=False, acquiring_heading=False):
        """Warn-level simple avoidance must not override waypoint bearing during turn acquisition."""
        if bool(lidar_emergency) or bool(camera_lidar_fused):
            return True
        return not (bool(nav_turn_priority) and bool(acquiring_heading))

    @staticmethod
    def _resolve_nav_align_pivot_speed(nav_turn_priority, heading_error_deg, front_min_m, speed_mps):
        """Prefer yaw-only pivot on large turns; optional creep only when configured."""
        large_turn_near_obstacle = (
            abs(float(heading_error_deg)) >= float(NAV_TURN_SPEED_CAP_HARD_ERR_DEG)
            and float(front_min_m) < float(P2_LIDAR_WARN_M)
        )
        creep_cap = max(0.0, float(NAV_ALIGN_CREEP_SPEED_MPS))
        if large_turn_near_obstacle or not bool(nav_turn_priority):
            return 0.0, "nav_align_pivot_hold"
        if creep_cap <= 0.0:
            return 0.0, "nav_align_yaw_only"
        return min(float(speed_mps), creep_cap), "nav_align_turn_creep"

    @staticmethod
    def _allow_local_minima_boost(nav_turn_priority, heading_error_deg, nav_align_mode="advance"):
        """Do not escalate to 65deg local-minima escape while acquiring waypoint heading."""
        if bool(nav_turn_priority):
            return False
        if str(nav_align_mode or "align") == "align":
            return False
        if bool(NAV_STRICT_HEADING_FIRST) and abs(float(heading_error_deg)) > float(NAV_ALIGN_HEADING_DONE_DEG):
            return False
        if abs(float(heading_error_deg)) >= float(NAV_ALIGN_REACQUIRE_DEG):
            return False
        return True

    def _p2_lidar_escape_heading_deg(self, heading_error_deg=None, nav_turn_priority=False):
        """En yakin on engelden kacinma (pozitif = saga don)."""
        samples = self._lidar_front_samples
        if not samples:
            return self._p2_lidar_escape_heading_sectors_fallback()
        min_r = 99.0
        min_deg = 0.0
        for deg, rng in samples:
            if -92 <= deg <= 92 and rng > 0.08 and rng < 20.0:
                if rng < min_r:
                    min_r = rng
                    min_deg = deg
        after_sample_min_r = float(min_r)
        raw_sec = float(getattr(self, "_lidar_raw_sector_min_m", 99.0) or 99.0)
        min_r = min(float(min_r), raw_sec)
        if raw_sec <= float(P2_LIDAR_WARN_M) and after_sample_min_r > float(P2_LIDAR_WARN_M):
            min_deg = 0.0
        if min_r > P2_LIDAR_WARN_M:
            self._p2_local_minima_start_ts = None
            self._p2_local_minima_active = False
            return self._p2_lidar_escape_heading_sectors_fallback()
        
        urgency = clamp(
            (P2_LIDAR_WARN_M - min_r) / max(P2_LIDAR_WARN_M - 0.15, 0.01),
            0.0,
            1.0,
        )
        
        try:
            heading_err = float(
                heading_error_deg
                if heading_error_deg is not None
                else getattr(self, "nav_heading_error_deg", 0.0)
            )
        except (TypeError, ValueError):
            heading_err = 0.0
        allow_local_minima_boost = self._allow_local_minima_boost(nav_turn_priority, heading_err, self._nav_align_mode)

        now = time.monotonic()
        if min_r < D_MIN_M:
            if self._p2_local_minima_start_ts is None:
                self._p2_local_minima_start_ts = now
                self._p2_local_minima_active = True
            elif (
                allow_local_minima_boost
                and (now - self._p2_local_minima_start_ts) >= P2_LOCAL_MINIMA_TIMEOUT_S
            ):
                max_deg = P2_ESCAPE_MAX_DEG_LOCAL_MINIMA
                if (now - self._p2_local_minima_start_ts) >= P2_LOCAL_MINIMA_TIMEOUT_S * 2:
                    max_deg = P2_ESCAPE_MAX_DEG_LOCAL_MINIMA * 1.3
                self._fusion_log(
                    "local_minima_escape",
                    f"[P2] local_minima timeout, increasing escape to {max_deg:.0f}deg",
                )
        else:
            self._p2_local_minima_start_ts = None
            self._p2_local_minima_active = False
        
        if (
            allow_local_minima_boost
            and self._p2_local_minima_active
            and self._p2_local_minima_start_ts is not None
        ):
            elapsed = now - self._p2_local_minima_start_ts
            if elapsed >= P2_LOCAL_MINIMA_TIMEOUT_S:
                max_escape = P2_ESCAPE_MAX_DEG_LOCAL_MINIMA
                if elapsed >= P2_LOCAL_MINIMA_TIMEOUT_S * 2:
                    max_escape = P2_ESCAPE_MAX_DEG_LOCAL_MINIMA * 1.3
                escape_dir = 1.0
                if abs(min_deg) < 14.0:
                    ld = self.lidar_left_dist
                    rd = self.lidar_right_dist
                    if ld >= 90.0 and rd >= 90.0:
                        escape_dir = 1.0
                    elif abs(ld - rd) >= 0.05:
                        escape_dir = 1.0 if rd > ld else -1.0
                else:
                    escape_dir = clamp(min_deg / 45.0, -1.0, 1.0)
                return max_escape * urgency * escape_dir
        
        if abs(min_deg) < 14.0:
            sector = self._lidar_sector_snapshot()
            ld = sector["left"]["distance_m"] if sector["left"]["known"] else 99.0
            rd = sector["right"]["distance_m"] if sector["right"]["known"] else 99.0
            if ld >= 90.0 and rd >= 90.0:
                return 0.0
            side = self._select_escape_side(sector["left"]["known"], ld, sector["right"]["known"], rd)
            return (-1.0 if side == "left" else 1.0) * P2_ESCAPE_MAX_DEG * urgency
        return P2_ESCAPE_MAX_DEG * urgency * clamp(min_deg / 45.0, -1.0, 1.0)

    def _p2_escape_boost_deg(self, esc_deg: float, threat_m: float) -> float:
        """Çok yakın engelde kaçınma sapmasını artır (dubanın etrafından dönüş)."""
        m = float(threat_m)
        if m >= 90.0 or not math.isfinite(m):
            return float(esc_deg)
        t = clamp(
            (float(P2_LIDAR_WARN_M) - m) / max(float(P2_LIDAR_WARN_M) - float(D_MIN_M), 0.08),
            0.0,
            1.0,
        )
        return float(esc_deg) * (1.0 + 0.28 * t * t)

    def _p2_camera_orange_escape_deg(self):
        """Turuncu kenar dubası — görüntüde merkeze en yakın TURUNCU_SINIR'dan sapma."""
        raw_det = bool(self.camera_status.get("orange_boundary_detected_raw"))
        gated_det = bool(self.camera_status.get("orange_boundary_detected"))
        if not raw_det and not gated_det:
            self._orange_seen_since = None
            return 0.0
        try:
            if gated_det:
                area = float(self.camera_status.get("orange_boundary_area_norm", 0.0))
                bear = float(self.camera_status.get("orange_boundary_bearing_deg", 0.0))
            else:
                area = float(self.camera_status.get("orange_boundary_area_norm_raw", 0.0))
                bear = float(self.camera_status.get("orange_boundary_bearing_deg_raw", 0.0))
        except (TypeError, ValueError):
            self._orange_seen_since = None
            return 0.0
        now = time.monotonic()
        if self._orange_seen_since is None:
            self._orange_seen_since = now
        min_area = 0.006 if gated_det else 0.010
        if area < min_area or not math.isfinite(bear):
            self._orange_seen_since = None
            return 0.0
        if not self._camera_obstacle_confirmed(
            bear,
            area,
            self._orange_seen_since,
            min_dwell_s=0.35,
            strong_area_norm=0.018,
        ):
            return 0.0
        gain = clamp(area * 5.5, 0.12, 0.95)
        return -bear * gain * (P2_ESCAPE_MAX_DEG / 40.0)

    def _p2_camera_yellow_escape_deg(self):
        """Sari engel goruntu merkezinden sapma; kacinma -bearing ile."""
        raw_det = bool(self.camera_status.get("yellow_obstacle_detected_raw"))
        gated_det = bool(self.camera_status.get("yellow_obstacle_detected"))
        if not raw_det and not gated_det:
            self._yellow_seen_since = None
            return 0.0
        try:
            if gated_det:
                area = float(self.camera_status.get("yellow_obstacle_area_norm", 0.0))
                bear = float(self.camera_status.get("yellow_obstacle_bearing_deg", 0.0))
            else:
                area = float(self.camera_status.get("yellow_obstacle_area_norm_raw", 0.0))
                bear = float(self.camera_status.get("yellow_obstacle_bearing_deg_raw", 0.0))
        except (TypeError, ValueError):
            self._yellow_seen_since = None
            return 0.0
        now = time.monotonic()
        if self._yellow_seen_since is None:
            self._yellow_seen_since = now
        min_area = 0.006 if gated_det else 0.010
        if area < min_area or not math.isfinite(bear):
            self._yellow_seen_since = None
            return 0.0
        if not self._camera_obstacle_confirmed(
            bear,
            area,
            self._yellow_seen_since,
            min_dwell_s=0.30,
            strong_area_norm=0.016,
        ):
            return 0.0
        gain = clamp(area * 6.0, 0.15, 1.0)
        return -bear * gain * (P2_ESCAPE_MAX_DEG / 40.0)

    def _p1_avoid_heading_smoothed(self, esc_l, esc_c, in_warn, lidar_emergency):
        """P1: lidar+kamera kaçınma komutunu sürekli sıçramayı azaltmak için yumuşat."""
        avoid_raw = esc_l + P2_CAM_YELLOW_WEIGHT * esc_c
        if in_warn or lidar_emergency:
            beta = 0.48
            self._p1_avoid_smooth = beta * avoid_raw + (1.0 - beta) * self._p1_avoid_smooth
            return self._p1_avoid_smooth
        self._p1_avoid_smooth = 0.0
        return 0.0

    def _p2_avoid_heading_smoothed(
        self,
        esc_l,
        esc_c,
        in_warn,
        lidar_emergency,
        cam_weight=None,
        esc_orange_deg=0.0,
    ):
        """P2: lidar + sarı + turuncu kenar kaçınmasını yumuşat."""
        w = float(P2_CAM_YELLOW_WEIGHT if cam_weight is None else cam_weight)
        eo = float(esc_orange_deg or 0.0)
        avoid_raw = esc_l + w * esc_c + float(P2_ORANGE_BOUNDARY_WEIGHT) * eo
        # in_warn disinda da ham kaçınma varsa yumuşat (log: esc_o/esc_c varken avoid=0 oluyordu)
        need_avoid = bool(
            in_warn or lidar_emergency or abs(float(avoid_raw)) >= 0.22
        )
        if need_avoid:
            beta = 0.36
            self._p2_avoid_smooth = beta * avoid_raw + (1.0 - beta) * self._p2_avoid_smooth
            return self._p2_avoid_smooth
        self._p2_avoid_smooth = 0.0
        return 0.0

    def _nav_cross_track_correction_deg(self, signed_ct_m: float, dist_m: float) -> float:
        """Stanley-style lateral correction toward the frozen leg (ENU signed cross-track, m)."""
        la = clamp(
            0.55 * float(dist_m) + 2.0,
            float(NAV_CROSS_TRACK_L1_MIN_M),
            float(NAV_CROSS_TRACK_L1_MAX_M),
        )
        raw = math.degrees(
            math.atan2(float(NAV_CROSS_TRACK_K) * float(signed_ct_m), max(la, 0.5))
        )
        return clamp(raw, -float(NAV_CROSS_TRACK_CORR_CAP_DEG), float(NAV_CROSS_TRACK_CORR_CAP_DEG))

    @staticmethod
    def _apply_cross_track_to_heading(
        gps_heading_err,
        signed_ct_m,
        ct_corr_deg,
        *,
        align_follow,
        in_warn,
        lidar_emergency,
    ):
        """Blend capped cross-track correction during align/large-turn when laterally offset."""
        if in_warn or lidar_emergency:
            return float(gps_heading_err), 0.0, False
        if abs(float(signed_ct_m)) < float(NAV_CROSS_TRACK_ALIGN_MIN_M):
            return float(gps_heading_err), 0.0, False

        heading_err = float(gps_heading_err)
        if not align_follow and abs(heading_err) < 35.0:
            blend = float(ct_corr_deg)
            return normalize_heading_error(heading_err + blend), blend, False

        align_cap = float(NAV_CROSS_TRACK_ALIGN_CAP_DEG)
        blend = clamp(float(ct_corr_deg), -align_cap, align_cap)
        return normalize_heading_error(heading_err + blend), blend, True

    def _apply_avoid_bias_slew(self, target_bias_deg, emergency=False):
        """Limit frame-to-frame avoidance bias jumps to reduce zig-zag on sparse scans."""
        try:
            target = float(target_bias_deg)
        except (TypeError, ValueError):
            target = 0.0
        target = clamp(target, -95.0, 95.0)
        now = time.monotonic()
        if emergency:
            self._avoid_bias_filtered_deg = target
            self._avoid_bias_last_ts = now
            return float(target)
        dt = max(0.01, min(0.30, now - float(self._avoid_bias_last_ts or now)))
        max_step = max(5.0, float(NAV_AVOID_BIAS_SLEW_DEG_PER_S) * dt)
        prev = float(self._avoid_bias_filtered_deg or 0.0)
        stepped = prev + clamp(target - prev, -max_step, max_step)
        self._avoid_bias_filtered_deg = float(stepped)
        self._avoid_bias_last_ts = now
        return float(stepped)

    def _apply_heading_damping(self, heading_error_deg):
        """Heading inner-loop damping using yaw-rate feedback + command slew limiting."""
        try:
            heading_err = float(heading_error_deg)
        except (TypeError, ValueError):
            heading_err = 0.0
        heading_err = clamp_heading_error(heading_err)
        now = time.monotonic()
        if not NAV_HEADING_DAMPING_ENABLED:
            self._heading_cmd_filtered_deg = float(heading_err)
            self._heading_cmd_last_ts = now
            self._heading_damping_hold_active = False
            return float(heading_err)

        dt = max(0.01, min(0.30, now - float(self._heading_cmd_last_ts or now)))
        try:
            yaw_rate_dps = float(self.current_yaw_rate_dps)
        except (TypeError, ValueError):
            yaw_rate_dps = 0.0
        raw_cmd = clamp_heading_error(heading_err - (float(NAV_HEADING_DAMPING_YAWRATE_GAIN) * yaw_rate_dps))
        sign_reversal = bool(
            abs(heading_err) > max(float(HEADING_DEADZONE_DEG), float(NAV_ALIGN_HEADING_DONE_DEG))
            and abs(heading_err) <= 45.0
            and raw_cmd != 0.0
            and (heading_err * raw_cmd) < 0.0
        )
        if sign_reversal:
            raw_cmd = 0.0
            self._heading_damping_hold_active = True
            self._heading_damping_hold_count += 1
            self.heading_control_diagnostic = "heading_damping_hold:sign_reversal"
            if (now - float(self._heading_damping_last_log_ts or 0.0)) >= 0.5:
                print(
                    "[HEADING_DAMPING] hold err={:.1f} yaw_rate={:.2f} count={} diagnostic=sign_reversal".format(
                        heading_err,
                        yaw_rate_dps,
                        int(self._heading_damping_hold_count),
                    )
                )
                log_jsonl(
                    "usv_main",
                    False,
                    event="heading_damping_hold",
                    heading_error_deg=round(float(heading_err), 3),
                    yaw_rate_dps=round(float(yaw_rate_dps), 3),
                    count=int(self._heading_damping_hold_count),
                    reason="sign_reversal",
                )
                self._heading_damping_last_log_ts = now
        else:
            self._heading_damping_hold_active = False
            if str(self.heading_control_diagnostic).startswith("heading_damping_hold"):
                self.heading_control_diagnostic = "nominal"
        prev_cmd = float(self._heading_cmd_filtered_deg or 0.0)
        max_step = max(5.0, float(NAV_HEADING_CMD_SLEW_DEG_PER_S) * dt)
        slew_cmd = prev_cmd + clamp(raw_cmd - prev_cmd, -max_step, max_step)
        alpha = clamp(float(NAV_HEADING_DAMPING_LPF_ALPHA), 0.01, 1.0)
        filtered_cmd = prev_cmd + (alpha * (slew_cmd - prev_cmd))
        filtered_cmd = clamp_heading_error(filtered_cmd)
        acquiring = str(getattr(self, "_nav_align_mode", "advance")) == "align"
        if acquiring and abs(heading_err) > float(NAV_ALIGN_HEADING_DONE_DEG):
            min_ratio = 0.65
            if abs(filtered_cmd) < abs(heading_err) * min_ratio:
                filtered_cmd = clamp_heading_error(float(heading_err) * min_ratio)
        if (
            abs(heading_err) > max(float(HEADING_DEADZONE_DEG), float(NAV_ALIGN_HEADING_DONE_DEG))
            and abs(heading_err) <= 45.0
            and filtered_cmd != 0.0
            and (heading_err * filtered_cmd) < 0.0
        ):
            filtered_cmd = 0.0
            self._heading_damping_hold_active = True
            self.heading_control_diagnostic = "heading_damping_hold:filtered_sign_reversal"
        self._heading_cmd_filtered_deg = float(filtered_cmd)
        self._heading_cmd_last_ts = now
        return float(filtered_cmd)

    def _apply_turn_priority_speed_caps(self, speed_mps, heading_error_deg, distance_m):
        """Prefer turning over forward surge on large heading error and near-waypoint turns."""
        speed = max(0.0, float(speed_mps))
        heading_abs = abs(float(heading_error_deg))
        if bool(NAV_STRICT_HEADING_FIRST) and heading_abs > float(NAV_ALIGN_HEADING_DONE_DEG):
            return 0.0
        if heading_abs >= float(NAV_TURN_SPEED_CAP_HARD_ERR_DEG):
            speed = min(speed, float(NAV_TURN_SPEED_CAP_HARD_MPS))
        elif heading_abs >= float(NAV_TURN_SPEED_CAP_MEDIUM_ERR_DEG):
            speed = min(speed, float(NAV_TURN_SPEED_CAP_MEDIUM_MPS))
        elif heading_abs >= float(NAV_TURN_SPEED_CAP_SOFT_ERR_DEG):
            speed = min(speed, float(NAV_TURN_SPEED_CAP_SOFT_MPS))
        if (
            float(distance_m) <= float(NAV_NEAR_WP_TURN_SPEED_CAP_DIST_M)
            and heading_abs >= float(NAV_NEAR_WP_TURN_ERR_DEG)
        ):
            speed = min(speed, float(NAV_NEAR_WP_TURN_SPEED_CAP_MPS))
        return float(speed)

    def _mix_rc_steer_throttle_to_twin_pwm(self, steer_pwm, throttle_pwm):
        """Map manual RC steer/throttle semantics to left/right twin-thruster PWM."""
        try:
            steer = int(float(steer_pwm))
        except (TypeError, ValueError):
            steer = 1500
        try:
            throttle = int(float(throttle_pwm))
        except (TypeError, ValueError):
            throttle = 1500
        steer = int(clamp(steer, 1100, 1900))
        throttle = int(clamp(throttle, 1100, 1900))
        throttle_norm = (float(throttle) - 1500.0) / 400.0
        steer_norm = (float(steer) - 1500.0) / 400.0
        if abs(throttle_norm) < 0.04:
            throttle_norm = 0.0
        if abs(steer_norm) < 0.04:
            steer_norm = 0.0
        left_mix = throttle_norm + steer_norm
        right_mix = throttle_norm - steer_norm
        max_mag = max(abs(left_mix), abs(right_mix), 1.0)
        if max_mag > 1.0:
            left_mix /= max_mag
            right_mix /= max_mag
        left_pwm = int(round(1500 + (left_mix * 400.0)))
        right_pwm = int(round(1500 + (right_mix * 400.0)))
        left_pwm = int(clamp(left_pwm, 1100, 1900))
        right_pwm = int(clamp(right_pwm, 1100, 1900))
        if abs(left_pwm - 1500) < 8:
            left_pwm = 1500
        if abs(right_pwm - 1500) < 8:
            right_pwm = 1500
        return left_pwm, right_pwm

    def _navigate_p2_waypoint(self, lat, lon, leg_start=None):
        self._sim_dist = 12.0
        self._reset_nav_no_motion_monitor()
        if os.environ.get("USV_SIM") == "1":
            start_snapshot = self._wait_for_stable_nav_solution(samples_required=3, timeout_s=1.0)
        else:
            start_snapshot = self._wait_for_stable_nav_solution(samples_required=5, timeout_s=3.0)
        self._configure_waypoint_leg(lat, lon, leg_start=leg_start, start_snapshot=start_snapshot)
        log_jsonl(
            "usv_main",
            False,
            event="waypoint_dispatch",
            waypoint_info=str(self._wp_info),
            target_lat=float(lat),
            target_lon=float(lon),
            acceptance_radius_m=round(float(R_WP_M), 3),
            hold_required_s=round(float(T_HOLD_S), 3),
            leg_start_lat=None if leg_start is None else float(leg_start[0]),
            leg_start_lon=None if leg_start is None else float(leg_start[1]),
            vehicle_mode=str(getattr(self, "vehicle_mode_name", "") or ""),
            armed=bool(self.vehicle_armed),
        )
        self._tracking_loss_since = None
        self._escape_lock_side = None
        self._escape_clear_since = None
        self._p2_warn_latch = False
        self._front_min_smooth = 99.0
        self._blocked_level = 0.0
        self._wrong_turn_since = None
        self._wrong_turn_active = False
        self._p2_local_minima_start_ts = None
        self._p2_local_minima_active = False
        self._avoid_bias_filtered_deg = 0.0
        self._avoid_bias_last_ts = time.monotonic()
        self._heading_cmd_filtered_deg = 0.0
        self._heading_cmd_last_ts = time.monotonic()
        self._heading_damping_hold_active = False
        self._wp_transition_until = time.monotonic() + float(NAV_ALIGN_TURN_IMMUNITY_S)
        self._nav_align_lock_until = time.monotonic() + float(NAV_ALIGN_TURN_IMMUNITY_S)
        while self.mission_active:
            if self._check_abort():
                return False
            self._track_gate_event()
            prev_best_distance = float(self._closest_waypoint_distance_seen)
            dist, gps_heading_err, nav_snapshot = self._distance_and_heading_error(lat, lon)
            self._wp_target = f"{lat:.7f}, {lon:.7f}"
            if not self.nav_fix_valid:
                self._command_speed_heading(0.0, 0.0)
                self._write_state()
                time.sleep(LOOP_DT)
                continue
            if not self._wp_leg_valid:
                self._configure_waypoint_leg(lat, lon, leg_start=leg_start, start_snapshot=nav_snapshot)
            accepted, hold_active = self._evaluate_waypoint_acceptance(dist, nav_snapshot)
            if hold_active:
                self._command_speed_heading(0.0, 0.0)
                if accepted:
                    print(
                        f"[OK] [NAV] WP_REACHED dist={dist:.2f}m hold={T_HOLD_S:.1f}s "
                        f"reason={self.waypoint_accept_reason} progress={self.progress_along_leg_m:.2f}m"
                    )
                    self._hold_waypoint_neutral("waypoint_reached")
                    log_jsonl(
                        "usv_main",
                        False,
                        event="waypoint_reached",
                        waypoint_info=str(self._wp_info),
                        target_lat=float(lat),
                        target_lon=float(lon),
                        distance_m=round(float(dist), 3),
                        acceptance_radius_m=round(float(R_WP_M), 3),
                        hold_required_s=round(float(T_HOLD_S), 3),
                        reason=str(self.waypoint_accept_reason),
                        progress_m=round(float(self.progress_along_leg_m), 3),
                        armed=bool(self.vehicle_armed),
                    )
                    return True
                self._write_state()
                time.sleep(LOOP_DT)
                continue

            sectors = self._lidar_sector_snapshot()
            left_known = bool(sectors["left"]["known"])
            center_known = bool(sectors["center"]["known"])
            right_known = bool(sectors["right"]["known"])
            left_d = sectors["left"]["distance_m"] if left_known else 99.0
            center_d = sectors["center"]["distance_m"] if center_known else 99.0
            right_d = sectors["right"]["distance_m"] if right_known else 99.0
            lidar_quality_ok = bool(self._lidar_quality_ready and self._lidar_stable_point_count > 0)
            current_left_d = float(getattr(self, "_lidar_current_left_m", 99.0) or 99.0)
            current_center_d = float(getattr(self, "_lidar_current_center_m", 99.0) or 99.0)
            current_right_d = float(getattr(self, "_lidar_current_right_m", 99.0) or 99.0)
            current_left_known = bool(lidar_quality_ok and current_left_d < 98.5)
            current_center_known = bool(lidar_quality_ok and current_center_d < 98.5)
            current_right_known = bool(lidar_quality_ok and current_right_d < 98.5)
            current_front_min = min(
                current_left_d if current_left_known else 99.0,
                current_center_d if current_center_known else 99.0,
                current_right_d if current_right_known else 99.0,
            )
            front_min = min(left_d, center_d, right_d)
            map_metrics = self._compute_map_sector_metrics()
            if lidar_quality_ok:
                front_min = min(front_min, float(map_metrics.get("front_min_m", 99.0)))
                left_d = min(left_d, float(map_metrics.get("left_min_m", 99.0)))
                right_d = min(right_d, float(map_metrics.get("right_min_m", 99.0)))
                self._send_obstacle_distance_from_lidar_if_due(left_d, front_min, right_d)
            front_risk = float(map_metrics.get("front_risk", 0.0))
            left_risk = float(map_metrics.get("left_risk", 0.0))
            right_risk = float(map_metrics.get("right_risk", 0.0))
            try:
                prev_smooth = float(getattr(self, "_front_min_smooth", 99.0) or 99.0)
            except (TypeError, ValueError):
                prev_smooth = 99.0
            self._front_min_smooth = (0.34 * float(front_min)) + (0.66 * prev_smooth)
            front_min_nav = float(self._front_min_smooth)
            wp_transition_s_early = max(0.0, float(getattr(self, "_wp_transition_until", 0.0) or 0.0) - time.monotonic())
            heading_not_aligned = abs(float(gps_heading_err)) >= float(NAV_ALIGN_HEADING_DONE_DEG)
            acquiring_heading = bool(self._nav_align_mode == "align")
            nav_turn_priority = bool(
                heading_not_aligned
                or (wp_transition_s_early > 0.0 and acquiring_heading)
            )
            lidar_guard_mode = self._update_lidar_degraded_guard()
            if lidar_guard_mode == "hold":
                log_jsonl(
                    "usv_main",
                    False,
                    event="lidar_invalid_timeout",
                    drop_reason=str(self._lidar_drop_reason),
                    degraded_age_s=round(float(self._lidar_degraded_age_s), 3),
                    timeout_s=float(LIDAR_DEGRADED_HOLD_TIMEOUT_S),
                )
                self._enter_hold("LIDAR_INVALID_TIMEOUT")
                return False
            # Yakın bacakta lidar "advance" zorlaması: önce burun WP'ye; yoksa cross-track sınır dubaları arasından iter
            h_abs_pre = abs(float(gps_heading_err))
            align_resume_blocked = False
            if acquiring_heading and float(front_min) < float(NAV_ALIGN_SUSPEND_NEAR_M):
                tight_m = float(front_min) < float(NAV_ALIGN_TIGHT_OBSTACLE_M)
                near_wp = float(dist) <= float(NAV_ALIGN_LIDAR_SUSPEND_MAX_DIST_M)
                head_ok = h_abs_pre <= float(NAV_ALIGN_HEADING_DONE_DEG)
                if (tight_m or near_wp) and not head_ok:
                    align_resume_blocked = True
            map_threat_allowed = bool(lidar_quality_ok and self._lidar_occ_cells_count > 0)
            map_front_min = float(map_metrics.get("front_min_m", 99.0)) if map_threat_allowed else 99.0
            lidar_emergency = bool(lidar_quality_ok and current_center_known and current_center_d < D_MIN_M)
            current_lidar_warning = bool(lidar_quality_ok and current_front_min < P2_LIDAR_WARN_M)
            if nav_turn_priority and acquiring_heading and not current_lidar_warning and not lidar_emergency:
                # During heading acquisition, do not let persistent map residue
                # or smoothed historical front distance erase the waypoint bearing.
                front_min_nav = max(float(front_min_nav), float(P2_LIDAR_WARN_M) + 0.65)
            raw_in_warn = bool(
                lidar_quality_ok
                and (
                    (current_center_known and current_center_d < P2_LIDAR_WARN_M)
                    or (current_front_min < P2_LIDAR_WARN_M)
                    or (front_min_nav < P2_LIDAR_WARN_M)
                )
            )
            if not lidar_quality_ok:
                self._p2_warn_latch = False
            elif raw_in_warn:
                self._p2_warn_latch = True
            else:
                clear_thr = float(P2_LIDAR_WARN_M) + float(P2_LIDAR_WARN_EXIT_MARGIN_M)
                if front_min_nav > clear_thr and (not center_known or center_d > clear_thr):
                    self._p2_warn_latch = False
            blocked_by_map = bool(map_threat_allowed and self._update_blocked_hysteresis(front_risk))
            if not map_threat_allowed:
                self._update_blocked_hysteresis(0.0)
                blocked_by_map = False
            in_warn = bool(self._p2_warn_latch or blocked_by_map)
            if nav_turn_priority and acquiring_heading and not current_lidar_warning and not lidar_emergency:
                in_warn = False
                blocked_by_map = False

            if front_min_nav > (P2_LIDAR_WARN_M + 0.4):
                if self._escape_clear_since is None:
                    self._escape_clear_since = time.monotonic()
                elif (time.monotonic() - self._escape_clear_since) >= 0.5:
                    self._escape_lock_side = None
            else:
                self._escape_clear_since = None

            unknown_escape = bool(lidar_quality_ok and center_known and center_d < P2_LIDAR_WARN_M and not left_known and not right_known)
            simple_avoidance = decide_three_sector_avoidance(
                left_m=current_left_d if current_left_known else None,
                front_m=current_center_d if current_center_known else current_front_min,
                right_m=current_right_d if current_right_known else None,
                stop_distance_m=D_MIN_M,
                warn_distance_m=P2_LIDAR_WARN_M,
                clear_hysteresis_m=P2_LIDAR_WARN_EXIT_MARGIN_M,
                previous_escape_side=self._escape_lock_side,
                failsafe_slow_mps=FAILSAFE_SLOW_MPS,
                max_yaw_bias_deg=P2_ESCAPE_MAX_DEG,
            )
            if simple_avoidance.active:
                self._escape_lock_side = str(simple_avoidance.escape_side)

            h_abs = abs(float(gps_heading_err))
            t_nav = time.monotonic()
            if self._nav_align_t0 is None:
                self._nav_align_t0 = t_nav
            if not in_warn and not lidar_emergency and not align_resume_blocked:
                if self._nav_align_mode == "align":
                    if h_abs <= float(NAV_ALIGN_ENTER_ADVANCE_DEG):
                        if self._nav_align_stable_since is None:
                            self._nav_align_stable_since = t_nav
                        elif (t_nav - float(self._nav_align_stable_since)) >= float(NAV_ALIGN_STABLE_S):
                            self._nav_align_mode = "advance"
                            self._nav_align_stable_since = None
                    else:
                        self._nav_align_stable_since = None
                    if (
                        self._nav_align_mode == "align"
                        and not bool(NAV_STRICT_HEADING_FIRST)
                        and (t_nav - float(self._nav_align_t0)) >= float(NAV_ALIGN_TIMEOUT_S)
                    ):
                        if h_abs <= float(NAV_ALIGN_REACQUIRE_DEG) and float(front_min) >= float(NAV_ALIGN_TIGHT_OBSTACLE_M):
                            self._nav_align_mode = "advance"
                            self._nav_align_stable_since = None
                        elif (
                            os.environ.get("USV_SIM") == "1"
                            and h_abs <= float(NAV_ALIGN_TIMEOUT_LARGE_ERR_DEG)
                            and float(front_min) >= float(NAV_ALIGN_TIGHT_OBSTACLE_M)
                        ):
                            self._nav_align_mode = "advance"
                            self._nav_align_stable_since = None
                elif (
                    self._nav_align_mode == "advance"
                    and h_abs > float(NAV_ALIGN_REVERT_ALIGN_DEG)
                    and float(getattr(self, "v_target", 0.0) or 0.0) > 0.1
                    and time.monotonic() >= float(getattr(self, "_nav_align_lock_until", 0.0) or 0.0)
                ):
                    self._nav_align_mode = "align"
                    self._nav_align_t0 = t_nav
                    self._nav_align_stable_since = None

            base_speed = schedule_waypoint_speed(
                distance_m=dist,
                cruise_speed_mps=P2_CRUISE_MPS,
                approach_speed_mps=P2_WAIT_SPEED_MPS,
                approach_window_m=6.0,
            )
            if dist <= float(NAV_WP_APPROACH_SPEED_CAP_DIST_M):
                base_speed = min(base_speed, NAV_WP_APPROACH_SPEED_CAP_MPS)

            align_follow = (
                self._nav_align_mode == "align"
                and not align_resume_blocked
                and not in_warn
                and not lidar_emergency
                and not unknown_escape
            )
            if align_follow:
                base_speed = min(base_speed, float(NAV_ALIGN_MAX_SPEED_MPS))

            align_pivot_threshold_deg = max(float(NAV_ALIGN_PIVOT_UNTIL_DEG), float(NAV_ALIGN_ACQUIRE_DEG))
            align_pivot_eligible = align_follow and (
                abs(float(gps_heading_err)) > align_pivot_threshold_deg
            )

            signed_ct_m = 0.0
            ct_corr_deg = 0.0
            cross_track_blend_deg = 0.0
            cross_track_align_active = False
            gps_heading_for_nav = float(gps_heading_err)
            leg_ok = self._wp_leg_valid and float(self._wp_leg_length_m or 0) >= float(NAV_CROSS_TRACK_MIN_LEG_M)
            if leg_ok:
                leg_len = float(self._wp_leg_length_m)
                le, ln = self._latlon_offset_m(
                    self._wp_leg_start_lat,
                    self._wp_leg_start_lon,
                    self._wp_leg_target_lat,
                    self._wp_leg_target_lon,
                )
                _cla = float(nav_snapshot.get("lat", self.current_lat))
                _clo = float(nav_snapshot.get("lon", self.current_lon))
                ce, cn = self._latlon_offset_m(
                    self._wp_leg_start_lat,
                    self._wp_leg_start_lon,
                    _cla,
                    _clo,
                )
                signed_ct_m = (le * cn - ln * ce) / max(leg_len, 0.001)
                ct_corr_deg = self._nav_cross_track_correction_deg(signed_ct_m, dist)
                gps_heading_for_nav, cross_track_blend_deg, cross_track_align_active = (
                    self._apply_cross_track_to_heading(
                        gps_heading_err,
                        signed_ct_m,
                        ct_corr_deg,
                        align_follow=bool(align_follow),
                        in_warn=bool(in_warn),
                        lidar_emergency=bool(lidar_emergency),
                    )
                )
            self.cross_track_error_m = round(float(signed_ct_m), 3)
            self.cross_track_corr_deg = round(float(cross_track_blend_deg), 3)
            self.cross_track_align_active = bool(cross_track_align_active)

            esc_l = (
                self._p2_escape_boost_deg(
                    self._p2_lidar_escape_heading_deg(
                        heading_error_deg=float(gps_heading_err),
                        nav_turn_priority=bool(nav_turn_priority),
                    ),
                    float(front_min),
                )
                if lidar_quality_ok
                else 0.0
            )
            esc_c = self._p2_camera_yellow_escape_deg()
            if not bool(self._get_perception_policy().get("yellow_obstacle", False)):
                esc_c = 0.0
            esc_o = self._p2_camera_orange_escape_deg()
            yellow_raw = bool(self.camera_status.get("yellow_obstacle_detected_raw"))
            orange_raw = bool(self.camera_status.get("orange_boundary_detected_raw"))
            try:
                yellow_bear = float(self.camera_status.get("yellow_obstacle_bearing_deg_raw", 0.0) or 0.0)
            except (TypeError, ValueError):
                yellow_bear = 0.0
            try:
                orange_bear = float(self.camera_status.get("orange_boundary_bearing_deg_raw", 0.0) or 0.0)
            except (TypeError, ValueError):
                orange_bear = 0.0
            yellow_fused = bool(self.camera_status.get("yellow_obstacle_detected")) or self._camera_lidar_obstacle_concurrent(
                raw_detected=yellow_raw,
                bearing_deg=yellow_bear,
            )
            orange_fused = bool(self.camera_status.get("orange_boundary_detected")) or self._camera_lidar_obstacle_concurrent(
                raw_detected=orange_raw,
                bearing_deg=orange_bear,
            )
            camera_lidar_fused = bool(yellow_fused or orange_fused)
            cam_w = float(P2_CAM_YELLOW_WEIGHT)
            if yellow_raw and bool(self.camera_ready) and bool(self.lidar_ready):
                if camera_lidar_fused or raw_in_warn or float(front_min) < float(P2_LIDAR_WARN_M) + 1.0:
                    cam_w = min(1.0, float(P2_CAM_YELLOW_WEIGHT) * float(P2_CAM_YELLOW_FUSION_MULT))
            if nav_turn_priority and acquiring_heading and not current_lidar_warning and not lidar_emergency:
                if not camera_lidar_fused:
                    esc_c = 0.0
                    esc_o = 0.0
                    cam_w = 0.0
                else:
                    esc_c = clamp(float(esc_c), -12.0, 12.0)
                    esc_o = clamp(float(esc_o), -12.0, 12.0)
            elif self._nav_align_mode == "advance":
                esc_c = clamp(float(esc_c), -18.0, 18.0)
                esc_o = clamp(float(esc_o), -18.0, 18.0)
            avoid = self._p2_avoid_heading_smoothed(
                esc_l,
                esc_c,
                in_warn,
                lidar_emergency,
                cam_weight=cam_w,
                esc_orange_deg=esc_o,
            )
            if blocked_by_map and not nav_turn_priority:
                side = self._choose_avoid_side_from_risk(left_risk, right_risk)
                self.escape_side = str(side)
                side_bias = (35.0 if side == "right" else -35.0) * clamp(front_risk, 0.0, 1.0)
                avoid = (0.4 * float(avoid)) + (0.6 * float(side_bias))
            elif nav_turn_priority and acquiring_heading and not camera_lidar_fused:
                avoid = self._apply_avoid_bias_slew(0.0, emergency=False)
            if (
                simple_avoidance.active
                and (abs(float(avoid)) < 1.0 or simple_avoidance.state == "blocked")
                and self._allow_simple_avoidance_bias(
                    nav_turn_priority,
                    lidar_emergency,
                    camera_lidar_fused=bool(camera_lidar_fused),
                    acquiring_heading=bool(acquiring_heading),
                )
            ):
                avoid = float(simple_avoidance.yaw_bias_deg)
                self.escape_side = str(simple_avoidance.escape_side)
            avoid = self._apply_avoid_bias_slew(avoid, emergency=bool(lidar_emergency))
            front_distance_for_decision = float(front_min)
            center_distance_for_decision = float(center_d)
            if not lidar_quality_ok:
                front_distance_for_decision = 99.0
                center_distance_for_decision = 99.0
            if blocked_by_map and front_distance_for_decision > float(P2_LIDAR_WARN_M):
                front_distance_for_decision = float(P2_LIDAR_WARN_M) - 0.05
            wp_transition_s = max(0.0, float(getattr(self, "_wp_transition_until", 0.0) or 0.0) - time.monotonic())
            large_turn = abs(float(gps_heading_err)) >= float(NAV_ALIGN_REACQUIRE_DEG)
            if (
                acquiring_heading
                and (wp_transition_s > 0.0 or large_turn)
                and not current_lidar_warning
                and not lidar_emergency
            ):
                front_distance_for_decision = max(
                    float(front_distance_for_decision),
                    float(P2_LIDAR_WARN_M) + 0.65,
                )
                center_distance_for_decision = max(
                    float(center_distance_for_decision),
                    float(P2_LIDAR_WARN_M) + 0.65,
                )
            if not (nav_turn_priority and acquiring_heading and not current_lidar_warning and not lidar_emergency):
                front_distance_for_decision = min(
                    float(front_distance_for_decision),
                    float(front_min_nav),
                )
            # Guidance contract mirrored from nav_guidance for compliance visibility:
            # Emergency: 100% avoidance
            # No obstacle: gate bearing preferred
            # No obstacle, no gate: pure waypoint bearing
            gate_detected = bool(self.camera_status.get("gate_detected", False))
            try:
                gate_stable_s = float(self.camera_status.get("gate_stable_s", 0.0) or 0.0)
            except (TypeError, ValueError):
                gate_stable_s = 0.0
            try:
                gate_bearing_deg = float(self.camera_status.get("gate_center_bearing_deg", 0.0) or 0.0)
            except (TypeError, ValueError):
                gate_bearing_deg = 0.0
            if nav_turn_priority and acquiring_heading and not current_lidar_warning and not lidar_emergency:
                gate_detected = False
                gate_stable_s = 0.0
                gate_bearing_deg = 0.0
            decision = compute_nav_decision(
                distance_m=dist,
                gps_heading_error_deg=gps_heading_for_nav,
                gate_detected=gate_detected,
                gate_stable_s=gate_stable_s,
                gate_bearing_deg=gate_bearing_deg,
                base_speed_mps=base_speed,
                avoidance_bias_deg=avoid,
                front_distance_m=front_distance_for_decision,
                center_distance_m=center_distance_for_decision,
                warn_distance_m=P2_LIDAR_WARN_M,
                stop_distance_m=D_MIN_M,
                gate_stable_threshold_s=P2_STABLE_S,
                failsafe_slow_mps=FAILSAFE_SLOW_MPS,
            )
            if unknown_escape:
                decision = GuidanceDecision(
                    speed_mps=0.0,
                    heading_error_deg=0.0,
                    mode="nav_obstacle_wait",
                    reason="obstacle_direction_unknown",
                    avoidance_bias_deg=0.0,
                    cross_track_error_m=0.0,
                    nominal_heading_deg=float(gps_heading_err),
                    gate_assist_bias_deg=0.0,
                    limit_reason="unknown_sector_stop",
                )
            camera_yellow_threat = bool(abs(float(esc_c)) > 0.01)
            camera_orange_threat = bool(abs(float(esc_o)) > 0.02)
            simple_avoidance_threat = bool(
                simple_avoidance.active
                and self._allow_simple_avoidance_bias(
                    nav_turn_priority,
                    lidar_emergency,
                    camera_lidar_fused=bool(camera_lidar_fused),
                    acquiring_heading=bool(acquiring_heading),
                )
            )
            obstacle_threat = bool(
                lidar_emergency
                or in_warn
                or blocked_by_map
                or unknown_escape
                or simple_avoidance_threat
                or camera_yellow_threat
                or camera_orange_threat
                or camera_lidar_fused
            )
            threat_src = []
            if camera_lidar_fused:
                threat_src.append("camera_lidar_fused")
            if camera_yellow_threat:
                threat_src.append("camera_yellow")
            if camera_orange_threat:
                threat_src.append("camera_orange")
            if lidar_emergency:
                threat_src.append("lidar_emergency")
            elif in_warn:
                threat_src.append("lidar_warn")
            elif blocked_by_map:
                threat_src.append("lidar_map")
            elif simple_avoidance_threat:
                threat_src.append(str(simple_avoidance.reason))
            self._obstacle_threat_active = bool(obstacle_threat)
            self._obstacle_threat_source = "+".join(threat_src) if threat_src else "none"

            center_obstacle = bool(lidar_quality_ok and center_known and center_d < D_MIN_M)
            self._record_guidance_decision(decision)
            if self._maybe_trigger_sim_avoidance_recovery(dist, decision.mode):
                continue
            if decision.mode in ("nav_avoid", "nav_obstacle_wait"):
                self._set_guidance_source("p2_avoid")
            elif decision.mode == "nav_waypoint_track_gate_assist":
                self._set_guidance_source("p2_gate")
            else:
                self._set_guidance_source("p2_waypoint_fallback")
            corrected_heading_err = float(decision.heading_error_deg)
            if (
                decision.mode == "nav_waypoint_track"
                and dist > 1.5
                and not center_obstacle
                and not in_warn
                and not align_follow
            ):
                corrected_heading_err = self._apply_wind_assist(
                    decision.heading_error_deg,
                    "NAV",
                    decision.speed_mps,
                    center_obstacle=False,
                )
            else:
                self._set_wind_assist_idle()

            speed = self._apply_dynamic_speed(decision.speed_mps, corrected_heading_err, "NAV")
            speed = self._apply_turn_priority_speed_caps(speed, corrected_heading_err, dist)
            speed, limiter_reason = self._apply_nav_speed_governor(
                speed_cmd=speed,
                heading_err_deg=corrected_heading_err,
                blocked_level=self._blocked_level,
                dist_m=dist,
                align_follow=align_follow,
                lidar_emergency=lidar_emergency,
            )
            if simple_avoidance.active and simple_avoidance.speed_limit_mps is not None:
                speed = min(float(speed), float(simple_avoidance.speed_limit_mps))
                limiter_reason = str(simple_avoidance.reason)
            if unknown_escape:
                speed = 0.0
                limiter_reason = "unknown_sector_stop"
            if lidar_guard_mode == "limited":
                speed = min(float(speed), float(FAILSAFE_SLOW_MPS))
                if limiter_reason in ("nominal", "heading_full", "heading_reduced", "heading_full_min_progress"):
                    limiter_reason = "lidar_degraded_limit"

            if align_pivot_eligible:
                speed, pivot_reason = self._resolve_nav_align_pivot_speed(
                    nav_turn_priority,
                    gps_heading_err,
                    front_min,
                    speed,
                )
                self.motor_limit_reason = str(pivot_reason)
                limiter_reason = str(pivot_reason)

            if dist < max(0.0, prev_best_distance - 0.05):
                self._tracking_loss_since = None
            else:
                if self._tracking_loss_since is None:
                    self._tracking_loss_since = time.monotonic()
                elif (
                    (time.monotonic() - self._tracking_loss_since) >= 8.0
                    and not in_warn
                    and not lidar_emergency
                    and not align_pivot_eligible
                    and not align_follow
                ):
                    corrected_heading_err = gps_heading_err
                    speed = min(0.30, base_speed)
                    self._p2_avoid_smooth = 0.0
                    self._set_wind_assist_idle()
                    self.guidance_mode = "nav_tracking_recovery"
                    self.guidance_reason = "tracking_loss"
                    self.motor_limit_reason = "tracking_recovery"
                    self._set_guidance_source("p2_waypoint_fallback")
                    limiter_reason = "tracking_recovery"

            command_heading_err = self._apply_heading_damping(corrected_heading_err)
            if unknown_escape:
                command_heading_err = 0.0
                self._heading_cmd_filtered_deg = 0.0
                self._heading_cmd_last_ts = time.monotonic()
            if self._heading_damping_hold_active:
                speed = 0.0
                limiter_reason = "heading_damping_hold"
                self.motor_limit_reason = "heading_damping_hold"
            self.avoidance_active = bool(decision.mode in ("nav_avoid", "nav_obstacle_wait"))
            if abs(esc_o) > 0.02:
                self.avoidance_source = "lidar_yellow_orange" if abs(esc_c) > 0.01 else "lidar_orange"
            else:
                self.avoidance_source = "lidar_yellow" if abs(esc_c) > 0.01 else ("lidar" if abs(esc_l) > 0.01 else "none")
            self.escape_side = str(self._escape_lock_side or "none")
            self._final_speed_limiter = str(limiter_reason)
            nav_align_phase = "ACQUIRE_HEADING" if self._nav_align_mode == "align" else "TRACK_LEG"
            if nav_align_phase != str(getattr(self, "_nav_align_phase_last", "idle")):
                self._nav_align_phase_last = str(nav_align_phase)
                log_jsonl(
                    "usv_main",
                    False,
                    event="nav_align_phase",
                    phase=str(nav_align_phase),
                    waypoint_info=str(self._wp_info),
                    dist_m=round(float(dist), 3),
                    heading_error_deg=round(float(gps_heading_err), 3),
                    front_min_m=round(float(front_min_nav), 3),
                    current_front_min_m=round(float(current_front_min), 3),
                    nav_turn_priority=bool(nav_turn_priority),
                    lidar_emergency=bool(lidar_emergency),
                )

            now_nav = time.monotonic()
            if not hasattr(self, "_last_nav_track_jsonl_ts"):
                self._last_nav_track_jsonl_ts = 0.0
            if now_nav - float(self._last_nav_track_jsonl_ts) >= 2.0:
                self._last_nav_track_jsonl_ts = now_nav
                log_jsonl(
                    "usv_main",
                    False,
                    event="nav_track",
                    heading_error_deg=round(float(command_heading_err), 3),
                    dist_m=round(float(dist), 3),
                    gps_heading_err_deg=round(float(gps_heading_err), 3),
                    corrected_heading_err_deg=round(float(corrected_heading_err), 3),
                    command_heading_err_deg=round(float(command_heading_err), 3),
                    heading_target_deg=round(float((self.current_heading + command_heading_err) % 360.0), 3),
                    current_heading_deg=round(float(self.current_heading), 3),
                    yaw_rate_dps=round(float(self.current_yaw_rate_dps or 0.0), 3),
                    guidance_source=str(self._get_guidance_source()),
                    guidance_detail_source=str(self.guidance_detail_source),
                    decision_mode=str(decision.mode),
                    unknown_escape=bool(unknown_escape),
                    speed_mps=round(float(speed), 3),
                    blocked_level=round(float(self._blocked_level), 3),
                    obstacle_threat_active=bool(self._obstacle_threat_active),
                    obstacle_threat_source=str(self._obstacle_threat_source),
                    lidar_degraded_mode=str(self._lidar_degraded_mode),
                    stable_points=int(self._lidar_stable_point_count),
                    map_occupied_cells=int(self._lidar_occ_cells_count),
                    avoid_switch_count=int(self._avoid_switch_count),
                    final_speed_limiter=str(self._final_speed_limiter),
                    nav_align_mode=str(self._nav_align_mode),
                    nav_align_phase=str(nav_align_phase),
                    nav_strict_heading_first=bool(NAV_STRICT_HEADING_FIRST),
                    nav_align_stable_s=round(float(NAV_ALIGN_STABLE_S), 3),
                    surge_allowed=bool(self._nav_surge_allowed(gps_heading_err)),
                    advance_stable_elapsed_s=round(float(self._nav_advance_stable_elapsed_s()), 3),
                    align_follow=bool(align_follow),
                    target_bearing_deg=round(float(self.nav_target_bearing_deg), 3),
                    current_front_min_m=round(float(current_front_min), 3),
                    front_min_nav_m=round(float(front_min_nav), 3),
                    nav_turn_priority=bool(nav_turn_priority),
                    signed_cross_track_m=round(float(signed_ct_m), 3),
                    cross_track_corr_deg=round(float(cross_track_blend_deg), 3),
                    cross_track_align_active=bool(cross_track_align_active),
                )

            self._command_speed_heading(speed, command_heading_err)
            if self._check_nav_no_motion_hold(dist, speed, obstacle_threat=obstacle_threat):
                return False
            self._write_state()
            time.sleep(LOOP_DT)
        return False

    def _validate_p3_target_lock(self):
        """Fail closed if P3 target color changed after mission start."""
        target_state = load_target_state(TARGET_STATE_FILE)
        state_color = str(target_state.get("target_color") or "").strip().upper()
        profile_color = str((self.mission_profile or {}).get("target_color") or "").strip().upper()
        runtime_color = str(self.target_color or "").strip().upper()
        locked_color = state_color or profile_color or runtime_color
        changed_at_raw = target_state.get("target_color_changed_at", 0.0)
        try:
            changed_at = float(changed_at_raw or 0.0)
        except (TypeError, ValueError):
            changed_at = 0.0

        reject_reason = ""
        try:
            locked_color = validate_mission_target_color(locked_color)
        except ValueError:
            reject_reason = "target_color_not_locked"

        if not reject_reason and profile_color and state_color and profile_color != state_color:
            reject_reason = "target_color_profile_state_mismatch"
        if (
            not reject_reason
            and float(self.mission_start_time or 0.0) > 0.0
            and changed_at > float(self.mission_start_time) + 1e-3
        ):
            reject_reason = "target_color_changed_after_start"
        if not reject_reason and str(self.p3_engagement_mode or "") not in MISSION_PROFILE_ALLOWED_P3_ENGAGEMENT_MODES:
            reject_reason = "invalid_p3_engagement_mode"

        if reject_reason:
            self.mission_end_reason = f"p3_target_lock_rejected:{reject_reason}"
            print(
                f"[ERR] [P3] Target lock gecersiz: reason={reject_reason} "
                f"state={state_color or '--'} profile={profile_color or '--'} runtime={runtime_color or '--'} "
                f"changed_at={changed_at:.3f} start={float(self.mission_start_time or 0.0):.3f}"
            )
            log_jsonl(
                "usv_main",
                False,
                event="p3_target_color_lock_rejected",
                reason=str(reject_reason),
                state_target_color=state_color or None,
                profile_target_color=profile_color or None,
                runtime_target_color=runtime_color or None,
                target_color_changed_at=round(float(changed_at), 3),
                mission_start_time=round(float(self.mission_start_time or 0.0), 3),
                p3_engagement_mode=str(self.p3_engagement_mode or ""),
            )
            return False

        self.target_color = locked_color
        log_jsonl(
            "usv_main",
            False,
            event="p3_target_color_lock_verified",
            target_color=str(self.target_color),
            target_color_changed_at=round(float(changed_at), 3),
            p3_engagement_mode=str(self.p3_engagement_mode or ""),
        )
        return True

    def _p3_contact_evidence(
        self,
        *,
        target_detected,
        target_area,
        dist_m,
        lidar_center_m,
        lidar_ready,
        collision_stuck,
        wrong_target_contact_risk,
        wrong_target_detected,
        wrong_target_area,
        wrong_target_bearing,
        wrong_target_class,
    ):
        """Return P3 contact quorum decision and detailed evidence state."""
        gps_valid = self.engage_wp is not None and 0.0 <= float(dist_m) < 9000.0
        evidence = {
            "vision_area": bool(target_detected and float(target_area) >= 0.20),
            "gps_proximity": bool(gps_valid and float(dist_m) <= 1.0),
            "lidar_proximity": bool(lidar_ready and float(lidar_center_m) < 0.8),
            "lidar_collision": bool(collision_stuck),
            "low_speed": bool(abs(float(self.current_speed_mps or 0.0)) <= 0.25),
            "wrong_target_contact_risk": bool(wrong_target_contact_risk),
            "wrong_target_detected": bool(wrong_target_detected),
        }
        primary_sources = [
            name
            for name in ("vision_area", "gps_proximity", "lidar_proximity", "lidar_collision")
            if evidence[name]
        ]
        sources = [
            name
            for name in ("vision_area", "gps_proximity", "lidar_proximity", "lidar_collision", "low_speed")
            if evidence[name]
        ]
        quorum_ok = False
        if not evidence["wrong_target_contact_risk"]:
            quorum_ok = (
                (evidence["vision_area"] and (evidence["gps_proximity"] or evidence["lidar_proximity"] or evidence["lidar_collision"]))
                or (evidence["gps_proximity"] and evidence["lidar_proximity"])
                or (evidence["lidar_collision"] and (evidence["low_speed"] or evidence["lidar_proximity"]))
                or len(primary_sources) >= 2
            )
        details = {
            **evidence,
            "sources": list(sources),
            "primary_source_count": int(len(primary_sources)),
            "target_area_norm": round(float(target_area), 4),
            "gps_dist_m": round(float(dist_m), 3),
            "lidar_center_m": round(float(lidar_center_m), 3),
            "current_speed_mps": round(float(self.current_speed_mps or 0.0), 3),
            "wrong_target_area_norm": round(float(wrong_target_area), 4),
            "wrong_target_bearing_deg": round(float(wrong_target_bearing), 3),
            "wrong_target_class": str(wrong_target_class or ""),
        }
        return bool(quorum_ok), sources, details

    def _run_engage_attempt(self, timeout_s):
        start_t = time.monotonic()
        contact_start = None
        next_invalid_log = 0.0
        gps_fallback_allowed = (
            self.engage_wp is not None
            and str(self.p3_engagement_mode or "") == "vision_color_track_with_gps_fallback"
        )
        wp = self.engage_wp if gps_fallback_allowed else [self.current_lat, self.current_lon]
        while self.mission_active and (time.monotonic() - start_t) < timeout_s:
            if self._check_abort():
                return False
            dist, gps_heading_err, _ = self._distance_and_heading_error(wp[0], wp[1])
            target_detected = bool(self.camera_status.get("target_detected", False))
            target_area = float(self.camera_status.get("target_area_norm", 0.0))
            target_bearing = float(self.camera_status.get("target_bearing_error_deg", 0.0))
            wrong_target_detected = bool(self.camera_status.get("wrong_target_detected", False))
            wrong_target_area = float(self.camera_status.get("wrong_target_area_norm", 0.0))
            wrong_target_bearing = float(self.camera_status.get("wrong_target_bearing_deg", 0.0))
            wrong_target_class = str(self.camera_status.get("wrong_target_class", ""))
            wrong_target_contact_risk = bool(
                wrong_target_detected
                and (
                    wrong_target_area >= float(CAM_WRONG_TARGET_STRONG_AREA_NORM)
                    or (
                        wrong_target_area >= float(CAM_WRONG_TARGET_MIN_AREA_NORM)
                        and abs(wrong_target_bearing) <= float(CAM_WRONG_TARGET_BEARING_MAX_DEG)
                    )
                )
            )
            if dist >= 9000.0 and not target_detected:
                self._command_speed_heading(0.0, 0.0)
                if time.monotonic() >= next_invalid_log:
                    print("[WARN] [P3] Gecersiz navigasyon/hedef verisi, beklemede")
                    next_invalid_log = time.monotonic() + 1.5
                self._write_state()
                time.sleep(LOOP_DT)
                continue
            if wrong_target_contact_risk:
                away_sign = -1.0 if wrong_target_bearing >= 0.0 else 1.0
                heading_err = clamp(
                    away_sign * max(22.0, min(42.0, abs(wrong_target_bearing) + 18.0)),
                    -float(P3_TARGET_HEADING_CLAMP_DEG),
                    float(P3_TARGET_HEADING_CLAMP_DEG),
                )
                speed = 0.25
                fallback_mode = "p3_wrong_target_avoid"
                fallback_reason = "wrong_target_contact_risk"
                fallback_source = "p3_wrong_target_avoid"
                fallback_limit = "wrong_target_avoidance"
                self.p3_wrong_target_avoidance = True
                self.p3_wrong_target_class = wrong_target_class
                if (time.monotonic() - float(self._p3_wrong_target_last_log_ts or 0.0)) >= 1.0:
                    print(
                        f"[WARN] [P3] Yanlis hedef temas riski: class={wrong_target_class or '--'} "
                        f"area={wrong_target_area:.3f} bearing={wrong_target_bearing:.1f}; kacis uygulanıyor"
                    )
                    log_jsonl(
                        "usv_main",
                        False,
                        event="P3_WRONG_TARGET_AVOID",
                        wrong_target_class=wrong_target_class or None,
                        wrong_target_area_norm=round(wrong_target_area, 4),
                        wrong_target_bearing_deg=round(wrong_target_bearing, 3),
                        heading_error_deg=round(heading_err, 3),
                        heading_target_deg=round(float((self.current_heading + heading_err) % 360.0), 3),
                        current_heading_deg=round(float(self.current_heading), 3),
                        yaw_rate_dps=round(float(self.current_yaw_rate_dps or 0.0), 3),
                        guidance_source=str(self._get_guidance_source()),
                    )
                    self._p3_wrong_target_last_log_ts = time.monotonic()
            elif target_detected:
                heading_err = clamp(
                    float(target_bearing) * float(P3_TARGET_BEARING_GAIN),
                    -float(P3_TARGET_HEADING_CLAMP_DEG),
                    float(P3_TARGET_HEADING_CLAMP_DEG),
                )
                speed = max(0.25, P3_MAX_SPEED_MPS * (1.0 - clamp(target_area, 0.0, 0.85)))
                fallback_mode = "p3_target_track"
                fallback_reason = "vision_target_track"
                fallback_source = "p3_target"
                fallback_limit = "vision_close_loop"
                self.p3_wrong_target_avoidance = False
                self.p3_wrong_target_class = ""
            elif gps_fallback_allowed:
                heading_err = gps_heading_err
                speed = 0.6
                fallback_mode = "p3_waypoint_fallback"
                fallback_reason = "gps_waypoint_fallback"
                fallback_source = "p3_waypoint"
                fallback_limit = "gps_fallback"
                self.p3_wrong_target_avoidance = False
                self.p3_wrong_target_class = ""
            else:
                heading_err = 0.0
                speed = 0.0
                fallback_mode = "p3_target_search_hold"
                fallback_reason = "vision_target_not_detected"
                fallback_source = "p3_search_hold"
                fallback_limit = "no_target_hold"
                self.p3_wrong_target_avoidance = False
                self.p3_wrong_target_class = ""
            if dist < 2.0:
                speed = min(speed, 0.6)
            fallback_decision = GuidanceDecision(
                speed_mps=float(min(speed, P3_MAX_SPEED_MPS)),
                heading_error_deg=float(heading_err),
                mode=str(fallback_mode),
                reason=str(fallback_reason),
                avoidance_bias_deg=0.0,
                cross_track_error_m=0.0,
                nominal_heading_deg=float(heading_err),
                gate_assist_bias_deg=0.0,
                limit_reason=str(fallback_limit),
            )
            decision = fallback_decision
            self.guidance_mode = str(decision.mode)
            self.guidance_reason = str(decision.reason)
            self._set_guidance_source(fallback_source)
            self.motor_limit_reason = str(decision.limit_reason)
            self.avoidance_bias_deg = round(float(decision.avoidance_bias_deg), 3)
            self.cross_track_error_m = round(float(decision.cross_track_error_m), 3)
            self.nominal_heading_deg = round(float(getattr(decision, "nominal_heading_deg", heading_err)), 3)
            self.gate_assist_bias_deg = round(float(getattr(decision, "gate_assist_bias_deg", 0.0)), 3)
            speed_cmd = min(max(0.0, float(decision.speed_mps)), P3_MAX_SPEED_MPS)
            heading_cmd = float(decision.heading_error_deg)
            self._command_speed_heading(speed_cmd, heading_cmd)
            if not hasattr(self, "_last_p3_track_jsonl_ts"):
                self._last_p3_track_jsonl_ts = 0.0
            now_p3 = time.monotonic()
            if now_p3 - float(self._last_p3_track_jsonl_ts) >= 1.0:
                self._last_p3_track_jsonl_ts = now_p3
                log_jsonl(
                    "usv_main",
                    False,
                    event="p3_track",
                    heading_error_deg=round(float(heading_cmd), 3),
                    heading_target_deg=round(float(self.heading_target), 3),
                    current_heading_deg=round(float(self.current_heading), 3),
                    yaw_rate_dps=round(float(self.current_yaw_rate_dps or 0.0), 3),
                    guidance_source=str(self._get_guidance_source()),
                    guidance_detail_source=str(self.guidance_detail_source),
                    decision_mode=str(decision.mode),
                    target_detected=bool(target_detected),
                    target_bearing_deg=round(float(target_bearing), 3),
                    target_bearing_gain=round(float(P3_TARGET_BEARING_GAIN), 3),
                    target_heading_clamp_deg=round(float(P3_TARGET_HEADING_CLAMP_DEG), 3),
                    target_area_norm=round(float(target_area), 4),
                    speed_mps=round(float(speed_cmd), 3),
                    wrong_target_contact_risk=bool(wrong_target_contact_risk),
                )

            # Multi-cue contact decision: never latch P3 from a single weak cue.
            lidar_center = float(self.lidar_center_dist)
            lidar_ready = bool(self.lidar_ready)
            lidar_proximity_ok = lidar_ready and lidar_center < 0.8
            is_collision_stuck = self._check_collision(for_p3_contact=True)
            contact_ok, contact_sources, contact_evidence = self._p3_contact_evidence(
                target_detected=target_detected,
                target_area=target_area,
                dist_m=dist,
                lidar_center_m=lidar_center,
                lidar_ready=lidar_ready,
                collision_stuck=is_collision_stuck,
                wrong_target_contact_risk=wrong_target_contact_risk,
                wrong_target_detected=wrong_target_detected,
                wrong_target_area=wrong_target_area,
                wrong_target_bearing=wrong_target_bearing,
                wrong_target_class=wrong_target_class,
            )
            if contact_ok:
                if contact_start is None:
                    contact_start = time.monotonic()
                elif (time.monotonic() - contact_start) >= 0.6:
                    self.p3_contact_confirmation_source = "+".join(contact_sources) if contact_sources else "unknown"
                    print(
                        f"[ENGAGE] [P3] Angajman tamam (source={self.p3_contact_confirmation_source}, "
                        f"dist={dist:.2f}m area={target_area:.3f} lidar={lidar_center:.2f}m collision={is_collision_stuck})"
                    )
                    log_jsonl(
                        "usv_main",
                        False,
                        event="P3_CONTACT_LATCH",
                        p3_contact_confirmation_source=str(self.p3_contact_confirmation_source),
                        heading_error_deg=round(float(heading_cmd), 3),
                        heading_target_deg=round(float(self.heading_target), 3),
                        current_heading_deg=round(float(self.current_heading), 3),
                        yaw_rate_dps=round(float(self.current_yaw_rate_dps or 0.0), 3),
                        guidance_source=str(self._get_guidance_source()),
                        **contact_evidence,
                    )
                    self.stop_motors()
                    return True
            else:
                contact_start = None
                self.p3_contact_confirmation_source = "none"

            self._write_state()
            time.sleep(LOOP_DT)
        if self.mission_active and (time.monotonic() - start_t) >= timeout_s:
            self.timeout_count += 1
            print(f"[TIMEOUT] [P3] Deneme zaman asimi ({timeout_s:.0f}s), sayac={self.timeout_count}")
            self._write_state()
        return False

    def _retreat_and_hold(self):
        print("[RETREAT] [P3] Retry basarisiz, geri cekilme")
        start_lat, start_lon = self.current_lat, self.current_lon
        t0 = time.monotonic()
        while (time.monotonic() - t0) < P3_REVERSE_TIMEOUT_S:
            self._drain_mav_messages()
            if self._consume_flag(FLAG_STOP):
                self._trigger_estop("YKI", force_rc7=True)
                return
            moved = 0.0
            if start_lat != 0.0 and start_lon != 0.0 and self.current_lat != 0.0 and self.current_lon != 0.0:
                moved = haversine_distance(start_lat, start_lon, self.current_lat, self.current_lon)
            if moved >= P3_REVERSE_DISTANCE_M:
                break
            self._record_file3_if_due()
            time.sleep(LOOP_DT)
        self.stop_motors()
        self._enter_hold("P3_RETREAT")

    def _run_nav_auto_mission(self):
        """Race-mode NAV path.

        Race contract: P1 runs as pure Pixhawk AUTO. After this monitor returns,
        run_nav() switches P2/P3 navigation to GUIDED setpoints.
        """
        # Compliance-required literals kept here:
        #   self.guidance_mode = "nav_pixhawk_auto"
        #   self._set_guidance_source("nav_auto_monitor")
        #   self.motor_limit_reason = "pixhawk_auto"
        p1_auto_env = os.environ.get("USV_RACE_P1_AUTO_WAYPOINTS", "").strip()
        p1_auto_count = 0
        if p1_auto_env:
            try:
                p1_auto_count = max(1, min(len(self.nav_waypoints), int(p1_auto_env)))
            except (TypeError, ValueError):
                self.mission_end_reason = "invalid_p1_auto_waypoint_count"
                print(f"[ERR] [RACE] USV_RACE_P1_AUTO_WAYPOINTS gecersiz: {p1_auto_env}")
                log_jsonl(
                    "usv_main",
                    False,
                    event="race_p1_auto_config_invalid",
                    value=str(p1_auto_env),
                )
                return False
        prev_wp = None
        self._race_p1_auto_completed_count = 0
        self.race_p1_completion_source = "none"
        if USV_MODE == USV_MODE_RACE and not p1_auto_env:
            print("[RACE] [P1_AUTO] Statik P1/P2 waypoint split yok; P2 kamera/lidar kaniti bekleniyor")
            deadline = time.monotonic() + float(P1_AUTO_WAYPOINT_TIMEOUT_S)
            next_log = 0.0
            while self.mission_active:
                if self._check_abort():
                    return False
                self._drain_mav_messages()
                ok, reason = self._p2_entry_evidence()
                self._wp_info = f"AUTO / {len(self.nav_waypoints)}"
                self._active_waypoint_index = self._p2_remaining_start_index_from_fc()
                self.guidance_mode = "nav_pixhawk_auto"
                self.guidance_reason = "pixhawk_auto_p2_evidence_wait"
                self._set_guidance_source("nav_auto_monitor")
                self.avoidance_bias_deg = 0.0
                self.cross_track_error_m = 0.0
                self.nominal_heading_deg = 0.0
                self.gate_assist_bias_deg = 0.0
                self.motor_limit_reason = "pixhawk_auto"
                if ok:
                    self._race_p1_auto_completed_count = self._p2_remaining_start_index_from_fc()
                    self.race_p1_completion_source = f"p2_evidence:{reason}"
                    print(
                        f"[OK] [RACE] P1_AUTO tamam source={self.race_p1_completion_source} "
                        f"next_idx={self._race_p1_auto_completed_count}"
                    )
                    log_jsonl(
                        "usv_main",
                        False,
                        event="race_p1_auto_completed",
                        source=str(self.race_p1_completion_source),
                        mission_current_seq=int(self.mission_current_seq),
                        mission_reached_seq=int(self.mission_reached_seq),
                        next_local_index=int(self._race_p1_auto_completed_count),
                    )
                    return True
                if time.monotonic() >= deadline:
                    self.mission_end_reason = "p1_auto_p2_evidence_timeout"
                    print(f"[ERR] [RACE] P1_AUTO P2 evidence timeout reason={reason}")
                    log_jsonl(
                        "usv_main",
                        False,
                        event="race_p1_auto_timeout",
                        reason=str(reason),
                        timeout_s=round(float(P1_AUTO_WAYPOINT_TIMEOUT_S), 3),
                    )
                    return False
                if time.monotonic() >= next_log:
                    print(
                        f"[WAIT] [RACE] P1_AUTO source=pixhawk_auto reason={reason} "
                        f"mission_current={self.mission_current_seq} mission_reached={self.mission_reached_seq}"
                    )
                    next_log = time.monotonic() + 1.5
                self._write_state()
                time.sleep(LOOP_DT)
            return False

        for idx, wp in enumerate(self.nav_waypoints[:p1_auto_count], start=1):
            self._wp_info = f"{idx} / {len(self.nav_waypoints)}"
            self._active_waypoint_index = idx - 1
            start_snapshot = self._wait_for_stable_nav_solution(samples_required=5, timeout_s=3.0)
            self._configure_waypoint_leg(wp[0], wp[1], leg_start=prev_wp, start_snapshot=start_snapshot)
            wp_deadline = time.monotonic() + float(P1_AUTO_WAYPOINT_TIMEOUT_S)
            while self.mission_active:
                if self._check_abort():
                    return False
                if time.monotonic() > wp_deadline:
                    leg_len = float(self._wp_leg_length_m or 0.0)
                    if self.progress_along_leg_m >= leg_len * 0.5:
                        print(
                            f"[WARN] [NAV] AUTO WP_TIMEOUT idx={idx} timeout={P1_AUTO_WAYPOINT_TIMEOUT_S:.0f}s "
                            f"progress={self.progress_along_leg_m:.2f}m/{leg_len:.2f}m → accepting by progress"
                        )
                        prev_wp = wp
                        self._race_p1_auto_completed_count = idx
                        self.race_p1_completion_source = "explicit_count_timeout_progress"
                        break
                    else:
                        print(
                            f"[ERR] [NAV] AUTO WP_TIMEOUT idx={idx} timeout={P1_AUTO_WAYPOINT_TIMEOUT_S:.0f}s "
                            f"progress={self.progress_along_leg_m:.2f}m/{leg_len:.2f}m → abort"
                        )
                        return False
                dist, _, nav_snapshot = self._distance_and_heading_error(wp[0], wp[1])
                self._wp_target = f"{wp[0]:.7f}, {wp[1]:.7f}"
                self.guidance_mode = "nav_pixhawk_auto"
                self.guidance_reason = "pixhawk_auto_waypoint"
                self._set_guidance_source("nav_auto_monitor")
                self.avoidance_bias_deg = 0.0
                self.cross_track_error_m = 0.0
                self.nominal_heading_deg = 0.0
                self.gate_assist_bias_deg = 0.0
                self.motor_limit_reason = "pixhawk_auto"
                if not self.nav_fix_valid:
                    self._write_state()
                    time.sleep(LOOP_DT)
                    continue
                if not self._wp_leg_valid:
                    self._configure_waypoint_leg(wp[0], wp[1], leg_start=prev_wp, start_snapshot=nav_snapshot)
                accepted, hold_active = self._evaluate_waypoint_acceptance(dist, nav_snapshot)
                if hold_active and accepted:
                    print(
                        f"[OK] [NAV] AUTO WP_REACHED dist={dist:.2f}m hold={T_HOLD_S:.1f}s "
                        f"reason={self.waypoint_accept_reason} progress={self.progress_along_leg_m:.2f}m"
                    )
                    prev_wp = wp
                    self._race_p1_auto_completed_count = idx
                    self.race_p1_completion_source = "explicit_count_waypoint_acceptance"
                    break
                self._write_state()
                time.sleep(LOOP_DT)
        return True

    def _run_nav_unified(self):
        """
        Unified Pi-guided navigation — single deterministic decision engine.

        Produces IDENTICAL behavior in TEST and RACE modes:
          - Pixhawk operates in GUIDED mode (actuator-only interface)
          - Pi computes all navigation: GPS cross-track, lidar avoidance, gate assist
          - Sensor fusion, dynamic speed, wind assist all active

        Architecture rule (sim-first):
          Core logic (decision + navigation) = SHARED
          I/O layer only differs: sensor source (sim vs real) / actuator target (sim vs real)
        """
        if not self._wait_p2_ready():
            return False
        if not self._set_mode("GUIDED"):
            print("[ERR] [NAV] GUIDED moda gecilemedi (unified path)")
            return False

        prev_wp = None
        for idx, wp in enumerate(self.nav_waypoints, start=1):
            self._wp_info = f"{idx} / {len(self.nav_waypoints)}"
            self._active_waypoint_index = idx - 1
            print(
                f"[NAV_UNIFIED] Waypoint {idx}/{len(self.nav_waypoints)}: "
                f"({wp[0]:.6f}, {wp[1]:.6f})"
            )
            if not self._navigate_p2_waypoint(wp[0], wp[1], leg_start=None):
                return False
            self._track_gate_event()
            self._write_state()

        if self.gate_count < int(self.p2_min_gate_count):
            if not self._wait_for_p2_gate_minimum():
                return False
        print(f"[OK] [NAV] Unified tamam gate={self.gate_count}")
        return True

    def run_nav(self):
        """
        Navigation phase: iterate all nav_waypoints with the project control split.

        Race P1 is pure Pixhawk AUTO and monitored passively. Test NAV and
        race P2/P3 use the shared Pi-guided waypoint engine with GUIDED
        velocity/yaw setpoints.
        """
        print("=" * 52)
        print("  [NAV] WAYPOINT TAKIBI + LIDAR + KAPI")
        print("=" * 52)
        # Guard: heading stabilization before first navigation command
        elapsed_s = time.time() - self.mission_start_time
        if elapsed_s < 1.0:
            print(f"[GUARD] Heading sync ({elapsed_s:.2f}s), waiting before NAV commands")
            time.sleep(max(0.5, 1.0 - elapsed_s))

        self.state = self.STATE_NAV
        self._p1_avoid_smooth = 0.0
        self._p2_avoid_smooth = 0.0
        self._p2_local_minima_start_ts = None
        self._p2_local_minima_active = False
        self._refresh_mission_progress_ui_at_start()
        self._write_state()
        nav_mode = "GUIDED" if USV_MODE != USV_MODE_RACE else "AUTO"
        if os.environ.get("USV_SIM") == "1":
            if not self._sitl_prepare_autopilot_for_guided():
                print("[ERR] [NAV] [SIM] ArduPilot GPS/EKF hazir degil; NAV modlari reddedildi")
                self.mission_end_reason = "fc_nav_prereq_timeout"
                self._enter_hold("FC_NAV_PREREQ_TIMEOUT")
                return False
        if self._sim_mavlink_actuation_rc:
            if not self._set_mode_sim_retry("MANUAL", extra_attempts=3):
                print("[ERR] [NAV] MANUAL moda gecilemedi (SIM RC fallback)")
                return False
            self._write_state()
        elif str(self.vehicle_mode_name or "").upper() != str(nav_mode).upper() and not self._set_mode_sim_retry(nav_mode):
            sim_rc_ok = (
                os.environ.get("USV_SIM") == "1"
                and USV_MODE != USV_MODE_RACE
                and nav_mode == "GUIDED"
                and SIM_MANUAL_RC_FALLBACK_ENABLED
            )
            if sim_rc_ok:
                self._mark_sim_manual_rc_fallback("guided_mode_rejected")
                if not self._set_mode_sim_retry("MANUAL"):
                    print("[ERR] [NAV] MANUAL moda da gecilemedi (SIM RC fallback)")
                    return False
            else:
                print(f"[ERR] [NAV] {nav_mode} moda gecilemedi")
                if nav_mode == "GUIDED":
                    self.mission_end_reason = "mode_change_failed:GUIDED"
                    self._enter_hold("GUIDED_MODE_REJECTED")
                return False

        if USV_MODE == USV_MODE_RACE:
            assert nav_mode == "AUTO", "Race mode NAV must be pure Pixhawk AUTO"
            print("[RACE] [NAV] Pure Pixhawk AUTO mode - no Pi takeover or fallback")

        self._write_state()
        if not self.nav_waypoints:
            print("[WARN] [NAV] Nav waypoint yok, dogrudan ENGAGE'e geciliyor")
            return True

        if USV_MODE == USV_MODE_RACE:
            auto_result = self._run_nav_auto_mission()
            if not auto_result:
                return False
            if not self._wait_p2_ready():
                return False
            if not self._set_mode_sim_retry("GUIDED"):
                print("[ERR] [NAV] GUIDED moda gecilemedi (P2/P3 race)")
                self.mission_end_reason = "mode_change_failed:GUIDED"
                self._enter_hold("GUIDED_MODE_REJECTED")
                return False
            p2_start_idx = int(self._race_p1_auto_completed_count or 0)
            for idx, wp in enumerate(self.nav_waypoints[p2_start_idx:], start=p2_start_idx + 1):
                self._wp_info = f"{idx} / {len(self.nav_waypoints)}"
                self._active_waypoint_index = idx - 1
                _wp_dist = haversine_distance(self.current_lat, self.current_lon, wp[0], wp[1])
                _wp_bearing = calculate_bearing(self.current_lat, self.current_lon, wp[0], wp[1])
                _wp_hdg_err = normalize_heading_error(_wp_bearing - self.current_heading)
                print(f"[RACE] [P2_GUIDED] Waypoint {idx}/{len(self.nav_waypoints)}: ({wp[0]:.6f}, {wp[1]:.6f}) current=({self.current_lat:.6f}, {self.current_lon:.6f}) dist={_wp_dist:.2f}m hdg_err={_wp_hdg_err:.1f}° source={self.nav_solution_source}")
                if not self._navigate_p2_waypoint(wp[0], wp[1], leg_start=None):
                    return False
                self._track_gate_event()
                self._write_state()
            if self.gate_count < int(self.p2_min_gate_count):
                if not self._wait_for_p2_gate_minimum():
                    return False
            print("[OK] [NAV] Tamam")
            return True

        # Test mode: Pi navigation (gate + lidar); GUIDED setpoints veya SIM'de MANUAL+RC override
        if not self._wait_p2_ready():
            return False
        if not self._sim_mavlink_actuation_rc:
            if not self._set_mode_sim_retry("GUIDED"):
                print("[ERR] [NAV] GUIDED moda gecilemedi")
                self.mission_end_reason = "mode_change_failed:GUIDED"
                self._enter_hold("GUIDED_MODE_REJECTED")
                return False

        for idx, wp in enumerate(self.nav_waypoints, start=1):
            self._wp_info = f"{idx} / {len(self.nav_waypoints)}"
            self._active_waypoint_index = idx - 1
            _wp_dist = haversine_distance(self.current_lat, self.current_lon, wp[0], wp[1])
            _wp_bearing = calculate_bearing(self.current_lat, self.current_lon, wp[0], wp[1])
            _wp_hdg_err = normalize_heading_error(_wp_bearing - self.current_heading)
            print(f"[NAV] Waypoint {idx}/{len(self.nav_waypoints)}: ({wp[0]:.6f}, {wp[1]:.6f}) current=({self.current_lat:.6f}, {self.current_lon:.6f}) dist={_wp_dist:.2f}m hdg_err={_wp_hdg_err:.1f}° source={self.nav_solution_source}")
            if not self._navigate_p2_waypoint(wp[0], wp[1], leg_start=None):
                return False
            self._track_gate_event()
            self._write_state()

        if self.gate_count < int(self.p2_min_gate_count):
            if not self._wait_for_p2_gate_minimum():
                return False
        print(f"[OK] [NAV] Tamam gate={self.gate_count}")
        return True

    def run_engage(self):
        """
        Engagement phase: HSV color detection → target contact (kamikaze).
        """
        print("\n" + "=" * 52)
        print("  [ENGAGE] HSV HEDEFLEME + ANGAJMAN")
        print("=" * 52)
        time.sleep(0.3)  # Allow brief heading stabilization
        if not self._validate_p3_target_lock():
            self._enter_hold("P3_TARGET_LOCK_INVALID")
            return False
        self.state = self.STATE_ENGAGE
        self.p3_wrong_target_avoidance = False
        self.p3_wrong_target_class = ""
        self.p3_contact_confirmation_source = "none"
        self._set_dynamic_speed_idle()
        self._set_wind_assist_idle()
        if not self._set_mode("GUIDED"):
            print("[ERR] [ENGAGE] GUIDED moda gecilemedi")
            return False
        self._write_state()
        if self.engage_wp is None:
            print("[ENGAGE] P3 waypoint yok; renk algilama ile vision-only angajman yapilacak")
        if self._run_engage_attempt(P3_TIMEOUT_S):
            self.mission_end_reason = "engage_success"
            return True
        for retry in range(P3_RETRY_COUNT):
            print(f"[RETRY] [ENGAGE] Retry {retry + 1}/{P3_RETRY_COUNT}")
            if self._run_engage_attempt(P3_RETRY_S):
                self.mission_end_reason = "engage_success_retry"
                return True
        self._retreat_and_hold()
        self.mission_end_reason = "engage_timeout_retreat"
        return False

    def _wait_for_next_parkur(self, current_name, next_name):
        print(f"🔄 [{current_name}] -> [{next_name}] otomatik gecis (kullanici girdisi kapali)")
        return True

    def _check_time(self):
        elapsed = time.time() - self.mission_start_time
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)
        if elapsed > 20 * 60:
            print(f"⏰ [SURE] 20dk asildi ({minutes}:{seconds:02d})")
        elif elapsed > 18 * 60 and int(elapsed) % 30 == 0:
            print(f"[WARN] [SURE] Kritik bolge ({minutes}:{seconds:02d})")

    def _update_heading_from_vehicle_position(self):
        """Update heading from vehicle_position.json (Gazebo-sourced heading_rad)."""
        try:
            pos_file = Path(CONTROL_DIR) / "vehicle_position.json"
            with open(pos_file) as f:
                pos_data = json.load(f)
            heading_rad = pos_data.get('heading_rad', None)
            if heading_rad is not None:
                heading_deg = heading_rad_to_nav_deg(float(heading_rad))
                # Angle-aware EMA: blend shortest angular delta to avoid wrap jumps
                # such as 359° -> 1° being averaged through 180°.
                alpha = 0.40
                old_hdg = self.current_heading
                delta_deg = normalize_heading_error(heading_deg - old_hdg)
                self.current_heading = (old_hdg + alpha * delta_deg) % 360.0
                
                # Log heading changes (every 2s or when delta > 2°)
                now = time.monotonic()
                if not hasattr(self, '_last_hdg_log'):
                    self._last_hdg_log = now
                    self._last_hdg_logged = old_hdg
                if (now - self._last_hdg_log) >= 2.0 or abs(self.current_heading - self._last_hdg_logged) >= 2.0:
                    print(
                        f"[HDG_UPDATE] Gazebo={heading_deg:.1f}° "
                        f"delta={delta_deg:.1f}° "
                        f"→ current_heading={self.current_heading:.1f}° "
                        f"(was {old_hdg:.1f}°)"
                    )
                    self._last_hdg_log = now
                    self._last_hdg_logged = self.current_heading
        except Exception as exc:
            if not hasattr(self, '_hdg_error_logged'):
                print(f"[WARN] [HDG_UPDATE] Error reading vehicle_position: {exc}")
                self._hdg_error_logged = True

    def _check_collision(self, for_p3_contact=False):
        """Detect: moving with speed command but vessel stuck against obstacle.

        Args:
            for_p3_contact: If True, returns collision state for P3 contact decision.
                           Otherwise, logs collision events globally.
        """
        now = time.time()

        # Cooldown: don't log same collision repeatedly
        if now - self._last_collision_ts < self._collision_cooldown_s:
            return False if for_p3_contact else None
        
        # Get current position from vehicle_position.json
        try:
            pos_file = Path(CONTROL_DIR) / "vehicle_position.json"
            pos_data = json.load(open(pos_file))
            current_pos = (pos_data['pos_x'], pos_data['pos_y'])
        except:
            return False if for_p3_contact else None
        
        # Has speed command?
        has_speed_cmd = self.v_target > self._collision_speed_thresh_mps
        
        # Position change from last check
        if self._last_collision_check_pos != (0.0, 0.0):
            dx = current_pos[0] - self._last_collision_check_pos[0]
            dy = current_pos[1] - self._last_collision_check_pos[1]
            pos_change = math.sqrt(dx*dx + dy*dy)
        else:
            self._last_collision_check_pos = current_pos
            return False if for_p3_contact else None
        
        # Collision = moving command, minimal position change, close lidar
        is_collision = (
            has_speed_cmd and 
            pos_change < self._collision_position_thresh_m and
            self.lidar_center_dist < 1.0
        )
        
        if is_collision:
            if for_p3_contact:
                # Return collision state for P3 contact decision without logging
                self._last_collision_check_pos = current_pos
                return True
            log_jsonl("usv_main", False, 
                event="collision_detected",
                ts=now,
                pos_x=round(current_pos[0], 2),
                pos_y=round(current_pos[1], 2),
                heading_deg=round(self.current_heading, 1),
                v_target_mps=round(self.v_target, 2),
                heading_target_deg=round(self.heading_target, 1),
                lidar_center_m=round(self.lidar_center_dist, 2),
                lidar_left_m=round(self.lidar_left_dist, 2),
                lidar_right_m=round(self.lidar_right_dist, 2),
                pos_change_cm=round(pos_change * 100, 1),
                reason="stuck_at_obstacle"
            )
            print(f"[COLLISION] X={current_pos[0]:+.2f}m Y={current_pos[1]:+.2f}m lidar_center={self.lidar_center_dist:.2f}m pos_change={pos_change*100:.1f}cm")
            self._last_collision_ts = now
        
        self._last_collision_check_pos = current_pos
        return is_collision if for_p3_contact else None

    def run(self):
        while self.mission_active:
            self._check_time()
            if self.state == self.STATE_NAV:
                if self.run_nav():
                    self.state = self.STATE_ENGAGE
                else:
                    if self.state != self.STATE_HOLD:
                        if str(self.mission_end_reason or "none") == "none":
                            self.mission_end_reason = "nav_abort"
                        self._enter_hold("NAV_ABORT")
                    break
            elif self.state == self.STATE_ENGAGE:
                if self.run_engage():
                    self.state = self.STATE_COMPLETED
                else:
                    if self.state != self.STATE_HOLD:
                        self._enter_hold("ENGAGE_ABORT")
                    break
            elif self.state in (self.STATE_COMPLETED, self.STATE_HOLD):
                break
            # Update compass heading from Gazebo every tick
            self._update_heading_from_vehicle_position()
            # Check for collisions every tick
            self._check_collision()
            self._update_mode_state(source="run_loop")
            self._write_state()

        if self.state == self.STATE_COMPLETED:
            print("[DONE] [GOREV] Tum parkurlar tamamlandi")
            self._wp_target = "TAMAMLANDI"
            self._wp_info = "-- / --"
            self._active_waypoint_index = -1
            if str(self.mission_end_reason or "none") == "none":
                self.mission_end_reason = "completed"
        self.mission_active = False
        if os.path.exists(FLAG_START):
            self._consume_flag(FLAG_START)
        if self.state == self.STATE_COMPLETED:
            self._set_guidance_idle(reason="mission_completed")
        elif self.state == self.STATE_HOLD:
            self._set_guidance_idle(reason=f"hold:{self.hold_reason.lower()}")
        else:
            self._set_guidance_idle(reason="mission_end")
        self._set_dynamic_speed_idle()
        self._set_wind_assist_idle()
        self._set_horizon_lock_idle(reason="mission_end", channel="none")
        if self.state == self.STATE_HOLD and str(self.hold_reason).startswith("FAILSAFE"):
            self._set_virtual_anchor_idle(reason="armed", clear_center=False)
        else:
            self._set_virtual_anchor_idle(reason="mission_end", clear_center=True)
        if not self.estop_latched:
            self.command_lock = False
        self.stop_motors()
        self._update_mode_state(source="run_loop", reason="mission_end")
        self._write_state()


if __name__ == "__main__":
    def _sigterm_handler(signum, frame):
        print("\n[STOP] [SISTEM] SIGTERM alindi, kapatiliyor...")
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, _sigterm_handler)

    usv = USVStateMachine()
    try:
        mission_path = sys.argv[1] if len(sys.argv) > 1 else MISSION_FILE
        usv.load_mission(mission_path)
        usv.stop_motors()
        usv._set_mode("MANUAL")
        print("\n[READY] [SISTEM] Hazir - gorev baslatma bekleniyor")
        if USV_MODE == USV_MODE_RACE:
            print("   Race start: Sadece RC CH5 >= 1700 (API/flag kapali)")
        else:
            print("   Test start: Web controller Start veya POST http://127.0.0.1:8080/api/start_mission")
        usv._refresh_health_check()
        usv._write_state()

        # SIM boot: JSON/SITL GPS is the primary FC GPS/EKF source. GPS_INPUT
        # injection remains optional via SIM_SITL_GPS_INPUT_PUMP=1 for bench
        # diagnostics only.
        _sim_gps_pump_last_ts = 0.0
        _sim_gps_origin_last_ts = 0.0
        _is_sim = bool(os.environ.get("USV_SIM") == "1")

        while True:
            usv._drain_mav_messages()
            usv._read_camera_status()
            usv._compute_trust_bar()
            usv._update_watchdog()
            usv._refresh_health_check()
            usv._record_file3_if_due()

            # SIM GPS/EKF upkeep: optional GPS_INPUT bench pump + periodic origin seed.
            if _is_sim and usv.master:
                _now_pump = time.monotonic()
                if (_now_pump - _sim_gps_pump_last_ts) >= 0.1:
                    _sim_gps_pump_last_ts = _now_pump
                    try:
                        usv._sitl_gps_input_from_sim()
                    except Exception:
                        pass
                if (_now_pump - _sim_gps_origin_last_ts) >= 5.0:
                    _sim_gps_origin_last_ts = _now_pump
                    try:
                        usv._sitl_seed_global_origin_if_needed()
                    except Exception:
                        pass

            if usv._consume_flag(FLAG_STOP):
                usv._trigger_estop("YKI", force_rc7=True)

            if not usv.mission_active:
                if usv.state == usv.STATE_HOLD:
                    usv._run_virtual_anchor_step()
                else:
                    usv.v_target = 0.0
                    usv.heading_target = usv.current_heading
                if usv.master and not usv._start_request_pending():
                    _im_now = time.monotonic()
                    _skip_idle_ph = bool(
                        os.environ.get("USV_SIM") == "1"
                        and str(getattr(usv, "mission_upload_source", "") or "")
                        in _SIM_SKIP_IDLE_PIXHAWK_SOURCES
                    )
                    if _skip_idle_ph:
                        if (_im_now - getattr(usv, "_last_idle_pixhawk_sync_mono", 0.0)) >= PIXHAWK_MISSION_IDLE_MIRROR_S:
                            usv._last_idle_pixhawk_sync_mono = _im_now
                            coords = usv._current_local_mission_coords()
                            if coords and usv._upload_local_mission_to_pixhawk(coords, usv.mission_upload_source):
                                usv._write_state()
                    elif (_im_now - getattr(usv, "_last_idle_pixhawk_sync_mono", 0.0)) >= PIXHAWK_MISSION_IDLE_MIRROR_S:
                        usv._last_idle_pixhawk_sync_mono = _im_now
                        if usv._sync_pixhawk_mission_if_available():
                            usv._write_state()
                if USV_MODE == USV_MODE_RACE and os.path.exists(FLAG_START):
                    usv._consume_flag(FLAG_START)
                    if (time.monotonic() - usv._last_race_flag_purge_log) >= 2.0:
                        print("[POLICY] [RACE] API start flag temizlendi (RC-only baslatma politikasi)")
                        usv._last_race_flag_purge_log = time.monotonic()

                # Check for API start.
                # RACE mode is always RC-CH5 only; TEST mode is API/flag only.
                start_from_api = bool(USV_MODE != USV_MODE_RACE and usv._consume_flag(FLAG_START))
                start_from_rc = bool(USV_MODE == USV_MODE_RACE and usv.rc_channels.get("ch5", 0) >= RC_RACE_START_PWM)

                if start_from_api or start_from_rc:
                    src = "RC" if start_from_rc else "API"
                    print(f"[START] [START] Kaynak={src}")
                    start_result = usv.start_mission()
                    if start_result:
                        usv.run()
            usv._update_mode_state(source="main_loop")
            usv._write_state()
            if not hasattr(usv, "_jsonl_tick_ts"):
                usv._jsonl_tick_ts = 0.0
            _tn = time.monotonic()
            if _tn - usv._jsonl_tick_ts >= 0.25:
                usv._jsonl_tick_ts = _tn
                log_jsonl(
                    "usv_main",
                    False,
                    event="main_tick",
                    state=int(usv.state),
                    mission_active=bool(usv.mission_active),
                    estop=bool(usv.estop_latched),
                    v_target=float(getattr(usv, "v_target", 0.0) or 0.0),
                    heading_target=float(getattr(usv, "heading_target", 0.0) or 0.0),
                    current_heading=float(getattr(usv, "current_heading", 0.0) or 0.0),
                    lidar_ready=bool(getattr(usv, "lidar_ready", False)),
                    camera_ready=bool(getattr(usv, "camera_ready", False)),
                    guidance_mode=str(getattr(usv, "guidance_mode", "idle") or "idle"),
                    guidance_reason=str(getattr(usv, "guidance_reason", "idle") or "idle"),
                    avoidance_active=bool(getattr(usv, "avoidance_active", False)),
                    avoidance_source=str(getattr(usv, "avoidance_source", "none") or "none"),
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[STOP] [SISTEM] Kapatiliyor...")
    finally:
        usv.stop_motors()
        usv._close_file3_recorder()

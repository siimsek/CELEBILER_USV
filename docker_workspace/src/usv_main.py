"""
CELEBILER USV - Kritik Rapor Uyum Durum Makinesi
"""

import csv
import glob
import json
import math
import os
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
    LIDAR_READY_TIMEOUT_S,
    LINK_TOPOLOGY,
    LOG_DIR,
    MISSION_FILE_DEFAULT,
    MISSION_INPUT_FORMAT,
    MISSION_SPLIT_P2_COUNT,
    MISSION_SPLIT_P3_COUNT,
    P1_SPEED_APPROACH_MPS,
    P1_SPEED_CRUISE_MPS,
    P2_CAM_YELLOW_WEIGHT,
    P2_CRUISE_MPS,
    P2_ESCAPE_MAX_DEG,
    P2_ESCAPE_MAX_DEG_LOCAL_MINIMA,
    P2_GATE_CONFIRM_S,
    P2_LIDAR_WARN_M,
    P2_LOCAL_MINIMA_TIMEOUT_S,
    P2_STABLE_S,
    P2_WAIT_SPEED_MPS,
    P3_MAX_SPEED_MPS,
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
    TARGET_STATE_FILE,
    default_sim_mission,
    get_mission_split_profile,
    load_target_state,
    split_mission_waypoints,
    validate_coordinate_mission,
)
from runtime_debug_log import log_jsonl, setup_component_logger

_usv_dbg = setup_component_logger("usv_main")
_usv_dbg.info(
    "boot LOG_DIR=%s USV_SIM=%s USV_MODE=%s",
    LOG_DIR,
    os.environ.get("USV_SIM"),
    os.environ.get("USV_MODE"),
)

BAUD_RATE = 115200
LOOP_DT = 1.0 / CONTROL_HZ

# Mission file default to profile-resolved path
MISSION_FILE = os.environ.get("MISSION_FILE", MISSION_FILE_DEFAULT)

FLAG_START = f"{CONTROL_DIR}/start_mission.flag"
FLAG_STOP = f"{CONTROL_DIR}/emergency_stop.flag"
STATE_FILE = f"{CONTROL_DIR}/mission_state.json"
LINK_STATE_FILE = f"{CONTROL_DIR}/telemetry_link_state.json"
CAMERA_STATUS_FILE = f"{CONTROL_DIR}/camera_status.json"

FILE3_MAP_MP4 = f"{LOG_DIR}/file3_local_map.mp4"
FILE3_INDEX_CSV = f"{LOG_DIR}/file3_local_map_index.csv"
FILE3_MAP_JPG = f"{LOG_DIR}/file3_local_map_latest.jpg"


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
    if error > 180:
        error -= 360
    if error < -180:
        error += 360
    return error




class USVStateMachine:
    STATE_IDLE = 0
    STATE_PARKUR1 = 1
    STATE_PARKUR2 = 2
    STATE_PARKUR3 = 3
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
        self.gps_satellites_visible = 0
        self.gps_fix_type = 0
        self.gps_global_position_int_received = False  # Track if GLOBAL_POSITION_INT arrived (simulation GPS_RAW_INT fallback)
        self.battery_voltage = 0.0  # Volts (from SYS_STATUS MAVLink message)
        self.rc_channels = {f"ch{i}": 1500 for i in range(1, 9)}  # Initialize to neutral PWM
        self._last_sent_rc_override = {"ch1": 1500, "ch3": 1500, "ts": 0.0}

        self.v_target = 0.0
        self.heading_target = 0.0

        self.camera_status = {}
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
        self._lidar_front_samples = []
        self._lidar_sector_hold = {"left": None, "center": None, "right": None}
        self._p1_avoid_smooth = 0.0
        self._p2_avoid_smooth = 0.0

        self.fusion_policy = "reject_if_lidar_unavailable"
        self.ghost_gate_count = 0
        self.ghost_target_count = 0
        self.last_confirmed_gate_dist_m = None
        self.last_confirmed_target_dist_m = None
        self._last_gate_confirm_ts = 0.0
        self._last_target_confirm_ts = 0.0
        self._gate_fusion_hold_since = None
        self._target_fusion_hold_since = None
        self._gate_ghost_latched = False
        self._target_ghost_latched = False
        self._gate_camera_only_since = None
        self._target_camera_only_since = None
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
        self.heartbeat_age_s = 0.0
        self.link_heartbeat_age_s = 0.0
        self.link_heartbeat_source = "onboard_fallback"
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

        self.waypoints_p1 = []
        self.waypoints_p2 = []
        self.waypoints_p3 = []
        self.target_color = ""
        self.mission_input_format = MISSION_INPUT_FORMAT
        self.mission_upload_source = "local_flat_file"
        self.mission_validated_at_timestamp = None
        self.mission_split_profile = get_mission_split_profile(0)
        self.gate_count = 0
        self._gate_event_start = None
        self._gate_event_latched = False
        self._gate_bearing_prev = None  # Track bearing sign for spec-compliant gate passage
        self._wp_target = "--"
        self._wp_info = "-- / --"

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
        self._connect_pixhawk()
        self._try_connect_lidar()
        self._init_file3_recorder()
        self._refresh_health_check()
        self._write_state()

    def _active_parkur_label(self):
        mapping = {
            self.STATE_IDLE: "IDLE",
            self.STATE_PARKUR1: "P1",
            self.STATE_PARKUR2: "P2",
            self.STATE_PARKUR3: "P3",
            self.STATE_COMPLETED: "COMPLETED",
            self.STATE_HOLD: "HOLD",
        }
        return mapping.get(self.state, "UNKNOWN")

    def _get_objective_phase(self):
        mapping = {
            self.STATE_IDLE: "IDLE",
            self.STATE_PARKUR1: "P1_CORRIDOR",
            self.STATE_PARKUR2: "P2_OBSTACLE_CORRIDOR",
            self.STATE_PARKUR3: "P3_TARGET_ENGAGEMENT",
            self.STATE_COMPLETED: "COMPLETED",
            self.STATE_HOLD: "HOLD",
        }
        return mapping.get(self.state, "UNKNOWN")

    def _get_perception_policy(self):
        active_parkur = self._active_parkur_label()
        return {
            "gate": active_parkur in ("P1", "P2"),
            "yellow_obstacle": active_parkur == "P2",
            "target": active_parkur == "P3",
        }

    def _get_guidance_source(self):
        """
        Return explicit guidance source identifying execution mode.
        
        Returns:
        - "p1_pixhawk_auto": Race mode P1 (Pixhawk autonomous)
        - "p1_pi_guided": Test mode P1 (Pi-based guidance)
        - "p2_pi_guided": P2 guidance (Pi in both modes)
        - "p3_pi_guided": P3 guidance (Pi in both modes)
        - "idle": Not in active mission
        """
        if self.state == self.STATE_PARKUR1:
            if USV_MODE == USV_MODE_RACE:
                return "p1_pixhawk_auto"
            else:
                return "p1_pi_guided"
        elif self.state == self.STATE_PARKUR2:
            return "p2_pi_guided"
        elif self.state == self.STATE_PARKUR3:
            return "p3_pi_guided"
        else:
            return "idle"

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
            gps_q = camera_q = lidar_q = rc_q = 1.0
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

    def _gps_position_valid(self):
        try:
            lat = float(self.current_lat)
            lon = float(self.current_lon)
        except (TypeError, ValueError):
            return False
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return False
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return False
        return True

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
                if parkur_label == "P1":
                    turn_floor = DYN_SPEED_MIN_MPS_P1
                elif parkur_label == "P2":
                    turn_floor = DYN_SPEED_MIN_MPS_P2
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
            camera_only_attr = "_gate_camera_only_since" if "gate" in label else "_target_camera_only_since"
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

        return fused

    def _refresh_link_heartbeat(self):
        onboard_age = max(0.0, time.monotonic() - self.last_heartbeat_time)
        if self.simulation_mode or not self.master:
            self.link_heartbeat_age_s = 0.0
            self.link_heartbeat_source = "simulation"
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
            mavlink_ok = True
            heartbeat_ok = True
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
            # Sim ortaminda /scan kaynakli gecici veri bosluklari startup'i bloklamamali.
            camera_fresh = bool(self.camera_ready)
            lidar_fresh = bool(self.lidar_ready)
        else:
            storage_health = evaluate_storage_health(USV_MODE, local_dir=LOG_DIR)
            camera_fresh = self.camera_ready
            lidar_fresh = bool(self.lidar_ready and lidar_point_count > 0)

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
                "lidar_ready": bool(self.lidar_ready),
            }
            payload = {
                "ts_monotonic": round(time.monotonic(), 3),
                "mode": "race" if USV_MODE == USV_MODE_RACE else "test",
                "state": self.state,
                "active": self.mission_active,
                "start_time": self.mission_start_time,
                "hold_reason": self.hold_reason,
                "target": self._wp_target,
                "wp_info": self._wp_info,
                "active_parkur": active_parkur,
                "objective_phase": objective_phase,
                "guidance_source": self._get_guidance_source(),
                "target_color": self.target_color or "--",
                "mission_input_format": self.mission_input_format,
                "perception_policy": perception_policy,
                "gate_count": self.gate_count,
                "command_lock": self.command_lock,
                "failsafe_state": self.failsafe_state,
                "mission_lifecycle": {
                    "schema_version": 1,
                    "upload_source": self.mission_upload_source,
                    "validated_at_timestamp": self.mission_validated_at_timestamp,
                },
                "waypoint_counts": {
                    "parkur1": len(self.waypoints_p1) if hasattr(self, 'waypoints_p1') else 0,
                    "parkur2": len(self.waypoints_p2) if hasattr(self, 'waypoints_p2') else 0,
                    "parkur3": len(self.waypoints_p3) if hasattr(self, 'waypoints_p3') else 0,
                },
                "mission_split_profile": dict(self.mission_split_profile),
                "gate_gecildi": bool(self.gate_count > 0),  # Event flag: gate passed
                "angajman_tamam": bool(self.state == self.STATE_COMPLETED),  # Event flag: engagement complete
                "timeout": bool(self.timeout_count > 0),  # Event flag: timeout occurred
                "estop_state": self.estop_latched,
                "estop_source": self.estop_source or "--",
                "camera_ready": self.camera_ready,
                "lidar_ready": self.lidar_ready,
                "roll_deg": round(self.current_roll_deg, 3),
                "pitch_deg": round(self.current_pitch_deg, 3),
                "gps_satellites_visible": int(self.gps_satellites_visible),
                "gps_fix_type": int(self.gps_fix_type),
                "battery_voltage": round(float(self.battery_voltage), 3),
                "heartbeat_age_s": round(self.heartbeat_age_s, 3),
                "gps_ok": bool(self.gps_fix_type >= 2 or (self.gps_global_position_int_received and os.environ.get("USV_SIM") == "1")),  # GPS OK: fix_type>=2 OR GLOBAL_POSITION_INT in simulation
                "ekf_ok": bool(self.health_ready),  # EKF health represented by overall health readiness
                "imu_ok": bool(abs(self.current_roll_deg) < 180 and abs(self.current_pitch_deg) < 180),  # IMU health: valid roll/pitch
                "link_heartbeat_age_s": round(self.link_heartbeat_age_s, 3),
                "link_heartbeat_source": self.link_heartbeat_source,
                "current_heading": round(self.current_heading, 3),
                "v_target": round(self.v_target, 3),
                "heading_target": round(self.heading_target, 3),
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
                    old.get('target') == payload['target'] and
                    old.get('wp_info') == payload['wp_info'] and
                    old.get('objective_phase') == payload['objective_phase'] and
                    old.get('target_color') == payload['target_color'] and
                    old.get('perception_policy') == payload['perception_policy'] and
                    old.get('gate_count') == payload['gate_count'] and
                    old.get('command_lock') == payload['command_lock'] and
                    old.get('failsafe_state') == payload['failsafe_state'] and
                    old.get('estop_state') == payload['estop_state'] and
                    old.get('estop_source') == payload['estop_source'] and
                    old.get('v_target') == payload['v_target'] and
                    old.get('heading_target') == payload['heading_target'] and
                    old.get('timeout_count') == payload['timeout_count'] and
                    old.get('camera_ready') == payload['camera_ready'] and
                    old.get('lidar_ready') == payload['lidar_ready'] and
                    old.get('sensor_fusion') == payload['sensor_fusion'] and
                    old.get('dynamic_speed_profile') == payload['dynamic_speed_profile'] and
                    old.get('wind_assist') == payload['wind_assist'] and
                    old.get('horizon_lock') == payload['horizon_lock'] and
                    old.get('camera_adaptation') == payload['camera_adaptation'] and
                    old.get('autonomy_health') == payload['autonomy_health'] and
                    old.get('virtual_anchor') == payload['virtual_anchor']):
                    logical_state_changed = False
            
            # Write only if logical state changed or 1 second heartbeat timeout exceeded
            if logical_state_changed or (now - self._last_state_write_time >= 1.0):
                os.makedirs(CONTROL_DIR, exist_ok=True)
                # Use temporary file with atomic rename for safe I/O
                import tempfile
                try:
                    # Create temp file in same directory (for atomic rename)
                    temp_fd, tmp_path = tempfile.mkstemp(dir=CONTROL_DIR, prefix='.tmp_state_', suffix='.json')
                    with os.fdopen(temp_fd, 'w', encoding='utf-8') as f:
                        json.dump(payload, f)
                        f.flush()
                        os.fsync(f.fileno())  # Force disk write
                    # Atomic rename
                    os.replace(tmp_path, STATE_FILE)
                except Exception as tmp_exc:
                    # Fallback: direct write (less safe but better than failure)
                    with open(STATE_FILE, 'w', encoding='utf-8') as f:
                        json.dump(payload, f)
                
                # Update cache
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
                        candidate.wait_heartbeat(timeout=5)
                        self.master = candidate
                        print(f"[OK] [MAV] SIM endpoint baglandi: {endpoint}")
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
            self.last_heartbeat_time = time.monotonic()
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
                    # CRITICAL FIX: Match ros_gz_bridge /scan publisher QoS (BEST_EFFORT + KEEP_LAST=10)
                    lidar_qos = QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=10
                    )
                    self.create_subscription(
                        LaserScan, "/scan", self.cb, lidar_qos
                    )

                def cb(self, msg):
                    self.parent.last_lidar_time = time.monotonic()
                    left = 99.0
                    center = 99.0
                    right = 99.0
                    points = []
                    front_samples = []
                    angle = msg.angle_min
                    for rng in msg.ranges:
                        try:
                            r = float(rng)
                        except (TypeError, ValueError):
                            angle += msg.angle_increment
                            continue
                        if not math.isfinite(r) or not (0.15 < r < 20.0):
                            angle += msg.angle_increment
                            continue
                        deg = math.degrees(angle)
                        if -90 <= deg <= 90:
                            x_m = r * math.cos(angle)
                            y_m = r * math.sin(angle)
                            points.append((x_m, y_m))
                            front_samples.append((deg, float(r)))
                            if deg > 15:
                                left = min(left, r)
                            elif deg < -15:
                                right = min(right, r)
                            else:
                                center = min(center, r)
                        angle += msg.angle_increment
                    hold = self.parent._lidar_sector_hold
                    if left < 98.5:
                        hold["left"] = left
                    elif hold["left"] is not None:
                        left = hold["left"]
                    if center < 98.5:
                        hold["center"] = center
                    elif hold["center"] is not None:
                        center = hold["center"]
                    if right < 98.5:
                        hold["right"] = right
                    elif hold["right"] is not None:
                        right = hold["right"]
                    self.parent.lidar_left_dist = left
                    self.parent.lidar_center_dist = center
                    self.parent.lidar_right_dist = right
                    self.parent.min_obstacle_distance = min(left, center, right)
                    self.parent.obstacle_detected = center < D_MIN_M
                    self.parent._map_points = points
                    self.parent._lidar_front_samples = front_samples

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
                return False
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )
            print(f"[MODE] [MODE] {mode_name}")
            return True
        except Exception as exc:
            print(f"[WARN] [MODE] Hata ({mode_name}): {exc}")
            return False

    def _arm(self):
        if not self.master:
            return True
        try:
            from pymavlink import mavutil

            def wait_until_armed(timeout_s):
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline:
                    self._drain_mav_messages()
                    if self.master.motors_armed():
                        print("[OK] [ARM] Arac armed")
                        return True
                    time.sleep(0.1)
                return False

            self.master.arducopter_arm()
            if wait_until_armed(6.0):
                return True

            if os.environ.get("USV_SIM") == "1":
                print("[WARN] [ARM] Normal arm timeout, force-arm deneniyor")
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1,
                    21196,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
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

    def _set_rc_override(self, left_pwm, right_pwm):
        left_pwm = int(clamp(left_pwm, 1100, 1900))
        right_pwm = int(clamp(right_pwm, 1100, 1900))
        if not self.master:
            if time.monotonic() % 5.0 < 0.1:
                print(f"[WARN] [MOTOR] master is None, cannot send motor command")
            return
        try:
            # RC CHANNELS OVERRIDE (stable path for motorboat)
            # CH1 = left motor, CH3 = right motor (1100-1900 μs)
            # Channels 0-7 (8 total), unused channels = 0
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                int(left_pwm),    # CH1 (left motor)
                0,                # CH2 (unused)
                int(right_pwm),   # CH3 (right motor)
                0,                # CH4–CH8 (unused)
                0,
                0,
                0,
                0
            )
            self._last_sent_rc_override = {
                "ch1": int(left_pwm),
                "ch3": int(right_pwm),
                "ts": time.monotonic(),
            }

            now = time.monotonic()
            if now % 2.0 < 0.1:  # Log every 2 sec to avoid spam
                print(f"[DEBUG] [MOTOR] RC_OVERRIDE CH1={left_pwm} CH3={right_pwm}")
            last_left = int(getattr(self, "_last_motor_log_left_pwm", left_pwm))
            last_right = int(getattr(self, "_last_motor_log_right_pwm", right_pwm))
            last_ts = float(getattr(self, "_last_motor_log_ts", 0.0) or 0.0)
            if abs(left_pwm - last_left) >= 8 or abs(right_pwm - last_right) >= 8 or (now - last_ts) >= 1.0:
                print(f"[MOTOR] RC override: left={left_pwm} right={right_pwm}")
                self._last_motor_log_left_pwm = left_pwm
                self._last_motor_log_right_pwm = right_pwm
                self._last_motor_log_ts = now
        except AttributeError as exc:
            print(f"[ERROR] [MOTOR] master attribute missing: {exc}")
        except Exception as exc:
            print(f"[WARN] [MOTOR] Motor command error: {exc}")

    def _command_speed_heading(self, speed_mps, heading_error_deg):
        """
        Command motor speed and heading. 
        
        COMMAND LOCK: If command_lock=True (mission safety), reject autonomy commands.
        RC OVERRIDE: If RC sticks manually deflected (ch1/ch3 away from neutral), use RC values directly.
        
        Per AGENTS.md Section 2.1: RC override ALWAYS preempts autonomy.
        E-stop (RC7) always works and bypasses all checks.
        """
        # CHECK 1: Command lock (mission safety gate)
        if self.command_lock:
            self._set_rc_override(1500, 1500)  # Neutral position
            if not hasattr(self, '_last_motor_lock_log'):
                self._last_motor_lock_log = 0.0
            now = time.monotonic()
            if (now - self._last_motor_lock_log) >= 5.0:  # Log every 5 sec
                print(f"[MOTOR_LOCK] LOCKED - commanded(speed={speed_mps:.2f}m/s, hdg_err={heading_error_deg:.1f}°) → neutral. Reason: {self.hold_reason}")
                self._last_motor_lock_log = now
            return
        
        # CHECK 2: RC manual override (absolute preemption per AGENTS.md 2.1)
        if self._is_rc_stick_active():
            # RC sticks are being manually moved → use RC values directly, ignore autonomy
            rc_ch1 = self.rc_channels.get("ch1", 1500)  # Steering
            rc_ch3 = self.rc_channels.get("ch3", 1500)  # Throttle
            self._set_rc_override(rc_ch1, rc_ch3)
            
            # Log RC override (throttled to avoid spam)
            if not hasattr(self, '_last_rc_override_log'):
                self._last_rc_override_log = 0.0
            now = time.monotonic()
            if (now - self._last_rc_override_log) >= 2.0:
                print(f"[RC_OVERRIDE] CH1={rc_ch1} CH3={rc_ch3} (ignoring autonomy speed={speed_mps:.2f}m/s)")
                self._last_rc_override_log = now
            return
        
        # CHECK 3: Sensor fusion policy (AGENTS.md §5.7 derating on low trust)
        trust_bar = self._compute_trust_bar()
        if trust_bar < TRUST_WARN_THRESHOLD:
            # Autonomy health degraded → cap speed to FAILSAFE_SLOW_MPS (0.3 m/s)
            speed_mps = min(float(speed_mps), FAILSAFE_SLOW_MPS)
            if not hasattr(self, '_last_failsafe_log'):
                self._last_failsafe_log = 0.0
            now = time.monotonic()
            if (now - self._last_failsafe_log) >= 3.0:
                print(f"[FAILSAFE] Autonomy health trust={trust_bar:.1f}% < {TRUST_WARN_THRESHOLD}% → speed capped to {FAILSAFE_SLOW_MPS} m/s")
                self._last_failsafe_log = now
        
        # NORMAL AUTONOMY: No command lock, no RC override, trust OK → execute commanded track
        # Final commanded values (after all checks/caps applied)
        self.v_target = float(speed_mps)
        self.heading_target = (self.current_heading + heading_error_deg) % 360

        heading_abs = abs(float(heading_error_deg))
        throttle_norm = clamp((speed_mps * 210.0) / 400.0, -1.0, 1.0)
        if speed_mps > 0.0:
            if heading_abs >= 85.0:
                throttle_norm = max(min(throttle_norm, 0.28), 0.16)
            elif heading_abs >= 55.0:
                throttle_norm = max(min(throttle_norm, 0.42), 0.22)
            elif heading_abs >= 30.0:
                throttle_norm = max(min(throttle_norm, 0.58), 0.30)

        if heading_abs < 15.0:
            steer_gain = 2.0  # Reduced from 2.4 to prevent overshoot near target
        elif heading_abs < 35.0:
            steer_gain = 2.5  # Reduced from 3.0 to reduce oscillation
        else:
            steer_gain = 3.0  # Reduced from 3.6 for large errors
        steer_norm = clamp((heading_error_deg * steer_gain) / 400.0, -1.0, 1.0)

        # SKID STEER KINEMATICS: test original formula
        left_mix = throttle_norm - steer_norm
        right_mix = throttle_norm + steer_norm
        max_mag = max(abs(left_mix), abs(right_mix))
        if max_mag > 1.0:
            left_mix /= max_mag
            right_mix /= max_mag

        left_pwm = int(1500 + left_mix * 400)
        right_pwm = int(1500 + right_mix * 400)
        if os.environ.get("USV_SIM") == "1" and time.monotonic() % 1.0 < 0.1:
            print(
                f"[DEBUG] [MOTOR] SIM mode: left={left_pwm} right={right_pwm} "
                f"(speed={speed_mps:.2f}, hdg_err={heading_error_deg:.1f})"
            )
        self._set_rc_override(left_pwm, right_pwm)

    def stop_motors(self):
        # Stop motors - always works (emergency handler)
        self.v_target = 0.0
        self._set_rc_override(1500, 1500)
        self._disarm()

    def _force_safe_outputs(self, include_estop=False, repeat=6):
        if not self.master:
            return
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
        
        if not hasattr(self, '_mav_message_counts'):
            self._mav_message_counts = {}
        if not hasattr(self, '_last_mav_debug_log'):
            self._last_mav_debug_log = 0.0
        
        try:
            message_count = 0
            while True:
                msg = self.master.recv_match(blocking=False)
                if not msg:
                    break
                message_count += 1
                mtype = msg.get_type()
                log_jsonl("usv_mavlink", False, event="drain_rx", mtype=mtype)
                self._mav_message_counts[mtype] = self._mav_message_counts.get(mtype, 0) + 1
                if mtype == "GLOBAL_POSITION_INT":
                    self.current_lat = msg.lat / 1e7
                    self.current_lon = msg.lon / 1e7
                    self.gps_global_position_int_received = True  # Mark GPS valid for simulation mode (JSON backend)
                    # Set minimal GPS_RAW_INT values if not explicitly set (simulation GPS RAW fallback)
                    if os.environ.get("USV_SIM") == "1" and self.gps_fix_type == 0:
                        self.gps_fix_type = 3  # 3D fix equivalent (from GLOBAL_POSITION_INT)
                        self.gps_satellites_visible = 12  # Simulate 12 satellites visible
                    # Heading with exponential moving average filter (reduces compass drift oscillation)
                    raw_heading = msg.hdg / 100.0
                    if self._heading_ema is None:
                        self._heading_ema = raw_heading
                    else:
                        # EMA: smoothed_heading = alpha * raw + (1 - alpha) * previous
                        self._heading_ema = self._heading_ema * (1.0 - self._heading_ema_alpha) + raw_heading * self._heading_ema_alpha
                    self.current_heading = self._heading_ema
                elif mtype == "GPS_RAW_INT":
                    self.gps_satellites_visible = int(getattr(msg, "satellites_visible", 0) or 0)
                    self.gps_fix_type = int(getattr(msg, "fix_type", 0) or 0)
                elif mtype == "ATTITUDE":
                    self.current_roll_deg = math.degrees(getattr(msg, "roll", 0.0))
                    self.current_pitch_deg = math.degrees(getattr(msg, "pitch", 0.0))
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
                            self._heading_ema = self._heading_ema * (1.0 - self._heading_ema_alpha) + yaw_deg * self._heading_ema_alpha
                        self.current_heading = self._heading_ema
                elif mtype == "HEARTBEAT":
                    if msg.get_srcSystem() == self.master.target_system:
                        self.last_heartbeat_time = time.monotonic()
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
            "target_detected_raw": False,
            "target_bearing_error_deg_raw": 0.0,
            "target_area_norm_raw": 0.0,
            "target_detected": False,
            "target_bearing_error_deg": 0.0,
            "target_area_norm": 0.0,
            "camera_adaptation": dict(self.camera_adaptation),
        }
        data = default
        try:
            if os.path.exists(CAMERA_STATUS_FILE):
                with open(CAMERA_STATUS_FILE, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
                data = {**default, **loaded}
        except Exception as exc:
            self._bump_error("camera_state_read_error", f"[WARN] [CAM] Status okuma hatasi: {exc}")
            data = default

        for key in (
            "gate_detected",
            "gate_stable_s",
            "gate_center_bearing_deg",
            "gate_passed_event",
            "yellow_obstacle_detected",
            "yellow_obstacle_bearing_deg",
            "yellow_obstacle_area_norm",
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
            "target": bool(perception_policy.get("target", default_policy["target"])),
        }
        data["objective_phase"] = str(data.get("objective_phase", default["objective_phase"]) or default["objective_phase"])
        data["perception_policy"] = perception_policy
        gate_actionable = bool(data.get("gate_actionable", perception_policy["gate"]))
        yellow_actionable = bool(data.get("yellow_actionable", perception_policy["yellow_obstacle"]))
        target_actionable = bool(data.get("target_actionable", perception_policy["target"]))
        data["gate_actionable"] = gate_actionable
        data["yellow_actionable"] = yellow_actionable
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
        if not target_actionable:
            data["target_detected"] = False
            data["target_bearing_error_deg"] = 0.0
            data["target_area_norm"] = 0.0

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

        self.camera_ready = bool(frame_age_s < CAMERA_FRAME_TIMEOUT_S)
        self.lidar_ready = bool(
            self.lidar_available and (time.monotonic() - self.last_lidar_time) < LIDAR_READY_TIMEOUT_S
        )
        if self.simulation_mode:
            self.camera_ready = True
            self.lidar_ready = True

        fusion_scope_active = bool(FUSION_ENABLED and self.state in (self.STATE_PARKUR2, self.STATE_PARKUR3))
        if fusion_scope_active:
            data = self._apply_camera_lidar_fusion(data)
        else:
            self._gate_fusion_hold_since = None
            self._target_fusion_hold_since = None
            self._gate_ghost_latched = False
            self._target_ghost_latched = False
            self._gate_camera_only_since = None
            self._target_camera_only_since = None

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
        if self.simulation_mode or not self.master:
            self.heartbeat_age_s = 0.0
            self.link_heartbeat_age_s = 0.0
            self.link_heartbeat_source = "simulation"
            self.failsafe_state = "normal"
            if not hasattr(self, '_sim_watchdog_logged'):
                print(f"[WATCHDOG_SIM] Watchdog in simulation mode (no real MAVLink)")
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

        if link_fail and self.link_heartbeat_source == "telemetry" and self.failsafe_state != "hold":
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
        else:
            self.failsafe_state = "normal"

    def _trigger_estop(self, source, force_rc7=False):
        if self.estop_latched:
            return
        print(f"[ESTOP] [ESTOP] Tetiklendi: {source} (force_rc7={force_rc7})")
        print(f"[ESTOP_DEBUG] Mission active={self.mission_active}, state={self.state}, reason=investigate later")
        self.estop_latched = True
        self.estop_source = source
        self.command_lock = True
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
        self.v_target = 0.0
        self.heading_target = self.current_heading
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
        self._write_state()

    def _distance_and_heading_error(self, target_lat, target_lon):
        lat = self.current_lat
        lon = self.current_lon
        # Simülasyon modunda GLOBAL_POSITION_INT henüz gelmemişse SIM_HOME kullan
        if self.simulation_mode and abs(lat) < 1e-6 and abs(lon) < 1e-6:
            lat = getattr(self, '_sim_home_lat', -35.363262)
            lon = getattr(self, '_sim_home_lon', 149.165237)
            if not hasattr(self, '_sim_home_gps_warn_ts') or (time.monotonic() - self._sim_home_gps_warn_ts) >= 5.0:
                print(f"[GPS_WARN] GLOBAL_POSITION_INT beklenirken SIM_HOME pozisyonu kullaniliyor: ({lat:.7f},{lon:.7f})")
                self._sim_home_gps_warn_ts = time.monotonic()
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return 9999.0, 0.0
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return 9999.0, 0.0
        dist = haversine_distance(lat, lon, target_lat, target_lon)
        target_bearing = calculate_bearing(lat, lon, target_lat, target_lon)
        heading_error = normalize_heading_error(target_bearing - self.current_heading)
        return dist, heading_error

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
        if os.path.exists(fp):
            try:
                with open(fp, "r", encoding="utf-8") as f:
                    raw_data = json.load(f)

                mission_target_color = ""
                if isinstance(raw_data, list):
                    coords = validate_coordinate_mission(raw_data)
                    self.waypoints_p1, self.waypoints_p2, self.waypoints_p3 = split_mission_waypoints(
                        coords,
                        validate_lengths=False,
                    )
                    self.mission_input_format = MISSION_INPUT_FORMAT
                    self.mission_upload_source = "local_flat_file"
                    self.mission_split_profile = get_mission_split_profile(len(coords))
                elif isinstance(raw_data, dict):
                    mission = adapt_mission_to_structured(raw_data, strict=True)
                    self.waypoints_p1 = mission["parkur1"]
                    self.waypoints_p2 = mission["parkur2"]
                    self.waypoints_p3 = mission["parkur3"]
                    mission_target_color = str(mission.get("target_color") or "").strip().upper()
                    self.mission_input_format = mission.get("_mission_input_format", "structured_legacy")
                    self.mission_upload_source = mission.get("_adapter_source", "structured_legacy")
                    self.mission_split_profile = dict(mission.get("_split_profile") or {})
                else:
                    raise ValueError(f"Mission must be list or object, got {type(raw_data).__name__}")

                total_count = len(self.waypoints_p1) + len(self.waypoints_p2) + len(self.waypoints_p3)
                if not self.mission_split_profile:
                    self.mission_split_profile = get_mission_split_profile(total_count)
                target_state = load_target_state(TARGET_STATE_FILE)
                target_from_state = str(target_state.get("target_color") or "").strip().upper()
                self.target_color = target_from_state or mission_target_color or self.target_color or "RED"
                self.mission_validated_at_timestamp = time.time()

                print(
                    f"[OK] [GOREV] VALIDATED P1={len(self.waypoints_p1)} P2={len(self.waypoints_p2)} "
                    f"P3={len(self.waypoints_p3)} Hedef={self.target_color} "
                    f"format={self.mission_input_format} split=P2:{MISSION_SPLIT_P2_COUNT}/P3:{MISSION_SPLIT_P3_COUNT}"
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
            self.waypoints_p1, self.waypoints_p2, self.waypoints_p3 = split_mission_waypoints(
                coords,
                validate_lengths=False,
            )
            self.mission_input_format = MISSION_INPUT_FORMAT
            self.mission_upload_source = "sim_default_flat_file"
            self.mission_split_profile = get_mission_split_profile(len(coords))
            target_state = load_target_state(TARGET_STATE_FILE)
            self.target_color = str(target_state.get("target_color") or self.target_color or "RED").strip().upper() or "RED"
            self.mission_validated_at_timestamp = time.time()
            print(
                f"[SIM] [GOREV] Varsayilan gorev yuklendi "
                f"P1={len(self.waypoints_p1)} P2={len(self.waypoints_p2)} P3={len(self.waypoints_p3)} "
                f"split=P2:{MISSION_SPLIT_P2_COUNT}/P3:{MISSION_SPLIT_P3_COUNT}"
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

    def start_mission(self):
        if self.mission_active:
            print("[WARN] [GOREV] Zaten aktif")
            return False
        if not self.load_mission(MISSION_FILE):
            print("❌ [MISSION] Guncel mission yuklenemedi")
            return False
        self._drain_mav_messages()
        self._read_camera_status()
        self._update_watchdog()
        self._refresh_health_check()

        if not self._clear_estop_if_safe():
            return False

        if not self._check_rc_connected():
            print("❌ [GOREV] RC bagli degil")
            return False

        self._refresh_health_check()
        if not self.health_ready:
            missing = ", ".join(self.health_missing) if self.health_missing else "unknown"
            print(f"❌ [READY] HEALTH_CHECK gecmedi: {missing}")
            self._write_state()
            return False

        self.mission_active = True
        self.mission_start_time = time.time()
        self.command_lock = True
        self.state = self.STATE_PARKUR1
        self.hold_reason = "NONE"
        self.failsafe_state = "normal"
        self._set_dynamic_speed_idle()
        self._set_wind_assist_idle()
        self._set_horizon_lock_idle(reason="mission_active", channel="none")
        self._set_virtual_anchor_idle(reason="mission_active", clear_center=True)
        self.gate_count = 0
        self.timeout_count = 0
        self._gate_event_start = None
        self._gate_event_latched = False
        self._wp_target = "--"
        self._wp_info = "-- / --"
        if not self._arm():
            print("❌ [START] Arming basarisiz")
            self.mission_active = False
            self.command_lock = False
            self.state = self.STATE_IDLE
            self.stop_motors()
            self._write_state()
            return False
        # Arming basarili → command_lock kutulayalim
        self.command_lock = False
        self._write_state()
        print("[START] [GOREV] Baslatildi")
        return True

    def _navigate_p1_waypoint(self, lat, lon):
        self._sim_dist = 14.0
        hold_start = None
        next_invalid_log = 0.0
        while self.mission_active:
            if self._check_abort():
                return False
            dist, heading_err = self._distance_and_heading_error(lat, lon)
            self._wp_target = f"{lat:.7f}, {lon:.7f}"
            if dist >= 9000.0:
                self._command_speed_heading(0.0, 0.0)
                if time.monotonic() >= next_invalid_log:
                    print("[WARN] [P1] Gecersiz navigasyon verisi, beklemede")
                    next_invalid_log = time.monotonic() + 1.5
                self._write_state()
                time.sleep(LOOP_DT)
                continue
            if dist <= R_WP_M:
                if hold_start is None:
                    hold_start = time.monotonic()
                self._command_speed_heading(0.0, 0.0)
                if (time.monotonic() - hold_start) >= T_HOLD_S:
                    print(f"[OK] [P1] WP_REACHED dist={dist:.2f}m hold={T_HOLD_S:.1f}s")
                    self.stop_motors()
                    return True
            else:
                hold_start = None
                base_speed = P1_SPEED_APPROACH_MPS if dist < 8.0 else P1_SPEED_CRUISE_MPS

                # Lidar avoidance and wind assist: GUIDED mode only (test mode)
                # In race mode (AUTO), rely on Pixhawk onboard obstacle avoidance
                if USV_MODE != USV_MODE_RACE:
                    # P1: Center-sector only (ignore side clutter from left/right sensors)
                    center_d = self.lidar_center_dist
                    left_d = self.lidar_left_dist
                    right_d = self.lidar_right_dist
                    lidar_emergency = center_d < D_MIN_M
                    in_warn = center_d < P2_LIDAR_WARN_M

                    # Center escape heading: if obstacle ahead, choose available side
                    if in_warn or lidar_emergency:
                        urgency = clamp((P2_LIDAR_WARN_M - center_d) / max(P2_LIDAR_WARN_M - D_MIN_M, 0.01), 0.0, 1.0)
                        if left_d > right_d:
                            esc_l = -P2_ESCAPE_MAX_DEG * urgency * 0.6  # Go left at 60% intensity
                        else:
                            esc_l = P2_ESCAPE_MAX_DEG * urgency * 0.6   # Go right at 60% intensity
                    else:
                        esc_l = 0.0

                    # P1 goal is corridor progression only. Yellow-obstacle and
                    # target-color semantics belong to later phases.
                    esc_c = 0.0
                    avoid = self._p1_avoid_heading_smoothed(esc_l, esc_c, in_warn, lidar_emergency)

                    # Smooth blending: waypoint bearing + obstacle avoidance (not binary cutoff)
                    # in_warn=True → 30% waypoint, 70% avoidance
                    # in_warn=False → 100% waypoint, 0% avoidance
                    blend_factor = 0.3 if in_warn or lidar_emergency else 0.0
                    heading_err_nav = (1.0 - blend_factor) * heading_err + blend_factor * avoid
                else:
                    # Race mode: no Pi lidar override, pure Pixhawk AUTO
                    lidar_emergency = False
                    in_warn = False
                    heading_err_nav = heading_err

                center_obstacle = in_warn or lidar_emergency if USV_MODE != USV_MODE_RACE else False
                corrected_heading_err = self._apply_wind_assist(
                    heading_err_nav,
                    "P1",
                    base_speed,
                    center_obstacle=center_obstacle or lidar_emergency if USV_MODE != USV_MODE_RACE else False,
                )
                speed = self._apply_dynamic_speed(base_speed, corrected_heading_err, "P1")
                if (lidar_emergency or in_warn) and USV_MODE != USV_MODE_RACE:
                    speed = min(speed, FAILSAFE_SLOW_MPS if lidar_emergency else base_speed * 0.85)
                self._command_speed_heading(speed, corrected_heading_err)
            self._write_state()
            time.sleep(LOOP_DT)
        return False

    def _wait_p2_ready(self):
        print("[CHECK] [P1->P2] Kamera/Lidar hazirlik kontrolu")
        next_log = 0.0
        deadline = time.monotonic() + 10.0
        while self.mission_active:
            if self._check_abort():
                return False
            if self.camera_ready and self.lidar_ready:
                print("[OK] [P1->P2] Hazirlik tamam")
                return True
            if time.monotonic() >= deadline:
                print(
                    f"[WARN] [P1->P2] Sensor bekleme zamani doldu "
                    f"(camera={self.camera_ready}, lidar={self.lidar_ready}), devam ediliyor"
                )
                return True
            self._command_speed_heading(P2_WAIT_SPEED_MPS, 0.0)
            if time.monotonic() >= next_log:
                remaining = max(0, deadline - time.monotonic())
                print(
                    f"[WAIT] [P1->P2] Bekleniyor camera_ready={self.camera_ready} "
                    f"lidar_ready={self.lidar_ready} ({remaining:.0f}s kaldi)"
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

    def _p2_lidar_escape_heading_sectors_fallback(self):
        """Scan ornegi yoksa sol/orta/sag sektor min mesafeleriyle kacinma."""
        ld = self.lidar_left_dist
        cd = self.lidar_center_dist
        rd = self.lidar_right_dist
        fm = min(ld, cd, rd)
        if fm >= P2_LIDAR_WARN_M:
            return 0.0
        urgency = clamp(
            (P2_LIDAR_WARN_M - fm) / max(P2_LIDAR_WARN_M - 0.15, 0.01),
            0.0,
            1.0,
        )
        if abs(ld - rd) < 0.05:
            return P2_ESCAPE_MAX_DEG * urgency * 1.0
        return P2_ESCAPE_MAX_DEG * urgency * (1.0 if rd > ld else -1.0)

    def _p2_lidar_escape_heading_deg(self):
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
        if min_r > P2_LIDAR_WARN_M:
            self._p2_local_minima_start_ts = None
            self._p2_local_minima_active = False
            return self._p2_lidar_escape_heading_sectors_fallback()
        
        urgency = clamp(
            (P2_LIDAR_WARN_M - min_r) / max(P2_LIDAR_WARN_M - 0.15, 0.01),
            0.0,
            1.0,
        )
        
        now = time.monotonic()
        if min_r < D_MIN_M:
            if self._p2_local_minima_start_ts is None:
                self._p2_local_minima_start_ts = now
                self._p2_local_minima_active = True
            elif (now - self._p2_local_minima_start_ts) >= P2_LOCAL_MINIMA_TIMEOUT_S:
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
        
        if self._p2_local_minima_active and self._p2_local_minima_start_ts is not None:
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
            ld = self.lidar_left_dist
            rd = self.lidar_right_dist
            if ld >= 90.0 and rd >= 90.0:
                return 0.0
            if abs(ld - rd) < 0.05:
                return P2_ESCAPE_MAX_DEG * urgency * 1.0
            return P2_ESCAPE_MAX_DEG * urgency * (1.0 if rd > ld else -1.0)
        return P2_ESCAPE_MAX_DEG * urgency * clamp(min_deg / 45.0, -1.0, 1.0)

    def _p2_camera_yellow_escape_deg(self):
        """Sari engel goruntu merkezinden sapma; kacinma -bearing ile."""
        if not self.camera_status.get("yellow_obstacle_detected"):
            return 0.0
        try:
            area = float(self.camera_status.get("yellow_obstacle_area_norm", 0.0))
            bear = float(self.camera_status.get("yellow_obstacle_bearing_deg", 0.0))
        except (TypeError, ValueError):
            return 0.0
        if area < 0.004:
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

    def _p2_avoid_heading_smoothed(self, esc_l, esc_c, in_warn, lidar_emergency):
        """P2: lidar+kamera kaçınma komutunu sürekli sıçramayı azaltmak için yumuşat."""
        avoid_raw = esc_l + P2_CAM_YELLOW_WEIGHT * esc_c
        if in_warn or lidar_emergency:
            beta = 0.48
            self._p2_avoid_smooth = beta * avoid_raw + (1.0 - beta) * self._p2_avoid_smooth
            return self._p2_avoid_smooth
        self._p2_avoid_smooth = 0.0
        return 0.0

    def _navigate_p2_waypoint(self, lat, lon):
        self._sim_dist = 12.0
        next_invalid_log = 0.0
        while self.mission_active:
            if self._check_abort():
                return False
            self._track_gate_event()
            dist, gps_heading_err = self._distance_and_heading_error(lat, lon)
            self._wp_target = f"{lat:.7f}, {lon:.7f}"
            if dist >= 9000.0:
                self._command_speed_heading(0.0, 0.0)
                if time.monotonic() >= next_invalid_log:
                    print("[WARN] [P2] Gecersiz navigasyon verisi, beklemede")
                    next_invalid_log = time.monotonic() + 1.5
                self._write_state()
                time.sleep(LOOP_DT)
                continue
            if dist <= R_WP_M:
                self.stop_motors()
                return True

            base_speed = P2_CRUISE_MPS if dist >= 8.0 else P2_WAIT_SPEED_MPS

            left_d = self.lidar_left_dist
            center_d = self.lidar_center_dist
            right_d = self.lidar_right_dist
            front_min = min(left_d, center_d, right_d)
            lidar_emergency = front_min < D_MIN_M
            in_warn = front_min < P2_LIDAR_WARN_M

            esc_l = self._p2_lidar_escape_heading_deg()
            esc_c = self._p2_camera_yellow_escape_deg()
            avoid = self._p2_avoid_heading_smoothed(esc_l, esc_c, in_warn, lidar_emergency)

            heading_err = gps_heading_err
            gate_detected = bool(self.camera_status.get("gate_detected", False))
            gate_stable_s = float(self.camera_status.get("gate_stable_s", 0.0))
            
            # P2 heading priority: obstacle > gate > waypoint, but blend instead of binary cutoff
            if lidar_emergency:
                # Emergency: 100% avoidance
                heading_err = avoid
            elif in_warn:
                # Warning: 60% avoidance, 40% navigation (waypoint or gate)
                if gate_detected and gate_stable_s >= P2_STABLE_S:
                    nav_bearing = float(self.camera_status.get("gate_center_bearing_deg", 0.0))
                else:
                    nav_bearing = gps_heading_err
                heading_err = 0.4 * nav_bearing + 0.6 * avoid
            elif gate_detected and gate_stable_s >= P2_STABLE_S:
                # No obstacle: gate bearing preferred
                heading_err = float(self.camera_status.get("gate_center_bearing_deg", 0.0))
            else:
                # No obstacle, no gate: pure waypoint bearing
                heading_err = gps_heading_err

            center_obstacle = center_d < D_MIN_M

            corrected_heading_err = self._apply_wind_assist(
                heading_err,
                "P2",
                base_speed,
                center_obstacle=center_obstacle or lidar_emergency,
            )
            speed = self._apply_dynamic_speed(base_speed, corrected_heading_err, "P2")
            if lidar_emergency or in_warn:
                speed = min(speed, FAILSAFE_SLOW_MPS if lidar_emergency else P2_CRUISE_MPS * 0.85)

            self._command_speed_heading(speed, corrected_heading_err)
            self._write_state()
            time.sleep(LOOP_DT)
        return False

    def _run_p3_attempt(self, timeout_s):
        start_t = time.monotonic()
        contact_start = None
        next_invalid_log = 0.0
        wp = self.waypoints_p3[0] if self.waypoints_p3 else [self.current_lat, self.current_lon]
        while self.mission_active and (time.monotonic() - start_t) < timeout_s:
            if self._check_abort():
                return False
            dist, gps_heading_err = self._distance_and_heading_error(wp[0], wp[1])
            target_detected = bool(self.camera_status.get("target_detected", False))
            target_area = float(self.camera_status.get("target_area_norm", 0.0))
            target_bearing = float(self.camera_status.get("target_bearing_error_deg", 0.0))
            if dist >= 9000.0 and not target_detected:
                self._command_speed_heading(0.0, 0.0)
                if time.monotonic() >= next_invalid_log:
                    print("[WARN] [P3] Gecersiz navigasyon/hedef verisi, beklemede")
                    next_invalid_log = time.monotonic() + 1.5
                self._write_state()
                time.sleep(LOOP_DT)
                continue
            if target_detected:
                heading_err = target_bearing
                speed = max(0.25, P3_MAX_SPEED_MPS * (1.0 - clamp(target_area, 0.0, 0.85)))
            else:
                heading_err = gps_heading_err
                speed = 0.6
            if dist < 2.0:
                speed = min(speed, 0.6)
            self._command_speed_heading(min(speed, P3_MAX_SPEED_MPS), heading_err)

            proximity_ok = (dist <= 1.0) or (target_area >= 0.20)
            if proximity_ok:
                if contact_start is None:
                    contact_start = time.monotonic()
                elif (time.monotonic() - contact_start) >= 0.6:
                    print("[ENGAGE] [P3] Angajman tamam")
                    self.stop_motors()
                    return True
            else:
                contact_start = None

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

    def run_parkur1(self):
        print("=" * 52)
        print("  [PARKUR-1] GUIDED WAYPOINT + LIDAR KACINMA")
        print("=" * 52)
        self.state = self.STATE_PARKUR1
        self._p1_avoid_smooth = 0.0
        p1_mode = "GUIDED" if USV_MODE != USV_MODE_RACE else "AUTO"
        if not self._set_mode(p1_mode):
            print(f"❌ [P1] {p1_mode} moda gecilemedi")
            return False
        
        # RACE COMPLIANCE: P1 in race mode is pure Pixhawk AUTO.
        # No Pi override, no AUTO→GUIDED fallback, no lidar avoidance.
        # If AUTO fails (waypoint timeout, Pixhawk unresponsive), return False → mission enters HOLD.
        if USV_MODE == USV_MODE_RACE:
            assert p1_mode == "AUTO", "Race mode P1 must be pure Pixhawk AUTO"
            print("[RACE] [P1] Pure Pixhawk AUTO mode - no Pi takeover or fallback")
        
        self._write_state()
        if not self.waypoints_p1:
            print("[WARN] [P1] Waypoint yok")
            return True
        for idx, wp in enumerate(self.waypoints_p1, start=1):
            self._wp_info = f"{idx} / {len(self.waypoints_p1)}"
            print(f"[P1] Waypoint {idx}/{len(self.waypoints_p1)}")
            if not self._navigate_p1_waypoint(wp[0], wp[1]):
                return False
        print("[OK] [P1] Tamam")
        return True

    def run_parkur2(self):
        print("\n" + "=" * 52)
        print("  [PARKUR-2] GUIDED KAPI + ENGEL KACINMA")
        print("=" * 52)
        self.state = self.STATE_PARKUR2
        self._p2_avoid_smooth = 0.0
        self._p2_local_minima_start_ts = None
        self._p2_local_minima_active = False
        self._write_state()
        if not self._wait_p2_ready():
            return False
        if not self._set_mode("GUIDED"):
            print("❌ [P2] GUIDED moda gecilemedi")
            return False
        if not self.waypoints_p2:
            print("[WARN] [P2] Waypoint yok")
            return True
        for idx, wp in enumerate(self.waypoints_p2, start=1):
            self._wp_info = f"{idx} / {len(self.waypoints_p2)}"
            if not self._navigate_p2_waypoint(wp[0], wp[1]):
                return False
            self._track_gate_event()
            self._write_state()
        if self.gate_count <= 0:
            print("[WARN] [P2] gate_gecildi olayi yok; koridor waypoint + kacınma ile tamamlandi")
        print(f"[OK] [P2] Tamam gate={self.gate_count}")
        return True

    def run_parkur3(self):
        print("\n" + "=" * 52)
        print("  [PARKUR-3] HSV HEDEFLEME + ANGAJMAN")
        print("=" * 52)
        self.state = self.STATE_PARKUR3
        self._set_dynamic_speed_idle()
        self._set_wind_assist_idle()
        if not self._set_mode("GUIDED"):
            print("❌ [P3] GUIDED moda gecilemedi")
            return False
        self._write_state()
        if not self.waypoints_p3:
            print("[WARN] [P3] Hedef waypoint yok")
            return True
        if self._run_p3_attempt(P3_TIMEOUT_S):
            return True
        for retry in range(P3_RETRY_COUNT):
            print(f"[RETRY] [P3] Retry {retry + 1}/{P3_RETRY_COUNT}")
            if self._run_p3_attempt(P3_RETRY_S):
                return True
        self._retreat_and_hold()
        return True

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
                heading_deg = math.degrees(heading_rad) % 360
                # EMA filter: α=0.40 for faster heading response (reduces overshoot)
                alpha = 0.40
                old_hdg = self.current_heading
                self.current_heading = alpha * heading_deg + (1.0 - alpha) * self.current_heading
                
                # Log heading changes (every 2s or when delta > 2°)
                now = time.monotonic()
                if not hasattr(self, '_last_hdg_log'):
                    self._last_hdg_log = now
                    self._last_hdg_logged = old_hdg
                if (now - self._last_hdg_log) >= 2.0 or abs(self.current_heading - self._last_hdg_logged) >= 2.0:
                    print(f"[HDG_UPDATE] Gazebo={heading_deg:.1f}° → current_heading={self.current_heading:.1f}° (was {old_hdg:.1f}°)")
                    self._last_hdg_log = now
                    self._last_hdg_logged = self.current_heading
        except Exception as exc:
            if not hasattr(self, '_hdg_error_logged'):
                print(f"[WARN] [HDG_UPDATE] Error reading vehicle_position: {exc}")
                self._hdg_error_logged = True

    def _check_collision(self):
        """Detect: moving with speed command but vessel stuck against obstacle."""
        now = time.time()
        
        # Cooldown: don't log same collision repeatedly
        if now - self._last_collision_ts < self._collision_cooldown_s:
            return
        
        # Get current position from vehicle_position.json
        try:
            pos_file = Path(CONTROL_DIR) / "vehicle_position.json"
            pos_data = json.load(open(pos_file))
            current_pos = (pos_data['pos_x'], pos_data['pos_y'])
        except:
            return
        
        # Has speed command?
        has_speed_cmd = self.v_target > self._collision_speed_thresh_mps
        
        # Position change from last check
        if self._last_collision_check_pos != (0.0, 0.0):
            dx = current_pos[0] - self._last_collision_check_pos[0]
            dy = current_pos[1] - self._last_collision_check_pos[1]
            pos_change = math.sqrt(dx*dx + dy*dy)
        else:
            self._last_collision_check_pos = current_pos
            return
        
        # Collision = moving command, minimal position change, close lidar
        is_collision = (
            has_speed_cmd and 
            pos_change < self._collision_position_thresh_m and
            self.lidar_center_dist < 1.0
        )
        
        if is_collision:
            log_jsonl("usv_collision", False, 
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

    def run(self):
        while self.mission_active:
            self._check_time()
            if self.state == self.STATE_PARKUR1:
                if self.run_parkur1():
                    if not self._wait_for_next_parkur("P1", "P2"):
                        break
                    self.state = self.STATE_PARKUR2
                else:
                    break
            elif self.state == self.STATE_PARKUR2:
                if self.run_parkur2():
                    if not self._wait_for_next_parkur("P2", "P3"):
                        break
                    self.state = self.STATE_PARKUR3
                else:
                    break
            elif self.state == self.STATE_PARKUR3:
                if self.run_parkur3():
                    self.state = self.STATE_COMPLETED
                else:
                    break
            elif self.state in (self.STATE_COMPLETED, self.STATE_HOLD):
                break
            # Update compass heading from Gazebo every tick
            self._update_heading_from_vehicle_position()
            # Check for collisions every tick
            self._check_collision()
            self._write_state()

        if self.state == self.STATE_COMPLETED:
            print("[DONE] [GOREV] Tum parkurlar tamamlandi")
            self._wp_target = "TAMAMLANDI"
            self._wp_info = "-- / --"
        self.mission_active = False
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

        while True:
            usv._drain_mav_messages()
            usv._read_camera_status()
            usv._compute_trust_bar()
            usv._update_watchdog()
            usv._refresh_health_check()
            usv._record_file3_if_due()

            if usv._consume_flag(FLAG_STOP):
                usv._trigger_estop("YKI", force_rc7=True)

            if not usv.mission_active:
                if usv.state == usv.STATE_HOLD:
                    usv._run_virtual_anchor_step()
                else:
                    usv.v_target = 0.0
                    usv.heading_target = usv.current_heading
                if USV_MODE == USV_MODE_RACE and os.path.exists(FLAG_START):
                    usv._consume_flag(FLAG_START)
                    if (time.monotonic() - usv._last_race_flag_purge_log) >= 2.0:
                        print("[POLICY] [RACE] API start flag temizlendi (RC-only baslatma politikasi)")
                        usv._last_race_flag_purge_log = time.monotonic()

                # Check for API start
                start_from_api = bool(USV_MODE != USV_MODE_RACE and usv._consume_flag(FLAG_START))
                start_from_rc = bool(USV_MODE == USV_MODE_RACE and usv.rc_channels.get("ch5", 0) >= RC_RACE_START_PWM)

                if start_from_api or start_from_rc:
                    src = "RC" if start_from_rc else "API"
                    print(f"[START] [START] Kaynak={src}")
                    start_result = usv.start_mission()
                    if start_result:
                        usv.run()
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
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[STOP] [SISTEM] Kapatiliyor...")
    finally:
        usv.stop_motors()
        usv._close_file3_recorder()

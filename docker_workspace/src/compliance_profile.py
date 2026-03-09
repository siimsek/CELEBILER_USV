"""
Shared compliance profile for TEKNOFEST 2026 IDA report alignment.
This module centralizes mode rules, thresholds, telemetry frequencies,
communication topology metadata, and readiness checks.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


USV_MODE_TEST = "test"
USV_MODE_RACE = "race"


def resolve_usv_mode(raw_mode: str | None = None) -> str:
    raw = (raw_mode if raw_mode is not None else os.environ.get("USV_MODE", "")).strip().lower()
    return USV_MODE_RACE if raw == USV_MODE_RACE else USV_MODE_TEST


USV_MODE = resolve_usv_mode()


# Mission / control profile
CONTROL_HZ = 10
GENERAL_TELEMETRY_HZ = 5
OBSTACLE_TELEMETRY_HZ = 5
LIDAR_PROCESSING_HZ = 10

R_WP_M = 2.5
T_HOLD_S = 2.0
P1_SPEED_CRUISE_MPS = 1.2
P1_SPEED_APPROACH_MPS = 0.6
P2_WAIT_SPEED_MPS = 0.6
P2_CRUISE_MPS = 1.0
P2_STABLE_S = 1.0
P2_GATE_CONFIRM_S = 0.5
P3_MAX_SPEED_MPS = 1.5
P3_TIMEOUT_S = 180
P3_RETRY_S = 60
P3_RETRY_COUNT = 1
P3_REVERSE_SPEED_MPS = 0.6
P3_REVERSE_DISTANCE_M = 3.0
P3_REVERSE_TIMEOUT_S = 4.0
D_MIN_M = 2.0
HEARTBEAT_WARN_S = 5.0
HEARTBEAT_FAIL_S = 30.0
FAILSAFE_SLOW_MPS = 0.3
CAMERA_FRAME_TIMEOUT_S = 1.0
LIDAR_READY_TIMEOUT_S = 1.0
FUSION_ENABLED = True
FUSION_BEARING_WINDOW_DEG = 10.0
FUSION_LIDAR_MIN_VALID_M = 0.4
FUSION_LIDAR_CONFIRM_MAX_M = 12.0
FUSION_CONFIRM_HOLD_S = 0.3
FUSION_GATE_EVENT_CONFIRM_WINDOW_S = 1.2
FUSION_LOG_PERIOD_S = 2.0
DYN_SPEED_ENABLED = True
DYN_SPEED_SCOPE = ("P1", "P2")
DYN_SPEED_BAND_SOFT_DEG = 8.0
DYN_SPEED_BAND_MEDIUM_DEG = 18.0
DYN_SPEED_BAND_HARD_DEG = 30.0
DYN_SPEED_FACTOR_STRAIGHT = 1.00
DYN_SPEED_FACTOR_SOFT = 0.80
DYN_SPEED_FACTOR_MEDIUM = 0.60
DYN_SPEED_FACTOR_HARD = 0.40
DYN_SPEED_MIN_MPS_P1 = 0.35
DYN_SPEED_MIN_MPS_P2 = 0.30
DYN_SPEED_LOG_PERIOD_S = 2.0
WIND_ASSIST_ENABLED = True
WIND_ASSIST_SCOPE = ("P1", "P2")
WIND_ASSIST_I_GAIN = 0.05
WIND_ASSIST_BIAS_MAX_DEG = 12.0
WIND_ASSIST_ACTIVE_ERR_MAX_DEG = 18.0
WIND_ASSIST_DECAY_PER_S = 0.6
WIND_ASSIST_LOG_PERIOD_S = 2.0
GEOFENCE_ENABLED = True
GEOFENCE_FAILSAFE_ONLY = True
GEOFENCE_RADIUS_M = 12.0
GEOFENCE_DRIFT_TRIGGER_M = 2.0
GEOFENCE_ANCHOR_SPEED_MPS = 0.35
GEOFENCE_ANCHOR_PULSE_S = 0.8
GEOFENCE_ANCHOR_COOLDOWN_S = 1.5
GEOFENCE_LOG_PERIOD_S = 2.0
HORIZON_LOCK_ENABLED = True
HORIZON_LOCK_SCOPE = ("P2", "P3")
HORIZON_LOCK_ROLL_GAIN = 1.0
HORIZON_LOCK_PITCH_GAIN = 0.15
HORIZON_LOCK_MIN_TILT_DEG = 1.0
HORIZON_LOCK_MAX_CORRECTION_DEG = 15.0
HORIZON_LOCK_LOG_PERIOD_S = 2.0
CAM_ADAPT_ENABLED = True
CAM_ADAPT_LUMA_DARK_THRESHOLD = 78.0
CAM_ADAPT_LUMA_BRIGHT_THRESHOLD = 182.0
CAM_ADAPT_EXPOSURE_MAX_GAIN = 1.30
CAM_ADAPT_EXPOSURE_MIN_GAIN = 0.78
CAM_ADAPT_DARK_BETA = 18.0
CAM_ADAPT_BRIGHT_BETA = 12.0
CAM_ADAPT_HSV_DARK_S_RELAX = 32
CAM_ADAPT_HSV_DARK_V_RELAX = 38
CAM_ADAPT_HSV_BRIGHT_S_SHIFT = 34
CAM_ADAPT_HSV_BRIGHT_V_SHIFT = 20
CAM_ADAPT_LOG_PERIOD_S = 2.0


# Safety / IO
RC7_SAFE_PWM = 1100
RC7_ESTOP_PWM = 1900
RC7_ESTOP_FORCE_PWM = 2011
RC_RACE_START_PWM = 1700

CONTROL_DIR = "/root/workspace/control"
LOG_DIR = "/root/workspace/logs"
USB_STORAGE_CANDIDATES = (
    "/root/workspace/usb",
    "/media/usb",
    "/mnt/usb",
    "/media/pi",
)
USB_REQUIRED_IN_RACE = True


LINK_TOPOLOGY = {
    "link_a": {
        "name": "RC + remote power cut",
        "technology": "Crossfire",
        "functions": ["mission_start_trigger", "remote_estop_trigger"],
    },
    "link_b": {
        "name": "Telemetry + mission upload",
        "technology": "433MHz serial modem",
        "protocol": "MAVLink",
        "functions": ["telemetry", "mission_upload"],
    },
    "internal_links": [
        {"path": "Pixhawk<->RaspberryPi", "technology": "MAVLink"},
        {"path": "RaspberryPi<->STM32", "technology": "USB serial"},
    ],
}

REPORT_TELEMETRY_GROUPS = {
    "mode_state": ["mode", "state", "active_parkur"],
    "mission_progress": ["target", "wp_info", "gate_count"],
    "event_flags": ["gate_gecildi", "angajman_tamam", "timeout", "failsafe_state"],
    "navigation_health": ["gps", "ekf", "imu", "camera_ready", "lidar_ready"],
    "link_health": ["heartbeat_age_s", "rc_link_active"],
    "safety": ["estop_state", "estop_source", "command_lock", "health_ready"],
    "energy": ["battery_voltage"],
}

COMMS_POLICY = {
    "wifi_bands_forbidden": ["2.4-2.8GHz", "5.15-5.85GHz"],
    "wifi_must_be_disabled": True,
    "cellular_modem_allowed": False,
    "video_tx_allowed": False,
    "frequency_channel_selection_required": True,
}


def _is_dir_writable(path: Path) -> bool:
    try:
        path.mkdir(parents=True, exist_ok=True)
        probe = path / ".write_probe.tmp"
        probe.write_text(f"{time.time():.6f}", encoding="utf-8")
        probe.unlink(missing_ok=True)
        return True
    except Exception:
        return False


def evaluate_storage_health(mode: str, local_dir: str = LOG_DIR, usb_candidates: Iterable[str] = USB_STORAGE_CANDIDATES) -> Dict[str, object]:
    local_path = Path(local_dir)
    local_ok = _is_dir_writable(local_path)

    candidate_paths = [Path(p) for p in usb_candidates]
    existing_candidates = [p for p in candidate_paths if p.exists()]
    usb_required = bool(mode == USV_MODE_RACE and USB_REQUIRED_IN_RACE)

    if existing_candidates:
        usb_ok = any(_is_dir_writable(p) for p in existing_candidates)
    else:
        usb_ok = not usb_required

    return {
        "local_writable": local_ok,
        "usb_required": usb_required,
        "usb_present": bool(existing_candidates),
        "usb_writable": usb_ok,
        "usb_candidates": [str(p) for p in candidate_paths],
    }


def evaluate_readiness_flags(
    *,
    mode: str,
    mavlink_vehicle_link: bool,
    telemetry_heartbeat_ok: bool,
    rc_link_active: bool,
    estop_safe: bool,
    camera_fresh: bool,
    lidar_fresh: bool,
    storage_health: Dict[str, object],
) -> Tuple[Dict[str, bool], List[str]]:
    flags = {
        "mavlink_vehicle_link": bool(mavlink_vehicle_link),
        "telemetry_heartbeat_ok": bool(telemetry_heartbeat_ok),
        "rc_link_active": bool(rc_link_active),
        "estop_safe": bool(estop_safe),
        "camera_fresh": bool(camera_fresh),
        "lidar_fresh": bool(lidar_fresh),
        "storage_local_writable": bool(storage_health.get("local_writable", False)),
        "storage_usb_writable": bool(storage_health.get("usb_writable", False)),
    }

    required_keys = [
        "mavlink_vehicle_link",
        "telemetry_heartbeat_ok",
        "rc_link_active",
        "estop_safe",
        "camera_fresh",
        "lidar_fresh",
        "storage_local_writable",
    ]
    if storage_health.get("usb_required", False):
        required_keys.append("storage_usb_writable")

    missing = [key for key in required_keys if not flags.get(key, False)]
    return flags, missing

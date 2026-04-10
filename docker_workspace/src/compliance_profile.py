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


def _resolve_runtime_root() -> Path:
    env_root = os.environ.get("USV_PROJECT_ROOT", "").strip()
    if env_root:
        return Path(env_root).expanduser().resolve()

    here = Path(__file__).resolve()
    for candidate in (here.parent, *here.parents):
        if (candidate / "src").is_dir() and (candidate / "scripts").is_dir():
            return candidate
        if (candidate / "docker_workspace" / "src").is_dir() and (candidate / "host_scripts").is_dir():
            return candidate
    return here.parent


def _is_container_layout(root: Path) -> bool:
    return (root / "src").is_dir() and (root / "scripts").is_dir()


RUNTIME_ROOT = _resolve_runtime_root()


# Mission / control profile
CONTROL_HZ = 10
GENERAL_TELEMETRY_HZ = 5
OBSTACLE_TELEMETRY_HZ = 5
LIDAR_PROCESSING_HZ = 10
if os.environ.get("USV_SIM") == "1":
    # Sim: yüksek Hz — pose/telemetri akıcı, ışınlanma hissi azalır
    CONTROL_HZ = 40
    GENERAL_TELEMETRY_HZ = 40
    OBSTACLE_TELEMETRY_HZ = 40
    LIDAR_PROCESSING_HZ = 40

R_WP_M = 2.5
T_HOLD_S = 2.0
P1_SPEED_CRUISE_MPS = 1.2
P1_SPEED_APPROACH_MPS = 0.6
P2_WAIT_SPEED_MPS = 0.6
P2_CRUISE_MPS = 1.5
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
# P2: lidar + kamera birlesik kacinma (warn mesafesinde yumuak, D_MIN altinda sert)
P2_LIDAR_WARN_M = 5.0
P2_ESCAPE_MAX_DEG = 40.0
P2_ESCAPE_MAX_DEG_LOCAL_MINIMA = 65.0
P2_LOCAL_MINIMA_TIMEOUT_S = 8.0
P2_CAM_YELLOW_WEIGHT = 0.55
HEARTBEAT_WARN_S = 5.0
HEARTBEAT_FAIL_S = 30.0
FAILSAFE_SLOW_MPS = 0.3
CAMERA_FRAME_TIMEOUT_S = 1.0
LIDAR_READY_TIMEOUT_S = 1.0
SIM_ALLOW_RC_OVERRIDE = os.environ.get("SIM_ALLOW_RC_OVERRIDE", "").strip() == "1"

# Innovation switches: set to False to hard-disable from code before startup.
INNOVATION_SWITCHES = {
    "sensor_fusion": True,
    "dynamic_speed_profile": True,
    "wind_assist": True,
    "virtual_anchor_geofence": True,
    "horizon_lock": True,
    "camera_adaptation": True,
    "autonomy_health_trust_bar": True,
}

FUSION_ENABLED = bool(INNOVATION_SWITCHES.get("sensor_fusion", True))
FUSION_BEARING_WINDOW_DEG = 10.0
FUSION_LIDAR_MIN_VALID_M = 0.4
FUSION_LIDAR_CONFIRM_MAX_M = 12.0
FUSION_CONFIRM_HOLD_S = 0.3
FUSION_CAMERA_ONLY_TIMEOUT_S = 3.0
FUSION_GATE_EVENT_CONFIRM_WINDOW_S = 1.2
FUSION_LOG_PERIOD_S = 2.0
DYN_SPEED_ENABLED = bool(INNOVATION_SWITCHES.get("dynamic_speed_profile", True))
DYN_SPEED_SCOPE = ("P1", "P2")
DYN_SPEED_BAND_SOFT_DEG = 8.0
DYN_SPEED_BAND_MEDIUM_DEG = 18.0
DYN_SPEED_BAND_HARD_DEG = 30.0
DYN_SPEED_FACTOR_STRAIGHT = 1.00
DYN_SPEED_FACTOR_SOFT = 0.90
DYN_SPEED_FACTOR_MEDIUM = 0.78
DYN_SPEED_FACTOR_HARD = 0.64
DYN_SPEED_MIN_MPS_P1 = 0.35
DYN_SPEED_MIN_MPS_P2 = 0.40
DYN_SPEED_LOG_PERIOD_S = 2.0
WIND_ASSIST_ENABLED = bool(INNOVATION_SWITCHES.get("wind_assist", True))
WIND_ASSIST_SCOPE = ("P1", "P2")
WIND_ASSIST_I_GAIN = 0.05
WIND_ASSIST_BIAS_MAX_DEG = 12.0
WIND_ASSIST_ACTIVE_ERR_MAX_DEG = 18.0
WIND_ASSIST_DECAY_PER_S = 0.6
WIND_ASSIST_LOG_PERIOD_S = 2.0
GEOFENCE_ENABLED = bool(INNOVATION_SWITCHES.get("virtual_anchor_geofence", True))
GEOFENCE_FAILSAFE_ONLY = True
GEOFENCE_RADIUS_M = 12.0
GEOFENCE_DRIFT_TRIGGER_M = 2.0
GEOFENCE_ANCHOR_SPEED_MPS = 0.35
GEOFENCE_ANCHOR_PULSE_S = 0.8
GEOFENCE_ANCHOR_COOLDOWN_S = 1.5
GEOFENCE_LOG_PERIOD_S = 2.0
HORIZON_LOCK_ENABLED = bool(INNOVATION_SWITCHES.get("horizon_lock", True))
HORIZON_LOCK_SCOPE = ("P2", "P3")
HORIZON_LOCK_ROLL_GAIN = 1.0
HORIZON_LOCK_PITCH_GAIN = 0.15
HORIZON_LOCK_MIN_TILT_DEG = 1.0
HORIZON_LOCK_MAX_CORRECTION_DEG = 15.0
HORIZON_LOCK_LOG_PERIOD_S = 2.0
CAM_ADAPT_ENABLED = bool(INNOVATION_SWITCHES.get("camera_adaptation", True))
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
TRUST_BAR_ENABLED = bool(INNOVATION_SWITCHES.get("autonomy_health_trust_bar", True))
TRUST_WEIGHT_GPS = 0.30
TRUST_WEIGHT_CAMERA = 0.20
TRUST_WEIGHT_LIDAR = 0.25
TRUST_WEIGHT_RC = 0.25
TRUST_GOOD_THRESHOLD = 85.0
TRUST_WARN_THRESHOLD = 60.0
TRUST_LIDAR_POINTS_FULL = 360
if os.environ.get("USV_SIM") == "1":
    TRUST_LIDAR_POINTS_FULL = 720
TRUST_LOG_PERIOD_S = 2.0


# Safety / IO
RC7_SAFE_PWM = 1100
RC7_ESTOP_PWM = 1900
RC7_ESTOP_FORCE_PWM = 2011
RC_RACE_START_PWM = 1700

def _default_control_dir() -> str:
    if os.environ.get("USV_SIM") == "1":
        sim_control = RUNTIME_ROOT / "sim" / "control"
        if (RUNTIME_ROOT / "sim").is_dir():
            return str(sim_control)
    if _is_container_layout(RUNTIME_ROOT):
        return str(RUNTIME_ROOT / "control")
    if (RUNTIME_ROOT / "docker_workspace").is_dir():
        return str(RUNTIME_ROOT / "docker_workspace" / "control")
    return str(RUNTIME_ROOT / "control")


def _default_log_dir() -> str:
    if os.environ.get("USV_SIM") == "1":
        sim_log_dir = RUNTIME_ROOT / "logs" / "system"
        if (RUNTIME_ROOT / "sim").is_dir():
            return str(sim_log_dir)
    if _is_container_layout(RUNTIME_ROOT):
        return str(RUNTIME_ROOT / "logs")
    if (RUNTIME_ROOT / "docker_workspace").is_dir():
        return str(RUNTIME_ROOT / "logs" / "system")
    return str(RUNTIME_ROOT / "logs")


def _default_mission_file() -> str:
    if os.environ.get("USV_SIM") == "1":
        sim_mission = RUNTIME_ROOT / "sim" / "configs" / "mission_parkour_all.json"
        if (RUNTIME_ROOT / "sim" / "configs").is_dir():
            return str(sim_mission)
    if _is_container_layout(RUNTIME_ROOT):
        return str(RUNTIME_ROOT / "mission.json")
    if (RUNTIME_ROOT / "docker_workspace").is_dir():
        return str(RUNTIME_ROOT / "docker_workspace" / "mission.json")
    return str(RUNTIME_ROOT / "mission.json")


CONTROL_DIR = os.environ.get("CONTROL_DIR", _default_control_dir())
LOG_DIR = os.environ.get("LOG_DIR", _default_log_dir())
MISSION_FILE_DEFAULT = os.environ.get("MISSION_FILE", _default_mission_file())
USB_STORAGE_CANDIDATES = (
    "/home/siimsek/Desktop/CELEBILER_USV/logs/usb",
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
    existing_candidates = []
    for p in candidate_paths:
        try:
            if p.exists():
                existing_candidates.append(p)
        except (PermissionError, OSError):
            pass
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

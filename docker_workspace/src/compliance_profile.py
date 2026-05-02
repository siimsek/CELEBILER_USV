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


def env_flag(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() in ("1", "true", "yes", "on")


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

R_WP_M = max(0.5, min(3.0, float(os.environ.get("USV_SIM_R_WP_M", os.environ.get("R_WP_M", "2.5")))))
T_HOLD_S = 2.0
MISSION_INPUT_FORMAT = "flat_ordered"
MISSION_ALLOW_STRUCTURED_LEGACY = os.environ.get("MISSION_ALLOW_STRUCTURED_LEGACY", "").strip() == "1"
# Unified mission: split is done by split_nav_engage() — no fixed P2/P3 count needed.
P1_SPEED_CRUISE_MPS = float(os.environ.get("USV_SIM_P1_CRUISE_MPS", "1.5"))
P1_SPEED_APPROACH_MPS = float(os.environ.get("USV_SIM_P1_APPROACH_MPS", "0.8"))
P1_AUTO_WAYPOINT_TIMEOUT_S = 180.0  # per-waypoint max wait in P1 AUTO monitor
P2_WAIT_SPEED_MPS = float(os.environ.get("USV_SIM_P2_WAIT_MPS", "1.0"))
P2_CRUISE_MPS = float(os.environ.get("USV_SIM_P2_CRUISE_MPS", "2.0"))
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
P2_LIDAR_WARN_M = 2.5
P2_LIDAR_WARN_EXIT_MARGIN_M = 0.40
# NAV: donmus bacak geometrisi yeterliyse lateral hata ile bearing düzeltmesi (saf WP bearing spiralını azaltır)
NAV_CROSS_TRACK_K = 0.45
NAV_CROSS_TRACK_L1_MIN_M = 2.8
NAV_CROSS_TRACK_L1_MAX_M = 15.0
NAV_CROSS_TRACK_CORR_CAP_DEG = 22.0
NAV_CROSS_TRACK_MIN_LEG_M = 3.75  # max(R_WP_M * 1.5, 3.0) ile _waypoint_leg_progress ile uyumlu
# NAV iki faz (her WP bacaginda): (1) yalnız yönelt — |err| <= HEADING_DONE olmadan gitme yok; (2) hat + ileri hız
NAV_ALIGN_HEADING_DONE_DEG = float(os.environ.get("USV_SIM_ALIGN_HEADING_DEG", "8.0"))
# Geriye uyumluluk (eski ad); geçişler NAV_ALIGN_HEADING_DONE_DEG kullanır
NAV_ALIGN_ENTER_ADVANCE_DEG = 9.5
NAV_ALIGN_PIVOT_UNTIL_DEG = 11.0
# İleri fazda burun şaşarsa tekrar (1) dönüşe dön — 24° çok gevşekti (log: 15°+ ile hâlâ advance)
NAV_ALIGN_REVERT_ALIGN_DEG = 12.0
NAV_ALIGN_MAX_SPEED_MPS = float(os.environ.get("USV_SIM_ALIGN_MAX_MPS", "0.8"))
NAV_ALIGN_CREEP_SPEED_MPS = 0.28
NAV_ALIGN_TIMEOUT_S = 14.0
NAV_WP_APPROACH_SPEED_CAP_DIST_M = 1.5  # within this distance, cap speed near waypoint
NAV_WP_APPROACH_SPEED_CAP_MPS = float(os.environ.get("USV_SIM_WP_APPROACH_CAP_MPS", "0.6"))  # was hardcoded 0.35
# Heading damping (inner-loop style): error + yaw-rate feedback + command slew.
NAV_HEADING_DAMPING_ENABLED = True
NAV_HEADING_DAMPING_YAWRATE_GAIN = 0.32
NAV_HEADING_DAMPING_LPF_ALPHA = 0.42
NAV_HEADING_CMD_SLEW_DEG_PER_S = 140.0
# Turn-priority speed caps.
NAV_TURN_SPEED_CAP_SOFT_ERR_DEG = 25.0
NAV_TURN_SPEED_CAP_MEDIUM_ERR_DEG = 40.0
NAV_TURN_SPEED_CAP_HARD_ERR_DEG = 65.0
NAV_TURN_SPEED_CAP_SOFT_MPS = 0.95
NAV_TURN_SPEED_CAP_MEDIUM_MPS = 0.70
NAV_TURN_SPEED_CAP_HARD_MPS = 0.45
NAV_NEAR_WP_TURN_SPEED_CAP_DIST_M = 1.4
NAV_NEAR_WP_TURN_ERR_DEG = 20.0
NAV_NEAR_WP_TURN_SPEED_CAP_MPS = 0.40
# Avoidance-bias slew: suppress one-scan spikes without weakening emergency turns.
NAV_AVOID_BIAS_SLEW_DEG_PER_S = 180.0
# Hiza fazinda dusuk hiz; onde dubalar yakinlasinca tam NAV+kaçınma (ileri faz) zorunlu
NAV_ALIGN_SUSPEND_NEAR_M = 3.9
# WP4→5 gibi uzun bacakta duba 2–3 m iken hâlâ align’da kalınmasın; bu altında mesafe şartsız advance
NAV_ALIGN_TIGHT_OBSTACLE_M = 3.2
# 3.2–3.9 m “yumuşak” bant: yalnız WP yakınında advance (açık su gürültüsünü uzakta kes)
NAV_ALIGN_LIDAR_SUSPEND_MAX_DIST_M = 14.0
P2_ESCAPE_MAX_DEG = 48.0
P2_ESCAPE_MAX_DEG_LOCAL_MINIMA = 65.0
P2_LOCAL_MINIMA_TIMEOUT_S = 8.0
P2_CAM_YELLOW_WEIGHT = 0.55
# Kamera (sarı duba) + lidar aynı tehdidi görünce sarı sapma ağırlığı
P2_CAM_YELLOW_FUSION_MULT = 1.5
# Turuncu kenar (TURUNCU_SINIR) — temas yasak; sarıdan biraz düşük ağırlık (kapı hizası ile çakışmayı azalt)
P2_ORANGE_BOUNDARY_WEIGHT = 0.48
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
    # Disabled by race contract: P1 remains pure Pixhawk AUTO, while test/P2/P3
    # use the shared Pi-guided waypoint engine.
    "unified_execution_path": False,
}

FUSION_ENABLED = bool(INNOVATION_SWITCHES.get("sensor_fusion", True))
# Legacy switch retained for compatibility; run_nav() enforces the race/test split.
UNIFIED_EXECUTION_PATH = bool(INNOVATION_SWITCHES.get("unified_execution_path", True))
FUSION_BEARING_WINDOW_DEG = 10.0
FUSION_LIDAR_MIN_VALID_M = 0.4
FUSION_LIDAR_CONFIRM_MAX_M = 12.0
FUSION_CONFIRM_HOLD_S = 0.3
FUSION_CAMERA_ONLY_TIMEOUT_S = 3.0
FUSION_GATE_EVENT_CONFIRM_WINDOW_S = 1.2
FUSION_LOG_PERIOD_S = 2.0
DYN_SPEED_ENABLED = bool(INNOVATION_SWITCHES.get("dynamic_speed_profile", True))
DYN_SPEED_SCOPE = ("NAV",)  # Unified: active during all waypoint navigation
DYN_SPEED_BAND_SOFT_DEG = 8.0
DYN_SPEED_BAND_MEDIUM_DEG = 18.0
DYN_SPEED_BAND_HARD_DEG = 30.0
DYN_SPEED_FACTOR_STRAIGHT = 1.00
DYN_SPEED_FACTOR_SOFT = 0.90
DYN_SPEED_FACTOR_MEDIUM = 0.78
DYN_SPEED_FACTOR_HARD = 0.64
DYN_SPEED_MIN_MPS_NAV = 0.45  # Unified: min speed during turns in NAV phase
# Backward-compat aliases (deprecated, use DYN_SPEED_MIN_MPS_NAV)
DYN_SPEED_MIN_MPS_P1 = DYN_SPEED_MIN_MPS_NAV
DYN_SPEED_MIN_MPS_P2 = DYN_SPEED_MIN_MPS_NAV
DYN_SPEED_LOG_PERIOD_S = 2.0
WIND_ASSIST_ENABLED = bool(INNOVATION_SWITCHES.get("wind_assist", True))
WIND_ASSIST_SCOPE = ("NAV",)  # Unified: active during all waypoint navigation
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
HORIZON_LOCK_SCOPE = ("NAV", "ENGAGE")  # Unified: active in both nav and engage phases
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
TRUST_LOG_PERIOD_S = 5.0


# Safety / IO
RC7_SAFE_PWM = 1100
RC7_ESTOP_PWM = 1900
RC7_ESTOP_FORCE_PWM = 2011
RC_RACE_START_PWM = 1700

# Canonical mode-state / mixer profile (sim-first: same logic in sim + real).
MODE_NEUTRAL_DWELL_S = float(os.environ.get("MODE_NEUTRAL_DWELL_S", "1.0"))

HEADING_DEADZONE_DEG = float(os.environ.get("HEADING_DEADZONE_DEG", "2.0"))
HEADING_HYST_ENTER_DEG = float(os.environ.get("HEADING_HYST_ENTER_DEG", "3.0"))
HEADING_HYST_EXIT_DEG = float(os.environ.get("HEADING_HYST_EXIT_DEG", "1.5"))

HEADING_WRONG_TURN_ERR_DEG = float(os.environ.get("HEADING_WRONG_TURN_ERR_DEG", "7.0"))
HEADING_WRONG_TURN_YAWRATE_DPS = float(os.environ.get("HEADING_WRONG_TURN_YAWRATE_DPS", "0.25"))
HEADING_WRONG_TURN_TRIGGER_S = float(os.environ.get("HEADING_WRONG_TURN_TRIGGER_S", "0.35"))
HEADING_WRONG_TURN_GAIN = float(os.environ.get("HEADING_WRONG_TURN_GAIN", "1.25"))
HEADING_WRONG_TURN_SPEED_CAP_MPS = float(os.environ.get("HEADING_WRONG_TURN_SPEED_CAP_MPS", "0.32"))

PWM_NEUTRAL_US = int(float(os.environ.get("PWM_NEUTRAL_US", "1500")))
PWM_MIN_US = int(float(os.environ.get("PWM_MIN_US", "1100")))
PWM_MAX_US = int(float(os.environ.get("PWM_MAX_US", "1900")))
PWM_DEADBAND_US = int(float(os.environ.get("PWM_DEADBAND_US", "20")))
PWM_MIN_EFFECTIVE_US = int(float(os.environ.get("PWM_MIN_EFFECTIVE_US", "60")))
PWM_SLEW_RATE_US_PER_S = float(os.environ.get("PWM_SLEW_RATE_US_PER_S", "120"))
PWM_JERK_LIMIT_US_PER_S2 = float(os.environ.get("PWM_JERK_LIMIT_US_PER_S2", "360"))
PWM_TRIM_MAX_US = int(float(os.environ.get("PWM_TRIM_MAX_US", "40")))
PWM_TRIM_LEARN_GAIN = float(os.environ.get("PWM_TRIM_LEARN_GAIN", "0.12"))
PWM_TRIM_ERR_BAND_DEG = float(os.environ.get("PWM_TRIM_ERR_BAND_DEG", "4.0"))
PWM_TRIM_YAW_RATE_BAND_DPS = float(os.environ.get("PWM_TRIM_YAW_RATE_BAND_DPS", "0.8"))

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
        "functions": ["mission_planner_gcs", "telemetry", "mission_upload"],
        "sim_endpoint": "udp:14552",
    },
    "internal_links": [
        {"path": "Pixhawk<->RaspberryPi", "technology": "MAVLink", "endpoints": ["udp:14551", "tcp:5760(sim)"]},
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

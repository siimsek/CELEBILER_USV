import time
from collections import deque
import csv as csv_mod
from pymavlink import mavutil
import sys
import threading
import os
import glob
import json
import random
import serial
import math
import urllib.request
from flask import Flask, Response, jsonify, render_template_string, request, send_file

from console_utils import make_console_printer
from compliance_profile import (
    COMMS_POLICY,
    CONTROL_DIR as DEFAULT_CONTROL_DIR,
    GENERAL_TELEMETRY_HZ,
    HEARTBEAT_WARN_S,
    LIDAR_PROCESSING_HZ,
    LINK_TOPOLOGY,
    LOG_DIR as DEFAULT_LOG_DIR,
    MISSION_ALLOW_STRUCTURED_LEGACY,
    MISSION_FILE_DEFAULT,
    MISSION_INPUT_FORMAT,
    OBSTACLE_TELEMETRY_HZ,
    REPORT_TELEMETRY_GROUPS,
    USV_MODE,
    USV_MODE_RACE,
)
from json_atomic import atomic_read_json, atomic_write_json
from mission_adapter import (
    adapt_mission_to_structured,
    validate_target_color,
)
from mission_config import (
    TARGET_STATE_FILE,
    get_mission_split_profile,
    load_target_state,
    split_mission_waypoints,
    split_nav_engage,
    validate_coordinate_mission,
)
from sim_nav_state import load_sim_nav_state, mavlink_heading_cdeg_valid, parse_sim_home, heading_rad_to_nav_deg
from spatial_frame import (
    compute_spatial_bounds,
    latlon_to_spatial_enu_m,
    lidar_local_to_world_enu_compass,
    load_course_static_features,
    resolve_boat_spatial_enu,
    resolve_spatial_origin_candidates,
)

print = make_console_printer("TELEM")

# --- YAPILANDIRMA VE SABİTLER ---
BAUD_RATE_PIXHAWK = 115200
BAUD_RATES_STM32 = [9600, 115200] # Otomatik denenir
WEB_PORT = 8080
LOG_DIR = os.environ.get("LOG_DIR", DEFAULT_LOG_DIR)
CONTROL_DIR = os.environ.get("CONTROL_DIR", DEFAULT_CONTROL_DIR)
os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(CONTROL_DIR, exist_ok=True)

from runtime_debug_log import (
    install_module_function_tracing,
    log_jsonl,
    redirect_std_streams,
    setup_component_logger,
)

_telem_dbg = setup_component_logger("telemetry")
redirect_std_streams(_telem_dbg)
_telem_dbg.info(
    "telemetry module load LOG_DIR=%s USV_MODE=%s CONTROL_DIR=%s",
    LOG_DIR,
    USV_MODE,
    CONTROL_DIR,
)

CSV_FILE = f"{LOG_DIR}/telemetri_verisi.csv"
RC7_SAFE_PWM = 1100
RC7_ESTOP_FORCE_PWM = 2011
PWM_VALID_MIN = 900
PWM_VALID_MAX = 2100
RC_SIGNAL_TIMEOUT_S = 1.5
LINK_STATE_FILE = f"{CONTROL_DIR}/telemetry_link_state.json"

# IPC: usv_main.py ile dosya üzerinden iletişim (farklı süreçler)
FLAG_START = f"{CONTROL_DIR}/start_mission.flag"
FLAG_STOP = f"{CONTROL_DIR}/emergency_stop.flag"
STATE_FILE = f"{CONTROL_DIR}/mission_state.json"
MOTOR_COMMAND_FILE = f"{CONTROL_DIR}/motor_command.json"
CAMERA_LATEST_JPG = f"{LOG_DIR}/camera_latest.jpg"
LIDAR_STREAM_PROXY_SOURCE = os.environ.get("LIDAR_STREAM_PROXY_SOURCE", "http://127.0.0.1:5001/")
LIDAR_MAP_JPG = f"{LOG_DIR}/file3_local_map_latest.jpg"
GPS_MIN_FIX_TYPE = 3

# --- GLOBAL DURUM ---
# Şartname Bölüm 6: lat, lon, hız, roll, pitch, heading, hız_setpoint, yön_setpoint
telemetry_data = {
    "Timestamp": "--", "Lat": None, "Lon": None, "Heading": 0,
    "Battery": 0, "Mode": "DISCONNECTED", "Speed": 0, "Roll": 0, "Pitch": 0,
    "Speed_Setpoint": 0, "Heading_Setpoint": 0,  # Şartname 6 zorunlu alanları
    "STM_Date": "--:--:--", "Env_Temp": 0, "Env_Hum": 0, "Rain_Val": 0, "Rain_Status": "DRY",
    "Sys_CPU": 0, "Sys_RAM": 0, "Sys_Temp": 0,
    "RC1": 0, "RC2": 0, "RC3": 0, "RC4": 0, "RC7": 0,
    "Out1": 0, "Out3": 0,
    "CMD_Port": 1500, "CMD_Stbd": 1500, "CMD_Source": "--",
    "GPS_FixType": 0, "GPS_Satellites": 0,
}
CSV_COLUMNS = [
    "timestamp",
    "lat",
    "lon",
    "ground_speed",
    "roll",
    "pitch",
    "heading",
    "speed_setpoint",
    "heading_setpoint",
    "left_pwm",
    "right_pwm",
    "mode",
    "waypoint_index",
    "obstacle_state",
]
COLUMNS = CSV_COLUMNS
CSV_LOG_INTERVAL = 1.0  # Şartname: en az 1 Hz
SIMULATION_MODE = False

# --- GÖREV DURUMU (Dashboard <-> usv_main.py iletişimi) ---
# usv_main.py bu dict'i import edip günceller, dashboard okur
mission_data = {
    "state": 0,         # 0=Bekleme, 1=Parkur1, 2=Parkur2, 3=Parkur3, 4=Tamamlandı
    "active": False,    # Görev çalışıyor mu?
    "start_time": 0,    # time.time() başlangıç
    "target": "--",     # Hedef açıklama
    "wp_info": "-- / --",  # Waypoint bilgisi
    "start_requested": False,  # Görev başlat isteği
    "start_requested_at": 0.0,
    "stop_requested": False,   # Acil durdur isteği
}
last_state_cache = dict(mission_data)
last_motor_command_cache = {
    "cmd_port_pwm": 1500,
    "cmd_stbd_pwm": 1500,
    "source": "--",
    "mode_state": {},
}
EVENT_QUEUE = deque(maxlen=256)
EVENT_ID = 0
EVENT_LAST = {
    "gate_count": 0,
    "failsafe_state": "--",
    "estop_state": False,
    "timeout_count": 0,
    "start_phase": "idle",
}
EVENT_LOCK = threading.Lock()
SPATIAL_TRAIL = deque(maxlen=2000)
SPATIAL_LAST_TRAIL_TS = 0.0
SPATIAL_TRAIL_MIN_STEP_M = 0.20
SPATIAL_ORIGIN = {"lat": None, "lon": None}
SPATIAL_LAST_START_TIME = None
_sim_ld = os.environ.get("SIM_LOG_DIR", "").strip()
_sim_base = _sim_ld if _sim_ld else LOG_DIR
LOG_FILE_ALLOWLIST = {
    "telemetry.log": f"{LOG_DIR}/telemetry.debug.log",
    "telemetry.debug.log": f"{LOG_DIR}/telemetry.debug.log",
    "telemetry.jsonl": f"{LOG_DIR}/telemetry.jsonl",
    "sitl.log": f"{_sim_base}/sitl.log",
    "gazebo.log": f"{_sim_base}/gazebo.log",
    "cam.log": f"{LOG_DIR}/cam.debug.log",
    "cam.debug.log": f"{LOG_DIR}/cam.debug.log",
    "cam.jsonl": f"{LOG_DIR}/cam.jsonl",
    "cam_bridge.log": f"{_sim_base}/ros_to_tcp_cam.debug.log",
    "pose.log": f"{_sim_base}/sitl_gazebo_bridge.debug.log",
    "ros_gz.log": f"{_sim_base}/ros_gz.log",
    "usv_main.log": f"{LOG_DIR}/usv_main.debug.log",
    "usv_main.debug.log": f"{LOG_DIR}/usv_main.debug.log",
    "usv_main.jsonl": f"{LOG_DIR}/usv_main.jsonl",
    "lidar_map.log": f"{LOG_DIR}/lidar_map.debug.log",
    "lidar_map.debug.log": f"{LOG_DIR}/lidar_map.debug.log",
    "lidar_map.jsonl": f"{LOG_DIR}/lidar_map.jsonl",
    "check_stack.log": f"{_sim_base}/check_stack.log",
    "compliance_race_test.log": f"{_sim_base}/compliance_race_test.log",
    "telemetri_verisi.csv": CSV_FILE,
    "file3_local_map_index.csv": f"{LOG_DIR}/file3_local_map_index.csv",
}
if _sim_ld:
    _sim_join = lambda n: os.path.join(_sim_ld, n)
    LOG_FILE_ALLOWLIST["sitl_gazebo_bridge.debug.log"] = _sim_join("sitl_gazebo_bridge.debug.log")
    LOG_FILE_ALLOWLIST["sitl_gazebo_bridge.jsonl"] = _sim_join("sitl_gazebo_bridge.jsonl")
    LOG_FILE_ALLOWLIST["ros_to_tcp_cam.debug.log"] = _sim_join("ros_to_tcp_cam.debug.log")
    LOG_FILE_ALLOWLIST["ros_to_tcp_cam.jsonl"] = _sim_join("ros_to_tcp_cam.jsonl")
    LOG_FILE_ALLOWLIST["terminal.log"] = os.path.join(
        os.path.dirname(_sim_ld.rstrip(os.sep)), "terminal.log"
    )


def _ros2_logs_dir():
    ros_ld = os.environ.get("ROS_LOG_DIR", "").strip()
    if ros_ld:
        return ros_ld
    return os.path.join(_sim_base, "ros2")


def _scan_dir_logs(directory, merged):
    """directory altindaki .log/.jsonl dosyalarini merged'e ekler (setdefault)."""
    try:
        if os.path.isdir(directory):
            for fn in os.listdir(directory):
                if not fn.endswith((".log", ".jsonl")):
                    continue
                path = os.path.join(directory, fn)
                if os.path.isfile(path):
                    merged.setdefault(fn, path)
    except OSError:
        pass


def _merged_log_index():
    """Statik liste + LOG_DIR + SIM_LOG_DIR + ROS2 altinda olusan dosyalar."""
    merged = dict(LOG_FILE_ALLOWLIST)
    _scan_dir_logs(LOG_DIR, merged)
    if _sim_ld and _sim_ld != LOG_DIR:
        _scan_dir_logs(_sim_ld, merged)
    rd = _ros2_logs_dir()
    if os.path.isdir(rd):
        _scan_dir_logs(rd, merged)
    return merged


def _read_allowed_log(name):
    safe_name = os.path.basename(str(name or "")).strip()
    if not safe_name:
        return "", None
    merged = _merged_log_index()
    if safe_name in merged:
        return safe_name, merged[safe_name]
    return safe_name, None


def _is_valid_pwm(value):
    try:
        iv = int(value)
    except (TypeError, ValueError):
        return False
    return PWM_VALID_MIN <= iv <= PWM_VALID_MAX


def _pick_primary_fallback(primary, fallback, default=1500):
    if _is_valid_pwm(primary):
        return int(primary)
    if _is_valid_pwm(fallback):
        return int(fallback)
    return int(default)


def _touch_flag(path):
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8"):
            pass
        return True
    except Exception as exc:
        print(f"[WARN] [IPC] Flag yazilamadi ({path}): {exc}")
        return False


def _clear_flag(path):
    try:
        if os.path.exists(path):
            os.remove(path)
        return True
    except Exception as exc:
        print(f"[WARN] [IPC] Flag silinemedi ({path}): {exc}")
        return False

def _start_request_pending(state=None):
    state_dict = state if isinstance(state, dict) else _read_mission_state()
    flag_pending = bool(os.path.exists(FLAG_START))
    if flag_pending:
        return True
    if bool(mission_data.get("start_requested", False)):
        try:
            request_age_s = time.monotonic() - float(mission_data.get("start_requested_at", 0.0) or 0.0)
        except (TypeError, ValueError):
            request_age_s = 999.0
        if request_age_s <= 8.0 and not bool(state_dict.get("active", False)):
            return True
        mission_data["start_requested"] = False
        mission_data["start_requested_at"] = 0.0
    if bool(state_dict.get("active", False)):
        return False
    return False



def _emit_event(kind, payload=None):
    global EVENT_ID
    with EVENT_LOCK:
        EVENT_ID += 1
        event = {
            "id": EVENT_ID,
            "ts_unix": round(time.time(), 3),
            "kind": kind,
            "payload": payload or {},
        }
        EVENT_QUEUE.append(event)
        return event


def _sync_events_from_state(state):
    if not isinstance(state, dict):
        return

    to_emit = []
    with EVENT_LOCK:
        try:
            gate_count = max(0, int(state.get("gate_count", 0) or 0))
        except (TypeError, ValueError):
            gate_count = 0
        prev_gate = int(EVENT_LAST.get("gate_count", 0) or 0)
        if gate_count > prev_gate:
            for idx in range(prev_gate + 1, gate_count + 1):
                to_emit.append(("gate_gecildi", {"gate_count": idx}))
        EVENT_LAST["gate_count"] = gate_count

        failsafe_state = str(state.get("failsafe_state", "--"))
        prev_failsafe = str(EVENT_LAST.get("failsafe_state", "--"))
        if failsafe_state != prev_failsafe:
            to_emit.append(
                (
                    "failsafe",
                    {
                        "state": failsafe_state,
                        "previous": prev_failsafe,
                    },
                )
            )
        EVENT_LAST["failsafe_state"] = failsafe_state

        estop_state = bool(state.get("estop_state", False))
        prev_estop = bool(EVENT_LAST.get("estop_state", False))
        if estop_state != prev_estop:
            to_emit.append(
                (
                    "estop",
                    {
                        "state": estop_state,
                        "source": state.get("estop_source", "--"),
                    },
                )
            )
        EVENT_LAST["estop_state"] = estop_state

        try:
            timeout_count = max(0, int(state.get("timeout_count", 0) or 0))
        except (TypeError, ValueError):
            timeout_count = 0
        prev_timeout = int(EVENT_LAST.get("timeout_count", 0) or 0)
        if timeout_count > prev_timeout:
            for idx in range(prev_timeout + 1, timeout_count + 1):
                to_emit.append(("timeout", {"timeout_count": idx}))
        EVENT_LAST["timeout_count"] = timeout_count

        start_phase = str(state.get("start_phase", "idle") or "idle")
        prev_start_phase = str(EVENT_LAST.get("start_phase", "idle") or "idle")
        if start_phase == "rejected" and prev_start_phase != "rejected":
            mission_end_reason = str(state.get("mission_end_reason", "none") or "none")
            to_emit.append(
                (
                    "start_rejected",
                    {
                        "start_phase": start_phase,
                        "mission_end_reason": mission_end_reason,
                        "fc_start_block_reason": str(state.get("fc_start_block_reason", "unknown") or "unknown"),
                        "guided_ready_missing": list(state.get("guided_ready_missing", []) or []),
                    },
                )
            )
        EVENT_LAST["start_phase"] = start_phase

    for kind, payload in to_emit:
        _emit_event(kind, payload)

# Flask Uygulaması
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR) # Gereksiz logları kapat
app = Flask(__name__)
st = None


@app.before_request
def _telemetry_http_before():
    try:
        request._usv_t0 = time.time()
    except Exception:
        pass


@app.after_request
def _telemetry_http_after(response):
    try:
        t0 = getattr(request, "_usv_t0", None)
        dt_ms = round((time.time() - (t0 if t0 is not None else time.time())) * 1000.0, 2)
        log_jsonl(
            "telemetry",
            False,
            event="http_response",
            path=request.path,
            method=request.method,
            status=int(response.status_code),
            ms=dt_ms,
        )
    except Exception:
        pass
    return response

# Shared Unified Spatial Map canvas renderer (single source)
_UNIFIED_SPATIAL_MAP_DRAW_JS_PATH = os.path.join(os.path.dirname(__file__), "unified_spatial_map_draw.js")
try:
    with open(_UNIFIED_SPATIAL_MAP_DRAW_JS_PATH, "r", encoding="utf-8") as _spatial_draw_handle:
        UNIFIED_SPATIAL_MAP_DRAW_JS = _spatial_draw_handle.read()
except OSError as exc:
    _telem_dbg.warning("unified_spatial_map_draw.js load failed: %s", exc)
    UNIFIED_SPATIAL_MAP_DRAW_JS = (
        "function drawUnifiedSpatialMap(canvas, payload) {"
        " if (!canvas || !payload) return;"
        " const ctx = canvas.getContext('2d');"
        " if (!ctx) return;"
        " ctx.fillStyle = '#050812';"
        " ctx.fillRect(0, 0, canvas.width, canvas.height);"
        " ctx.fillStyle = '#fca5a5';"
        " ctx.fillText('spatial map js missing', 12, 20);"
        "}"
    )

# --- DASHBOARD ARAYÜZÜ (HTML/CSS/JS) ---
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <title>CELEBILER USV - MISSION CONTROL</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700;800&family=JetBrains+Mono:wght@400;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg: #0a0e1a;
            --card: #131a2e;
            --card-border: rgba(255,255,255,0.06);
            --accent: #38bdf8;
            --accent-glow: rgba(56,189,248,0.15);
            --success: #22c55e;
            --danger: #ef4444;
            --warning: #eab308;
            --text: #f1f5f9;
            --text2: #64748b;
            --mono: 'JetBrains Mono', monospace;
            --sans: 'Inter', system-ui, sans-serif;
        }
        * { margin:0; padding:0; box-sizing:border-box; }
        body { background:var(--bg); color:var(--text); font-family:var(--sans); height:100vh; overflow:hidden; display:flex; flex-direction:column; }

        /* === HEADER === */
        header { background:rgba(10,14,26,0.95); border-bottom:1px solid var(--card-border); padding:0 16px; height:52px; display:flex; align-items:center; justify-content:space-between; backdrop-filter:blur(12px); z-index:10; flex-shrink:0; }
        .h-left { display:flex; align-items:center; gap:12px; }
        .h-right { display:flex; align-items:center; gap:14px; }
        .logo { font-size:1rem; font-weight:800; color:var(--accent); letter-spacing:0.5px; }
        .mode-badge { font-size:0.7rem; font-weight:700; padding:3px 10px; border-radius:20px; }
        .mode-test { background:rgba(34,197,94,0.15); color:var(--success); border:1px solid rgba(34,197,94,0.3); }
        .mode-race { background:rgba(234,179,8,0.15); color:var(--warning); border:1px solid rgba(234,179,8,0.3); }
        .hm { font-size:0.75rem; color:var(--text2); font-family:var(--mono); background:rgba(255,255,255,0.04); padding:4px 10px; border-radius:6px; border:1px solid var(--card-border); }
        .hm b { color:var(--text); }
        .clock { font-family:var(--mono); font-size:0.8rem; color:var(--text2); }

        /* === MAIN GRID === */
        .main { flex:1; display:grid; grid-template-columns:280px 1fr 1fr; grid-template-rows:1fr auto; gap:8px; padding:8px; overflow:hidden; }

        /* === CARDS === */
        .card { background:var(--card); border-radius:10px; border:1px solid var(--card-border); display:flex; flex-direction:column; overflow:hidden; }
        .card-h { padding:6px 12px; font-size:0.68rem; font-weight:700; color:var(--accent); text-transform:uppercase; letter-spacing:0.8px; border-bottom:1px solid var(--card-border); background:rgba(10,14,26,0.5); display:flex; justify-content:space-between; align-items:center; flex-shrink:0; }
        .card-b { flex:1; padding:10px; overflow:auto; }
        .dot { width:7px; height:7px; border-radius:50%; display:inline-block; }
        .dot-g { background:var(--success); box-shadow:0 0 6px var(--success); }
        .dot-r { background:var(--danger); box-shadow:0 0 6px var(--danger); }
        .dot-y { background:var(--warning); box-shadow:0 0 6px var(--warning); }

        /* === MISSION CONTROL (Left Sidebar) === */
        .mission-panel { grid-row:span 2; }
        .mission-panel .card-b { display:flex; flex-direction:column; gap:8px; padding:10px; }
        .m-timer { text-align:center; font-family:var(--mono); font-size:1.8rem; font-weight:700; color:var(--accent); padding:8px 0; }
        .m-timer small { display:block; font-size:0.65rem; color:var(--text2); font-weight:400; margin-top:2px; }
        .m-state { text-align:center; padding:6px; background:rgba(56,189,248,0.08); border-radius:8px; border:1px solid rgba(56,189,248,0.15); }
        .m-state .label { font-size:0.65rem; color:var(--text2); text-transform:uppercase; }
        .m-state .val { font-size:1rem; font-weight:700; color:var(--accent); margin-top:2px; }
        .m-wp { font-size:0.75rem; color:var(--text2); text-align:center; padding:4px 0; }
        .m-wp b { color:var(--text); }
        .m-mp-line { font-size:0.65rem; color:var(--text2); text-align:left; margin-top:6px; line-height:1.4; padding:0 2px; }
        .m-mp-line b { color:var(--accent); font-family:var(--mono); font-weight:600; }
        .m-reject-reason { font-size:0.72rem; color:var(--danger); margin-top:8px; padding:6px 8px; border-radius:6px; border:1px solid rgba(255,80,80,0.35); background:rgba(80,20,20,0.25); display:none; text-align:left; }
        .m-reject-reason.visible { display:block; }
        .conn-banner { font-size:0.78rem; color:var(--danger); margin:8px 0; padding:8px 10px; border-radius:6px; border:1px solid rgba(255,80,80,0.35); background:rgba(80,20,20,0.25); display:none; text-align:center; }
        .conn-banner.visible { display:block; }
        .m-btns { display:flex; flex-direction:column; gap:6px; margin-top:auto; }
        .btn { padding:10px; border:none; border-radius:8px; font-weight:700; font-size:0.8rem; cursor:pointer; transition:all 0.15s; text-transform:uppercase; letter-spacing:0.5px; font-family:var(--sans); }
        .btn:active { transform:scale(0.97); }
        .btn-start { background:linear-gradient(135deg,#22c55e,#16a34a); color:#fff; }
        .btn-start:hover { box-shadow:0 0 20px rgba(34,197,94,0.4); }
        .btn-next { background:linear-gradient(135deg,#38bdf8,#0284c7); color:#fff; }
        .btn-next:hover { box-shadow:0 0 20px rgba(56,189,248,0.4); }
        .btn-stop { background:linear-gradient(135deg,#ef4444,#b91c1c); color:#fff; }
        .btn-stop:hover { box-shadow:0 0 20px rgba(239,68,68,0.4); }
        .btn-close { background:linear-gradient(135deg,#8b5cf6,#6d28d9); color:#fff; }
        .btn-close:hover { box-shadow:0 0 20px rgba(139,92,246,0.4); }
        .btn:disabled { opacity:0.4; cursor:not-allowed; box-shadow:none !important; }

        /* Parkur Progress */
        .parkur-steps { display:flex; gap:4px; justify-content:center; padding:6px 0; }
        .p-step { flex:1; height:6px; border-radius:3px; background:rgba(255,255,255,0.08); transition:all 0.3s; }
        .p-step.active { background:var(--accent); box-shadow:0 0 8px var(--accent-glow); }
        .p-step.done { background:var(--success); }

        /* === FEEDS === */
        .feed { background:#000; flex:1; display:flex; justify-content:center; align-items:center; position:relative; }
        .feed img { width:100%; height:100%; object-fit:contain; }
        .feed canvas { width:100%; height:100%; display:block; background:#050812; }
        .feed-off { color:var(--text2); font-size:0.85rem; text-align:center; padding:20px; }

        /* === STATS ROW === */
        .stats-row { grid-column:span 2; display:grid; grid-template-columns:repeat(6,1fr); gap:8px; }
        .s-card { background:linear-gradient(180deg,rgba(19,26,46,0.9),rgba(10,14,26,0.9)); border-radius:8px; padding:6px 8px; border:1px solid var(--card-border); display:flex; flex-direction:column; justify-content:space-between; min-height:0; max-height:120px; }
        .s-label { font-size:0.55rem; color:var(--text2); text-transform:uppercase; font-weight:600; letter-spacing:0.5px; }
        .s-val { font-family:var(--mono); font-size:0.95rem; font-weight:700; }
        .s-sub { font-size:0.65rem; color:var(--accent); margin-top:1px; }
        .s-unit { font-size:0.7rem; color:var(--text2); font-weight:400; }

        /* Sticks */
        .sticks { display:flex; justify-content:space-around; align-items:center; padding:6px 0; }
        .stick-box { position:relative; width:42px; height:42px; border:2px solid rgba(255,255,255,0.08); border-radius:50%; background:rgba(0,0,0,0.3); }
        .stick-dot { position:absolute; width:8px; height:8px; background:var(--accent); border-radius:50%; top:17px; left:17px; transition:all 0.05s; }
        .stick-lbl { text-align:center; font-size:0.6rem; color:var(--text2); margin-top:4px; }

        /* Motor Bars */
        .mot-row { display:flex; align-items:center; gap:6px; margin-bottom:4px; }
        .mot-lbl { font-size:0.65rem; color:var(--text2); width:55px; }
        .mot-bar-bg { flex:1; height:5px; background:rgba(255,255,255,0.08); border-radius:3px; overflow:hidden; }
        .mot-bar { height:100%; background:var(--accent); transition:width 0.1s; border-radius:3px; }
        .mot-val { font-family:var(--mono); font-size:0.75rem; width:35px; text-align:right; }

        /* Race mode layout */
        .main.race-mode { grid-template-columns:280px 1fr; }
        .main.race-mode .stats-row { grid-column:span 1; grid-template-columns:repeat(3,1fr); }
    </style>
</head>
<body>
    <header>
        <div class="h-left">
            <span class="logo">⚓ CELEBILER USV</span>
            <span class="mode-badge {% if usv_mode == 'race' %}mode-race{% else %}mode-test{% endif %}">
                {% if usv_mode == 'race' %}🏁 YARIŞMA{% else %}🔧 TEST{% endif %}
            </span>
            <span class="clock" id="ts">--:--:--</span>
        </div>
        <div class="h-right">
            <span class="hm">CPU <b id="sys_cpu">--%</b></span>
            <span class="hm">RAM <b id="sys_ram">--%</b></span>
            <span class="hm">TMP <b id="sys_temp">--°C</b></span>
        </div>
    </header>

    <div class="main {% if usv_mode == 'race' %}race-mode{% endif %}">

        <!-- LEFT: MISSION CONTROL -->
        <div class="card mission-panel">
            <div class="card-h"><span>Mission Control</span><span class="dot dot-g" id="mc_dot"></span></div>
            <div class="card-b">
                <div class="conn-banner" id="conn_banner"></div>
                <div class="m-timer" id="m_timer">00:00<small>ELAPSED</small></div>

                <div class="m-state">
                    <div class="label">DURUM</div>
                    <div class="val" id="m_state">BEKLEME</div>
                </div>

                <div class="parkur-steps">
                    <div class="p-step" id="ps1"></div>
                    <div class="p-step" id="ps2"></div>
                    <div class="p-step" id="ps3"></div>
                </div>

                <div class="m-wp" id="m_wp">Waypoint: <b>-- / --</b></div>
                <div class="m-wp">Hedef: <b id="m_target">--</b></div>
                <div class="m-mp-line">Mission kaynagi: <b id="m_upload_src">--</b></div>
                <div class="m-mp-line">Pixhawk sync: <b id="m_sync_err">--</b></div>
                <div class="m-mp-line">FC start: <b id="m_fc_start">--</b></div>
                <div class="m-mp-line">Son FC mesaji: <b id="m_fc_text">--</b></div>
                <div class="m-mp-line">Nav faz: <b id="m_nav_phase">--</b></div>
                <div class="m-mp-line">Heading hata: <b id="m_nav_hdg_err">--</b></div>
                <div class="m-mp-line">Surge izni: <b id="m_nav_surge">--</b></div>
                <div class="m-mp-line">MP/GCS heartbeat yasi: <b id="m_gcs_age">--</b></div>
                <div class="m-reject-reason" id="m_reject_reason"></div>

                <div class="m-btns">
                    <button class="btn btn-start" id="btn_start" onclick="cmdStart()" {% if usv_mode == 'race' %}disabled{% endif %}>▶ Görevi Başlat</button>
                    <button class="btn btn-stop" id="btn_stop" onclick="cmdStop()">⛔ Acil Durdur</button>
                    <button class="btn btn-close" id="btn_close" onclick="cmdShutdown()">🔌 Sistemi Kapat</button>
                </div>
            </div>
        </div>

        {% if usv_mode == 'test' %}
        <!-- CAMERA FEED -->
        <div class="card">
            <div class="card-h"><span>Front Vision</span><span class="dot dot-g"></span></div>
            <div class="feed"><img id="cam_img" alt="Camera" /></div>
        </div>

        <!-- UNIFIED SPATIAL MAP -->
        <div class="card">
            <div class="card-h"><span>Unified Spatial Map</span><span class="dot dot-g"></span></div>
            <div class="feed"><canvas id="unified_map_canvas" aria-label="Unified Spatial Map"></canvas></div>
        </div>
        {% else %}
        <!-- RACE MODE: No feeds -->
        <div class="card" style="grid-column:span 1; grid-row:span 2;">
            <div class="card-h"><span>Yarışma Modu</span><span class="dot dot-y"></span></div>
            <div class="card-b" style="display:flex; align-items:center; justify-content:center; text-align:center;">
                <div>
                    <div style="font-size:2rem; margin-bottom:8px;">🏁</div>
                    <div style="font-size:0.85rem; color:var(--text2);">Görüntü aktarımı kapalı<br/>(Şartname 3.7)</div>
                    <div style="margin-top:12px; font-size:0.75rem; color:var(--text2);">Kamera onboard çalışıyor<br/>Engel tespiti aktif</div>
                </div>
            </div>
        </div>
        {% endif %}

        <!-- BOTTOM STATS -->
        <div class="stats-row">
            <!-- 1: Battery & Mode -->
            <div class="s-card">
                <div class="s-label">System</div>
                <div class="s-val" id="bat">-- <span class="s-unit">V</span></div>
                <div class="s-sub" id="mode">--</div>
            </div>

            <!-- 2: GPS -->
            <div class="s-card">
                <div class="s-label">GPS</div>
                <div class="s-val" id="lat" style="font-size:0.9rem;">--</div>
                <div style="font-size:0.9rem; font-family:var(--mono); opacity:0.7;" id="lon">--</div>
            </div>

            <!-- 3: Speed / Heading -->
            <div class="s-card">
                <div class="s-label">Nav</div>
                <div class="s-val"><span id="spd">--</span> <span class="s-unit">m/s</span></div>
                <div class="s-sub">HDG: <span id="hdg">--</span>°</div>
            </div>

            <!-- 4: Atmosphere -->
            <div class="s-card">
                <div class="s-label">Atmosphere</div>
                <div class="s-val" id="temp">-- <span class="s-unit">°C</span></div>
                <div class="s-sub">HUM: <span id="hum">--%</span> | <span id="rain" style="font-weight:700;">--</span></div>
            </div>

            <!-- 5: RC Sticks -->
            <div class="s-card">
                <div class="s-label">Pilot</div>
                <div class="sticks">
                    <div>
                        <div class="stick-box"><div class="stick-dot" id="stick_left"></div></div>
                        <div class="stick-lbl">CRUISE</div>
                    </div>
                    <div>
                        <div class="stick-box"><div class="stick-dot" id="stick_right"></div></div>
                        <div class="stick-lbl">STEER</div>
                    </div>
                </div>
            </div>

            <!-- 6: Motors -->
            <div class="s-card">
                <div class="s-label">Motors</div>
                <div style="padding-top:4px;">
                    <div class="mot-row">
                        <span class="mot-lbl">L (CH1)</span>
                        <div class="mot-bar-bg"><div class="mot-bar" id="mot1_bar" style="width:0%"></div></div>
                        <span class="mot-val" id="out1">--</span>
                    </div>
                    <div class="mot-row">
                        <span class="mot-lbl">R (CH3)</span>
                        <div class="mot-bar-bg"><div class="mot-bar" id="mot3_bar" style="width:0%"></div></div>
                        <span class="mot-val" id="out3">--</span>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // --- DATA UPDATE ---
        let missionStartTime = 0;
        let missionStartPending = false;
        let lastStartRejectAlert = '';

        const ARM_BLOCK_HINTS = {
            accel_inconsistent_recent: 'IMU/ivme sensörü henüz hazır değil. 10-15 sn bekleyip tekrar deneyin.',
            gyro_inconsistent_recent: 'Jiroskop henüz hazır değil. 10-15 sn bekleyip tekrar deneyin.',
            bad_position_recent: 'FC konum çözümü henüz arm için hazır değil. Biraz bekleyip tekrar deneyin.',
            ekf_variance_recent: 'EKF varyansı yüksek. FC oturana kadar bekleyin.',
        };

        function humanizeStartReject(reason) {
            const text = String(reason || '').trim();
            if (!text.startsWith('start_rejected:')) return text;
            const code = text.split(':').pop() || '';
            return ARM_BLOCK_HINTS[code] || text;
        }

        function showStartRejectAlert(reason) {
            const text = String(reason || '').trim();
            if (!text.startsWith('start_rejected:') || text === lastStartRejectAlert) {
                return;
            }
            lastStartRejectAlert = text;
            alert('Görev başlatılamadı: ' + humanizeStartReject(text));
        }

        function setDashboardConnectionState(ok, detail) {
            const banner = document.getElementById('conn_banner');
            const modeEl = document.getElementById('mode');
            const dotEl = document.getElementById('mc_dot');
            if (ok) {
                if (banner) {
                    banner.textContent = '';
                    banner.className = 'conn-banner';
                }
                return;
            }
            if (banner) {
                banner.textContent = detail || 'Telemetry API baglantisi yok. Sim stack calisiyor mu?';
                banner.className = 'conn-banner visible';
            }
            if (modeEl) {
                modeEl.innerText = 'BAGLANTI YOK';
                modeEl.style.color = 'var(--danger)';
            }
            if (dotEl) dotEl.className = 'dot dot-r';
        }

        function updateStats() {
            fetch('/api/data')
                .then(r => {
                    if (!r.ok) throw new Error('HTTP ' + r.status);
                    return r.json();
                })
                .then(d => {
                    setDashboardConnectionState(true);
                    // Header
                    document.getElementById('sys_cpu').innerText = d.Sys_CPU + '%';
                    document.getElementById('sys_ram').innerText = d.Sys_RAM + '%';
                    document.getElementById('sys_temp').innerText = d.Sys_Temp + '°C';
                    document.getElementById('ts').innerText = d.Timestamp || '--:--:--';

                    // System
                    document.getElementById('bat').innerHTML = (d.Battery || 0).toFixed(1) + '<span class="s-unit"> V</span>';
                    const modeEl = document.getElementById('mode');
                    modeEl.innerText = d.Mode || '--';
                    modeEl.style.color = d.Mode === 'DISCONNECTED' ? 'var(--danger)' : (d.Mode === 'SIMULATION' ? 'var(--warning)' : 'var(--success)');

                    // GPS
                    document.getElementById('lat').innerText = (d.Lat === null || d.Lat === undefined || Number.isNaN(Number(d.Lat))) ? '--' : Number(d.Lat).toFixed(7);
                    document.getElementById('lon').innerText = (d.Lon === null || d.Lon === undefined || Number.isNaN(Number(d.Lon))) ? '--' : Number(d.Lon).toFixed(7);

                    // Nav
                    document.getElementById('spd').innerText = (d.Speed || 0).toFixed(1);
                    document.getElementById('hdg').innerText = (d.Heading || 0).toFixed(0);

                    // Atmosphere
                    document.getElementById('temp').innerHTML = (d.Env_Temp || 0).toFixed(1) + '<span class="s-unit"> °C</span>';
                    document.getElementById('hum').innerText = (d.Env_Hum || 0).toFixed(0) + '%';
                    const rainEl = document.getElementById('rain');
                    rainEl.innerText = d.Rain_Status || '--';
                    rainEl.style.color = d.Rain_Status === 'WET' ? 'var(--warning)' : 'var(--success)';

                    // RC Sticks
                    const mapS = (v) => { if(!v) return 0; let n=(v-1500)/500.0; if(Math.abs(n)<0.1) n=0; return Math.max(-1,Math.min(1,n))*20; };
                    document.getElementById('stick_left').style.transform = 'translate(0px,' + (-mapS(d.RC3)) + 'px)';
                    document.getElementById('stick_right').style.transform = 'translate(' + mapS(d.RC1) + 'px,0px)';

                    // Motors
                    const mapPWM = (v) => { if(!v) return 0; let p=(v-1000)/10.0; return Math.max(0,Math.min(100,p)); };
                    document.getElementById('out1').innerText = d.Out1 || '--';
                    document.getElementById('mot1_bar').style.width = mapPWM(d.Out1) + '%';
                    document.getElementById('out3').innerText = d.Out3 || '--';
                    document.getElementById('mot3_bar').style.width = mapPWM(d.Out3) + '%';

                    // Mission Control
                    if(d.mission_state !== undefined) {
                        const mainStateMap = {0: 'BEKLEME', 1: 'NAV', 2: 'ENGAGE', 4: 'TAMAMLANDI', 5: 'HOLD'};
                        const stateEl = document.getElementById('m_state');
                        stateEl.innerText = mainStateMap[d.mission_state] ?? 'BEKLEME';

                        // Parkur progress
                        for(let i=1;i<=3;i++) {
                            const el = document.getElementById('ps'+i);
                            if(d.mission_state > i) el.className='p-step done';
                            else if(d.mission_state === i) el.className='p-step active';
                            else el.className='p-step';
                        }

                        // Buttons
                        document.getElementById('btn_start').disabled =
                            d.mission_active ||
                            d.start_requested ||
                            missionStartPending ||
                            d.usv_mode === 'race' ||
                            !d.ready_state ||
                            !d.camera_ready ||
                            !d.lidar_ready ||
                            d.guided_ready === false ||
                            d.fc_start_ready === false;
                        if (!d.start_requested && !d.mission_active && !missionStartPending) {
                            document.getElementById('btn_start').innerText = '▶ Görevi Başlat';
                        }
                        document.getElementById('btn_stop').disabled = !d.mission_active;
                        if (d.mission_active || !d.start_requested) {
                            missionStartPending = false;
                        }
                        if (d.start_phase === 'rejected' && !d.mission_active) {
                            missionStartPending = false;
                            const btn = document.getElementById('btn_start');
                            if (btn) btn.innerText = '▶ Görevi Başlat';
                        }

                        // Mission dot
                        document.getElementById('mc_dot').className = 'dot ' + (d.mission_active ? 'dot-g' : 'dot-r');
                    }

                    if(d.mission_target) document.getElementById('m_target').innerText = d.mission_target;
                    if(d.mission_wp_info) document.getElementById('m_wp').innerHTML = 'Waypoint: <b>' + d.mission_wp_info + '</b>';

                    const upSrc = document.getElementById('m_upload_src');
                    const syncEl = document.getElementById('m_sync_err');
                    if (upSrc) upSrc.innerText = d.mission_upload_source || '--';
                    if (syncEl) {
                        const mirrorStatus = (d.pixhawk_mirror_status || '').toString();
                        const mirrorError = (d.pixhawk_mirror_error || '').toString();
                        const legacyError = (d.pixhawk_mission_sync_error || '').toString();
                        const localActive = ((d.mission_upload_source || '').toString() !== 'pixhawk_mission');
                        if (mirrorStatus) {
                            const label = localActive ? 'local mission active, mirror ' : 'pixhawk mission ';
                            syncEl.innerText = label + mirrorStatus + (mirrorError ? ': ' + mirrorError : '');
                            syncEl.style.color = mirrorStatus === 'synced' ? 'var(--success)' :
                                (mirrorStatus === 'failed' ? 'var(--danger)' : 'var(--warn)');
                        } else {
                            syncEl.innerText = legacyError || 'OK';
                            syncEl.style.color = legacyError ? 'var(--danger)' : 'var(--success)';
                        }
                    }
                    const fcStartEl = document.getElementById('m_fc_start');
                    if (fcStartEl) {
                        const fcReady = Boolean(d.fc_start_ready);
                        const fcBlock = String(d.fc_start_block_reason || 'none');
                        fcStartEl.innerText = (fcReady ? 'hazir' : fcBlock);
                        fcStartEl.style.color = fcReady ? 'var(--success)' : 'var(--danger)';
                    }
                    const fcTextEl = document.getElementById('m_fc_text');
                    if (fcTextEl) {
                        const st = (d.last_statustext && d.last_statustext.text) ? String(d.last_statustext.text) : '--';
                        fcTextEl.innerText = st;
                        fcTextEl.style.color = /prearm:|arm:/i.test(st) ? 'var(--danger)' : 'var(--text2)';
                    }
                    const navPhaseEl = document.getElementById('m_nav_phase');
                    if (navPhaseEl) {
                        navPhaseEl.innerText = String(d.nav_align_phase || d.nav_align_mode || '--');
                        navPhaseEl.style.color = String(d.nav_align_phase || '').includes('ACQUIRE') ? 'var(--warning)' : 'var(--success)';
                    }
                    const navHdgEl = document.getElementById('m_nav_hdg_err');
                    if (navHdgEl) {
                        const herr = d.nav_heading_error_deg;
                        navHdgEl.innerText = (herr === null || herr === undefined || Number.isNaN(Number(herr))) ? '--' : Number(herr).toFixed(1) + ' deg';
                    }
                    const navSurgeEl = document.getElementById('m_nav_surge');
                    if (navSurgeEl) {
                        const surge = Boolean(d.surge_allowed);
                        navSurgeEl.innerText = surge ? 'evet' : 'hayir';
                        navSurgeEl.style.color = surge ? 'var(--success)' : 'var(--warning)';
                    }
                    const gcsAgeEl = document.getElementById('m_gcs_age');
                    if (gcsAgeEl) {
                        const ga = d.mission_planner_gcs_age_s;
                        const optional = d.mission_planner_status === 'optional_not_connected';
                        gcsAgeEl.innerText = (ga === null || ga === undefined) ? (optional ? 'optional' : '\u2014') : (Number(ga).toFixed(1) + ' s');
                        gcsAgeEl.style.color = (ga === null || ga === undefined) ? 'var(--text2)' : 'var(--success)';
                    }
                    const rej = document.getElementById('m_reject_reason');
                    if (rej) {
                        const mer = (d.mission_end_reason || 'none').toString();
                        const show = mer.startsWith('start_rejected:');
                        rej.textContent = show ? mer : '';
                        rej.className = 'm-reject-reason' + (show ? ' visible' : '');
                        if (show) showStartRejectAlert(mer);
                    }
                })
                .catch(() => {
                    setDashboardConnectionState(false, 'Veri alinamadi: sim stack kapali veya http://127.0.0.1:8080/dashboard erisilemiyor.');
                });
        }

        // Mission Timer
        function updateTimer() {
            fetch('/api/mission_status')
                .then(r => r.json())
                .then(d => {
                    if(d.elapsed !== undefined && d.active) {
                        const m = Math.floor(d.elapsed/60);
                        const s = Math.floor(d.elapsed%60);
                        const el = document.getElementById('m_timer');
                        el.innerHTML = String(m).padStart(2,'0') + ':' + String(s).padStart(2,'0') + '<small>ELAPSED</small>';
                        if(d.elapsed > 1200) el.style.color = 'var(--danger)';
                        else if(d.elapsed > 1080) el.style.color = 'var(--warning)';
                        else el.style.color = 'var(--accent)';
                    }
                })
                .catch(() => {});
        }

        // Commands
        function cmdStart() {
            if (missionStartPending) return;
            missionStartPending = true;
            const btn = document.getElementById('btn_start');
            btn.disabled = true;
            btn.innerText = '⏳ Start Bekleniyor';
            fetch('/api/start_mission', {method:'POST'})
                .then(async (res) => {
                    if (!res.ok) {
                        missionStartPending = false;
                        btn.innerText = '▶ Görevi Başlat';
                    }
                })
                .catch(() => {
                    missionStartPending = false;
                    btn.innerText = '▶ Görevi Başlat';
                });
        }
        function cmdStop() {
            missionStartPending = false;
            const btn = document.getElementById('btn_start');
            btn.innerText = '▶ Görevi Başlat';
            fetch('/api/emergency_stop', {method:'POST'});
        }
        function cmdShutdown() { if(confirm('Sistemi kapatmak istediğinizden emin misiniz?')) { fetch('/api/shutdown', {method:'POST'}).then(()=>{alert('Sistem kapatılıyor...');}); } }

        function bindCameraFeed(el) {
            if (!el) return;
            const host = window.location.hostname || '127.0.0.1';
            const candidates = [
                '/api/camera_stream',
                'http://' + host + ':5000/'
            ];
            if (!el.dataset.camHooked) {
                el.dataset.camHooked = '1';
                el.dataset.camIndex = el.dataset.camIndex || '0';
                el.onerror = function() {
                    const idx = Number(el.dataset.camIndex || '0');
                    el.dataset.camIndex = String((idx + 1) % candidates.length);
                    el.dataset.camBound = '0';
                };
            }
            const lastAttempt = Number(el.dataset.camAttemptTs || '0');
            const currentSrc = el.getAttribute('src') || '';
            const shouldRetry = !currentSrc || el.naturalWidth === 0 || el.dataset.camBound !== '1';
            if (!shouldRetry && (Date.now() - lastAttempt) < 5000) {
                return;
            }
            const idx = Number(el.dataset.camIndex || '0') % candidates.length;
            el.src = candidates[idx] + '?t=' + Date.now();
            el.dataset.camAttemptTs = String(Date.now());
            el.dataset.camBound = '1';
        }

        let spatialMapState = null;
        let spatialMapFetchBusy = false;

        async function refreshSpatialMapState() {
            if (spatialMapFetchBusy) return;
            spatialMapFetchBusy = true;
            try {
                const r = await fetch('/api/spatial_map');
                if (!r.ok) return;
                spatialMapState = await r.json();
            } catch (_) {
            } finally {
                spatialMapFetchBusy = false;
            }
        }

        {{ unified_spatial_map_draw_js | safe }}


        function refreshVisualFeeds() {
            {% if usv_mode == 'test' %}
            var camEl = document.getElementById('cam_img');
            var mapEl = document.getElementById('unified_map_canvas');
            bindCameraFeed(camEl);
            drawUnifiedSpatialMap(mapEl, spatialMapState);
            {% endif %}
        }

        setInterval(updateStats, 200);
        setInterval(updateTimer, 1000);
        setInterval(refreshSpatialMapState, 200);
        setInterval(refreshVisualFeeds, 200);

        // Load feeds (test mode only)
        window.onload = function() {
            refreshSpatialMapState();
            refreshVisualFeeds();
        };
    </script>
</body>
</html>
"""

def clean_port(port):
    import os
    print(f"🧹 Port {port} temizleniyor...")
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")

class SmartTelemetry:
    def __init__(self):
        self.pixhawk = None
        self.stm32 = None
        self.pixhawk_port = None
        self.stm32_port = None
        self.target_system_id = None
        
        self.running = True
        self.lock = threading.Lock()
        self.boot_monotonic = time.monotonic()
        self.last_gnss_epoch = 0.0
        self.last_gps_fix_type = 0
        self.last_gps_satellites = 0
        self.last_link_heartbeat_time = 0.0
        self.link_heartbeat_age_s = 999.0
        self.link_heartbeat_source = "telemetry"  # Initialize missing attribute
        self._last_link_state_write = 0.0
        self._warn_last = {}
        self.error_counters = {
            "csv_write_error": 0,
            "mav_read_error": 0,
            "state_read_error": 0,
            "rc_override_error": 0,
            "link_state_write_error": 0,
        }
        
        # Simülasyon Değişkenleri
        self.sim_lat = 38.4192
        self.sim_lon = 27.1287
        self.sim_heading = 0
        self._sim_pose_epoch = 0.0
        
        # Log Klasörü
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)
            
        # CSV init - Her oturumda sıfırdan (Şartname 6: İDA karaya alındıktan sonra teslim)
        with open(CSV_FILE, "w", newline="", encoding="utf-8") as f:
            csv_mod.writer(f).writerow(COLUMNS)
        self.last_csv_log_time = 0

        # Motor Kontrolcüsünü Başlat
        self.motor_ctrl = MotorController(self)

    def _csv_timestamp(self):
        """GNSS zamanını önceliklendir, yoksa monotonic fallback kullan."""
        if self.last_gnss_epoch > 0:
            return time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(self.last_gnss_epoch))
        return f"MONO+{(time.monotonic() - self.boot_monotonic):.3f}"

    def _warn_throttled(self, key, message, period_s=5.0):
        now = time.monotonic()
        last = self._warn_last.get(key, 0.0)
        if now - last >= period_s:
            print(message)
            self._warn_last[key] = now

    def _update_sim_pose_fallback(self):
        if os.environ.get("USV_SIM") != "1":
            return
        try:
            nav = load_sim_nav_state(control_dir=CONTROL_DIR)
            ts_epoch = float(nav.get("ts_epoch", 0.0) or 0.0)
            if ts_epoch <= float(getattr(self, "_sim_pose_epoch", 0.0) or 0.0):
                return
            if not nav.get("valid"):
                self._sim_pose_epoch = ts_epoch
                return
            with self.lock:
                telemetry_data["Lat"] = float(nav.get("lat"))
                telemetry_data["Lon"] = float(nav.get("lon"))
                telemetry_data["Heading"] = float(nav.get("heading_deg"))
                telemetry_data["GPS_FixType"] = max(int(telemetry_data.get("GPS_FixType", 0) or 0), GPS_MIN_FIX_TYPE)
                telemetry_data["GPS_Satellites"] = max(int(telemetry_data.get("GPS_Satellites", 0) or 0), 10)
            self.last_gps_fix_type = max(int(self.last_gps_fix_type or 0), GPS_MIN_FIX_TYPE)
            self.last_gps_satellites = max(int(self.last_gps_satellites or 0), 10)
            self._sim_pose_epoch = ts_epoch
        except FileNotFoundError:
            return
        except json.JSONDecodeError:
            return
        except Exception as exc:
            self._warn_throttled("sim_pose_fallback", f"[WARN] [SIM_GPS] vehicle_position fallback hatasi: {exc}")

    def _bump_error(self, key, message=None, period_s=5.0):
        self.error_counters[key] = int(self.error_counters.get(key, 0)) + 1
        if message:
            self._warn_throttled(
                f"err_{key}",
                f"{message} (count={self.error_counters[key]})",
                period_s=period_s,
            )

    def _publish_link_state(self, force=False):
        self._update_sim_pose_fallback()
        now = time.monotonic()
        if SIMULATION_MODE:
            self.link_heartbeat_age_s = 0.0
        elif self.pixhawk and self.last_link_heartbeat_time > 0.0:
            self.link_heartbeat_age_s = max(0.0, now - self.last_link_heartbeat_time)
        else:
            self.link_heartbeat_age_s = 999.0

        # Keep updates frequent enough for watchdog while avoiding needless disk churn.
        if not force and (now - self._last_link_state_write) < 0.20:
            return

        payload = {
            "ts_monotonic": round(now, 3),
            "link_heartbeat_age_s": round(self.link_heartbeat_age_s, 3),
            "error_counters": dict(self.error_counters),
        }
        try:
            atomic_write_json(LINK_STATE_FILE, payload)
            self._last_link_state_write = now
            # Logging only every 5 seconds to avoid spam
            if not hasattr(self, '_last_heartbeat_log') or (now - self._last_heartbeat_log) >= 5.0:
                hb_source = getattr(self, 'link_heartbeat_source', 'unknown')  # Safe attribute access
                print(f"[HEARTBEAT] Age={self.link_heartbeat_age_s:.1f}s Source={hb_source} Ts={now:.1f}")
                self._last_heartbeat_log = now
        except Exception as exc:
            self._bump_error("link_state_write_error", f"[WARN] [LINK] state yazim hatasi: {exc}")

    def force_estop_relay(self):
        """YKİ E-stop tetiklenince araci neutral + HOLD + disarm zinciriyle bastir."""
        if not self.pixhawk:
            return False
        try:
            sysid = self.target_system_id if getattr(self, "target_system_id", None) else (getattr(self.pixhawk, "target_system", 0) or 1)
            compid = getattr(self.pixhawk, "target_component", 0) or 1
            hold_mode = None
            try:
                hold_mode = (self.pixhawk.mode_mapping() or {}).get("HOLD")
            except Exception:
                hold_mode = None
            if hold_mode is not None:
                self.pixhawk.mav.set_mode_send(
                    sysid,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    hold_mode,
                )
            for _ in range(10):
                rc_override = [65535] * 8
                rc_override[0] = 1500
                rc_override[2] = 1500
                rc_override[6] = RC7_ESTOP_FORCE_PWM
                self.pixhawk.mav.rc_channels_override_send(
                    sysid,
                    compid,
                    *rc_override
                )
                time.sleep(0.04)
            self.pixhawk.mav.command_long_send(
                sysid,
                compid,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                21196,
                0,
                0,
                0,
                0,
                0,
            )
            with self.lock:
                telemetry_data["RC7"] = RC7_ESTOP_FORCE_PWM
                telemetry_data["Out1"] = 1500
                telemetry_data["Out3"] = 1500
                telemetry_data["CMD_Port"] = 1500
                telemetry_data["CMD_Stbd"] = 1500
                telemetry_data["CMD_Source"] = "ESTOP"
                telemetry_data["Manual_Lock_Reason"] = "ESTOP"
            print("🚨 [ESTOP] RC7+HOLD+PWM1500+DISARM gönderildi")
            return True
        except Exception as e:
            self._bump_error("rc_override_error", f"[WARN] [ESTOP] RC7 force hatasi: {e}")
            return False

    def _sync_setpoints_from_state(self):
        try:
            if os.path.exists(STATE_FILE):
                with open(STATE_FILE, "r", encoding="utf-8") as f:
                    state = json.load(f)
                if isinstance(state, dict) and state.get("active", False):
                    v_t = state.get("v_target")
                    h_t = state.get("heading_target")
                    if v_t is not None:
                        telemetry_data["Speed_Setpoint"] = round(float(v_t), 3)
                    if h_t is not None:
                        telemetry_data["Heading_Setpoint"] = round(float(h_t), 3)
            motor_cmd = _read_motor_command()
            cmd_port = _pick_primary_fallback(
                motor_cmd.get("cmd_port_pwm"),
                telemetry_data.get("Out1"),
                default=1500,
            )
            cmd_stbd = _pick_primary_fallback(
                motor_cmd.get("cmd_stbd_pwm"),
                telemetry_data.get("Out3"),
                default=1500,
            )
            telemetry_data["Out1"] = cmd_port
            telemetry_data["Out3"] = cmd_stbd
            telemetry_data["CMD_Port"] = cmd_port
            telemetry_data["CMD_Stbd"] = cmd_stbd
            telemetry_data["CMD_Source"] = str(motor_cmd.get("source", "--") or "--")
        except Exception:
            pass

    def _csv_row(self):
        state = _read_mission_state()
        obstacle_state = (
            state.get("obstacle_threat_source")
            or state.get("avoidance_source")
            or state.get("lidar_frame_status")
            or "none"
        )
        return [
            self._csv_timestamp(),
            telemetry_data.get("Lat"),
            telemetry_data.get("Lon"),
            telemetry_data.get("Speed"),
            telemetry_data.get("Roll"),
            telemetry_data.get("Pitch"),
            telemetry_data.get("Heading"),
            telemetry_data.get("Speed_Setpoint"),
            telemetry_data.get("Heading_Setpoint"),
            telemetry_data.get("CMD_Port", telemetry_data.get("Out1", 1500)),
            telemetry_data.get("CMD_Stbd", telemetry_data.get("Out3", 1500)),
            telemetry_data.get("Mode"),
            state.get("active_waypoint_index", -1),
            obstacle_state,
        ]

    def csv_logger(self):
        """Şartname Bölüm 6: Telemetri CSV 1 Hz kayıt"""
        while self.running:
            try:
                if time.time() - self.last_csv_log_time >= CSV_LOG_INTERVAL:
                    self._sync_setpoints_from_state()
                    with self.lock:
                        row = self._csv_row()
                    with open(CSV_FILE, 'a', newline='') as f:
                        csv_mod.writer(f).writerow(row)
                    self.last_csv_log_time = time.time()
            except Exception as exc:
                self._bump_error("csv_write_error", f"[WARN] [CSV] Yazma hatasi: {exc}")
            time.sleep(0.2)

    def start(self):
        """Tüm threadleri başlatır."""
        threads = [
            threading.Thread(target=self.read_pixhawk, daemon=True),
            threading.Thread(target=self.read_stm32, daemon=True),
            threading.Thread(target=self.connection_manager, daemon=True),
            threading.Thread(target=self.csv_logger, daemon=True),
        ]
        
        for t in threads:
            t.start()
        
        print(f"🌍 WEB SERVER BAŞLATILIYOR: Port {WEB_PORT}")
        print(f"📋 [CSV] Telemetri 1 Hz kayıt: {CSV_FILE}")
        _telem_dbg.info(
            "flask app.run host=0.0.0.0 port=%s csv=%s SIMULATION_MODE=%s",
            WEB_PORT,
            CSV_FILE,
            SIMULATION_MODE,
        )
        app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False, threaded=True)

    # --- BAĞLANTI YÖNETİMİ ---
    def connection_manager(self):
        """Cihaz bağlantılarını periyodik olarak kontrol eder."""
        print("🕵️ [SYSTEM] Bağlantı Yöneticisi Devrede...")
        first_scan = True
        
        while self.running:
            if not self.pixhawk or not self.stm32:
                if not first_scan:
                    pass # print("🔍 [SCAN] Eksik cihazlar taranıyor...")
                self.scan_ports()
            
            # Simülasyon Kontrolü
            global SIMULATION_MODE
            if not self.pixhawk and not self.stm32:
                if not SIMULATION_MODE:
                    print("⚠️ [SYSTEM] Donanım Yok -> SİMÜLASYON MODU AKTİF")
                    SIMULATION_MODE = True
                self.update_simulation()
            else:
                if SIMULATION_MODE:
                    print("🌊 [SYSTEM] Donanım Bulundu -> SİMÜLASYON KAPATILDI")
                    SIMULATION_MODE = False
                
                # Periyodik olarak veri akışını tazele (Her 30 saniyede bir)
                stream_counter = getattr(self, '_stream_refresh_counter', 0) + 1
                self._stream_refresh_counter = stream_counter
                if self.pixhawk and stream_counter % 3 == 0:  # 3 * 10s = 30s
                     try:
                         self._request_mavlink_streams(self.pixhawk)
                     except Exception as exc:
                         self._warn_throttled("stream_refresh", f"[WARN] [MAV] Stream refresh hatasi: {exc}")

                self._publish_link_state()
            
            first_scan = False
            # KRITIK: Heartbeat her 1 saniyede yazılması gerekiyor (usv_main 30s timeout için)
            time.sleep(1)  # Reduced from 10s to ensure frequent heartbeat updates

    def scan_ports(self):
        """Bostaki portlari tarar ve uygun cihazlari eslestirir."""
        # Sim tarafında telemetry akışı doğrudan UDP 14550'ye yayınlanır.
        if not self.pixhawk:
            if self._probe_pixhawk('udpin:0.0.0.0:14550'):
                print("📡 [MAV] Pixhawk bağlantısı: UDP:14550 (SITL telemetry)")
            else:
                # Fallback: doğrudan seri port tara
                all_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
                active_ports = [self.stm32_port]
                active_ports = [p for p in active_ports if p]
                for port in all_ports:
                    if port in active_ports: continue
                    if self._probe_pixhawk(port):
                        break

        # STM32 Tara
        if not self.stm32:
            all_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            active_ports = [self.pixhawk_port]
            active_ports = [p for p in active_ports if p]
            scan_list = [p for p in all_ports if p not in active_ports]
            if scan_list:
                print(f"🔍 [SCAN] STM32 aranacak portlar: {scan_list}")
                for port in scan_list:
                    if self._probe_stm32(port):
                        break

    def _probe_stm32(self, port):
        """Belirtilen portta STM32 var mı bakar (Otomatik Baud)."""
        for baud in BAUD_RATES_STM32:
            try:
                s = serial.Serial(port, baud, timeout=2)
                time.sleep(2.0)
                s.reset_input_buffer()
                
                detected = False
                for _ in range(5):
                    try:
                        line = s.readline().decode('utf-8', errors='ignore').strip()
                        if "tarih" in line and "temp" in line:
                            print(f"✅ STM32 Bulundu: {port} @ {baud}")
                            with self.lock:
                                self.stm32 = s
                                self.stm32_port = port
                            detected = True
                            break
                    except Exception as exc:
                        self._warn_throttled("probe_stm32_line", f"[WARN] [STM32] Satir okuma hatasi: {exc}")
                
                if detected: return True
                s.close()
            except Exception as exc:
                self._warn_throttled("probe_stm32", f"[WARN] [STM32] Probe hatasi ({port}@{baud}): {exc}")
        return False

    def _probe_pixhawk(self, port):
        """Belirtilen portta Pixhawk var mı bakar."""
        try:
            master = mavutil.mavlink_connection(port, baud=BAUD_RATE_PIXHAWK)
            if master.wait_heartbeat(timeout=1):
                print(f"✅ Pixhawk Bulundu: {port}")
                self._request_mavlink_streams(master)
                with self.lock:
                    self.pixhawk = master
                    self.pixhawk_port = port
                self.last_link_heartbeat_time = time.monotonic()
                return True
            master.close()
        except Exception as exc:
            self._warn_throttled("probe_pixhawk", f"[WARN] [MAV] Pixhawk probe hatasi ({port}): {exc}")
        return False

    def _request_mavlink_streams(self, master):
        """Pixhawk'tan gerekli veri akışlarını ister."""
        if not master: return
        # Genel veriler (profil: GENERAL_TELEMETRY_HZ)
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, GENERAL_TELEMETRY_HZ, 1
        )
        # RC kanalları
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, GENERAL_TELEMETRY_HZ, 1
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
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
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
        except Exception as exc:
            print(f"[WARN] [MAV] SET_MESSAGE_INTERVAL basarisiz: {exc}")
        print("📨 [MAV] Veri Akış İsteği Gönderildi")

    # --- VERİ OKUMA THREADLERİ ---
    def read_stm32(self):
        while self.running:
            self.update_system_metrics() # Her turda sistem yükünü güncelle
            
            if not self.stm32:
                time.sleep(1); continue
                
            try:
                if self.stm32.in_waiting > 0:
                    line = self.stm32.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("{"):
                        import json
                        data = json.loads(line)
                        with self.lock:
                            telemetry_data["STM_Date"] = data.get("tarih", "--:--:--")
                            telemetry_data["Env_Temp"] = float(data.get("temp", 0))
                            telemetry_data["Env_Hum"] = float(data.get("hum", 0))
                            telemetry_data["Rain_Val"] = int(data.get("rain", 4095))
                            telemetry_data["Rain_Status"] = "WET" if telemetry_data["Rain_Val"] < 2000 else "DRY"
            except Exception as e:
                print(f"❌ STM32 Error: {e}")
                _telem_dbg.exception("stm32 read: %s", e)

    def read_pixhawk(self):
        print("📡 [MAV] Pixhawk Dinleme Servisi Aktif")
        rc_logged = False
        
        while self.running:
            if not self.pixhawk:
                self._publish_link_state()
                time.sleep(0.1); continue
                
            try:
                msg = self.pixhawk.recv_match(blocking=True, timeout=1.0)
                if msg:
                    self._process_mavlink_msg(msg, rc_logged)
                    if msg.get_type() == 'RC_CHANNELS' and not rc_logged:
                        if msg.chan3_raw > 0:
                            print(f"🎮 [RC] Sinyal Tespit Edildi: CH3={msg.chan3_raw}")
                            rc_logged = True
                    self._publish_link_state()
                else:
                    log_jsonl("telemetry", False, event="rx_timeout", blocking_timeout_s=1.0)
                    self._publish_link_state()
            except Exception as e:
                self._bump_error("mav_read_error", f"[WARN] [MAV] Okuma hatasi: {e}")
                self._publish_link_state(force=True)  # Force write on error

    def _process_mavlink_msg(self, msg, rc_logged):
        """MAVLink mesajlarını işleyen yardımcı metot."""
        mtype = msg.get_type()
        log_jsonl("telemetry", False, event="rx", mtype=mtype)

        with self.lock:
            telemetry_data["Timestamp"] = self._csv_timestamp()
            
            if mtype == 'GLOBAL_POSITION_INT':
                lat = float(getattr(msg, 'lat', 0) or 0) / 1e7
                lon = float(getattr(msg, 'lon', 0) or 0) / 1e7
                sim_position_valid = (
                    bool(os.environ.get("USV_SIM") == "1")
                    and -90.0 <= lat <= 90.0
                    and -180.0 <= lon <= 180.0
                    and (abs(lat) > 1e-6 or abs(lon) > 1e-6)
                )
                gps_valid = (
                    (self.last_gps_fix_type >= GPS_MIN_FIX_TYPE or sim_position_valid)
                    and -90.0 <= lat <= 90.0
                    and -180.0 <= lon <= 180.0
                    and (abs(lat) > 1e-6 or abs(lon) > 1e-6)
                )
                if gps_valid:
                    telemetry_data['Lat'] = lat
                    telemetry_data['Lon'] = lon
                elif self.last_gps_fix_type < GPS_MIN_FIX_TYPE and os.environ.get("USV_SIM") != "1":
                    telemetry_data['Lat'] = None
                    telemetry_data['Lon'] = None
                sim_heading_authority = (
                    os.environ.get("USV_SIM") == "1"
                    and bool(load_sim_nav_state(control_dir=CONTROL_DIR).get("valid"))
                )
                trust_zero_hdg = os.environ.get("USV_SIM") != "1"
                raw_heading = float(getattr(msg, 'hdg', 65535) or 65535)
                if (not sim_heading_authority) and mavlink_heading_cdeg_valid(
                    raw_heading, trust_zero=trust_zero_hdg
                ):
                    telemetry_data['Heading'] = raw_heading / 100.0
                # Manuel modda yön setpoint = mevcut yön; AUTO/GUIDED'da NAV_CONTROLLER_OUTPUT kullanılır
                mode_str = telemetry_data.get('Mode', '')
                if 'AUTO' not in mode_str and 'GUIDED' not in mode_str:
                    telemetry_data['Heading_Setpoint'] = telemetry_data['Heading']

            elif mtype == 'GPS_RAW_INT':
                # time_usec GNSS epoch ise CSV zamanında önceliklidir
                t_usec = getattr(msg, 'time_usec', 0)
                if t_usec and t_usec > 1e14:
                    self.last_gnss_epoch = t_usec / 1e6
                fix_type = int(getattr(msg, 'fix_type', 0) or 0)
                satellites = int(getattr(msg, 'satellites_visible', 0) or 0)
                if os.environ.get("USV_SIM") == "1" and fix_type < GPS_MIN_FIX_TYPE:
                    self.last_gps_fix_type = max(int(self.last_gps_fix_type or 0), GPS_MIN_FIX_TYPE)
                    self.last_gps_satellites = max(int(self.last_gps_satellites or 0), 10)
                    telemetry_data['GPS_FixType'] = max(int(telemetry_data.get('GPS_FixType', 0) or 0), GPS_MIN_FIX_TYPE)
                    telemetry_data['GPS_Satellites'] = max(int(telemetry_data.get('GPS_Satellites', 0) or 0), 10)
                else:
                    self.last_gps_fix_type = fix_type
                    self.last_gps_satellites = satellites
                    telemetry_data['GPS_FixType'] = fix_type
                    telemetry_data['GPS_Satellites'] = satellites
                gps_lat = float(getattr(msg, 'lat', 0) or 0) / 1e7
                gps_lon = float(getattr(msg, 'lon', 0) or 0) / 1e7
                gps_valid = (
                    fix_type >= GPS_MIN_FIX_TYPE
                    and -90.0 <= gps_lat <= 90.0
                    and -180.0 <= gps_lon <= 180.0
                    and (abs(gps_lat) > 1e-6 or abs(gps_lon) > 1e-6)
                )
                if gps_valid:
                    telemetry_data['Lat'] = gps_lat
                    telemetry_data['Lon'] = gps_lon
                elif os.environ.get("USV_SIM") != "1":
                    telemetry_data['Lat'] = None
                    telemetry_data['Lon'] = None
                
            elif mtype == 'RC_CHANNELS':
                telemetry_data['RC1'] = msg.chan1_raw
                telemetry_data['RC2'] = msg.chan2_raw
                telemetry_data['RC3'] = msg.chan3_raw
                telemetry_data['RC4'] = msg.chan4_raw
                telemetry_data['RC7'] = getattr(msg, 'chan7_raw', 0)
                
                # Motor Kontrolcüsüne Veri Gönder
                # CH1/4 steer, CH2/3 throttle fallback
                self.motor_ctrl.update_inputs(msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw)
                
                # --- RC DEBUG (Kanal Tespiti) ---
                # Her 25 mesajda bir (~5 saniyede 1) loga bas — CPU tasarrufu
                self.rc_debug_counter = getattr(self, 'rc_debug_counter', 0) + 1
                if self.rc_debug_counter % 25 == 0:
                    print(f"🎮 RC: 1:{msg.chan1_raw} 2:{msg.chan2_raw} 3:{msg.chan3_raw} 4:{msg.chan4_raw}")

                self._update_physics_sim(msg.chan1_raw, msg.chan3_raw)
                
            elif mtype == 'SYS_STATUS':
                telemetry_data['Battery'] = msg.voltage_battery / 1000.0
                
            elif mtype == 'HEARTBEAT':
                # --- HEARTBEAT DEBUG ---
                src_sys = msg.get_srcSystem()
                
                # 1. AUTO-DISCOVERY (Önceki mantığın aynısı, loglu)
                if getattr(self, 'target_system_id', None) is None:
                    # GCS ve diğerlerini ele
                    ignore_types = [
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                        mavutil.mavlink.MAV_TYPE_GIMBAL,
                        mavutil.mavlink.MAV_TYPE_ADSB
                    ]
                    
                    if msg.type not in ignore_types and msg.type <= 30: 
                         print(f"🎯 [SYSTEM LOCKED] Kilitlenen Sistem ID: {src_sys} (Type: {msg.type})")
                         self.target_system_id = src_sys
                         self.last_link_heartbeat_time = time.monotonic()
                    else:
                        # GCS'i sessizce geç veya çok nadir bas
                        pass
                
                # 2. FİLTRE (Sadece hedef sistem)
                if getattr(self, 'target_system_id', None) != src_sys:
                    return
                self.last_link_heartbeat_time = time.monotonic()

                # 3. MOD DEĞİŞİM LOGU
                custom_mode = msg.custom_mode
                previous_mode = getattr(self, 'last_mode_debug', None)
                
                if previous_mode != custom_mode:
                     print(f"🔄 [MODE CHANGE] Yeni Mod: {custom_mode} (Eski: {previous_mode}) - Base: {msg.base_mode}")
                     self.last_mode_debug = custom_mode
                elif time.time() % 10 < 0.1: # Arada sırada heartbeat olduğunu hatırlat
                     print(f"💓 [ALIVE] SysID: {src_sys} Mode: {custom_mode}")

                # ArduRover Mode Mapping (Tam Liste)
                # Kaynak: https://ardupilot.org/rover/docs/parameters.html#mode1
                modes = {
                    0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD',
                    5: 'LOITER', 6: 'FOLLOW', 7: 'SIMPLE', 10: 'AUTO',
                    11: 'RTL', 12: 'SMART_RTL', 15: 'GUIDED', 16: 'INITIALISING'
                }
                
                mode_name = modes.get(custom_mode, f"UNKNOWN({custom_mode})")
                
                # Ekranda "AUTO (ARMED)" şeklinde görünsün
                mode_str = mode_name
                if not msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    mode_str += " (DISARMED)"
                else:
                    mode_str += " (ARMED)"
                    
                telemetry_data['Mode'] = mode_str
                
            elif mtype == 'VFR_HUD':
                telemetry_data['Speed'] = msg.groundspeed
                # Manuel çıkış nötr civarındaysa mevcut hız setpoint olarak korunur.
                if abs(telemetry_data.get('Out1', 1500) - 1500) + abs(telemetry_data.get('Out3', 1500) - 1500) < 50:
                    telemetry_data['Speed_Setpoint'] = msg.groundspeed

            elif mtype == 'NAV_CONTROLLER_OUTPUT':
                # Otonom modda hedef yön (Şartname 6)
                telemetry_data['Heading_Setpoint'] = msg.target_bearing

            elif mtype == 'ATTITUDE':
                telemetry_data['Roll'] = msg.roll * 57.2958
                telemetry_data['Pitch'] = msg.pitch * 57.2958

    def _update_physics_sim(self, rc1, rc3):
        """RC girdilerine göre sanal fizik motorunu çalıştırır."""
        try:
            if rc1 is None or rc3 is None: return
            
            # Normalize (-1.0 to 1.0)
            throttle = (rc3 - 1500) / 500.0
            steer = (rc1 - 1500) / 500.0
            
            # Deadzone
            if abs(throttle) < 0.1: throttle = 0
            if abs(steer) < 0.1: steer = 0
            
            # Dönüş
            self.sim_heading += steer * 5.0
            self.sim_heading %= 360
            
            # İlerleme
            rad = math.radians(self.sim_heading)
            speed_coef = 0.00001 * 5.0 # Hız çarpanı
            self.sim_lat += math.cos(rad) * throttle * speed_coef
            self.sim_lon += math.sin(rad) * throttle * speed_coef
            
            # Telemetriyi Ez (Görsel Test İçin)
            # Not: Gerçek GPS varsa bunu kapatmak isteyebiliriz ama şimdilik hibrit.
            # telemetry_data['Lat'] = self.sim_lat
            # telemetry_data['Lon'] = self.sim_lon
        except Exception as exc:
            self._warn_throttled("physics_sim", f"[WARN] [SIM] Fizik guncelleme hatasi: {exc}")

    # --- EKLENEN EKSİK METOTLAR (Move from bottom) ---
    def update_system_metrics(self):
        """RPi Kaynak Tüketimi (CPU/RAM/Temp)."""
        try:
            # CPU
            load1, _, _ = os.getloadavg()
            cpu = int((load1 / 4.0) * 100)
            
            # RAM
            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()
                total = int(lines[0].split()[1])
                avail = int(lines[2].split()[1])
                ram = int(100 * (1 - (avail/total)))
            
            # Temp
            temp = 0
            if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    temp = int(int(f.read()) / 1000)
                    
            with self.lock:
                telemetry_data["Sys_CPU"] = min(cpu, 100)
                telemetry_data["Sys_RAM"] = ram
                telemetry_data["Sys_Temp"] = temp
        except Exception as exc:
            self._warn_throttled("sys_metrics", f"[WARN] [SYS] Metric okuma hatasi: {exc}")

    def update_simulation(self):
        """Donanım yoksa rastgele veriler üretir."""
        with self.lock:
            # Rastgele Drift
            self.sim_lat += random.uniform(-0.00005, 0.00005)
            self.sim_lon += random.uniform(-0.00005, 0.00005)
            
            hdg = (telemetry_data.get("Heading", 0) + 1) % 360
            spd = random.uniform(0, 2.0)
            telemetry_data.update({
                "Timestamp": time.strftime("%H:%M:%S"),
                "Lat": self.sim_lat,
                "Lon": self.sim_lon,
                "Heading": hdg,
                "Heading_Setpoint": hdg,
                "Battery": 12.0 + random.uniform(0, 0.5),
                "Speed": spd,
                "Speed_Setpoint": spd,
                "Mode": "SIMULATION",
                # STM32 Mock
                "STM_Date": time.strftime("%H:%M:%S"),
                "Env_Temp": 24.5 + random.uniform(-0.5, 0.5),
                "Env_Hum": 45.0 + random.uniform(-2, 2),
                "Rain_Val": int(random.uniform(4000, 4095)),
                "Rain_Status": "DRY"
            })

# --- MOTOR KONTROL (CRUISE CONTROL & SOFT START) ---
class MotorController:
    def __init__(self, parent):
        self.parent = parent # SmartTelemetry referansı (Mavlink erişimi için)
        self.active = True
        
        # Durum Değişkenleri
        self.target_pwm = 1500.0   # Hedeflenen diferansiyel thrust merkezi
        self.current_pwm = 1500.0  # Anlık fiziksel thrust merkezi (ramping uygulanır)
        
        # Girdiler
        self.input_throttle = 1500 # CH3 (Sol Stick)
        self.input_steer = 1500    # CH1 (Sağ Stick)
        self.left_motor_pwm = 1500
        self.right_motor_pwm = 1500
        self.last_input_ts = time.monotonic()
        
        # Ayarlar
        self.PWM_DEADZONE = 50     # 1450-1550 arası stick hareketsiz sayılır
        
        self.MAX_FWD = 1900
        self.MAX_REV = 1100
        
        self.MAX_FWD = 1900
        self.MAX_REV = 1100
        
        # HIZLANMA / YAVAŞLAMA AYARLARI
        # Kullanıcı İsteği: %15 İvmelenme (Hızlı tepki ama yine de rampalı)
        self.RAMP_STEP = 15.0       # Gaz Rampa Hızı (0.5 -> 15.0 : Çok daha seri)
        
        # DÖNÜŞ YUMUŞATMA (Steering Ramping)
        # Kullanıcı İsteği: "Step step olsun" (Ani dönüşleri engelle)
        self.STEER_RAMP_STEP = 10.0 # Dönüş Rampa Hızı
        self.current_turn = 0.0     # Şu anki dönüş değeri (Rampalanmış)
        
        self.CRUISE_STEP = 2.0     # Sol Stick Hız Değişim Hassasiyeti
        
        # YÖN ÇEVİRME (INVERT) - Varsayılan False (Standart)
        self.INV_THROTTLE = False   
        self.INV_STEER = False      
        
        # Thread
        self.thread = threading.Thread(target=self.control_loop, daemon=True)
        self.thread.start()

    def update_inputs(self, rc1, rc2, rc3, rc4):
        """RC verilerini filtreleyip güvenli girişlere dönüştür."""
        # Primary/fallback: throttle rc2->rc3, steer rc4->rc1
        self.input_throttle = _pick_primary_fallback(rc2, rc3, default=1500)
        self.input_steer = _pick_primary_fallback(rc4, rc1, default=1500)
        self.last_input_ts = time.monotonic()

    def control_loop(self):
        print("⚙️ [MOTOR] Kontrolcü Aktif. Yönler: INV_THR={}, INV_STR={}".format(self.INV_THROTTLE, self.INV_STEER))
        while self.active:
            # 25 Hz Döngü (RC data 5Hz, motor smoothing yeterli)
            time.sleep(0.04)

            # Görev aktif/lock/estop durumunda tek komut üreticisi usv_main olmalıdır.
            state = _read_mission_state()
            mode_str = str(telemetry_data.get("Mode", "")).upper()
            mode_locked = ("AUTO" in mode_str) or ("GUIDED" in mode_str) or ("HOLD" in mode_str)
            try:
                state_ts = float(state.get("ts_monotonic", 0.0) or 0.0)
            except (TypeError, ValueError):
                state_ts = 0.0
            state_stale = bool(state_ts <= 0.0 or (time.monotonic() - state_ts) > 2.0)
            autonomy_owner = bool(
                state.get("active", False)
                or state.get("command_lock", False)
                or mode_locked
            )
            safety_lock = bool(
                state.get("estop_state", False)
                or state_stale
            )
            mission_locked = bool(autonomy_owner or safety_lock)
            if state.get("active", False):
                lock_reason = "MISSION_ACTIVE"
            elif state.get("command_lock", False):
                lock_reason = "COMMAND_LOCK"
            elif state.get("estop_state", False):
                lock_reason = "ESTOP"
            elif mode_locked:
                lock_reason = "MODE_LOCK"
            elif state_stale:
                lock_reason = "STATE_STALE"
            else:
                lock_reason = "--"
            if mission_locked:
                self.target_pwm = 1500
                self.current_pwm = 1500
                self.current_turn = 0.0
                self.left_motor_pwm = 1500
                self.right_motor_pwm = 1500
                if safety_lock and self.parent.pixhawk:
                    try:
                        rc_override = [65535] * 8
                        rc_override[0] = 1500
                        rc_override[2] = 1500
                        self.parent.pixhawk.mav.rc_channels_override_send(
                            self.parent.target_system_id if self.parent.target_system_id else 1,
                            self.parent.pixhawk.target_component,
                            *rc_override
                        )
                    except Exception as exc:
                        self.parent._bump_error("rc_override_error", f"[WARN] [MOTOR] Lock RC override hatasi: {exc}")
                with self.parent.lock:
                    if safety_lock:
                        telemetry_data["Out1"] = 1500
                        telemetry_data["Out3"] = 1500
                        telemetry_data["CMD_Port"] = 1500
                        telemetry_data["CMD_Stbd"] = 1500
                        telemetry_data["CMD_Source"] = "SAFETY_LOCK"
                    telemetry_data["Manual_Lock_Reason"] = lock_reason
                continue

            # RC güncellemesi kesilirse manuel çıkışları nötre al.
            if (time.monotonic() - self.last_input_ts) > RC_SIGNAL_TIMEOUT_S:
                self.target_pwm = 1500
                self.current_pwm = 1500
                self.current_turn = 0.0
                self.input_throttle = 1500
                self.input_steer = 1500
                self.left_motor_pwm = 1500
                self.right_motor_pwm = 1500
                if self.parent.pixhawk:
                    try:
                        rc_override = [65535] * 8
                        rc_override[0] = 1500
                        rc_override[2] = 1500
                        self.parent.pixhawk.mav.rc_channels_override_send(
                            self.parent.target_system_id if self.parent.target_system_id else 1,
                            self.parent.pixhawk.target_component,
                            *rc_override
                        )
                    except Exception as exc:
                        self.parent._bump_error("rc_override_error", f"[WARN] [MOTOR] RC timeout override hatasi: {exc}")
                with self.parent.lock:
                    telemetry_data["Out1"] = 1500
                    telemetry_data["Out3"] = 1500
                    telemetry_data["CMD_Port"] = 1500
                    telemetry_data["CMD_Stbd"] = 1500
                    telemetry_data["CMD_Source"] = "RC_TIMEOUT"
                    telemetry_data["Manual_Lock_Reason"] = "RC_TIMEOUT"
                continue
             
            # --- 1. DIFFERENTIAL THRUST GİRDİSİ (vites yok, signed throttle) ---
            throttle_raw_diff = self.input_throttle - 1500
            if self.INV_THROTTLE: throttle_raw_diff *= -1
            
            if abs(throttle_raw_diff) > self.PWM_DEADZONE:
                power_factor = (abs(throttle_raw_diff) - self.PWM_DEADZONE) / (500.0 - self.PWM_DEADZONE)
                desired_throttle_pwm = 1500 + math.copysign(power_factor * 400.0, throttle_raw_diff)
            else:
                desired_throttle_pwm = 1500.0
            self.target_pwm += (desired_throttle_pwm - self.target_pwm) * 0.18
            self.target_pwm = max(self.MAX_REV, min(self.MAX_FWD, self.target_pwm))

            # --- 2. EXPONENTIAL RAMPING (YUMUŞAK GEÇİŞ - LPF) ---
            # Lineer rampa yerine üstel filtre (Low Pass Filter) ile daha gerçekçi tekne ataleti
            alpha_pwm = 0.15  # 0.0 - 1.0 (küçük değerler = daha yavaş/yumuşak hızlanma)
            self.current_pwm += (self.target_pwm - self.current_pwm) * alpha_pwm
            
            # Küsüratları temizle ve tam durmayı garantiye al
            if abs(self.target_pwm - self.current_pwm) < 2.0:
                self.current_pwm = self.target_pwm

            # --- 3. MIXING (NORMALIZED DIFFERENTIAL DRIVE) ---
            # Pixhawk SERVO_PASS_THROUGH ayarlandığında tüm miksaj Python'da yapılır:
            throttle_norm = (self.current_pwm - 1500) / 400.0  # -1.0 to 1.0
            
            steer_input = (self.input_steer - 1500)
            if self.INV_STEER: steer_input *= -1
            
            # Joystick deadband
            if abs(steer_input) < 25: steer_input = 0.0
            
            alpha_steer = 0.2
            self.current_turn += (steer_input - self.current_turn) * alpha_steer
            steer_norm = self.current_turn / 400.0
            steer_norm = max(-1.0, min(steer_norm, 1.0))
            
            # Normalized Mixing
            left_mix = throttle_norm + steer_norm
            right_mix = throttle_norm - steer_norm
            
            # Limitleri aşarsa orantılı kıs (Normalization)
            max_mag = max(abs(left_mix), abs(right_mix))
            if max_mag > 1.0:
                left_mix /= max_mag
                right_mix /= max_mag
                
            left_pwm_out = int(1500 + left_mix * 400)
            right_pwm_out = int(1500 + right_mix * 400)
            
            # --- 4. SAFETY CLAMP ---
            left_pwm_out = int(max(1100, min(left_pwm_out, 1900)))
            right_pwm_out = int(max(1100, min(right_pwm_out, 1900)))
            
            # Düşük pwm kesilmesi
            if abs(left_pwm_out - 1500) < 15: left_pwm_out = 1500
            if abs(right_pwm_out - 1500) < 15: right_pwm_out = 1500
            
            self.left_motor_pwm = left_pwm_out
            self.right_motor_pwm = right_pwm_out
            
            # --- 5. ÇIKIŞ (MAVLINK OVERRIDE) ---
            if self.parent.pixhawk:
                try:
                    rc_override = [65535]*8
                    # Pixhawk CH1 ve CH3 Pass-Through olduğu için direkt Sol ve Sağ gönderilir.
                    rc_override[0] = left_pwm_out     # CH1 -> SERVO1 (Left ESC)
                    rc_override[2] = right_pwm_out    # CH3 -> SERVO3 (Right ESC)
                    
                    self.parent.pixhawk.mav.rc_channels_override_send(
                        self.parent.target_system_id if self.parent.target_system_id else 1,
                        self.parent.pixhawk.target_component,
                        *rc_override
                    )
                    
                    with self.parent.lock:
                        telemetry_data["Out1"] = left_pwm_out
                        telemetry_data["Out3"] = right_pwm_out
                        telemetry_data["CMD_Port"] = left_pwm_out
                        telemetry_data["CMD_Stbd"] = right_pwm_out
                        telemetry_data["CMD_Source"] = "MANUAL_ACTIVE"
                        telemetry_data["Manual_Lock_Reason"] = "MANUAL_ACTIVE"
                        
                        thrust_norm = (self.current_pwm - 1500) / 400.0
                        telemetry_data["Speed_Setpoint"] = thrust_norm * 3.0
                        telemetry_data["RC1"] = self.input_steer
                        telemetry_data["RC3"] = self.input_throttle
                        
                except Exception as e:
                    self.parent._bump_error("rc_override_error", f"[WARN] [MOTOR] Override hatasi: {e}")



CONTROLLER_PAGE = """
<!DOCTYPE html>
<html lang="tr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>CELEBILER USV Controller</title>
    <style>
        :root {
            --bg: #0a1220;
            --card: #121d30;
            --card2: #0f1726;
            --line: rgba(255,255,255,0.08);
            --text: #e5eefc;
            --muted: #8da2c0;
            --accent: #3fb0ff;
            --ok: #22c55e;
            --warn: #f59e0b;
            --bad: #ef4444;
            --mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
            --sans: Inter, system-ui, sans-serif;
        }
        * { box-sizing: border-box; }
        body {
            margin: 0;
            font-family: var(--sans);
            background:
                radial-gradient(circle at top left, rgba(63,176,255,0.18), transparent 28%),
                radial-gradient(circle at top right, rgba(34,197,94,0.12), transparent 24%),
                radial-gradient(circle at top, #152846 0%, var(--bg) 45%);
            color: var(--text);
            min-height: 100vh;
        }
        a { color: var(--accent); text-decoration: none; }
        .page { padding: 12px; display: grid; gap: 10px; }
        .topbar, .grid, .logs {
            display: grid;
            gap: 10px;
        }
        .topbar {
            grid-template-columns: 1.35fr 0.95fr;
        }
        .grid {
            grid-template-columns: repeat(3, minmax(0, 1fr));
        }
        .logs {
            grid-template-columns: 1.25fr 0.75fr;
        }
        .feeds {
            display: grid;
            gap: 12px;
            grid-template-columns: repeat(2, minmax(0, 1fr));
        }
        .card {
            background: linear-gradient(180deg, rgba(255,255,255,0.03), rgba(255,255,255,0.01));
            border: 1px solid var(--line);
            border-radius: 16px;
            overflow: hidden;
            min-width: 0;
        }
        .card header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 12px;
            border-bottom: 1px solid var(--line);
            background: rgba(255,255,255,0.02);
        }
        .card header h2 {
            margin: 0;
            font-size: 12px;
            letter-spacing: 0.04em;
            text-transform: uppercase;
            color: var(--muted);
        }
        .card .body {
            padding: 12px;
        }
        .hero {
            display: flex;
            justify-content: space-between;
            align-items: flex-start;
            gap: 10px;
        }
        .hero h1 {
            margin: 0 0 8px 0;
            font-size: 24px;
            line-height: 1;
        }
        .hero p {
            margin: 0;
            color: var(--muted);
            line-height: 1.35;
            font-size: 13px;
        }
        .badge {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            padding: 7px 10px;
            border-radius: 999px;
            background: rgba(63,176,255,0.12);
            border: 1px solid rgba(63,176,255,0.25);
            font-size: 12px;
            white-space: nowrap;
        }
        .dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: var(--warn);
            box-shadow: 0 0 12px rgba(245,158,11,0.5);
        }
        .dot.ok { background: var(--ok); box-shadow: 0 0 12px rgba(34,197,94,0.5); }
        .dot.bad { background: var(--bad); box-shadow: 0 0 12px rgba(239,68,68,0.5); }
        .kv {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 8px;
        }
        .kv .item, .full-item {
            background: rgba(255,255,255,0.02);
            border: 1px solid var(--line);
            border-radius: 12px;
            padding: 10px;
        }
        .item .label, .full-item .label {
            font-size: 11px;
            color: var(--muted);
            margin-bottom: 5px;
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }
        .item .value, .full-item .value {
            font-size: 15px;
            font-weight: 700;
            word-break: break-word;
            line-height: 1.15;
        }
        .meta {
            display: flex;
            flex-wrap: wrap;
            gap: 6px;
            margin-top: 10px;
        }
        .pill {
            padding: 5px 9px;
            border-radius: 999px;
            background: rgba(255,255,255,0.04);
            border: 1px solid var(--line);
            color: var(--muted);
            font-size: 11px;
        }
        .actions {
            display: flex;
            flex-wrap: wrap;
            gap: 8px;
            min-width: 250px;
            justify-content: flex-end;
        }
        button, .link-btn {
            appearance: none;
            border: none;
            border-radius: 12px;
            padding: 10px 12px;
            font-weight: 700;
            cursor: pointer;
            color: white;
            font-size: 13px;
        }
        .start { background: linear-gradient(135deg, #16a34a, #22c55e); }
        .stop { background: linear-gradient(135deg, #b91c1c, #ef4444); }
        .close { background: linear-gradient(135deg, #6d28d9, #8b5cf6); }
        .soft { background: linear-gradient(135deg, #0369a1, #3fb0ff); }
        .link-btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            text-decoration: none;
        }
        .health-list, .event-list {
            display: grid;
            gap: 7px;
            max-height: 24vh;
            overflow: auto;
            padding-right: 2px;
        }
        .health-row, .event-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 10px;
            padding: 9px 10px;
            border-radius: 12px;
            background: rgba(255,255,255,0.02);
            border: 1px solid var(--line);
        }
        .health-row .name, .event-row .name {
            color: var(--muted);
            font-size: 12px;
        }
        .health-row .state, .event-row .state {
            font-weight: 700;
            font-size: 12px;
        }
        .state.ok { color: var(--ok); }
        .state.bad { color: var(--bad); }
        .state.warn { color: var(--warn); }
        .toolbar {
            display: flex;
            gap: 8px;
            align-items: center;
            flex-wrap: wrap;
            margin-bottom: 10px;
        }
        select, input {
            background: var(--card2);
            color: var(--text);
            border: 1px solid var(--line);
            border-radius: 10px;
            padding: 8px 10px;
            font-size: 12px;
        }
        pre {
            margin: 0;
            white-space: pre-wrap;
            word-break: break-word;
            font-family: var(--mono);
            font-size: 11px;
            line-height: 1.45;
            background: #08101c;
            border: 1px solid var(--line);
            border-radius: 12px;
            padding: 12px;
            min-height: 240px;
            max-height: 34vh;
            overflow: auto;
        }
        .mini {
            font-family: var(--mono);
            font-size: 11px;
            color: var(--muted);
        }
        .feed-shell {
            width: 100%;
            aspect-ratio: 16 / 8.5;
            object-fit: cover;
            display: block;
            border-radius: 12px;
            border: 1px solid var(--line);
            background: #08101c;
        }
        @media (min-width: 1201px) {
            body { overflow: hidden; }
            .page {
                min-height: 100vh;
                height: 100vh;
                grid-template-rows: auto auto auto minmax(0, 1fr);
            }
            .logs {
                min-height: 0;
            }
            .logs .card,
            .logs .body {
                min-height: 0;
            }
        }
        @media (max-width: 1200px) {
            .grid, .logs, .topbar, .feeds { grid-template-columns: 1fr; }
            .actions { justify-content: flex-start; min-width: 0; }
            .health-list, .event-list { max-height: none; }
            pre { max-height: none; min-height: 260px; }
            body { overflow: auto; }
        }
    </style>
</head>
<body>
    <div class="page">
        <section class="topbar">
            <div class="card">
                <header>
                    <h2>Web Controller</h2>
                    <div class="badge"><span id="readyDot" class="dot"></span><span id="readyText">Bekleniyor</span></div>
                </header>
                <div class="body hero">
                    <div>
                        <h1>CELEBILER USV</h1>
                        <p>Görev durumu, current position, parkur, güvenlik ve loglar tek ekranda. Klasik dashboard ayrı tutuldu.</p>
                        <div class="meta">
                            <span class="pill">Mod: <b id="usvMode">--</b></span>
                            <span class="pill">Araç Modu: <b id="vehicleMode">--</b></span>
                            <span class="pill">Mission File: <b id="missionFile">--</b></span>
                        </div>
                    </div>
                    <div class="actions">
                        <button id="startBtn" class="start">Görevi Başlat</button>
                        <button id="stopBtn" class="stop">Acil Durdur</button>
                        <button id="shutdownBtn" class="close">🔌 Sistemi Kapat</button>
                        <a class="link-btn soft" href="/dashboard" target="_blank" rel="noreferrer">Klasik Dashboard</a>
                        <a class="link-btn soft" href="http://127.0.0.1:5000" target="_blank" rel="noreferrer">Kamera</a>
                    </div>
                </div>
            </div>
            <div class="card">
                <header><h2>Görev Özeti</h2><span class="mini" id="elapsed">--</span></header>
                <div class="body">
                    <div class="kv">
                        <div class="item"><div class="label">Parkur</div><div class="value" id="activeParkur">IDLE</div></div>
                        <div class="item"><div class="label">Görev State</div><div class="value" id="missionState">0</div></div>
                        <div class="item"><div class="label">Waypoint</div><div class="value" id="wpInfo">-- / --</div></div>
                        <div class="item"><div class="label">Target</div><div class="value" id="missionTarget">--</div></div>
                    </div>
                    <div class="meta" style="margin-top:14px">
                        <span class="pill">Gate: <b id="gateCount">0</b></span>
                        <span class="pill">Timeout: <b id="timeoutCount">0</b></span>
                        <span class="pill">Failsafe: <b id="failsafeState">normal</b></span>
                        <span class="pill">E-Stop: <b id="estopState">false</b></span>
                    </div>
                </div>
            </div>
        </section>

        <section class="grid">
            <div class="card">
                <header><h2>Navigasyon</h2><span class="mini" id="positionAge">--</span></header>
                <div class="body">
                    <div class="kv">
                        <div class="item"><div class="label">Latitude</div><div class="value" id="lat">--</div></div>
                        <div class="item"><div class="label">Longitude</div><div class="value" id="lon">--</div></div>
                        <div class="item"><div class="label">Heading</div><div class="value" id="heading">--</div></div>
                        <div class="item"><div class="label">Speed</div><div class="value" id="speed">--</div></div>
                        <div class="item"><div class="label">Battery</div><div class="value" id="battery">--</div></div>
                        <div class="item"><div class="label">Link Age</div><div class="value" id="linkAge">--</div></div>
                        <div class="item"><div class="label">Lidar L / C / R (m)</div><div class="value" id="lidarLcr">--</div></div>
                    </div>
                </div>
            </div>

            <div class="card">
                <header><h2>Manuel ve Güvenlik</h2><span class="mini" id="commandLock">--</span></header>
                <div class="body">
                    <div class="kv">
                        <div class="item"><div class="label">Manual Lock</div><div class="value" id="manualLock">--</div></div>
                        <div class="item"><div class="label">RC1 / RC3</div><div class="value" id="rcSummary">--</div></div>
                        <div class="item"><div class="label">Out1 / Out3</div><div class="value" id="outSummary">--</div></div>
                    </div>
                    <div class="meta" style="margin-top:14px">
                        <span class="pill">Camera Ready: <b id="cameraReady">--</b></span>
                        <span class="pill">Lidar Ready: <b id="lidarReady">--</b></span>
                        <span class="pill">Health: <b id="healthReady">--</b></span>
                        <span class="pill">FC Start: <b id="fcStartReady">--</b></span>
                    </div>
                </div>
            </div>

            <div class="card">
                <header><h2>Sağlık Bayrakları</h2><span class="mini">readiness</span></header>
                <div class="body health-list" id="healthList"></div>
            </div>
        </section>

        <section class="feeds">
            <div class="card">
                <header><h2>Kamera Önizleme</h2><span class="mini">processed stream</span></header>
                <div class="body">
                    <img id="controllerCameraFeed" class="feed-shell" alt="Camera Preview" />
                </div>
            </div>
            <div class="card">
                <header><h2>Unified Spatial Map</h2><span class="mini">ENU overlay</span></header>
                <div class="body">
                    <canvas id="controllerUnifiedMapCanvas" class="feed-shell" aria-label="Unified Spatial Map"></canvas>
                </div>
            </div>
        </section>

        <section class="logs">
            <div class="card">
                <header><h2>Canlı Loglar</h2><span class="mini" id="logMeta">--</span></header>
                <div class="body">
                    <div class="toolbar">
                        <select id="logSelect"></select>
                        <input id="logLines" type="number" min="20" max="800" step="10" value="120">
                        <button id="refreshLogBtn" class="soft">Log Yenile</button>
                    </div>
                    <pre id="logBody">Log yükleniyor...</pre>
                </div>
            </div>

            <div class="card">
                <header><h2>Event Akışı</h2><span class="mini" id="eventMeta">0 event</span></header>
                <div class="body event-list" id="eventList"></div>
            </div>
        </section>
    </div>

    <script>
        const stateMap = {0: 'BEKLEME', 1: 'NAV', 2: 'ENGAGE', 4: 'TAMAMLANDI', 5: 'HOLD'};
        let latestEventId = 0;
        let logFilesLoaded = false;
        let startPending = false;
        let lastStartRejectAlert2 = '';

        function showStartRejectAlert2(reason) {
            const text = String(reason || '').trim();
            if (!text.startsWith('start_rejected:') || text === lastStartRejectAlert2) {
                return;
            }
            lastStartRejectAlert2 = text;
            const hints = {
                accel_inconsistent_recent: 'IMU/ivme sensörü henüz hazır değil. 10-15 sn bekleyip tekrar deneyin.',
                gyro_inconsistent_recent: 'Jiroskop henüz hazır değil. 10-15 sn bekleyip tekrar deneyin.',
                bad_position_recent: 'FC konum çözümü henüz arm için hazır değil. Biraz bekleyip tekrar deneyin.',
                ekf_variance_recent: 'EKF varyansı yüksek. FC oturana kadar bekleyin.',
            };
            const code = text.split(':').pop() || '';
            alert('Görev başlatılamadı: ' + (hints[code] || text));
        }

        function fmtNum(v, d=3) {
            if (v === null || v === undefined || Number.isNaN(Number(v))) return '--';
            return Number(v).toFixed(d);
        }

        /** Lidar sector meters: avoid fake "0" from rounding; show open range as "off". */
        function fmtLidarM(v) {
            if (v === null || v === undefined || Number.isNaN(Number(v))) return '--';
            const x = Number(v);
            if (x >= 90) return 'off';
            if (x < 0.2) return '<0.20';
            return x.toFixed(2);
        }

        function text(id, value) {
            document.getElementById(id).textContent = value;
        }

        function renderHealth(flags) {
            const root = document.getElementById('healthList');
            root.innerHTML = '';
            const entries = Object.entries(flags || {});
            if (!entries.length) {
                root.innerHTML = '<div class="event-row"><span class="name">Flag bulunamadı</span><span class="state warn">--</span></div>';
                return;
            }
            for (const [name, value] of entries) {
                const row = document.createElement('div');
                row.className = 'health-row';
                row.innerHTML = `<span class="name">${name}</span><span class="state ${value ? 'ok' : 'bad'}">${value ? 'OK' : 'FAIL'}</span>`;
                root.appendChild(row);
            }
        }

        function prependEvent(kind, payload) {
            const root = document.getElementById('eventList');
            const row = document.createElement('div');
            row.className = 'event-row';
            row.innerHTML = `<span class="name">${kind}</span><span class="state">${payload}</span>`;
            root.prepend(row);
            while (root.children.length > 18) {
                root.removeChild(root.lastChild);
            }
            text('eventMeta', `${root.children.length} event`);
        }

        const LOG_DEFAULT_CANDIDATES = [
            'usv_main.debug.log',
            'usv_main.jsonl',
            'telemetry.debug.log',
            'telemetry.jsonl',
        ];

        function pickDefaultLogName(files) {
            const list = Array.isArray(files) ? files : [];
            for (const preferred of LOG_DEFAULT_CANDIDATES) {
                const hit = list.find((item) => item.name === preferred && item.exists && Number(item.size || 0) > 0);
                if (hit) return hit.name;
            }
            const nonempty = list
                .filter((item) => item.exists && Number(item.size || 0) > 0)
                .sort((a, b) => Number(b.size || 0) - Number(a.size || 0));
            if (nonempty.length) return nonempty[0].name;
            return list.length ? list[0].name : '';
        }

        async function loadLogFiles() {
            try {
                const res = await fetch('/api/log_files');
                if (!res.ok) throw new Error(`HTTP ${res.status}`);
                const data = await res.json();
                const select = document.getElementById('logSelect');
                const current = select.value;
                select.innerHTML = '';
                for (const item of data.files || []) {
                    const opt = document.createElement('option');
                    opt.value = item.name;
                    const sizeKb = Number(item.size || 0) > 0 ? ` (${Math.max(1, Math.round(item.size / 1024))} KB)` : ' (boş)';
                    opt.textContent = `${item.name}${sizeKb}`;
                    select.appendChild(opt);
                }
                if (current && [...select.options].some((opt) => opt.value === current)) {
                    select.value = current;
                } else {
                    const picked = pickDefaultLogName(data.files || []);
                    if (picked) select.value = picked;
                }
                logFilesLoaded = true;
            } catch (err) {
                logFilesLoaded = false;
                text('logMeta', 'Log listesi alınamadı (sim stack kapalı olabilir)');
                const body = document.getElementById('logBody');
                if (body && !body.textContent) {
                    body.textContent = '[sim stack kapalı — dosyalar diskte kalır: logs/system/*.debug.log ve *.jsonl]';
                }
            }
        }

        async function refreshLogs() {
            try {
                if (!logFilesLoaded) await loadLogFiles();
                const name = document.getElementById('logSelect').value;
                const lines = document.getElementById('logLines').value || '120';
                if (!name) return;
                const res = await fetch(`/api/log_tail?name=${encodeURIComponent(name)}&lines=${encodeURIComponent(lines)}`);
                if (!res.ok) throw new Error(`HTTP ${res.status}`);
                const data = await res.json();
                text('logMeta', `${data.name || name} | ${data.exists ? 'OK' : 'missing'} | ${data.lines || 0} satır`);
                document.getElementById('logBody').textContent = data.content || '[boş]';
            } catch (err) {
                text('logMeta', 'Log okunamadı (sim stack kapalı olabilir)');
            }
        }

        function bindCameraFeed(el) {
            if (!el) return;
            const host = window.location.hostname || '127.0.0.1';
            const candidates = [
                '/api/camera_stream',
                `http://${host}:5000/`
            ];
            if (!el.dataset.camHooked) {
                el.dataset.camHooked = '1';
                el.dataset.camIndex = el.dataset.camIndex || '0';
                el.onerror = function() {
                    const idx = Number(el.dataset.camIndex || '0');
                    el.dataset.camIndex = String((idx + 1) % candidates.length);
                    el.dataset.camBound = '0';
                };
            }
            const lastAttempt = Number(el.dataset.camAttemptTs || '0');
            const currentSrc = el.getAttribute('src') || '';
            const shouldRetry = !currentSrc || el.naturalWidth === 0 || el.dataset.camBound !== '1';
            if (!shouldRetry && (Date.now() - lastAttempt) < 5000) {
                return;
            }
            const idx = Number(el.dataset.camIndex || '0') % candidates.length;
            el.src = `${candidates[idx]}?t=${Date.now()}`;
            el.dataset.camAttemptTs = String(Date.now());
            el.dataset.camBound = '1';
        }

        let spatialMapState = null;
        let spatialMapFetchBusy = false;

        async function refreshSpatialMapState() {
            if (spatialMapFetchBusy) return;
            spatialMapFetchBusy = true;
            try {
                const r = await fetch('/api/spatial_map');
                if (!r.ok) return;
                spatialMapState = await r.json();
            } catch (_) {
            } finally {
                spatialMapFetchBusy = false;
            }
        }

        {{ unified_spatial_map_draw_js | safe }}


        function refreshVisualFeeds() {
            const cameraIds = ['controllerCameraFeed', 'cam_img'];
            for (const id of cameraIds) {
                const el = document.getElementById(id);
                bindCameraFeed(el);
            }
            const mapIds = ['controllerUnifiedMapCanvas', 'unified_map_canvas'];
            for (const id of mapIds) {
                const el = document.getElementById(id);
                drawUnifiedSpatialMap(el, spatialMapState);
            }
        }

        async function refreshData() {
            const res = await fetch('/api/data');
            const data = await res.json();
            text('usvMode', data.usv_mode || '--');
            const canonicalMode = data.mode_state_mode || '--';
            const canonicalSrc = data.mode_state_source || '--';
            text('vehicleMode', `${data.Mode || '--'} | ${canonicalMode} (${canonicalSrc})`);
            text('missionFile', data.mission_file || '--');
            text('activeParkur', data.active_parkur || 'IDLE');
            text('missionState', `${data.mission_state} / ${stateMap[data.mission_state] || 'UNKNOWN'}`);
            text('wpInfo', data.mission_wp_info || '-- / --');
            text('missionTarget', data.mission_target || '--');
            text('gateCount', String(data.gate_count ?? 0));
            text('timeoutCount', String(data.timeout_count ?? 0));
            text('failsafeState', data.failsafe_state || '--');
            text('estopState', String(Boolean(data.estop_state)));
            text('lat', fmtNum(data.Lat, 7));
            text('lon', fmtNum(data.Lon, 7));
            text('heading', `${fmtNum(data.Heading, 2)} deg`);
            text('speed', `${fmtNum(data.Speed, 2)} m/s`);
            text('battery', `${fmtNum(data.Battery, 2)} V`);
            text('linkAge', `${fmtNum(data.link_heartbeat_age_s, 2)} s`);
            text('positionAge', `state age ${fmtNum(data.state_age_s, 2)} s`);
            text('lidarLcr', `${fmtLidarM(data.lidar_left_m)} / ${fmtLidarM(data.lidar_center_m)} / ${fmtLidarM(data.lidar_right_m)}`);
            text('manualLock', data.manual_lock_reason || '--');
            text('commandLock', `command_lock=${Boolean(data.command_lock)}`);
            text('rcSummary', `${data.RC1 ?? '--'} / ${data.RC3 ?? '--'}`);
            text('outSummary', `${data.CMD_Port ?? data.Out1 ?? '--'} / ${data.CMD_Stbd ?? data.Out3 ?? '--'}`);
            text('cameraReady', String(Boolean(data.camera_ready)));
            text('lidarReady', String(Boolean(data.lidar_ready)));
            text('healthReady', String(Boolean(data.ready_state)));
            text('fcStartReady', `${Boolean(data.fc_start_ready)} ${data.fc_start_block_reason || ''}`.trim());

            const elapsed = Number(data.mission_active) && data.report_view ? data.report_view.mode_state.active_parkur : 'IDLE';
            text('elapsed', `parkur=${elapsed}`);

            renderHealth((data.health_check || {}).flags || {});

            const ready = Boolean(data.ready_state);
            const active = Boolean(data.mission_active);
            const dot = document.getElementById('readyDot');
            dot.className = 'dot ' + (active ? 'ok' : (ready ? 'ok' : 'bad'));
            text('readyText', active ? 'Görev Aktif' : (ready ? 'Hazır' : 'Hazır Değil'));
            if (active || !Boolean(data.start_requested)) {
                startPending = false;
            }
            if (data.start_phase === 'rejected' && !active) {
                startPending = false;
                const mer = String(data.mission_end_reason || 'none');
                if (mer.startsWith('start_rejected:')) {
                    showStartRejectAlert2(mer);
                }
            }
            document.getElementById('startBtn').disabled =
                active ||
                Boolean(data.start_requested) ||
                startPending ||
                !ready ||
                !Boolean(data.camera_ready) ||
                !Boolean(data.lidar_ready) ||
                !Boolean(data.fc_start_ready);
            if (!startPending && !Boolean(data.start_requested) && !active) {
                document.getElementById('startBtn').textContent = 'Görevi Başlat';
            }
        }

        async function pollEvents() {
            try {
                const res = await fetch(`/api/events?since_id=${latestEventId}`);
                const data = await res.json();
                latestEventId = data.latest_id || latestEventId;
                for (const ev of data.events || []) {
                    prependEvent(ev.kind || 'event', JSON.stringify(ev.payload || {}));
                    if (ev.kind === 'start_rejected') {
                        const payload = ev.payload || {};
                        showStartRejectAlert2(payload.mission_end_reason || payload.reason || '');
                    }
                }
            } catch (err) {
                prependEvent('events_error', String(err));
            } finally {
                setTimeout(pollEvents, 1500);
            }
        }

        async function doPost(url) {
            if (url === '/api/start_mission') {
                if (startPending) return;
                startPending = true;
                const btn = document.getElementById('startBtn');
                btn.disabled = true;
                btn.textContent = 'Start Bekleniyor';
            }
            const res = await fetch(url, {method: 'POST'});
            const data = await res.json().catch(() => ({}));
            if (!res.ok) {
                if (url === '/api/start_mission') {
                    startPending = false;
                    const btn = document.getElementById('startBtn');
                    btn.textContent = 'Görevi Başlat';
                }
                prependEvent('request_error', `${url} -> ${JSON.stringify(data)}`);
                return;
            }
            if (url === '/api/emergency_stop') {
                startPending = false;
                const btn = document.getElementById('startBtn');
                btn.textContent = 'Görevi Başlat';
            }
            prependEvent('request_ok', `${url} -> ${JSON.stringify(data)}`);
            await refreshData();
            await refreshLogs();
        }

        document.getElementById('startBtn').addEventListener('click', () => doPost('/api/start_mission'));
        document.getElementById('stopBtn').addEventListener('click', () => doPost('/api/emergency_stop'));
        document.getElementById('shutdownBtn').addEventListener('click', () => {
            if (confirm('Sistemi kapatmak istediğinizden emin misiniz?')) {
                doPost('/api/shutdown');
            }
        });
        document.getElementById('refreshLogBtn').addEventListener('click', refreshLogs);
        document.getElementById('logSelect').addEventListener('change', refreshLogs);

        (async () => {
            await loadLogFiles();
            await refreshData();
            await refreshLogs();
            await refreshSpatialMapState();
            refreshVisualFeeds();
            setInterval(refreshData, 1500);
            setInterval(refreshLogs, 2500);
            setInterval(refreshSpatialMapState, 200);
            setInterval(refreshVisualFeeds, 200);
            pollEvents();
        })();
    </script>
</body>
</html>
"""


def _tail_text_file(path, lines):
    if not path or not os.path.exists(path):
        return False, ""
    max_lines = max(20, min(int(lines or 120), 800))
    recent = deque(maxlen=max_lines)
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            recent.append(line.rstrip("\n"))
    return True, "\n".join(recent)


def _svg_placeholder(title, subtitle):
    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="1280" height="720" viewBox="0 0 1280 720">
<rect width="1280" height="720" fill="#08101c"/>
<rect x="32" y="32" width="1216" height="656" rx="24" fill="#0f1726" stroke="#22324a" stroke-width="2"/>
<text x="640" y="320" text-anchor="middle" fill="#e5eefc" font-family="Arial, sans-serif" font-size="42" font-weight="700">{title}</text>
<text x="640" y="382" text-anchor="middle" fill="#8da2c0" font-family="Arial, sans-serif" font-size="24">{subtitle}</text>
</svg>"""
    return Response(svg, mimetype="image/svg+xml")


def _stream_mjpeg_proxy(source_url):
    upstream = urllib.request.urlopen(source_url, timeout=3.0)
    mimetype = upstream.info().get_content_type() or "multipart/x-mixed-replace; boundary=frame"

    def generate():
        try:
            while True:
                chunk = upstream.read(4096)
                if not chunk:
                    break
                yield chunk
        finally:
            try:
                upstream.close()
            except Exception:
                pass

    return Response(generate(), mimetype=mimetype)


def _stream_camera_from_file():
    """Serve camera stream from cam.py headless output file (no separate webserver)."""
    def generate():
        while True:
            try:
                if os.path.exists(CAMERA_LATEST_JPG):
                    with open(CAMERA_LATEST_JPG, 'rb') as f:
                        frame = f.read()
                    if frame:
                        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception:
                pass
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


def _stream_lidar_proxy():
    return _stream_mjpeg_proxy(LIDAR_STREAM_PROXY_SOURCE)


# --- FLASK ROUTES ---
@app.route('/')
@app.route('/controller')
def controller():
    return redirect('/dashboard', code=302)


@app.route('/dashboard')
def dashboard():
    return render_template_string(HTML_PAGE, usv_mode=USV_MODE, unified_spatial_map_draw_js=UNIFIED_SPATIAL_MAP_DRAW_JS)


@app.route('/api/camera_stream')
def camera_stream():
    # Race mode guard: image transmission prohibited
    if USV_MODE == USV_MODE_RACE:
        return jsonify({'error': 'Camera stream disabled in race mode'}), 403
    try:
        return _stream_camera_from_file()
    except Exception as exc:
        if st is not None:
            st._warn_throttled("camera_file", f"[WARN] [CAM] Dosya akisi acilamadi: {exc}")
        return _svg_placeholder("Kamera akisi yok", "cam.py headless calismiyor veya frame dosyasi yok")


@app.route('/api/lidar_stream')
def lidar_stream():
    # Race mode guard: image transmission prohibited
    if USV_MODE == USV_MODE_RACE:
        return jsonify({'error': 'Lidar stream disabled in race mode'}), 403
    try:
        return _stream_lidar_proxy()
    except Exception as exc:
        if st is not None:
            st._warn_throttled("lidar_proxy", f"[WARN] [LIDAR] Proxy acilamadi: {exc}")
        if os.path.exists(LIDAR_MAP_JPG):
            response = send_file(LIDAR_MAP_JPG, mimetype="image/jpeg", conditional=False)
            response.cache_control.no_store = True
            response.cache_control.max_age = 0
            return response
        return _svg_placeholder("Lidar akisi yok", "lidar_map.py veya ROS-GZ bridge kontrol edilmeli")


@app.route('/api/lidar_map.jpg')
def lidar_map():
    # Race mode guard: image transmission prohibited
    if USV_MODE == USV_MODE_RACE:
        return jsonify({'error': 'Lidar map disabled in race mode'}), 403
    if os.path.exists(LIDAR_MAP_JPG):
        response = send_file(LIDAR_MAP_JPG, mimetype="image/jpeg", conditional=False)
        response.cache_control.no_store = True
        response.cache_control.max_age = 0
        return response
    return _svg_placeholder("Lidar haritasi yok", "file3 local map snapshot henuz olusmadi")

def _read_mission_state():
    """usv_main tarafından yazılan state dosyasını oku."""
    global last_state_cache
    state = atomic_read_json(STATE_FILE, default=None)
    if isinstance(state, dict):
        last_state_cache = state
        return state
    return last_state_cache


def _read_motor_command():
    """usv_main motor komut dosyasını oku; parse hatasında son geçerli değere düş."""
    global last_motor_command_cache
    try:
        if os.path.exists(MOTOR_COMMAND_FILE):
            with open(MOTOR_COMMAND_FILE, "r", encoding="utf-8") as f:
                payload = json.load(f)
            if isinstance(payload, dict):
                port = payload.get("cmd_port_pwm", payload.get("left_pwm", 1500))
                stbd = payload.get("cmd_stbd_pwm", payload.get("right_pwm", 1500))
                last_motor_command_cache = {
                    "cmd_port_pwm": _pick_primary_fallback(port, last_motor_command_cache.get("cmd_port_pwm"), default=1500),
                    "cmd_stbd_pwm": _pick_primary_fallback(stbd, last_motor_command_cache.get("cmd_stbd_pwm"), default=1500),
                    "source": str(payload.get("source", "--") or "--"),
                    "mode_state": payload.get("mode_state", {}),
                    "ts_monotonic": payload.get("ts_monotonic"),
                }
                return last_motor_command_cache
    except Exception:
        pass
    return last_motor_command_cache



def _resolve_spatial_origin(state):
    lat, lon = resolve_spatial_origin_candidates(
        cached_origin=SPATIAL_ORIGIN,
        state=state if isinstance(state, dict) else {},
        telemetry_lat=telemetry_data.get("Lat"),
        telemetry_lon=telemetry_data.get("Lon"),
    )
    if lat is not None and lon is not None:
        SPATIAL_ORIGIN["lat"] = float(lat)
        SPATIAL_ORIGIN["lon"] = float(lon)
    return lat, lon


def _load_mission_waypoints():
    mission_file = os.environ.get("MISSION_FILE", MISSION_FILE_DEFAULT)
    if not os.path.exists(mission_file):
        return []
    try:
        with open(mission_file, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        if isinstance(payload, list):
            coords = validate_coordinate_mission(payload)
            nav_wps, engage_wp = split_nav_engage(coords)
            out = list(nav_wps)
            if engage_wp is not None:
                out.append(engage_wp)
            return out
        if isinstance(payload, dict) and MISSION_ALLOW_STRUCTURED_LEGACY:
            mission = adapt_mission_to_structured(payload, strict=True)
            out = list(mission.get("parkur1", [])) + list(mission.get("parkur2", [])) + list(mission.get("parkur3", []))
            return out
    except Exception:
        pass
    return []


def _build_spatial_payload(state):
    global SPATIAL_LAST_START_TIME, SPATIAL_LAST_TRAIL_TS
    start_time = state.get("start_time")
    active = bool(state.get("active", False))
    if not active:
        if SPATIAL_TRAIL:
            SPATIAL_TRAIL.clear()
        SPATIAL_LAST_START_TIME = None
        SPATIAL_LAST_TRAIL_TS = 0.0
    elif start_time != SPATIAL_LAST_START_TIME:
        SPATIAL_TRAIL.clear()
        SPATIAL_LAST_START_TIME = start_time
        SPATIAL_LAST_TRAIL_TS = 0.0

    origin_lat, origin_lon = _resolve_spatial_origin(state)
    if origin_lat is None or origin_lon is None:
        return {
            "frame": {"name": "enu_local", "origin_lat": None, "origin_lon": None, "x_axis": "east", "y_axis": "north"},
            "boat": None,
            "trail": {"points": [], "max_points": int(SPATIAL_TRAIL.maxlen or 0)},
            "waypoints": [],
            "course": {"static_features": []},
            "bounds": compute_spatial_bounds([]),
            "lidar": {
                "points": [],
                "frame": "enu_world",
                "point_count": 0,
                "ts_monotonic": state.get("ts_monotonic"),
                "age_s": None,
                "source": "state",
                "stale": True,
            },
        }

    boat = resolve_boat_spatial_enu(
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        telemetry_lat=telemetry_data.get("Lat"),
        telemetry_lon=telemetry_data.get("Lon"),
        telemetry_heading_deg=telemetry_data.get("Heading", 0.0),
    )
    if isinstance(boat, dict):
        boat = dict(boat)
        boat["ts_monotonic"] = state.get("ts_monotonic")
    else:
        boat = None

    now_mono = time.monotonic()
    if boat is not None:
        should_add = False
        if not SPATIAL_TRAIL:
            should_add = True
        else:
            px, py = SPATIAL_TRAIL[-1]
            step = math.hypot(float(boat["x_m"]) - float(px), float(boat["y_m"]) - float(py))
            if step >= float(SPATIAL_TRAIL_MIN_STEP_M):
                should_add = True
        if should_add or (now_mono - float(SPATIAL_LAST_TRAIL_TS)) >= 1.0:
            SPATIAL_TRAIL.append((float(boat["x_m"]), float(boat["y_m"])))
            SPATIAL_LAST_TRAIL_TS = now_mono

    wps = []
    all_wps = _load_mission_waypoints()
    active_idx = int(state.get("active_waypoint_index", -1) or -1)
    for idx, wp in enumerate(all_wps):
        try:
            lat_wp = float(wp[0])
            lon_wp = float(wp[1])
        except (TypeError, ValueError, IndexError):
            continue
        wx, wy = latlon_to_spatial_enu_m(lat_wp, lon_wp, origin_lat, origin_lon)
        status = "future"
        if active_idx >= 0 and idx < active_idx:
            status = "past"
        elif active_idx >= 0 and idx == active_idx:
            status = "active"
        wps.append(
            {
                "idx": int(idx),
                "x_m": round(float(wx), 3),
                "y_m": round(float(wy), 3),
                "status": status,
            }
        )

    lidar_frame = str(state.get("lidar_points_frame", "") or "")
    lidar_points_raw = state.get("lidar_points_world", [])
    lidar_source = "state:lidar_points_world"
    if not isinstance(lidar_points_raw, list) or not lidar_points_raw:
        lidar_points_raw = state.get("lidar_points_local", [])
        lidar_source = "state:lidar_points_local_legacy"
        lidar_frame = "boat_local"
    lidar_points = []
    if isinstance(lidar_points_raw, list):
        for item in lidar_points_raw[:8192]:
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                continue
            try:
                lx = float(item[0])
                ly = float(item[1])
            except (TypeError, ValueError):
                continue
            if lidar_frame == "boat_local" and boat is not None:
                east_m, north_m = lidar_local_to_world_enu_compass(
                    lx,
                    ly,
                    float(boat["x_m"]),
                    float(boat["y_m"]),
                    float(boat.get("heading_deg", 0.0) or 0.0),
                )
                lidar_points.append([round(east_m, 2), round(north_m, 2)])
            else:
                lidar_points.append([round(lx, 2), round(ly, 2)])
    if lidar_frame == "boat_local" and boat is not None:
        lidar_frame = "enu_world_from_legacy_local"
    elif lidar_frame != "enu_world":
        lidar_frame = "enu_world" if lidar_points else str(lidar_frame or "unknown")
    state_ts = float(state.get("ts_monotonic", 0.0) or 0.0)
    age_s = round(max(0.0, time.monotonic() - state_ts), 3) if state_ts > 0.0 else None
    lidar_stale = bool(
        age_s is None
        or age_s > 2.0
        or not bool(state.get("lidar_quality_ready", False))
        or not str(state.get("lidar_frame_status", "") or "").startswith("ok")
    )
    course_features = load_course_static_features()
    point_groups = [
        [(float(w["x_m"]), float(w["y_m"])) for w in wps],
        [(float(p[0]), float(p[1])) for p in lidar_points if isinstance(p, (list, tuple)) and len(p) >= 2],
        [(float(f["x_m"]), float(f["y_m"])) for f in course_features],
    ]
    if isinstance(boat, dict):
        point_groups.append([(float(boat["x_m"]), float(boat["y_m"]))])
    if SPATIAL_TRAIL:
        point_groups.append([(float(x), float(y)) for x, y in SPATIAL_TRAIL])
    bounds = compute_spatial_bounds(point_groups)

    return {
        "frame": {
            "name": "enu_local",
            "origin_lat": round(float(origin_lat), 7),
            "origin_lon": round(float(origin_lon), 7),
            "x_axis": "east",
            "y_axis": "north",
            "source": "sim_vehicle_position" if os.environ.get("USV_SIM") == "1" else "telemetry_latlon",
        },
        "boat": boat,
        "trail": {
            "points": [[round(float(x), 3), round(float(y), 3)] for x, y in SPATIAL_TRAIL],
            "max_points": int(SPATIAL_TRAIL.maxlen or 0),
        },
        "waypoints": wps,
        "course": {
            "static_features": course_features,
            "source": "sim/configs/spatial_course_layout.json" if os.environ.get("USV_SIM") == "1" else "",
        },
        "bounds": bounds,
        "lidar": {
            "points": lidar_points,
            "frame": lidar_frame,
            "point_count": int(len(lidar_points)),
            "ts_monotonic": state.get("ts_monotonic"),
            "age_s": age_s,
            "source": lidar_source,
            "stale": bool(lidar_stale),
            "quality_ready": bool(state.get("lidar_quality_ready", False)),
            "frame_status": str(state.get("lidar_frame_status", "") or ""),
        },
    }

@app.route('/api/data')
def get_data():
    out = dict(telemetry_data)
    out['usv_mode'] = USV_MODE
    out['mission_file'] = os.environ.get('MISSION_FILE', '--')
    state = _read_mission_state()
    mission_data['start_requested'] = _start_request_pending(state)
    motor_cmd = _read_motor_command()
    target_state = load_target_state(TARGET_STATE_FILE)
    _sync_events_from_state(state)
    out['mission_state'] = state.get('state', mission_data['state'])
    out['mission_active'] = state.get('active', mission_data['active'])
    out['mission_end_reason'] = state.get('mission_end_reason', 'none')
    out['start_requested'] = _start_request_pending(state)
    out['mission_target'] = state.get('target', mission_data['target'])
    out['mission_wp_info'] = state.get('wp_info', mission_data['wp_info'])
    out['guidance_source'] = state.get('guidance_source', '--')
    out['failsafe_state'] = state.get('failsafe_state', '--')
    out['estop_state'] = state.get('estop_state', False)
    out['estop_source'] = state.get('estop_source', '--')
    out['command_lock'] = state.get('command_lock', False)
    out['gate_count'] = state.get('gate_count', 0)
    out['health_check'] = state.get('health_check', {})
    out['ready_state'] = state.get('ready_state', False)
    out['ready_missing'] = state.get('ready_missing', [])
    out['guided_ready'] = bool(state.get('guided_ready', False))
    out['guided_ready_missing'] = state.get('guided_ready_missing', [])
    out['guided_position_source'] = state.get('guided_position_source', 'invalid')
    out['fc_nav_mode_ready'] = bool(state.get('fc_nav_mode_ready', False))
    out['fc_guided_block_reason'] = state.get('fc_guided_block_reason', 'none')
    out['fc_ekf_flags'] = int(state.get('fc_ekf_flags', 0) or 0)
    out['fc_ekf_variances'] = state.get('fc_ekf_variances', {})
    out['fc_ekf_report_seen'] = bool(state.get('fc_ekf_report_seen', False))
    out['guided_prereq'] = state.get('guided_prereq', {})
    out['fc_guided_ready'] = bool(state.get('fc_guided_ready', out['guided_ready']))
    out['fc_start_ready'] = bool(state.get('fc_start_ready', out['guided_ready']))
    out['fc_start_block_reason'] = state.get('fc_start_block_reason', 'unknown')
    out['fc_gps'] = state.get('fc_gps', {})
    out['fc_ekf'] = state.get('fc_ekf', {})
    out['last_mavlink_message_ages'] = state.get('last_mavlink_message_ages', {})
    out['fc_gps_ekf_ready'] = bool(state.get('fc_gps_ekf_ready', False))
    out['fc_position_valid'] = bool(state.get('fc_position_valid', False))
    out['start_phase'] = state.get('start_phase', 'idle')
    out['sim_actuation_fallback'] = state.get('sim_actuation_fallback', 'none')
    out['sim_actuation_fallback_reason'] = state.get('sim_actuation_fallback_reason', 'none')
    out['fc_gps_fix_type'] = state.get('fc_gps_fix_type', state.get('gps_fix_type', 0))
    out['gps_sats'] = state.get('gps_sats', state.get('gps_satellites_visible', 0))
    out['gpi_received'] = bool(state.get('gpi_received', False))
    out['sim_nav_valid'] = bool(state.get('sim_nav_valid', False))
    out['last_mode_command'] = state.get('last_mode_command', {})
    out['last_command_ack'] = state.get('last_command_ack', {})
    out['last_statustext'] = state.get('last_statustext', {})
    mode_state = state.get('mode_state', {})
    if not isinstance(mode_state, dict):
        mode_state = {}
    out['mode_state'] = mode_state
    out['mode_state_mode'] = mode_state.get('canonical_mode', '--')
    out['mode_state_source'] = mode_state.get('source', '--')
    out['mode_state_reason'] = mode_state.get('reason', '--')
    out['mode_state_armed'] = bool(mode_state.get('armed', False))
    cmd_port = _pick_primary_fallback(
        motor_cmd.get("cmd_port_pwm"),
        state.get("cmd_port_pwm"),
        default=out.get('Out1', 1500),
    )
    cmd_stbd = _pick_primary_fallback(
        motor_cmd.get("cmd_stbd_pwm"),
        state.get("cmd_stbd_pwm"),
        default=out.get('Out3', 1500),
    )
    out['Out1'] = cmd_port
    out['Out3'] = cmd_stbd
    out['CMD_Port'] = cmd_port
    out['CMD_Stbd'] = cmd_stbd
    out['CMD_Source'] = str(motor_cmd.get("source", state.get("motor_command_source", "--")) or "--")
    out['motor_command_source'] = out['CMD_Source']
    out['cmd_port_pwm'] = cmd_port
    out['cmd_stbd_pwm'] = cmd_stbd
    out['sim_rc_override_semantics'] = state.get(
        'sim_rc_override_semantics',
        motor_cmd.get('sim_rc_override_semantics', '--'),
    )
    out['rc_override_ch1'] = int(state.get(
        'rc_override_ch1',
        motor_cmd.get('rc_override_ch1', 1500),
    ) or 1500)
    out['rc_override_ch3'] = int(state.get(
        'rc_override_ch3',
        motor_cmd.get('rc_override_ch3', 1500),
    ) or 1500)
    out['desired_left_pwm'] = int(state.get(
        'desired_left_pwm',
        motor_cmd.get('desired_left_pwm', cmd_port),
    ) or cmd_port)
    out['desired_right_pwm'] = int(state.get(
        'desired_right_pwm',
        motor_cmd.get('desired_right_pwm', cmd_stbd),
    ) or cmd_stbd)
    out['heartbeat_age_s'] = state.get('heartbeat_age_s', 0.0)
    out['link_heartbeat_age_s'] = state.get(
        'link_heartbeat_age_s',
        round(st.link_heartbeat_age_s, 3) if st is not None else out['heartbeat_age_s'],
    )
    out['mission_planner_gcs_age_s'] = state.get('mission_planner_gcs_age_s')
    out['timeout_count'] = state.get('timeout_count', 0)
    state_errors = state.get('error_counters', {})
    telemetry_errors = dict(st.error_counters) if st is not None else {}
    if isinstance(state_errors, dict):
        out['error_counters'] = dict(state_errors)
    else:
        out['error_counters'] = {}
    for k, v in telemetry_errors.items():
        out['error_counters'][f"telemetry_{k}"] = v
    out['manual_lock_reason'] = out.get('Manual_Lock_Reason', state.get('manual_lock_reason', '--'))
    out['camera_ready'] = state.get('camera_ready', False)
    out['camera_status_age_s'] = state.get('camera_status_age_s')
    out['camera_status_mtime_age_s'] = state.get('camera_status_mtime_age_s')
    out['camera_status_stale'] = bool(state.get('camera_status_stale', False))
    out['lidar_ready'] = state.get('lidar_ready', False)
    out['nav_position_source'] = state.get('nav_position_source', '--')
    out['nav_heading_source'] = state.get('nav_heading_source', '--')
    out['nav_solution_source'] = state.get('nav_solution_source', '--')
    out['nav_fix_valid'] = bool(state.get('nav_fix_valid', False))
    out['nav_state_age_s'] = state.get('nav_state_age_s')
    out['nav_target_bearing_deg'] = state.get('nav_target_bearing_deg')
    out['nav_target_distance_m'] = state.get('nav_target_distance_m')
    out['nav_heading_error_deg'] = state.get('nav_heading_error_deg')
    out['nav_source_detail'] = state.get('nav_source_detail', '--')
    out['nav_leg_start_lat'] = state.get('nav_leg_start_lat')
    out['nav_leg_start_lon'] = state.get('nav_leg_start_lon')
    out['closest_waypoint_distance_m'] = state.get('closest_waypoint_distance_m')
    out['nav_arrival_phase'] = state.get('nav_arrival_phase', '--')
    out['nav_align_mode'] = state.get('nav_align_mode', '--')
    out['nav_align_phase'] = state.get('nav_align_phase', '--')
    out['nav_heading_error_deg'] = state.get('nav_heading_error_deg')
    out['surge_allowed'] = bool(state.get('surge_allowed', False))
    out['nav_strict_heading_first'] = bool(state.get('nav_strict_heading_first', True))
    out['advance_stable_elapsed_s'] = state.get('advance_stable_elapsed_s', 0.0)
    out['guided_vx_north_mps'] = state.get('guided_vx_north_mps', 0.0)
    out['guided_vy_east_mps'] = state.get('guided_vy_east_mps', 0.0)
    out['avoidance_active'] = bool(state.get('avoidance_active', False))
    out['avoidance_source'] = state.get('avoidance_source', '--')
    out['escape_side'] = state.get('escape_side', '--')
    out['lidar_sector_ages'] = state.get('lidar_sector_ages', {})
    out['lidar_left_m'] = state.get('lidar_left_m')
    out['lidar_center_m'] = state.get('lidar_center_m')
    out['lidar_right_m'] = state.get('lidar_right_m')
    out['min_obstacle_m'] = state.get('min_obstacle_m')
    out['lidar_local_obstacle_valid'] = bool(state.get('lidar_local_obstacle_valid', False))
    out['lidar_degraded_mode'] = str(state.get('lidar_degraded_mode', 'normal') or 'normal')
    out['lidar_degraded_age_s'] = float(state.get('lidar_degraded_age_s', 0.0) or 0.0)
    out['lidar_clock_source'] = state.get('lidar_clock_source', 'none')
    out['lidar_stamp_age_s'] = state.get('lidar_stamp_age_s')
    out['active_parkur'] = state.get('active_parkur', '--')
    out['active_waypoint_index'] = state.get('active_waypoint_index', -1)
    out['guidance_detail_source'] = state.get('guidance_detail_source', '--')
    out['guidance_mode'] = state.get('guidance_mode', '--')
    out['guidance_reason'] = state.get('guidance_reason', '--')
    out['avoidance_bias_deg'] = state.get('avoidance_bias_deg', 0.0)
    out['cross_track_error_m'] = state.get('cross_track_error_m', 0.0)
    out['nominal_heading_deg'] = state.get('nominal_heading_deg', 0.0)
    out['gate_assist_bias_deg'] = state.get('gate_assist_bias_deg', 0.0)
    out['progress_along_leg_m'] = state.get('progress_along_leg_m', 0.0)
    out['nav_target_distance_delta_m'] = state.get('nav_target_distance_delta_m', 0.0)
    out['nav_target_distance_increase_count'] = int(state.get('nav_target_distance_increase_count', 0) or 0)
    out['nav_heading_error_delta_deg'] = state.get('nav_heading_error_delta_deg', 0.0)
    out['nav_diagnostic_reason'] = state.get('nav_diagnostic_reason', 'nominal')
    out['waypoint_passed_gate'] = bool(state.get('waypoint_passed_gate', False))
    out['waypoint_accept_reason'] = state.get('waypoint_accept_reason', '--')
    out['motor_limit_reason'] = state.get('motor_limit_reason', '--')
    out['current_yaw_rate_dps'] = state.get('current_yaw_rate_dps', 0.0)
    out['obstacle_threat_active'] = bool(state.get('obstacle_threat_active', False))
    out['obstacle_threat_source'] = str(state.get('obstacle_threat_source', 'none') or 'none')
    out['both_reverse_count'] = int(state.get('both_reverse_count', 0) or 0)
    out['single_side_reverse_count'] = int(state.get('single_side_reverse_count', 0) or 0)
    out['reverse_while_heading_small_count'] = int(state.get('reverse_while_heading_small_count', 0) or 0)
    out['heading_damping_hold_active'] = bool(state.get('heading_damping_hold_active', False))
    out['heading_damping_hold_count'] = int(state.get('heading_damping_hold_count', 0) or 0)
    out['heading_control_diagnostic'] = state.get('heading_control_diagnostic', 'nominal')
    camera_pipeline = state.get('camera_pipeline', {})
    out['camera_pipeline'] = camera_pipeline if isinstance(camera_pipeline, dict) else {}
    out['target_color'] = target_state.get('target_color', state.get('target_color', '--'))
    out['mission_input_format'] = state.get('mission_input_format', MISSION_INPUT_FORMAT)
    out['mission_split_profile'] = state.get('mission_split_profile', {})
    out['mission_profile'] = state.get('mission_profile', {})
    out['mission_profile_valid'] = bool(state.get('mission_profile_valid', False))
    out['mission_profile_race_ready'] = bool(state.get('mission_profile_race_ready', False))
    out['mission_profile_error'] = str(state.get('mission_profile_error', '') or '')
    out['p2_min_gate_count'] = int(state.get('p2_min_gate_count', 2) or 2)
    out['p3_engagement_mode'] = str(state.get('p3_engagement_mode', 'vision_color_track') or 'vision_color_track')
    mission_lifecycle = state.get('mission_lifecycle', {})
    if not isinstance(mission_lifecycle, dict):
        mission_lifecycle = {}
    out['mission_lifecycle'] = mission_lifecycle
    out['mission_upload_source'] = state.get(
        'mission_upload_source',
        mission_lifecycle.get('upload_source', '--'),
    )
    out['mission_synced'] = bool(state.get(
        'mission_synced',
        mission_lifecycle.get('mission_synced', False),
    ))
    out['pixhawk_mission_count'] = int(state.get(
        'pixhawk_mission_count',
        mission_lifecycle.get('pixhawk_mission_count', 0),
    ) or 0)
    out['pixhawk_mission_synced_at'] = state.get(
        'pixhawk_mission_synced_at',
        mission_lifecycle.get('pixhawk_mission_synced_at'),
    )
    out['mission_validated_at_timestamp'] = state.get(
        'mission_validated_at_timestamp',
        mission_lifecycle.get('validated_at_timestamp'),
    )
    out['pixhawk_mission_sync_error'] = state.get(
        'pixhawk_mission_sync_error',
        mission_lifecycle.get('pixhawk_mission_sync_error', ''),
    )
    out['pixhawk_mirror_status'] = state.get(
        'pixhawk_mirror_status',
        mission_lifecycle.get('pixhawk_mirror_status', 'unknown'),
    )
    out['pixhawk_mirror_error'] = state.get(
        'pixhawk_mirror_error',
        mission_lifecycle.get('pixhawk_mirror_error', ''),
    )
    out['post_start_mission_change_rejected'] = bool(state.get(
        'post_start_mission_change_rejected',
        mission_lifecycle.get('post_start_mission_change_rejected', False),
    ))
    try:
        link_age = float(out.get('link_heartbeat_age_s', 999.0) or 999.0)
    except (TypeError, ValueError):
        link_age = 999.0
    mission_planner_last_seen = None
    if link_age < 999.0:
        mission_planner_last_seen = round(time.time() - max(0.0, link_age), 3)
    if out['pixhawk_mission_synced_at'] is not None:
        try:
            mission_planner_last_seen = max(float(mission_planner_last_seen or 0.0), float(out['pixhawk_mission_synced_at']))
        except (TypeError, ValueError):
            pass
    try:
        gcs_age = float(out['mission_planner_gcs_age_s'])
    except (TypeError, ValueError):
        gcs_age = None
    mission_planner_required = bool(USV_MODE == USV_MODE_RACE)
    gcs_connected = bool(gcs_age is not None and gcs_age < HEARTBEAT_WARN_S)
    out['mission_planner_required'] = mission_planner_required
    out['mission_planner_connected'] = bool(gcs_connected)
    if gcs_connected:
        mission_planner_status = 'connected'
    elif mission_planner_required:
        mission_planner_status = 'required_not_connected'
    else:
        mission_planner_status = 'optional_not_connected'
    out['mission_planner_status'] = mission_planner_status
    out['mission_planner_last_seen'] = mission_planner_last_seen
    out['mission_planner_link'] = {
        'role': 'primary_gcs',
        'sim_connection': 'UDP 14552',
        'real_connection': '433MHz MAVLink telemetry modem',
        'required': out['mission_planner_required'],
        'status': out['mission_planner_status'],
        'connected': out['mission_planner_connected'],
        'last_seen': out['mission_planner_last_seen'],
        'gcs_heartbeat_age_s': out['mission_planner_gcs_age_s'],
        'pixhawk_mission_count': out['pixhawk_mission_count'],
        'pixhawk_mission_synced_at': out['pixhawk_mission_synced_at'],
        'pixhawk_mission_sync_error': out['pixhawk_mission_sync_error'],
        'pixhawk_mirror_status': out['pixhawk_mirror_status'],
        'pixhawk_mirror_error': out['pixhawk_mirror_error'],
        'mission_upload_source': out['mission_upload_source'],
        'heartbeat_note': (
            'mission_planner_connected reflects autopilot MAVLink freshness (link_heartbeat_age_s). '
            'gcs_heartbeat_age_s is time since last HEARTBEAT from a GCS-like source (e.g. MAV_TYPE_GCS / MP comp 190); '
            'it does not prove the Mission Planner UDP socket is up.'
        ),
    }
    sensor_fusion = state.get('sensor_fusion', {})
    out['sensor_fusion'] = sensor_fusion if isinstance(sensor_fusion, dict) else {}
    dynamic_speed_profile = state.get('dynamic_speed_profile', {})
    out['dynamic_speed_profile'] = dynamic_speed_profile if isinstance(dynamic_speed_profile, dict) else {}
    wind_assist = state.get('wind_assist', {})
    out['wind_assist'] = wind_assist if isinstance(wind_assist, dict) else {}
    horizon_lock = state.get('horizon_lock', {})
    out['horizon_lock'] = horizon_lock if isinstance(horizon_lock, dict) else {}
    camera_adaptation = state.get('camera_adaptation', {})
    out['camera_adaptation'] = camera_adaptation if isinstance(camera_adaptation, dict) else {}
    autonomy_health = state.get('autonomy_health', {})
    out['autonomy_health'] = autonomy_health if isinstance(autonomy_health, dict) else {}
    virtual_anchor = state.get('virtual_anchor', {})
    out['virtual_anchor'] = virtual_anchor if isinstance(virtual_anchor, dict) else {}
    out['telemetry_profile'] = {
        'general_hz': GENERAL_TELEMETRY_HZ,
        'obstacle_hz': OBSTACLE_TELEMETRY_HZ,
        'lidar_processing_hz': LIDAR_PROCESSING_HZ,
        'critical_events': 'event',
    }
    out['report_groups'] = REPORT_TELEMETRY_GROUPS
    out['link_topology'] = LINK_TOPOLOGY
    out['comms_policy'] = COMMS_POLICY
    try:
        state_ts = float(state.get('ts_monotonic', 0.0) or 0.0)
    except (TypeError, ValueError):
        state_ts = 0.0
    out['state_age_s'] = round(max(0.0, time.monotonic() - state_ts), 3) if state_ts > 0.0 else 999.0
    fusion_summary = {
        'enabled': bool(out['sensor_fusion'].get('enabled', False)),
        'policy': out['sensor_fusion'].get('policy', '--'),
        'ghost_gate_count': int(out['sensor_fusion'].get('ghost_gate_count', 0) or 0),
        'ghost_target_count': int(out['sensor_fusion'].get('ghost_target_count', 0) or 0),
        'lidar_ready': bool(out['sensor_fusion'].get('lidar_ready', out['lidar_ready'])),
    }
    dyn_speed_summary = {
        'enabled': bool(out['dynamic_speed_profile'].get('enabled', False)),
        'mode': out['dynamic_speed_profile'].get('mode', '--'),
        'scope': out['dynamic_speed_profile'].get('scope', []),
        'active': bool(out['dynamic_speed_profile'].get('active', False)),
        'band': out['dynamic_speed_profile'].get('band', '--'),
        'factor': float(out['dynamic_speed_profile'].get('factor', 1.0) or 1.0),
        'heading_error_abs_deg': float(out['dynamic_speed_profile'].get('heading_error_abs_deg', 0.0) or 0.0),
        'base_speed_mps': float(out['dynamic_speed_profile'].get('base_speed_mps', 0.0) or 0.0),
        'output_speed_mps': float(out['dynamic_speed_profile'].get('output_speed_mps', 0.0) or 0.0),
    }
    wind_assist_summary = {
        'enabled': bool(out['wind_assist'].get('enabled', False)),
        'mode': out['wind_assist'].get('mode', '--'),
        'scope': out['wind_assist'].get('scope', []),
        'active': bool(out['wind_assist'].get('active', False)),
        'reason': out['wind_assist'].get('reason', '--'),
        'i_gain': float(out['wind_assist'].get('i_gain', 0.0) or 0.0),
        'bias_deg': float(out['wind_assist'].get('bias_deg', 0.0) or 0.0),
        'bias_max_deg': float(out['wind_assist'].get('bias_max_deg', 0.0) or 0.0),
        'heading_error_abs_deg': float(out['wind_assist'].get('heading_error_abs_deg', 0.0) or 0.0),
        'corrected_heading_error_deg': float(out['wind_assist'].get('corrected_heading_error_deg', 0.0) or 0.0),
    }
    horizon_lock_summary = {
        'enabled': bool(out['horizon_lock'].get('enabled', False)),
        'mode': out['horizon_lock'].get('mode', '--'),
        'scope': out['horizon_lock'].get('scope', []),
        'active': bool(out['horizon_lock'].get('active', False)),
        'reason': out['horizon_lock'].get('reason', '--'),
        'channel': out['horizon_lock'].get('channel', '--'),
        'roll_deg': float(out['horizon_lock'].get('roll_deg', 0.0) or 0.0),
        'pitch_deg': float(out['horizon_lock'].get('pitch_deg', 0.0) or 0.0),
        'raw_bearing_deg': float(out['horizon_lock'].get('raw_bearing_deg', 0.0) or 0.0),
        'correction_deg': float(out['horizon_lock'].get('correction_deg', 0.0) or 0.0),
        'corrected_bearing_deg': float(out['horizon_lock'].get('corrected_bearing_deg', 0.0) or 0.0),
    }
    camera_adaptation_summary = {
        'enabled': bool(out['camera_adaptation'].get('enabled', False)),
        'mode': out['camera_adaptation'].get('mode', '--'),
        'luma_mean': float(out['camera_adaptation'].get('luma_mean', 0.0) or 0.0),
        'exposure_gain': float(out['camera_adaptation'].get('exposure_gain', 1.0) or 1.0),
        'exposure_beta': float(out['camera_adaptation'].get('exposure_beta', 0.0) or 0.0),
        'hsv_s_shift': int(out['camera_adaptation'].get('hsv_s_shift', 0) or 0),
        'hsv_v_shift': int(out['camera_adaptation'].get('hsv_v_shift', 0) or 0),
        'hsv_profile': out['camera_adaptation'].get('hsv_profile', '--'),
    }
    autonomy_health_summary = {
        'enabled': bool(out['autonomy_health'].get('enabled', False)),
        'trust_score': float(out['autonomy_health'].get('trust_score', 0.0) or 0.0),
        'level': out['autonomy_health'].get('level', '--'),
        'color': out['autonomy_health'].get('color', '--'),
        'label': out['autonomy_health'].get('label', '--'),
        'advisory': out['autonomy_health'].get('advisory', '--'),
        'failsafe_state': out['autonomy_health'].get('failsafe_state', '--'),
        'gps_satellites_visible': int(out['autonomy_health'].get('gps_satellites_visible', 0) or 0),
        'gps_fix_type': int(out['autonomy_health'].get('gps_fix_type', 0) or 0),
        'camera_mode': out['autonomy_health'].get('camera_mode', '--'),
        'lidar_point_count': int(out['autonomy_health'].get('lidar_point_count', 0) or 0),
        'rc_link_active': bool(out['autonomy_health'].get('rc_link_active', False)),
        'component_scores': out['autonomy_health'].get('component_scores', {}),
        'weights': out['autonomy_health'].get('weights', {}),
    }
    virtual_anchor_summary = {
        'enabled': bool(out['virtual_anchor'].get('enabled', False)),
        'mode': out['virtual_anchor'].get('mode', '--'),
        'active': bool(out['virtual_anchor'].get('active', False)),
        'reason': out['virtual_anchor'].get('reason', '--'),
        'fence_radius_m': float(out['virtual_anchor'].get('fence_radius_m', 0.0) or 0.0),
        'drift_trigger_m': float(out['virtual_anchor'].get('drift_trigger_m', 0.0) or 0.0),
        'drift_from_center_m': float(out['virtual_anchor'].get('drift_from_center_m', 0.0) or 0.0),
        'inside_fence': bool(out['virtual_anchor'].get('inside_fence', True)),
        'anchor_set': bool(out['virtual_anchor'].get('anchor_set', False)),
        'pulse_speed_mps': float(out['virtual_anchor'].get('pulse_speed_mps', 0.0) or 0.0),
        'pulse_heading_error_deg': float(out['virtual_anchor'].get('pulse_heading_error_deg', 0.0) or 0.0),
        'pulse_count': int(out['virtual_anchor'].get('pulse_count', 0) or 0),
        'breach_count': int(out['virtual_anchor'].get('breach_count', 0) or 0),
    }

    # Report section 3.4 grouped projection for YKI consumers.
    out['report_view'] = {
        'mode_state': {
            'usv_mode': USV_MODE,
            'vehicle_mode': telemetry_data.get('Mode', '--'),
            'canonical_mode': out.get('mode_state_mode', '--'),
            'canonical_source': out.get('mode_state_source', '--'),
            'canonical_reason': out.get('mode_state_reason', '--'),
            'canonical_armed': out.get('mode_state_armed', False),
            'mission_state': out['mission_state'],
            'active_parkur': out['active_parkur'],
        },
        'mission_progress': {
            'target': out['mission_target'],
            'wp_info': out['mission_wp_info'],
            'mission_end_reason': out['mission_end_reason'],
            'mission_upload_source': out['mission_upload_source'],
            'mission_synced': out['mission_synced'],
            'pixhawk_mission_count': out['pixhawk_mission_count'],
            'pixhawk_mission_synced_at': out['pixhawk_mission_synced_at'],
            'pixhawk_mission_sync_error': out['pixhawk_mission_sync_error'],
            'pixhawk_mirror_status': out['pixhawk_mirror_status'],
            'pixhawk_mirror_error': out['pixhawk_mirror_error'],
            'start_phase': out['start_phase'],
            'mission_planner_gcs_age_s': out['mission_planner_gcs_age_s'],
            'mission_planner_status': out['mission_planner_status'],
            'post_start_mission_change_rejected': out['post_start_mission_change_rejected'],
            'active_waypoint_index': out['active_waypoint_index'],
            'target_color': out['target_color'],
            'mission_profile_valid': out['mission_profile_valid'],
            'mission_profile_race_ready': out['mission_profile_race_ready'],
            'p2_min_gate_count': out['p2_min_gate_count'],
            'p3_engagement_mode': out['p3_engagement_mode'],
            'gate_count': out['gate_count'],
        },
        'event_flags': {
            'gate_gecildi': bool(out['gate_count'] > 0),
            'failsafe_state': out['failsafe_state'],
            'estop_state': out['estop_state'],
            'timeout_count': out['timeout_count'],
        },
        'navigation_health': {
            'camera_ready': out['camera_ready'],
            'lidar_ready': out['lidar_ready'],
            'fc_gps_ekf_ready': out['fc_gps_ekf_ready'],
            'fc_position_valid': out['fc_position_valid'],
            'guidance_source': out['guidance_source'],
            'guidance_detail_source': out['guidance_detail_source'],
            'guidance_mode': out['guidance_mode'],
            'guidance_reason': out['guidance_reason'],
            'avoidance_bias_deg': out['avoidance_bias_deg'],
            'cross_track_error_m': out['cross_track_error_m'],
            'nominal_heading_deg': out['nominal_heading_deg'],
            'gate_assist_bias_deg': out['gate_assist_bias_deg'],
            'progress_along_leg_m': out['progress_along_leg_m'],
            'waypoint_passed_gate': out['waypoint_passed_gate'],
            'waypoint_accept_reason': out['waypoint_accept_reason'],
            'motor_limit_reason': out['motor_limit_reason'],
            'sim_rc_override_semantics': out['sim_rc_override_semantics'],
            'rc_override_ch1': out['rc_override_ch1'],
            'rc_override_ch3': out['rc_override_ch3'],
            'guided_position_source': out['guided_position_source'],
            'nav_position_source': out['nav_position_source'],
            'nav_heading_source': out['nav_heading_source'],
            'nav_solution_source': out['nav_solution_source'],
            'nav_fix_valid': out['nav_fix_valid'],
            'nav_state_age_s': out['nav_state_age_s'],
            'nav_target_bearing_deg': out['nav_target_bearing_deg'],
            'nav_target_distance_m': out['nav_target_distance_m'],
            'nav_heading_error_deg': out['nav_heading_error_deg'],
            'nav_source_detail': out['nav_source_detail'],
            'nav_leg_start_lat': out['nav_leg_start_lat'],
            'nav_leg_start_lon': out['nav_leg_start_lon'],
            'closest_waypoint_distance_m': out['closest_waypoint_distance_m'],
            'nav_arrival_phase': out['nav_arrival_phase'],
            'avoidance_active': out['avoidance_active'],
            'avoidance_source': out['avoidance_source'],
            'escape_side': out['escape_side'],
            'lidar_sector_ages': out['lidar_sector_ages'],
            'lidar_left_m': state.get('lidar_left_m'),
            'lidar_center_m': state.get('lidar_center_m'),
            'lidar_right_m': state.get('lidar_right_m'),
            'min_obstacle_m': state.get('min_obstacle_m'),
            'lidar_clock_source': out['lidar_clock_source'],
            'lidar_stamp_age_s': out['lidar_stamp_age_s'],
            'heartbeat_age_s': out['heartbeat_age_s'],
            'link_heartbeat_age_s': out['link_heartbeat_age_s'],
            'state_age_s': out['state_age_s'],
            'camera_pipeline': out['camera_pipeline'],
            'sensor_fusion': fusion_summary,
            'dynamic_speed_profile': dyn_speed_summary,
            'wind_assist': wind_assist_summary,
            'horizon_lock': horizon_lock_summary,
            'camera_adaptation': camera_adaptation_summary,
            'autonomy_health': autonomy_health_summary,
            'virtual_anchor': virtual_anchor_summary,
        },
        'link_health': {
            'rc_link_active': bool(900 <= telemetry_data.get('RC1', 0) <= 2100 or 900 <= telemetry_data.get('RC3', 0) <= 2100),
            'telemetry_heartbeat_age_s': out['link_heartbeat_age_s'],
            'onboard_heartbeat_age_s': out['heartbeat_age_s'],
            'mission_planner': out['mission_planner_link'],
        },
        'safety': {
            'command_lock': out['command_lock'],
            'estop_state': out['estop_state'],
            'estop_source': out['estop_source'],
            'manual_lock_reason': out['manual_lock_reason'],
            'motor_command_source': out.get('motor_command_source', '--'),
            'cmd_port_pwm': out.get('cmd_port_pwm'),
            'cmd_stbd_pwm': out.get('cmd_stbd_pwm'),
            'health_ready': out['ready_state'],
            'health_missing': out['ready_missing'],
        },
        'energy': {
            'battery_v': telemetry_data.get('Battery', 0),
        },
    }
    return jsonify(out)


@app.route('/api/spatial_map')
def spatial_map():
    if USV_MODE == USV_MODE_RACE:
        return jsonify({'error': 'Spatial map disabled in race mode'}), 403
    state = _read_mission_state()
    payload = _build_spatial_payload(state if isinstance(state, dict) else {})
    return jsonify(payload)


@app.route('/api/log_files')
def log_files():
    files = []
    for name, path in sorted(_merged_log_index().items()):
        exists = bool(path and os.path.exists(path))
        size = os.path.getsize(path) if exists else 0
        mtime = round(os.path.getmtime(path), 3) if exists else None
        files.append({
            'name': name,
            'exists': exists,
            'size': size,
            'mtime': mtime,
        })
    files.sort(key=lambda item: item['name'])
    return jsonify({'files': files})


@app.route('/api/log_tail')
def log_tail():
    name = request.args.get('name', 'telemetry.debug.log')
    lines = request.args.get('lines', 120)
    safe_name, path = _read_allowed_log(name)
    if not path:
        return jsonify({
            'name': safe_name or str(name),
            'exists': False,
            'lines': 0,
            'content': '',
            'error': 'Log dosyasi izinli degil',
        }), 400
    try:
        exists, content = _tail_text_file(path, lines)
        return jsonify({
            'name': safe_name,
            'exists': exists,
            'lines': max(20, min(int(lines or 120), 800)),
            'content': content,
        })
    except Exception as exc:
        # Silent fail for log tail endpoint
        return jsonify({
            'name': safe_name,
            'exists': bool(os.path.exists(path)),
            'lines': 0,
            'content': '',
            'error': str(exc),
        }), 500

@app.route('/api/mission', methods=['POST'])
def upload_mission():
    """Mission dosyasını yükle ve valide et.

    Operational POST body (flat ordered format):
    [
      [lat, lon],
      [lat, lon]
    ]
    """
    if USV_MODE == USV_MODE_RACE:
        return jsonify({'error': 'Race modunda mission yükleme API kapalıdır.'}), 403
    
    # Check if mission is already running (post-start lock)
    state = _read_mission_state()
    if state.get('active', False):
        return jsonify({'error': 'Görev aktifken yeni mission yüklenemez.'}), 409
    
    # İstek gövdesinden mission verisini oku
    try:
        mission_data_upload = request.get_json()
        if not mission_data_upload:
            return jsonify({'error': 'JSON body boş'}), 400
    except Exception as e:
        return jsonify({'error': f'JSON parse hatası: {str(e)}'}), 400
    
    try:
        target_color_from_legacy = ""
        if isinstance(mission_data_upload, list):
            flat_mission = validate_coordinate_mission(mission_data_upload)
        elif isinstance(mission_data_upload, dict) and MISSION_ALLOW_STRUCTURED_LEGACY:
            mission = adapt_mission_to_structured(mission_data_upload, strict=True)
            flat_mission = mission["parkur1"] + mission["parkur2"] + mission["parkur3"]
            target_color_from_legacy = str(mission.get("target_color") or "").strip().upper()
        elif isinstance(mission_data_upload, dict):
            return jsonify({'error': 'Structured mission payload disabled; flat ordered [lat, lon] list gönderin.'}), 400
        else:
            return jsonify({'error': f'Mission list veya object olmalı, gelen tip: {type(mission_data_upload).__name__}'}), 400

        p1_wps, p2_wps, p3_wps = split_mission_waypoints(flat_mission, validate_lengths=False)
        split_profile = get_mission_split_profile(len(flat_mission))
    except ValueError as exc:
        return jsonify({'error': str(exc)}), 400
    
    # Mission dosyasına canonical flat formatta yaz.
    mission_file = os.environ.get("MISSION_FILE", MISSION_FILE_DEFAULT)
    try:
        if not atomic_write_json(mission_file, flat_mission, indent=2):
            return jsonify({'error': 'Mission dosyası yazılamadı (atomic write failed)'}), 500

        if target_color_from_legacy:
            target_payload = {
                'target_color': validate_target_color(target_color_from_legacy),
                'target_color_changed_at': time.time(),
            }
            if not atomic_write_json(TARGET_STATE_FILE, target_payload, indent=2):
                return jsonify({'error': 'Target state yazılamadı (atomic write failed)'}), 500

        print(f"✓ [API] Mission dosyası başarıyla yüklendi: {mission_file}")
        return jsonify({
            'ok': True,
            'message': 'Mission başarıyla yüklendi',
            'mission': {
                'input_format': MISSION_INPUT_FORMAT,
                'total_count': len(flat_mission),
                'p1_count': len(p1_wps),
                'p2_count': len(p2_wps),
                'p3_count': len(p3_wps),
                'split_profile': split_profile,
                'file': mission_file
            }
        }), 200
    except Exception as e:
        print(f"✗ [API] Mission yazma hatası: {e}")
        return jsonify({'error': f'Mission dosyası yazılamadı: {str(e)}'}), 500


@app.route('/api/target_color', methods=['POST'])
def set_target_color():
    """Target color'u dinamik olarak ayarla.
    
    POST body:
    {
      "target_color": "RED|GREEN|BLACK|KIRMIZI_SANCAK|YESIL_SANCAK|SIYAH_HEDEF"
    }
    """
    if USV_MODE == USV_MODE_RACE:
        return jsonify({'error': 'Race modunda target_color API kapalıdır.'}), 403
    
    try:
        data = request.get_json()
        if not data:
            return jsonify({'error': 'JSON body boş'}), 400
    except Exception as e:
        return jsonify({'error': f'JSON parse hatası: {str(e)}'}), 400
    
    target_color_raw = data.get('target_color', '')
    if not target_color_raw:
        return jsonify({'error': 'target_color alanı zorunlu'}), 400

    state = _read_mission_state()
    if state.get('active', False):
        return jsonify({'error': 'Görev aktifken target_color güncellenemez.'}), 409

    try:
        target_color = validate_target_color(target_color_raw)
    except ValueError as exc:
        return jsonify({'error': str(exc)}), 400

    try:
        os.makedirs(os.path.dirname(TARGET_STATE_FILE), exist_ok=True)
        target_payload = {
            'target_color': target_color,
            'target_color_changed_at': time.time(),
        }
        if not atomic_write_json(TARGET_STATE_FILE, target_payload, indent=2):
            return jsonify({'error': 'Target color yazılamadı (atomic write failed)'}), 500

        print(f"✓ [API] Target color güncellendi: {target_color}")
        return jsonify({
            'ok': True,
            'message': f'Target color başarıyla {target_color} olarak ayarlandı',
            'target_color': target_color
        }), 200
    except Exception as e:
        print(f"✗ [API] Target color yazma hatası: {e}")
        return jsonify({'error': f'Target color yazılamadı: {str(e)}'}), 500


@app.route('/api/mission_status')
def mission_status():
    state = _read_mission_state()
    elapsed = 0
    if state.get('active') and state.get('start_time', 0) > 0:
        elapsed = time.time() - state['start_time']
    return jsonify({
        'active': state.get('active', False),
        'state': state.get('state', 0),
        'elapsed': elapsed
    })


@app.route('/api/events')
def api_events():
    since_id = request.args.get('since_id', default=0, type=int)
    timeout_s = request.args.get('timeout_s', default=8.0, type=float)
    timeout_s = min(max(timeout_s, 0.0), 20.0)
    deadline = time.monotonic() + timeout_s

    while True:
        state = _read_mission_state()
        _sync_events_from_state(state)
        with EVENT_LOCK:
            latest_id = EVENT_ID
            events = [ev for ev in EVENT_QUEUE if ev.get("id", 0) > since_id]
        if events or time.monotonic() >= deadline:
            return jsonify({
                'events': events,
                'latest_id': latest_id,
            })
        time.sleep(0.1)

@app.route('/api/start_mission', methods=['POST'])
def start_mission():
    """Görevi başlat (usv_main dosya IPC ile algılar)."""
    state = _read_mission_state()
    if USV_MODE == USV_MODE_RACE:
        return jsonify({'error': 'Race modunda gorev baslatma sadece RC CH5 ile yapilir (API politikasi kapali).'}), 403
    if state.get('active', False):
        mission_data['start_requested'] = False
        mission_data['start_requested_at'] = 0.0
        return jsonify({'error': 'Gorev zaten aktif.'}), 409
    if _start_request_pending(state):
        return jsonify({'error': 'Start isteği zaten beklemede.'}), 409
    if not state.get('ready_state', False):
        mission_data['start_requested'] = False
        mission_data['start_requested_at'] = 0.0
        return jsonify({'error': 'Sistem hazir degil. Kamera/lidar/telemetri hazir olmadan start kabul edilmez.'}), 409
    if not state.get('camera_ready', False) or not state.get('lidar_ready', False):
        mission_data['start_requested'] = False
        mission_data['start_requested_at'] = 0.0
        return jsonify({'error': 'Kamera veya lidar hazir degil. Web controller readiness yesil olmadan start kabul edilmez.'}), 409
    sim_nav = {}
    sim_nav_ok = False
    if os.environ.get("USV_SIM") == "1":
        sim_nav = load_sim_nav_state(control_dir=CONTROL_DIR)
        sim_nav_ok = bool(sim_nav.get("valid"))
    fc_start_ready = bool(state.get('fc_start_ready', state.get('guided_ready', False)))
    if state.get('guided_ready') is False or not fc_start_ready:
        mission_data['start_requested'] = False
        mission_data['start_requested_at'] = 0.0
        missing = state.get('guided_ready_missing', [])
        if not isinstance(missing, list):
            missing = []
        guided_prereq = state.get('guided_prereq', {})
        if not isinstance(guided_prereq, dict):
            guided_prereq = {}
        fc_start_block_reason = str(state.get('fc_start_block_reason', guided_prereq.get('fc_start_block_reason', 'unknown')))
        # Simülasyon modunda daha açıklayıcı mesaj
        if os.environ.get("USV_SIM") == "1":
            block_reason = str(state.get('fc_guided_block_reason', guided_prereq.get('fc_guided_block_reason', 'unknown')))
            if fc_start_block_reason in ('gyro_inconsistent_recent', 'accel_inconsistent_recent'):
                error_msg = 'SITL IMU stabilize oluyor, lütfen bekleyin.'
            elif fc_start_block_reason in ('ekf_variance_recent', 'bad_position_recent') or block_reason in ('ekf_variance_recent', 'ekf_variance_high', 'ekf_hard_fault'):
                error_msg = 'SITL EKF stabilize oluyor, lütfen bekleyin.'
            else:
                error_msg = 'SITL GPS/EKF hazırlanıyor, lütfen 10-15 sn bekleyin.'
        else:
            error_msg = 'ArduPilot start icin stabilize degil.'
        return jsonify({
            'error': error_msg,
            'missing': missing,
            'guided_ready_missing': missing,
            'guided_prereq': guided_prereq,
            'fc_start_ready': bool(fc_start_ready),
            'fc_start_block_reason': fc_start_block_reason,
            'last_statustext': state.get('last_statustext', {}),
            'guided_position_source': state.get('guided_position_source', guided_prereq.get('position_source', 'invalid')),
            'fc_gps_ekf_ready': bool(state.get('fc_gps_ekf_ready', guided_prereq.get('fc_gps_ekf_ready', False))),
            'fc_position_valid': bool(state.get('fc_position_valid', guided_prereq.get('fc_position_valid', False))),
            'fc_nav_mode_ready': bool(state.get('fc_nav_mode_ready', guided_prereq.get('fc_nav_mode_ready', False))),
            'fc_guided_block_reason': state.get('fc_guided_block_reason', guided_prereq.get('fc_guided_block_reason', 'unknown')),
            'fc_ekf_flags': int(state.get('fc_ekf_flags', guided_prereq.get('fc_ekf_flags', 0)) or 0),
            'fc_ekf_variances': state.get('fc_ekf_variances', guided_prereq.get('fc_ekf_variances', {})),
            'start_phase': state.get('start_phase', 'idle'),
            'sim_nav_valid': bool(sim_nav_ok),
            'sim_nav_reason': str(sim_nav.get('reason', 'not_checked') if isinstance(sim_nav, dict) else 'not_checked'),
            'mission_end_reason': (
                'start_rejected:fc_start_blocked:' + fc_start_block_reason
                if state.get('guided_ready') is not False
                else 'start_rejected:guided_prereq_missing:' + (','.join(missing) if missing else 'unknown')
            ),
        }), 409
    mission_data['start_requested'] = True
    mission_data['start_requested_at'] = time.monotonic()
    if not _touch_flag(FLAG_START):
        mission_data['start_requested'] = False
        mission_data['start_requested_at'] = 0.0
        return jsonify({'error': 'Start flag yazilamadi'}), 500
    print("📡 [DASHBOARD] Görev başlat isteği alındı")
    return jsonify({'ok': True})

@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Acil durdur (usv_main dosya IPC ile algılar)."""
    mission_data['stop_requested'] = True
    mission_data['start_requested'] = False
    mission_data['start_requested_at'] = 0.0
    _clear_flag(FLAG_START)
    if not _touch_flag(FLAG_STOP):
        return jsonify({'error': 'E-stop flag yazilamadi'}), 500
    try:
        if st is not None:
            st.force_estop_relay()
    except Exception as exc:
        # Silent fail - estop relay triggered, error not critical
        pass
    print("🚨 [DASHBOARD] ACİL DURDUR isteği alındı")
    return jsonify({'ok': True})

@app.route('/api/shutdown', methods=['POST'])
def shutdown_system():
    """Sistemin tamamını kapat (ESC gibi)."""
    print("⚠️  [DASHBOARD] SİSTEM KAPATMA komutu alındı")
    os.system('pkill -9 -f "gazebo|ardupilot|usv_main|ros2|cam|telemetry"')
    return jsonify({'ok': True, 'message': 'System shutting down...'})

install_module_function_tracing(
    globals(),
    component="telemetry",
    logger=_telem_dbg,
    prefer_simulation=bool(os.environ.get("USV_SIM") == "1"),
)

if __name__ == "__main__":
    clean_port(WEB_PORT)
    st = SmartTelemetry()
    st.start()

import time
from collections import deque
import pandas as pd
from pymavlink import mavutil
import sys
import threading
import os
import glob
import json
import random
import serial
import math
from flask import Flask, jsonify, render_template_string, request

from console_utils import make_console_printer
from compliance_profile import (
    COMMS_POLICY,
    GENERAL_TELEMETRY_HZ,
    LIDAR_PROCESSING_HZ,
    LINK_TOPOLOGY,
    OBSTACLE_TELEMETRY_HZ,
    REPORT_TELEMETRY_GROUPS,
    USV_MODE,
    USV_MODE_RACE,
)

print = make_console_printer("TELEM")

# --- YAPILANDIRMA VE SABİTLER ---
BAUD_RATE_PIXHAWK = 115200
BAUD_RATES_STM32 = [9600, 115200] # Otomatik denenir
WEB_PORT = 8080
LOG_DIR = "/root/workspace/logs"
CONTROL_DIR = "/root/workspace/control"
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

# --- GLOBAL DURUM ---
# Şartname Bölüm 6: lat, lon, hız, roll, pitch, heading, hız_setpoint, yön_setpoint
telemetry_data = {
    "Timestamp": "--", "Lat": 0, "Lon": 0, "Heading": 0,
    "Battery": 0, "Mode": "DISCONNECTED", "Speed": 0, "Roll": 0, "Pitch": 0,
    "Speed_Setpoint": 0, "Heading_Setpoint": 0,  # Şartname 6 zorunlu alanları
    "STM_Date": "--:--:--", "Env_Temp": 0, "Env_Hum": 0, "Rain_Val": 0, "Rain_Status": "DRY",
    "Sys_CPU": 0, "Sys_RAM": 0, "Sys_Temp": 0,
    "RC1": 0, "RC2": 0, "RC3": 0, "RC4": 0, "RC7": 0,
    "Out1": 0, "Out3": 0
}
COLUMNS = list(telemetry_data.keys())
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
    "stop_requested": False,   # Acil durdur isteği
}
last_state_cache = dict(mission_data)
EVENT_QUEUE = deque(maxlen=256)
EVENT_ID = 0
EVENT_LAST = {
    "gate_count": 0,
    "failsafe_state": "--",
    "estop_state": False,
    "timeout_count": 0,
}
EVENT_LOCK = threading.Lock()


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

    for kind, payload in to_emit:
        _emit_event(kind, payload)

# Flask Uygulaması
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR) # Gereksiz logları kapat
app = Flask(__name__)
st = None

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
        .m-btns { display:flex; flex-direction:column; gap:6px; margin-top:auto; }
        .btn { padding:10px; border:none; border-radius:8px; font-weight:700; font-size:0.8rem; cursor:pointer; transition:all 0.15s; text-transform:uppercase; letter-spacing:0.5px; font-family:var(--sans); }
        .btn:active { transform:scale(0.97); }
        .btn-start { background:linear-gradient(135deg,#22c55e,#16a34a); color:#fff; }
        .btn-start:hover { box-shadow:0 0 20px rgba(34,197,94,0.4); }
        .btn-next { background:linear-gradient(135deg,#38bdf8,#0284c7); color:#fff; }
        .btn-next:hover { box-shadow:0 0 20px rgba(56,189,248,0.4); }
        .btn-stop { background:linear-gradient(135deg,#ef4444,#b91c1c); color:#fff; }
        .btn-stop:hover { box-shadow:0 0 20px rgba(239,68,68,0.4); }
        .btn:disabled { opacity:0.4; cursor:not-allowed; box-shadow:none !important; }

        /* Parkur Progress */
        .parkur-steps { display:flex; gap:4px; justify-content:center; padding:6px 0; }
        .p-step { flex:1; height:6px; border-radius:3px; background:rgba(255,255,255,0.08); transition:all 0.3s; }
        .p-step.active { background:var(--accent); box-shadow:0 0 8px var(--accent-glow); }
        .p-step.done { background:var(--success); }

        /* === FEEDS === */
        .feed { background:#000; flex:1; display:flex; justify-content:center; align-items:center; position:relative; }
        .feed img { width:100%; height:100%; object-fit:contain; }
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

        /* Gear */
        .gear-disp { text-align:center; font-size:0.9rem; font-weight:700; margin-top:4px; }

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

                <div class="m-btns">
                    <button class="btn btn-start" id="btn_start" onclick="cmdStart()" {% if usv_mode == 'race' %}disabled{% endif %}>▶ Görevi Başlat</button>
                    <button class="btn btn-stop" id="btn_stop" onclick="cmdStop()">⛔ Acil Durdur</button>
                </div>
            </div>
        </div>

        {% if usv_mode == 'test' %}
        <!-- CAMERA FEED -->
        <div class="card">
            <div class="card-h"><span>Front Vision</span><span class="dot dot-g"></span></div>
            <div class="feed"><img id="cam_img" alt="Camera" /></div>
        </div>

        <!-- LIDAR MAP -->
        <div class="card">
            <div class="card-h"><span>Lidar Map</span><span class="dot dot-g"></span></div>
            <div class="feed"><img id="map_img" alt="Lidar Map" /></div>
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
                <div class="gear-disp" id="gear_disp">--</div>
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
        let missionRunning = false;

        function updateStats() {
            fetch('/api/data')
                .then(r => r.json())
                .then(d => {
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
                    document.getElementById('lat').innerText = (d.Lat || 0).toFixed(7);
                    document.getElementById('lon').innerText = (d.Lon || 0).toFixed(7);

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

                    // Gear
                    const gearEl = document.getElementById('gear_disp');
                    if(d.Gear) {
                        gearEl.innerText = d.Gear;
                        gearEl.style.color = d.Gear==='FORWARD' ? 'var(--success)' : (d.Gear==='REVERSE' ? 'var(--danger)' : 'var(--warning)');
                    }

                    // Motors
                    const mapPWM = (v) => { if(!v) return 0; let p=(v-1000)/10.0; return Math.max(0,Math.min(100,p)); };
                    document.getElementById('out1').innerText = d.Out1 || '--';
                    document.getElementById('mot1_bar').style.width = mapPWM(d.Out1) + '%';
                    document.getElementById('out3').innerText = d.Out3 || '--';
                    document.getElementById('mot3_bar').style.width = mapPWM(d.Out3) + '%';

                    // Mission Control
                    if(d.mission_state !== undefined) {
                        const states = ['BEKLEME','PARKUR-1','PARKUR-2','PARKUR-3','TAMAMLANDI'];
                        const stateEl = document.getElementById('m_state');
                        stateEl.innerText = states[d.mission_state] || 'BEKLEME';

                        // Parkur progress
                        for(let i=1;i<=3;i++) {
                            const el = document.getElementById('ps'+i);
                            if(d.mission_state > i) el.className='p-step done';
                            else if(d.mission_state === i) el.className='p-step active';
                            else el.className='p-step';
                        }

                        // Buttons
                        document.getElementById('btn_start').disabled = d.mission_active || d.usv_mode === 'race';
                        document.getElementById('btn_stop').disabled = !d.mission_active;

                        // Mission dot
                        document.getElementById('mc_dot').className = 'dot ' + (d.mission_active ? 'dot-g' : 'dot-r');
                    }

                    if(d.mission_target) document.getElementById('m_target').innerText = d.mission_target;
                    if(d.mission_wp_info) document.getElementById('m_wp').innerHTML = 'Waypoint: <b>' + d.mission_wp_info + '</b>';
                })
                .catch(() => {});
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
        function cmdStart() { fetch('/api/start_mission', {method:'POST'}).then(()=>{missionRunning=true;}); }
        function cmdStop() { fetch('/api/emergency_stop', {method:'POST'}).then(()=>{missionRunning=false;}); }

        setInterval(updateStats, 200);
        setInterval(updateTimer, 1000);

        // Load feeds (test mode only)
        window.onload = function() {
            {% if usv_mode == 'test' %}
            var host = window.location.hostname;
            var r = Math.random();
            var camEl = document.getElementById('cam_img');
            var mapEl = document.getElementById('map_img');
            if(camEl) camEl.src = 'http://' + host + ':5000/?t=' + r;
            if(mapEl) mapEl.src = 'http://' + host + ':5001/?t=' + r;
            {% endif %}
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
        self.last_link_heartbeat_time = 0.0
        self.link_heartbeat_age_s = 999.0
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
        
        # Log Klasörü
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)
            
        # CSV init - Her oturumda sıfırdan (Şartname 6: İDA karaya alındıktan sonra teslim)
        pd.DataFrame(columns=COLUMNS).to_csv(CSV_FILE, index=False)
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

    def _bump_error(self, key, message=None, period_s=5.0):
        self.error_counters[key] = int(self.error_counters.get(key, 0)) + 1
        if message:
            self._warn_throttled(
                f"err_{key}",
                f"{message} (count={self.error_counters[key]})",
                period_s=period_s,
            )

    def _publish_link_state(self, force=False):
        now = time.monotonic()
        if SIMULATION_MODE:
            self.link_heartbeat_age_s = 0.0
        elif self.pixhawk and self.last_link_heartbeat_time > 0.0:
            self.link_heartbeat_age_s = max(0.0, now - self.last_link_heartbeat_time)
        else:
            self.link_heartbeat_age_s = 999.0

        if not force and (now - self._last_link_state_write) < 0.2:
            return

        payload = {
            "ts_monotonic": round(now, 3),
            "link_heartbeat_age_s": round(self.link_heartbeat_age_s, 3),
            "error_counters": dict(self.error_counters),
        }
        try:
            os.makedirs(CONTROL_DIR, exist_ok=True)
            tmp_path = f"{LINK_STATE_FILE}.tmp"
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(payload, f)
            os.replace(tmp_path, LINK_STATE_FILE)
            self._last_link_state_write = now
        except Exception as exc:
            self._bump_error("link_state_write_error", f"[WARN] [LINK] state yazim hatasi: {exc}")

    def force_estop_relay(self):
        """YKİ E-stop tetiklenince RC7'yi kesme seviyesine zorla."""
        if not self.pixhawk:
            return False
        try:
            for _ in range(6):
                rc_override = [65535] * 8
                rc_override[6] = RC7_ESTOP_FORCE_PWM
                self.pixhawk.mav.rc_channels_override_send(
                    self.target_system_id if getattr(self, "target_system_id", None) else 1,
                    self.pixhawk.target_component,
                    *rc_override
                )
                time.sleep(0.04)
            with self.lock:
                telemetry_data["RC7"] = RC7_ESTOP_FORCE_PWM
            print("🚨 [ESTOP] RC7 force gönderildi")
            return True
        except Exception as e:
            self._bump_error("rc_override_error", f"[WARN] [ESTOP] RC7 force hatasi: {e}")
            return False

    def csv_logger(self):
        """Şartname Bölüm 6: Telemetri CSV 1 Hz kayıt"""
        import csv as csv_mod
        while self.running:
            try:
                if time.time() - self.last_csv_log_time >= CSV_LOG_INTERVAL:
                    with self.lock:
                        row = [telemetry_data.get(k, '') for k in COLUMNS]
                        if row:
                            row[0] = self._csv_timestamp()
                    with open(CSV_FILE, 'a', newline='') as f:
                        csv_mod.writer(f).writerow(row)
                    self.last_csv_log_time = time.time()
            except Exception as exc:
                self._bump_error("csv_write_error", f"[WARN] [CSV] Yazma hatasi: {exc}")
            time.sleep(0.5)

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
        app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)

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
            time.sleep(10)

    def scan_ports(self):
        """Bostaki portlari tarar ve uygun cihazlari eslestirir."""
        # Pixhawk: Önce UDP dene (mavproxy aktifse)
        if not self.pixhawk:
            if self._probe_pixhawk('udpin:0.0.0.0:14550'):
                print("📡 [MAV] Pixhawk bağlantısı: UDP:14550 (mavproxy)")
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
        # Genel veriler rapor hedefi: 5 Hz
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, GENERAL_TELEMETRY_HZ, 1
        )
        # RC kanalları rapor hedefi: 5 Hz
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, GENERAL_TELEMETRY_HZ, 1
        )
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
            except Exception as e:
                self._bump_error("mav_read_error", f"[WARN] [MAV] Okuma hatasi: {e}")
                self._publish_link_state()

    def _process_mavlink_msg(self, msg, rc_logged):
        """MAVLink mesajlarını işleyen yardımcı metot."""
        mtype = msg.get_type()
        
        with self.lock:
            telemetry_data["Timestamp"] = self._csv_timestamp()
            
            if mtype == 'GLOBAL_POSITION_INT':
                telemetry_data['Lat'] = msg.lat / 1e7
                telemetry_data['Lon'] = msg.lon / 1e7
                telemetry_data['Heading'] = msg.hdg / 100.0
                # Manuel modda yön setpoint = mevcut yön; AUTO/GUIDED'da NAV_CONTROLLER_OUTPUT kullanılır
                mode_str = telemetry_data.get('Mode', '')
                if 'AUTO' not in mode_str and 'GUIDED' not in mode_str:
                    telemetry_data['Heading_Setpoint'] = msg.hdg / 100.0

            elif mtype == 'GPS_RAW_INT':
                # time_usec GNSS epoch ise CSV zamanında önceliklidir
                t_usec = getattr(msg, 'time_usec', 0)
                if t_usec and t_usec > 1e14:
                    self.last_gnss_epoch = t_usec / 1e6
                
            elif mtype == 'RC_CHANNELS':
                telemetry_data['RC1'] = msg.chan1_raw
                telemetry_data['RC2'] = msg.chan2_raw
                telemetry_data['RC3'] = msg.chan3_raw
                telemetry_data['RC4'] = msg.chan4_raw
                telemetry_data['RC7'] = getattr(msg, 'chan7_raw', 0)
                
                # Motor Kontrolcüsüne Veri Gönder
                # CH1: Direksiyon, CH3: Gaz Artır/Azalt, CH6: Vites
                rc6 = msg.chan6_raw
                # YENİ: Tüm kanalları gönderiyoruz (CH1, CH2, CH3, CH4, CH6)
                self.motor_ctrl.update_inputs(msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, rc6)
                
                # --- RC DEBUG (Kanal Tespiti) ---
                # Her 25 mesajda bir (~5 saniyede 1) loga bas — CPU tasarrufu
                self.rc_debug_counter = getattr(self, 'rc_debug_counter', 0) + 1
                if self.rc_debug_counter % 25 == 0:
                    gear_stat = telemetry_data.get('Gear', 'N')
                    print(f"🎮 RC: 1:{msg.chan1_raw} 2:{msg.chan2_raw} 3:{msg.chan3_raw} 4:{msg.chan4_raw} 6:{msg.chan6_raw} -> {gear_stat}")

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
                # Hız setpoint: Motor output yoksa (NEUTRAL) mevcut hız kullan
                if telemetry_data.get('Gear') == 'NEUTRAL' or abs(telemetry_data.get('Out1', 1500) - 1500) + abs(telemetry_data.get('Out3', 1500) - 1500) < 50:
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
                "Gear": "NEUTRAL",
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
        self.target_pwm = 1500.0   # Hedeflenen Hız (Sanal)
        self.current_pwm = 1500.0  # Anlık Fiziksel Hız (Ramping uygulanan)
        self.gear = "NEUTRAL"      # Vites: FORWARD, NEUTRAL, REVERSE
        self.last_gear_switch = 0  # Debouncing için
        
        # Girdiler
        self.input_throttle = 1500 # CH3 (Sol Stick)
        self.input_steer = 1500    # CH1 (Sağ Stick)
        self.input_gear = 1000     # CH6 (Switch)
        self.left_motor_pwm = 1500
        self.right_motor_pwm = 1500
        self.last_input_ts = time.monotonic()
        
        # Ayarlar
        self.PWM_NEUTRAL = 1500
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

    def update_inputs(self, rc1, rc2, rc3, rc4, rc6):
        """RC verilerini filtreleyip güvenli girişlere dönüştür."""
        # Primary/fallback: throttle rc2->rc3, steer rc4->rc1
        self.input_throttle = _pick_primary_fallback(rc2, rc3, default=1500)
        self.input_steer = _pick_primary_fallback(rc4, rc1, default=1500)
        if _is_valid_pwm(rc6):
            self.input_gear = int(rc6)
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
            mission_locked = bool(
                state.get("active", False)
                or state.get("command_lock", False)
                or state.get("estop_state", False)
                or mode_locked
                or state_stale
            )
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
                self.left_motor_pwm = 1500
                self.right_motor_pwm = 1500
                self.gear = "LOCKED"
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
                        self.parent._bump_error("rc_override_error", f"[WARN] [MOTOR] Lock RC override hatasi: {exc}")
                with self.parent.lock:
                    telemetry_data["Gear"] = "LOCKED"
                    telemetry_data["Out1"] = 1500
                    telemetry_data["Out3"] = 1500
                    telemetry_data["Manual_Lock_Reason"] = lock_reason
                continue

            # RC güncellemesi kesilirse manuel çıkışları nötre al.
            if (time.monotonic() - self.last_input_ts) > RC_SIGNAL_TIMEOUT_S:
                self.target_pwm = 1500
                self.current_pwm = 1500
                self.current_turn = 0.0
                self.input_throttle = 1500
                self.input_steer = 1500
                self.gear = "NEUTRAL"
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
                    telemetry_data["Gear"] = "RC_LOSS_HOLD"
                    telemetry_data["Out1"] = 1500
                    telemetry_data["Out3"] = 1500
                    telemetry_data["Manual_Lock_Reason"] = "RC_TIMEOUT"
                continue
             
            # --- 1. VİTES MANTIĞI (CH6) ---
            new_gear = "NEUTRAL"
            if self.input_gear > 1700: new_gear = "FORWARD"
            elif self.input_gear < 1300: new_gear = "REVERSE"
            
            if new_gear != self.gear:
                 if self.gear == "FORWARD" and new_gear == "REVERSE":
                     # Hızımız henüz durma noktasına gelmediyse, NEUTRAL'a alıp yavaşlamasını bekle
                     if abs(self.current_pwm - 1500) > 25: 
                         new_gear = "NEUTRAL"
                 elif self.gear == "REVERSE" and new_gear == "FORWARD":
                     if abs(self.current_pwm - 1500) > 25: 
                         new_gear = "NEUTRAL"
                 self.gear = new_gear
                 if self.gear == "NEUTRAL": 
                     self.target_pwm = 1500

            # --- 2. CRUISE CONTROL GİRDİSİ (SOL STICK - CH3) ---
            throttle_raw_diff = self.input_throttle - 1500
            if self.INV_THROTTLE: throttle_raw_diff *= -1
            
            if abs(throttle_raw_diff) > self.PWM_DEADZONE:
                power_factor = (abs(throttle_raw_diff) - self.PWM_DEADZONE) / (500.0 - self.PWM_DEADZONE)
                step_val = self.CRUISE_STEP * power_factor * 2.0 
                
                if throttle_raw_diff > 0:
                    # HIZ ARTIR
                    if self.gear == "FORWARD":
                        self.target_pwm = min(self.target_pwm + step_val, self.MAX_FWD)
                    elif self.gear == "REVERSE":
                        self.target_pwm = max(self.target_pwm - step_val, self.MAX_REV)
                else: 
                    # HIZ AZALT (Frenleme hissiyatı vermek için daha hızlı yavaşlayabiliriz)
                    brake_factor = 2.0
                    if self.target_pwm > 1500:
                        self.target_pwm = max(self.target_pwm - (step_val * brake_factor), 1500)
                    elif self.target_pwm < 1500:
                        self.target_pwm = min(self.target_pwm + (step_val * brake_factor), 1500)

            # --- 3. EXPONENTIAL RAMPING (YUMUŞAK GEÇİŞ - LPF) ---
            # Lineer rampa yerine üstel filtre (Low Pass Filter) ile daha gerçekçi tekne ataleti
            alpha_pwm = 0.15  # 0.0 - 1.0 (küçük değerler = daha yavaş/yumuşak hızlanma)
            self.current_pwm += (self.target_pwm - self.current_pwm) * alpha_pwm
            
            # Küsüratları temizle ve tam durmayı garantiye al
            if abs(self.target_pwm - self.current_pwm) < 2.0:
                self.current_pwm = self.target_pwm

            # --- 4. MIXING (NORMALIZED DIFFERENTIAL DRIVE) ---
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
            
            # --- 5. SAFETY CLAMP & DYNAMIC LIMITS ---
            # İleri viteste aniden tam geri veya geri viteste tam ileri komutlarını yumuşatma
            if self.gear == "FORWARD":
                left_pwm_out = max(1450, left_pwm_out)
                right_pwm_out = max(1450, right_pwm_out)
            elif self.gear == "REVERSE":
                left_pwm_out = min(1550, left_pwm_out)
                right_pwm_out = min(1550, right_pwm_out)
                
            if self.gear == "NEUTRAL": 
                left_pwm_out = 1500
                right_pwm_out = 1500
                
            left_pwm_out = int(max(1100, min(left_pwm_out, 1900)))
            right_pwm_out = int(max(1100, min(right_pwm_out, 1900)))
            
            # Düşük pwm kesilmesi
            if abs(left_pwm_out - 1500) < 15: left_pwm_out = 1500
            if abs(right_pwm_out - 1500) < 15: right_pwm_out = 1500
            
            self.left_motor_pwm = left_pwm_out
            self.right_motor_pwm = right_pwm_out
            
            # --- 6. ÇIKIŞ (MAVLINK OVERRIDE) ---
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
                        telemetry_data["Gear"] = self.gear
                        telemetry_data["Out1"] = left_pwm_out
                        telemetry_data["Out3"] = right_pwm_out
                        telemetry_data["Manual_Lock_Reason"] = "MANUAL_ACTIVE"
                        
                        thrust_norm = (self.current_pwm - 1500) / 400.0
                        telemetry_data["Speed_Setpoint"] = thrust_norm * 3.0
                        telemetry_data["RC1"] = self.input_steer
                        telemetry_data["RC3"] = self.input_throttle
                        
                except Exception as e:
                    self.parent._bump_error("rc_override_error", f"[WARN] [MOTOR] Override hatasi: {e}")



# --- FLASK ROUTES ---
@app.route('/')
def index():
    return render_template_string(HTML_PAGE, usv_mode=USV_MODE)

def _read_mission_state():
    """usv_main tarafından yazılan state dosyasını oku."""
    global last_state_cache
    try:
        if os.path.exists(STATE_FILE):
            with open(STATE_FILE, 'r', encoding='utf-8') as f:
                state = json.load(f)
            if isinstance(state, dict):
                last_state_cache = state
                return state
    except Exception as exc:
        # Atomic olmayan yazım anında parse hatası olabilir; son geçerli state'e düş.
        if st is not None:
            st._bump_error("state_read_error", f"[WARN] [IPC] State okuma hatasi: {exc}")
    return last_state_cache

@app.route('/api/data')
def get_data():
    out = dict(telemetry_data)
    out['usv_mode'] = USV_MODE
    state = _read_mission_state()
    _sync_events_from_state(state)
    out['mission_state'] = state.get('state', mission_data['state'])
    out['mission_active'] = state.get('active', mission_data['active'])
    out['mission_target'] = state.get('target', mission_data['target'])
    out['mission_wp_info'] = state.get('wp_info', mission_data['wp_info'])
    out['failsafe_state'] = state.get('failsafe_state', '--')
    out['estop_state'] = state.get('estop_state', False)
    out['estop_source'] = state.get('estop_source', '--')
    out['command_lock'] = state.get('command_lock', False)
    out['gate_count'] = state.get('gate_count', 0)
    out['health_check'] = state.get('health_check', {})
    out['ready_state'] = state.get('ready_state', False)
    out['ready_missing'] = state.get('ready_missing', [])
    out['heartbeat_age_s'] = state.get('heartbeat_age_s', 0.0)
    out['link_heartbeat_age_s'] = state.get(
        'link_heartbeat_age_s',
        round(st.link_heartbeat_age_s, 3) if st is not None else out['heartbeat_age_s'],
    )
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
    out['lidar_ready'] = state.get('lidar_ready', False)
    out['active_parkur'] = state.get('active_parkur', '--')
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
            'mission_state': out['mission_state'],
            'active_parkur': out['active_parkur'],
        },
        'mission_progress': {
            'target': out['mission_target'],
            'wp_info': out['mission_wp_info'],
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
            'heartbeat_age_s': out['heartbeat_age_s'],
            'link_heartbeat_age_s': out['link_heartbeat_age_s'],
            'state_age_s': out['state_age_s'],
            'sensor_fusion': fusion_summary,
            'dynamic_speed_profile': dyn_speed_summary,
            'wind_assist': wind_assist_summary,
            'horizon_lock': horizon_lock_summary,
            'camera_adaptation': camera_adaptation_summary,
            'virtual_anchor': virtual_anchor_summary,
        },
        'link_health': {
            'rc_link_active': bool(900 <= telemetry_data.get('RC1', 0) <= 2100 or 900 <= telemetry_data.get('RC3', 0) <= 2100),
            'telemetry_heartbeat_age_s': out['link_heartbeat_age_s'],
            'onboard_heartbeat_age_s': out['heartbeat_age_s'],
        },
        'safety': {
            'command_lock': out['command_lock'],
            'estop_state': out['estop_state'],
            'estop_source': out['estop_source'],
            'manual_lock_reason': out['manual_lock_reason'],
            'health_ready': out['ready_state'],
            'health_missing': out['ready_missing'],
        },
        'energy': {
            'battery_v': telemetry_data.get('Battery', 0),
        },
    }
    return jsonify(out)

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
    if USV_MODE == USV_MODE_RACE:
        return jsonify({'error': 'Race modunda gorev baslatma sadece RC CH5 ile yapilir (API politikasi kapali).'}), 403
    mission_data['start_requested'] = True
    if not _touch_flag(FLAG_START):
        return jsonify({'error': 'Start flag yazilamadi'}), 500
    print("📡 [DASHBOARD] Görev başlat isteği alındı")
    return jsonify({'ok': True})

@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Acil durdur (usv_main dosya IPC ile algılar)."""
    mission_data['stop_requested'] = True
    if not _touch_flag(FLAG_STOP):
        return jsonify({'error': 'E-stop flag yazilamadi'}), 500
    try:
        if st is not None:
            st.force_estop_relay()
    except Exception as exc:
        if st is not None:
            st._bump_error("rc_override_error", f"[WARN] [ESTOP] API estop relay hatasi: {exc}")
    print("🚨 [DASHBOARD] ACİL DURDUR isteği alındı")
    return jsonify({'ok': True})

if __name__ == "__main__":
    clean_port(WEB_PORT)
    st = SmartTelemetry()
    st.start()

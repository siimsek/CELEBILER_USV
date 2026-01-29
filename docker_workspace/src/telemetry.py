import time
import pandas as pd
from pymavlink import mavutil
import sys
import threading
import os
import glob
import json
import random
from flask import Flask, jsonify, render_template_string

# --- AYARLAR ---
BAUD_RATE_PIXHAWK = 115200
CSV_FILE = "/root/workspace/logs/telemetri_verisi.csv"
WEB_PORT = 8080

# Flask Uygulaması
app = Flask(__name__)

# Küresel Veri
telemetry_data = {
    "Timestamp": "--", "Lat": 0, "Lon": 0, "Heading": 0,
    "Battery": 0, "Mode": "DISCONNECTED", "Speed": 0, "Roll": 0, "Pitch": 0,
    "Rain_Val": 0, "Env_Temp": 0
}
COLUMNS = list(telemetry_data.keys())
SIMULATION_MODE = False

# --- HTML ARAYÜZ ---
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <title>CELEBILER USV - MISSION CONTROL</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        :root {
            --bg-color: #0f172a;
            --card-bg: #1e293b;
            --accent: #38bdf8;
            --text-primary: #f8fafc;
            --text-secondary: #94a3b8;
            --success: #4ade80;
            --danger: #ef4444;
            --font-mono: 'JetBrains Mono', 'Courier New', monospace;
        }

        body {
            background-color: var(--bg-color);
            color: var(--text-primary);
            font-family: 'Inter', system-ui, -apple-system, sans-serif;
            margin: 0;
            padding: 0;
            height: 100vh;
            overflow: hidden; /* Scroll yok */
            display: flex;
            flex-direction: column;
        }

        /* HEADER */
        header {
            background: rgba(30, 41, 59, 0.8);
            backdrop-filter: blur(10px);
            border-bottom: 1px solid rgba(255,255,255,0.1);
            padding: 0 20px;
            height: 60px;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        h1 {
            font-size: 1.2rem;
            font-weight: 700;
            margin: 0;
            color: var(--accent);
            letter-spacing: 1px;
            display: flex;
            align-items: center;
            gap: 10px;
        }

        /* MAIN LAYOUT */
        .dashboard-grid {
            flex: 1;
            display: grid;
            grid-template-columns: 1fr 1fr; /* Sol Kamera, Sağ Harita */
            grid-template-rows: 2fr 1fr;    /* Üst Video, Alt İstatistik */
            gap: 15px;
            padding: 15px;
            height: calc(100vh - 60px);
        }

        /* CARDS */
        .card {
            background: var(--card-bg);
            border-radius: 12px;
            border: 1px solid rgba(255,255,255,0.05);
            box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.3);
            display: flex;
            flex-direction: column;
            overflow: hidden;
            position: relative;
        }

        .card-header {
            padding: 10px 15px;
            background: rgba(0,0,0,0.2);
            font-size: 0.9rem;
            font-weight: 600;
            color: var(--text-secondary);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        /* VIDEO FRAMES */
        .video-container {
            flex: 1;
            position: relative;
            background: #000;
        }

        iframe {
            width: 100%;
            height: 100%;
            border: none;
            display: block;
        }

        /* STATS GRID */
        .stats-area {
            grid-column: span 2; /* Alt kısım boydan boya */
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 15px;
        }

        .stat-card {
            background: rgba(255,255,255,0.03);
            border-radius: 8px;
            padding: 15px;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            text-align: center;
            transition: transform 0.2s;
        }

        .stat-card:hover { transform: translateY(-2px); background: rgba(255,255,255,0.05); }

        .stat-label { font-size: 0.8rem; color: var(--text-secondary); text-transform: uppercase; letter-spacing: 0.5px; }
        .stat-value { font-family: var(--font-mono); font-size: 1.5rem; font-weight: 700; color: var(--text-primary); margin-top: 5px; }
        
        /* STATUS INDICATORS */
        .status-dot { width: 8px; height: 8px; border-radius: 50%; display: inline-block; margin-right: 6px; }
        .status-online { background-color: var(--success); box-shadow: 0 0 10px var(--success); }
        .status-offline { background-color: var(--danger); }

        /* MODIFIER COLORS */
        .val-accent { color: var(--accent); }
        .val-success { color: var(--success); }
    </style>
    <script>
        function updateStats() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('ts').innerText = data.Timestamp;
                    document.getElementById('mode').innerText = data.Mode;
                    document.getElementById('bat').innerText = data.Battery.toFixed(1) + " V";
                    document.getElementById('gps').innerText = data.Lat.toFixed(5) + ", " + data.Lon.toFixed(5);
                    document.getElementById('hdg').innerText = data.Heading.toFixed(0) + "°";
                    document.getElementById('spd').innerText = data.Speed.toFixed(1) + " m/s";

                    // Mod Renklendirme
                    const modeEl = document.getElementById('mode');
                    modeEl.style.color = data.Mode === "DISCONNECTED" ? "var(--danger)" : "var(--success)";
                });
        }
        setInterval(updateStats, 1000);

        // Dinamik IP Çözücü
        window.onload = function() {
            var host = window.location.hostname;
            console.log("Detected Host: " + host);
            document.getElementById('cam_frame').src = "http://" + host + ":5000";
            document.getElementById('map_frame').src = "http://" + host + ":5001";
        }
    </script>
</head>
<body>
    <header>
        <h1><span class="status-dot status-online"></span> ÇELEBİLER USV <span style="font-weight:400; font-size:0.9rem; color:var(--text-secondary)">// MISSION CONTROL</span></h1>
        <div style="font-family: var(--font-mono); font-size: 0.9rem;" id="ts">--:--:--</div>
    </header>

    <div class="dashboard-grid">
        <!-- KAMERA -->
        <div class="card">
            <div class="card-header">
                <span>FRONT CAMERA</span>
                <span class="status-dot status-online"></span>
            </div>
            <div class="video-container">
                <iframe id="cam_frame" src=""></iframe>
            </div>
        </div>

        <!-- LIDAR -->
        <div class="card">
            <div class="card-header">
                <span>LIDAR MAPPING</span>
                <span class="status-dot status-online"></span>
            </div>
            <div class="video-container">
                <iframe id="map_frame" src=""></iframe>
            </div>
        </div>

        <!-- İSTATİSTİKLER -->
        <div class="stats-area">
            <div class="stat-card">
                <div class="stat-label">BATTERY</div>
                <div class="stat-value val-accent" id="bat">-- V</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">SYSTEM MODE</div>
                <div class="stat-value" id="mode">--</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">GPS POSITION</div>
                <div class="stat-value" id="gps" style="font-size: 1.1rem;">--.----, --.----</div>
            </div>
            <div class="stat-card">
                <div class="stat-label">SPEED / HEADING</div>
                <div class="stat-value"><span id="spd">--</span> <span style="font-size:1rem; color:#555">|</span> <span id="hdg">--</span></div>
            </div>
        </div>
    </div>
</body>
</html>
"""

def clean_port(port):
    """Sadece belirtilen portu kullanan işlemi öldürür."""
    print(f"🧹 Port {port} temizleniyor...")
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")
    os.system(f"lsof -t -i:{port} | xargs kill -9 > /dev/null 2>&1")
    time.sleep(0.5)

class SmartTelemetry:
    def __init__(self):
        self.master = None
        self.running = True
        self.lock = threading.Lock()
        self.sim_lat = 38.4192
        self.sim_lon = 27.1287
        
        # Log klasörü kontrolü
        if not os.path.exists("/root/workspace/logs"):
            os.makedirs("/root/workspace/logs")

    def connect_pixhawk(self):
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        for port in ports:
            try:
                print(f"🔌 Port deneniyor: {port}")
                conn = mavutil.mavlink_connection(port, baud=BAUD_RATE_PIXHAWK)
                conn.wait_heartbeat(timeout=1)
                print(f"✅ Pixhawk bulundu: {port}")
                return conn
            except: pass
        return None

    def update_simulation(self):
        """Donanım yoksa rastgele veriler üret"""
        global telemetry_data
        with self.lock:
            # Random Walk GPS
            self.sim_lat += random.uniform(-0.0001, 0.0001)
            self.sim_lon += random.uniform(-0.0001, 0.0001)
            
            telemetry_data["Timestamp"] = time.strftime("%H:%M:%S")
            telemetry_data["Lat"] = self.sim_lat
            telemetry_data["Lon"] = self.sim_lon
            telemetry_data["Heading"] = (telemetry_data["Heading"] + 1) % 360
            telemetry_data["Battery"] = 12.0 + random.uniform(0, 0.5)
            telemetry_data["Speed"] = random.uniform(0, 3.0)
            telemetry_data["Mode"] = "SIMULATION"

    def read_loop(self):
        global SIMULATION_MODE
        
        # CSV Başlıkları
        if not os.path.isfile(CSV_FILE):
            pd.DataFrame(columns=COLUMNS).to_csv(CSV_FILE, index=False)

        print(f"📡 Telemetri Döngüsü Başladı...")

        while self.running:
            if not self.master:
                self.master = self.connect_pixhawk()
                if not self.master:
                    if not SIMULATION_MODE:
                        print("⚠️ DONANIM YOK - SIMULASYON MODUNDA ÇALIŞIYOR")
                        SIMULATION_MODE = True
                    
                    self.update_simulation()
                    time.sleep(1)
                    continue
                else:
                    SIMULATION_MODE = False
                    print("🚀 CANLI MOD AKTİF")

            try:
                # Canlı Veri Okuma
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if not msg:
                    # Timeout durumunda bağlantıyı kontrol et veya simülasyona dön
                    continue
                
                with self.lock:
                    telemetry_data["Timestamp"] = time.strftime("%H:%M:%S")
                    
                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        telemetry_data['Lat'] = msg.lat / 1e7
                        telemetry_data['Lon'] = msg.lon / 1e7
                        telemetry_data['Heading'] = msg.hdg / 100.0
                    elif msg.get_type() == 'SYS_STATUS':
                        telemetry_data['Battery'] = msg.voltage_battery / 1000.0
                    elif msg.get_type() == 'HEARTBEAT':
                        telemetry_data['Mode'] = mavutil.mode_string_v10(msg)
                    elif msg.get_type() == 'VFR_HUD':
                        telemetry_data['Speed'] = msg.groundspeed

                # CSV Kayıt (Saniyede bir)
                if int(time.time()) % 2 == 0:
                    df = pd.DataFrame([telemetry_data])
                    df.to_csv(CSV_FILE, mode='a', header=False, index=False)

            except Exception as e:
                print(f"⚠️ Hata: {e}")
                self.master = None # Yeniden bağlanmayı dene

    def start(self):
        t = threading.Thread(target=self.read_loop, daemon=True)
        t.start()
        
        print(f"🌍 WEB SERVER BAŞLATILIYOR: Port {WEB_PORT}")
        app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)

# Flask Rotaları
@app.route('/')
def index(): return render_template_string(HTML_PAGE)

@app.route('/api/data')
def get_data(): return jsonify(telemetry_data)

if __name__ == "__main__":
    clean_port(WEB_PORT)
    st = SmartTelemetry()
    st.start()

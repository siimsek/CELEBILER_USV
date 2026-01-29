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
<html>
<head>
    <title>CELEBILER USV - MISSION CONTROL</title>
    <meta charset="UTF-8">
    <style>
        body { background-color: #121212; color: #e0e0e0; font-family: monospace; margin: 0; padding: 20px; }
        h1 { text-align: center; color: #00d4ff; border-bottom: 2px solid #00d4ff; padding-bottom: 10px; }
        .grid-container { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-top: 20px; }
        .video-box { border: 2px solid #333; background: #000; text-align: center; height: 400px; }
        .video-box h2 { margin: 0; padding: 10px; background: #1f1f1f; color: #ffeb3b; }
        iframe { width: 100%; height: 350px; border: none; }
        .stats-box { grid-column: span 2; background: #1e1e1e; padding: 20px; border: 1px solid #444; }
        table { width: 100%; border-collapse: collapse; }
        th, td { border: 1px solid #333; padding: 10px; text-align: left; }
        th { background-color: #2c2c2c; color: #00d4ff; }
        .status-ok { color: #00ff00; }
    </style>
    <script>
        function updateStats() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('ts').innerText = data.Timestamp;
                    document.getElementById('mode').innerText = data.Mode;
                    document.getElementById('bat').innerText = data.Battery.toFixed(1) + " V";
                    document.getElementById('gps').innerText = data.Lat.toFixed(6) + ", " + data.Lon.toFixed(6);
                    document.getElementById('hdg').innerText = data.Heading.toFixed(1) + "°";
                    document.getElementById('spd').innerText = data.Speed.toFixed(1) + " m/s";
                });
        }
        setInterval(updateStats, 1000);
    </script>
</head>
<body>
    <h1>🚀 ÇELEBİLER USV - YER İSTASYONU</h1>
    <div class="grid-container">
        <div class="video-box">
            <h2>📷 KAMERA (Port 5000)</h2>
            <iframe src="http://192.168.11.5:5000"></iframe>
        </div>
        <div class="video-box">
            <h2>📡 LIDAR HARİTA (Port 5001)</h2>
            <iframe src="http://192.168.11.5:5001"></iframe>
        </div>
        <div class="stats-box">
            <h2>📊 CANLI TELEMETRİ</h2>
            <table>
                <tr><th>Zaman</th><td id="ts">--</td><th>Mod</th><td id="mode">--</td></tr>
                <tr><th>Batarya</th><td id="bat">--</td><th>Konum</th><td id="gps">--</td></tr>
                <tr><th>Pusula</th><td id="hdg">--</td><th>Hız</th><td id="spd">--</td></tr>
            </table>
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

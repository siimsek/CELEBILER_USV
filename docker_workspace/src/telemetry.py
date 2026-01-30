import time
import pandas as pd
from pymavlink import mavutil
import sys
import threading
import os
import glob
import json
import random
import serial
from flask import Flask, jsonify, render_template_string

# --- AYARLAR ---
BAUD_RATE_PIXHAWK = 115200
BAUD_RATE_STM32 = 115200
CSV_FILE = "/root/workspace/logs/telemetri_verisi.csv"
WEB_PORT = 8080

# Flask Uygulaması
app = Flask(__name__)

# Küresel Veri
telemetry_data = {
    "Timestamp": "--", "Lat": 0, "Lon": 0, "Heading": 0,
    "Battery": 0, "Mode": "DISCONNECTED", "Speed": 0, 
    "STM_Date": "--:--:--", "Env_Temp": 0, "Env_Hum": 0, "Rain_Val": 0, "Rain_Status": "DRY",
    "Sys_CPU": 0, "Sys_RAM": 0, "Sys_Temp": 0
}
COLUMNS = list(telemetry_data.keys())
SIMULATION_MODE = False

# --- HTML ARAYÜZ (GÜNCELLENMİŞ) ---
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
            --warning: #facc15;
            --font-mono: 'JetBrains Mono', 'Courier New', monospace;
        }

        body {
            background-color: var(--bg-color);
            color: var(--text-primary);
            font-family: 'Inter', system-ui, -apple-system, sans-serif;
            margin: 0;
            padding: 0;
            height: 100vh;
            overflow: hidden;
            display: flex;
            flex-direction: column;
        }

        /* HEADER */
        header {
            background: rgba(15, 23, 42, 0.95);
            border-bottom: 1px solid rgba(255,255,255,0.1);
            padding: 0 20px;
            height: 60px;
            display: flex;
            align-items: center;
            justify-content: space-between;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.2);
            z-index: 10;
        }

        .header-left { display: flex; align-items: center; gap: 15px; }
        .header-right { display: flex; align-items: center; gap: 20px; }

        h1 {
            font-size: 1.1rem;
            font-weight: 800;
            margin: 0;
            color: var(--accent);
            letter-spacing: 0.5px;
            display: flex;
            align-items: center;
            gap: 10px;
        }

        /* SYSTEM METRICS (HEADER) */
        .sys-metric {
            display: flex;
            align-items: center;
            gap: 6px;
            font-size: 0.8rem;
            color: var(--text-secondary);
            background: rgba(255,255,255,0.05);
            padding: 6px 12px;
            border-radius: 8px;
            border: 1px solid rgba(255,255,255,0.05);
        }
        
        .sys-metric span {
            font-family: var(--font-mono);
            font-weight: 700;
            color: var(--text-primary);
        }

        .metric-icon { opacity: 0.7; }

        /* DASHBOARD GRID */
        .dashboard-grid {
            flex: 1;
            display: grid;
            grid-template-columns: 1fr 1fr;
            grid-template-rows: 75% 25%; /* Video Alanı Artırıldı, İstatistik Azaltıldı */
            gap: 15px; /* Boşluklar azaltıldı */
            padding: 15px;
            max-width: 1920px;
            margin: 0 auto;
            width: 100%;
            box-sizing: border-box;
        }

        /* CARDS */
        .card {
            background: var(--card-bg);
            border-radius: 12px; /* Köşeler biraz daha keskin */
            border: 1px solid rgba(255,255,255,0.06);
            display: flex;
            flex-direction: column;
            overflow: hidden;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.2);
            position: relative;
        }

        .card-header {
            padding: 8px 15px; /* Header küçültüldü */
            background: rgba(15, 23, 42, 0.4);
            font-size: 0.8rem;
            font-weight: 700;
            color: var(--text-secondary);
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid rgba(255,255,255,0.03);
            letter-spacing: 0.5px;
        }

        /* VIDEO CONTAINER */
        .video-container {
            flex: 1;
            background: #000;
            position: relative;
            display: flex;
            justify-content: center;
            align-items: center;
            overflow: hidden;
        }

        .stream-img {
            max-width: 100%;
            max-height: 100%;
            width: auto;
            height: auto;
            object-fit: contain;
        }

        /* STATS GRID */
        .stats-area {
            grid-column: span 2;
            display: grid;
            grid-template-columns: repeat(5, 1fr);
            gap: 15px;
        }

        .stat-card {
            background: linear-gradient(145deg, rgba(255,255,255,0.03) 0%, rgba(255,255,255,0.01) 100%);
            border-radius: 10px;
            padding: 15px; /* İç boşluk azaltıldı */
            display: flex;
            flex-direction: column;
            justify-content: center; /* Ortalandı */
            gap: 5px;
            border: 1px solid rgba(255,255,255,0.05);
        }

        .stat-label { 
            font-size: 0.85rem; 
            color: var(--text-secondary); 
            text-transform: uppercase; 
            font-weight: 600;
            margin-bottom: 4px;
        }
        
        .stat-value { 
            font-family: var(--font-mono); 
            font-size: 1.8rem;
            font-weight: 700; 
            color: var(--text-primary); 
            white-space: nowrap;
        }
        
        .status-dot { width: 8px; height: 8px; border-radius: 50%; display: block; }
        .bg-green { background: var(--success); box-shadow: 0 0 8px var(--success); }
        .bg-red { background: var(--danger); }

        /* Helpers */
        .unit { font-size: 0.9rem; color: var(--text-secondary); font-weight: 400; }
        .sub-val { font-size: 0.8rem; color: var(--accent); display: block; margin-top: 4px;}
        .divider { width: 1px; height: 20px; background: rgba(255,255,255,0.1); margin: 0 10px; }
    </style>
    <script>
        function updateStats() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    // System (Header)
                    document.getElementById('sys_cpu').innerText = data.Sys_CPU + "%";
                    document.getElementById('sys_ram').innerText = data.Sys_RAM + "%";
                    document.getElementById('sys_temp').innerText = data.Sys_Temp + "°C";
                    document.getElementById('ts').innerText = data.Timestamp;

                    // Main Stats
                    document.getElementById('bat').innerHTML = data.Battery.toFixed(1) + "<span class='unit'> V</span>";
                    document.getElementById('mode').innerText = data.Mode;
                    
                    // GPS
                    document.getElementById('lat').innerText = data.Lat.toFixed(6);
                    document.getElementById('lon').innerText = data.Lon.toFixed(6);
                    
                    // Nav
                    document.getElementById('spd').innerText = data.Speed.toFixed(1);
                    document.getElementById('hdg').innerText = data.Heading.toFixed(0);
                    
                    // Environment
                    document.getElementById('temp').innerHTML = data.Env_Temp.toFixed(1) + "<span class='unit'>°C</span>";
                    document.getElementById('hum').innerHTML = data.Env_Hum.toFixed(1) + "<span class='unit'>%</span>";
                    
                    // Rain
                    const rainEl = document.getElementById('rain');
                    rainEl.innerText = data.Rain_Status;
                    rainEl.style.color = data.Rain_Status === "RAIN" ? "var(--warning)" : "var(--success)";
                    document.getElementById('rain_val').innerText = "VAL: " + data.Rain_Val;

                    // RC & Motors
                    document.getElementById('rc1').innerText = data.RC1 || "--";
                    document.getElementById('rc3').innerText = data.RC3 || "--";
                    document.getElementById('out1').innerText = data.Out1 || "--";
                    document.getElementById('out3').innerText = data.Out3 || "--";
                    document.getElementById('rc_mode').innerText = "MODE: " + data.Mode;

                    // Mode Color
                    const modeEl = document.getElementById('mode');
                    modeEl.style.color = data.Mode === "DISCONNECTED" ? "var(--danger)" : "var(--success)";
                });
        }
        setInterval(updateStats, 1000);

        // Dinamik IP ve IMG Yükleyici
        window.onload = function() {
            var host = window.location.hostname;
            console.log("Detected Host: " + host);
            var rand = Math.random();
            document.getElementById('cam_img').src = "http://" + host + ":5000/?t=" + rand;
            document.getElementById('map_img').src = "http://" + host + ":5001/?t=" + rand;
        }
    </script>
</head>
<body>
    <header>
        <div class="header-left">
            <h1><span class="status-dot bg-green"></span> CELEBILER USV</h1>
            <div class="divider"></div>
            <div id="ts" style="font-family: var(--font-mono); font-size: 0.85rem; color: var(--text-secondary);">--:--:--</div>
        </div>
        
        <div class="header-right">
            <!-- RPI METRICS -->
            <div class="sys-metric" title="CPU Load">
                <span class="metric-icon">CPU</span>
                <span id="sys_cpu">--%</span>
            </div>
            <div class="sys-metric" title="RAM Usage">
                <span class="metric-icon">RAM</span>
                <span id="sys_ram">--%</span>
            </div>
            <div class="sys-metric" title="SoC Temp">
                <span class="metric-icon">TMP</span>
                <span id="sys_temp">--°C</span>
            </div>
        </div>
    </header>

    <div class="dashboard-grid">
        <!-- KAMERA -->
        <div class="card">
            <div class="card-header">
                <span>FRONT VISION</span>
                <span class="status-dot bg-green"></span>
            </div>
            <div class="video-container">
                <img id="cam_img" class="stream-img" alt="Camera Feed Loading..." />
            </div>
        </div>

        <!-- LIDAR -->
        <div class="card">
            <div class="card-header">
                <span>LIDAR SLAM MAP</span>
                <span class="status-dot bg-green"></span>
            </div>
            <div class="video-container">
                <img id="map_img" class="stream-img" alt="Map Feed Loading..." />
            </div>
        </div>

        <!-- ISTATISTIKLER (5 Kolon) -->
        <div class="stats-area">
            <!-- 1. BATTERY & MODE -->
            <div class="stat-card">
                <div class="stat-label">SYSTEM STATUS</div>
                <div>
                    <div class="stat-value" id="bat">-- V</div>
                    <div class="sub-val" id="mode">--</div>
                </div>
            </div>

            <!-- 2. NAVIGATION -->
            <div class="stat-card">
                <div class="stat-label">SPEED / HEADING</div>
                <div>
                    <div class="stat-value"><span id="spd">--</span> <span class="unit">M/S</span></div>
                    <div class="sub-val">HDG: <span id="hdg">--</span>°</div>
                </div>
            </div>

            <!-- 3. GPS -->
            <div class="stat-card">
                <div class="stat-label">GPS COORDINATES</div>
                <div>
                    <div class="stat-value" id="lat" style="font-size: 1.2rem; margin-bottom: 2px;">--</div>
                    <div class="stat-value" id="lon" style="font-size: 1.2rem; opacity: 0.7;">--</div>
                </div>
            </div>

            <!-- 4. ENVIRONMENT (STM32) -->
            <div class="stat-card">
                <div class="stat-label">ATMOSPHERE</div>
                <div>
                    <div class="stat-value" id="temp">--°C</div>
                    <div class="sub-val">HUMIDITY: <span id="hum">--%</span></div>
                </div>
            </div>

            <!-- 5. RC / MOTORS -->
            <div class="stat-card">
                <div class="stat-label">CONTROLLER (RC)</div>
                <div>
                    <div style="display:flex; justify-content:space-between; font-size:0.8rem; color:var(--text-secondary);">
                        <span>CH1: <strong id="rc1" style="color:var(--text-primary)">--</strong></span>
                        <span>CH3: <strong id="rc3" style="color:var(--text-primary)">--</strong></span>
                    </div>
                    <div style="display:flex; justify-content:space-between; font-size:0.8rem; color:var(--text-secondary);">
                        <span>MOT1: <strong id="out1" style="color:var(--accent)">--</strong></span>
                        <span>MOT3: <strong id="out3" style="color:var(--accent)">--</strong></span>
                    </div>
                    <div class="sub-val" id="rc_mode" style="margin-top:5px; font-size:0.7rem; text-align:right;">--</div>
                </div>
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
        self.pixhawk = None
        self.stm32 = None # STM32 Bağlantısı
        self.running = True
        self.lock = threading.Lock()
        
        self.sim_lat = 38.4192
        self.sim_lon = 27.1287
        
        # Log klasörü kontrolü
        if not os.path.exists("/root/workspace/logs"):
            os.makedirs("/root/workspace/logs")

    def scan_ports(self):
        """Tüm portları tara ve cihazları ayırt et"""
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        print(f"🔍 Taranan Portlar: {ports}")
        
        found_pix = None
        found_stm = None
        
        # 1. Tur: STM32 Bul (JSON verisinden tanı)
        for port in ports:
            if found_stm: break
            try:
                # STM32 Genelde 115200 baud ile konuşur
                s = serial.Serial(port, BAUD_RATE_STM32, timeout=2)
                time.sleep(1.5) # Reset sonrası bekleme
                
                # 3 satır oku, JSON var mı bak
                for _ in range(3):
                    line = s.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("{") and "temp" in line:
                        print(f"✅ STM32 Sensör Kartı Bulundu: {port}")
                        found_stm = s
                        break
                
                if not found_stm: s.close()
            except: pass

        # 2. Tur: Pixhawk Bul (MAVLink heartbeat)
        for port in ports:
            # STM32 olan portu atla
            if found_stm and found_stm.port == port: continue
            
            try:
                master = mavutil.mavlink_connection(port, baud=BAUD_RATE_PIXHAWK)
                if master.wait_heartbeat(timeout=1):
                    print(f"✅ Pixhawk Bulundu: {port}")
                    found_pix = master
                    break
                else:
                    master.close()
            except: pass
            
        return found_pix, found_stm

    def update_simulation(self):
        """Donanım yoksa rastgele veriler üret"""
        global telemetry_data
        with self.lock:
            # GPS Drift
            self.sim_lat += random.uniform(-0.0001, 0.0001)
            self.sim_lon += random.uniform(-0.0001, 0.0001)
            
            telemetry_data["Timestamp"] = time.strftime("%H:%M:%S")
            telemetry_data["Lat"] = self.sim_lat
            telemetry_data["Lon"] = self.sim_lon
            telemetry_data["Heading"] = (telemetry_data["Heading"] + 1) % 360
            telemetry_data["Battery"] = 12.0 + random.uniform(0, 0.5)
            telemetry_data["Speed"] = random.uniform(0, 3.0)
            telemetry_data["Mode"] = "SIMULATION"
            
            # STM32 Simülasyonu
            telemetry_data["STM_Date"] = time.strftime("%H:%M:%S")
            telemetry_data["Env_Temp"] = 24.5 + random.uniform(-0.5, 0.5)
            telemetry_data["Env_Hum"] = 45.0 + random.uniform(-2, 2)
            telemetry_data["Rain_Val"] = int(random.uniform(4000, 4095))
            telemetry_data["Rain_Status"] = "DRY"
            
            # RPI Simülasyonu (Docker içinde /sys okuyamezsa diye)
            telemetry_data["Sys_CPU"] = int(random.uniform(10, 30))
            telemetry_data["Sys_RAM"] = int(random.uniform(40, 60))
            telemetry_data["Sys_Temp"] = int(random.uniform(45, 55))

    def get_system_metrics(self):
        """Raspberry Pi Sistem Verilerini Okur"""
        cpu_usage = 0
        ram_usage = 0
        temp = 0
        
        try:
            # CPU Load (Basic) - psutil yoksa loadavg kullan
            # loadavg 1 dakikalık ortalamayı CPU sayısına bölebiliriz ama 
            # basitçe 100 ile çarpıp normalize edelim (RPi 4 çekirdek).
            load1, _, _ = os.getloadavg()
            cpu_usage = int((load1 / 4.0) * 100)
            if cpu_usage > 100: cpu_usage = 100
            
            # RAM Usage (Free komutu parsing veya /proc/meminfo)
            # /proc/meminfo okuyalım
            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()
                mem_total = int(lines[0].split()[1])
                mem_available = int(lines[2].split()[1])
                ram_usage = int(100 * (1 - (mem_available / mem_total)))
                
            # CPU Temp
            # RPi genelde bu dosyada tutar
            if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    temp = int(int(f.read()) / 1000)
            else:
                temp = 0 # Okuyamazsa 0
                
        except: pass
        
        return cpu_usage, ram_usage, temp

    def read_stm32(self):
        """STM32'den JSON okuyan thread"""
        while self.running:
            # Sistem metriklerini güncelle (Her turda yapalım, hızlıdır)
            c, r, t = self.get_system_metrics()
            with self.lock:
                telemetry_data["Sys_CPU"] = c
                telemetry_data["Sys_RAM"] = r
                telemetry_data["Sys_Temp"] = t

            if not self.stm32:
                time.sleep(1); continue
                
            try:
                if self.stm32.in_waiting > 0:
                    line = self.stm32.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("{") and line.endswith("}"):
                        data = json.loads(line)
                        with self.lock:
                            telemetry_data["STM_Date"] = data.get("tarih", "--:--:--")
                            telemetry_data["Env_Temp"] = float(data.get("temp", 0))
                            telemetry_data["Env_Hum"] = float(data.get("hum", 0))
                            telemetry_data["Rain_Val"] = int(data.get("rain", 4095))
                            
                            # Yağmur Durumu (Sensör mantığı: Düşük değer = Islak)
                            # Genelde analog okuma 4095 (kuru) -> 0 (ıslak)
                            if telemetry_data["Rain_Val"] < 3000:
                                telemetry_data["Rain_Status"] = "RAIN"
                            else:
                                telemetry_data["Rain_Status"] = "DRY"
            except Exception as e:
                # print(f"STM32 Okuma Hatası: {e}") 
                pass

    def read_pixhawk(self):
        """Pixhawk'tan veri okuyan thread"""
        global SIMULATION_MODE
        
        while self.running:
            # Bağlantı Yönetimi
            if not self.pixhawk or not self.stm32:
                self.pixhawk, self.stm32 = self.scan_ports()
                
                # Eğer hala cihaz yoksa simülasyona düş
                if not self.pixhawk and not self.stm32:
                    if not SIMULATION_MODE:
                        print("⚠️ Cihazlar bulunamadı -> Simülasyon Modu")
                        SIMULATION_MODE = True
                    
                    self.update_simulation()
                    time.sleep(1)
                    continue
                else:
                    SIMULATION_MODE = False
                    
            # PIXHAWK OKUMA
            if self.pixhawk:
                try:
                    msg = self.pixhawk.recv_match(blocking=True, timeout=1.0)
                    if msg:
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
                            elif msg.get_type() == 'ATTITUDE':
                                telemetry_data['Roll'] = msg.roll * 57.2958
                                telemetry_data['Pitch'] = msg.pitch * 57.2958
                            elif msg.get_type() == 'RC_CHANNELS':
                                # Kumanda Girişleri (Stickler)
                                telemetry_data['RC1'] = msg.chan1_raw
                                telemetry_data['RC2'] = msg.chan2_raw
                                telemetry_data['RC3'] = msg.chan3_raw
                                telemetry_data['RC4'] = msg.chan4_raw
                            elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                                # Motor Çıkışları (ESC PWM)
                                telemetry_data['Out1'] = msg.servo1_raw
                                telemetry_data['Out3'] = msg.servo3_raw
                except:
                    self.pixhawk = None # Hata varsa bağlantıyı kopar ve tekrar tara

    def request_data_stream(self, master):
        """Veri akışını başlat (Özellikle RC kanalları için gerekli)"""
        if not master: return
        # Tüm akışları iste (RC_CHANNELS dahil)
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1 # 4 Hz
        )

    def scan_ports(self):
        """Tüm portları tara ve cihazları ayırt et"""
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        print(f"🔍 Taranan Portlar: {ports}")
        
        found_pix = None
        found_stm = None
        
        # 1. Tur: STM32 Bul (JSON verisinden tanı)
        # ... (STM32 kodu aynı) ...

        # 2. Tur: Pixhawk Bul (MAVLink heartbeat)
        for port in ports:
            if found_stm and found_stm.port == port: continue
            
            try:
                master = mavutil.mavlink_connection(port, baud=BAUD_RATE_PIXHAWK)
                if master.wait_heartbeat(timeout=1):
                    print(f"✅ Pixhawk Bulundu: {port}")
                    # VERİ AKIŞINI BAŞLAT
                    self.request_data_stream(master)
                    found_pix = master
                    break
                else:
                    master.close()
            except: pass
            
        return found_pix, found_stm
        # CSV Başlat
        if not os.path.isfile(CSV_FILE):
             pd.DataFrame(columns=COLUMNS).to_csv(CSV_FILE, index=False)
             
        # Threadleri Başlat
        t1 = threading.Thread(target=self.read_pixhawk, daemon=True) # Pixhawk + Auto Scan + Sim
        t2 = threading.Thread(target=self.read_stm32, daemon=True)   # STM32 Sadece okuma
        t1.start()
        t2.start()
        
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

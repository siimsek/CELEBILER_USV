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
import math
from flask import Flask, jsonify, render_template_string

# --- YAPILANDIRMA VE SABİTLER ---
BAUD_RATE_PIXHAWK = 115200
BAUD_RATES_STM32 = [9600, 115200] # Otomatik denenir
WEB_PORT = 8080
LOG_DIR = "/root/workspace/logs"
CSV_FILE = f"{LOG_DIR}/telemetri_verisi.csv"

# --- GLOBAL DURUM ---
telemetry_data = {
    "Timestamp": "--", "Lat": 0, "Lon": 0, "Heading": 0,
    "Battery": 0, "Mode": "DISCONNECTED", "Speed": 0, "Roll": 0, "Pitch": 0,
    "STM_Date": "--:--:--", "Env_Temp": 0, "Env_Hum": 0, "Rain_Val": 0, "Rain_Status": "DRY",
    "Sys_CPU": 0, "Sys_RAM": 0, "Sys_Temp": 0,
    "RC1": 0, "RC2": 0, "RC3": 0, "RC4": 0,
    "Out1": 0, "Out3": 0
}
COLUMNS = list(telemetry_data.keys())
SIMULATION_MODE = False

# Flask Uygulaması
app = Flask(__name__)

# --- DASHBOARD ARAYÜZÜ (HTML/CSS/JS) ---
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
                    // Stick Animasyonu (1000-2000 aralığını -24px ile +24px arasına map et)
                    const mapStick = (val) => {
                         if (!val) return 0;
                         let norm = (val - 1500) / 500.0; // -1 to 1
                         if (norm > 1) norm = 1; 
                         if (norm < -1) norm = -1;
                         return norm * 24; // Yarıçap
                    };

                    // Sol Stick (CH4=X, CH3=Y)
                    const s1_x = mapStick(data.RC4); 
                    const s1_y = -mapStick(data.RC3); // Y ekseni ters
                    document.getElementById('stick_left').style.transform = `translate(${s1_x}px, ${s1_y}px)`;
                    
                    // Sağ Stick (CH1=X, CH2=Y)
                    const s2_x = mapStick(data.RC1);
                    const s2_y = -mapStick(data.RC2);
                    document.getElementById('stick_right').style.transform = `translate(${s2_x}px, ${s2_y}px)`;

                    // Değerler
                    document.getElementById('rc1_val').innerText = data.RC1 || "--";
                    document.getElementById('rc3_val').innerText = data.RC3 || "--";

                    // Motor Barları (PWM 1000-2000 -> %0-%100)
                    const mapPWM = (val) => {
                        if (!val) return 0;
                        let p = (val - 1000) / 10.0;
                        if (p < 0) p = 0; if (p > 100) p = 100;
                        return p;
                    }
                    document.getElementById('out1').innerText = data.Out1 || "--";
                    document.getElementById('mot1_bar').style.width = mapPWM(data.Out1) + "%";
                    
                    document.getElementById('out3').innerText = data.Out3 || "--";
                    document.getElementById('mot3_bar').style.width = mapPWM(data.Out3) + "%";

                    // Mode Renkleri

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
                    <div class="sub-val" style="margin-top:2px;">RAIN: <span id="rain">--</span> <span id="rain_val" style="font-size:0.7em; opacity:0.7"></span></div>
                </div>
            </div>

            <!-- 5. RC / MOTORS (Görsel Simülasyon) -->
            <div class="stat-card" style="grid-column: span 1;">
                <div class="stat-label">CONTROLLER INPUT</div>
                <div style="display:flex; justify-content:space-around; align-items:center; padding:10px 0;">
                    <!-- Sol Stick (Throttle/Yaw) -->
                    <div style="position:relative; width:60px; height:60px; border:2px solid rgba(255,255,255,0.1); border-radius:50%; background:rgba(0,0,0,0.2);">
                        <div id="stick_left" style="position:absolute; width:12px; height:12px; background:var(--accent); border-radius:50%; top:24px; left:24px; transition:all 0.05s;"></div>
                        <span style="position:absolute; bottom:-20px; width:100%; text-align:center; font-size:0.7rem; color:var(--text-secondary);">MO3/4</span>
                    </div>

                    <!-- Sağ Stick (Pitch/Roll) -->
                    <div style="position:relative; width:60px; height:60px; border:2px solid rgba(255,255,255,0.1); border-radius:50%; background:rgba(0,0,0,0.2);">
                        <div id="stick_right" style="position:absolute; width:12px; height:12px; background:var(--accent); border-radius:50%; top:24px; left:24px; transition:all 0.05s;"></div>
                        <span style="position:absolute; bottom:-20px; width:100%; text-align:center; font-size:0.7rem; color:var(--text-secondary);">CH1/2</span>
                    </div>
                </div>
                <div style="text-align:center; margin-top:15px; font-size:0.7rem; color:var(--text-secondary);">
                    CH1: <span id="rc1_val" style="color:white">--</span> | CH3: <span id="rc3_val" style="color:white">--</span>
                </div>
            </div>

            <!-- MOTOR DURUMU -->
             <div class="stat-card" style="grid-column: span 1;">
                 <div class="stat-label">MOTORS (PWM)</div>
                 <div style="display:flex; flex-direction:column; gap:8px;">
                     <div style="display:flex; justify-content:space-between; align-items:center;">
                         <span style="font-size:0.8rem; color:var(--text-secondary);">MOT 1</span>
                         <div style="flex:1; height:6px; background:rgba(255,255,255,0.1); margin:0 10px; border-radius:3px; overflow:hidden;">
                             <div id="mot1_bar" style="width:0%; height:100%; background:var(--accent); transition:width 0.1s;"></div>
                         </div>
                         <strong id="out1" style="font-size:0.9rem;">--</strong>
                     </div>
                     <div style="display:flex; justify-content:space-between; align-items:center;">
                         <span style="font-size:0.8rem; color:var(--text-secondary);">MOT 3</span>
                         <div style="flex:1; height:6px; background:rgba(255,255,255,0.1); margin:0 10px; border-radius:3px; overflow:hidden;">
                             <div id="mot3_bar" style="width:0%; height:100%; background:var(--accent); transition:width 0.1s;"></div>
                         </div>
                         <strong id="out3" style="font-size:0.9rem;">--</strong>
                     </div>
                 </div>
             </div>
        </div>
    </div>
</body>
</html>
"""

def clean_port(port):
    """Belirtilen portu kullanan işlemleri temizler."""
    print(f"🧹 Port {port} temizleniyor...")
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")

class SmartTelemetry:
    def __init__(self):
        self.pixhawk = None
        self.stm32 = None
        self.pixhawk_port = None
        self.stm32_port = None
        
        self.running = True
        self.lock = threading.Lock()
        
        # Simülasyon Değişkenleri
        self.sim_lat = 38.4192
        self.sim_lon = 27.1287
        self.sim_heading = 0
        
        # Log Klasörü
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)
            
        # CSV init
        if not os.path.isfile(CSV_FILE):
             pd.DataFrame(columns=COLUMNS).to_csv(CSV_FILE, index=False)

    def start(self):
        """Tüm threadleri başlatır."""
        threads = [
            threading.Thread(target=self.read_pixhawk, daemon=True),
            threading.Thread(target=self.read_stm32, daemon=True),
            threading.Thread(target=self.connection_manager, daemon=True)
        ]
        
        for t in threads:
            t.start()
        
        print(f"🌍 WEB SERVER BAŞLATILIYOR: Port {WEB_PORT}")
        app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)

    # --- BAĞLANTI YÖNETİMİ ---
    def connection_manager(self):
        """Cihaz bağlantılarını periyodik olarak kontrol eder."""
        print("🕵️ [SYSTEM] Bağlantı Yöneticisi Devrede...")
        first_scan = True
        
        while self.running:
            if not self.pixhawk or not self.stm32:
                if not first_scan:
                    print("🔍 [SCAN] Eksik cihazlar taranıyor...")
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
            
            first_scan = False
            time.sleep(3)

    def scan_ports(self):
        """Boştaki portları tarar ve uygun cihazları eşleştirir."""
        all_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        active_ports = [self.pixhawk_port, self.stm32_port]
        active_ports = [p for p in active_ports if p] # None'ları temizle
        
        scan_list = [p for p in all_ports if p not in active_ports]
        if not scan_list and (not self.pixhawk or not self.stm32):
             return # Taranacak port yok

        print(f"🔍 [SCAN] Hedef Portlar: {scan_list}")

        # STM32 Tara
        if not self.stm32:
            for port in scan_list:
                if self._probe_stm32(port):
                    active_ports.append(port)
                    break # Bulduysam döngüyü kır (diğer cihazlara şans ver)

        # Pixhawk Tara
        if not self.pixhawk:
            for port in scan_list:
                if port in active_ports: continue
                if self._probe_pixhawk(port):
                    active_ports.append(port)
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
                    except: pass
                
                if detected: return True
                s.close()
            except: pass
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
                return True
            master.close()
        except: pass
        return False

    def _request_mavlink_streams(self, master):
        """Pixhawk'tan gerekli veri akışlarını ister."""
        if not master: return
        # Genel Veriler (4 Hz)
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
        )
        # RC Kanalları (5 Hz) - Özel İstek
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 5, 1
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
                    if line.startswith("{") and line.endswith("}"):
                        data = json.loads(line)
                        with self.lock:
                            telemetry_data["STM_Date"] = data.get("tarih", "--:--:--")
                            telemetry_data["Env_Temp"] = float(data.get("temp", 0))
                            telemetry_data["Env_Hum"] = float(data.get("hum", 0))
                            telemetry_data["Rain_Val"] = int(data.get("rain", 4095))
                            telemetry_data["Rain_Status"] = "RAIN" if telemetry_data["Rain_Val"] < 3000 else "DRY"
            except: pass

    def read_pixhawk(self):
        print("📡 [MAV] Pixhawk Dinleme Servisi Aktif")
        rc_logged = False
        
        while self.running:
            if not self.pixhawk:
                time.sleep(0.1); continue
                
            try:
                msg = self.pixhawk.recv_match(blocking=True, timeout=1.0)
                if msg:
                    self._process_mavlink_msg(msg, rc_logged)
                    if msg.get_type() == 'RC_CHANNELS' and not rc_logged:
                        if msg.chan3_raw > 0:
                            print(f"🎮 [RC] Sinyal Tespit Edildi: CH3={msg.chan3_raw}")
                            rc_logged = True
            except Exception as e:
                # print(f"❌ MAV Error: {e}")
                pass

    def _process_mavlink_msg(self, msg, rc_logged):
        """MAVLink mesajlarını işleyen yardımcı metot."""
        mtype = msg.get_type()
        
        with self.lock:
            telemetry_data["Timestamp"] = time.strftime("%H:%M:%S")
            
            if mtype == 'GLOBAL_POSITION_INT':
                telemetry_data['Lat'] = msg.lat / 1e7
                telemetry_data['Lon'] = msg.lon / 1e7
                telemetry_data['Heading'] = msg.hdg / 100.0
                
            elif mtype == 'RC_CHANNELS':
                telemetry_data['RC1'] = msg.chan1_raw
                telemetry_data['RC2'] = msg.chan2_raw
                telemetry_data['RC3'] = msg.chan3_raw
                telemetry_data['RC4'] = msg.chan4_raw
                
                # --- RC DEBUG (Kanal Tespiti) ---
                # Her 5 mesajda bir (yaklaşık saniyede 1) loga bas
                self.rc_debug_counter = getattr(self, 'rc_debug_counter', 0) + 1
                if self.rc_debug_counter % 5 == 0:
                    print(f"🎮 RC RAW: 1:{msg.chan1_raw} 2:{msg.chan2_raw} 3:{msg.chan3_raw} 4:{msg.chan4_raw} 5:{msg.chan5_raw} Mode:{telemetry_data.get('Mode')}")

                self._update_physics_sim(msg.chan1_raw, msg.chan3_raw)
                
            elif mtype == 'SYS_STATUS':
                telemetry_data['Battery'] = msg.voltage_battery / 1000.0
                
            elif mtype == 'HEARTBEAT':
                # --- HEARTBEAT DEBUG ---
                # Mod değişimini yakalamak için geçici log
                if getattr(self, 'last_mode_debug', None) != msg.custom_mode or time.time() % 5 < 0.1:
                    print(f"💓 [HEARTBEAT] Src: {msg.get_srcSystem()} Mode: {msg.custom_mode} Base: {msg.base_mode}")
                    self.last_mode_debug = msg.custom_mode

                # Sadece Pixhawk'tan gelen moda bak (Auto-Discovery)
                src_sys = msg.get_srcSystem()
                
                # Eğer henüz sistem ID'sini bilmiyorsak, ilk gelen OTOPİLOT mesajını sistemimiz kabul edelim
                if getattr(self, 'target_system_id', None) is None:
                    # MAV_TYPE_GENERIC=0 ... MAV_TYPE_HELICOPTER=4 arası genelde araçtır
                    if msg.type <= 20: 
                         print(f"🎯 [MAV] Hedef Sistem Tanımlandı: ID {src_sys} (Type: {msg.type})")
                         self.target_system_id = src_sys
                
                # Sadece hedef sistemden gelenleri işle
                if getattr(self, 'target_system_id', None) != src_sys:
                    return
                
                # ArduRover Mode Mapping
                custom_mode = msg.custom_mode
                base_mode = msg.base_mode
                
                # Rover Modları (ArduPilot Dokümantasyonundan)
                modes = {
                    0: 'MANUAL', 1: 'ACRO', 3: 'STEERING', 4: 'HOLD',
                    5: 'LOITER', 10: 'AUTO', 11: 'RTL', 12: 'SMART_RTL', 15: 'GUIDED'
                }
                
                mode_str = modes.get(custom_mode, f"Mode({custom_mode})")
                if not msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    mode_str += " (DISARMED)"
                    
                telemetry_data['Mode'] = mode_str
                
            elif mtype == 'VFR_HUD':
                telemetry_data['Speed'] = msg.groundspeed
                
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
        except: pass

    # --- SİSTEM & SİMÜLASYON ---
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
        except: pass

    def update_simulation(self):
        """Donanım yoksa rastgele veriler üretir."""
        with self.lock:
            # Rastgele Drift
            self.sim_lat += random.uniform(-0.00005, 0.00005)
            self.sim_lon += random.uniform(-0.00005, 0.00005)
            
            telemetry_data.update({
                "Timestamp": time.strftime("%H:%M:%S"),
                "Lat": self.sim_lat,
                "Lon": self.sim_lon,
                "Heading": (telemetry_data["Heading"] + 1) % 360,
                "Battery": 12.0 + random.uniform(0, 0.5),
                "Speed": random.uniform(0, 3.0),
                "Mode": "SIMULATION",
                # STM32 Mock
                "STM_Date": time.strftime("%H:%M:%S"),
                "Env_Temp": 24.5 + random.uniform(-0.5, 0.5),
                "Env_Hum": 45.0 + random.uniform(-2, 2),
                "Rain_Val": int(random.uniform(4000, 4095)),
                "Rain_Status": "DRY"
            })

# --- FLASK ROUTES ---
@app.route('/')
def index(): return render_template_string(HTML_PAGE)

@app.route('/api/data')
def get_data(): return jsonify(telemetry_data)

if __name__ == "__main__":
    clean_port(WEB_PORT)
    st = SmartTelemetry()
    st.start()

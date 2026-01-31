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
                         if (Math.abs(norm) < 0.1) norm = 0; // Deadzone
                         if (norm > 1) norm = 1; 
                         if (norm < -1) norm = -1;
                         return norm * 24; // Yarıçap
                    };

                    // Sol Stick (CH4=X, CH3=Y) -> İleri/Geri (CH3) ve Dönüş (CH4)
                    const s1_x = mapStick(data.RC4); 
                    const s1_y = -mapStick(data.RC3); // Y ekseni ters (Yukarı = Düşük Y)
                    document.getElementById('stick_left').style.transform = `translate(${s1_x}px, ${s1_y}px)`;
                    
                    // Sağ Stick (CH1=X, CH2=Y) -> Sağ/Sol (CH1) ve İleri/Geri (CH2)
                    const s2_x = mapStick(data.RC1);
                    const s2_y = -mapStick(data.RC2);
                    document.getElementById('stick_right').style.transform = `translate(${s2_x}px, ${s2_y}px)`;

                    // Değerler
                    // document.getElementById('rc1_val').innerText = data.RC1 || "--";
                    // document.getElementById('rc3_val').innerText = data.RC3 || "--";
                    
                    // Gear Göstergesi
                    const gearEl = document.getElementById('gear_disp');
                    if (gearEl && data.Gear) {
                        gearEl.innerText = data.Gear;
                        if (data.Gear === "FORWARD") gearEl.style.color = "var(--success)";
                        else if (data.Gear === "REVERSE") gearEl.style.color = "var(--danger)";
                        else gearEl.style.color = "var(--warning)";
                    }

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
                        <span style="position:absolute; bottom:-20px; width:100%; text-align:center; font-size:0.7rem; color:var(--text-secondary);">CRUISE</span>
                    </div>

                    <!-- Sağ Stick (Pitch/Roll) -->
                    <div style="position:relative; width:60px; height:60px; border:2px solid rgba(255,255,255,0.1); border-radius:50%; background:rgba(0,0,0,0.2);">
                        <div id="stick_right" style="position:absolute; width:12px; height:12px; background:var(--accent); border-radius:50%; top:24px; left:24px; transition:all 0.05s;"></div>
                        <span style="position:absolute; bottom:-20px; width:100%; text-align:center; font-size:0.7rem; color:var(--text-secondary);">STEER</span>
                    </div>
                </div>
                <div style="text-align:center; margin-top:15px; font-size:0.7rem; color:var(--text-secondary);">
                    GEAR: <strong id="gear_disp">--</strong>
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

        # Motor Kontrolcüsünü Başlat
        self.motor_ctrl = MotorController(self)

    def start(self):
        """Tüm threadleri başlatır."""
        threads = [
            threading.Thread(target=self.read_pixhawk, daemon=True),
            threading.Thread(target=self.read_stm32, daemon=True),
            threading.Thread(target=self.connection_manager, daemon=True)
            # Motor thread'i kendi içinde başlıyor zaten
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
                
                # Periyodik olarak veri akışını tazele (Her 5 saniyede bir)
                # Bu, mod simgesinin donmasını veya güncelleme almamasını engeller
                if self.pixhawk and time.time() % 5 < 1.0:
                     try:
                         self._request_mavlink_streams(self.pixhawk)
                     except: pass
            
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
                
                # Motor Kontrolcüsüne Veri Gönder
                # CH1: Direksiyon, CH3: Gaz Artır/Azalt, CH6: Vites
                rc6 = msg.chan6_raw
                self.motor_ctrl.update_inputs(msg.chan1_raw, msg.chan3_raw, rc6)
                
                # --- RC DEBUG (Kanal Tespiti) ---
                # Her 5 mesajda bir (yaklaşık saniyede 1) loga bas
                self.rc_debug_counter = getattr(self, 'rc_debug_counter', 0) + 1
                if self.rc_debug_counter % 5 == 0:
                    gear_stat = telemetry_data.get('Gear', 'N')
                    print(f"🎮 RC: 1:{msg.chan1_raw} 3:{msg.chan3_raw} 6:{rc6} -> Gear:{gear_stat}")

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
                    else:
                        # GCS'i sessizce geç veya çok nadir bas
                        pass
                
                # 2. FİLTRE (Sadece hedef sistem)
                if getattr(self, 'target_system_id', None) != src_sys:
                    return

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
        
        # Ayarlar
        self.PWM_NEUTRAL = 1500
        self.PWM_DEADZONE = 50     # 1450-1550 arası stick hareketsiz sayılır
        
        self.MAX_FWD = 1900
        self.MAX_REV = 1100
        
        self.RAMP_STEP = 2.0       # Her döngüde (50ms) artış miktarı (Yumuşak kalkış)
        self.CRUISE_STEP = 2.0     # Stick basılıyken hedef hızın artış hızı
        
        # Thread
        self.thread = threading.Thread(target=self.control_loop, daemon=True)
        self.thread.start()

    def update_inputs(self, rc1, rc3, rc6):
        """RC verilerini güncelle"""
        if rc1: self.input_steer = rc1
        if rc3: self.input_throttle = rc3
        if rc6: self.input_gear = rc6

    def control_loop(self):
        print("⚙️ [MOTOR] Cruise Control & Soft-Start Devrede")
        while self.active:
            time.sleep(0.05) # 20Hz Döngü
            
            # --- 1. VİTES MANTIĞI (CH6) ---
            # Switch Aşağı (<1300): GERİ
            # Switch Orta (1300-1700): BOŞ
            # Switch Yukarı (>1700): İLERİ
            
            new_gear = "NEUTRAL"
            if self.input_gear > 1700: new_gear = "FORWARD"
            elif self.input_gear < 1300: new_gear = "REVERSE"
            
            # Güvenli Geçiş Kontrolü (Hareket halindeyken ters vitese takma)
            if new_gear != self.gear:
                if self.gear == "FORWARD" and new_gear == "REVERSE":
                    # Önce durmalı
                    if abs(self.current_pwm - 1500) > 10: new_gear = "NEUTRAL"
                elif self.gear == "REVERSE" and new_gear == "FORWARD":
                    if abs(self.current_pwm - 1500) > 10: new_gear = "NEUTRAL"
                
                self.gear = new_gear
                
                # Vites Boşa alındıysa hedefi sıfırla
                if self.gear == "NEUTRAL":
                    self.target_pwm = 1500

            # --- 2. CRUISE CONTROL GİRDİSİ (SOL STICK - CH3) ---
            # Kullanıcı İsteği: Stick ne kadar itilirse o kadar hızlı ivmelensin
            # Merkez (1500) = Hız değişimi yok.
            
            throttle_diff = self.input_throttle - 1500
            
            if abs(throttle_diff) > self.PWM_DEADZONE:
                # Normalizasyon: 0.0 ile 1.0 arası güç çarpanı
                # 500 = Stick yarıçapı
                power_factor = (abs(throttle_diff) - self.PWM_DEADZONE) / (500.0 - self.PWM_DEADZONE)
                
                # CRUISE_STEP artık dinamik: Max 5 PWM/döngü
                step_val = self.CRUISE_STEP * power_factor * 2.0 
                
                if throttle_diff > 0:
                    # Hızlan (İleri viteste Max'a, Geri viteste MaxRev'e doğru değil, HIZ büyüklüğünü artır)
                    if self.gear == "FORWARD":
                        self.target_pwm = min(self.target_pwm + step_val, self.MAX_FWD)
                    elif self.gear == "REVERSE":
                        # Geri viteste hedef azaldıkça hız artar (1500 -> 1100)
                        self.target_pwm = max(self.target_pwm - step_val, self.MAX_REV)
                        
                else: 
                    # Yavaşla (Frene bas veya elini çek)
                    # 1500'e doğru yaklaş
                    if self.target_pwm > 1500:
                        self.target_pwm = max(self.target_pwm - step_val, 1500)
                    elif self.target_pwm < 1500:
                        self.target_pwm = min(self.target_pwm + step_val, 1500)

            # --- 3. RAMPING (YUMUŞAK GEÇİŞ) ---
            # current_pwm'i target_pwm'e yavaşça yaklaştır
            diff = self.target_pwm - self.current_pwm
            if abs(diff) < self.RAMP_STEP:
                self.current_pwm = self.target_pwm
            else:
                self.current_pwm += self.RAMP_STEP if diff > 0 else -self.RAMP_STEP

            # --- DEBUG LOG (Sadece değişim varsa) ---
            # CH6 Sorunu için özel takip
            if abs(self.input_gear - 1500) > 100:
                 # Vites eylemi var
                 pass

            # --- 4. DIFFERENTIAL STEERING (SAĞ STICK - CH1) ---
            # Dönüş faktörü (-1.0 ile 1.0 arası)
            steering = (self.input_steer - 1500) / 500.0
            
            # --- 5. ÇIKISH (MAVLINK OVERRIDE) ---
            
            if self.parent.pixhawk:
                try:
                    CMD_THROTTLE = int(self.current_pwm)
                    CMD_STEERING = int(self.input_steer)
                    
                    if self.gear == "NEUTRAL": CMD_THROTTLE = 1500
                    
                    # Kanal listesi (1-8 arası, diğerleri ignore)
                    rc_override = [65535]*8
                    rc_override[0] = CMD_STEERING     # CH1: Roll
                    rc_override[2] = CMD_THROTTLE     # CH3: Throttle
                    
                    self.parent.pixhawk.mav.rc_channels_override_send(
                        self.parent.target_system_id if self.parent.target_system_id else 1,
                        self.parent.pixhawk.target_component,
                        *rc_override
                    )
                    
                    with self.parent.lock:
                        telemetry_data["Gear"] = self.gear
                        # Debug: Hedef hızı UI'da görmek için Out1/3'e yazalım (Geçici)
                        telemetry_data["Out1"] = int(self.input_gear) # DEBUG: CH6 RAW Değerini buraya yazalım
                        telemetry_data["Out3"] = int(self.target_pwm)
                        
                except Exception as e:
                    pass

    # --- SİSTEM & SİMÜLASYON ---

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
                "Gear": "NEUTRAL",
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


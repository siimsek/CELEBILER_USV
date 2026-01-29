import time
import pandas as pd
from pymavlink import mavutil
import sys
import threading
import os
import glob
import serial
import json

# --- AYARLAR ---
BAUD_RATE_PIXHAWK = 115200
BAUD_RATE_STM32 = 115200  # STM32 kodunla aynı olmalı
CSV_FILE = "/root/workspace/telemetri_verisi.csv"

# Veri Başlıkları
COLUMNS = [
    "Timestamp", "Lat", "Lon", "Speed_m_s", "Roll", "Pitch", "Heading", "Mode",
    "STM_Date", "Env_Temp", "Env_Hum", "Rain_Val"
]

class SmartTelemetry:
    def __init__(self):
        self.data = {col: 0 for col in COLUMNS}
        self.data.update({
            "Mode": "DISCONNECTED", "STM_Date": "--:--:--", 
            "Env_Temp": 0.0, "Env_Hum": 0.0, "Rain_Val": 0
        })
        self.running = True
        self.pixhawk = None
        self.stm32 = None
        self.record_count = 0
        self.lock = threading.Lock() # Veri çakışmasını önlemek için kilit

    def scan_ports(self):
        """Sistemdeki tüm USB/ACM portlarını bulur ve kimlik tespiti yapar."""
        potential_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        print(f"\n🔍 Taranan Portlar: {potential_ports}")
        
        found_pix = None
        found_stm = None

        # 1. Tur: STM32 Avı (JSON Formatı Arayan Dedektif)
        for port in potential_ports:
            if found_stm: break
            try:
                print(f"   Testing STM32 @ {port}...", end=" ")
                # Timeout kısa tutulur ki hızlı geçsin
                s = serial.Serial(port, BAUD_RATE_STM32, timeout=2)
                time.sleep(1.5) # Arduino reset payı
                
                # 5 denemede geçerli JSON yakala
                for _ in range(5):
                    line = s.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("{") and "temp" in line:
                        print("✅ BULUNDU! (Sensör Kartı)")
                        found_stm = s
                        break
                
                if not found_stm: s.close()
                else: print("") # Yeni satır
            except: 
                print("❌")

        # 2. Tur: Pixhawk Avı (MAVLink Sinyali Arayan Dedektif)
        for port in potential_ports:
            # STM32 bulduğumuz portu elleme
            if found_stm and found_stm.port == port: continue
            
            try:
                print(f"   Testing Pixhawk @ {port}...", end=" ")
                master = mavutil.mavlink_connection(port, baud=BAUD_RATE_PIXHAWK)
                # Heartbeat bekle
                msg = master.wait_heartbeat(timeout=1)
                if msg:
                    print("✅ BULUNDU! (Uçuş Kontrolcü)")
                    found_pix = master
                    break
                else:
                    master.close()
                    print("❌")
            except: 
                print("❌")

        return found_pix, found_stm

    def connect_system(self):
        """Bağlantı koparsa veya başlangıçta çalışır."""
        while self.running:
            if self.pixhawk and self.stm32:
                return # İkisi de bağlıysa çık

            self.pixhawk, self.stm32 = self.scan_ports()
            
            if not self.pixhawk or not self.stm32:
                print("⚠️ Eksik cihaz var! 3 saniye sonra tekrar taranacak...")
                time.sleep(3)
            else:
                # Pixhawk veri akışını başlat
                self.pixhawk.mav.request_data_stream_send(
                    self.pixhawk.target_system, self.pixhawk.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
                print("🚀 SİSTEM HAZIR! Veri akışı başlıyor...")
                return

    def read_pixhawk(self):
        while self.running:
            if not self.pixhawk: 
                time.sleep(1); continue
            try:
                msg = self.pixhawk.recv_match(blocking=True, timeout=1.0)
                if not msg: continue
                
                with self.lock: # Veri yazarken kilitle
                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        self.data["Lat"] = msg.lat / 1e7
                        self.data["Lon"] = msg.lon / 1e7
                        self.data["Heading"] = msg.hdg / 100.0
                    elif msg.get_type() == 'VFR_HUD':
                        self.data["Speed_m_s"] = msg.groundspeed
                    elif msg.get_type() == 'ATTITUDE':
                        self.data["Roll"] = msg.roll * 57.2958
                        self.data["Pitch"] = msg.pitch * 57.2958
                    elif msg.get_type() == 'HEARTBEAT':
                        self.data["Mode"] = mavutil.mode_string_v10(msg)
            except Exception:
                # Bağlantı koptu mu?
                pass

    def read_stm32(self):
        while self.running:
            if not self.stm32: 
                time.sleep(1); continue
            try:
                if self.stm32.in_waiting > 0:
                    line = self.stm32.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("{") and line.endswith("}"):
                        try:
                            sensor_data = json.loads(line)
                            with self.lock:
                                self.data["STM_Date"] = sensor_data.get("tarih", "N/A")
                                self.data["Env_Temp"] = float(sensor_data.get("temp", 0.0))
                                self.data["Env_Hum"]  = float(sensor_data.get("hum", 0.0))
                                self.data["Rain_Val"] = int(sensor_data.get("rain", 0))
                        except: pass
            except Exception:
                # Bağlantı hatası durumunda pas geç, ana döngü yeniden bağlar
                pass

    def display_dashboard(self):
        # CSV Başlat
        df = pd.DataFrame(columns=COLUMNS)
        df.to_csv(CSV_FILE, index=False)
        
        while self.running:
            self.data["Timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
            
            # Ekranı Temizle ve Yaz
            os.system('clear')
            
            rain_val = self.data['Rain_Val']
            rain_status = "KURU ☀️" if rain_val > 3000 else "YAGISLI 🌧️"
            
            print(f"========== EGE İDA OTONOM SİSTEMİ ==========")
            print(f"📡 DURUM: {'BAĞLI' if (self.pixhawk and self.stm32) else 'ARANIYOR...'}")
            print(f"⏱️  Zaman: {self.data['Timestamp']} | Kayıt: {self.record_count}")
            print("-" * 44)
            print(f"🚁 PIXHAWK TELEMETRİ")
            print(f"   Konum    : {self.data['Lat']:.6f}, {self.data['Lon']:.6f}")
            print(f"   Hız/Mod  : {self.data['Speed_m_s']:.1f} m/s  [{self.data['Mode']}]")
            print(f"   Yönelim  : Head:{self.data['Heading']:.0f}° Roll:{self.data['Roll']:.1f}°")
            print("-" * 44)
            print(f"🌡️ STM32 SENSÖR KARTI")
            print(f"   Ortam    : {self.data['Env_Temp']}°C | {self.data['Env_Hum']}% Nem")
            print(f"   Yağmur   : {rain_status} ({rain_val})")
            print("-" * 44)
            print("Çıkış için CTRL+C yapın. (Otomatik Upload Aktif)")

            # CSV Kayıt
            with self.lock:
                df_new = pd.DataFrame([self.data])
            df_new.to_csv(CSV_FILE, mode='a', header=False, index=False)
            
            self.record_count += 1
            time.sleep(0.5)

    def start(self):
        print("Sistem başlatılıyor...")
        self.connect_system()
        
        # İş parçacıklarını başlat
        t1 = threading.Thread(target=self.read_pixhawk, daemon=True)
        t2 = threading.Thread(target=self.read_stm32, daemon=True)
        t3 = threading.Thread(target=self.display_dashboard, daemon=True)
        
        t1.start(); t2.start(); t3.start()
        
        try:
            while True: time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            print("\nVeriler kaydediliyor...")
            time.sleep(1)
            self.upload_csv()

    def upload_csv(self):
        if os.path.exists(CSV_FILE):
            print("☁️  up.sb'ye yükleniyor...")
            os.system(f"curl -s https://up.sb -T {CSV_FILE}")
            print("\n✅ Tamamlandı.")

if __name__ == "__main__":
    app = SmartTelemetry()
    app.start()

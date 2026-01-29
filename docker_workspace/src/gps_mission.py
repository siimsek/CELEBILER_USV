import time
import math
from pymavlink import mavutil

# --- AYARLAR ---
# Hedef Koordinat (Google Maps'ten yakın bir yer seçip buraya yaz)
TARGET_LAT = 41.123456  # Örnek Enlem
TARGET_LON = 29.123456  # Örnek Boylam

SPEED_PWM = 1650     # Temel Hız
REACH_DIST = 2.0     # Hedefe kaç metre kala duralım?
CONNECTION_STRING = '/dev/ttyACM1' 
BAUD_RATE = 115200

class AutonomousNavigator:
    def __init__(self):
        print(f"🔌 Bağlanılıyor: {CONNECTION_STRING}...")
        self.master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
        self.master.wait_heartbeat()
        print("✅ Pixhawk Bağlandı!")

    def get_location(self):
        """Mevcut Konumu ve Yönü Çeker"""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            heading = msg.hdg / 100.0
            return current_lat, current_lon, heading
        return None, None, None

    def get_distance_metres(self, lat1, lon1, lat2, lon2):
        """İki koordinat arası mesafeyi hesaplar (Haversine Formülü)"""
        dLat = math.radians(lat2 - lat1)
        dLon = math.radians(lon2 - lon1)
        a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(math.radians(lat1)) \
            * math.cos(math.radians(lat2)) * math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = 6371000 * c # Dünya yarıçapı
        return d

    def get_bearing(self, lat1, lon1, lat2, lon2):
        """Hedefin hangi açıda kaldığını hesaplar (0-360 derece)"""
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

    def set_motor(self, left, right):
        """Motorlara güç verir"""
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component,
            int(left), 0, int(right), 0, 0, 0, 0, 0)

    def start_mission(self):
        print("🌍 GPS Verisi Bekleniyor...")
        while True:
            curr_lat, curr_lon, curr_heading = self.get_location()
            if curr_lat is not None and curr_lat != 0:
                print(f"📍 Konum Bulundu: {curr_lat}, {curr_lon}")
                break
            print("⏳ GPS Aranıyor... (Dışarı çıkın!)")
            time.sleep(1)

        # Modu MANUAL'den GUIDED'a al
        print("🛠️  Mod: GUIDED")
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self.master.mode_mapping()['GUIDED'])
        
        print("💪 ARM Yapılıyor...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print(f"🚀 HEDEF: {TARGET_LAT}, {TARGET_LON}")

        try:
            while True:
                curr_lat, curr_lon, curr_heading = self.get_location()
                
                if not curr_lat: continue

                # 1. Hedefe Uzaklık ve Açı Hesapla
                dist = self.get_distance_metres(curr_lat, curr_lon, TARGET_LAT, TARGET_LON)
                target_bearing = self.get_bearing(curr_lat, curr_lon, TARGET_LAT, TARGET_LON)

                # 2. Hedefe Vardık mı?
                if dist < REACH_DIST:
                    print(f"🎉 HEDEFE ULAŞILDI! (Mesafe: {dist:.1f}m)")
                    self.set_motor(1500, 1500)
                    self.master.arducopter_disarm()
                    break

                # 3. Yön Hatasını Hesapla
                heading_error = target_bearing - curr_heading
                
                # -180 ile +180 arasına sıkıştır
                if heading_error > 180: heading_error -= 360
                if heading_error < -180: heading_error += 360

                # 4. P Kontrolcü (Düzeltme)
                correction = heading_error * 2.5 # Hassasiyet (Gain)
                
                left_motor = SPEED_PWM + correction
                right_motor = SPEED_PWM - correction

                # Limitler
                left_motor = max(1100, min(1900, left_motor))
                right_motor = max(1100, min(1900, right_motor))

                self.set_motor(left_motor, right_motor)

                print(f"Dist: {dist:.1f}m | Hdg: {curr_heading:.0f}° | Tgt: {target_bearing:.0f}° | Err: {heading_error:.0f}°")
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("🛑 GÖREV İPTAL!")
            self.set_motor(1500, 1500)
            self.master.arducopter_disarm()

if __name__ == "__main__":
    nav = AutonomousNavigator()
    nav.start_mission()

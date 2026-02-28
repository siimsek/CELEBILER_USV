"""
CELEBILER USV - Merkezi Otonom Durum Makinesi (State Machine)
=============================================================
Bu dosya, yarışmadaki 3 parkuru sırasıyla icra eden ana otonom pilottur.

Parkur-1: Engelsiz Nokta Takibi (GPS Waypoint Navigation)
Parkur-2: Engelli Nokta Takibi (GPS + Lidar + Kamera Obstacle Avoidance)
Parkur-3: Kamikaze Angajman  (GPS + Hedef Rengi ile Son Yaklaşma)

Mod Farkları:
  - TEST:  Parkur geçişleri MANUEL (operatör komutuyla), süre aşımı sadece uyarı
  - RACE:  Parkur geçişleri OTOMATİK (Şartname 4.2), harici komut kilidi (4.4)

Şartname Referansları:
  - 3.5: Tüm otonomi yazılımları İDA üzerinde çalışacak
  - 3.7: YKİ'ye görüntü aktarımı yasak — ama kamera İDA üzerinde engel tespiti için çalışır
  - 4.2: Parkurlar arası geçiş otomatik (yarışma modunda)
  - 4.4: Hareket başladıktan sonra komut verilemez (acil motor kesme hariç)
  - 6:   Telemetri CSV 1 Hz kayıt (telemetry.py tarafından yapılıyor)
"""

import time
import os
import math
import sys
import glob
import json
import threading
import random

# --- USV MODE (test | race) ---
USV_MODE = os.environ.get('USV_MODE', 'test')

# --- AYARLAR ---
BAUD_RATE = 115200
SPEED_PWM = 1650        # Temel ileri hız PWM
TURN_SPEED = 200        # Dönüş sertliği (PWM farkı)
REACH_DIST = 3.0        # Hedefe kaç metre kala "vardık" sayalım
OBSTACLE_DIST = 1.5     # Lidar engel eşiği (metre)
CONTROL_HZ = 10         # Ana kontrol döngüsü frekansı

# Görev noktaları dosyası (USB'den veya YKİ'den yüklenen)
MISSION_FILE = os.environ.get('MISSION_FILE', '/root/workspace/mission.json')

# IPC: Dashboard'dan gelen komutlar (telemetry.py dosyaları yazar)
CONTROL_DIR = '/root/workspace/control'
FLAG_START = f'{CONTROL_DIR}/start_mission.flag'
FLAG_STOP = f'{CONTROL_DIR}/emergency_stop.flag'
FLAG_NEXT = f'{CONTROL_DIR}/next_parkur.flag'
STATE_FILE = f'{CONTROL_DIR}/mission_state.json'


def clean_port(port):
    """Belirtilen portu kullanan işlemleri temizler."""
    print(f"🧹 Port {port} temizleniyor...")
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")


# ============================================================
# GPS NAVİGASYON YARDIMCILARI (gps_mission.py'den taşındı)
# ============================================================
def haversine_distance(lat1, lon1, lat2, lon2):
    """İki GPS koordinatı arasındaki mesafeyi metre cinsinden hesaplar."""
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = (math.sin(dLat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dLon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return 6371000 * c  # Dünya yarıçapı (metre)


def calculate_bearing(lat1, lon1, lat2, lon2):
    """Hedefin hangi açıda kaldığını hesaplar (0-360 derece)."""
    dLon = math.radians(lon2 - lon1)
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    y = math.sin(dLon) * math.cos(lat2_r)
    x = (math.cos(lat1_r) * math.sin(lat2_r) -
         math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dLon))
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


def normalize_heading_error(error):
    """Açı farkını -180 ile +180 arasına sıkıştırır (en kısa dönüş yolu)."""
    if error > 180:
        error -= 360
    if error < -180:
        error += 360
    return error


# ============================================================
# ANA DURUM MAKİNESİ
# ============================================================
class USVStateMachine:
    """
    Merkezi otonom pilot. Parkur-1, Parkur-2, Parkur-3 sırasıyla
    otomatik geçiş yapar (Şartname 4.2).
    """

    # Parkur durumları
    STATE_IDLE = 0
    STATE_PARKUR1 = 1
    STATE_PARKUR2 = 2
    STATE_PARKUR3 = 3
    STATE_COMPLETED = 4

    def __init__(self):
        print("=" * 60)
        print("  CELEBILER USV - OTONOM SİSTEM")
        print("=" * 60)
        print(f"📌 MOD: {'🏁 YARIŞMA' if USV_MODE == 'race' else '🔧 TEST'}")

        self.simulation_mode = False
        self.state = self.STATE_IDLE
        self.mission_active = False
        self.mission_start_time = 0

        # GPS Verileri
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0

        # Lidar Verileri
        self.obstacle_detected = False
        self.min_obstacle_distance = 99.0
        self.lidar_available = False

        # Görev Noktaları (Parkur bazlı)
        self.waypoints_p1 = []  # Parkur-1: Engelsiz nokta takibi
        self.waypoints_p2 = []  # Parkur-2: Engelli nokta takibi
        self.waypoints_p3 = []  # Parkur-3: Kamikaze hedef noktası
        self.target_color = ""  # Parkur-3: Angajman hedef rengi
        self._wp_target = "--"
        self._wp_info = "-- / --"

        # Pixhawk Bağlantısı
        self.master = None
        self._connect_pixhawk()

        # Lidar Bağlantısı (ROS 2 — opsiyonel)
        self._lidar_thread = None
        self._try_connect_lidar()

    def _write_state(self):
        """Dashboard için mission durumunu dosyaya yazar."""
        try:
            os.makedirs(CONTROL_DIR, exist_ok=True)
            with open(STATE_FILE, 'w') as f:
                json.dump({
                    'state': self.state,
                    'active': self.mission_active,
                    'start_time': self.mission_start_time,
                    'target': getattr(self, '_wp_target', '--'),
                    'wp_info': getattr(self, '_wp_info', '-- / --'),
                }, f)
        except Exception:
            pass

    # ----------------------------------------------------------
    # DONANIM BAĞLANTILARI
    # ----------------------------------------------------------
    def _find_pixhawk_port(self):
        """Otomatik port tarama (glob ile)."""
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        if not ports:
            return None
        # İlk bulunan portu dene
        for port in ports:
            try:
                from pymavlink import mavutil
                m = mavutil.mavlink_connection(port, baud=BAUD_RATE)
                if m.wait_heartbeat(timeout=2):
                    print(f"   ✅ Pixhawk bulundu: {port}")
                    return m
                m.close()
            except Exception:
                pass
        return None

    def _connect_pixhawk(self):
        """Pixhawk'a bağlanır. Bulamazsa simülasyon moduna geçer."""
        print("[MAV] Pixhawk aranıyor...")
        try:
            result = self._find_pixhawk_port()
            if result:
                self.master = result
                # Veri akışı iste
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    4,  # MAV_DATA_STREAM_ALL (pymavlink import edilemeyebilir)
                    4, 1)
                print("✅ Pixhawk Bağlandı!")
            else:
                raise Exception("Port bulunamadı")
        except Exception as e:
            print(f"❌ Pixhawk bağlantısı BAŞARISIZ: {e}")
            print("⚠️ DONANIM BULUNAMADI - SİMÜLASYON MODU AKTİF")
            self.simulation_mode = True
            self.master = None

    def _try_connect_lidar(self):
        """ROS 2 Lidar subscriber başlatmayı dener (opsiyonel)."""
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

            # ROS init (zaten init edilmişse geç)
            try:
                rclpy.init()
            except RuntimeError:
                pass

            class _LidarListener(Node):
                def __init__(self, parent):
                    super().__init__('usv_lidar_listener')
                    self.parent = parent
                    qos = QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=10)
                    self.create_subscription(LaserScan, '/scan', self.cb, qos)

                def cb(self, msg):
                    ranges = msg.ranges
                    count = len(ranges)
                    if count == 0:
                        return
                    # Ön sektör: ±30 derece
                    sector = int(count * (30 / 360))
                    front = list(ranges[-sector:]) + list(ranges[:sector])
                    valid = [r for r in front if 0.15 < r < 20.0]
                    if valid:
                        self.parent.min_obstacle_distance = min(valid)
                        self.parent.obstacle_detected = self.parent.min_obstacle_distance < OBSTACLE_DIST
                    else:
                        self.parent.min_obstacle_distance = 99.0
                        self.parent.obstacle_detected = False

            node = _LidarListener(self)

            def spin():
                try:
                    rclpy.spin(node)
                except Exception:
                    pass

            self._lidar_thread = threading.Thread(target=spin, daemon=True)
            self._lidar_thread.start()
            self.lidar_available = True
            print("✅ [ROS] Lidar dinleniyor...")

        except ImportError:
            print("⚠️ [ROS] rclpy bulunamadı — Lidar devre dışı (simülasyon)")
            self.lidar_available = False
        except Exception as e:
            print(f"⚠️ [ROS] Lidar hatası: {e} — devre dışı")
            self.lidar_available = False

    # ----------------------------------------------------------
    # GPS VERİ OKUMA
    # ----------------------------------------------------------
    def _update_gps(self):
        """Pixhawk'tan GPS verisini çeker. Simülasyonda sahte veri üretir."""
        if self.simulation_mode:
            # Simülasyonda sabit bir konumdan başla, yavaşça ilerle
            self.current_heading = (self.current_heading + 1) % 360
            return True

        if not self.master:
            return False

        try:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT',
                                         blocking=False)
            if msg:
                self.current_lat = msg.lat / 1e7
                self.current_lon = msg.lon / 1e7
                self.current_heading = msg.hdg / 100.0
                return True
        except Exception:
            pass
        return False

    # ----------------------------------------------------------
    # MOTOR KONTROLÜ
    # ----------------------------------------------------------
    def set_motor(self, left, right):
        """Motorlara PWM gönderir. Simülasyonda sadece log basar."""
        left = max(1100, min(1900, int(left)))
        right = max(1100, min(1900, int(right)))

        if self.simulation_mode:
            return

        if self.master:
            try:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    left, 0, right, 0, 0, 0, 0, 0)
            except Exception as e:
                print(f"❌ [MOTOR] Override hatası: {e}")

    def stop_motors(self):
        """Motorları durdurur ve DISARM yapar."""
        print("🛑 Motorlar Durduruluyor...")
        self.set_motor(1500, 1500)
        if not self.simulation_mode and self.master:
            try:
                self.master.arducopter_disarm()
            except Exception:
                pass

    # ----------------------------------------------------------
    # GÖREV YÜKLEME (Şartname 4.2: dd.ddddddd formatında)
    # ----------------------------------------------------------
    def load_mission(self, filepath=None):
        """
        Görev noktalarını JSON dosyasından yükler.
        Beklenen format:
        {
            "parkur1": [[lat, lon], [lat, lon], ...],
            "parkur2": [[lat, lon], [lat, lon], ...],
            "parkur3": [[lat, lon]],
            "target_color": "RED"
        }
        """
        fp = filepath or MISSION_FILE
        if os.path.exists(fp):
            try:
                with open(fp, 'r') as f:
                    data = json.load(f)
                self.waypoints_p1 = data.get('parkur1', [])
                self.waypoints_p2 = data.get('parkur2', [])
                self.waypoints_p3 = data.get('parkur3', [])
                self.target_color = data.get('target_color', '')
                print(f"✅ [GÖREV] Yüklendi: P1={len(self.waypoints_p1)} "
                      f"P2={len(self.waypoints_p2)} P3={len(self.waypoints_p3)} "
                      f"Hedef={self.target_color}")
                return True
            except Exception as e:
                print(f"❌ [GÖREV] Dosya okunamadı: {e}")
        else:
            print(f"⚠️ [GÖREV] Dosya bulunamadı: {fp}")

        # Simülasyon için örnek görev noktaları
        if self.simulation_mode:
            print("[GÖREV] Simülasyon görev noktaları oluşturuluyor...")
            base_lat, base_lon = 38.4192, 27.1287
            self.waypoints_p1 = [
                [base_lat + 0.0001, base_lon],
                [base_lat + 0.0002, base_lon + 0.0001],
                [base_lat + 0.0001, base_lon + 0.0002],
            ]
            self.waypoints_p2 = [
                [base_lat - 0.0001, base_lon + 0.0001],
                [base_lat - 0.0002, base_lon + 0.0002],
            ]
            self.waypoints_p3 = [
                [base_lat, base_lon + 0.0003],
            ]
            self.target_color = "RED"
            return True
        return False

    # ----------------------------------------------------------
    # GÖREV BAŞLATMA / DURDURMA
    # ----------------------------------------------------------
    def start_mission(self):
        """Görevi başlatır. RC bağlı değilse ARM yapmaz. ARM yapar ve Parkur-1'e geçer."""
        if self.mission_active:
            print("⚠️ Görev zaten aktif!")
            return

        # RC kontrolü: Kumanda bağlı değilse ARM yapma
        if not self.simulation_mode and self.master:
            if not self._check_rc_connected():
                print("❌ KUMANDA BAĞLI DEĞİL! Motorlar ARM edilmeyecek.")
                print("   Kumandayı bağlayıp sinyal gelene kadar bekleyin, sonra tekrar başlatın.")
                return
            print("✅ Kumanda sinyali alındı.")

        print("🚀 GÖREV BAŞLATILDI!")
        self.mission_active = True
        self.mission_start_time = time.time()
        self.state = self.STATE_PARKUR1
        self._write_state()

        if not self.simulation_mode and self.master:
            try:
                from pymavlink import mavutil
                print("🛠️  Mod: GUIDED")
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    self.master.mode_mapping().get('GUIDED', 15))
                time.sleep(0.5)

                print("💪 ARM Yapılıyor...")
                self.master.arducopter_arm()
                self.master.motors_armed_wait()
                print("✅ ARMED!")
            except Exception as e:
                print(f"❌ ARM hatası: {e}")

        if USV_MODE == 'race':
            print("🔒 YARIŞMA MODU — HARİCİ KOMUTLAR KİLİTLENDİ (Şartname 4.4)")

    def _check_rc_connected(self, timeout_sec=3):
        """
        Pixhawk'tan RC sinyali gelip gelmediğini kontrol eder.
        Kumanda bağlı değilse ArduPilot genelde 0 veya 65535 gönderir.
        Returns: True (RC bağlı), False (RC yok)
        """
        if self.simulation_mode or not self.master:
            return True  # Simülasyonda veya Pixhawk yoksa kabul et
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            msg = self.master.recv_match(type='RC_CHANNELS', blocking=True, timeout=0.5)
            if msg:
                ch1, ch2, ch3, ch4 = msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw
                # Geçerli RC aralığı: 900–2100 (tipik PWM). 0 veya 65535 = sinyal yok
                valid = (900 <= ch1 <= 2100 or 900 <= ch2 <= 2100 or
                         900 <= ch3 <= 2100 or 900 <= ch4 <= 2100)
                if valid:
                    return True
                # Tüm kanallar 0 veya 65535 ise RC yok
                all_null = all(v in (0, 65535) for v in (ch1, ch2, ch3, ch4))
                if all_null:
                    return False
        return False

    def emergency_stop(self):
        """Acil durdurma (E-STOP). Her modda çalışır."""
        print("🚨 ACİL DURDURMA (E-STOP) TETİKLENDİ!")
        self.mission_active = False
        self.state = self.STATE_IDLE
        self._write_state()
        self.stop_motors()

    # ----------------------------------------------------------
    # WAYPOINT NAVİGASYONU (Tek Nokta)
    # ----------------------------------------------------------
    def navigate_to_waypoint(self, target_lat, target_lon, avoid_obstacles=False):
        """
        Verilen GPS noktasına ulaşana kadar ilerler.
        avoid_obstacles=True ise Lidar ile engelden kaçar.
        
        Returns: True (ulaşıldı), False (görev iptal / hata)
        """
        print(f"   🎯 Hedef: {target_lat:.7f}, {target_lon:.7f}"
              f" {'(Engel Kaçınma AÇIK)' if avoid_obstacles else ''}")
        self._wp_target = f"{target_lat:.7f}, {target_lon:.7f}"
        self._write_state()

        while self.mission_active:
            # Acil durdurma kontrolü (Dashboard'dan E-STOP)
            if os.path.exists(FLAG_STOP):
                try:
                    os.remove(FLAG_STOP)
                except Exception:
                    pass
                return False

            self._update_gps()

            # Mesafe ve açı hesapla
            if self.simulation_mode:
                # Simülasyonda hedefe doğru yürü
                dist = random.uniform(0.5, 5.0)
                heading_error = random.uniform(-10, 10)
                # Birkaç döngü sonra "vardık" de
                if not hasattr(self, '_sim_nav_counter'):
                    self._sim_nav_counter = 0
                self._sim_nav_counter += 1
                if self._sim_nav_counter > 15:
                    self._sim_nav_counter = 0
                    print(f"   ✅ Simülasyon: Hedefe ulaşıldı")
                    return True
            else:
                if self.current_lat == 0 and self.current_lon == 0:
                    time.sleep(0.5)
                    continue

                dist = haversine_distance(
                    self.current_lat, self.current_lon,
                    target_lat, target_lon)

                if dist < REACH_DIST:
                    print(f"   ✅ Hedefe ulaşıldı! (Mesafe: {dist:.1f}m)")
                    self.set_motor(1500, 1500)
                    return True

                target_bearing = calculate_bearing(
                    self.current_lat, self.current_lon,
                    target_lat, target_lon)
                heading_error = normalize_heading_error(
                    target_bearing - self.current_heading)

            # Engelden kaçınma (Parkur-2)
            if avoid_obstacles and self.obstacle_detected:
                print(f"   🛑 ENGEL! ({self.min_obstacle_distance:.2f}m) → KAÇILIYOR")
                # Olduğu yerde sola dön (tank dönüşü)
                self.set_motor(1500 - TURN_SPEED, 1500 + TURN_SPEED)
                time.sleep(1.0 / CONTROL_HZ)
                continue

            # P kontrolcü ile yön düzeltme
            correction = heading_error * 2.5  # P Gain
            left_motor = SPEED_PWM + correction
            right_motor = SPEED_PWM - correction

            self.set_motor(left_motor, right_motor)

            if not self.simulation_mode:
                print(f"   📍 Dist: {dist:.1f}m | Hdg: {self.current_heading:.0f}° | "
                      f"Err: {heading_error:.0f}°")

            time.sleep(1.0 / CONTROL_HZ)

        return False  # Görev iptal edildi

    # ----------------------------------------------------------
    # PARKUR İCRASI
    # ----------------------------------------------------------
    def run_parkur1(self):
        """Parkur-1: Engelsiz ortamda nokta takibi."""
        print("\n" + "=" * 50)
        print("  [PARKUR-1] NOKTA TAKİP GÖREVİ")
        print("=" * 50)

        if not self.waypoints_p1:
            print("⚠️ Parkur-1 görev noktası yok — Atlıyorum")
            return True

        for i, wp in enumerate(self.waypoints_p1):
            self._wp_info = f"{i + 1} / {len(self.waypoints_p1)}"
            self._write_state()
            print(f"\n[P1] Waypoint {i + 1}/{len(self.waypoints_p1)}")
            if not self.navigate_to_waypoint(wp[0], wp[1], avoid_obstacles=False):
                return False

        print("✅ PARKUR-1 TAMAMLANDI!")
        return True

    def run_parkur2(self):
        """Parkur-2: Engelli ortamda nokta takibi (Lidar ile engel kaçınma)."""
        print("\n" + "=" * 50)
        print("  [PARKUR-2] ENGELLİ NOKTA TAKİP GÖREVİ")
        print("=" * 50)

        if not self.lidar_available and not self.simulation_mode:
            print("⚠️ Lidar yok — Engel kaçınma devre dışı, sadece GPS ile ilerliyor")

        if not self.waypoints_p2:
            print("⚠️ Parkur-2 görev noktası yok — Atlıyorum")
            return True

        for i, wp in enumerate(self.waypoints_p2):
            self._wp_info = f"{i + 1} / {len(self.waypoints_p2)}"
            self._write_state()
            print(f"\n[P2] Waypoint {i + 1}/{len(self.waypoints_p2)}")
            if not self.navigate_to_waypoint(wp[0], wp[1], avoid_obstacles=True):
                return False

        print("✅ PARKUR-2 TAMAMLANDI!")
        return True

    def run_parkur3(self):
        """Parkur-3: Kamikaze angajman (hedef dubaya fiziksel temas)."""
        print("\n" + "=" * 50)
        print("  [PARKUR-3] KAMİKAZE ANGAJMAN GÖREVİ")
        print(f"  Hedef Renk: {self.target_color or 'BELİRLENMEDİ'}")
        print("=" * 50)

        if not self.waypoints_p3:
            print("⚠️ Parkur-3 görev noktası yok — Atlıyorum")
            return True

        # Son hedefe tam hız ile ulaş (engel kaçınma kapalı — kamikaze)
        wp = self.waypoints_p3[0]
        print(f"\n[P3] Kamikaze Hedef: {wp[0]:.7f}, {wp[1]:.7f}")
        if not self.navigate_to_waypoint(wp[0], wp[1], avoid_obstacles=False):
            return False

        print("💥 HEDEF ANGAJE EDİLDİ!")
        return True

    # ----------------------------------------------------------
    # PARKUR GEÇİŞ KONTROLÜ
    # ----------------------------------------------------------
    def _wait_for_next_parkur(self, current_name, next_name):
        """
        TEST modunda: Operatörün sonraki parkura geçiş komutunu bekler.
        RACE modunda: Otomatik geçiş (Şartname 4.2).
        """
        if USV_MODE == 'race':
            print(f"🔄 {current_name} → {next_name} OTOMATİK GEÇİŞ")
            return True

        # TEST MODU: Manuel onay bekle
        print(f"\n{'=' * 50}")
        print(f"  ✅ {current_name} TAMAMLANDI")
        print(f"  ⏳ {next_name} için bekleniyor...")
        print(f"  👉 Devam etmek için telemetri API'den /api/next_parkur")
        print(f"     veya bu terminalde ENTER tuşuna basın")
        print(f"{'=' * 50}")

        # Bekle: stdin ENTER, advance_flag veya Dashboard'dan next_parkur.flag
        self._advance_flag = False
        while not self._advance_flag:
            # Dashboard'dan gelen sonraki parkur isteği (dosya IPC)
            if os.path.exists(FLAG_NEXT):
                try:
                    os.remove(FLAG_NEXT)
                except Exception:
                    pass
                self._advance_flag = True
                break
            # stdin kontrolü (non-blocking mümkün değilse kısa timeout)
            try:
                import select
                ready, _, _ = select.select([sys.stdin], [], [], 1.0)
                if ready:
                    sys.stdin.readline()
                    break
            except Exception:
                time.sleep(1)

            if self._advance_flag:
                break

        print(f"🚀 {next_name} BAŞLIYOR!")
        return True

    def advance_to_next_parkur(self):
        """Telemetri API tarafından çağrılır — sonraki parkura geçiş sinyali."""
        self._advance_flag = True

    # ----------------------------------------------------------
    # SÜRE KONTROLÜ
    # ----------------------------------------------------------
    def _check_time(self):
        """
        20 dakika kontrolü.
        TEST: Sadece uyarı basar, sistemi KAPATMAZ.
        RACE: Şartname 8 gereği süreyi loglar (hakem zaten durdurur).
        """
        elapsed = time.time() - self.mission_start_time
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)

        if elapsed > 20 * 60:
            # Her 30 saniyede bir uyarı bas
            if not hasattr(self, '_last_time_warn') or time.time() - self._last_time_warn > 30:
                print(f"⏰ [UYARI] SÜRE AŞILDI! ({minutes}:{seconds:02d} / 20:00)")
                self._last_time_warn = time.time()
        elif elapsed > 18 * 60:
            # 18. dakikadan sonra periyodik uyarı
            if not hasattr(self, '_last_time_warn') or time.time() - self._last_time_warn > 60:
                print(f"⚠️ [SÜRE] {minutes}:{seconds:02d} / 20:00 — Az kaldı!")
                self._last_time_warn = time.time()

    # ----------------------------------------------------------
    # ANA DÖNGÜ
    # ----------------------------------------------------------
    def run(self):
        """
        Ana döngü.
        TEST:  Parkur geçişleri MANUEL (operatör komutuyla)
        RACE:  Parkur-1 → 2 → 3 OTOMATİK geçiş (Şartname 4.2)
        """
        self._advance_flag = False
        try:
            while self.mission_active:
                self._check_time()

                if self.state == self.STATE_PARKUR1:
                    if self.run_parkur1():
                        self._wait_for_next_parkur("PARKUR-1", "PARKUR-2")
                        self.state = self.STATE_PARKUR2
                    else:
                        break

                elif self.state == self.STATE_PARKUR2:
                    if self.run_parkur2():
                        self._wait_for_next_parkur("PARKUR-2", "PARKUR-3")
                        self.state = self.STATE_PARKUR3
                    else:
                        break

                elif self.state == self.STATE_PARKUR3:
                    if self.run_parkur3():
                        self.state = self.STATE_COMPLETED
                    else:
                        break

                elif self.state == self.STATE_COMPLETED:
                    total_time = time.time() - self.mission_start_time
                    self._wp_info = "-- / --"
                    self._wp_target = "TAMAMLANDI"
                    self._write_state()
                    print("\n" + "=" * 50)
                    print("  🎉 TÜM GÖREVLER BAŞARIYLA TAMAMLANDI!")
                    print(f"  Toplam Süre: {total_time:.1f} saniye")
                    print("=" * 50)
                    break

        except KeyboardInterrupt:
            pass
        finally:
            self.emergency_stop()


# ============================================================
# PROGRAM GİRİŞ NOKTASI
# ============================================================
if __name__ == "__main__":
    usv = USVStateMachine()

    # Görev dosyasını yükle (USB veya varsayılan)
    mission_path = sys.argv[1] if len(sys.argv) > 1 else MISSION_FILE
    usv.load_mission(mission_path)

    # Kumanda bağlı değilse otomatik ARM yapma. Dashboard'dan "Görevi Başlat" beklenir.
    print("\n⏸️  SİSTEM HAZIR — GÖREV BEKLİYOR")
    print("   Kumanda bağlı olmalı. Dashboard'dan '▶ Görevi Başlat' ile başlatın.")
    print("   http://<RPi_IP>:8080\n")
    usv._write_state()

    while True:
        # Acil durdurma her zaman kontrol edilsin
        if os.path.exists(FLAG_STOP):
            try:
                os.remove(FLAG_STOP)
            except Exception:
                pass
            usv.emergency_stop()
            break

        # Görev başlatma isteği
        if os.path.exists(FLAG_START):
            try:
                os.remove(FLAG_START)
            except Exception:
                pass
            usv.start_mission()
            usv.run()
            # Görev bitti; tekrar başlatma için bekle
            usv.mission_active = False
            usv.state = usv.STATE_IDLE
            usv._write_state()
            print("\n⏸️  SİSTEM HAZIR — Yeni görev başlatmak için Dashboard kullanın.\n")

        time.sleep(0.5)

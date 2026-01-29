import os
# SHM hatasını kes (FastDDS sadece UDPv4 kullansın)
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

import time
import math
import threading
import sys
from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import numpy as np

# --- AYARLAR ---
TARGET_LAT = 41.123456
TARGET_LON = 29.123456

# LiDAR Ayarları (ROS2 /scan üzerinden)
MIN_DIST_MM = 1200          # 1.2 m
USE_FRONT_ONLY = True       # Sadece ön sektörü kullan
FRONT_DEG = 30              # Ön sektör: -30 derece ile +30 derece

LIDAR_TIMEOUT_S = 2.5        # Bu sürede scan gelmezse LiDAR yok varsay
LIDAR_MIN_VALID_M = 0.05     # Çok yakındaki gürültüyü atmak için

# Hız Ayarları
NORMAL_SPEED = 1650
TURN_SPEED = 1600
CONNECTION_STRING = "/dev/ttyACM1"
BAUD_RATE = 115200


class LidarMonitor(Node):
    """
    /scan LaserScan okuyup en yakın engel mesafesini hesaplar.
    obstacle_detected bayrağı üretir.
    """
    def __init__(self, min_dist_mm: int, use_front_only: bool, front_deg: float):
        super().__init__("ida_lidar_monitor")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.min_dist_m = min_dist_mm / 1000.0
        self.use_front_only = use_front_only
        self.front_rad = math.radians(front_deg)

        self.last_scan_time = time.time()
        self.current_min_dist_m = float("inf")
        self.obstacle_detected = False

        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos)

    def _scan_cb(self, msg: LaserScan):
        self.last_scan_time = time.time()

        ranges = np.array(msg.ranges, dtype=np.float32)

        # Geçerli ölçüm filtresi
        finite = np.isfinite(ranges)
        valid = (
            finite
            & (ranges > max(msg.range_min, LIDAR_MIN_VALID_M))
            & (ranges < msg.range_max)
        )

        if not np.any(valid):
            self.current_min_dist_m = float("inf")
            self.obstacle_detected = False
            return

        # Açılar
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment

        # Ön sektör filtresi
        if self.use_front_only:
            front_mask = (angles >= -self.front_rad) & (angles <= self.front_rad)
            valid = valid & front_mask

            if not np.any(valid):
                self.current_min_dist_m = float("inf")
                self.obstacle_detected = False
                return

        nearest = float(np.min(ranges[valid]))
        self.current_min_dist_m = nearest
        self.obstacle_detected = nearest <= self.min_dist_m

    def lidar_ok(self) -> bool:
        return (time.time() - self.last_scan_time) <= LIDAR_TIMEOUT_S


class AutonomousIda:
    def __init__(self):
        self.running = True

        # LiDAR durum değişkenleri (ROS thread'i bunları güncelleyecek)
        self.lidar_node = None
        self._ros_thread = None
        self._ros_lock = threading.Lock()

        # --- 1) Pixhawk Bağlantısı ---
        print(f"🔌 Pixhawk Aranıyor ({CONNECTION_STRING})...")
        try:
            self.master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE)
            self.master.wait_heartbeat(timeout=3)
            print("✅ Pixhawk Bağlandı!")
        except:
            try:
                print("⚠️ ACM1 başarısız, ACM0 deneniyor...")
                self.master = mavutil.mavlink_connection("/dev/ttyACM0", baud=BAUD_RATE)
                self.master.wait_heartbeat(timeout=3)
                print("✅ Pixhawk Bağlandı (ACM0)!")
            except:
                print("❌ Pixhawk Bulunamadı! USB Kablosunu Kontrol Et.")
                sys.exit()

        # --- 2) LiDAR (ROS2 /scan) ---
        print("🦇 LiDAR (ROS2 /scan) dinleniyor...")
        self._start_ros_lidar()

    def _start_ros_lidar(self):
        """
        ROS2 node'u ayrı thread'de başlatır.
        Bu scriptin çalıştığı ortamda ROS2 kurulu ve source edilmiş olmalı.
        """
        def ros_spin():
            rclpy.init()
            node = LidarMonitor(MIN_DIST_MM, USE_FRONT_ONLY, FRONT_DEG)
            with self._ros_lock:
                self.lidar_node = node

            try:
                rclpy.spin(node)
            finally:
                try:
                    node.destroy_node()
                except:
                    pass
                rclpy.shutdown()

        self._ros_thread = threading.Thread(target=ros_spin, daemon=True)
        self._ros_thread.start()

    def _get_lidar_state(self):
        with self._ros_lock:
            node = self.lidar_node
        if node is None:
            return False, False, float("inf")

        ok = node.lidar_ok()
        obstacle = node.obstacle_detected
        dist_m = node.current_min_dist_m
        return ok, obstacle, dist_m

    def get_location(self):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.1)
        if msg:
            return msg.lat / 1e7, msg.lon / 1e7, msg.hdg / 100.0
        return None, None, None

    def get_bearing(self, lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        y = math.sin(dLon) * math.cos(math.radians(lat2))
        x = (
            math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))
            - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dLon)
        )
        return (math.degrees(math.atan2(y, x)) + 360) % 360

    def set_motor(self, left, right):
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            int(left), 0, int(right), 0, 0, 0, 0, 0
        )

    def start(self):
        print("\n🚀 İDA SİSTEMİ BAŞLATILIYOR (LiDAR: /scan)...")
        print("-" * 50)

        # Modu GUIDED yap
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self.master.mode_mapping()["GUIDED"]
        )
        time.sleep(1)

        print("💪 ARM Yapılıyor...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("✅ MOTORLAR HAZIR!")

        last_lidar_print = 0.0

        try:
            while self.running:
                # --- 0) LiDAR Güvenlik Katmanı ---
                lidar_ok, obstacle, dist_m = self._get_lidar_state()

                now = time.time()
                if now - last_lidar_print > 0.5:
                    if lidar_ok:
                        mm = int(dist_m * 1000) if math.isfinite(dist_m) else 999999
                        print(f"🦇 LiDAR OK | min={mm}mm | obstacle={obstacle}", end="\r")
                    else:
                        print("🦇 LiDAR YOK (timeout) | DUR!", end="\r")
                    last_lidar_print = now

                # LiDAR yoksa veya engel varsa dur
                if (not lidar_ok) or obstacle:
                    self.set_motor(1500, 1500)
                    time.sleep(0.1)
                    continue

                # --- 1) Navigasyon ---
                lat, lon, heading = self.get_location()

                if lat and lat != 0:
                    target_dir = self.get_bearing(lat, lon, TARGET_LAT, TARGET_LON)
                    error = target_dir - heading
                    if error > 180:
                        error -= 360
                    if error < -180:
                        error += 360

                    correction = error * 2.5
                    left = NORMAL_SPEED + correction
                    right = NORMAL_SPEED - correction

                    left = max(1100, min(1900, left))
                    right = max(1100, min(1900, right))

                    self.set_motor(left, right)
                else:
                    self.set_motor(1500, 1500)
                    print("📡 GPS Aranıyor...", end="\r")

                time.sleep(0.1)

        except KeyboardInterrupt:
            self.running = False
            self.set_motor(1500, 1500)
            self.master.arducopter_disarm()
            print("\n🛑 Sistem Kapatıldı.")


if __name__ == "__main__":
    bot = AutonomousIda()
    bot.start()

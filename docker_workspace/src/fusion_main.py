import os
# FastDDS SHM hatasını engelle (ÇOK ÖNEMLİ)
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

import time
import math
import sys
import glob
import threading
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# --- AYARLAR ---
TARGET_HEADING = 0  # Pusula Hedefi (Kuzey)
SPEED_PWM = 1650    # İleri Hız
TURN_SPEED = 200    # Dönüş Sertliği
OBSTACLE_DIST = 1.5 # 1.5 metreden yakınsa engel var say

class USVController(Node):
    def __init__(self):
        super().__init__('usv_controller')
        
        # --- 1. LIDAR BAĞLANTISI (ROS 2) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        self.obstacle_detected = False
        self.min_distance = 99.0
        self.last_lidar_time = time.time()
        print("✅ [ROS] Lidar dinleniyor...")

        # --- 2. PIXHAWK BAĞLANTISI (MAVLINK) ---
        self.connection_string = self.find_pixhawk()
        print(f"🔌 [MAV] Pixhawk aranıyor... Bulunan: {self.connection_string}")
        
        try:
            self.master = mavutil.mavlink_connection(self.connection_string, baud=115200)
            self.master.wait_heartbeat(timeout=5)
            print("✅ [MAV] Pixhawk Kalp Atışı Alındı!")
        except:
            print("❌ [MAV] PIXHAWK BULUNAMADI! Simülasyon modunda devam ediliyor...")
            self.master = None

        # --- 3. KONTROL DÖNGÜSÜ ---
        # 10 Hz hızında ana kontrol döngüsü
        self.timer = self.create_timer(0.1, self.control_loop)

    def find_pixhawk(self):
        """Otomatik port bulucu"""
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        if not ports:
            return "/dev/ttyACM0" # Varsayılan
        return ports[0]

    def lidar_callback(self, msg):
        """Lidar verisini işle: Önümüzde engel var mı?"""
        self.last_lidar_time = time.time()
        ranges = msg.ranges
        
        # Ön Taraf: -30 derece ile +30 derece arası (Index mantığı: 0=Arka, 180=Ön ise değişir)
        # RPLidar genellikle 0 dereceyi kablonun olduğu yer veya tam karşısı verir.
        # Bu range ayarını sahada test edip 'scan' topic echo ile doğrula.
        # Varsayım: 0 derece tam karşı.
        
        # Veri sayısı
        count = len(ranges)
        # 30 dereceye denk gelen index sayısı
        sector_size = int(count * (30 / 360)) 
        
        # Ön Sektör: [Son 30] + [İlk 30]
        front_ranges = ranges[-sector_size:] + ranges[:sector_size]
        
        # Geçersiz verileri (inf, 0) temizle
        valid_ranges = [r for r in front_ranges if 0.15 < r < 20.0]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            if self.min_distance < OBSTACLE_DIST:
                self.obstacle_detected = True
            else:
                self.obstacle_detected = False
        else:
            self.min_distance = 99.0 # Veri yoksa uzakta say

    def set_motor(self, left, right):
        """Motorlara PWM gönder (Skid Steering)"""
        if self.master:
            # RC Override Kanal 1 ve 3 (Genelde Left/Right veya Throttle/Steer)
            # ArduRover Skid Steering modunda: Ch1=Steer, Ch3=Throttle olabilir.
            # Burada direk Left/Right motor olarak varsayıyoruz.
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                int(left), int(right), 65535, 65535, 65535, 65535, 65535, 65535
            )

    def control_loop(self):
        """ANA KARAR MEKANİZMASI"""
        
        # 1. Güvenlik Kontrolü: Lidar verisi taze mi?
        if time.time() - self.last_lidar_time > 3.0:
            print("⚠️ [UYARI] Lidar verisi kesildi! DURUYORUM.")
            self.set_motor(1500, 1500)
            return

        # 2. Engelden Kaçış Mantığı
        if self.obstacle_detected:
            print(f"🛑 ENGEL! Mesafe: {self.min_distance:.2f}m -> KAÇILIYOR (Sola Dön)")
            # Olduğu yerde sola dön (Tank dönüşü)
            self.set_motor(1500 - TURN_SPEED, 1500 + TURN_SPEED)
        
        else:
            print(f"🌊 Yol Açık ({self.min_distance:.2f}m) -> İLERİ")
            # Düz git
            self.set_motor(SPEED_PWM, SPEED_PWM)

def main(args=None):
    rclpy.init(args=args)
    usv_node = USVController()
    
    try:
        rclpy.spin(usv_node)
    except KeyboardInterrupt:
        print("Kapatılıyor...")
    finally:
        # Çıkışta motorları durdur
        usv_node.set_motor(1500, 1500)
        usv_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

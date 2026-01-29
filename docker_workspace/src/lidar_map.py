import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import math
import threading
import time
import os
from flask import Flask, Response

# --- AYARLAR ---
WEB_PORT = 5001
MAP_SIZE = 600       # 600x600 piksel harita
MAX_RANGE_M = 10.0   # Haritanın kenarı kaç metre olsun? (Zoom)
SCALE = (MAP_SIZE // 2) / MAX_RANGE_M 

# Flask Sunucusu
app = Flask(__name__)
output_frame = None
lock = threading.Lock()
SIMULATION_MODE = False

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper_web')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile
        )
        
        # Boş siyah harita
        self.blank_image = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self.last_msg_time = time.time()
        
        print(f"🗺️ Lidar Harita Sunucusu Başladı (Port {WEB_PORT})")

    def scan_callback(self, msg):
        global output_frame, SIMULATION_MODE
        self.last_msg_time = time.time()
        SIMULATION_MODE = False # Veri geliyorsa simülasyonu kapat
        
        # Haritayı Temizle (Siyah)
        img = self.blank_image.copy()
        
        # Merkez Noktası (USV)
        center_x, center_y = MAP_SIZE // 2, MAP_SIZE // 2
        
        # Tekneyi Çiz (Yeşil Ok)
        cv2.arrowedLine(img, (center_x, center_y + 10), (center_x, center_y - 20), (0, 255, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)

        # Lidar Noktalarını Çiz
        angle = msg.angle_min
        for r in msg.ranges:
            # Sadece geçerli mesafeler
            if 0.1 < r < MAX_RANGE_M:
                # Polar -> Kartezyen Dönüşümü
                # Lidar'da açı saat yönünün tersinedir, görselleştirmede düzeltiyoruz
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                
                # Koordinat Dönüşümü (Ekran Koordinatları)
                # X ekseni -> Ekranın Y'si (İleri), Y ekseni -> Ekranın X'i (Yan)
                px = int(center_x - (y * SCALE))  # Y ekseni ters (sol/sağ)
                py = int(center_y - (x * SCALE))  # X ekseni ters (yukarı/aşağı)
                
                # Nokta Koy (Beyaz)
                cv2.circle(img, (px, py), 1, (255, 255, 255), -1)
            
            angle += msg.angle_increment

        # Engel Mesafesi Uyarısı (Çember)
        # 1.5 Metre Çemberi (Kırmızı)
        r_px = int(1.5 * SCALE)
        cv2.circle(img, (center_x, center_y), r_px, (0, 0, 100), 1)
        
        # Bilgi
        cv2.putText(img, "LIVE LIDAR DATA", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Web Yayını İçin Kopyala
        with lock:
            output_frame = img

def get_simulated_map():
    """Lidar verisi yoksa rastgele dönen noktalar üret"""
    img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
    center_x, center_y = MAP_SIZE // 2, MAP_SIZE // 2
    
    # Tekne
    cv2.arrowedLine(img, (center_x, center_y + 10), (center_x, center_y - 20), (0, 255, 0), 2)
    cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
    
    # Dönen Engeller
    t = time.time()
    for i in range(0, 360, 10):
        angle_rad = math.radians(i + (t * 50)) # Dönme efekti
        dist = 3.0 + math.sin(t + i) # Nefes alma efekti
        
        # Polar -> Cartesian
        x = dist * math.cos(angle_rad)
        y = dist * math.sin(angle_rad)
        
        px = int(center_x - (y * SCALE))
        py = int(center_y - (x * SCALE))
        
        cv2.circle(img, (px, py), 2, (255, 255, 255), -1)

    # Uyarı
    cv2.putText(img, "⚠️ LIDAR BAGLANTISI YOK - SIMULASYON", (50, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    return img

def ros_thread():
    global SIMULATION_MODE, output_frame
    rclpy.init()
    node = None
    try:
        node = LidarMapper()
        
        # Custom Spin Loop for Timeout Detection
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Veri zaman aşımı kontrolü (3 saniye veri gelmezse simülasyon)
            if time.time() - node.last_msg_time > 3.0:
                SIMULATION_MODE = True
                print("⚠️ [LIDAR] Veri akışı yok -> Simülasyon Modu")
            
            if SIMULATION_MODE:
                with lock:
                    output_frame = get_simulated_map()
                time.sleep(0.1)
                
    except Exception as e:
        print(f"⚠️ [LIDAR] ROS Hatası: {e}")
        SIMULATION_MODE = True
        while True: # Crash etme, simülasyonda kal
            with lock:
                output_frame = get_simulated_map()
            time.sleep(0.1)
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

def generate():
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                time.sleep(0.1)
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag: continue
        
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encodedImage) + b'\r\n')
        time.sleep(0.05)

def clean_port(port):
    print(f"🧹 Port {port} temizleniyor...")
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")
    os.system(f"lsof -t -i:{port} | xargs kill -9 > /dev/null 2>&1")
    time.sleep(0.5)

@app.route("/")
def map_feed():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":

    clean_port(WEB_PORT)
    t = threading.Thread(target=ros_thread)
    t.daemon = True
    t.start()
    
    print(f"🌍 HARİTA ARAYÜZÜ BAŞLATILIYOR: http://0.0.0.0:{WEB_PORT}")
    app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)

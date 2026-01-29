import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import math
import threading
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
        self.blank_image = np.zeros((MAP_SIZE, MAP_SIZE, 3), np.uint8)
        
        print(f"🗺️ Lidar Harita Sunucusu Başladı (Port {WEB_PORT})")

    def scan_callback(self, msg):
        global output_frame
        
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

        # Web Yayını İçin Kopyala
        with lock:
            output_frame = img

def ros_thread():
    rclpy.init()
    node = LidarMapper()
    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def generate():
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag: continue
        
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encodedImage) + b'\r\n')

@app.route("/")
def map_feed():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":

    clean_port(WEB_PORT)
    t = threading.Thread(target=ros_thread)
    t.daemon = True
    t.start()
    
    app.run(host="0.0.0.0", port=WEB_PORT, debug=False, use_reloader=False)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from runtime_debug_log import log_jsonl, setup_component_logger
import cv2
import numpy as np
import math
import threading
import time
import os
import sys
from flask import Flask, Response
import logging
from compliance_profile import USV_MODE, USV_MODE_RACE
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR) # Gereksiz logları kapat

# --- YARIŞMA MODU GUARD (IDA 3.7 - görüntü aktarımı yasak) ---
if USV_MODE == USV_MODE_RACE:
    print("🏁 [lidar_map.py] YARIŞMA MODU - Harita yayını başlatılmıyor.")
    sys.exit(0)

# --- AYARLAR ---
WEB_PORT = 5001
MAP_SIZE = 600       # 600x600 piksel harita
MAX_RANGE_M = 10.0   # Haritanın kenarı kaç metre olsun? (Zoom)
SCALE = (MAP_SIZE // 2) / MAX_RANGE_M 

# Flask Sunucusu
app = Flask(__name__)
output_frame = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
cv2.putText(output_frame, "MAP LOADING...", (180, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
lock = threading.Lock()
SIMULATION_MODE = False

_lidar_dbg = setup_component_logger("lidar_map")
_lidar_dbg.info(
    "lidar_map module WEB_PORT=%s LOG_DIR=%s",
    WEB_PORT,
    os.environ.get("LOG_DIR"),
)

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper_web')
        self._scan_cb_n = 0
        _lidar_dbg.info("LidarMapper node init /scan subscriber")
        
        # CRITICAL FIX: ros_gz_bridge publishes /scan with BEST_EFFORT + KEEP_LAST=10
        # Using SENSOR_DATA causes QoS deadlock; must match publisher's policy
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            lidar_qos,  # Changed from qos_profile_sensor_data to match ros_gz_bridge
        )
        
        # Boş siyah harita
        self.blank_image = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        # CRITICAL FIX: Initialize last_msg_time with buffer for GPU_lidar plugin init (~3-5 sec)
        # Without this, first 3 seconds trigger SIMULATION_MODE false negative timeout
        # By setting to 6.5 seconds ago, we grant 10 total seconds for sensor startup
        self.last_msg_time = time.time() - 6.5
        
        _lidar_dbg.info(f"[LIDAR] Subscription created with BEST_EFFORT QoS (match ros_gz_bridge)")
        print(f"🗺️ Lidar Harita Sunucusu Başladı (Port {WEB_PORT})")

    def scan_callback(self, msg):
        global output_frame, SIMULATION_MODE
        self.last_msg_time = time.time()
        SIMULATION_MODE = False
        self._scan_cb_n += 1
        if self._scan_cb_n <= 3 or self._scan_cb_n % 60 == 0:
            _lidar_dbg.info(  # Changed from debug to info for first few messages
                "[LIDAR_CALLBACK] scan n=%s ranges=%s angle_min=%.4f inc=%.6f",
                self._scan_cb_n,
                len(msg.ranges),
                msg.angle_min,
                msg.angle_increment,
            )
        if self._scan_cb_n <= 5 or self._scan_cb_n % 10 == 0:
            log_jsonl(
                "lidar_map",
                False,
                event="scan",
                n=self._scan_cb_n,
                n_ranges=len(msg.ranges),
            )
        
        # Haritayı Temizle
        img = self.blank_image.copy()
        
        # Merkez
        cx, cy = MAP_SIZE // 2, MAP_SIZE // 2
        
        # Tekneyi Çiz
        cv2.arrowedLine(img, (cx, cy + 10), (cx, cy - 20), (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)

        # --- VECTORIZED PROCESSING (Optimize Edilmiş) ---
        ranges = np.array(msg.ranges)
        
        # Geçersiz verileri filtrele
        valid_indices = (ranges > 0.1) & (ranges < MAX_RANGE_M)
        ranges = ranges[valid_indices]
        
        if len(ranges) > 0:
            # Açıları hesapla
            angles = msg.angle_min + np.arange(len(msg.ranges))[valid_indices] * msg.angle_increment
            
            # Polar -> Cartesian (Vektörel)
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)
            
            # Koordinat Dönüşümü (Ekran)
            # px = cx - (y * SCALE)
            # py = cy - (x * SCALE)
            # Float -> Int
            px = (cx - (y * SCALE)).astype(int)
            py = (cy - (x * SCALE)).astype(int)
            
            # Sınır Kontrolü (Harita dışına taşmayı önle)
            mask = (px >= 0) & (px < MAP_SIZE) & (py >= 0) & (py < MAP_SIZE)
            px = px[mask]
            py = py[mask]
            
            # Hızlı Çizim (Piksel Atama)
            # cv2.circle yerine direkt matris ataması çok daha hızlıdır
            img[py, px] = (255, 255, 255)
            
            # Görünürlüğü artırmak için biraz kalınlaştır (Opsiyonel, CPU yerse kapatılabilir)
            # cv2.dilate(img, kernel, ...) yapılabilir ama şimdilik gerek yok.

        # Engel Mesafesi Uyarısı (1.5m)
        r_px = int(1.5 * SCALE)
        cv2.circle(img, (cx, cy), r_px, (0, 0, 100), 1)
        
        # Bilgi
        cv2.putText(img, f"LIVE LIDAR | PTS: {len(ranges)}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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
                _lidar_dbg.warning("scan timeout -> simulation mode")
            
            if SIMULATION_MODE:
                # Debug: Topic List
                if int(time.time()) % 5 == 0:
                    topics = node.get_topic_names_and_types()
                    topic_names = [t[0] for t in topics]
                    print(f"⚠️ [DIAGNOSTIC] Mevcut Topicler: {topic_names}")
                    if '/scan' in topic_names:
                         print("✅ /scan topici mevcut, ancak veri gelmiyor (QoS veya Ağ sorunu?)")
                    else:
                         print("❌ /scan topici LİSTEDE YOK! Driver çökmüş olabilir.")

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
    import os
    print(f"🧹 Port {port} temizleniyor...")
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")

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

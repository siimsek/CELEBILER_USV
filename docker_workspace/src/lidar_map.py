import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from runtime_debug_log import log_jsonl, redirect_std_streams, setup_component_logger
import cv2
import numpy as np
import math
import threading
import time
import os
import sys
import json
from pathlib import Path
from flask import Flask, Response
import logging
from compliance_profile import USV_MODE, USV_MODE_RACE, CONTROL_DIR
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR) # Gereksiz logları kapat
_lidar_dbg = setup_component_logger("lidar_map")
redirect_std_streams(_lidar_dbg)

# --- YARIŞMA MODU GUARD (IDA 3.7 - görüntü aktarımı yasak) ---
if USV_MODE == USV_MODE_RACE:
    print("🏁 [lidar_map.py] YARIŞMA MODU - Harita yayını başlatılmıyor.")
    sys.exit(0)

# --- AYARLAR ---
WEB_PORT = 5001
MAP_SIZE = 600       # 600x600 piksel harita
MAX_RANGE_M = 10.0   # Haritanın kenarı kaç metre olsun? (Zoom)
SCALE = (MAP_SIZE // 2) / MAX_RANGE_M
INSET_SIZE = 200
INSET_SCALE = (INSET_SIZE // 2 - 10) / MAX_RANGE_M


def _read_vehicle_pose():
    """Gazebo/SITL köprüsü vehicle_position.json — pos_x, pos_y, heading_rad."""
    try:
        p = Path(CONTROL_DIR) / "vehicle_position.json"
        if not p.is_file():
            return None, None, None
        with open(p, "r", encoding="utf-8") as f:
            j = json.load(f)
        hr = j.get("heading_rad")
        x = j.get("pos_x")
        y = j.get("pos_y")
        if hr is None or x is None or y is None:
            return None, None, None
        return float(x), float(y), float(hr)
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return None, None, None


def build_north_up_inset(msg: LaserScan, heading_rad: float) -> np.ndarray:
    """Lidar gövde çerçevesinden sabit dünya (kuzey yukarı) mini harita."""
    img = np.zeros((INSET_SIZE, INSET_SIZE, 3), dtype=np.uint8)
    cx = INSET_SIZE // 2
    cy = INSET_SIZE // 2
    c = math.cos(float(heading_rad))
    s = math.sin(float(heading_rad))
    ranges = np.array(msg.ranges, dtype=np.float64)
    valid = (ranges > 0.1) & (ranges < MAX_RANGE_M) & np.isfinite(ranges)
    if np.any(valid):
        idx = np.flatnonzero(valid)
        r = ranges[idx]
        angles = float(msg.angle_min) + idx.astype(np.float64) * float(msg.angle_increment)
        xb = r * np.cos(angles)
        yb = r * np.sin(angles)
        xw = xb * c - yb * s
        yw = xb * s + yb * c
        px = (cx - (yw * INSET_SCALE)).astype(np.int32)
        py = (cy - (xw * INSET_SCALE)).astype(np.int32)
        m = (px >= 0) & (px < INSET_SIZE) & (py >= 0) & (py < INSET_SIZE)
        px = px[m]
        py = py[m]
        img[py, px] = (210, 230, 255)
    cv2.arrowedLine(img, (cx, cy + 5), (cx, cy - 14), (0, 255, 0), 2)
    cv2.circle(img, (cx, cy), 4, (0, 100, 255), -1)
    cv2.putText(img, "N", (cx - 6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 220, 255), 2)
    cv2.putText(
        img,
        "kuzey",
        (6, INSET_SIZE - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (160, 200, 220),
        1,
    )
    return img

# Flask Sunucusu
app = Flask(__name__)
output_frame = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
cv2.putText(output_frame, "MAP LOADING...", (180, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
lock = threading.Lock()
SIMULATION_MODE = False

_lidar_dbg.info(
    "lidar_map module WEB_PORT=%s LOG_DIR=%s",
    WEB_PORT,
    os.environ.get("LOG_DIR"),
)

GLOBAL_MAP_SIZE = 4000
GLOBAL_CENTER = GLOBAL_MAP_SIZE // 2

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper_web')
        self._scan_cb_n = 0
        _lidar_dbg.info("LidarMapper node init /scan subscriber")
        
        # BEST_EFFORT + depth=1: guncel tarama (derin kuyruk haritada/eski usv_main ile uyumsuzluk)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            lidar_qos,  # Changed from qos_profile_sensor_data to match ros_gz_bridge
        )
        
        # Kalıcı Dünya Haritası (SLAM tarzı iz bırakma için)
        self.global_map = np.zeros((GLOBAL_MAP_SIZE, GLOBAL_MAP_SIZE, 3), dtype=np.uint8)
        self.last_msg_time = time.time() - 6.5
        
        _lidar_dbg.info(f"[LIDAR] Subscription created with BEST_EFFORT QoS (match ros_gz_bridge)")
        print(f"🗺️ Lidar Harita Sunucusu Başladı (Port {WEB_PORT})")

    def scan_callback(self, msg):
        global output_frame, SIMULATION_MODE
        self.last_msg_time = time.time()
        SIMULATION_MODE = False
        self._scan_cb_n += 1
        
        # Pozisyon Bilgisi Al (Dünya koordinatları için)
        pos_x, pos_y, hdg = _read_vehicle_pose()
        if pos_x is None:
            pos_x, pos_y, hdg = 0.0, 0.0, 0.0

        # --- DÜNYA HARİTASINI HAFİFÇE KARART (Decay) ---
        if self._scan_cb_n % 5 == 0:
            self.global_map = cv2.addWeighted(self.global_map, 0.98, np.zeros_like(self.global_map), 0.02, 0)

        # --- VECTORIZED PROCESSING (Polar -> Global Cartesian) ---
        ranges = np.array(msg.ranges)
        valid_indices = (ranges > 0.1) & (ranges < MAX_RANGE_M)
        ranges = ranges[valid_indices]
        
        if len(ranges) > 0:
            angles = msg.angle_min + np.arange(len(msg.ranges))[valid_indices] * msg.angle_increment
            
            # Tekneye Göre Lokal Koordinatlar
            local_x = ranges * np.cos(angles)
            local_y = ranges * np.sin(angles)
            
            # Dünya Koordinatlarına Çevir (Odometri Entegrasyonu)
            c = math.cos(hdg)
            s = math.sin(hdg)
            global_x_m = pos_x + (local_x * c - local_y * s)
            global_y_m = pos_y + (local_x * s + local_y * c)
            
            # Global Harita Piksel İndeksleri
            map_px = (GLOBAL_CENTER - global_y_m * SCALE).astype(int)
            map_py = (GLOBAL_CENTER - global_x_m * SCALE).astype(int)
            
            # Sınır Kontrolü
            mask = (map_px >= 0) & (map_px < GLOBAL_MAP_SIZE) & (map_py >= 0) & (map_py < GLOBAL_MAP_SIZE)
            map_px = map_px[mask]
            map_py = map_py[mask]
            
            # Global Haritaya Çiz (Kalınlaştırılmış 2x2 Noktalar)
            self.global_map[map_py, map_px] = (255, 255, 255)
            mask_y1 = (map_py + 1 < GLOBAL_MAP_SIZE)
            self.global_map[map_py[mask_y1] + 1, map_px[mask_y1]] = (255, 255, 255)
            mask_x1 = (map_px + 1 < GLOBAL_MAP_SIZE)
            self.global_map[map_py[mask_x1], map_px[mask_x1] + 1] = (255, 255, 255)

        # --- ARAYÜZ (UI) İÇİN HARİTAYI KES VE DÖNDÜR ---
        # Tekne merkezli ROI (Bölge) çıkart
        boat_px = int(GLOBAL_CENTER - pos_y * SCALE)
        boat_py = int(GLOBAL_CENTER - pos_x * SCALE)
        
        crop_size = int(MAP_SIZE * 1.5)  # Rotasyon için ekstra pay
        half_crop = crop_size // 2
        
        x1 = max(0, boat_px - half_crop)
        y1 = max(0, boat_py - half_crop)
        x2 = min(GLOBAL_MAP_SIZE, boat_px + half_crop)
        y2 = min(GLOBAL_MAP_SIZE, boat_py + half_crop)
        
        roi = np.zeros((crop_size, crop_size, 3), dtype=np.uint8)
        
        # ROI sınırlarını güvenli kopyala
        roi_x1 = max(0, half_crop - (boat_px - x1))
        roi_y1 = max(0, half_crop - (boat_py - y1))
        roi_x2 = roi_x1 + (x2 - x1)
        roi_y2 = roi_y1 + (y2 - y1)
        roi[roi_y1:roi_y2, roi_x1:roi_x2] = self.global_map[y1:y2, x1:x2]
        
        # ROI'yi Tekne "Yukarı" Bakacak Şekilde Döndür
        center = (half_crop, half_crop)
        M = cv2.getRotationMatrix2D(center, math.degrees(hdg), 1.0)
        rotated_roi = cv2.warpAffine(roi, M, (crop_size, crop_size), flags=cv2.INTER_LINEAR)
        
        # 600x600 Final Görüntüyü Kes
        final_x1 = half_crop - (MAP_SIZE // 2)
        final_y1 = half_crop - (MAP_SIZE // 2)
        img = rotated_roi[final_y1:final_y1+MAP_SIZE, final_x1:final_x1+MAP_SIZE].copy()
        
        # --- TEKNEYİ ÇİZ ---
        cx, cy = MAP_SIZE // 2, MAP_SIZE // 2
        cv2.arrowedLine(img, (cx, cy + 10), (cx, cy - 20), (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
        r_px = int(1.5 * SCALE)
        cv2.circle(img, (cx, cy), r_px, (0, 0, 100), 1)
        
        # --- BİLGİ METİNLERİ ---
        cv2.putText(img, f"SLAM MAP | PTS: {len(ranges)}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(img, "burun yukari (lidar govde)", (10, MAP_SIZE - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 200, 140), 1)

        if pos_x is not None:
            inset = build_north_up_inset(msg, hdg)
            x0 = MAP_SIZE - INSET_SIZE - 10
            y0 = 10
            img[y0 : y0 + INSET_SIZE, x0 : x0 + INSET_SIZE] = inset
            cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + INSET_SIZE, y0 + INSET_SIZE), (0, 200, 255), 1)
            cv2.putText(img, f"hdg {math.degrees(hdg):.0f}deg", (x0, y0 + INSET_SIZE + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 255), 1)
        else:
            cv2.putText(
                img,
                "heading yok",
                (MAP_SIZE - 120, MAP_SIZE - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (80, 80, 120),
                1,
            )

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
                time.sleep(0.05)
                
    except Exception as e:
        print(f"⚠️ [LIDAR] ROS Hatası: {e}")
        SIMULATION_MODE = True
        while True: # Crash etme, simülasyonda kal
            with lock:
                output_frame = get_simulated_map()
            time.sleep(0.05)
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

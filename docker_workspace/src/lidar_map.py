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
DEFAULT_SPAN_M = 20.0  # Default span (metre) - dynamic olarak ayarlanacak


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
        
        # Accumulated lidar points in world coordinates (dashboard ile aynı sistem)
        self.world_points = []  # List of (x_m, y_m) tuples
        self.last_msg_time = time.time() - 6.5
        self.max_points = 10000  # Maximum points to keep (memory limit)
        
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

        # --- VECTORIZED PROCESSING (Polar -> World Cartesian) ---
        # Dashboard ile aynı transform: wx = bx + (lx*c) - (ly*s), wy = by + (lx*s) + (ly*c)
        ranges = np.array(msg.ranges)
        MAX_RANGE_M = 10.0
        valid_indices = (ranges > 0.1) & (ranges < MAX_RANGE_M)
        ranges = ranges[valid_indices]
        
        if len(ranges) > 0:
            angles = msg.angle_min + np.arange(len(msg.ranges))[valid_indices] * msg.angle_increment
            
            # Tekneye Göre Lokal Koordinatlar
            local_x = ranges * np.cos(angles)
            local_y = ranges * np.sin(angles)
            
            # Dünya Koordinatlarına Çevir (Dashboard ile aynı transform)
            c = math.cos(hdg)
            s = math.sin(hdg)
            world_x = pos_x + (local_x * c - local_y * s)
            world_y = pos_y + (local_x * s + local_y * c)
            
            # Accumulate world points (no decay - points stay fixed)
            for wx, wy in zip(world_x, world_y):
                self.world_points.append((float(wx), float(wy)))
            
            # Limit memory usage
            if len(self.world_points) > self.max_points:
                self.world_points = self.world_points[-self.max_points:]

        # --- DYNAMIC SPAN CALCULATION (Dashboard ile aynı) ---
        # Calculate span based on all accumulated points
        if len(self.world_points) > 0:
            all_x = [p[0] for p in self.world_points]
            all_y = [p[1] for p in self.world_points]
            cx = pos_x
            cy = pos_y
            
            # Calculate span
            span = DEFAULT_SPAN_M
            for wx, wy in self.world_points:
                dx = wx - cx
                dy = wy - cy
                span = max(span, math.hypot(dx, dy) * 1.3)
        else:
            cx, cy = pos_x, pos_y
            span = DEFAULT_SPAN_M

        # --- DYNAMIC SCALE (Dashboard ile aynı) ---
        scale = min(MAP_SIZE, MAP_SIZE) / (2.0 * span)
        
        # --- COORDINATE TRANSFORM (Dashboard ile aynı: North-up) ---
        # toPx(x, y) = [(width*0.5) + ((x-cx)*scale), (height*0.5) - ((y-cy)*scale)]
        def to_px(x, y):
            px = (MAP_SIZE * 0.5) + ((x - cx) * scale)
            py = (MAP_SIZE * 0.5) - ((y - cy) * scale)
            return int(px), int(py)

        # --- DRAW MAP ---
        img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        
        # Draw grid (dashboard ile aynı)
        grid_step_m = 2.0 if span <= 30.0 else 5.0
        left_m = cx - span
        right_m = cx + span
        bottom_m = cy - span
        top_m = cy + span
        
        gx = math.floor(left_m / grid_step_m) * grid_step_m
        while gx <= right_m:
            p0 = to_px(gx, bottom_m)
            p1 = to_px(gx, top_m)
            cv2.line(img, p0, p1, (110, 130, 170), 1)
            gx += grid_step_m
        
        gy = math.floor(bottom_m / grid_step_m) * grid_step_m
        while gy <= top_m:
            p0 = to_px(left_m, gy)
            p1 = to_px(right_m, gy)
            cv2.line(img, p0, p1, (110, 130, 170), 1)
            gy += grid_step_m

        # Draw lidar points (dashboard ile aynı color)
        for wx, wy in self.world_points:
            px, py = to_px(wx, wy)
            if 0 <= px < MAP_SIZE and 0 <= py < MAP_SIZE:
                img[py, px] = (56, 189, 248)  # Light blue

        # Draw boat (center)
        boat_px, boat_py = to_px(cx, cy)
        cv2.arrowedLine(img, (boat_px, boat_py + 10), (boat_px, boat_py - 20), (0, 255, 0), 2)
        cv2.circle(img, (boat_px, boat_py), 5, (0, 0, 255), -1)
        
        # --- BİLGİ METİNLERİ ---
        cv2.putText(img, f"UNIFIED MAP | PTS: {len(self.world_points)}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(img, f"SPAN: {span:.1f}m | SCALE: {scale:.2f} px/m", (10, MAP_SIZE - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 200, 140), 1)

        with lock:
            output_frame = img

def get_simulated_map():
    """Lidar verisi yoksa rastgele dönen noktalar üret (dashboard ile aynı coordinate system)"""
    img = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
    center_x, center_y = MAP_SIZE // 2, MAP_SIZE // 2
    
    # Tekne
    cv2.arrowedLine(img, (center_x, center_y + 10), (center_x, center_y - 20), (0, 255, 0), 2)
    cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
    
    # Dönen Engeller (North-up coordinate system)
    t = time.time()
    scale = MAP_SIZE / (2.0 * DEFAULT_SPAN_M)
    for i in range(0, 360, 10):
        angle_rad = math.radians(i + (t * 50)) # Dönme efekti
        dist = 3.0 + math.sin(t + i) # Nefes alma efekti
        
        # Polar -> Cartesian (North-up: y is up)
        x = dist * math.cos(angle_rad)
        y = dist * math.sin(angle_rad)
        
        # Dashboard ile aynı coordinate transform
        px = int((MAP_SIZE * 0.5) + (x * scale))
        py = int((MAP_SIZE * 0.5) - (y * scale))
        
        if 0 <= px < MAP_SIZE and 0 <= py < MAP_SIZE:
            img[py, px] = (255, 255, 255)

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

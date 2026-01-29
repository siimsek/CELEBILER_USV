#!/usr/bin/env python3
import os
import time
import threading
import subprocess
from typing import List, Optional

# SHM hatasını kes (FastDDS sadece UDPv4 kullansın)
os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import cv2
import numpy as np
from flask import Flask, Response


# --- AYARLAR ---
PORT = 5001            # Harita Yayını Portu (SADECE BU PORT TEMİZLENİR)
MAP_SIZE = 800         # Harita Boyutu (px)
MAX_DISTANCE = 15.0    # Menzil (m)

SCALE = MAP_SIZE / (MAX_DISTANCE * 2.0)
CENTER = MAP_SIZE // 2


def _run(cmd: List[str], timeout: float = 3.0) -> subprocess.CompletedProcess:
    return subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
        check=False,
    )


def _has_cmd(name: str) -> bool:
    return _run(["bash", "-lc", f"command -v {name} >/dev/null 2>&1"]).returncode == 0


def _is_root() -> bool:
    try:
        return os.geteuid() == 0
    except Exception:
        return False


def _kill_port_with_fuser(port: int) -> bool:
    """
    fuser ile portu tutan PID'leri öldür.
    root ise sudo'suz, değilse sudo ile dene (sudo yoksa pas geç).
    """
    if not _has_cmd("fuser"):
        return False

    if _is_root():
        p = _run(["bash", "-lc", f"fuser -k {port}/tcp >/dev/null 2>&1"])
        return p.returncode == 0

    if _has_cmd("sudo"):
        p = _run(["bash", "-lc", f"sudo fuser -k {port}/tcp >/dev/null 2>&1"])
        return p.returncode == 0

    return False


def _kill_port_with_ss(port: int) -> bool:
    """
    ss ile LISTEN PID bulup kill -9 at.
    """
    if not _has_cmd("ss"):
        return False

    p = _run(["bash", "-lc", f"ss -ltnp 'sport = :{port}' 2>/dev/null | tail -n +2"])
    if p.returncode != 0 or not p.stdout.strip():
        return False

    pids = set()
    for line in p.stdout.splitlines():
        if "pid=" in line:
            parts = line.split("pid=")
            for seg in parts[1:]:
                pid_str = ""
                for ch in seg:
                    if ch.isdigit():
                        pid_str += ch
                    else:
                        break
                if pid_str:
                    pids.add(pid_str)

    if not pids:
        return False

    for pid in pids:
        _run(["bash", "-lc", f"kill -9 {pid} >/dev/null 2>&1 || true"])

    return True


def _kill_port_with_lsof(port: int) -> bool:
    """
    lsof ile LISTEN PID bulup kill -9 at.
    """
    if not _has_cmd("lsof"):
        return False

    p = _run(["bash", "-lc", f"lsof -t -iTCP:{port} -sTCP:LISTEN 2>/dev/null"])
    if p.returncode != 0 or not p.stdout.strip():
        return False

    pids = {x.strip() for x in p.stdout.splitlines() if x.strip().isdigit()}
    if not pids:
        return False

    for pid in pids:
        _run(["bash", "-lc", f"kill -9 {pid} >/dev/null 2>&1 || true"])

    return True


def kill_existing_process(port: int) -> None:
    """
    SADECE verilen portu temizler.
    Sıra: fuser -> ss -> lsof
    """
    print(f"🧹 Port {port} temizleniyor...")

    cleaned = False

    # 1) fuser
    cleaned = _kill_port_with_fuser(port)

    # 2) ss fallback
    if not cleaned:
        cleaned = _kill_port_with_ss(port)

    # 3) lsof fallback
    if not cleaned:
        cleaned = _kill_port_with_lsof(port)

    time.sleep(0.5)

    # doğrula
    still_listening = False
    if _has_cmd("ss"):
        chk = _run(["bash", "-lc", f"ss -ltn 'sport = :{port}' 2>/dev/null | tail -n +2"])
        still_listening = bool(chk.stdout.strip())
    elif _has_cmd("lsof"):
        chk = _run(["bash", "-lc", f"lsof -iTCP:{port} -sTCP:LISTEN 2>/dev/null | head -n 1"])
        still_listening = bool(chk.stdout.strip())

    if still_listening:
        print(f"⚠️ Port {port} hala dolu görünüyor. (fuser/ss/lsof yetmedi)")
    else:
        if cleaned:
            print(f"💀 Port {port} boşaltıldı.")
        else:
            print(f"✅ Port {port} zaten temiz.")


class LidarMapper(Node):
    def __init__(self):
        super().__init__("lidar_mapper")

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback,
            qos_profile,
        )

        self.lock = threading.Lock()
        self.latest_map = None

        self.base_map = np.zeros((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8)
        self._draw_hud_template()

        self.last_scan_time = time.time()
        self.create_timer(2.0, self._health_check)

        print("🚀 LiDAR Haritalama Başlatıldı (Auto-Cleanup Aktif)...")

    def _health_check(self):
        dt = time.time() - self.last_scan_time
        if dt > 2.5:
            self.get_logger().warn(f"/scan gelmiyor (son {dt:.1f}s). Driver calisiyor mu?")

    def _draw_hud_template(self):
        for i in range(5, int(MAX_DISTANCE) + 1, 5):
            r_px = int(i * SCALE)
            cv2.circle(self.base_map, (CENTER, CENTER), r_px, (30, 30, 30), 1)
            cv2.putText(
                self.base_map,
                f"{i}m",
                (CENTER + 5, CENTER - r_px + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (100, 100, 100),
                1,
            )

        pts = np.array(
            [[CENTER, CENTER - 8], [CENTER - 6, CENTER + 6], [CENTER + 6, CENTER + 6]],
            np.int32,
        )
        cv2.fillPoly(self.base_map, [pts], (255, 0, 0))

    def listener_callback(self, msg: LaserScan):
        self.last_scan_time = time.time()

        map_img = self.base_map.copy()

        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment

        finite = np.isfinite(ranges)
        valid = finite & (ranges > msg.range_min) & (ranges < msg.range_max)

        vr = ranges[valid]
        va = angles[valid]

        if vr.size > 0:
            x = vr * np.cos(va)
            y = vr * np.sin(va)

            px = (CENTER + x * SCALE).astype(np.int32)
            py = (CENTER - y * SCALE).astype(np.int32)

            in_bounds = (px >= 0) & (px < MAP_SIZE) & (py >= 0) & (py < MAP_SIZE)
            map_img[py[in_bounds], px[in_bounds]] = (0, 255, 255)

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(
            map_img,
            f"TARIH: {timestamp}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            map_img,
            "EGE IDA - HARITA",
            (10, MAP_SIZE - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        with self.lock:
            self.latest_map = map_img

    def get_jpeg(self) -> Optional[bytes]:
        with self.lock:
            if self.latest_map is None:
                return None
            img = self.latest_map.copy()

        ok, jpeg = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return None
        return jpeg.tobytes()


app = Flask(__name__)
mapper: Optional[LidarMapper] = None


def ros_spin():
    global mapper
    rclpy.init()
    mapper = LidarMapper()
    try:
        rclpy.spin(mapper)
    finally:
        mapper.destroy_node()
        rclpy.shutdown()


@app.route("/")
def video_feed():
    def generate():
        while True:
            if mapper is None:
                time.sleep(0.1)
                continue

            frame = mapper.get_jpeg()
            if frame is None:
                time.sleep(0.05)
                continue

            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.03)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    kill_existing_process(PORT)

    threading.Thread(target=ros_spin, daemon=True).start()

    print(f"🛰️ Harita Sunucusu Başlatılıyor... http://0.0.0.0:{PORT}")
    try:
        app.run(host="0.0.0.0", port=PORT, threaded=True, debug=False)
    except OSError as e:
        print(f"⚠️ Sunucu acilamadi: {e}")
        print("⚠️ Port dolu olabilir. 2 saniye bekleyip tekrar dene.")

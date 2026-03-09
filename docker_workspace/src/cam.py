import json
import logging
import os
import socket
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response

from console_utils import make_console_printer
from compliance_profile import (
    CAM_ADAPT_BRIGHT_BETA,
    CAM_ADAPT_ENABLED,
    CAM_ADAPT_EXPOSURE_MAX_GAIN,
    CAM_ADAPT_EXPOSURE_MIN_GAIN,
    CAM_ADAPT_HSV_BRIGHT_S_SHIFT,
    CAM_ADAPT_HSV_BRIGHT_V_SHIFT,
    CAM_ADAPT_HSV_DARK_S_RELAX,
    CAM_ADAPT_HSV_DARK_V_RELAX,
    CAM_ADAPT_LOG_PERIOD_S,
    CAM_ADAPT_LUMA_BRIGHT_THRESHOLD,
    CAM_ADAPT_LUMA_DARK_THRESHOLD,
    CAM_ADAPT_DARK_BETA,
    USV_MODE,
    USV_MODE_RACE,
)

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

print = make_console_printer("CAM")

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

RACE_MODE = USV_MODE == USV_MODE_RACE
if RACE_MODE:
    print("[RACE] [cam.py] YARISMA MODU - web yayini kapali, onboard isleme aktif")

HOST = os.environ.get("CAM_HOST", "127.0.0.1")
PORT = 8888
WEB_PORT = 5000
CONTROL_DIR = "/root/workspace/control"
STATUS_FILE = f"{CONTROL_DIR}/camera_status.json"
VIDEO_DIR = "/root/workspace/logs/video"
FILE1_MP4 = f"{VIDEO_DIR}/file1_camera_processed.mp4"

COLOR_RANGES = {
    "SARI_ENGEL": {
        "lower": np.array([20, 100, 100]),
        "upper": np.array([35, 255, 255]),
        "color": (0, 255, 255),
    },
    "YESIL_SANCAK": {
        "lower": np.array([40, 80, 80]),
        "upper": np.array([85, 255, 255]),
        "color": (0, 255, 0),
    },
    "KIRMIZI_SANCAK": {
        "lower": np.array([0, 150, 100]),
        "upper": np.array([10, 255, 255]),
        "lower2": np.array([170, 150, 100]),
        "upper2": np.array([180, 255, 255]),
        "color": (0, 0, 255),
    },
}

TARGET_CLASS = os.environ.get("TARGET_CLASS", "KIRMIZI_SANCAK")


def clean_port(port):
    import os
    print(f"🧹 Port {port} temizleniyor...")
    os.system(f"fuser -k {port}/tcp > /dev/null 2>&1")


class VideoCamera:
    def __init__(self):
        self.frame = None
        self.connected = False
        self.stopped = False
        self._warn_last = {}
        self.error_counters = {"status_write_error": 0}
        self.last_frame_ts = 0.0
        self.gate_seen_since = None
        self.last_gate_seen_ts = 0.0
        self.last_gate_stable = 0.0
        self.gate_passed_until = 0.0
        self.status = {
            "ts_monotonic": 0.0,
            "frame_age_s": 999.0,
            "gate_detected": False,
            "gate_stable_s": 0.0,
            "gate_center_bearing_deg": 0.0,
            "gate_passed_event": False,
            "target_detected": False,
            "target_bearing_error_deg": 0.0,
            "target_area_norm": 0.0,
            "camera_adaptation": {
                "enabled": bool(CAM_ADAPT_ENABLED),
                "mode": "normal",
                "luma_mean": 0.0,
                "exposure_gain": 1.0,
                "exposure_beta": 0.0,
                "hsv_s_shift": 0,
                "hsv_v_shift": 0,
                "hsv_profile": "base",
            },
        }
        
        # IO Caching state
        self._last_written_status = None
        self._last_status_write_time = 0.0

        os.makedirs(VIDEO_DIR, exist_ok=True)
        os.makedirs(CONTROL_DIR, exist_ok=True)

        self.writer = None
        try:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(FILE1_MP4, fourcc, 10.0, (1280, 720))
            if not self.writer.isOpened():
                print("[WARN] [CAM] File-1 writer acilamadi")
                self.writer = None
            else:
                print(f"[REC] [CAM] File-1 kaydi: {FILE1_MP4}")
        except Exception as exc:
            print(f"[WARN] [CAM] Video writer hatasi: {exc}")

        print(f"[CAM] [CAM] Kamera sistemi baslatiliyor ({HOST}:{PORT})")

    def _warn_throttled(self, key, message, period_s=5.0):
        now = time.monotonic()
        last = self._warn_last.get(key, 0.0)
        if now - last >= period_s:
            print(message)
            self._warn_last[key] = now

    def _bump_error(self, key, message=None, period_s=5.0):
        self.error_counters[key] = int(self.error_counters.get(key, 0)) + 1
        if message:
            self._warn_throttled(
                f"err_{key}",
                f"{message} (count={self.error_counters[key]})",
                period_s=period_s,
            )

    def _adapt_log(self, key, message):
        self._warn_throttled(f"cam_adapt_{key}", message, period_s=CAM_ADAPT_LOG_PERIOD_S)

    def _shift_hsv_lower(self, lower_arr, s_shift, v_shift):
        shifted = lower_arr.astype(np.int16)
        shifted[1] = int(clamp(int(shifted[1]) + int(s_shift), 0, 255))
        shifted[2] = int(clamp(int(shifted[2]) + int(v_shift), 0, 255))
        return shifted.astype(lower_arr.dtype)

    def _resolve_adaptation_profile(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        luma_mean = float(np.mean(gray))

        profile = {
            "enabled": bool(CAM_ADAPT_ENABLED),
            "mode": "normal",
            "luma_mean": round(luma_mean, 2),
            "exposure_gain": 1.0,
            "exposure_beta": 0.0,
            "hsv_s_shift": 0,
            "hsv_v_shift": 0,
            "hsv_profile": "base",
        }

        if not CAM_ADAPT_ENABLED:
            return profile

        dark_th = float(CAM_ADAPT_LUMA_DARK_THRESHOLD)
        bright_th = float(CAM_ADAPT_LUMA_BRIGHT_THRESHOLD)

        if luma_mean <= dark_th:
            strength = clamp((dark_th - luma_mean) / max(1.0, dark_th), 0.0, 1.0)
            gain = 1.0 + (float(CAM_ADAPT_EXPOSURE_MAX_GAIN) - 1.0) * strength
            beta = float(CAM_ADAPT_DARK_BETA) * strength
            s_shift = -int(round(float(CAM_ADAPT_HSV_DARK_S_RELAX) * strength))
            v_shift = -int(round(float(CAM_ADAPT_HSV_DARK_V_RELAX) * strength))
            profile.update(
                {
                    "mode": "dark",
                    "exposure_gain": round(float(gain), 3),
                    "exposure_beta": round(float(beta), 3),
                    "hsv_s_shift": int(s_shift),
                    "hsv_v_shift": int(v_shift),
                    "hsv_profile": "dark_relaxed",
                }
            )
        elif luma_mean >= bright_th:
            strength = clamp((luma_mean - bright_th) / max(1.0, 255.0 - bright_th), 0.0, 1.0)
            gain = 1.0 - (1.0 - float(CAM_ADAPT_EXPOSURE_MIN_GAIN)) * strength
            beta = -float(CAM_ADAPT_BRIGHT_BETA) * strength
            s_shift = int(round(float(CAM_ADAPT_HSV_BRIGHT_S_SHIFT) * strength))
            v_shift = int(round(float(CAM_ADAPT_HSV_BRIGHT_V_SHIFT) * strength))
            profile.update(
                {
                    "mode": "bright",
                    "exposure_gain": round(float(gain), 3),
                    "exposure_beta": round(float(beta), 3),
                    "hsv_s_shift": int(s_shift),
                    "hsv_v_shift": int(v_shift),
                    "hsv_profile": "bright_guard",
                }
            )

        self._adapt_log(
            profile["mode"],
            (
                f"[CAM_ADAPT] mode={profile['mode']} luma={profile['luma_mean']:.1f} "
                f"gain={profile['exposure_gain']:.2f} beta={profile['exposure_beta']:.1f} "
                f"hsv_shift=({profile['hsv_s_shift']},{profile['hsv_v_shift']})"
            ),
        )
        return profile

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        time.sleep(3)
        while not self.stopped:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(5)
                sock.connect((HOST, PORT))
                stream = sock.makefile("rb")
                self.connected = True
                print("[OK] [CAM] Kamera baglandi")

                stream_bytes = b""
                while not self.stopped:
                    data = stream.read(4096)
                    if not data:
                        break
                    stream_bytes += data
                    first = stream_bytes.find(b"\xff\xd8")
                    last = stream_bytes.find(b"\xff\xd9")
                    if first != -1 and last != -1:
                        jpg = stream_bytes[first : last + 2]
                        stream_bytes = stream_bytes[last + 2 :]
                        image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if image is not None:
                            if image.shape[:2] != (720, 1280):
                                image = cv2.resize(image, (1280, 720))
                            self.frame = image
                            self.last_frame_ts = time.monotonic()
            except Exception as exc:
                self.connected = False
                print(f"[WARN] [CAM] Baglanti yok: {exc}")
                time.sleep(1.5)

    def _write_status(self):
        try:
            self.status["error_counters"] = dict(self.error_counters)
            
            now = time.monotonic()
            logical_state_changed = True
            
            if self._last_written_status is not None:
                old = self._last_written_status
                old_adapt = old.get("camera_adaptation", {}) if isinstance(old.get("camera_adaptation", {}), dict) else {}
                new_adapt = (
                    self.status.get("camera_adaptation", {})
                    if isinstance(self.status.get("camera_adaptation", {}), dict)
                    else {}
                )
                if (old.get('gate_detected') == self.status['gate_detected'] and
                    old.get('gate_passed_event') == self.status['gate_passed_event'] and
                    old.get('target_detected') == self.status['target_detected'] and
                    abs(old.get('gate_center_bearing_deg', 0) - self.status['gate_center_bearing_deg']) < 0.1 and
                    abs(old.get('target_bearing_error_deg', 0) - self.status['target_bearing_error_deg']) < 0.1 and
                    abs(old.get('target_area_norm', 0) - self.status['target_area_norm']) < 0.01 and
                    old_adapt.get("mode") == new_adapt.get("mode") and
                    abs(float(old_adapt.get("exposure_gain", 1.0)) - float(new_adapt.get("exposure_gain", 1.0))) < 0.02 and
                    int(old_adapt.get("hsv_s_shift", 0)) == int(new_adapt.get("hsv_s_shift", 0)) and
                    int(old_adapt.get("hsv_v_shift", 0)) == int(new_adapt.get("hsv_v_shift", 0))):
                    logical_state_changed = False

            if logical_state_changed or (now - self._last_status_write_time >= 1.0):
                tmp_path = f"{STATUS_FILE}.tmp"
                with open(tmp_path, "w", encoding="utf-8") as f:
                    json.dump(self.status, f)
                os.replace(tmp_path, STATUS_FILE)
                
                self._last_status_write_time = now
                self._last_written_status = dict(self.status)
                
        except Exception as exc:
            self._bump_error("status_write_error", f"[WARN] [CAM] Status yazma hatasi: {exc}")

    def _extract_detections(self, frame, adapt_profile):
        small = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
        detections = []
        s_shift = int(adapt_profile.get("hsv_s_shift", 0))
        v_shift = int(adapt_profile.get("hsv_v_shift", 0))

        for name, params in COLOR_RANGES.items():
            lower = self._shift_hsv_lower(params["lower"], s_shift, v_shift)
            mask = cv2.inRange(hsv, lower, params["upper"])
            if "lower2" in params:
                lower2 = self._shift_hsv_lower(params["lower2"], s_shift, v_shift)
                mask2 = cv2.inRange(hsv, lower2, params["upper2"])
                mask = cv2.bitwise_or(mask, mask2)
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 250:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                x, y, w, h = x * 2, y * 2, w * 2, h * 2
                detections.append(
                    {
                        "name": name,
                        "area": float(area * 4.0),
                        "x": x,
                        "y": y,
                        "w": w,
                        "h": h,
                        "cx": x + w / 2.0,
                        "cy": y + h / 2.0,
                    }
                )
        return detections

    def process_frame(self, frame):
        now = time.monotonic()
        h, w = frame.shape[:2]
        adapt_profile = self._resolve_adaptation_profile(frame)
        detect_frame = frame
        if CAM_ADAPT_ENABLED:
            detect_frame = cv2.convertScaleAbs(
                frame,
                alpha=float(adapt_profile.get("exposure_gain", 1.0)),
                beta=float(adapt_profile.get("exposure_beta", 0.0)),
            )
        detections = self._extract_detections(detect_frame, adapt_profile)

        for det in detections:
            color = COLOR_RANGES.get(det["name"], {}).get("color", (255, 255, 255))
            x, y, bw, bh = int(det["x"]), int(det["y"]), int(det["w"]), int(det["h"])
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(frame, det["name"], (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        red = [d for d in detections if d["name"] == "KIRMIZI_SANCAK"]
        green = [d for d in detections if d["name"] == "YESIL_SANCAK"]
        red_best = max(red, key=lambda d: d["area"]) if red else None
        green_best = max(green, key=lambda d: d["area"]) if green else None

        gate_detected = bool(red_best and green_best)
        gate_bearing = 0.0
        if gate_detected:
            center_x = (red_best["cx"] + green_best["cx"]) / 2.0
            gate_bearing = ((center_x - (w / 2.0)) / (w / 2.0)) * 35.0
            cv2.circle(frame, (int(center_x), int(h * 0.5)), 8, (255, 255, 0), -1)
            if self.gate_seen_since is None:
                self.gate_seen_since = now
            self.last_gate_seen_ts = now
            self.last_gate_stable = now - self.gate_seen_since
        else:
            if self.gate_seen_since is not None and (now - self.last_gate_seen_ts) <= 0.4 and self.last_gate_stable >= 1.0:
                self.gate_passed_until = now + 0.5
            self.gate_seen_since = None
            self.last_gate_stable = 0.0

        gate_passed_event = now < self.gate_passed_until

        target_pool = [d for d in detections if d["name"] == TARGET_CLASS]
        if not target_pool:
            target_pool = detections
        target = max(target_pool, key=lambda d: d["area"]) if target_pool else None

        target_detected = bool(target)
        target_bearing = 0.0
        target_area_norm = 0.0
        if target_detected:
            target_bearing = ((target["cx"] - (w / 2.0)) / (w / 2.0)) * 35.0
            target_area_norm = clamp((target["w"] * target["h"]) / float(w * h), 0.0, 1.0)
            cv2.circle(frame, (int(target["cx"]), int(target["cy"])), 8, (255, 255, 255), -1)

        frame_age = 999.0 if self.last_frame_ts <= 0.0 else max(0.0, now - self.last_frame_ts)
        self.status = {
            "ts_monotonic": round(now, 3),
            "frame_age_s": round(frame_age, 3),
            "gate_detected": gate_detected,
            "gate_stable_s": round(self.last_gate_stable, 3),
            "gate_center_bearing_deg": round(gate_bearing, 3),
            "gate_passed_event": gate_passed_event,
            "target_detected": target_detected,
            "target_bearing_error_deg": round(target_bearing, 3),
            "target_area_norm": round(target_area_norm, 4),
            "camera_adaptation": {
                "enabled": bool(adapt_profile.get("enabled", False)),
                "mode": str(adapt_profile.get("mode", "normal")),
                "luma_mean": round(float(adapt_profile.get("luma_mean", 0.0)), 2),
                "exposure_gain": round(float(adapt_profile.get("exposure_gain", 1.0)), 3),
                "exposure_beta": round(float(adapt_profile.get("exposure_beta", 0.0)), 3),
                "hsv_s_shift": int(adapt_profile.get("hsv_s_shift", 0)),
                "hsv_v_shift": int(adapt_profile.get("hsv_v_shift", 0)),
                "hsv_profile": str(adapt_profile.get("hsv_profile", "base")),
            },
        }
        self._write_status()

        ts_label = time.strftime("%H:%M:%S")
        cv2.rectangle(frame, (0, 0), (w, 36), (0, 0, 0), -1)
        cv2.putText(frame, f"REC {ts_label} | FILE1", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(
            frame,
            (
                f"ADAPT {adapt_profile['mode']} "
                f"L:{adapt_profile['luma_mean']:.0f} "
                f"G:{adapt_profile['exposure_gain']:.2f}"
            ),
            (440, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (180, 220, 255),
            2,
        )
        cv2.line(frame, (w // 2 - 20, h // 2), (w // 2 + 20, h // 2), (200, 200, 200), 2)
        cv2.line(frame, (w // 2, h // 2 - 20), (w // 2, h // 2 + 20), (200, 200, 200), 2)

        if self.writer:
            self.writer.write(frame)

        return frame

    def get_frame(self):
        if self.frame is None:
            sim = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(sim, "KAMERA BEKLENIYOR...", (430, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            self.status["ts_monotonic"] = round(time.monotonic(), 3)
            self.status["frame_age_s"] = 999.0
            self._write_status()
            ok, jpeg = cv2.imencode(".jpg", sim)
            return jpeg.tobytes() if ok else b""

        processed = self.process_frame(self.frame.copy())
        ok, jpeg = cv2.imencode(".jpg", processed, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        return jpeg.tobytes() if ok else b""

    def close(self):
        self.stopped = True
        if self.writer:
            self.writer.release()


app = Flask(__name__)
camera_stream = None


@app.route("/")
def video_feed():
    def generate():
        while True:
            frame = camera_stream.get_frame()
            if frame:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.033)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    clean_port(WEB_PORT)
    camera_stream = VideoCamera().start()

    try:
        if RACE_MODE:
            print("[CAM] [cam.py] Race modda sadece onboard isleme calisiyor")
            while True:
                if camera_stream.frame is not None:
                    camera_stream.process_frame(camera_stream.frame.copy())
                else:
                    camera_stream.status["ts_monotonic"] = round(time.monotonic(), 3)
                    camera_stream.status["frame_age_s"] = 999.0
                    camera_stream._write_status()
                time.sleep(0.1)
        else:
            print(f"[WEB] [CAM] WEB ARAYUZU: http://0.0.0.0:{WEB_PORT}")
            app.run(host="0.0.0.0", port=WEB_PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        camera_stream.close()

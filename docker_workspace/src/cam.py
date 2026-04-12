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
    CONTROL_DIR as DEFAULT_CONTROL_DIR,
    LOG_DIR as DEFAULT_LOG_DIR,
    USV_MODE,
    USV_MODE_RACE,
)
from mission_config import TARGET_STATE_FILE, load_target_state

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

print = make_console_printer("CAM")

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

RACE_MODE = USV_MODE == USV_MODE_RACE
if RACE_MODE:
    print("[RACE] [cam.py] YARISMA MODU - web yayini kapali, onboard isleme aktif")

CAM_SOURCE = os.environ.get("CAM_SOURCE", "hw")
BEARING_HALF_DEG = 45.0 if CAM_SOURCE == "sim" else 35.0

from runtime_debug_log import install_module_function_tracing, log_jsonl, setup_component_logger

_cam_file_log = setup_component_logger("cam")
_cam_file_log.info(
    "cam.py CAM_SOURCE=%s USV_MODE=%s LOG_DIR=%s",
    CAM_SOURCE,
    USV_MODE,
    os.environ.get("LOG_DIR"),
)

if CAM_SOURCE == "sim":
    print("[SIM] [cam.py] Otonomi Kamera Simulasyon modunda calisiyor...")
    HOST = "127.0.0.1"
else:
    HOST = os.environ.get("CAM_HOST", "127.0.0.1")

PORT = 8888
WEB_PORT = 5000
CONTROL_DIR = os.environ.get("CONTROL_DIR", DEFAULT_CONTROL_DIR)
STATUS_FILE = f"{CONTROL_DIR}/camera_status.json"
MISSION_STATE_FILE = f"{CONTROL_DIR}/mission_state.json"
os.makedirs(CONTROL_DIR, exist_ok=True)
VIDEO_DIR = os.path.join(os.environ.get("LOG_DIR", DEFAULT_LOG_DIR), "video")
FILE1_MP4 = f"{VIDEO_DIR}/file1_camera_processed.mp4"
os.makedirs(VIDEO_DIR, exist_ok=True)

COLOR_RANGES = {
    "TURUNCU_SINIR": {
        "lower": np.array([5, 65, 90]),
        "upper": np.array([22, 255, 255]),
        "color": (0, 140, 255),
    },
    "SARI_ENGEL": {
        "lower": np.array([22, 100, 100]),
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
        "upper": np.array([5, 255, 255]),
        "lower2": np.array([172, 150, 100]),
        "upper2": np.array([180, 255, 255]),
        "color": (0, 0, 255),
    },
    "SIYAH_HEDEF": {
        "lower": np.array([0, 0, 0]),
        "upper": np.array([180, 60, 45]),
        "color": (32, 32, 32),
    },
}

TARGET_CANDIDATE_CLASSES = (
    "KIRMIZI_SANCAK",
    "YESIL_SANCAK",
    "SIYAH_HEDEF",
)
TARGET_CLASS_ALIASES = {
    "RED": "KIRMIZI_SANCAK",
    "KIRMIZI": "KIRMIZI_SANCAK",
    "KIRMIZI_SANCAK": "KIRMIZI_SANCAK",
    "GREEN": "YESIL_SANCAK",
    "YESIL": "YESIL_SANCAK",
    "YESIL_SANCAK": "YESIL_SANCAK",
    "BLACK": "SIYAH_HEDEF",
    "SIYAH": "SIYAH_HEDEF",
    "SIYAH_HEDEF": "SIYAH_HEDEF",
}


def resolve_target_classes(raw_target):
    token = str(raw_target or "").strip().upper()
    resolved = TARGET_CLASS_ALIASES.get(token, token)
    if resolved in TARGET_CANDIDATE_CLASSES:
        return (resolved,)
    return TARGET_CANDIDATE_CLASSES


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
        self._last_writer_frame_ts = 0.0
        self.gate_seen_since = None
        self.last_gate_seen_ts = 0.0
        self.last_gate_stable = 0.0
        self.gate_passed_until = 0.0
        self.status = {
            "ts_monotonic": 0.0,
            "frame_age_s": 999.0,
            "recording": False,
            "recording_file": None,
            "target_class": "ANY",
            "objective_phase": "IDLE",
            "perception_policy": {
                "gate": False,
                "yellow_obstacle": False,
                "target": False,
            },
            "gate_actionable": False,
            "yellow_actionable": False,
            "target_actionable": False,
            "gate_detected_raw": False,
            "gate_stable_s_raw": 0.0,
            "gate_center_bearing_deg_raw": 0.0,
            "gate_passed_event_raw": False,
            "gate_detected": False,
            "gate_stable_s": 0.0,
            "gate_center_bearing_deg": 0.0,
            "gate_passed_event": False,
            "yellow_obstacle_detected_raw": False,
            "yellow_obstacle_bearing_deg_raw": 0.0,
            "yellow_obstacle_area_norm_raw": 0.0,
            "yellow_obstacle_detected": False,
            "yellow_obstacle_bearing_deg": 0.0,
            "yellow_obstacle_area_norm": 0.0,
            "target_detected_raw": False,
            "target_bearing_error_deg_raw": 0.0,
            "target_area_norm_raw": 0.0,
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
        self._recording_active = False
        self._last_mission_state = {"active": False}
        self._target_state_mtime = 0.0
        self._process_lock = threading.Lock()
        self._last_processed_frame_ts = 0.0
        self._last_processed_frame = None
        self._last_processed_jpeg = b""
        
        # Frame counting for debug logging
        self._frame_count = 0
        self._last_frame_log_time = time.monotonic()
        self._last_frame_log_count = 0
        self.target_classes = TARGET_CANDIDATE_CLASSES
        self.target_class = "ANY"
        self._set_target_classes(os.environ.get("TARGET_CLASS", "KIRMIZI_SANCAK"), source="env")

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

    def _set_target_classes(self, raw_target, source="target_state"):
        classes = tuple(resolve_target_classes(raw_target))
        if classes == tuple(self.target_classes):
            return
        self.target_classes = classes
        self.target_class = self.target_classes[0] if len(self.target_classes) == 1 else "ANY"
        print(f"[CAM] [TARGET] source={source} class={self.target_class}")

    def _refresh_target_class(self):
        try:
            mtime = os.path.getmtime(TARGET_STATE_FILE)
        except OSError:
            return
        if mtime <= float(self._target_state_mtime or 0.0):
            return
        try:
            target_state = load_target_state(TARGET_STATE_FILE)
            self._target_state_mtime = mtime
            self._set_target_classes(target_state.get("target_color"), source="target_state")
        except Exception as exc:
            self._warn_throttled("target_state_refresh", f"[WARN] [CAM] Target state okunamadi: {exc}")

    def _objective_phase_for_parkur(self, parkur_label):
        parkur = str(parkur_label or "IDLE").upper()
        mapping = {
            "P1": "P1_CORRIDOR",
            "P2": "P2_OBSTACLE_CORRIDOR",
            "P3": "P3_TARGET_ENGAGEMENT",
            "COMPLETED": "COMPLETED",
            "HOLD": "HOLD",
        }
        return mapping.get(parkur, "IDLE")

    def _default_perception_policy(self, parkur_label):
        parkur = str(parkur_label or "IDLE").upper()
        return {
            "gate": parkur in ("P1", "P2"),
            "yellow_obstacle": parkur == "P2",
            "target": parkur == "P3",
        }

    def _sync_perception_context(self, mission_state):
        state = mission_state if isinstance(mission_state, dict) else {}
        parkur_label = str(state.get("active_parkur", "IDLE") or "IDLE").upper()
        objective_phase = str(state.get("objective_phase", "") or "").upper()
        if not objective_phase:
            objective_phase = self._objective_phase_for_parkur(parkur_label)
        defaults = self._default_perception_policy(parkur_label)
        incoming_policy = state.get("perception_policy", {})
        if not isinstance(incoming_policy, dict):
            incoming_policy = {}
        resolved_policy = {
            key: bool(incoming_policy.get(key, default))
            for key, default in defaults.items()
        }
        self.status["objective_phase"] = objective_phase
        self.status["perception_policy"] = dict(resolved_policy)
        return resolved_policy

    def _check_mission_state(self):
        """Mission state'i oku ve recording kontrol et."""
        self._refresh_target_class()
        self._sync_perception_context(self._last_mission_state)
        try:
            if os.path.exists(MISSION_STATE_FILE):
                with open(MISSION_STATE_FILE, "r", encoding="utf-8") as f:
                    mission_state = json.load(f)
                    self._sync_perception_context(mission_state)
                    mission_active = mission_state.get("active", False)
                    
                    # Mission başlattıysa recording başlat
                    if mission_active and not self._recording_active:
                        self._recording_active = True
                        # Yeni video dosyası oluştur (timestamp ile)
                        ts = time.strftime("%Y%m%d_%H%M%S")
                        new_file = f"{VIDEO_DIR}/mission_{ts}.mp4"
                        if self.writer:
                            self.writer.release()
                        try:
                            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                            self.writer = cv2.VideoWriter(new_file, fourcc, 10.0, (1280, 720))
                            if self.writer.isOpened():
                                print(f"[REC] [CAM] Recording başladı: {new_file}")
                                self.status["recording"] = True
                                self.status["recording_file"] = new_file
                            else:
                                self._recording_active = False
                                self.status["recording"] = False
                        except Exception as e:
                            print(f"[WARN] [CAM] Recording başlama hatası: {e}")
                            self._recording_active = False
                            self.status["recording"] = False
                    
                    # Mission durduysa recording durdur
                    elif not mission_active and self._recording_active:
                        self._recording_active = False
                        if self.writer:
                            self.writer.release()
                            self.writer = None
                            print("[REC] [CAM] Recording durduruldu")
                        self.status["recording"] = False
                        
                    self._last_mission_state = mission_state
        except Exception as e:
            self._warn_throttled("mission_state_read", f"[WARN] [CAM] Mission state oku hatası: {e}")

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
        if not RACE_MODE:
            threading.Thread(target=self.processing_loop, daemon=True).start()
        return self

    def update(self):
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
            self.status["target_class"] = self.target_class
            
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
                if (old.get('objective_phase') == self.status['objective_phase'] and
                    old.get('perception_policy') == self.status['perception_policy'] and
                    old.get('gate_actionable') == self.status['gate_actionable'] and
                    old.get('yellow_actionable') == self.status['yellow_actionable'] and
                    old.get('target_actionable') == self.status['target_actionable'] and
                    old.get('gate_detected') == self.status['gate_detected'] and
                    old.get('gate_passed_event') == self.status['gate_passed_event'] and
                    old.get('yellow_obstacle_detected') == self.status['yellow_obstacle_detected'] and
                    old.get('target_detected') == self.status['target_detected'] and
                    abs(old.get('gate_center_bearing_deg', 0) - self.status['gate_center_bearing_deg']) < 0.1 and
                    abs(old.get('yellow_obstacle_bearing_deg', 0) - self.status['yellow_obstacle_bearing_deg']) < 0.1 and
                    abs(old.get('yellow_obstacle_area_norm', 0) - self.status['yellow_obstacle_area_norm']) < 0.01 and
                    abs(old.get('target_bearing_error_deg', 0) - self.status['target_bearing_error_deg']) < 0.1 and
                    abs(old.get('target_area_norm', 0) - self.status['target_area_norm']) < 0.01 and
                    old_adapt.get("mode") == new_adapt.get("mode") and
                    abs(float(old_adapt.get("exposure_gain", 1.0)) - float(new_adapt.get("exposure_gain", 1.0))) < 0.02 and
                    int(old_adapt.get("hsv_s_shift", 0)) == int(new_adapt.get("hsv_s_shift", 0)) and
                    int(old_adapt.get("hsv_v_shift", 0)) == int(new_adapt.get("hsv_v_shift", 0))):
                    logical_state_changed = False

            if logical_state_changed or (now - self._last_status_write_time >= 1.0):
                os.makedirs(os.path.dirname(STATUS_FILE) or ".", exist_ok=True)
                tmp_path = f"{STATUS_FILE}.tmp"
                with open(tmp_path, "w", encoding="utf-8") as f:
                    json.dump(self.status, f)
                os.replace(tmp_path, STATUS_FILE)
                
                self._last_status_write_time = now
                self._last_written_status = dict(self.status)
                
        except Exception as exc:
            self._bump_error("status_write_error", f"[WARN] [CAM] Status yazma hatasi: {exc}")

    def _process_latest_frame(self):
        frame = self.frame
        frame_ts = float(self.last_frame_ts or 0.0)
        if frame is None or frame_ts <= 0.0:
            return None
        with self._process_lock:
            if frame_ts <= float(self._last_processed_frame_ts or 0.0) and self._last_processed_frame is not None:
                return self._last_processed_frame
            processed = self.process_frame(frame.copy())
            self._last_processed_frame_ts = frame_ts
            self._last_processed_frame = processed
            ok, jpeg = cv2.imencode(".jpg", processed, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            self._last_processed_jpeg = jpeg.tobytes() if ok else b""
            return processed

    def processing_loop(self):
        while not self.stopped:
            try:
                # Mission state'i kontrol et (recording start/stop)
                self._check_mission_state()
                
                if self.frame is None:
                    self.status["ts_monotonic"] = round(time.monotonic(), 3)
                    self.status["frame_age_s"] = 999.0
                    self._write_status()
                else:
                    processed = self._process_latest_frame()
                    if processed is not None and self._recording_active and self.writer and self.last_frame_ts > self._last_writer_frame_ts:
                        self.writer.write(processed)
                        self._last_writer_frame_ts = self.last_frame_ts
            except Exception as exc:
                self._warn_throttled("process_loop", f"[WARN] [CAM] Isleme dongusu hatasi: {exc}")
            time.sleep(0.1)

    def _extract_detections(self, frame, adapt_profile):
        small = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        blur_ksize = 3 if CAM_SOURCE == "sim" else 5
        hsv = cv2.cvtColor(cv2.GaussianBlur(small, (blur_ksize, blur_ksize), 0), cv2.COLOR_BGR2HSV)
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
            
            # FIX (Phase 2a): Increase kernel for better noise removal
            # PREVIOUS: kernel = np.ones((3, 3), np.uint8)
            kernel_morph_open = np.ones((5, 5), np.uint8)  # Larger kernel for noise removal
            kernel_morph_close = np.ones((3, 3), np.uint8)  # Keep smaller for fill
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_morph_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_morph_close)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # FIX (Phase 2b): Filter merged contours by density
            # REASON: Contours with low area/bbox_area ratio are likely merged multiple buoys
            # SOLUTION: Skip contours with density < 0.3 (filled ratio indicates merging)
            filtered_contours = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 120:
                    continue
                
                # Density check: area / bounding_box_area
                # High density (>0.3) = single compact object
                # Low density (<0.3) = stretched/merged objects
                x, y, w, h = cv2.boundingRect(cnt)
                bbox_area = float(w * h + 1)
                density = area / bbox_area if bbox_area > 0 else 0.0
                
                # Skip if likely merged blob (very low density indicates spread across multiple objects)
                if density < 0.3:
                    continue  # Too spread out; likely multiple buoys merged
                
                filtered_contours.append(cnt)
            
            # Process filtered contours only
            for cnt in filtered_contours:
                area = cv2.contourArea(cnt)
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

    def _target_boxes_related(self, a, b):
        pad = max(18.0, 0.18 * max(a["w"], a["h"], b["w"], b["h"]))
        ax1, ay1 = float(a["x"]) - pad, float(a["y"]) - pad
        ax2, ay2 = float(a["x"] + a["w"]) + pad, float(a["y"] + a["h"]) + pad
        bx1, by1 = float(b["x"]) - pad, float(b["y"]) - pad
        bx2, by2 = float(b["x"] + b["w"]) + pad, float(b["y"] + b["h"]) + pad
        return not (ax2 < bx1 or bx2 < ax1 or ay2 < by1 or by2 < ay1)

    def _merge_target_clusters(self, detections):
        target_like = [dict(det) for det in detections if det["name"] in TARGET_CANDIDATE_CLASSES]
        if len(target_like) < 2:
            for det in target_like:
                det["member_names"] = (det["name"],)
            return [*([det for det in detections if det["name"] not in TARGET_CANDIDATE_CLASSES]), *target_like]

        merged = []
        used = [False] * len(target_like)
        for idx, det in enumerate(target_like):
            if used[idx]:
                continue
            cluster = [det]
            used[idx] = True
            changed = True
            while changed:
                changed = False
                for jdx, other in enumerate(target_like):
                    if used[jdx]:
                        continue
                    if any(self._target_boxes_related(member, other) for member in cluster):
                        cluster.append(other)
                        used[jdx] = True
                        changed = True

            if len(cluster) == 1:
                cluster[0]["member_names"] = (cluster[0]["name"],)
                merged.append(cluster[0])
                continue

            x1 = min(item["x"] for item in cluster)
            y1 = min(item["y"] for item in cluster)
            x2 = max(item["x"] + item["w"] for item in cluster)
            y2 = max(item["y"] + item["h"] for item in cluster)
            member_names = tuple(dict.fromkeys(item["name"] for item in cluster))
            primary = max(cluster, key=lambda item: item["area"])
            merged.append(
                {
                    "name": primary["name"],
                    "area": float(sum(item["area"] for item in cluster)),
                    "x": int(x1),
                    "y": int(y1),
                    "w": int(x2 - x1),
                    "h": int(y2 - y1),
                    "cx": (x1 + x2) / 2.0,
                    "cy": (y1 + y2) / 2.0,
                    "member_names": member_names,
                }
            )
            self._warn_throttled(
                "target_cluster_merge",
                f"[CAM] [MERGE] target_cluster={'+'.join(member_names)} size={len(cluster)}",
                period_s=1.5,
            )

        others = [dict(det) for det in detections if det["name"] not in TARGET_CANDIDATE_CLASSES]
        return others + merged

    def process_frame(self, frame):
        now = time.monotonic()
        self._frame_count += 1
        
        if (now - self._last_frame_log_time) >= 1.0:
            delta_frames = max(1, self._frame_count - self._last_frame_log_count)
            elapsed = now - self._last_frame_log_time
            fps = delta_frames / elapsed if elapsed > 0 else 0.0
            print(f"[CAM] [FRAME] count={self._frame_count} fps={fps:.1f}")
            self._last_frame_log_time = now
            self._last_frame_log_count = self._frame_count
        
        h, w = frame.shape[:2]
        adapt_profile = self._resolve_adaptation_profile(frame)
        if CAM_SOURCE == "sim":
            detect_frame = frame
            adapt_profile = {
                **adapt_profile,
                "mode": "sim_raw",
                "enabled": False,
                "exposure_gain": 1.0,
                "exposure_beta": 0.0,
                "hsv_s_shift": 0,
                "hsv_v_shift": 0,
                "hsv_profile": "sim_no_adapt",
            }
        else:
            detect_frame = frame
            if CAM_ADAPT_ENABLED:
                detect_frame = cv2.convertScaleAbs(
                    frame,
                    alpha=float(adapt_profile.get("exposure_gain", 1.0)),
                    beta=float(adapt_profile.get("exposure_beta", 0.0)),
                )
        detections = self._merge_target_clusters(self._extract_detections(detect_frame, adapt_profile))

        for det in detections:
            color = COLOR_RANGES.get(det["name"], {}).get("color", (255, 255, 255))
            x, y, bw, bh = int(det["x"]), int(det["y"]), int(det["w"]), int(det["h"])
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(frame, det["name"], (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        orange = [d for d in detections if d["name"] == "TURUNCU_SINIR"]
        orange_sorted = sorted(orange, key=lambda d: d["cx"])

        gate_detected_raw = len(orange_sorted) >= 2
        gate_bearing_raw = 0.0
        if gate_detected_raw:
            left_buoy = orange_sorted[0]
            right_buoy = orange_sorted[-1]
            center_x = (left_buoy["cx"] + right_buoy["cx"]) / 2.0
            gate_bearing_raw = ((center_x - (w / 2.0)) / (w / 2.0)) * BEARING_HALF_DEG
            cv2.circle(frame, (int(center_x), int(h * 0.5)), 8, (255, 255, 0), -1)
        elif len(orange_sorted) == 1:
            marker = orange_sorted[0]
            marker_bearing = ((marker["cx"] - (w / 2.0)) / (w / 2.0)) * BEARING_HALF_DEG
            marker_offset = 8.0 if marker["cx"] < (w / 2.0) else -8.0
            gate_bearing_raw = clamp(marker_bearing + marker_offset, -BEARING_HALF_DEG, BEARING_HALF_DEG)
            gate_detected_raw = True
            cv2.circle(frame, (int(marker["cx"]), int(marker["cy"])), 8, (255, 255, 0), -1)

        if gate_detected_raw:
            if self.gate_seen_since is None:
                self.gate_seen_since = now
            self.last_gate_seen_ts = now
            self.last_gate_stable = now - self.gate_seen_since
        else:
            if self.gate_seen_since is not None and (now - self.last_gate_seen_ts) <= 0.4 and self.last_gate_stable >= 1.0:
                self.gate_passed_until = now + 0.5
            self.gate_seen_since = None
            self.last_gate_stable = 0.0

        gate_passed_event_raw = now < self.gate_passed_until

        yellow_pool = [d for d in detections if d["name"] == "SARI_ENGEL"]
        yellow = max(yellow_pool, key=lambda d: d["area"]) if yellow_pool else None
        yellow_detected_raw = bool(yellow)
        yellow_bearing_raw = 0.0
        yellow_area_norm_raw = 0.0
        if yellow_detected_raw:
            yellow_bearing_raw = ((yellow["cx"] - (w / 2.0)) / (w / 2.0)) * BEARING_HALF_DEG
            yellow_area_norm_raw = clamp((yellow["w"] * yellow["h"]) / float(w * h), 0.0, 1.0)
            cv2.circle(frame, (int(yellow["cx"]), int(yellow["cy"])), 7, (0, 255, 255), 2)

        target_pool = [
            d for d in detections
            if any(name in self.target_classes for name in d.get("member_names", (d["name"],)))
        ]
        target = max(target_pool, key=lambda d: d["area"]) if target_pool else None

        target_detected_raw = bool(target)
        target_bearing_raw = 0.0
        target_area_norm_raw = 0.0
        if target_detected_raw:
            target_bearing_raw = ((target["cx"] - (w / 2.0)) / (w / 2.0)) * BEARING_HALF_DEG
            target_area_norm_raw = clamp((target["w"] * target["h"]) / float(w * h), 0.0, 1.0)
            cv2.circle(frame, (int(target["cx"]), int(target["cy"])), 8, (255, 255, 255), -1)

        current_status = self.status if isinstance(self.status, dict) else {}
        policy = current_status.get("perception_policy", {})
        if not isinstance(policy, dict):
            policy = {}

        gate_actionable = bool(policy.get("gate", False))
        yellow_actionable = bool(policy.get("yellow_obstacle", False))
        target_actionable = bool(policy.get("target", False))

        gate_detected = gate_detected_raw if gate_actionable else False
        gate_stable_s = self.last_gate_stable if gate_actionable else 0.0
        gate_bearing = gate_bearing_raw if gate_actionable else 0.0
        gate_passed_event = gate_passed_event_raw if gate_actionable else False
        yellow_detected = yellow_detected_raw if yellow_actionable else False
        yellow_bearing = yellow_bearing_raw if yellow_actionable else 0.0
        yellow_area_norm = yellow_area_norm_raw if yellow_actionable else 0.0
        target_detected = target_detected_raw if target_actionable else False
        target_bearing = target_bearing_raw if target_actionable else 0.0
        target_area_norm = target_area_norm_raw if target_actionable else 0.0

        if target_actionable:
            if target_detected:
                self._warn_throttled(
                    "target_found",
                    (
                        f"[CAM] [DETECT] target_found={target['name']} "
                        f"area_norm={target_area_norm:.4f} bearing={target_bearing:.1f}deg"
                    ),
                    period_s=1.0,
                )
            else:
                self._warn_throttled(
                    "target_missing",
                    f"[CAM] [DETECT] no_target_detected frame={self._frame_count} target={self.target_class}",
                    period_s=3.0,
                )

        frame_age = 999.0 if self.last_frame_ts <= 0.0 else max(0.0, now - self.last_frame_ts)
        self.status = {
            "ts_monotonic": round(now, 3),
            "frame_age_s": round(frame_age, 3),
            "recording": bool(current_status.get("recording", False)),
            "recording_file": current_status.get("recording_file"),
            "target_class": self.target_class,
            "objective_phase": str(current_status.get("objective_phase", "IDLE")),
            "perception_policy": {
                "gate": gate_actionable,
                "yellow_obstacle": yellow_actionable,
                "target": target_actionable,
            },
            "gate_actionable": gate_actionable,
            "yellow_actionable": yellow_actionable,
            "target_actionable": target_actionable,
            "gate_detected_raw": gate_detected_raw,
            "gate_stable_s_raw": round(self.last_gate_stable, 3),
            "gate_center_bearing_deg_raw": round(gate_bearing_raw, 3),
            "gate_passed_event_raw": gate_passed_event_raw,
            "gate_detected": gate_detected,
            "gate_stable_s": round(gate_stable_s, 3),
            "gate_center_bearing_deg": round(gate_bearing, 3),
            "gate_passed_event": gate_passed_event,
            "yellow_obstacle_detected_raw": yellow_detected_raw,
            "yellow_obstacle_bearing_deg_raw": round(yellow_bearing_raw, 3),
            "yellow_obstacle_area_norm_raw": round(yellow_area_norm_raw, 4),
            "yellow_obstacle_detected": yellow_detected,
            "yellow_obstacle_bearing_deg": round(yellow_bearing, 3),
            "yellow_obstacle_area_norm": round(yellow_area_norm, 4),
            "target_detected_raw": target_detected_raw,
            "target_bearing_error_deg_raw": round(target_bearing_raw, 3),
            "target_area_norm_raw": round(target_area_norm_raw, 4),
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

        return frame

    def get_frame(self):
        if self.frame is None:
            sim = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(sim, "KAMERA BEKLENIYOR...", (430, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            self.status["ts_monotonic"] = round(time.monotonic(), 3)
            self.status["frame_age_s"] = 999.0
            self.status["target_class"] = self.target_class
            self._write_status()
            ok, jpeg = cv2.imencode(".jpg", sim)
            return jpeg.tobytes() if ok else b""

        self._process_latest_frame()
        return self._last_processed_jpeg

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
            time.sleep(0.02)

    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


def run_headless_processing_loop():
    _loop_n = 0
    _last_loop_log = 0.0
    _jsonl_cam_ts = 0.0
    while True:
        # Mission state'i kontrol et
        camera_stream._check_mission_state()
        
        if camera_stream.frame is not None:
            processed = camera_stream._process_latest_frame()
            if processed is not None and camera_stream._recording_active and camera_stream.writer and camera_stream.last_frame_ts > camera_stream._last_writer_frame_ts:
                camera_stream.writer.write(processed)
                camera_stream._last_writer_frame_ts = camera_stream.last_frame_ts
        else:
            camera_stream.status["ts_monotonic"] = round(time.monotonic(), 3)
            camera_stream.status["frame_age_s"] = 999.0
            camera_stream.status["target_class"] = camera_stream.target_class
            camera_stream._write_status()
        _loop_n += 1
        now = time.monotonic()
        if _loop_n <= 3 or now - _last_loop_log >= 5.0:
            _last_loop_log = now
            _cam_file_log.debug(
                "headless_loop n=%s frame=%s frame_age_s=%s",
                _loop_n,
                camera_stream.frame is not None,
                camera_stream.status.get("frame_age_s"),
            )
            _tnow = time.monotonic()
            if _tnow - _jsonl_cam_ts >= 0.5:
                _jsonl_cam_ts = _tnow
                log_jsonl(
                    "cam",
                    False,
                    event="headless_tick",
                    n=_loop_n,
                    has_frame=camera_stream.frame is not None,
                    frame_age_s=camera_stream.status.get("frame_age_s"),
                )
        time.sleep(0.1)

install_module_function_tracing(
    globals(),
    component="cam",
    logger=_cam_file_log,
    prefer_simulation=bool(os.environ.get("USV_SIM") == "1"),
)


if __name__ == "__main__":
    clean_port(WEB_PORT)
    camera_stream = VideoCamera().start()

    try:
        if RACE_MODE:
            print("[CAM] [cam.py] Race modda sadece onboard isleme calisiyor")
            run_headless_processing_loop()
        else:
            print(f"[WEB] [CAM] WEB ARAYUZU: http://0.0.0.0:{WEB_PORT}")
            try:
                app.run(host="0.0.0.0", port=WEB_PORT, debug=False, threaded=True)
            except OSError as exc:
                print(f"[WARN] [CAM] Web arayuzu acilamadi, headless moda geciliyor: {exc}")
                run_headless_processing_loop()
    except KeyboardInterrupt:
        pass
    finally:
        camera_stream.close()

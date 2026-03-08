"""
CELEBILER USV - Kritik Rapor Uyum Durum Makinesi
"""

import csv
import glob
import json
import math
import os
import random
import sys
import threading
import time

from console_utils import make_console_printer
from compliance_profile import (
    CAMERA_FRAME_TIMEOUT_S,
    COMMS_POLICY,
    CONTROL_HZ,
    CONTROL_DIR,
    D_MIN_M,
    FAILSAFE_SLOW_MPS,
    HEARTBEAT_FAIL_S,
    HEARTBEAT_WARN_S,
    LIDAR_READY_TIMEOUT_S,
    LINK_TOPOLOGY,
    LOG_DIR,
    P1_SPEED_APPROACH_MPS,
    P1_SPEED_CRUISE_MPS,
    P2_CRUISE_MPS,
    P2_GATE_CONFIRM_S,
    P2_STABLE_S,
    P2_WAIT_SPEED_MPS,
    P3_MAX_SPEED_MPS,
    P3_RETRY_COUNT,
    P3_RETRY_S,
    P3_REVERSE_DISTANCE_M,
    P3_REVERSE_SPEED_MPS,
    P3_REVERSE_TIMEOUT_S,
    P3_TIMEOUT_S,
    R_WP_M,
    RC7_ESTOP_FORCE_PWM,
    RC7_ESTOP_PWM,
    RC7_SAFE_PWM,
    RC_RACE_START_PWM,
    REPORT_TELEMETRY_GROUPS,
    T_HOLD_S,
    USV_MODE,
    USV_MODE_RACE,
    evaluate_readiness_flags,
    evaluate_storage_health,
)

print = make_console_printer("USV")

BAUD_RATE = 115200
LOOP_DT = 1.0 / CONTROL_HZ

MISSION_FILE = os.environ.get("MISSION_FILE", "/root/workspace/mission.json")

FLAG_START = f"{CONTROL_DIR}/start_mission.flag"
FLAG_STOP = f"{CONTROL_DIR}/emergency_stop.flag"
STATE_FILE = f"{CONTROL_DIR}/mission_state.json"
LINK_STATE_FILE = f"{CONTROL_DIR}/telemetry_link_state.json"
CAMERA_STATUS_FILE = f"{CONTROL_DIR}/camera_status.json"

FILE3_MAP_MP4 = f"{LOG_DIR}/file3_local_map.mp4"
FILE3_INDEX_CSV = f"{LOG_DIR}/file3_local_map_index.csv"


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def haversine_distance(lat1, lon1, lat2, lon2):
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (
        math.sin(d_lat / 2) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(d_lon / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return 6371000 * c


def calculate_bearing(lat1, lon1, lat2, lon2):
    d_lon = math.radians(lon2 - lon1)
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    y = math.sin(d_lon) * math.cos(lat2_r)
    x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(d_lon)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def normalize_heading_error(error):
    if error > 180:
        error -= 360
    if error < -180:
        error += 360
    return error


class USVStateMachine:
    STATE_IDLE = 0
    STATE_PARKUR1 = 1
    STATE_PARKUR2 = 2
    STATE_PARKUR3 = 3
    STATE_COMPLETED = 4
    STATE_HOLD = 5

    def __init__(self):
        print("=" * 60)
        print("  CELEBILER USV - KRITIK UYUM MODU")
        print("=" * 60)
        print(f"[INFO] MOD: {'[RACE] YARISMA' if USV_MODE == USV_MODE_RACE else '[TEST] TEST'}")

        self.simulation_mode = False
        self.state = self.STATE_IDLE
        self.mission_active = False
        self.mission_start_time = 0.0
        self.command_lock = False

        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_heading = 0.0
        self.rc_channels = {f"ch{i}": 0 for i in range(1, 9)}

        self.v_target = 0.0
        self.heading_target = 0.0

        self.camera_status = {}
        self.camera_ready = False
        self.lidar_available = False
        self.lidar_ready = False
        self.last_lidar_time = 0.0
        self.lidar_left_dist = 99.0
        self.lidar_center_dist = 99.0
        self.lidar_right_dist = 99.0
        self.min_obstacle_distance = 99.0
        self.obstacle_detected = False
        self._map_points = []

        self.last_heartbeat_time = time.monotonic()
        self.heartbeat_age_s = 0.0
        self.link_heartbeat_age_s = 0.0
        self.link_heartbeat_source = "onboard_fallback"
        self.failsafe_state = "normal"
        self.timeout_count = 0
        self.error_counters = {
            "state_write_error": 0,
            "mav_read_error": 0,
            "camera_state_read_error": 0,
            "link_state_read_error": 0,
        }
        self._warn_last = {}
        self._last_race_flag_purge_log = 0.0

        self.estop_latched = False
        self.estop_source = ""

        self.waypoints_p1 = []
        self.waypoints_p2 = []
        self.waypoints_p3 = []
        self.target_color = ""
        self.gate_count = 0
        self._gate_event_start = None
        self._gate_event_latched = False
        self._wp_target = "--"
        self._wp_info = "-- / --"

        self.health_flags = {}
        self.health_missing = []
        self.health_storage = {}
        self.health_ready = False
        self.health_checked_at = 0.0

        self._cv2 = None
        self._file3_writer = None
        self._file3_index_file = None
        self._file3_index_writer = None
        self._file3_last_ts = 0.0

        self.master = None
        self._connect_pixhawk()
        self._try_connect_lidar()
        self._init_file3_recorder()
        self._refresh_health_check()
        self._write_state()

    def _active_parkur_label(self):
        mapping = {
            self.STATE_IDLE: "IDLE",
            self.STATE_PARKUR1: "P1",
            self.STATE_PARKUR2: "P2",
            self.STATE_PARKUR3: "P3",
            self.STATE_COMPLETED: "COMPLETED",
            self.STATE_HOLD: "HOLD",
        }
        return mapping.get(self.state, "UNKNOWN")

    def _rc_link_active(self):
        if self.simulation_mode:
            return True
        return any(900 <= self.rc_channels.get(f"ch{i}", 0) <= 2100 for i in range(1, 5))

    def _warn_throttled(self, key, message, period_s=5.0):
        now = time.monotonic()
        last = self._warn_last.get(key, 0.0)
        if now - last >= period_s:
            print(message)
            self._warn_last[key] = now

    def _bump_error(self, key, message=None, period_s=5.0):
        self.error_counters[key] = int(self.error_counters.get(key, 0)) + 1
        if message:
            self._warn_throttled(f"err_{key}", f"{message} (count={self.error_counters[key]})", period_s=period_s)

    def _refresh_link_heartbeat(self):
        onboard_age = max(0.0, time.monotonic() - self.last_heartbeat_time)
        if self.simulation_mode or not self.master:
            self.link_heartbeat_age_s = 0.0
            self.link_heartbeat_source = "simulation"
            return

        link_age = None
        try:
            if os.path.exists(LINK_STATE_FILE):
                with open(LINK_STATE_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    link_age = float(data.get("link_heartbeat_age_s", onboard_age))
                    ts = float(data.get("ts_monotonic", 0.0))
                    if ts > 0.0:
                        link_age += max(0.0, time.monotonic() - ts)
                    link_age = max(0.0, link_age)
                    self.link_heartbeat_source = "telemetry"
                else:
                    self.link_heartbeat_source = "onboard_fallback"
            else:
                self.link_heartbeat_source = "onboard_fallback"
        except Exception as exc:
            self._bump_error("link_state_read_error", f"[WARN] [LINK] Link state okuma hatasi: {exc}")
            self.link_heartbeat_source = "onboard_fallback"
            link_age = None

        if link_age is None:
            link_age = onboard_age

        self.link_heartbeat_age_s = float(link_age)

    def _refresh_health_check(self):
        if self.simulation_mode:
            mavlink_ok = True
            heartbeat_ok = True
            rc_ok = True
        else:
            mavlink_ok = bool(self.master is not None)
            heartbeat_ok = bool(self.link_heartbeat_age_s < HEARTBEAT_WARN_S)
            rc_ok = self._rc_link_active()

        estop_safe = bool((not self.estop_latched) and self.rc_channels.get("ch7", 0) <= RC7_SAFE_PWM and not os.path.exists(FLAG_STOP))
        storage_health = evaluate_storage_health(USV_MODE, local_dir=LOG_DIR)
        flags, missing = evaluate_readiness_flags(
            mode=USV_MODE,
            mavlink_vehicle_link=mavlink_ok,
            telemetry_heartbeat_ok=heartbeat_ok,
            rc_link_active=rc_ok,
            estop_safe=estop_safe,
            camera_fresh=self.camera_ready,
            lidar_fresh=self.lidar_ready,
            storage_health=storage_health,
        )
        self.health_flags = flags
        self.health_missing = missing
        self.health_storage = storage_health
        self.health_ready = not missing
        self.health_checked_at = round(time.monotonic(), 3)

    def _write_state(self):
        try:
            os.makedirs(CONTROL_DIR, exist_ok=True)
            payload = {
                "ts_monotonic": round(time.monotonic(), 3),
                "state": self.state,
                "active": self.mission_active,
                "start_time": self.mission_start_time,
                "target": self._wp_target,
                "wp_info": self._wp_info,
                "active_parkur": self._active_parkur_label(),
                "gate_count": self.gate_count,
                "command_lock": self.command_lock,
                "failsafe_state": self.failsafe_state,
                "estop_state": self.estop_latched,
                "estop_source": self.estop_source or "--",
                "camera_ready": self.camera_ready,
                "lidar_ready": self.lidar_ready,
                "heartbeat_age_s": round(self.heartbeat_age_s, 3),
                "link_heartbeat_age_s": round(self.link_heartbeat_age_s, 3),
                "link_heartbeat_source": self.link_heartbeat_source,
                "v_target": round(self.v_target, 3),
                "heading_target": round(self.heading_target, 3),
                "timeout_count": self.timeout_count,
                "error_counters": dict(self.error_counters),
                "health_check": {
                    "ready": self.health_ready,
                    "missing": self.health_missing,
                    "flags": self.health_flags,
                    "storage": self.health_storage,
                    "checked_at_monotonic": self.health_checked_at,
                },
                "ready_state": self.health_ready,
                "ready_missing": self.health_missing,
                "telemetry_groups": REPORT_TELEMETRY_GROUPS,
                "link_topology": LINK_TOPOLOGY,
                "comms_policy": COMMS_POLICY,
            }

            tmp_path = f"{STATE_FILE}.tmp"
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(payload, f)
            os.replace(tmp_path, STATE_FILE)
        except Exception as exc:
            self._bump_error("state_write_error", f"[WARN] [STATE] Yazma hatasi: {exc}")

    def _init_file3_recorder(self):
        try:
            import cv2

            self._cv2 = cv2
            os.makedirs(LOG_DIR, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._file3_writer = cv2.VideoWriter(FILE3_MAP_MP4, fourcc, 1.0, (600, 600))
            if not self._file3_writer or not self._file3_writer.isOpened():
                print("[WARN] [FILE3] VideoWriter acilamadi.")
                self._file3_writer = None
            self._file3_index_file = open(FILE3_INDEX_CSV, "w", newline="", encoding="utf-8")
            self._file3_index_writer = csv.writer(self._file3_index_file)
            self._file3_index_writer.writerow(
                ["ts_monotonic", "lat", "lon", "heading", "left_m", "center_m", "right_m"]
            )
            self._file3_index_file.flush()
            print(f"[REC] [FILE3] Harita kaydi: {FILE3_MAP_MP4}")
            print(f"[IDX] [FILE3] Indeks: {FILE3_INDEX_CSV}")
        except Exception as exc:
            print(f"[WARN] [FILE3] Baslatma hatasi: {exc}")
            self._file3_writer = None
            self._file3_index_file = None
            self._file3_index_writer = None

    def _close_file3_recorder(self):
        try:
            if self._file3_writer:
                self._file3_writer.release()
        except Exception as exc:
            self._warn_throttled("file3_close_writer", f"[WARN] [FILE3] Writer kapatma hatasi: {exc}")
        try:
            if self._file3_index_file:
                self._file3_index_file.close()
        except Exception as exc:
            self._warn_throttled("file3_close_index", f"[WARN] [FILE3] Index kapatma hatasi: {exc}")

    def _record_file3_if_due(self):
        now = time.monotonic()
        if now - self._file3_last_ts < 1.0:
            return
        self._file3_last_ts = now

        if self._file3_index_writer:
            self._file3_index_writer.writerow(
                [
                    round(now, 3),
                    round(self.current_lat, 7),
                    round(self.current_lon, 7),
                    round(self.current_heading, 2),
                    round(self.lidar_left_dist, 2),
                    round(self.lidar_center_dist, 2),
                    round(self.lidar_right_dist, 2),
                ]
            )
            self._file3_index_file.flush()

        if not self._file3_writer or not self._cv2:
            return

        cv2 = self._cv2
        img = cv2.cvtColor(cv2.UMat(600, 600, cv2.CV_8UC1).get(), cv2.COLOR_GRAY2BGR)
        cx, cy = 300, 300
        cv2.arrowedLine(img, (cx, cy + 16), (cx, cy - 26), (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)

        scale = 20.0
        for x_m, y_m in self._map_points[:2500]:
            px = int(cx - y_m * scale)
            py = int(cy - x_m * scale)
            if 0 <= px < 600 and 0 <= py < 600:
                img[py, px] = (255, 255, 255)

        cv2.putText(
            img,
            f"L:{self.lidar_left_dist:.1f} C:{self.lidar_center_dist:.1f} R:{self.lidar_right_dist:.1f}",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
        )
        cv2.putText(
            img,
            f"TS:{time.strftime('%H:%M:%S')}",
            (12, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (180, 180, 180),
            1,
        )
        self._file3_writer.write(img)

    def _find_pixhawk_port(self):
        try:
            from pymavlink import mavutil

            m = mavutil.mavlink_connection("udpin:0.0.0.0:14551", baud=BAUD_RATE)
            if m.wait_heartbeat(timeout=3):
                print("[OK] [MAV] Pixhawk UDP:14551")
                return m
            m.close()
        except Exception as exc:
            self._bump_error("mav_read_error", f"[WARN] [MAV] Mesaj drain hatasi: {exc}")

        for port in glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"):
            try:
                from pymavlink import mavutil

                m = mavutil.mavlink_connection(port, baud=BAUD_RATE)
                if m.wait_heartbeat(timeout=2):
                    print(f"[OK] [MAV] Pixhawk seri: {port}")
                    return m
                m.close()
            except Exception:
                pass
        return None

    def _connect_pixhawk(self):
        print("[MAV] Pixhawk aranıyor...")
        try:
            self.master = self._find_pixhawk_port()
            if not self.master:
                raise RuntimeError("Pixhawk bulunamadi")
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                4,
                10,
                1,
            )
            self.last_heartbeat_time = time.monotonic()
            print("[OK] [MAV] Baglanti hazir")
        except Exception as exc:
            print(f"[WARN] [MAV] Baglanti yok: {exc}")
            print("[WARN] DONANIM BULUNAMADI - SIMULASYON MODU AKTIF")
            self.master = None
            self.simulation_mode = True

    def _try_connect_lidar(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import LaserScan

            try:
                rclpy.init()
            except RuntimeError:
                pass

            class _LidarNode(Node):
                def __init__(self, parent):
                    super().__init__("usv_lidar_listener")
                    self.parent = parent
                    qos = QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=10,
                    )
                    self.create_subscription(LaserScan, "/scan", self.cb, qos)

                def cb(self, msg):
                    self.parent.last_lidar_time = time.monotonic()
                    left = 99.0
                    center = 99.0
                    right = 99.0
                    points = []
                    angle = msg.angle_min
                    for rng in msg.ranges:
                        if 0.15 < rng < 20.0:
                            deg = math.degrees(angle)
                            if -90 <= deg <= 90:
                                x_m = rng * math.cos(angle)
                                y_m = rng * math.sin(angle)
                                points.append((x_m, y_m))
                                if deg > 20:
                                    left = min(left, rng)
                                elif deg < -20:
                                    right = min(right, rng)
                                else:
                                    center = min(center, rng)
                        angle += msg.angle_increment
                    self.parent.lidar_left_dist = left
                    self.parent.lidar_center_dist = center
                    self.parent.lidar_right_dist = right
                    self.parent.min_obstacle_distance = min(left, center, right)
                    self.parent.obstacle_detected = center < D_MIN_M
                    self.parent._map_points = points

            node = _LidarNode(self)

            def spin():
                try:
                    rclpy.spin(node)
                except Exception:
                    pass

            t = threading.Thread(target=spin, daemon=True)
            t.start()
            self.lidar_available = True
            print("[OK] [LIDAR] Dinleniyor")
        except ImportError:
            print("[WARN] [LIDAR] rclpy yok - lidar devre disi")
            self.lidar_available = False
        except Exception as exc:
            print(f"[WARN] [LIDAR] Hata: {exc}")
            self.lidar_available = False

    def _set_mode(self, mode_name):
        if self.simulation_mode or not self.master:
            return True
        try:
            from pymavlink import mavutil

            mapping = self.master.mode_mapping() or {}
            custom_mode = mapping.get(mode_name)
            if custom_mode is None:
                print(f"[WARN] [MODE] {mode_name} bulunamadi")
                return False
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
            )
            print(f"[MODE] [MODE] {mode_name}")
            return True
        except Exception as exc:
            print(f"[WARN] [MODE] Hata ({mode_name}): {exc}")
            return False

    def _arm(self):
        if self.simulation_mode or not self.master:
            return True
        try:
            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            print("[OK] [ARM] Arac armed")
            return True
        except Exception as exc:
            print(f"[WARN] [ARM] Hata: {exc}")
            return False

    def _disarm(self):
        if self.simulation_mode or not self.master:
            return True
        try:
            self.master.arducopter_disarm()
            return True
        except Exception:
            return False

    def _set_rc7_estop(self, enabled):
        if self.simulation_mode or not self.master:
            return True
        pwm = RC7_ESTOP_FORCE_PWM if enabled else RC7_SAFE_PWM
        try:
            for _ in range(5):
                rc = [65535] * 8
                rc[6] = pwm
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    *rc,
                )
                time.sleep(0.04)
            return True
        except Exception as exc:
            print(f"[WARN] [RC7] E-stop komutu hatasi: {exc}")
            return False

    def _set_motor_raw(self, left_pwm, right_pwm):
        left_pwm = int(clamp(left_pwm, 1100, 1900))
        right_pwm = int(clamp(right_pwm, 1100, 1900))
        if self.simulation_mode or not self.master:
            return
        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                left_pwm,
                65535,
                right_pwm,
                65535,
                65535,
                65535,
                65535,
                65535,
            )
        except Exception as exc:
            print(f"[WARN] [MOTOR] Override hatasi: {exc}")

    def _command_speed_heading(self, speed_mps, heading_error_deg):
        self.v_target = float(speed_mps)
        self.heading_target = (self.current_heading + heading_error_deg) % 360
        base_pwm = 1500 + (speed_mps * 170.0)
        turn_pwm = clamp(heading_error_deg * 2.4, -220, 220)
        self._set_motor_raw(base_pwm + turn_pwm, base_pwm - turn_pwm)

    def _command_reverse(self, reverse_speed_mps):
        self.v_target = -abs(reverse_speed_mps)
        self.heading_target = self.current_heading
        base_pwm = 1500 - (abs(reverse_speed_mps) * 170.0)
        self._set_motor_raw(base_pwm, base_pwm)

    def stop_motors(self):
        self.v_target = 0.0
        self._set_motor_raw(1500, 1500)
        self._disarm()

    def _consume_flag(self, path):
        if not os.path.exists(path):
            return False
        try:
            os.remove(path)
            return True
        except Exception as exc:
            print(f"[WARN] [FLAG] Silme hatasi ({path}): {exc}")
            return False

    def _drain_mav_messages(self):
        if self.simulation_mode or not self.master:
            return
        try:
            while True:
                msg = self.master.recv_match(blocking=False)
                if not msg:
                    break
                mtype = msg.get_type()
                if mtype == "GLOBAL_POSITION_INT":
                    self.current_lat = msg.lat / 1e7
                    self.current_lon = msg.lon / 1e7
                    self.current_heading = msg.hdg / 100.0
                elif mtype == "HEARTBEAT":
                    if msg.get_srcSystem() == self.master.target_system:
                        self.last_heartbeat_time = time.monotonic()
                elif mtype == "RC_CHANNELS":
                    self.rc_channels["ch1"] = getattr(msg, "chan1_raw", 0)
                    self.rc_channels["ch2"] = getattr(msg, "chan2_raw", 0)
                    self.rc_channels["ch3"] = getattr(msg, "chan3_raw", 0)
                    self.rc_channels["ch4"] = getattr(msg, "chan4_raw", 0)
                    self.rc_channels["ch5"] = getattr(msg, "chan5_raw", 0)
                    self.rc_channels["ch6"] = getattr(msg, "chan6_raw", 0)
                    self.rc_channels["ch7"] = getattr(msg, "chan7_raw", 0)
                    self.rc_channels["ch8"] = getattr(msg, "chan8_raw", 0)
                    if self.rc_channels["ch7"] >= RC7_ESTOP_PWM:
                        self._trigger_estop("RC7", force_rc7=False)
        except Exception as exc:
            self._bump_error("mav_read_error", f"[WARN] [MAV] Mesaj drain hatasi: {exc}")

    def _read_camera_status(self):
        default = {
            "ts_monotonic": 0.0,
            "frame_age_s": 999.0,
            "gate_detected": False,
            "gate_stable_s": 0.0,
            "gate_center_bearing_deg": 0.0,
            "gate_passed_event": False,
            "target_detected": False,
            "target_bearing_error_deg": 0.0,
            "target_area_norm": 0.0,
        }
        data = default
        try:
            if os.path.exists(CAMERA_STATUS_FILE):
                with open(CAMERA_STATUS_FILE, "r", encoding="utf-8") as f:
                    loaded = json.load(f)
                data = {**default, **loaded}
        except Exception as exc:
            self._bump_error("camera_state_read_error", f"[WARN] [CAM] Status okuma hatasi: {exc}")
            data = default
        self.camera_status = data
        self.camera_ready = bool(data.get("frame_age_s", 999.0) < CAMERA_FRAME_TIMEOUT_S)
        if self.simulation_mode:
            self.camera_ready = True
            self.lidar_ready = True
        else:
            self.lidar_ready = bool(
                self.lidar_available and (time.monotonic() - self.last_lidar_time) < LIDAR_READY_TIMEOUT_S
            )

    def _update_watchdog(self):
        if self.simulation_mode or not self.master:
            self.heartbeat_age_s = 0.0
            self.link_heartbeat_age_s = 0.0
            self.link_heartbeat_source = "simulation"
            self.failsafe_state = "normal"
            return

        self.heartbeat_age_s = time.monotonic() - self.last_heartbeat_time
        self._refresh_link_heartbeat()

        link_fail = self.link_heartbeat_age_s >= HEARTBEAT_FAIL_S
        onboard_fail = self.heartbeat_age_s >= HEARTBEAT_FAIL_S
        link_warn = self.link_heartbeat_age_s >= HEARTBEAT_WARN_S
        onboard_warn = self.heartbeat_age_s >= HEARTBEAT_WARN_S

        if link_fail and self.link_heartbeat_source == "telemetry" and self.failsafe_state != "hold":
            print("[ESTOP] [FAILSAFE] Telemetri link heartbeat timeout >=30s")
            self.failsafe_state = "triggered"
            if self.mission_active:
                self._command_speed_heading(FAILSAFE_SLOW_MPS, 0.0)
                time.sleep(1.0)
            self.failsafe_state = "hold"
            self._enter_hold("FAILSAFE_LINK_HEARTBEAT")
        elif onboard_fail and self.failsafe_state != "hold":
            print("[ESTOP] [FAILSAFE] Onboard heartbeat timeout >=30s")
            self.failsafe_state = "triggered"
            if self.mission_active:
                self._command_speed_heading(FAILSAFE_SLOW_MPS, 0.0)
                time.sleep(1.0)
            self.failsafe_state = "hold"
            self._enter_hold("FAILSAFE_ONBOARD_HEARTBEAT")
        elif link_warn or onboard_warn:
            self.failsafe_state = "warning"
        else:
            self.failsafe_state = "normal"

    def _trigger_estop(self, source, force_rc7=False):
        if self.estop_latched:
            return
        print(f"[ESTOP] [ESTOP] Tetiklendi: {source}")
        self.estop_latched = True
        self.estop_source = source
        self.command_lock = True
        if force_rc7:
            self._set_rc7_estop(True)
        self._enter_hold("ESTOP")

    def _clear_estop_if_safe(self):
        if not self.estop_latched:
            return True
        if self.rc_channels.get("ch7", 0) <= RC7_SAFE_PWM and not os.path.exists(FLAG_STOP):
            print("[OK] [ESTOP] Latch temizlendi (RC7 safe)")
            self.estop_latched = False
            self.estop_source = ""
            self.command_lock = False
            self._set_rc7_estop(False)
            self._refresh_health_check()
            self._write_state()
            return True
        print("[WARN] [ESTOP] RC7 hala kesme konumunda, görev baslatilamiyor")
        return False

    def _enter_hold(self, reason):
        self.state = self.STATE_HOLD
        self.mission_active = False
        self.v_target = 0.0
        self.heading_target = self.current_heading
        self.stop_motors()
        self._set_mode("HOLD")
        if reason.startswith("FAILSAFE"):
            self.failsafe_state = "hold"
        self._refresh_health_check()
        self._write_state()

    def _distance_and_heading_error(self, target_lat, target_lon):
        if self.simulation_mode:
            if not hasattr(self, "_sim_dist"):
                self._sim_dist = 14.0
            self._sim_dist = max(0.0, self._sim_dist - random.uniform(0.15, 0.45))
            self.current_heading = (self.current_heading + random.uniform(-5, 5)) % 360
            return self._sim_dist, random.uniform(-6, 6)
        lat = self.current_lat
        lon = self.current_lon
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return 9999.0, 0.0
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return 9999.0, 0.0
        dist = haversine_distance(lat, lon, target_lat, target_lon)
        target_bearing = calculate_bearing(lat, lon, target_lat, target_lon)
        heading_error = normalize_heading_error(target_bearing - self.current_heading)
        return dist, heading_error

    def _check_abort(self):
        self._drain_mav_messages()
        if self._consume_flag(FLAG_STOP):
            self._trigger_estop("YKI", force_rc7=True)
            return True
        self._read_camera_status()
        self._update_watchdog()
        self._refresh_health_check()
        self._record_file3_if_due()
        if self.estop_latched:
            return True
        return self.state == self.STATE_HOLD

    def _check_rc_connected(self, timeout_sec=3):
        if self.simulation_mode or not self.master:
            return True
        end_t = time.time() + timeout_sec
        while time.time() < end_t:
            self._drain_mav_messages()
            valid = self._rc_link_active()
            if valid:
                return True
            time.sleep(0.1)
        return False

    def load_mission(self, filepath=None):
        fp = filepath or MISSION_FILE
        if os.path.exists(fp):
            try:
                with open(fp, "r", encoding="utf-8") as f:
                    data = json.load(f)
                self.waypoints_p1 = data.get("parkur1", [])
                self.waypoints_p2 = data.get("parkur2", [])
                self.waypoints_p3 = data.get("parkur3", [])
                self.target_color = data.get("target_color", "")
                print(
                    f"[OK] [GOREV] P1={len(self.waypoints_p1)} P2={len(self.waypoints_p2)} "
                    f"P3={len(self.waypoints_p3)} Hedef={self.target_color}"
                )
                return True
            except Exception as exc:
                print(f"[WARN] [GOREV] Okuma hatasi: {exc}")
        if self.simulation_mode:
            base_lat, base_lon = 38.4192, 27.1287
            self.waypoints_p1 = [[base_lat + 0.0001, base_lon], [base_lat + 0.0002, base_lon + 0.0001]]
            self.waypoints_p2 = [[base_lat + 0.00005, base_lon + 0.00018]]
            self.waypoints_p3 = [[base_lat, base_lon + 0.00024]]
            self.target_color = "RED"
            print("[OK] [GOREV] Simulasyon gorevi olusturuldu")
            return True
        print(f"[WARN] [GOREV] Dosya bulunamadi: {fp}")
        return False

    def start_mission(self):
        if self.mission_active:
            print("[WARN] [GOREV] Zaten aktif")
            return False
        self._drain_mav_messages()
        self._read_camera_status()
        self._update_watchdog()
        self._refresh_health_check()
        if not self._clear_estop_if_safe():
            return False
        if not self._check_rc_connected():
            print("❌ [GOREV] RC bagli degil")
            return False
        self._refresh_health_check()
        if not self.health_ready:
            missing = ", ".join(self.health_missing) if self.health_missing else "unknown"
            print(f"❌ [READY] HEALTH_CHECK gecmedi: {missing}")
            self._write_state()
            return False
        self.mission_active = True
        self.mission_start_time = time.time()
        self.command_lock = True
        self.state = self.STATE_PARKUR1
        self.failsafe_state = "normal"
        self.gate_count = 0
        self.timeout_count = 0
        self._gate_event_start = None
        self._gate_event_latched = False
        self._wp_target = "--"
        self._wp_info = "-- / --"
        if not self._set_mode("AUTO"):
            print("❌ [START] AUTO moda gecilemedi")
            self.mission_active = False
            self.command_lock = False
            self.state = self.STATE_IDLE
            self._write_state()
            return False
        if not self._arm():
            print("❌ [START] Arming basarisiz")
            self.mission_active = False
            self.command_lock = False
            self.state = self.STATE_IDLE
            self.stop_motors()
            self._write_state()
            return False
        self._write_state()
        print("[START] [GOREV] Baslatildi")
        return True

    def _navigate_p1_waypoint(self, lat, lon):
        self._sim_dist = 14.0
        hold_start = None
        next_invalid_log = 0.0
        while self.mission_active:
            if self._check_abort():
                return False
            dist, heading_err = self._distance_and_heading_error(lat, lon)
            self._wp_target = f"{lat:.7f}, {lon:.7f}"
            if dist >= 9000.0:
                self._command_speed_heading(0.0, 0.0)
                if time.monotonic() >= next_invalid_log:
                    print("[WARN] [P1] Gecersiz navigasyon verisi, beklemede")
                    next_invalid_log = time.monotonic() + 1.5
                self._write_state()
                time.sleep(LOOP_DT)
                continue
            if dist <= R_WP_M:
                if hold_start is None:
                    hold_start = time.monotonic()
                self._command_speed_heading(0.0, 0.0)
                if (time.monotonic() - hold_start) >= T_HOLD_S:
                    print(f"[OK] [P1] WP_REACHED dist={dist:.2f}m hold={T_HOLD_S:.1f}s")
                    self.stop_motors()
                    return True
            else:
                hold_start = None
                speed = P1_SPEED_APPROACH_MPS if dist < 8.0 else P1_SPEED_CRUISE_MPS
                self._command_speed_heading(speed, heading_err)
            self._write_state()
            time.sleep(LOOP_DT)
        return False

    def _wait_p2_ready(self):
        print("[CHECK] [P1->P2] Kamera/Lidar hazirlik kontrolu")
        next_log = 0.0
        while self.mission_active:
            if self._check_abort():
                return False
            if self.camera_ready and self.lidar_ready:
                print("[OK] [P1->P2] Hazirlik tamam")
                return True
            self._command_speed_heading(P2_WAIT_SPEED_MPS, 0.0)
            if time.monotonic() >= next_log:
                print(
                    f"[WAIT] [P1->P2] Bekleniyor camera_ready={self.camera_ready} "
                    f"lidar_ready={self.lidar_ready}"
                )
                next_log = time.monotonic() + 1.5
            self._write_state()
            time.sleep(LOOP_DT)
        return False

    def _track_gate_event(self):
        event_active = bool(self.camera_status.get("gate_passed_event", False))
        if event_active:
            if self._gate_event_start is None:
                self._gate_event_start = time.monotonic()
            if (time.monotonic() - self._gate_event_start) >= P2_GATE_CONFIRM_S and not self._gate_event_latched:
                self.gate_count += 1
                self._gate_event_latched = True
                print(f"[GATE] [P2] gate_gecildi -> {self.gate_count}")
        else:
            self._gate_event_start = None
            self._gate_event_latched = False

    def _navigate_p2_waypoint(self, lat, lon):
        self._sim_dist = 12.0
        next_invalid_log = 0.0
        while self.mission_active:
            if self._check_abort():
                return False
            self._track_gate_event()
            dist, gps_heading_err = self._distance_and_heading_error(lat, lon)
            self._wp_target = f"{lat:.7f}, {lon:.7f}"
            if dist >= 9000.0:
                self._command_speed_heading(0.0, 0.0)
                if time.monotonic() >= next_invalid_log:
                    print("[WARN] [P2] Gecersiz navigasyon verisi, beklemede")
                    next_invalid_log = time.monotonic() + 1.5
                self._write_state()
                time.sleep(LOOP_DT)
                continue
            if dist <= R_WP_M:
                self.stop_motors()
                return True

            speed = P2_CRUISE_MPS if dist >= 8.0 else P2_WAIT_SPEED_MPS
            heading_err = gps_heading_err

            gate_detected = bool(self.camera_status.get("gate_detected", False))
            gate_stable_s = float(self.camera_status.get("gate_stable_s", 0.0))
            if gate_detected and gate_stable_s >= P2_STABLE_S:
                heading_err = float(self.camera_status.get("gate_center_bearing_deg", 0.0))

            left_d = self.lidar_left_dist
            center_d = self.lidar_center_dist
            right_d = self.lidar_right_dist

            if center_d < D_MIN_M:
                speed = min(speed, FAILSAFE_SLOW_MPS)
                heading_err = 35.0 if left_d > right_d else -35.0
            elif left_d < D_MIN_M:
                heading_err -= 18.0
            elif right_d < D_MIN_M:
                heading_err += 18.0

            self._command_speed_heading(speed, heading_err)
            self._write_state()
            time.sleep(LOOP_DT)
        return False

    def _run_p3_attempt(self, timeout_s):
        start_t = time.monotonic()
        contact_start = None
        next_invalid_log = 0.0
        wp = self.waypoints_p3[0] if self.waypoints_p3 else [self.current_lat, self.current_lon]
        self._sim_dist = 10.0
        while self.mission_active and (time.monotonic() - start_t) < timeout_s:
            if self._check_abort():
                return False
            dist, gps_heading_err = self._distance_and_heading_error(wp[0], wp[1])
            target_detected = bool(self.camera_status.get("target_detected", False))
            target_area = float(self.camera_status.get("target_area_norm", 0.0))
            if dist >= 9000.0 and not target_detected:
                self._command_speed_heading(0.0, 0.0)
                if time.monotonic() >= next_invalid_log:
                    print("[WARN] [P3] Gecersiz navigasyon/hedef verisi, beklemede")
                    next_invalid_log = time.monotonic() + 1.5
                self._write_state()
                time.sleep(LOOP_DT)
                continue
            if target_detected:
                heading_err = float(self.camera_status.get("target_bearing_error_deg", 0.0))
                speed = max(0.25, P3_MAX_SPEED_MPS * (1.0 - clamp(target_area, 0.0, 0.85)))
            else:
                heading_err = gps_heading_err
                speed = 0.6
            if dist < 2.0:
                speed = min(speed, 0.6)
            self._command_speed_heading(min(speed, P3_MAX_SPEED_MPS), heading_err)

            proximity_ok = (dist <= 1.0) or (target_area >= 0.20)
            if proximity_ok:
                if contact_start is None:
                    contact_start = time.monotonic()
                elif (time.monotonic() - contact_start) >= 0.6:
                    print("[ENGAGE] [P3] Angajman tamam")
                    self.stop_motors()
                    return True
            else:
                contact_start = None

            self._write_state()
            time.sleep(LOOP_DT)
        if self.mission_active and (time.monotonic() - start_t) >= timeout_s:
            self.timeout_count += 1
            print(f"[TIMEOUT] [P3] Deneme zaman asimi ({timeout_s:.0f}s), sayac={self.timeout_count}")
            self._write_state()
        return False

    def _retreat_and_hold(self):
        print("[RETREAT] [P3] Retry basarisiz, geri cekilme")
        start_lat, start_lon = self.current_lat, self.current_lon
        t0 = time.monotonic()
        while (time.monotonic() - t0) < P3_REVERSE_TIMEOUT_S:
            self._drain_mav_messages()
            if self._consume_flag(FLAG_STOP):
                self._trigger_estop("YKI", force_rc7=True)
                return
            moved = 0.0
            if start_lat != 0.0 and start_lon != 0.0 and self.current_lat != 0.0 and self.current_lon != 0.0:
                moved = haversine_distance(start_lat, start_lon, self.current_lat, self.current_lon)
            if moved >= P3_REVERSE_DISTANCE_M:
                break
            self._command_reverse(P3_REVERSE_SPEED_MPS)
            self._record_file3_if_due()
            time.sleep(LOOP_DT)
        self.stop_motors()
        self._enter_hold("P3_RETREAT")

    def run_parkur1(self):
        print("\n" + "=" * 52)
        print("  [PARKUR-1] AUTO WAYPOINT TAKIBI")
        print("=" * 52)
        self.state = self.STATE_PARKUR1
        self._set_mode("AUTO")
        self._write_state()
        if not self.waypoints_p1:
            print("[WARN] [P1] Waypoint yok")
            return True
        for idx, wp in enumerate(self.waypoints_p1, start=1):
            self._wp_info = f"{idx} / {len(self.waypoints_p1)}"
            if not self._navigate_p1_waypoint(wp[0], wp[1]):
                return False
        print("[OK] [P1] Tamam")
        return True

    def run_parkur2(self):
        print("\n" + "=" * 52)
        print("  [PARKUR-2] GUIDED KAPI + ENGEL KACINMA")
        print("=" * 52)
        self.state = self.STATE_PARKUR2
        self._write_state()
        if not self._wait_p2_ready():
            return False
        if not self._set_mode("GUIDED"):
            print("❌ [P2] GUIDED moda gecilemedi")
            return False
        if not self.waypoints_p2:
            print("[WARN] [P2] Waypoint yok")
            return True
        for idx, wp in enumerate(self.waypoints_p2, start=1):
            self._wp_info = f"{idx} / {len(self.waypoints_p2)}"
            if not self._navigate_p2_waypoint(wp[0], wp[1]):
                return False

        required_gates = 2 if USV_MODE == USV_MODE_RACE else 1
        deadline = time.monotonic() + 20.0
        while self.gate_count < required_gates and self.mission_active and time.monotonic() < deadline:
            if self._check_abort():
                return False
            self._track_gate_event()
            if self.camera_status.get("gate_detected", False):
                heading_err = float(self.camera_status.get("gate_center_bearing_deg", 0.0))
            else:
                heading_err = 0.0
            self._command_speed_heading(P2_WAIT_SPEED_MPS, heading_err)
            self._write_state()
            time.sleep(LOOP_DT)

        if self.gate_count < required_gates:
            print(f"❌ [P2] gate_gecildi yetersiz ({self.gate_count}/{required_gates})")
            return False
        print(f"[OK] [P2] Tamam gate={self.gate_count}")
        return True

    def run_parkur3(self):
        print("\n" + "=" * 52)
        print("  [PARKUR-3] HSV HEDEFLEME + ANGAJMAN")
        print("=" * 52)
        self.state = self.STATE_PARKUR3
        if not self._set_mode("GUIDED"):
            print("❌ [P3] GUIDED moda gecilemedi")
            return False
        self._write_state()
        if not self.waypoints_p3:
            print("[WARN] [P3] Hedef waypoint yok")
            return True
        if self._run_p3_attempt(P3_TIMEOUT_S):
            return True
        for retry in range(P3_RETRY_COUNT):
            print(f"[RETRY] [P3] Retry {retry + 1}/{P3_RETRY_COUNT}")
            if self._run_p3_attempt(P3_RETRY_S):
                return True
        self._retreat_and_hold()
        return True

    def _wait_for_next_parkur(self, current_name, next_name):
        print(f"🔄 [{current_name}] -> [{next_name}] otomatik gecis (kullanici girdisi kapali)")
        return True

    def _check_time(self):
        elapsed = time.time() - self.mission_start_time
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)
        if elapsed > 20 * 60:
            print(f"⏰ [SURE] 20dk asildi ({minutes}:{seconds:02d})")
        elif elapsed > 18 * 60 and int(elapsed) % 30 == 0:
            print(f"[WARN] [SURE] Kritik bolge ({minutes}:{seconds:02d})")

    def run(self):
        while self.mission_active:
            self._check_time()
            if self.state == self.STATE_PARKUR1:
                if self.run_parkur1():
                    if not self._wait_for_next_parkur("P1", "P2"):
                        break
                    self.state = self.STATE_PARKUR2
                else:
                    break
            elif self.state == self.STATE_PARKUR2:
                if self.run_parkur2():
                    if not self._wait_for_next_parkur("P2", "P3"):
                        break
                    self.state = self.STATE_PARKUR3
                else:
                    break
            elif self.state == self.STATE_PARKUR3:
                if self.run_parkur3():
                    self.state = self.STATE_COMPLETED
                else:
                    break
            elif self.state in (self.STATE_COMPLETED, self.STATE_HOLD):
                break
            self._write_state()

        if self.state == self.STATE_COMPLETED:
            print("[DONE] [GOREV] Tum parkurlar tamamlandi")
            self._wp_target = "TAMAMLANDI"
            self._wp_info = "-- / --"
        self.mission_active = False
        if not self.estop_latched:
            self.command_lock = False
        self.stop_motors()
        self._write_state()


if __name__ == "__main__":
    usv = USVStateMachine()
    try:
        mission_path = sys.argv[1] if len(sys.argv) > 1 else MISSION_FILE
        usv.load_mission(mission_path)
        print("\n[READY] [SISTEM] Hazir - gorev baslatma bekleniyor")
        if USV_MODE == USV_MODE_RACE:
            print("   Race start: Sadece RC CH5 >= 1700 (API/flag kapali)")
        else:
            print("   Test start: /api/start_mission")
        usv._refresh_health_check()
        usv._write_state()

        while True:
            usv._drain_mav_messages()
            usv._read_camera_status()
            usv._update_watchdog()
            usv._refresh_health_check()
            usv._record_file3_if_due()

            if usv._consume_flag(FLAG_STOP):
                usv._trigger_estop("YKI", force_rc7=True)

            if not usv.mission_active:
                if USV_MODE == USV_MODE_RACE and os.path.exists(FLAG_START):
                    usv._consume_flag(FLAG_START)
                    if (time.monotonic() - usv._last_race_flag_purge_log) >= 2.0:
                        print("[POLICY] [RACE] API start flag temizlendi (RC-only baslatma politikasi)")
                        usv._last_race_flag_purge_log = time.monotonic()

                start_from_api = bool(USV_MODE != USV_MODE_RACE and usv._consume_flag(FLAG_START))
                start_from_rc = bool(USV_MODE == USV_MODE_RACE and usv.rc_channels.get("ch5", 0) >= RC_RACE_START_PWM)
                if start_from_api or start_from_rc:
                    src = "RC" if start_from_rc else "API"
                    print(f"[START] [START] Kaynak={src}")
                    if usv.start_mission():
                        usv.run()
            usv._write_state()
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[STOP] [SISTEM] Kapatiliyor...")
    finally:
        usv.stop_motors()
        usv._close_file3_recorder()

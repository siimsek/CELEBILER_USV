#!/usr/bin/env python3
"""
JSON Backend Bridge for ArduPilot SITL <-> Gazebo Physics
- Receives ROS2 nav_msgs/msg/Odometry from Gazebo and sends via UDP 9002 to ArduPilot JSON API.
- Receives JSON PWM from ArduPilot on UDP 9003 and sends ROS2 wrench commands to Gazebo.
- Updates vehicle_position.json.
"""
import json
import math
import os
import sys
import threading
import time
import socket
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT / "docker_workspace" / "src") not in sys.path:
    sys.path.insert(0, str(_ROOT / "docker_workspace" / "src"))
from json_atomic import atomic_write_json
from runtime_debug_log import log_jsonl, redirect_std_streams, setup_component_logger
from run_logger import get_run_logger

log = setup_component_logger("sitl_gazebo_bridge", prefer_simulation=True)
redirect_std_streams(log)

_rl_bridge = get_run_logger(component="sitl_gazebo_bridge")
_rl_bridge.info("service_init_begin", sitl_port=9002)

CONTROL_DIR = os.environ.get("CONTROL_DIR", str(_ROOT / "sim" / "control"))
POS_FILE = Path(CONTROL_DIR) / "vehicle_position.json"
MISSION_STATE_FILE = Path(CONTROL_DIR) / "mission_state.json"
MODEL_NAME = os.environ.get("SIM_GZ_MODEL_NAME", "usv_model")
SPAWN_X = float(os.environ.get("SIM_GZ_SPAWN_X", "0"))
SPAWN_Y = float(os.environ.get("SIM_GZ_SPAWN_Y", "0"))
POSE_ARM_S = max(0.0, float(os.environ.get("SIM_GZ_POSE_ARM_S", "0")))
MAX_LINEAR_VELOCITY_MPS = max(0.1, float(os.environ.get("SIM_GZ_MAX_LINEAR_VELOCITY_MPS", "3.0")))
MAX_YAW_RATE_RAD_S = max(0.1, float(os.environ.get("SIM_GZ_MAX_YAW_RATE_RAD_S", "1.0")))
PWM_MIN_US = float(os.environ.get("SIM_GZ_PWM_MIN_US", "1100"))
PWM_TRIM_US = float(os.environ.get("SIM_GZ_PWM_TRIM_US", "1500"))
PWM_MAX_US = float(os.environ.get("SIM_GZ_PWM_MAX_US", "1900"))
try:
    SIM_GZ_YAW_SIGN = -1.0 if float(os.environ.get("SIM_GZ_YAW_SIGN", "-1.0")) < 0.0 else 1.0
except (TypeError, ValueError):
    SIM_GZ_YAW_SIGN = -1.0
LEFT_MOTOR_REVERSED = os.environ.get("SIM_GZ_LEFT_MOTOR_REVERSED", "0").lower() in ("1", "true", "yes")
RIGHT_MOTOR_REVERSED = os.environ.get("SIM_GZ_RIGHT_MOTOR_REVERSED", "0").lower() in ("1", "true", "yes")
SIM_GZ_SERVO1_MOTOR = os.environ.get("SIM_GZ_SERVO1_MOTOR", "left").strip().lower()
SIM_GZ_SERVO3_MOTOR = os.environ.get("SIM_GZ_SERVO3_MOTOR", "right").strip().lower()
USV_MODE = os.environ.get("USV_MODE", "test").strip().lower()
ALLOW_MOTOR_COMMAND_JSON_OVERRIDE = (
    os.environ.get("SIM_GZ_ALLOW_MOTOR_COMMAND_JSON", "0").lower() in ("1", "true", "yes")
    and USV_MODE != "race"
)

DRIVE_CMD_HZ = max(1.0, float(os.environ.get("SIM_GZ_DRIVE_CMD_HZ", "50")))

SITL_JSON_OUT = ("127.0.0.1", 9003) # Send physics here (DEPRECATED: now use sender_addr from PWM RX)
SITL_JSON_IN_PORT = 9002            # Listen for PWM here (and respond to sender bidirectionally)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    ROS2_OK = True
except ImportError:
    ROS2_OK = False

def _normalize_pwm(value, default=1500.0):
    try:
        pwm = float(value)
        return default if pwm <= 0 else pwm
    except:
        return float(default)

def _normalized_motor_outputs(left_pwm, right_pwm):
    def norm(pwm):
        pwm = float(pwm)
        if pwm >= PWM_TRIM_US:
            span = max(1.0, PWM_MAX_US - PWM_TRIM_US)
            return (pwm - PWM_TRIM_US) / span
        span = max(1.0, PWM_TRIM_US - PWM_MIN_US)
        return (pwm - PWM_TRIM_US) / span

    left_n = norm(left_pwm)
    right_n = norm(right_pwm)
    if LEFT_MOTOR_REVERSED:
        left_n *= -1.0
    if RIGHT_MOTOR_REVERSED:
        right_n *= -1.0
    left_n = max(-1.0, min(1.0, left_n))
    right_n = max(-1.0, min(1.0, right_n))
    return left_n, right_n

def _servo_outputs_to_left_right(ch1_pwm, ch3_pwm):
    """Map ArduPilot SERVO1/SERVO3 outputs into physical left/right thrusters."""
    if SIM_GZ_SERVO1_MOTOR == "right" and SIM_GZ_SERVO3_MOTOR == "left":
        return float(ch3_pwm), float(ch1_pwm)
    return float(ch1_pwm), float(ch3_pwm)

def _wrap_180(deg):
    return ((float(deg) + 180.0) % 360.0) - 180.0

def _nav_heading_deg_from_gazebo_yaw(heading_rad):
    return (-math.degrees(float(heading_rad)) + 360.0) % 360.0

def _mission_expects_autonomous_pwm():
    try:
        with open(MISSION_STATE_FILE, "r", encoding="utf-8") as f:
            state = json.load(f)
        if bool(state.get("active", False)):
            return True
        mode_state = state.get("mode_state", {})
        if not isinstance(mode_state, dict):
            mode_state = {}
        vehicle_mode = str(state.get("vehicle_mode") or mode_state.get("vehicle_mode") or "").upper()
        canonical = str(mode_state.get("canonical_mode") or "").upper()
        if vehicle_mode in ("AUTO", "GUIDED") or canonical == "NAV_ACTIVE":
            return True
    except Exception:
        return False
    return False

class JsonBridgeNode(Node):
    def __init__(self, bridge):
        super().__init__("sitl_gazebo_bridge")
        self.bridge = bridge
        # Diagnostic mirrors for current PWM state.
        self.pub_left = self.create_publisher(Float64, "/motor_left_pwm", 10)
        self.pub_right = self.create_publisher(Float64, "/motor_right_pwm", 10)
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # CRITICAL FIX: Add diagnostic logging for ROS2 initialization
        self.get_logger().info(f"JsonBridgeNode init: Creating odometry subscription to /model/{MODEL_NAME}/odometry")
        
        self.sub_odom = self.create_subscription(Odometry, f"/model/{MODEL_NAME}/odometry", self.odom_callback, qos_profile_sensor_data)
        self.get_logger().info(f"Subscribed to /model/{MODEL_NAME}/odometry (QoS: SENSOR_DATA)")

    def odom_callback(self, msg):
        self.bridge.handle_odometry(msg)

class SitlGazeboBridge:
    def __init__(self):
        self.running = True
        self.lock = threading.Lock()
        
        # State
        self.pos_x = SPAWN_X
        self.pos_y = SPAWN_Y
        self.pos_z = 0.0
        self.heading_rad = 0.0
        self.heading_nav_deg = _nav_heading_deg_from_gazebo_yaw(0.0)
        self.heading_delta_deg = 0.0
        self.observed_yaw_rate_dps = 0.0
        self._last_heading_nav_deg = None
        self._last_heading_ts = None
        self.last_yaw_cmd_norm = 0.0
        self.motor_pwm_ch1 = 1500.0 # Left motor output (Servo 1)
        self.motor_pwm_ch2 = 1500.0 # Right motor output (Servo 3)
        self.motor_pwm_raw = []
        self.servo_response_addr = ("127.0.0.1", SITL_JSON_IN_PORT)  # CRITICAL FIX: Default addr for first packet
        self.latest_sensor_payload = None  # Cache latest sensor data for JSON response
        self._bench_override_active = False
        self._first_pwm_received = False  # Track first PWM packet for robust initialization
        
        self.ros_node = None
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("127.0.0.1", SITL_JSON_IN_PORT))
        self.udp_sock.settimeout(0.5)
        
        self._pose_arm_deadline = time.monotonic() + POSE_ARM_S
        self._last_stats = time.time()
        self._last_drive_cmd_pub = 0.0
        self._msg_count_pwm = 0
        self._msg_count_odom = 0
        self._neutral_pwm_since = None
        self._last_neutral_pwm_diag = 0.0
        log.info(
            "[SIM-MOTOR] Servo mapping: SERVO1/CH1=%s SERVO3/CH3=%s yaw_sign=%.1f left_rev=%s right_rev=%s "
            "pwm_min/trim/max=%.0f/%.0f/%.0f bench_json_override=%s "
            "(override with SIM_GZ_SERVO1_MOTOR/SIM_GZ_SERVO3_MOTOR/SIM_GZ_YAW_SIGN/SIM_GZ_PWM_*_US)",
            SIM_GZ_SERVO1_MOTOR,
            SIM_GZ_SERVO3_MOTOR,
            SIM_GZ_YAW_SIGN,
            LEFT_MOTOR_REVERSED,
            RIGHT_MOTOR_REVERSED,
            PWM_MIN_US,
            PWM_TRIM_US,
            PWM_MAX_US,
            ALLOW_MOTOR_COMMAND_JSON_OVERRIDE,
        )
        log_jsonl(
            "sitl_gazebo_bridge",
            True,
            event="sim_motor_contract",
            servo1_motor=str(SIM_GZ_SERVO1_MOTOR),
            servo3_motor=str(SIM_GZ_SERVO3_MOTOR),
            yaw_sign=float(SIM_GZ_YAW_SIGN),
            pwm_min_us=float(PWM_MIN_US),
            pwm_trim_us=float(PWM_TRIM_US),
            pwm_max_us=float(PWM_MAX_US),
            pwm_1100_norm=_normalized_motor_outputs(PWM_MIN_US, PWM_TRIM_US)[0],
            pwm_1500_norm=_normalized_motor_outputs(PWM_TRIM_US, PWM_TRIM_US)[0],
            pwm_1900_norm=_normalized_motor_outputs(PWM_MAX_US, PWM_TRIM_US)[0],
            bench_json_override_allowed=bool(ALLOW_MOTOR_COMMAND_JSON_OVERRIDE),
            left_motor_reversed=bool(LEFT_MOTOR_REVERSED),
            right_motor_reversed=bool(RIGHT_MOTOR_REVERSED),
            yaw_contract="left_norm_minus_right_norm_positive_increases_nav_heading",
        )
        log_jsonl(
            "sitl_gazebo_bridge",
            True,
            event="sim_bridge_self_check",
            yaw_sign=float(SIM_GZ_YAW_SIGN),
            servo1_motor=str(SIM_GZ_SERVO1_MOTOR),
            servo3_motor=str(SIM_GZ_SERVO3_MOTOR),
            race_mode=USV_MODE == "race",
            bench_json_override_env=os.environ.get("SIM_GZ_ALLOW_MOTOR_COMMAND_JSON", "0"),
            bench_json_override_allowed=bool(ALLOW_MOTOR_COMMAND_JSON_OVERRIDE),
            pwm_normalization="piecewise_min_trim_max",
            pwm_min_us=float(PWM_MIN_US),
            pwm_trim_us=float(PWM_TRIM_US),
            pwm_max_us=float(PWM_MAX_US),
        )

        # Parse SIM_HOME
        try:
            home_parts = os.environ.get("SIM_HOME", "-35.363262,149.165237,584,0").split(',')
            self.home_lat = float(home_parts[0])
            self.home_lon = float(home_parts[1])
            self.home_alt = float(home_parts[2])
        except:
            self.home_lat, self.home_lon, self.home_alt = -35.363262, 149.165237, 584.0

        if ROS2_OK:
            rclpy.init()
            self.ros_node = JsonBridgeNode(self)
            # Start ROS2 executor in daemon thread to process callbacks
            executor = MultiThreadedExecutor(num_threads=1)
            executor.add_node(self.ros_node)
            threading.Thread(target=executor.spin, daemon=True).start()
        else:
            log.warning("ROS2 not available")

    def q_to_euler(self, w, x, y, z):
        # Yaw (z-axis rotation) only - for compass heading
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def handle_odometry(self, msg):
        # Convert ROS Odometry to ArduPilot JSON format
        self._msg_count_odom += 1
        
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        av = msg.twist.twist.angular
        
        x_ned = p.x
        y_ned = -p.y
        z_ned = -p.z
        
        vx_ned = v.x
        vy_ned = -v.y
        vz_ned = -v.z
        
        w, nx, ny, nz = q.w, q.x, q.y, q.z
        
        with self.lock:
            self.pos_x = p.x
            self.pos_y = p.y
            self.pos_z = p.z
            self.heading_rad = self.q_to_euler(q.w, q.x, q.y, q.z)
            
        # Convert NED to Global Lat/Lon using earth coordinates
        # Reference: SIM_HOME = {home_lat}, {home_lon}, {home_alt}
        lat = self.home_lat + (x_ned / 111320.0)
        lon = self.home_lon + (y_ned / (111320.0 * math.cos(math.radians(self.home_lat))))
        alt = self.home_alt - z_ned
        
        if self._msg_count_odom % 100 == 1:
            log_jsonl("sitl_gazebo_bridge", True,
                event="coordinate_transform",
                msg_count=self._msg_count_odom,
                gazebo_xyz=[round(p.x, 3), round(p.y, 3), round(p.z, 3)],
                ned_xyz=[round(x_ned, 3), round(y_ned, 3), round(z_ned, 3)],
                gps_latlon=[round(lat, 7), round(lon, 7)],
                home_ref=[self.home_lat, self.home_lon],
                validation="OK" if (-90 <= lat <= 90 and -180 <= lon <= 180) else "RANGE_ERROR",
            )

        # Calculate roll, pitch, yaw
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * nx + ny * nz)
        cosr_cosp = 1 - 2 * (nx * nx + ny * ny)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (w * ny - nx * nz))
        cosp = math.sqrt(1 - 2 * (w * ny - nx * nz))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * nz + nx * ny)
        cosy_cosp = 1 - 2 * (ny * ny + nz * nz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        heading_nav_deg = _nav_heading_deg_from_gazebo_yaw(yaw)
        heading_now = time.monotonic()
        with self.lock:
            if self._last_heading_nav_deg is None or self._last_heading_ts is None:
                heading_delta_deg = 0.0
                yaw_rate_dps = 0.0
            else:
                dt_heading = max(0.01, heading_now - float(self._last_heading_ts))
                heading_delta_deg = _wrap_180(heading_nav_deg - float(self._last_heading_nav_deg))
                yaw_rate_dps = heading_delta_deg / dt_heading
            self.heading_nav_deg = float(heading_nav_deg)
            self.heading_delta_deg = float(heading_delta_deg)
            self.observed_yaw_rate_dps = float(yaw_rate_dps)
            self._last_heading_nav_deg = float(heading_nav_deg)
            self._last_heading_ts = float(heading_now)
            
        t = time.time()
        payload = {
            "timestamp": t,
            "imu": {
                "gyro": [av.x, -av.y, -av.z],
                "accel_body": [0.0, 0.0, -9.81]
            },
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "position": [x_ned, y_ned, z_ned],
            "attitude": [roll, -pitch, -yaw],
            "velocity": [vx_ned, vy_ned, vz_ned]
        }
        
        with self.lock:
            self.latest_sensor_payload = payload  # Cache for servo response (CRITICAL FIX: guarantee payload exists)
            
            # CRITICAL FIX: Log odometry reception for diagnostics
            if self._msg_count_odom <= 3:
                log.info(f"[ODOM] Odometry received and cached, msg_count={self._msg_count_odom}")


    def receive_pwm(self):
        log.info(f"Listening for binary PWM on UDP {SITL_JSON_IN_PORT}...")
        import struct
        while self.running:
            try:
                data, addr = self.udp_sock.recvfrom(4096)
                pwms = []
                if len(data) == 40:
                    unpacked = struct.unpack('<HHI16H', data)
                    if unpacked[0] == 18458:
                        pwms = unpacked[3:]
                elif len(data) == 72:
                    unpacked = struct.unpack('<HHI32H', data)
                    if unpacked[0] == 29569:
                        pwms = unpacked[3:]
                
                # Debug: Log received data
                count = self._msg_count_pwm
                if count % 500 == 0:
                    log.debug(f"[PWM-RX] len={len(data)} addr={addr} pwms_count={len(pwms)}")
                
                if len(pwms) >= 3:
                    self.motor_pwm_raw = list(pwms)
                    # Skid steering: Servo1 is Left, Servo3 is Right
                    ch1 = _normalize_pwm(pwms[0], 1500.0)
                    ch2 = _normalize_pwm(pwms[2], 1500.0)
                    with self.lock:
                        self.motor_pwm_ch1 = ch1
                        self.motor_pwm_ch2 = ch2
                        self.servo_response_addr = addr  # Store sender for response (bidirectional protocol fix)
                        first_pwm = not self._first_pwm_received
                        if first_pwm:
                            self._first_pwm_received = True
                    self._msg_count_pwm += 1
                    self.publish_forces()
                    
                    # CRITICAL FIX: Send JSON sensor response back to ArduPilot (bidirectional protocol)
                    # ArduPilot --model JSON expects request/response, not one-way servo stream
                    # First packet response guaranteed even if odom/sensor payload delayed
                    self.send_servo_response_to_ardupilot(force_response_on_first_packet=first_pwm)
                else:
                    if count % 500 == 0:
                        log.warn(f"[PWM-RX] Invalid PWM data: len={len(data)}, parsed_pwms={len(pwms)}")
            except socket.timeout:
                pass
            except Exception as e:
                log.error(f"Error parsing PWM: {e}")

    def publish_forces(self):
        if not self.ros_node: return
        with self.lock:
            c1_sitl = self.motor_pwm_ch1
            c2_sitl = self.motor_pwm_ch2
            count = self._msg_count_pwm

        # Motor source selection:
        # Default is ArduPilot SITL servo output. That keeps sim-to-real honest:
        # Pi sends MAVLink setpoints, ArduPilot owns mixer/PWM, and the bridge
        # only transfers the resulting servo outputs into Gazebo physics.
        #
        # motor_command.json is a visible bench-test escape hatch only. It is
        # off by default and forced off in race mode.
        c1, c2 = _servo_outputs_to_left_right(c1_sitl, c2_sitl)
        motor_source = "sitl_servo"
        try:
            mc_path = Path(CONTROL_DIR) / "motor_command.json"
            if ALLOW_MOTOR_COMMAND_JSON_OVERRIDE and mc_path.exists():
                mc = json.loads(mc_path.read_text())
                if time.time() - mc.get("ts", 0) < 2.0:
                    c1 = float(mc["left_pwm"])
                    c2 = float(mc["right_pwm"])
                    motor_source = "motor_command_json_bench"
                    if not self._bench_override_active:
                        log.info(
                            f"[SIM-MOTOR] Bench override enabled: left={c1:.0f} right={c2:.0f} "
                            f"(SITL servo was CH1={c1_sitl:.0f} CH3={c2_sitl:.0f})"
                        )
                        self._bench_override_active = True
                    with self.lock:
                        self.motor_pwm_ch1 = c1
                        self.motor_pwm_ch2 = c2
                else:
                    self._bench_override_active = False
            else:
                self._bench_override_active = False
        except Exception:
            self._bench_override_active = False

        now = time.monotonic()
        neutral_sitl = abs(float(c1_sitl) - 1500.0) <= 3.0 and abs(float(c2_sitl) - 1500.0) <= 3.0
        autonomous_pwm_expected = _mission_expects_autonomous_pwm()
        if motor_source == "sitl_servo" and neutral_sitl and count > 200:
            if self._neutral_pwm_since is None:
                self._neutral_pwm_since = now
            neutral_age_s = now - float(self._neutral_pwm_since)
            if neutral_age_s >= 3.0 and (now - self._last_neutral_pwm_diag) >= 5.0:
                log_fn = log.warning if autonomous_pwm_expected else log.info
                log_fn(
                    "[PWM-DIAG] SITL servo outputs neutral for %.1fs: CH1=%.0f CH3=%.0f. "
                    "Gazebo will stay still; check FC GPS/EKF/mode or test sim manual_rc_override fallback.",
                    neutral_age_s,
                    c1_sitl,
                    c2_sitl,
                )
                self._last_neutral_pwm_diag = now
        else:
            if self._neutral_pwm_since is not None and not neutral_sitl:
                log.info(
                    "[PWM-DIAG] SITL servo outputs left neutral: CH1=%.0f CH3=%.0f after %.1fs",
                    c1_sitl,
                    c2_sitl,
                    now - float(self._neutral_pwm_since),
                )
            self._neutral_pwm_since = None

        self.ros_node.pub_left.publish(Float64(data=c1))
        self.ros_node.pub_right.publish(Float64(data=c2))

        # PWM (1500 neutral, 1100-1900 range) => -1 to +1 normalized.
        # Rover skid defaults drive the starboard output reversed, so apply the
        # configured sign correction before generating hull wrench.
        left_n, right_n = _normalized_motor_outputs(c1, c2)

        # Use Gazebo VelocityControl on base_link. Unlike DiffDrive, it doesn't
        # assume wheel-ground traction and can drive the floating hull directly.
        try:
            avg_thrust = (left_n + right_n) / 2.0
            # Canonical sim yaw contract:
            #   CH1/left_n > CH3/right_n means positive nav heading rate.
            #   ROS/Gazebo angular.z is positive CCW, so the default sign is -1.
            yaw_cmd_norm = (left_n - right_n) / 2.0
            yaw_diff = SIM_GZ_YAW_SIGN * yaw_cmd_norm
            with self.lock:
                self.last_yaw_cmd_norm = float(yaw_cmd_norm)

            if (now - self._last_drive_cmd_pub) >= (1.0 / DRIVE_CMD_HZ):
                self._last_drive_cmd_pub = now
                twist = Twist()
                # POSE_ARM: spawn'da fiziksel kaymayı önle — ama SITL JSON yanıtını asla kesme
                # (JSON kesilirse AP GPS/EKF oluşmaz → GUIDED modu reddedilir; kanıt: mode_change_failed:GUIDED).
                if now > self._pose_arm_deadline:
                    avg_velocity = avg_thrust * MAX_LINEAR_VELOCITY_MPS
                    yaw_rate = yaw_diff * MAX_YAW_RATE_RAD_S
                    twist.linear.x = float(avg_velocity)
                    twist.angular.z = float(yaw_rate)
                self.ros_node.pub_vel.publish(twist)

                if count % 1000 == 0 and now > self._pose_arm_deadline:
                    log.debug(
                        f"[MOTOR-CMD] vx={twist.linear.x:.2f}m/s wz={twist.angular.z:.2f}rad/s "
                        f"left_n={left_n:.2f} right_n={right_n:.2f} yaw_cmd={yaw_cmd_norm:.2f} "
                        f"src={motor_source}"
                    )
        except Exception as e:
            if count % 500 == 0:
                log.error(f"[MOTOR-CMD-ERROR] Exception: {type(e).__name__}: {e}")

        # Debug: Log motor commands every 100 publishes
        if count % 100 == 0:
            mode_str = f"[{motor_source.upper()}]"
            with self.lock:
                observed_yaw_rate_dps = self.observed_yaw_rate_dps
                heading_delta_deg = self.heading_delta_deg
            log.debug(
                f"{mode_str} [MOTOR] PWM: CH1={c1:.0f} CH3={c2:.0f} → "
                f"left_n={left_n:.2f} right_n={right_n:.2f} yaw_cmd={self.last_yaw_cmd_norm:.2f} "
                f"yaw_rate_nav={observed_yaw_rate_dps:.2f}dps heading_delta={heading_delta_deg:.2f}deg"
            )
            # P0 idle-yaw drift diagnostic (şartname 4.3: istemsiz dönüş → video başarısız).
            # Tespit: ileri itki ~0 iken ArduPilot SITL asimetrik servo üretip gövdeyi döndürüyorsa.
            # Yalnızca tanı/loj; motor çıkışını değiştirmez. Eşik env ile ayarlanabilir.
            _avg_thrust_chk = (float(left_n) + float(right_n)) / 2.0
            _yaw_cmd_chk = (float(left_n) - float(right_n)) / 2.0
            _obs_yaw_chk = float(observed_yaw_rate_dps or 0.0)
            _idle_yaw_thresh = max(0.5, float(os.environ.get("SIM_GZ_IDLE_YAW_DRIFT_DPS", "1.5")))
            if (
                abs(_avg_thrust_chk) <= 0.05
                and abs(_yaw_cmd_chk) >= 0.05
                and abs(_obs_yaw_chk) >= _idle_yaw_thresh
            ):
                if not hasattr(self, "_last_idle_yaw_diag"):
                    self._last_idle_yaw_diag = 0.0
                if (now - float(self._last_idle_yaw_diag)) >= 5.0:
                    self._last_idle_yaw_diag = now
                    log.warning(
                        "[IDLE-YAW-DRIFT] Zero-thrust yaw command: CH1=%.0f CH3=%.0f "
                        "left_n=%.3f right_n=%.3f avg_thrust=%.3f yaw_cmd=%.3f "
                        "observed_yaw=%.2fdps. ArduPilot SITL servo trim "
                        "(SERVO1/3_TRIM, SERVO3_REVERSED, MOT_TRIM) veya SIM_GZ_PWM_TRIM_US "
                        "off-neutral olabilir; v=0 iken GUIDED yaw hold kontrol edilmeli.",
                        c1_sitl, c2_sitl, left_n, right_n,
                        _avg_thrust_chk, _yaw_cmd_chk, _obs_yaw_chk,
                    )
                    log_jsonl(
                        "sitl_gazebo_bridge", True, event="idle_yaw_drift",
                        ch1_pwm=round(float(c1_sitl), 1), ch3_pwm=round(float(c2_sitl), 1),
                        left_norm=round(float(left_n), 4), right_norm=round(float(right_n), 4),
                        avg_thrust=round(float(_avg_thrust_chk), 4),
                        yaw_cmd_norm=round(float(_yaw_cmd_chk), 4),
                        observed_yaw_rate_dps=round(float(_obs_yaw_chk), 4),
                        motor_source=str(motor_source), threshold_dps=float(_idle_yaw_thresh),
                    )
            if count % 1000 == 0:
                log_jsonl(
                    "sitl_gazebo_bridge",
                    True,
                    event="sim_motor_yaw_diagnostic",
                    motor_source=str(motor_source),
                    ch1_pwm=round(float(c1_sitl), 1),
                    ch3_pwm=round(float(c2_sitl), 1),
                    left_pwm=round(float(c1), 1),
                    right_pwm=round(float(c2), 1),
                    servo1_motor=str(SIM_GZ_SERVO1_MOTOR),
                    servo3_motor=str(SIM_GZ_SERVO3_MOTOR),
                    left_norm=round(float(left_n), 4),
                    right_norm=round(float(right_n), 4),
                    yaw_cmd_norm=round(float(self.last_yaw_cmd_norm), 4),
                    observed_yaw_rate_dps=round(float(observed_yaw_rate_dps), 4),
                    heading_delta_deg=round(float(heading_delta_deg), 4),
                    yaw_sign=float(SIM_GZ_YAW_SIGN),
                )

    def send_servo_response_to_ardupilot(self, force_response_on_first_packet=False):
        """Send JSON sensor response back to ArduPilot (bidirectional protocol fix)
        
        ArduPilot --model JSON expects:
        1. Servo command OUT to bridge on port 9002
        2. Sensor data response BACK from SAME sender address (bidirectional)
        Without response, ArduPilot keeps resending same servo values (0.9 Hz jitter)
        With response, ArduPilot generates new servo commands at ~50 Hz
        """
        try:
            with self.lock:
                payload = self.latest_sensor_payload
                addr = self.servo_response_addr
                msg_count = self._msg_count_pwm
                home_lat, home_lon, home_alt = self.home_lat, self.home_lon, self.home_alt
            
            if payload is None:
                # CRITICAL FIX: Force minimal response on first PWM even if sensor not ready
                if force_response_on_first_packet and addr is not None:
                    payload = {
                        "timestamp": time.time(),
                        "imu": {"gyro": [0.0, 0.0, 0.0], "accel_body": [0.0, 0.0, -9.81]},
                        "latitude": home_lat,
                        "longitude": home_lon,
                        "altitude": home_alt,
                        "position": [0.0, 0.0, 0.0],
                        "attitude": [0.0, 0.0, 0.0],
                        "velocity": [0.0, 0.0, 0.0]
                    }
                    log.debug(f"[SERVO] First PWM packet: sending default sensor payload to guarantee bidirectional handshake (addr={addr})")
                else:
                    if msg_count % 500 == 0:
                        log.warn(f"[SERVO] payload=None, cannot respond (first_pkt={force_response_on_first_packet})")
                    return
            
            if addr is None:
                if msg_count % 500 == 0:
                    log.warn(f"[SERVO] addr=None, cannot respond")
                return

            # CRITICAL FIX: Send sensor response back to the SAME address that sent PWM (bidirectional)
            # NOT to SITL_JSON_OUT! ArduPilot uses request-response on same address
            data = ('\n' + json.dumps(payload) + '\n').encode('utf-8')
            bytes_sent = self.udp_sock.sendto(data, addr)  # Send back to sender (bidirectional!)
            
            if msg_count % 200 == 0 or msg_count < 10:  # Log first 10 and then every 200
                log.info(f"[SERVO] Response {bytes_sent} bytes to {addr} (payload keys: {list(payload.keys())})")
                
        except Exception as e:
            log.error(f"[SERVO] Response error: {e}")

    def write_position_json(self):
        while self.running:
            try:
                with self.lock:
                    ch1, ch2 = self.motor_pwm_ch1, self.motor_pwm_ch2
                    x, y, hdg = self.pos_x, self.pos_y, self.heading_rad
                    heading_nav_deg = self.heading_nav_deg
                    heading_delta_deg = self.heading_delta_deg
                    observed_yaw_rate_dps = self.observed_yaw_rate_dps
                    yaw_cmd_norm = self.last_yaw_cmd_norm
                
                left_pwm, right_pwm = _servo_outputs_to_left_right(ch1, ch2)
                left_n, right_n = _normalized_motor_outputs(left_pwm, right_pwm)
                motor_norm = max(-1.0, min(1.0, (left_n + right_n) / 2.0))
                POS_FILE.parent.mkdir(parents=True, exist_ok=True)
                payload = {
                    "ts": time.time(),
                    "pos_x": x,
                    "pos_y": y,
                    "heading_rad": hdg,
                    "heading_nav_deg": heading_nav_deg,
                    "heading_delta_deg": heading_delta_deg,
                    "observed_yaw_rate_dps": observed_yaw_rate_dps,
                    "motor_ch1_pwm": ch1,
                    "motor_ch2_pwm": ch2,
                    "motor_left_pwm": left_pwm,
                    "motor_right_pwm": right_pwm,
                    "motor_left_norm": left_n,
                    "motor_right_norm": right_n,
                    "motor_normalized": motor_norm,
                    "yaw_cmd_norm": yaw_cmd_norm,
                    "yaw_sign": SIM_GZ_YAW_SIGN,
                    "source": "sitl_gazebo_bridge_json"
                }
                if not atomic_write_json(POS_FILE, payload):
                    log.warning("vehicle_position.json atomic write failed; previous file preserved")
            except Exception as exc:
                log.warning("JSON write error: %s", exc)
            time.sleep(0.1)

    def stats_loop(self):
        while self.running:
            time.sleep(5.0)
            mode_str = "[BENCH]" if self._bench_override_active else "[ARMED]"
            log.info(f"{mode_str} Stats - Odom Rx: {self._msg_count_odom/5.0:.1f} Hz | PWM Rx: {self._msg_count_pwm/5.0:.1f} Hz | Posi: {self.pos_x:.2f},{self.pos_y:.2f} | PWM: {self.motor_pwm_ch1:.0f},{self.motor_pwm_ch2:.0f}")
            self._msg_count_odom = 0
            self._msg_count_pwm = 0

    def run(self):
        t1 = threading.Thread(target=self.receive_pwm, daemon=True)
        t1.start()
        t2 = threading.Thread(target=self.write_position_json, daemon=True)
        t2.start()
        t3 = threading.Thread(target=self.stats_loop, daemon=True)
        t3.start()
        
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            self.running = False

if __name__ == "__main__":
    _rl_bridge.info("service_init_complete", sitl_port=9002)
    b = SitlGazeboBridge()
    try:
        b.run()
    except KeyboardInterrupt:
        _rl_bridge.info("shutdown_requested", reason="keyboard_interrupt")
    finally:
        _rl_bridge.info("shutdown_complete")

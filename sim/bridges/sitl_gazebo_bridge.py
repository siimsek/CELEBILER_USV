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
from runtime_debug_log import log_jsonl, setup_component_logger

log = setup_component_logger("sitl_gazebo_bridge", prefer_simulation=True)

CONTROL_DIR = os.environ.get("CONTROL_DIR", str(_ROOT / "sim" / "control"))
POS_FILE = Path(CONTROL_DIR) / "vehicle_position.json"
MODEL_NAME = os.environ.get("SIM_GZ_MODEL_NAME", "usv_model")
SPAWN_X = float(os.environ.get("SIM_GZ_SPAWN_X", "0"))
SPAWN_Y = float(os.environ.get("SIM_GZ_SPAWN_Y", "0"))
POSE_ARM_S = max(0.0, float(os.environ.get("SIM_GZ_POSE_ARM_S", "0")))
MAX_LINEAR_VELOCITY_MPS = max(0.1, float(os.environ.get("SIM_GZ_MAX_LINEAR_VELOCITY_MPS", "1.4")))
MAX_YAW_RATE_RAD_S = max(0.1, float(os.environ.get("SIM_GZ_MAX_YAW_RATE_RAD_S", "1.0")))
LEFT_MOTOR_REVERSED = os.environ.get("SIM_GZ_LEFT_MOTOR_REVERSED", "0").lower() in ("1", "true", "yes")
RIGHT_MOTOR_REVERSED = os.environ.get("SIM_GZ_RIGHT_MOTOR_REVERSED", "0").lower() in ("1", "true", "yes")

# Simulation test mode: optional auto-inject for bench debugging only.
# Default is OFF to keep motion fully driven by SITL outputs.
SIM_TEST_MODE_ENABLED = os.environ.get("SIM_TEST_MODE", "0").lower() in ("1", "true", "yes")
TEST_MOTOR_CH1 = float(os.environ.get("SIM_TEST_MOTOR_CH1", "1600"))  # Test left motor (steering)
TEST_MOTOR_CH3 = float(os.environ.get("SIM_TEST_MOTOR_CH3", "1600"))  # Test right motor (throttle)
DRIVE_CMD_HZ = max(1.0, float(os.environ.get("SIM_GZ_DRIVE_CMD_HZ", "30")))

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
    left_n = (float(left_pwm) - 1500.0) / 500.0
    right_n = (float(right_pwm) - 1500.0) / 500.0
    if LEFT_MOTOR_REVERSED:
        left_n *= -1.0
    if RIGHT_MOTOR_REVERSED:
        right_n *= -1.0
    left_n = max(-1.0, min(1.0, left_n))
    right_n = max(-1.0, min(1.0, right_n))
    return left_n, right_n

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
        self.motor_pwm_ch1 = 1500.0 # Left motor output (Servo 1)
        self.motor_pwm_ch2 = 1500.0 # Right motor output (Servo 3)
        self.motor_pwm_raw = []
        self.servo_response_addr = ("127.0.0.1", SITL_JSON_IN_PORT)  # CRITICAL FIX: Default addr for first packet
        self.latest_sensor_payload = None  # Cache latest sensor data for JSON response
        self._test_mode_active = False  # Simulation test mode: auto-inject motor commands
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
        
        x_ned = p.y
        y_ned = p.x
        z_ned = -p.z
        
        vx_ned = v.y
        vy_ned = v.x
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
        
        # VALIDATION: Log coordinate transform periodically for diagnostics
        if self._msg_count_odom % 100 == 1:
            log_jsonl("coordinate_transform", {
                "msg_count": self._msg_count_odom,
                "gazebo_xyz": [round(p.x, 3), round(p.y, 3), round(p.z, 3)],
                "ned_xyz": [round(x_ned, 3), round(y_ned, 3), round(z_ned, 3)],
                "gps_latlon": [round(lat, 7), round(lon, 7)],
                "home_ref": [self.home_lat, self.home_lon],
                "validation": "OK" if (-90 <= lat <= 90 and -180 <= lon <= 180) else "RANGE_ERROR"
            })

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
            
        t = time.time()
        payload = {
            "timestamp": t,
            "imu": {
                "gyro": [av.y, av.x, -av.z],
                "accel_body": [0.0, 0.0, -9.81]
            },
            "position": [lat, lon, alt],
            "attitude": [roll, pitch, yaw],
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
                        self.servo_response_addr = addr  # Store sender for response (VEXP: bidirectional protocol fix)
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
            c1 = self.motor_pwm_ch1
            c2 = self.motor_pwm_ch2
            count = self._msg_count_pwm
        
        # [SIMULATION TEST MODE] Auto-inject motor commands when SITL disarmed
        # Real system: stays at 1500 (safe, disarmed)
        # Simulation: after POSE_ARM_S, inject test commands to move boat
        if SIM_TEST_MODE_ENABLED and time.monotonic() > self._pose_arm_deadline:
            if abs(c1 - 1500.0) < 10 and abs(c2 - 1500.0) < 10:  # SITL sending neutral (disarmed)
                if not self._test_mode_active:
                    log.info(f"[SIM-TEST] Activating simulation test motors: CH1={TEST_MOTOR_CH1} CH3={TEST_MOTOR_CH3}")
                    self._test_mode_active = True
                # Override with test motor values for simulation
                c1 = TEST_MOTOR_CH1
                c2 = TEST_MOTOR_CH3
                # Update state for vehicle_position.json tracking (Adım 4: state sync)
                with self.lock:
                    self.motor_pwm_ch1 = c1
                    self.motor_pwm_ch2 = c2
            
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
            yaw_diff = (left_n - right_n) / 2.0

            now = time.monotonic()
            if (now - self._last_drive_cmd_pub) >= (1.0 / DRIVE_CMD_HZ):
                self._last_drive_cmd_pub = now
                avg_velocity = avg_thrust * MAX_LINEAR_VELOCITY_MPS
                yaw_rate = yaw_diff * MAX_YAW_RATE_RAD_S

                twist = Twist()
                twist.linear.x = float(avg_velocity)
                twist.angular.z = float(yaw_rate)
                self.ros_node.pub_vel.publish(twist)

                if count % 1000 == 0:
                    log.debug(
                        f"[MOTOR-CMD] vx={avg_velocity:.2f}m/s wz={yaw_rate:.2f}rad/s "
                        f"left_n={left_n:.2f} right_n={right_n:.2f}"
                    )
        except Exception as e:
            if count % 500 == 0:
                log.error(f"[MOTOR-CMD-ERROR] Exception: {type(e).__name__}: {e}")
        
        # Debug: Log motor commands every 100 publishes
        if count % 100 == 0:
            mode_str = "[TEST]" if self._test_mode_active else "[ARMED]"
            log.debug(f"{mode_str} [MOTOR] PWM: CH1={c1:.0f} CH3={c2:.0f} → left_n={left_n:.2f} right_n={right_n:.2f}")

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
                        "position": [home_lat, home_lon, home_alt],
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
                
            if time.monotonic() <= self._pose_arm_deadline:
                if msg_count % 500 == 0:
                    log.debug(f"[SERVO] pose not armed yet (deadline {self._pose_arm_deadline:.1f})")
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
                
                left_n, right_n = _normalized_motor_outputs(ch1, ch2)
                motor_norm = max(-1.0, min(1.0, (left_n + right_n) / 2.0))
                POS_FILE.parent.mkdir(parents=True, exist_ok=True)
                payload = {
                    "ts": time.time(),
                    "pos_x": x,
                    "pos_y": y,
                    "heading_rad": hdg,
                    "motor_ch1_pwm": ch1,
                    "motor_ch2_pwm": ch2,
                    "motor_normalized": motor_norm,
                    "source": "sitl_gazebo_bridge_json"
                }
                POS_FILE.write_text(json.dumps(payload))
            except Exception as exc:
                log.warning("JSON write error: %s", exc)
            time.sleep(0.5)

    def stats_loop(self):
        while self.running:
            time.sleep(10.0)
            mode_str = "[TEST]" if self._test_mode_active else "[ARMED]"
            log.info(f"{mode_str} Stats - Odom Rx: {self._msg_count_odom/10.0:.1f} Hz | PWM Rx: {self._msg_count_pwm/10.0:.1f} Hz | Posi: {self.pos_x:.2f},{self.pos_y:.2f} | PWM: {self.motor_pwm_ch1:.0f},{self.motor_pwm_ch2:.0f}")
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
    b = SitlGazeboBridge()
    b.run()

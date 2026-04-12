#!/usr/bin/env python3
"""
ros_to_tcp_cam.py - Simulates the USV's TCP camera server
Subscribes to Gazebo camera topic over ROS 2 and serves JPEG
frames over localhost:8888, completely mimicking real hardware.
"""

import os
import socket
import sys
import threading
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT / "docker_workspace" / "src") not in sys.path:
    sys.path.insert(0, str(_ROOT / "docker_workspace" / "src"))
from runtime_debug_log import install_module_function_tracing, log_jsonl, setup_component_logger
from compliance_profile import USV_MODE, USV_MODE_RACE

# --- RACE MODE GUARD: Image transmission prohibited in race mode ---
if USV_MODE == USV_MODE_RACE:
    print("[RACE] [ros_to_tcp_cam.py] Race mode: camera stream disabled, exiting.")
    sys.exit(0)

log = setup_component_logger("ros_to_tcp_cam", prefer_simulation=True)
log.info("module_load cwd=%s SIM_LOG_DIR=%s", os.getcwd(), os.environ.get("SIM_LOG_DIR"))

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_tcp_bridge')
        self.bridge = CvBridge()
        self.latest_jpeg = None
        self.placeholder_jpeg = self._build_placeholder_jpeg()
        self._closing = False
        self._cb_count = 0
        self._last_cb_log = 0.0
        
        # Subscribe to bridge output
        # E.g., we expect Gazebo camera at /camera/image_raw
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            50
        )
        self.get_logger().info('Subscribed to /camera/image_raw')
        log.info("Subscribed to /camera/image_raw")

        # Start TCP Server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.settimeout(1.0)
        self.server_socket.bind(('127.0.0.1', 8888))
        self.server_socket.listen(5)
        self.get_logger().info('TCP Server running on 127.0.0.1:8888')
        log.info("TCP Server running on 127.0.0.1:8888")
        
        self.client_thread = threading.Thread(target=self.accept_clients, daemon=True)
        self.client_thread.start()

    def _build_placeholder_jpeg(self):
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        frame[:, :] = (180, 110, 30)
        cv2.putText(frame, 'SIM CAMERA FALLBACK', (310, 320), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255), 3)
        cv2.putText(frame, 'ROS image missing, synthetic stream active', (265, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (245, 245, 245), 2)
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        return buf.tobytes() if ok else b''

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Compress to JPG
            ret, buf = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ret:
                # Add magic bytes FFD8 and FFD9 if they are not implicit,
                # though imencode provides them automatically for jpeg.
                self.latest_jpeg = buf.tobytes()
                self._cb_count += 1
                now = time.time()
                if self._cb_count <= 3 or now - self._last_cb_log >= 5.0:
                    self._last_cb_log = now
                    h, w = cv_img.shape[:2]
                    jb = len(self.latest_jpeg or b"")
                    log.debug(
                        "image_callback n=%s size=%sx%s jpeg_bytes=%s",
                        self._cb_count,
                        w,
                        h,
                        jb,
                    )
                    log_jsonl(
                        "ros_to_tcp_cam",
                        True,
                        event="frame",
                        n=self._cb_count,
                        w=w,
                        h=h,
                        jpeg_bytes=jb,
                    )
        except Exception as e:
            self.get_logger().error(f"Error encoding image: {e}")
            log.exception("image encode error: %s", e)

    def accept_clients(self):
        while not self._closing:
            try:
                client_sock, addr = self.server_socket.accept()
                self.get_logger().info(f"Client connected from {addr}")
                log.info("tcp client connected from %s", addr)
                client_handler = threading.Thread(
                    target=self.handle_client,
                    args=(client_sock,),
                    daemon=True
                )
                client_handler.start()
            except socket.timeout:
                continue
            except OSError as e:
                if self._closing:
                    break
                self.get_logger().error(f"Accept error: {e}")
            except Exception as e:
                self.get_logger().error(f"Accept error: {e}")

    def handle_client(self, client_sock):
        try:
            while True:
                frame = self.latest_jpeg if self.latest_jpeg is not None else self.placeholder_jpeg
                if frame:
                    client_sock.sendall(frame)
                time.sleep(0.05) # 20 Hz approx
        except Exception as e:
            self.get_logger().info(f"Client disconnected: {e}")
        finally:
            client_sock.close()

    def close(self):
        self._closing = True
        try:
            self.server_socket.close()
        except Exception:
            pass

def main(args=None):
    log.info("main() rclpy.init")
    rclpy.init(args=args)
    bridge_node = CameraBridge()
    try:
        rclpy.spin(bridge_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        bridge_node.close()
        bridge_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

install_module_function_tracing(
    globals(),
    component="ros_to_tcp_cam",
    logger=log,
    prefer_simulation=True,
)

if __name__ == '__main__':
    main()

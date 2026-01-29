#!/bin/bash
# Dosya Yeri: /root/workspace/scripts/internal_start.sh

# ROS Ortamı
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

echo "[DOCKER] Lidar baslatiliyor..."
ros2 launch rplidar_ros rplidar_s2e_launch.py channel_type:=udp tcp_ip:=192.168.11.2 tcp_port:=20108 frame_id:=laser_frame > /root/workspace/logs/lidar.log 2>&1 &
LIDAR_PID=$!
sleep 5

echo "[DOCKER] Otonom kod (autonomous_main.py) baslatiliyor..."
# Buradaki dosya yolunu kendi ana koduna göre kontrol et!
python3 /root/workspace/src/autonomous_main.py &
MAIN_PID=$!

wait $LIDAR_PID $MAIN_PID

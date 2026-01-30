#!/bin/bash
# Dosya Yeri: /root/workspace/scripts/internal_start.sh

# --- ORTAM KURULUMU ---
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export TZ=Europe/Istanbul

mkdir -p /root/workspace/logs

echo "--- [DOCKER] USV SİSTEMLERİ BAŞLATILIYOR ---"

# 1. LIDAR BAŞLAT
echo "🚀 [LIDAR] Başlatılıyor..."
# S2E / UDP Modu için genelde 'ip' parametresi kullanılır, tcp_ip eski olabilir.
ros2 launch rplidar_ros rplidar_s2e_launch.py channel_type:=udp ip:=192.168.11.2 tcp_port:=20108 frame_id:=laser_frame scan_mode:=Standard > /root/workspace/logs/lidar.log 2>&1 &
# Lidar'ın ısınması için bekle
sleep 5

# 2. HARİTA SUNUCUSU (Port 5001)
echo "🗺️  [WEB] Harita Sunucusu (lidar_map.py) Başlatılıyor..."
python3 -u /root/workspace/src/lidar_map.py > /root/workspace/logs/map.log 2>&1 &

# 3. KAMERA SUNUCUSU (Port 5000)
echo "📷 [WEB] Kamera Sunucusu (cam.py) Başlatılıyor..."
nice -n -10 python3 -u /root/workspace/src/cam.py > /root/workspace/logs/cam.log 2>&1 &

# 4. DASHBOARD (Port 8080)
echo "🌍 [WEB] Ana Dashboard (telemetry.py) Başlatılıyor..."
python3 -u /root/workspace/src/telemetry.py > /root/workspace/logs/telemetry.log 2>&1 &

# 5. OTONOM BEYİN (Fusion Main - Lidar Destekli)
# autonomous_main.py sadece pusula kullanır, fusion_main.py lidar kullanır.
# Hangisini kullanmak istiyorsan onu aç. Tavsiyem fusion_main.py
echo "🧠 [MAIN] Otonom Pilot (fusion_main.py) Başlatılıyor..."
python3 /root/workspace/src/fusion_main.py > /root/workspace/logs/fusion.log 2>&1 &

# Arka plandaki süreçlerin çalışmaya devam etmesi için bekle
wait

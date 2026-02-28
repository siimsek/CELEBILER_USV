#!/bin/bash
# Dosya Yeri: /root/workspace/scripts/internal_start.sh

# --- ORTAM KURULUMU ---
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export TZ=Europe/Istanbul

# USV_MODE: test (full web) | race (no image transmission - IDA 3.7)
USV_MODE="${USV_MODE:-test}"
export USV_MODE

mkdir -p /root/workspace/logs
mkdir -p /root/workspace/control

echo "--- [DOCKER] USV SİSTEMLERİ BAŞLATILIYOR (Mod: $USV_MODE) ---"

# 1. LIDAR BAŞLAT (Her iki modda da - onboard kullanım için)
echo "🚀 [LIDAR] Başlatılıyor..."
ros2 launch rplidar_ros rplidar_s2e_launch.py channel_type:=udp ip:=192.168.11.2 tcp_port:=20108 frame_id:=laser_frame > /root/workspace/logs/lidar.log 2>&1 &
sleep 5

# 2. HARİTA SUNUCUSU (Port 5001) - Sadece TEST modunda (görüntü aktarımı yasak)
if [ "$USV_MODE" = "race" ]; then
    echo "🏁 [MOD] YARIŞMA - lidar_map.py başlatılmıyor (IDA 3.7)"
else
    echo "🗺️  [WEB] Harita Sunucusu (lidar_map.py) Başlatılıyor..."
    python3 -u /root/workspace/src/lidar_map.py > /root/workspace/logs/map.log 2>&1 &
fi

# 3. KAMERA SUNUCUSU - Her iki modda da başlatılır
# Race modunda: onboard engel tespiti (web yayını kapalı - IDA 3.7)
# Test modunda: web yayını aktif (port 5000)
echo "📷 [CAM] Kamera Sunucusu (cam.py) Başlatılıyor..."
nice -n -10 python3 -u /root/workspace/src/cam.py > /root/workspace/logs/cam.log 2>&1 &

# 4. DASHBOARD (Port 8080) - Her iki modda da (YKİ gereksinimi)
echo "🌍 [WEB] Ana Dashboard (telemetry.py) Başlatılıyor..."
python3 -u /root/workspace/src/telemetry.py > /root/workspace/logs/telemetry.log 2>&1 &

# 5. OTONOM BEYİN (usv_main.py — Merkezi Durum Makinesi)
# GPS navigasyon + Lidar engel kaçınma + Parkur otomatik geçişi
echo "🧠 [MAIN] Otonom Pilot (usv_main.py) Başlatılıyor..."
python3 -u /root/workspace/src/usv_main.py > /root/workspace/logs/autonomous.log 2>&1 &

# Arka plandaki süreçlerin çalışmaya devam etmesi için bekle
wait

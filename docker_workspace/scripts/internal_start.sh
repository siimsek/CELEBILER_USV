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

# --- 0. MAVPROXY KURULUM & BAŞLATMA ---
# Pixhawk seri portunu paylaştırır (telemetry.py + usv_main.py aynı anda bağlanabilir)
if ! command -v mavproxy.py &> /dev/null; then
    echo "📦 [SETUP] mavproxy kuruluyor..."
    pip3 install -q MAVProxy 2>/dev/null || pip install -q MAVProxy 2>/dev/null
fi

# Pixhawk portunu otomatik bul
PIXHAWK_PORT=""
for port in /dev/ttyACM* /dev/ttyUSB*; do
    [ -e "$port" ] || continue
    # STM32 genelde 9600 baud JSON gönderir, Pixhawk 115200 MAVLink
    # İlk bulunan ACM portu genelde Pixhawk; hepsini dene
    timeout 2 python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('$port', baud=115200)
if m.wait_heartbeat(timeout=1.5):
    print('OK')
    m.close()
else:
    m.close()
    exit(1)
" 2>/dev/null && PIXHAWK_PORT="$port" && break
done

if [ -n "$PIXHAWK_PORT" ]; then
    echo "📡 [MAV] Pixhawk: $PIXHAWK_PORT -> UDP :14550 (telemetry) + :14551 (autonomous)"
    mavproxy.py --master=$PIXHAWK_PORT --baudrate=115200 \
        --out=udpout:127.0.0.1:14550 \
        --out=udpout:127.0.0.1:14551 \
        --daemon --non-interactive \
        > /root/workspace/logs/mavproxy.log 2>&1 &
    sleep 2
    echo "✅ [MAV] MAVProxy aktif"
else
    echo "⚠️ [MAV] Pixhawk bulunamadı — doğrudan bağlantı denenecek"
fi

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

#!/bin/bash
# Dosya Yeri: /root/workspace/scripts/internal_start.sh

# --- ORTAM KURULUMU ---
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export TZ=Europe/Istanbul
export USV_PROJECT_ROOT="/root/workspace"

# USV_MODE: test (full web) | race (no image transmission - IDA 3.7)
USV_MODE="${USV_MODE:-test}"
export USV_MODE
if [ "$USV_MODE" = "race" ]; then
    if [ "${USV_USE_RC_OVERRIDE:-}" = "1" ]; then
        echo "❌ [HATA] Race modda USV_USE_RC_OVERRIDE=1 yasak; Pixhawk/ArduPilot PWM sahibi kalmalıdır."
        exit 1
    fi
    if [ "${SIM_GZ_ALLOW_MOTOR_COMMAND_JSON:-}" = "1" ]; then
        echo "❌ [HATA] Race modda SIM_GZ_ALLOW_MOTOR_COMMAND_JSON=1 yasak; bench motor bypass kapalı olmalıdır."
        exit 1
    fi
fi

mkdir -p /root/workspace/logs
mkdir -p /root/workspace/control

export LOG_DIR="/root/workspace/logs"
export USV_LOG_ROOT="/root/workspace/logs"
export CONTROL_DIR="/root/workspace/control"
export MISSION_FILE="${MISSION_FILE:-/root/workspace/mission.json}"
echo "[$(date -Iseconds)] internal_start USV_MODE=$USV_MODE" >> "$LOG_DIR/host_trace.log"

echo "--- [DOCKER] USV SİSTEMLERİ BAŞLATILIYOR (Mod: $USV_MODE) ---"

# --- 0. MAVPROXY KURULUM & BAŞLATMA ---
# Pixhawk seri portunu paylaştırır (telemetry.py + usv_main.py aynı anda bağlanabilir)
if ! command -v mavproxy.py &> /dev/null; then
    echo "📦 [SETUP] mavproxy kuruluyor..."
    pip3 install -q MAVProxy 2>/dev/null || pip install -q MAVProxy 2>/dev/null
fi

# Pixhawk portunu otomatik bul
PIXHAWK_PORT="${PIXHAWK_PORT:-auto}"
PIXHAWK_BAUD="${PIXHAWK_BAUD:-115200}"
if [ "$PIXHAWK_PORT" = "auto" ]; then
    PIXHAWK_PORT=""
    for port in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$port" ] || continue
        # STM32 genelde 9600 baud JSON gönderir, Pixhawk MAVLink baud'u env ile belirlenir.
        timeout 2 python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('$port', baud=int('$PIXHAWK_BAUD'))
if m.wait_heartbeat(timeout=1.5):
    print('OK')
    m.close()
else:
    m.close()
    exit(1)
" 2>/dev/null && PIXHAWK_PORT="$port" && break
    done
fi

MAVPROXY_ARGS=(
    "--master=$PIXHAWK_PORT"
    "--baudrate=$PIXHAWK_BAUD"
    "--out=udpout:127.0.0.1:14550"
    "--out=udpout:127.0.0.1:14551"
)
if [ -n "${MISSION_PLANNER_UDP_OUT:-}" ]; then
    MAVPROXY_ARGS+=("--out=${MISSION_PLANNER_UDP_OUT}")
fi

if [ -n "$PIXHAWK_PORT" ]; then
    echo "📡 [MAV] Pixhawk: $PIXHAWK_PORT baud=$PIXHAWK_BAUD"
    echo "📡 [MAV] MAVProxy out: udpout:127.0.0.1:14550 (telemetry)"
    echo "📡 [MAV] MAVProxy out: udpout:127.0.0.1:14551 (usv_main)"
    echo "📡 [MAV] Mission Planner birincil bağlantı: 433 MHz Pixhawk telemetri modemi"
    if [ -n "${MISSION_PLANNER_UDP_OUT:-}" ]; then
        echo "📡 [MAV] Ek Mission Planner UDP çıkışı: ${MISSION_PLANNER_UDP_OUT}"
    fi
    mavproxy.py "${MAVPROXY_ARGS[@]}" \
        --daemon --non-interactive \
        > /root/workspace/logs/mavproxy.log 2>&1 &
    sleep 0.5
    echo "✅ [MAV] MAVProxy aktif"
else
    echo "⚠️ [MAV] Pixhawk bulunamadı — doğrudan bağlantı denenecek"
fi

# 1. LIDAR BAŞLAT (Her iki modda da - onboard kullanım için)
echo "🚀 [LIDAR] Başlatılıyor..."
ros2 launch rplidar_ros rplidar_s2e_launch.py channel_type:=udp ip:=192.168.11.2 tcp_port:=20108 frame_id:=laser_frame > /root/workspace/logs/lidar.log 2>&1 &
sleep 2

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

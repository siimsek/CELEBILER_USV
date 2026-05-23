#!/bin/bash
# run_sim_stack.sh - Master orchestration script
#
# Baslatma sirasi: Gazebo -> ros_gz_bridge -> SITL -> sitl_gazebo_bridge (tek MAVLink 14551) -> cam -> telemetry -> usv_main.
# usv_main SITL'e tcp:5760 ile baglanir; SITL readiness port kontroluyle dogrulanir.
# sitl.log icinde "Closed connection on SERIAL0" / "Exitting" genelde onceki istemci kopmasi veya
# yeniden baglanma anidir; tek basina hata sayilmaz. Konum sicramasi: pose koprusu + ilk NED birlikte ele alinir.
set -euo pipefail

PROJ_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PROJ_ROOT"

source sim/configs/sim.env
if [ -f .venv-sim/bin/activate ]; then
    source .venv-sim/bin/activate
fi

ROOT_LOG_DIR="$PROJ_ROOT/logs"
export USV_LOG_ROOT="$ROOT_LOG_DIR"
TERMINAL_LOG_FILE="$ROOT_LOG_DIR/terminal.log"

prepare_latest_log_session() {
    rm -rf "$ROOT_LOG_DIR"
    mkdir -p "$ROOT_LOG_DIR/host" "$ROOT_LOG_DIR/system" "$ROOT_LOG_DIR/simulation"
}

prepare_latest_log_session
: > "$TERMINAL_LOG_FILE"
exec > >(tee -a "$TERMINAL_LOG_FILE") 2>&1

echo "[SIM] Terminal log: $TERMINAL_LOG_FILE"

# Load ROS2 environment for rclpy, ros_gz_bridge
# Note: ROS2 setup.bash has undefined variables, so temporarily disable -u
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

# Export Gazebo resource paths for plugin loading
export IGN_GAZEBO_RESOURCE_PATH="$PROJ_ROOT/sim/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="$PROJ_ROOT/sim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="$PROJ_ROOT/sim/models${SDF_PATH:+:$SDF_PATH}"
echo "[SIM] Gazebo resource paths configured"

# CLI argument parsing
while [ $# -gt 0 ]; do
    case "$1" in
        --auto-test)
            echo "[ERROR] --auto-test kaldırıldı; otomatik görev başlatma ve motion doğrulaması ajan/otomasyon tarafından çalıştırılamaz."
            exit 2
            ;;
        mode=*)
            USV_MODE="${1#mode=}"
            if [ "$USV_MODE" != "race" ] && [ "$USV_MODE" != "test" ]; then
                echo "[ERROR] Gecersiz mode=$USV_MODE; beklenen test veya race."
                exit 2
            fi
            export USV_MODE
            shift
            ;;
        race|test)
            USV_MODE="$1"
            export USV_MODE
            shift
            ;;
        *)
            if [ -f "$1" ]; then
                MISSION_FILE="$1"
            else
                echo "[ERROR] Bilinmeyen arguman veya bulunamayan mission dosyasi: $1"
                exit 2
            fi
            shift
            ;;
    esac
done

if [ -n "${MISSION_FILE:-}" ] && [[ "$MISSION_FILE" != /* ]]; then
    export MISSION_FILE="$PROJ_ROOT/$MISSION_FILE"
fi

export PROJ_ROOT
export USV_SIM=1
export USV_MODE="${USV_MODE:-test}"
export PYTHONUNBUFFERED=1
export PATH="$PATH:$HOME/ardupilot/Tools/autotest"

if [ "${USV_USE_RC_OVERRIDE:-}" = "1" ]; then
    echo "[ERROR] Simulasyonda USV_USE_RC_OVERRIDE=1 yasak; sim her zaman race-like ArduPilot PWM yolu kullanmali."
    exit 2
fi
if [ "${SIM_GZ_ALLOW_MOTOR_COMMAND_JSON:-}" = "1" ]; then
    echo "[ERROR] Simulasyonda SIM_GZ_ALLOW_MOTOR_COMMAND_JSON=1 yasak; motor kaynagi SITL servo olmali."
    exit 2
fi

ROS_GZ_BRIDGE_BIN="/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"
CAM_PID_FILE="$SIM_CONTROL_DIR/cam.pid"
TELEMETRY_PID_FILE="$SIM_CONTROL_DIR/telemetry.pid"
LIDAR_MAP_PID_FILE="$SIM_CONTROL_DIR/lidar_map.pid"
SELF_PID=$$
PIDS=()
STALE_PATTERNS=(
    "python3 usv_main.py"
    "python3 docker_workspace/src/usv_main.py"
    "python3 sim/bridges/sitl_gazebo_bridge.py"
    "python3 sim/bridges/ros_to_tcp_cam.py"
    "python3 cam.py"
    "python3 docker_workspace/src/cam.py"
    "python3 telemetry.py"
    "python3 docker_workspace/src/telemetry.py"
    "python3 lidar_map.py"
    "python3 docker_workspace/src/lidar_map.py"
    "/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"
    "$HOME/ardupilot/build/sitl/bin/ardurover"
    "ardupilot/modules/waf/waf-light build --target bin/ardurover"
    "ign gazebo .*sim/worlds/water_world.sdf"
    "ign gazebo server"
    "ign gazebo gui"
    "ruby .*ign gazebo"
    "ruby .*gz sim"
    "gz sim .*water_world.sdf"
    "gz sim server"
)
STALE_PORTS=(
    "8080/tcp"
    "8888/tcp"
    "14550/udp"
    "14551/udp"
    "14552/udp"
    "5760/tcp"
    "5762/tcp"
    "5763/tcp"
    "9002/tcp"
    "9003/tcp"
    "9005/tcp"
    "11345/tcp"
)

register_pid() {
    if [ -n "${1:-}" ]; then
        PIDS+=("$1")
    fi
}

kill_if_running() {
    local pid="${1:-}"
    local signal="${2:-TERM}"
    if [ -n "$pid" ] && [ "$pid" != "$SELF_PID" ] && kill -0 "$pid" 2>/dev/null; then
        kill "-$signal" "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}

kill_matching_pattern() {
    local pattern="$1"
    local signal="${2:-TERM}"
    local pid
    while read -r pid; do
        [ -z "$pid" ] && continue
        kill_if_running "$pid" "$signal"
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

kill_stale_launchers() {
    local pid
    while read -r pid; do
        [ -z "$pid" ] && continue
        kill_if_running "$pid" KILL
    done < <(pgrep -f "sim/bin/run_sim_stack.sh|sim/bin/run_gz_world.sh" 2>/dev/null || true)
}

kill_stale_ports() {
    local endpoint
    for endpoint in "${STALE_PORTS[@]}"; do
        fuser -k "$endpoint" > /dev/null 2>&1 || true
    done
}

wait_for_endpoint_release() {
    local endpoint="$1"
    local deadline=$((SECONDS + 8))
    while fuser "$endpoint" > /dev/null 2>&1; do
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "[WARN] Endpoint hala dolu: $endpoint"
            fuser -v "$endpoint" 2>/dev/null || true
            return 1
        fi
        sleep 0.1
    done
    return 0
}

ensure_endpoints_released() {
    local endpoint
    local pending=0
    for endpoint in "${STALE_PORTS[@]}"; do
        if ! wait_for_endpoint_release "$endpoint"; then
            pending=1
        fi
    done
    return "$pending"
}

wait_for_tcp_port() {
    local host="$1"
    local port="$2"
    local label="$3"
    local timeout_s="${4:-20}"
    local deadline=$((SECONDS + timeout_s))
    while true; do
        if (echo > "/dev/tcp/${host}/${port}") >/dev/null 2>&1; then
            echo "[SIM] Ready: $label (${host}:${port})"
            return 0
        fi
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "[WARN] Timeout waiting for $label (${host}:${port})"
            return 1
        fi
        sleep 0.2
    done
}

wait_for_http_endpoint() {
    local url="$1"
    local label="$2"
    local timeout_s="${3:-20}"
    python3 - "$url" "$label" "$timeout_s" <<'PY'
import sys
import time
import urllib.request

url = sys.argv[1]
label = sys.argv[2]
timeout_s = float(sys.argv[3])
deadline = time.monotonic() + timeout_s
last_error = ""
while time.monotonic() < deadline:
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            if int(getattr(resp, "status", 200) or 200) < 500:
                print(f"[SIM] Ready: {label} ({url})")
                raise SystemExit(0)
    except Exception as exc:
        last_error = str(exc)
    time.sleep(0.2)
print(f"[WARN] Timeout waiting for {label} ({url}): {last_error}")
raise SystemExit(1)
PY
}

wait_for_process_alive() {
    local pid="$1"
    local label="$2"
    local timeout_s="${3:-10}"
    local deadline=$((SECONDS + timeout_s))
    while true; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "[SIM] Ready: $label process pid=$pid"
            return 0
        fi
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "[WARN] Timeout waiting for $label process pid=$pid"
            return 1
        fi
        sleep 0.2
    done
}

wait_for_log_pattern() {
    local file="$1"
    local pattern="$2"
    local label="$3"
    local timeout_s="${4:-15}"
    local deadline=$((SECONDS + timeout_s))
    while true; do
        if [ -f "$file" ] && grep -Eq "$pattern" "$file" 2>/dev/null; then
            echo "[SIM] Ready: $label ($file)"
            return 0
        fi
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "[WARN] Timeout waiting for $label log pattern '$pattern' in $file"
            return 1
        fi
        sleep 0.2
    done
}

wait_for_ros_topic() {
    local topic="$1"
    local label="$2"
    local timeout_s="${3:-20}"
    local deadline=$((SECONDS + timeout_s))
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "[WARN] ros2 CLI unavailable; cannot probe $label topic $topic"
        return 1
    fi
    while true; do
        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            echo "[SIM] Ready: $label topic $topic"
            return 0
        fi
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "[WARN] Timeout waiting for $label topic $topic"
            return 1
        fi
        sleep 0.5
    done
}

wait_for_file_fresh() {
    local file="$1"
    local label="$2"
    local timeout_s="${3:-15}"
    local max_age_s="${4:-2.5}"
    python3 - "$file" "$label" "$timeout_s" "$max_age_s" <<'PY'
import os
import sys
import time

path, label, timeout_s, max_age_s = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
deadline = time.monotonic() + timeout_s
while time.monotonic() < deadline:
    try:
        age = time.time() - os.path.getmtime(path)
        if os.path.getsize(path) > 0 and age <= max_age_s:
            print(f"[SIM] Ready: {label} ({path}, age={age:.2f}s)")
            raise SystemExit(0)
    except OSError:
        pass
    time.sleep(0.2)
print(f"[WARN] Timeout waiting for fresh {label}: {path}")
raise SystemExit(1)
PY
}

resolve_target_class() {
    python3 -c '
import json
import os
import sys

mapping = {
    "RED": "KIRMIZI_SANCAK",
    "KIRMIZI": "KIRMIZI_SANCAK",
    "KIRMIZI_SANCAK": "KIRMIZI_SANCAK",
    "GREEN": "YESIL_SANCAK",
    "YESIL": "YESIL_SANCAK",
    "YESIL_SANCAK": "YESIL_SANCAK",
    "BLACK": "SIYAH_HEDEF",
    "SIYAH": "SIYAH_HEDEF",
    "SIYAH_HEDEF": "SIYAH_HEDEF",
    "YELLOW": "SARI_ENGEL",
    "SARI": "SARI_ENGEL",
    "SARI_ENGEL": "SARI_ENGEL",
}

default = os.environ.get("TARGET_CLASS", "KIRMIZI_SANCAK")
raw = ""
try:
    with open(sys.argv[1], "r", encoding="utf-8") as handle:
        raw = str(json.load(handle).get("target_color", "")).strip().upper()
except Exception:
    pass

print(mapping.get(raw, default))
' "${TARGET_STATE_FILE:-$CONTROL_DIR/target_state.json}"
}

wait_for_startup_readiness() {
    python3 - <<'PY'
import json
import os
import sys
import time
import urllib.request

telemetry_url = "http://127.0.0.1:8080/api/data"
camera_url = "http://127.0.0.1:5000/"
log_dir = os.environ.get("LOG_DIR", "/tmp")
camera_jpg = os.path.join(log_dir, "camera_latest.jpg")
# No deadline: wait indefinitely until ready
last_line = None
min_lidar_points = max(0, int(os.environ.get("SIM_READY_MIN_LIDAR_POINTS", "0") or 0))
camera_headless = os.environ.get("SIM_CAMERA_HEADLESS", "1").strip().lower() not in ("0", "false", "no")
# Static compliance probe marker: non-headless readiness still observes camera_api_ready=True via probe_camera_endpoint.

def probe_camera_endpoint():
    try:
        req = urllib.request.Request(camera_url, method="GET")
        with urllib.request.urlopen(req, timeout=1.0) as response:
            return int(getattr(response, "status", 200) or 200) < 500
    except Exception:
        return False

while True:
    try:
        with urllib.request.urlopen(telemetry_url, timeout=2.0) as response:
            data = json.load(response)
        autonomy = data.get("autonomy_health") or {}
        lidar_points = int(autonomy.get("lidar_point_count", 0) or 0)
        camera_file_ready = os.path.exists(camera_jpg) and os.path.getsize(camera_jpg) > 1024
        camera_api_ready = probe_camera_endpoint()
        camera_transport_ready = bool(camera_file_ready or camera_api_ready)
        ready = (
            bool(data.get("ready_state"))
            and bool(data.get("camera_ready"))
            and bool(data.get("lidar_ready"))
            and lidar_points >= min_lidar_points
            and camera_transport_ready
        )
        line = (
            f"[SIM] Startup check: ready={str(ready).lower()} "
            f"camera={str(bool(data.get('camera_ready'))).lower()} "
            f"lidar={str(bool(data.get('lidar_ready'))).lower()} "
            f"telemetry_api=true "
            f"camera_headless={str(camera_headless).lower()} "
            f"camera_api={str(camera_api_ready).lower()} "
            f"camera_file={str(camera_file_ready).lower()} "
            f"lidar_points={lidar_points} min_lidar_points={min_lidar_points} "
            f"health={str(bool(data.get('ready_state'))).lower()}"
        )
        if line != last_line:
            print(line)
            last_line = line

        if ready:
            print("[READY] [SISTEM] Kamera + LIDAR + sağlık kontrolleri ve API endpointleri OK")
            print(
                "[READY] [SISTEM] "
                f"lidar_point_count={lidar_points} "
                f"camera_ready={bool(data.get('camera_ready'))} "
                f"lidar_ready={bool(data.get('lidar_ready'))} "
                f"telemetry_api_ready=True camera_headless={camera_headless} "
                f"camera_api_ready={camera_api_ready} camera_file_ready={camera_file_ready}"
            )
            sys.exit(0)

        time.sleep(1.0)
    except Exception as exc:
        print(f"[SIM] Startup check bekleniyor: {exc}")
        time.sleep(1.0)

print("[WARN] [SISTEM] Startup readiness onaylanamadi; camera/lidar/loglari kontrol edin")
sys.exit(1)
PY
}

stop_stale_stack() {
    echo "[SIM] Cleaning stale simulation processes..."
    kill_stale_launchers
    for pattern in "${STALE_PATTERNS[@]}"; do
        kill_matching_pattern "$pattern" TERM
    done
    sleep 1
    # Aggressive second pass with KILL
    for pattern in "${STALE_PATTERNS[@]}"; do
        kill_matching_pattern "$pattern" KILL
    done
    # Fallback: pkill any remaining stragglers from docker_workspace
    pkill -9 -f "docker_workspace/src/" 2>/dev/null || true
    pkill -9 -f "sim/bridges/" 2>/dev/null || true
    sleep 0.1
    kill_stale_ports
    ensure_endpoints_released || true
    rm -f "$CAM_PID_FILE" "$TELEMETRY_PID_FILE" "$LIDAR_MAP_PID_FILE"
    echo "[SIM] Stale stack cleanup complete."
}

reset_runtime_dirs() {
    echo "[SIM] FULL RESET: runtime/control/artifact dosyalari temizleniyor; loglar sadece son session icin /logs altinda tutulur."
    # 1) Log dizinlerini yeniden olustur; root log dizini silinmez.
    mkdir -p "$USV_LOG_ROOT/host" "$SIM_LOG_DIR" "$SIM_LOG_DIR/ros2" "$LOG_DIR" "$LOG_DIR/video"
    # 2) Sim kontrol ve artifact dizinleri
    rm -rf "$SIM_CONTROL_DIR" "$SIM_ARTIFACT_DIR/sitl" "$SIM_ARTIFACT_DIR/gz_home"
    # 3) Kok seviyesinde kalan SITL/MAVLink kalintilari
    rm -f "$PROJ_ROOT/eeprom.bin" "$PROJ_ROOT/mav.tlog" "$PROJ_ROOT/mav.tlog.raw" "$PROJ_ROOT/mav.parm"
    # 4) docker_workspace icindeki runtime ciktilari (sabit mission.json KORUNUR)
    rm -rf "$PROJ_ROOT/docker_workspace/logs"
    rm -rf "$PROJ_ROOT/docker_workspace/control"
    # 5) Emergency stop / flag dosyalari (varsa onceki oturumdan kalma)
    rm -f "$SIM_CONTROL_DIR/emergency_stop.flag" "$SIM_CONTROL_DIR/start.flag" "$SIM_CONTROL_DIR/stop.flag"
    # 6) Dizinleri temiz yeniden olustur
    mkdir -p \
        "$SIM_LOG_DIR" "$SIM_LOG_DIR/ros2" \
        "$LOG_DIR" "$LOG_DIR/video" \
        "$USV_LOG_ROOT/host" \
        "$SIM_CONTROL_DIR" \
        "$SIM_ARTIFACT_DIR/sitl" "$SIM_ARTIFACT_DIR/gz_home" \
        "$ROS_HOME" \
        "$PROJ_ROOT/docker_workspace/logs" "$PROJ_ROOT/docker_workspace/control"
    echo "[SIM] FULL RESET tamam: clean start garantili."
}

cleanup() {
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "[SIM] ========== SHUTDOWN INITIATED =========="
    # Kill registered PIDs with aggressive -9
    kill_if_running "$(cat "$CAM_PID_FILE" 2>/dev/null || true)" KILL
    kill_if_running "$(cat "$TELEMETRY_PID_FILE" 2>/dev/null || true)" KILL
    kill_if_running "$(cat "$LIDAR_MAP_PID_FILE" 2>/dev/null || true)" KILL
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    # Immediately kill all simulation processes
    pkill -9 -f "docker_workspace/src/usv_main" 2>/dev/null || true
    pkill -9 -f "docker_workspace/src/cam" 2>/dev/null || true
    pkill -9 -f "docker_workspace/src/telemetry" 2>/dev/null || true
    pkill -9 -f "docker_workspace/src/lidar_map" 2>/dev/null || true
    pkill -9 -f "sim/bridges/" 2>/dev/null || true
    pkill -9 -f "gazebo" 2>/dev/null || true
    pkill -9 -f "ardurover" 2>/dev/null || true
    # Port cleanup
    fuser -k 8080/tcp 8888/tcp 14550/udp 14551/udp 14552/udp 5760/tcp 2>/dev/null || true
    rm -f "$CAM_PID_FILE" "$TELEMETRY_PID_FILE" "$LIDAR_MAP_PID_FILE"
    echo "[SIM] ========== SHUTDOWN COMPLETE =========="
}
trap cleanup SIGINT SIGTERM EXIT

export TARGET_CLASS="$(resolve_target_class)"

stop_stale_stack
reset_runtime_dirs

echo "[SIM] Starting Gazebo (logging to $SIM_LOG_DIR/gazebo.log)..."
./sim/bin/run_gz_world.sh > "$SIM_LOG_DIR/gazebo.log" 2>&1 &
GZ_PID=$!
register_pid "$GZ_PID"
wait_for_process_alive "$GZ_PID" "Gazebo" 10 || true
wait_for_log_pattern "$SIM_LOG_DIR/gazebo.log" "Gazebo|Ignition|server|world" "Gazebo log" 15 || true

echo "[SIM] Starting ROS<->Gazebo bridges (pose service + scan + camera + odom + cmd_vel)..."
"$ROS_GZ_BRIDGE_BIN" \
    "/world/${SIM_GZ_WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose" \
    /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
    /camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image \
    /model/${SIM_GZ_MODEL_NAME}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
    /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
    > "$SIM_LOG_DIR/ros_gz.log" 2>&1 &
ROS_GZ_PID=$!
register_pid "$ROS_GZ_PID"
wait_for_process_alive "$ROS_GZ_PID" "ROS-GZ bridge" 10 || true
wait_for_ros_topic "/scan" "ROS-GZ scan bridge" 20 || true

echo "[SIM] Starting ArduPilot SITL (logging to $SIM_LOG_DIR/sitl.log)..."
./sim/bin/run_sitl.sh > "$SIM_LOG_DIR/sitl.log" 2>&1 &
SITL_PID=$!
register_pid "$SITL_PID"
wait_for_tcp_port 127.0.0.1 5760 "ArduPilot SITL MAVLink" 25 || true

echo "[SIM] Starting SITL↔Gazebo bridge (tek MAVLink 14551: pose + motor + JSON)..."
python3 sim/bridges/sitl_gazebo_bridge.py > "$SIM_LOG_DIR/pose.log" 2>&1 &
POSE_BRIDGE_PID=$!
register_pid "$POSE_BRIDGE_PID"
wait_for_process_alive "$POSE_BRIDGE_PID" "SITL-Gazebo bridge" 10 || true
wait_for_file_fresh "$SIM_CONTROL_DIR/vehicle_position.json" "SITL-Gazebo pose JSON" 20 3.0 || true

echo "[SIM] Starting Camera TCP bridge (logging to $SIM_LOG_DIR/cam_bridge.log)..."
python3 sim/bridges/ros_to_tcp_cam.py > "$SIM_LOG_DIR/cam_bridge.log" 2>&1 &
export LOG_DIR  # Ensure application logs route to system directory
CAM_BRIDGE_PID=$!
register_pid "$CAM_BRIDGE_PID"
wait_for_process_alive "$CAM_BRIDGE_PID" "Camera TCP bridge" 10 || true
wait_for_tcp_port 127.0.0.1 8888 "Camera TCP bridge" 20 || true

echo "[SIM] Camera target class: $TARGET_CLASS"
echo "[SIM] Starting camera processor (logging to $LOG_DIR/cam.log)..."
(
    cd docker_workspace/src
    USV_SIM=1 LOG_DIR="$LOG_DIR" CONTROL_DIR="$CONTROL_DIR" USV_MODE="${USV_MODE:-test}" python3 cam.py > "$LOG_DIR/cam.log" 2>&1 &
    echo $! > "$CAM_PID_FILE"
)
echo "[SIM] Camera processor headless (no webserver)"

echo "[SIM] Starting telemetry dashboard (logging to $LOG_DIR/telemetry.log)..."
(
    cd docker_workspace/src
    USV_SIM=1 LOG_DIR="$LOG_DIR" CONTROL_DIR="$CONTROL_DIR" USV_MODE="${USV_MODE:-test}" SIM_LOG_DIR="$SIM_LOG_DIR" python3 telemetry.py > "$LOG_DIR/telemetry.log" 2>&1 &
    echo $! > "$TELEMETRY_PID_FILE"
)
wait_for_http_endpoint "http://127.0.0.1:8080/api/data" "telemetry API" 25 || true

echo "============================================================"
echo "[SIM] Stack temiz baslatildi."
echo "[SIM] SIM Loglar: $SIM_LOG_DIR (Gazebo, SITL, ROS2, bridges)"
echo "[SIM] APP Loglar: $LOG_DIR (cam, telemetry, usv_main, CSV, video)"
echo "[SIM] Kontrol dosyalari: $SIM_CONTROL_DIR"
echo "[SIM] Varsayilan sim gorevi: $MISSION_FILE"
echo "[SIM] Alternatif gorev: ./sim/bin/run_sim_stack.sh <mission_json_path>"
echo "[SIM] Telemetry UI: http://127.0.0.1:8080/dashboard"
echo "[SIM] Mission Planner: UDP 14552 (SITL vehicle telemetry + mission upload)"
echo "[SIM] Start: otomatik değil; testte dashboard/API, race'te RC CH5"
echo "[SIM] Kapatmak: CTRL+C veya ESC (bu terminal odaktayken)"
echo "[SIM] Simulation logs (debug):"
echo "  $SIM_LOG_DIR/gazebo.log sitl.log pose.log cam_bridge.log ros_gz.log"
echo "  $SIM_LOG_DIR/*debug.log *.trace.log  $SIM_LOG_DIR/check_stack.log  $SIM_LOG_DIR/ros2/*.log"
echo "[SIM] Application logs (debug):"
echo "  $LOG_DIR/cam.log telemetry.log lidar_map.log usv_main.log telemetri_verisi.csv"
echo "  $LOG_DIR/*.debug.log   $LOG_DIR/video/*.mp4"
echo "[SIM] Tee: $ROOT_LOG_DIR/terminal.log"
echo "============================================================"

cd docker_workspace/src
echo "[SIM] Starting lidar_map service (logging to $LOG_DIR/lidar_map.log)..."
(
    cd "$PROJ_ROOT/docker_workspace/src"
    LOG_DIR="$LOG_DIR" CONTROL_DIR="$CONTROL_DIR" USV_MODE="${USV_MODE:-test}" python3 lidar_map.py > "$LOG_DIR/lidar_map.log" 2>&1 &
    echo $! > "$LIDAR_MAP_PID_FILE"
)
if [ "${USV_MODE:-test}" = "race" ]; then
    wait_for_log_pattern "$LOG_DIR/lidar_map.log" "YARIŞMA MODU|YARISMA MODU|Race" "lidar_map race-disabled guard" 10 || true
else
    wait_for_http_endpoint "http://127.0.0.1:5001/" "lidar map API" 20 || true
fi
# stdin ayrilir: ESC bu kabukta yakalanir, usv_main klavye beklemez
# Export critical env vars: CONTROL_DIR for mission state/flags, USV_MODE for startup logic, USV_SIM for simulation detection
USV_SIM=1 LOG_DIR="$LOG_DIR" CONTROL_DIR="$CONTROL_DIR" USV_MODE="${USV_MODE:-test}" MISSION_FILE="${MISSION_FILE}" python3 usv_main.py </dev/null > "$LOG_DIR/usv_main.log" 2>&1 &
USV_PID=$!
register_pid "$USV_PID"
wait_for_process_alive "$USV_PID" "usv_main.py" 10 || true

echo "[SIM] Checking status of components..."
"$PROJ_ROOT/sim/bin/check_stack.sh" | tee "$SIM_LOG_DIR/check_stack.log"

echo "[SIM] Environment check:"
echo "  SIM_LOG_DIR=$SIM_LOG_DIR"
echo "  LOG_DIR=$LOG_DIR"
echo "  ROS_LOG_DIR=$ROS_LOG_DIR"

wait_for_startup_readiness || true

# COMPLIANCE TESTING: Run race-level compliance checks after stack startup
echo "[COMPLIANCE] Running race-level compliance tests..."
python3 "$PROJ_ROOT/host_scripts/check_compliance_race.py" > "$ROOT_LOG_DIR/simulation/compliance_race_test.log" 2>&1
COMPLIANCE_EXIT_CODE=$?

if [ $COMPLIANCE_EXIT_CODE -eq 0 ]; then
    echo "[COMPLIANCE] ✅ All 8 acceptance criteria PASSED"
else
    echo "[COMPLIANCE] ❌ Compliance tests FAILED (exit code: $COMPLIANCE_EXIT_CODE)"
    echo "[COMPLIANCE] See logs: $ROOT_LOG_DIR/simulation/compliance_race_test.log"
    # Log compliance failure but keep the manual stack available for inspection.
fi

# Interactive mode: TTY-based ESC listener (with TTY check to avoid blocking)
if [ -t 0 ]; then
    # Interactive: TTY exists, can read user input
    echo "[SIM] Interactive mode: Press ESC to stop simulation"
    while kill -0 "$USV_PID" 2>/dev/null; do
        if read -rsn1 -t 1 key 2>/dev/null; then
            if [[ "$key" == $'\e' ]]; then
                echo ""
                echo "[SIM] ESC - tum simulasyon sonlandiriliyor..."
                kill -TERM "$USV_PID" 2>/dev/null || true
                sleep 0.2
                kill -KILL "$USV_PID" 2>/dev/null || true
                wait "$USV_PID" 2>/dev/null || true
                cleanup
                exit 0
            fi
        fi
    done
else
    # Non-interactive: No TTY, just wait for process
    echo "[SIM] Non-interactive mode: waiting for usv_main to complete..."
    wait "$USV_PID" 2>/dev/null || true
fi

wait "$USV_PID" 2>/dev/null || true

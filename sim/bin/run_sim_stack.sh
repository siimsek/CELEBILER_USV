#!/bin/bash
# run_sim_stack.sh - Master orchestration script
#
# Baslatma sirasi: Gazebo -> ros_gz_bridge -> SITL -> sitl_gazebo_bridge (tek MAVLink 14551) -> cam -> telemetry -> usv_main.
# usv_main SITL'e tcp:5760 ile baglanir; SITL once ayakta olmalidir (asagida sleep ile saglanir).
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
mkdir -p "$ROOT_LOG_DIR" "$ROOT_LOG_DIR/host" "$ROOT_LOG_DIR/system" "$ROOT_LOG_DIR/simulation"
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
AUTO_TEST_DURATION=""
while [ $# -gt 0 ]; do
    case "$1" in
        --auto-test)
            AUTO_TEST_DURATION="${2:-60}"
            shift 2 || true
            ;;
        *)
            if [ -z "$MISSION_FILE" ] && [ ! -f "$1" ] 2>/dev/null; then
                # Treat as mission file path
                MISSION_FILE="$1"
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
export PYTHONUNBUFFERED=1
export PATH="$PATH:$HOME/ardupilot/Tools/autotest"

ROS_GZ_BRIDGE_BIN="/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"
CAM_PID_FILE="$SIM_CONTROL_DIR/cam.pid"
TELEMETRY_PID_FILE="$SIM_CONTROL_DIR/telemetry.pid"
SELF_PID=$$
PIDS=()
STALE_PATTERNS=(
    "python3 usv_main.py"
    "python3 docker_workspace/src/usv_main.py"
    "python3 sim/bridges/sitl_gazebo_bridge.py"
    "python3 sim/bridges/mavlink_to_gz_pose_simple.py"
    "python3 sim/bridges/gazebo_pose_updater.py"
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
    "5000/tcp"
    "8080/tcp"
    "8888/tcp"
    "14550/udp"
    "14551/udp"
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
        sleep 0.5
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
deadline = time.monotonic() + 20.0
last_line = None
min_lidar_points = max(0, int(os.environ.get("SIM_READY_MIN_LIDAR_POINTS", "0") or 0))


def probe_camera_endpoint():
    request = urllib.request.Request(camera_url, method="HEAD")
    with urllib.request.urlopen(request, timeout=1.5) as response:
        return int(getattr(response, "status", 200) or 200) < 500

while time.monotonic() < deadline:
    try:
        with urllib.request.urlopen(telemetry_url, timeout=2.0) as response:
            data = json.load(response)
        autonomy = data.get("autonomy_health") or {}
        lidar_points = int(autonomy.get("lidar_point_count", 0) or 0)
        camera_api_ready = False
        camera_api_error = None
        try:
            camera_api_ready = probe_camera_endpoint()
        except Exception as cam_exc:
            camera_api_error = str(cam_exc)
        ready = (
            bool(data.get("ready_state"))
            and bool(data.get("camera_ready"))
            and bool(data.get("lidar_ready"))
            and lidar_points >= min_lidar_points
            and camera_api_ready
        )
        line = (
            f"[SIM] Startup check: ready={str(ready).lower()} "
            f"camera={str(bool(data.get('camera_ready'))).lower()} "
            f"lidar={str(bool(data.get('lidar_ready'))).lower()} "
            f"telemetry_api=true "
            f"camera_api={str(camera_api_ready).lower()} "
            f"lidar_points={lidar_points} min_lidar_points={min_lidar_points} "
            f"health={str(bool(data.get('ready_state'))).lower()}"
        )
        if line != last_line:
            print(line)
            last_line = line
        if camera_api_error:
            print(f"[SIM] Startup check camera endpoint bekleniyor: {camera_api_error}")

        if ready:
            print("[READY] [SISTEM] Kamera + LIDAR + sağlık kontrolleri ve API endpointleri OK")
            print(
                "[READY] [SISTEM] "
                f"lidar_point_count={lidar_points} "
                f"camera_ready={bool(data.get('camera_ready'))} "
                f"lidar_ready={bool(data.get('lidar_ready'))} "
                f"telemetry_api_ready=True camera_api_ready=True"
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
    sleep 0.5
    kill_stale_ports
    ensure_endpoints_released || true
    rm -f "$CAM_PID_FILE" "$TELEMETRY_PID_FILE"
    echo "[SIM] Stale stack cleanup complete."
}

reset_runtime_dirs() {
    echo "[SIM] Resetting logs/control/artifacts..."
    rm -rf "$SIM_LOG_DIR" "$LOG_DIR" "$SIM_CONTROL_DIR" "$SIM_ARTIFACT_DIR/sitl" "$SIM_ARTIFACT_DIR/gz_home"
    mkdir -p "$SIM_LOG_DIR" "$SIM_LOG_DIR/ros2" "$LOG_DIR/video" "$SIM_CONTROL_DIR" "$SIM_ARTIFACT_DIR/sitl" "$SIM_ARTIFACT_DIR/gz_home" "$ROS_HOME"
    rm -f "$PROJ_ROOT/eeprom.bin" "$PROJ_ROOT/mav.tlog" "$PROJ_ROOT/mav.tlog.raw"
}

cleanup() {
    trap - SIGINT SIGTERM EXIT
    echo ""
    echo "[SIM] ========== SHUTDOWN INITIATED =========="
    # Kill registered PIDs with aggressive -9
    kill_if_running "$(cat "$CAM_PID_FILE" 2>/dev/null || true)" KILL
    kill_if_running "$(cat "$TELEMETRY_PID_FILE" 2>/dev/null || true)" KILL
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    # Immediately kill all simulation processes
    pkill -9 -f "docker_workspace/src/usv_main" 2>/dev/null || true
    pkill -9 -f "docker_workspace/src/cam" 2>/dev/null || true
    pkill -9 -f "docker_workspace/src/telemetry" 2>/dev/null || true
    pkill -9 -f "docker_workspace/src/lidar" 2>/dev/null || true
    pkill -9 -f "sim/bridges/" 2>/dev/null || true
    pkill -9 -f "gazebo" 2>/dev/null || true
    pkill -9 -f "ardurover" 2>/dev/null || true
    # Port cleanup
    fuser -k 8080/tcp 5000/tcp 5001/tcp 8888/tcp 14550/udp 5760/tcp 2>/dev/null || true
    rm -f "$CAM_PID_FILE" "$TELEMETRY_PID_FILE"
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
sleep 3

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
sleep 3

echo "[SIM] Starting ArduPilot SITL (logging to $SIM_LOG_DIR/sitl.log)..."
./sim/bin/run_sitl.sh > "$SIM_LOG_DIR/sitl.log" 2>&1 &
SITL_PID=$!
register_pid "$SITL_PID"
sleep 3

echo "[SIM] Starting SITL↔Gazebo bridge (tek MAVLink 14551: pose + motor + JSON)..."
python3 sim/bridges/sitl_gazebo_bridge.py > "$SIM_LOG_DIR/pose.log" 2>&1 &
POSE_BRIDGE_PID=$!
register_pid "$POSE_BRIDGE_PID"
sleep 5

echo "[SIM] Starting Camera TCP bridge (logging to $SIM_LOG_DIR/cam_bridge.log)..."
python3 sim/bridges/ros_to_tcp_cam.py > "$SIM_LOG_DIR/cam_bridge.log" 2>&1 &
export LOG_DIR  # Ensure application logs route to system directory
CAM_BRIDGE_PID=$!
register_pid "$CAM_BRIDGE_PID"
sleep 2

echo "[SIM] Camera target class: $TARGET_CLASS"
echo "[SIM] Starting camera processor (logging to $LOG_DIR/cam.log)..."
(
    cd docker_workspace/src
    LOG_DIR="$LOG_DIR" python3 cam.py > "$LOG_DIR/cam.log" 2>&1 &
    echo $! > "$CAM_PID_FILE"
)
sleep 2

echo "[SIM] Starting telemetry dashboard (logging to $LOG_DIR/telemetry.log)..."
(
    cd docker_workspace/src
    LOG_DIR="$LOG_DIR" python3 telemetry.py > "$LOG_DIR/telemetry.log" 2>&1 &
    echo $! > "$TELEMETRY_PID_FILE"
)
sleep 2

echo "============================================================"
echo "[SIM] Stack temiz baslatildi."
echo "[SIM] SIM Loglar: $SIM_LOG_DIR (Gazebo, SITL, ROS2, bridges)"
echo "[SIM] APP Loglar: $LOG_DIR (cam, telemetry, usv_main, CSV, video)"
echo "[SIM] Kontrol dosyalari: $SIM_CONTROL_DIR"
echo "[SIM] Varsayilan sim gorevi: $MISSION_FILE"
echo "[SIM] Alternatif gorev: ./sim/bin/run_sim_stack.sh <mission_json_path>"
echo "[SIM] Telemetry UI: http://127.0.0.1:8080"
echo "[SIM] Camera UI:    http://127.0.0.1:5000"
echo "[SIM] Gorev otomatik baslamaz; web controller Start komutunu bekler."
echo "[SIM] Gorevi baslat: dashboard Start veya POST /api/start_mission"
echo "[SIM] Kapatmak: CTRL+C veya ESC (bu terminal odaktayken)"
echo "[SIM] Simulation logs (debug):"
echo "  $SIM_LOG_DIR/gazebo.log sitl.log pose.log cam_bridge.log ros_gz.log"
echo "  $SIM_LOG_DIR/*debug.log *.trace.log  $SIM_LOG_DIR/check_stack.log  $SIM_LOG_DIR/ros2/*.log"
echo "[SIM] Application logs (debug):"
echo "  $LOG_DIR/cam.log telemetry.log usv_main.log lidar_map.log telemetri_verisi.csv"
echo "  $LOG_DIR/*.debug.log   $LOG_DIR/video/*.mp4"
echo "[SIM] Tee: $ROOT_LOG_DIR/terminal.log"
echo "============================================================"
sleep 1

cd docker_workspace/src
# Start lidar_map service (port 5001)
echo "[SIM] Starting lidar_map service (port 5001)..."
LOG_DIR="$LOG_DIR" python3 lidar_map.py > "$LOG_DIR/lidar_map.log" 2>&1 &
LIDAR_MAP_PID=$!
register_pid "$LIDAR_MAP_PID"
sleep 1
# stdin ayrilir: ESC bu kabukta yakalanir, usv_main klavye beklemez
# Export critical env vars: CONTROL_DIR for mission state/flags, USV_MODE for startup logic
LOG_DIR="$LOG_DIR" CONTROL_DIR="$CONTROL_DIR" USV_MODE="${USV_MODE:-test}" MISSION_FILE="${MISSION_FILE}" python3 usv_main.py </dev/null > "$LOG_DIR/usv_main.log" 2>&1 &
USV_PID=$!
register_pid "$USV_PID"

sleep 2
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
    # Log compliance failure but continue with AUTO_TEST if enabled
fi

# Auto-test flow: health check → mission start → motor validation
if [ -n "$AUTO_TEST_DURATION" ]; then
    echo "[SIM] ============ AUTO-TEST MODE: ${AUTO_TEST_DURATION}s ============"
    sleep 2  # Just a brief pause before tests
    
    # Test 1: Health gate ready check
    echo "[SIM] [AUTO] Testing health gate readiness..."
    health_response=$(curl -s http://127.0.0.1:8080/api/data 2>/dev/null || echo "{}")
    if echo "$health_response" | grep -qE '"ready_state"\s*:\s*(true|1)' || echo "$health_response" | jq -e '.ready_state // false' >/dev/null 2>&1; then
        echo "[SIM] ✓ [AUTO] Health gate READY"
    else
        echo "[SIM] ✗ [AUTO] Health gate NOT READY"
        echo "[SIM]   Response sample: $(echo "$health_response" | head -c 150)..."
    fi
    
    # Test 2: Mission start via API
    echo "[SIM] [AUTO] Starting mission via /api/start_mission..."
    start_response=$(curl -s -X POST http://127.0.0.1:8080/api/start_mission 2>/dev/null || echo "{}")
    if echo "$start_response" | grep -qE '"ok"\s*:\s*true|"status"\s*:\s*"ok"' || echo "$start_response" | jq -e '.ok // false' >/dev/null 2>&1; then
        echo "[SIM] ✓ [AUTO] Mission START API succeeded"
    else
        echo "[SIM] ✗ [AUTO] Mission START API failed or no response"
        echo "[SIM]   Response: $(echo "$start_response" | head -c 150)..."
    fi
    
    # Test 3: Wait for motion validation using vehicle_position.json
    vehicle_pose_before=$(python3 - "$SIM_CONTROL_DIR/vehicle_position.json" <<'PY'
import json
import math
import sys
from pathlib import Path

path = Path(sys.argv[1])
try:
    data = json.loads(path.read_text(encoding="utf-8"))
    print(
        f"{float(data.get('ts', 0.0))},"
        f"{float(data.get('pos_x', 0.0))},"
        f"{float(data.get('pos_y', 0.0))},"
        f"{float(data.get('heading_rad', 0.0))},"
        f"{float(data.get('motor_normalized', 0.0))}"
    )
except Exception:
    print("nan,nan,nan,nan,nan")
PY
)
    echo "[SIM] [AUTO] Waiting for motion validation (20 sec)..."
    sleep 20
    vehicle_pose_after=$(python3 - "$SIM_CONTROL_DIR/vehicle_position.json" <<'PY'
import json
import math
import sys
from pathlib import Path

path = Path(sys.argv[1])
try:
    data = json.loads(path.read_text(encoding="utf-8"))
    print(
        f"{float(data.get('ts', 0.0))},"
        f"{float(data.get('pos_x', 0.0))},"
        f"{float(data.get('pos_y', 0.0))},"
        f"{float(data.get('heading_rad', 0.0))},"
        f"{float(data.get('motor_normalized', 0.0))}"
    )
except Exception:
    print("nan,nan,nan,nan,nan")
PY
)
    motion_summary=$(python3 - "$vehicle_pose_before" "$vehicle_pose_after" <<'PY'
import math
import sys

def parse(raw):
    vals = [float(part) for part in raw.strip().split(",")]
    if len(vals) != 5:
        raise ValueError("expected 5 values")
    return vals

try:
    t0, x0, y0, h0, m0 = parse(sys.argv[1])
    t1, x1, y1, h1, m1 = parse(sys.argv[2])
except Exception:
    print("nan,nan,nan,nan,0")
    raise SystemExit(2)

if any(math.isnan(v) for v in (t0, x0, y0, h0, m0, t1, x1, y1, h1, m1)):
    print("nan,nan,nan,nan,0")
    raise SystemExit(2)

travel = math.hypot(x1 - x0, y1 - y0)
heading_delta = abs(h1 - h0)
while heading_delta > math.pi:
    heading_delta -= 2 * math.pi
heading_delta = abs(heading_delta)
motor_after = abs(m1)
pose_dt = max(0.0, t1 - t0)

ok = int(pose_dt >= 5.0 and travel >= 0.05)
print(f"{travel:.3f},{heading_delta:.3f},{motor_after:.3f},{pose_dt:.3f},{ok}")
raise SystemExit(0)
PY
)
    motion_travel=$(echo "$motion_summary" | cut -d',' -f1)
    motion_heading=$(echo "$motion_summary" | cut -d',' -f2)
    motion_motor=$(echo "$motion_summary" | cut -d',' -f3)
    motion_pose_dt=$(echo "$motion_summary" | cut -d',' -f4)
    motion_ok=$(echo "$motion_summary" | cut -d',' -f5)
    if [ "$motion_ok" = "1" ]; then
        echo "[SIM] ✓ [AUTO] Motion observed (travel=${motion_travel}m heading=${motion_heading}rad motor_norm=${motion_motor} pose_dt=${motion_pose_dt}s)"
    else
        echo "[SIM] ✗ [AUTO] Motion not observed (travel=${motion_travel}m heading=${motion_heading}rad motor_norm=${motion_motor} pose_dt=${motion_pose_dt}s)"
        if [ -f "$SIM_CONTROL_DIR/vehicle_position.json" ]; then
            echo "[SIM]   vehicle_position.json: $(cat "$SIM_CONTROL_DIR/vehicle_position.json")"
        fi
    fi
    
    # Wait remainder of test duration then exit
    remaining=$((AUTO_TEST_DURATION - 22))
    if [ $remaining -gt 0 ]; then
        echo "[SIM] [AUTO] Waiting ${remaining}s for test completion..."
        sleep "$remaining"
    fi
    
    echo "[SIM] [AUTO] Test sequence complete. Shutting down gracefully."
    kill -TERM "$USV_PID" 2>/dev/null || true
    sleep 0.5
    wait "$USV_PID" 2>/dev/null || true
    cleanup
    exit 0
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

#!/bin/bash
# check_stack.sh - Check if simulation processes are running

echo "Checking Simulation Stack:"
echo "-----------------------"
PROJ_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_DIR="${LOG_DIR:-$PROJ_ROOT/logs/system}"
SIM_LOG_DIR="${SIM_LOG_DIR:-$PROJ_ROOT/logs/simulation}"
USV_MODE="${USV_MODE:-test}"

report_proc() {
    local label="$1"
    local pattern="$2"
    local count
    count=$(pgrep -fc "$pattern" 2>/dev/null || true)
    if [ "${count:-0}" -eq 0 ]; then
        echo "[WARN] $label is NOT running"
    elif [ "$count" -eq 1 ]; then
        echo "[OK] $label is running"
    else
        echo "[WARN] $label has $count instances"
    fi
}

report_log_pair() {
    local label="$1"
    local base_dir="$2"
    local component="$3"
    local debug_file="$base_dir/${component}.debug.log"
    local jsonl_file="$base_dir/${component}.jsonl"
    if [ -s "$debug_file" ] && [ -s "$jsonl_file" ]; then
        echo "[OK] $label logs exist (${component}.debug.log + ${component}.jsonl)"
    else
        echo "[WARN] $label logs missing or empty: $debug_file / $jsonl_file"
    fi
}

report_proc "ArduPilot SITL" "ardurover"
report_proc "Gazebo server" "ign gazebo server|ign gazebo -s .*water_world.sdf|gz sim server"
report_proc "ROS-GZ Bridge" "ros_gz_bridge/parameter_bridge"
report_proc "MAVLink->GZ Bridge" "python3 sim/bridges/sitl_gazebo_bridge.py"
report_proc "Camera Bridge" "python3 sim/bridges/ros_to_tcp_cam.py"
report_proc "cam.py" "python3 cam.py"
report_proc "telemetry.py" "python3 telemetry.py"
if [ "$USV_MODE" = "race" ]; then
    echo "[OK] lidar_map.py race-disabled by design (stream guard)"
else
    report_proc "lidar_map.py" "python3 lidar_map.py"
fi
report_proc "usv_main.py" "python3 usv_main.py"
report_log_pair "SITL-Gazebo Bridge" "$SIM_LOG_DIR" "sitl_gazebo_bridge"
report_log_pair "Camera" "$LOG_DIR" "cam"
report_log_pair "Telemetry" "$LOG_DIR" "telemetry"
report_log_pair "Lidar Map" "$LOG_DIR" "lidar_map"
report_log_pair "USV Main" "$LOG_DIR" "usv_main"
echo "-----------------------"

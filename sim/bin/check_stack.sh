#!/bin/bash
# check_stack.sh - Check if simulation processes are running

echo "Checking Simulation Stack:"
echo "-----------------------"
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

report_proc "ArduPilot SITL" "ardurover"
report_proc "Gazebo server" "ign gazebo server|ign gazebo -s .*water_world.sdf|gz sim server"
report_proc "ROS-GZ Bridge" "ros_gz_bridge/parameter_bridge"
report_proc "MAVLink->GZ Bridge" "python3 sim/bridges/sitl_gazebo_bridge.py"
report_proc "Camera Bridge" "python3 sim/bridges/ros_to_tcp_cam.py"
report_proc "cam.py" "python3 cam.py"
report_proc "telemetry.py" "python3 telemetry.py"
report_proc "usv_main.py" "python3 usv_main.py"
echo "-----------------------"

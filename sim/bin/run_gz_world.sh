#!/bin/bash
# run_gz_world.sh - Launch Gazebo World

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SIM_MODEL_DIR="$SIM_PROJECT_ROOT/sim/models"
SIM_WORLD_FILE="$SIM_PROJECT_ROOT/sim/worlds/water_world.sdf"
SIM_GZ_HOME="${SIM_GZ_HOME:-$SIM_PROJECT_ROOT/sim/artifacts/gz_home}"
SIM_LOG_DIR="${SIM_LOG_DIR:-$SIM_PROJECT_ROOT/logs/simulation}"

mkdir -p "$SIM_LOG_DIR"
echo "[$(date -Iseconds)] run_gz_world pid=$$ SIM_GUI=${SIM_GUI:-} world=$SIM_WORLD_FILE" >> "$SIM_LOG_DIR/run_gz_world.trace.log"

mkdir -p "$SIM_GZ_HOME"
export HOME="$SIM_GZ_HOME"

export IGN_GAZEBO_RESOURCE_PATH="$SIM_MODEL_DIR${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="$SIM_MODEL_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="$SIM_MODEL_DIR${SDF_PATH:+:$SDF_PATH}"

gui_mode_raw="${SIM_GUI:-auto}"
gui_mode="$(printf '%s' "$gui_mode_raw" | tr '[:upper:]' '[:lower:]')"
launch_gui=0

case "$gui_mode" in
    1|true|yes|on|gui)
        launch_gui=1
        ;;
    0|false|no|off|headless|server)
        launch_gui=0
        ;;
    auto|"")
        if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
            launch_gui=1
        fi
        ;;
    *)
        echo "[SIM] Bilinmeyen SIM_GUI='$SIM_GUI', auto olarak devam ediliyor."
        if [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; then
            launch_gui=1
        fi
        ;;
esac

if [ "$launch_gui" -eq 1 ]; then
    echo "[SIM] Launching Gazebo Fortress World (GUI)..."
    exec ign gazebo -r "$SIM_WORLD_FILE"
fi

echo "[SIM] Launching Gazebo Fortress World (server-only)..."
exec ign gazebo -s -r "$SIM_WORLD_FILE"

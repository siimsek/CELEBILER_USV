#!/bin/bash
# run_sitl.sh - Launch ArduPilot SITL without MAVProxy
set -euo pipefail

PROJ_ROOT="${PROJ_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
source "$PROJ_ROOT/sim/configs/sim.env"

SITL_DIR="$SIM_ARTIFACT_DIR/sitl"
FRAME="${SIM_SITL_FRAME:-motorboat-skid}"
HOME_LOCATION="${SIM_HOME:--35.363262,149.165237,584,0}"
SPEEDUP="${SIM_SPEEDUP:-1}"
ARDUROVER_BIN="$HOME/ardupilot/build/sitl/bin/ardurover"
DEFAULTS_ROOT="$HOME/ardupilot/Tools/autotest/default_params"

case "$FRAME" in
    motorboat-skid)
        DEFAULTS="$DEFAULTS_ROOT/rover.parm,$DEFAULTS_ROOT/motorboat.parm,$DEFAULTS_ROOT/rover-skid.parm"
        ;;
    rover-skid)
        DEFAULTS="$DEFAULTS_ROOT/rover.parm,$DEFAULTS_ROOT/rover-skid.parm"
        ;;
    motorboat)
        DEFAULTS="$DEFAULTS_ROOT/rover.parm,$DEFAULTS_ROOT/motorboat.parm"
        ;;
    *)
        DEFAULTS="$DEFAULTS_ROOT/rover.parm"
        ;;
esac

mkdir -p "$SITL_DIR"
cd "$SITL_DIR"

echo "[$(date -Iseconds)] run_sitl pid=$$ frame=$FRAME home=$HOME_LOCATION sitl_dir=$SITL_DIR" >> "$SIM_LOG_DIR/run_sitl.trace.log"

echo "[SIM] Starting ArduPilot SITL frame=$FRAME"
echo "[SIM] SITL home: $HOME_LOCATION"
echo "[SIM] SITL dir: $SITL_DIR"
echo "[SIM] ARDUROVER_BIN: $ARDUROVER_BIN"
echo "[SIM] MAVLink control: tcp 5760 (usv_main)"
echo "[SIM] MAVLink out: udp 14550 (telemetry), udp 14551 (pose bridge)"
echo "[SIM] GPS and sensor simulation enabled via parameters"

# DEBUG: Print exact command being executed
echo "[SIM-DEBUG] Executing command:"
echo "[SIM-DEBUG] Binary: $ARDUROVER_BIN"
echo "[SIM-DEBUG] Frame: $FRAME"
echo "[SIM-DEBUG] Speedup: $SPEEDUP"
echo "[SIM-DEBUG] Defaults: $DEFAULTS"

# Use array to properly handle parameters with spaces/special chars
run_cmd=(
    "$ARDUROVER_BIN"
    "--model" "JSON"
    "--speedup" "$SPEEDUP"
    "--home" "$HOME_LOCATION"
    "--defaults" "$DEFAULTS"
    "--serial0" "tcp:5760"
    "--serial1" "udpclient:127.0.0.1:14550"
    "--serial2" "udpclient:127.0.0.1:14551"
    "--serial3" "udpclient:127.0.0.1:9002"
    # NOTE: ArduPilot SITL doesn't support --set from command line.
    # Parameter settings are applied via --defaults files instead.
    # Original unsupported params were: --set AHRS_EKF_TYPE=3, --set SIM_GPS_DELAY_MS=100, --set SIM_LIDAR_ENABLE=1
)

echo "[SIM-DEBUG] Full command array (${#run_cmd[@]} args):"
for ((i=0; i<${#run_cmd[@]}; i++)); do
    echo "[SIM-DEBUG]   [$i] = '${run_cmd[$i]}'"
done

exec "${run_cmd[@]}"

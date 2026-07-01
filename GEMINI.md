# 🚢 ÇELEBİLER USV - Project Context & Instructions

This document provides foundational context and instructions for the **ÇELEBİLER USV** project, a TEKNOFEST 2026 İDA (Unmanned Surface Vehicle) competition entry.

## 📁 Project Overview
The project implements an autonomous navigation and mission execution system for a USV. It features a dual-layer architecture: a **Host (Raspberry Pi 4)** running hardware-interfacing scripts and a **Docker Container (Ubuntu 22.04 + ROS 2 Humble)** hosting the core "brain," telemetry, and sensor processing modules.

Primary architecture references, in order:
1. `documents/ida_sartname.md`
2. `documents/rapor_calismasistemi.md`
3. `host_scripts/` startup/stop/compliance flows
4. `README.md`
5. `AGENTS.md` and this file

The current operating architecture for sim-first behavior, control ownership, mission lifecycle, P3 engagement, and sim-to-real calibration is kept in the two documents above, the main README, and executable host/sim scripts.

### Key Components:
- **`usv_main.py`**: The central "brain" and state machine. It handles autonomous logic, MAVLink communication with Pixhawk (ArduRover), and mission orchestration.
- **`compliance_profile.py`**: Centralized configuration hub. Contains all thresholds, innovation switches, and mode-specific (test/race) rules.
- **`telemetry.py`**: Flask-based Mission Control Dashboard and REST API (Port 8080).
- **`cam.py`**: Computer vision module for target detection and MJPEG streaming (Port 5000).
- **`lidar_map.py`**: Lidar processing service and map server (Port 5001).
- **`sim/`**: Comprehensive simulation stack using Gazebo Fortress and ArduPilot SITL.

---

## 🛠️ Technology Stack
- **OS**: Raspberry Pi OS (Host) / Ubuntu 22.04 (Docker)
- **Middleware**: ROS 2 Humble, MAVProxy, MAVLink (pymavlink)
- **Languages**: Python 3.10+, Bash
- **Web**: Flask (Dashboard & API)
- **Vision**: OpenCV
- **Simulation**: Gazebo Fortress, ArduPilot SITL (ArduRover motorboat-skid)

---

## 🚀 Key Commands

### Hardware Execution (Raspberry Pi)
- **Start System**: `./host_scripts/system_start.sh [test|race]`
- **Stop System**: `./host_scripts/system_stop.sh`
- **View Logs**: `./host_scripts/usv_logs.sh`

### Simulation
- **Run Full Stack**: `./sim/bin/run_sim_stack.sh`
- **Check Stack Health**: `./sim/bin/check_stack.sh`
- **Run Gazebo Only**: `./sim/bin/run_gz_world.sh`
- **Run SITL Only**: `./sim/bin/run_sitl.sh`

---

## 📜 Development Conventions

### 1. Configuration & Thresholds
- **NEVER** hardcode physical thresholds or logic switches in `usv_main.py`.
- **ALWAYS** use `compliance_profile.py`. If a new parameter is needed, add it there.
- Use `INNOVATION_SWITCHES` in `compliance_profile.py` to toggle advanced features (Sensor Fusion, Horizon Lock, etc.).

### 2. Architecture & Modularity
- Keep `usv_main.py` focused on orchestration; move heavy math to `nav_guidance.py`.
- Use `json_atomic.py` for file-based IPC to ensure data integrity.

### 3. Operating Modes (`USV_MODE`)
- **`test`**: Full visibility. MJPEG streams (5000) and Lidar maps (5001) are enabled.
- **`race`**: Competition mode. RC CH5-only start, no image/map streaming, Pixhawk mission sync required before start.

### 4. Logging
- Logs are stored in the `logs/` directory at the project root.
- `logs/system/`: Core module logs (`usv_main.log`, `telemetry.log`).
- `logs/simulation/`: Gazebo and SITL logs.
- `logs/host/`: Startup and hardware logs.
- Use `runtime_debug_log.py` for rotating debug logs.

### 5. Mission Data
- Test missions may be defined in `docker_workspace/mission.json` as a flat list of `[lat, lon]` coordinates.
- Race mission source is Mission Planner -> Pixhawk. `usv_main.py` mirrors the Pixhawk mission before start with `MISSION_REQUEST_INT` / `MISSION_ITEM_INT`.
- Start is rejected if `mission_synced=false`; race also rejects non-`pixhawk_mission` sources.

### 6. Navigation Contract
- Race P1 is pure Pixhawk `AUTO`.
- Race P2/P3 and test NAV/ENGAGE use Raspberry Pi guidance with ArduRover `GUIDED` velocity/yaw setpoints.
- Primary setpoint message is `SET_POSITION_TARGET_GLOBAL_INT` with `MAV_FRAME_GLOBAL_INT` and type mask `0x9C7`.
- Race PWM ownership stays in Pixhawk/ArduPilot. Raspberry Pi produces `v_target` and `heading/yaw` targets only.
- ArduPilot owns heading/yaw closed-loop control, speed tracking, mixer behavior, and ESC PWM output.
- `RC_CHANNELS_OVERRIDE` is not the normal autonomy path. It is limited to manual/safety/bench contexts; race autonomy must not use it.
- If GUIDED setpoint delivery fails, Raspberry Pi must enter `HOLD`/failsafe semantics instead of falling back to RC override.

### 7. Control Ownership
- P1: Pixhawk `AUTO` mission execution; Pi monitors health, mission progress, logs, and failsafe.
- P2: Pi performs sensor fusion, waypoint/gate/avoidance decisions, and sends GUIDED setpoints; Pixhawk produces PWM.
- P3: Pi performs locked target color tracking and wrong-target avoidance, then sends GUIDED setpoints; Pixhawk produces PWM.
- E-stop: physical power-cut chain has priority over all autonomy and PWM logic.
- Race rejects `USV_USE_RC_OVERRIDE=1`.
- Race-like simulation rejects `SIM_GZ_ALLOW_MOTOR_COMMAND_JSON=1`; `sitl_gazebo_bridge.py` should use ArduPilot SITL servo/PWM output as the physics source.

### 8. P3 Engagement Contract
- This boat has no contact sensor.
- P3 completion cannot rely on a physical contact sensor event.
- Completion is inferred from indirect evidence: target proximity, target image area, lidar proximity, and sudden slowdown/stall consistency.
- The chosen evidence source must be visible in `mission_state.json` as `p3_contact_confirmation_source`.
- Wrong target detections must be carried through `camera_status.json` as `wrong_target_*` fields.
- If wrong-target contact risk is active, engagement completion must not latch.

### 9. Realistic Simulation Contract
- Real hull reference: approximately `160 cm` length, `81 cm` width, `35 cm` above-water height.
- Gazebo hull target: `1.60-1.62 m x 0.81 m x 0.35 m`.
- Sim motion must be driven by ArduPilot SITL servo/PWM through `sitl_gazebo_bridge.py`, not direct pose shortcuts.
- Calibrate mass, inertia, damping, motor reversal, yaw sign, max speed, and max yaw rate against real water tests.
- Motion/mission progression tests remain manual operator actions; agents may run only static/validator checks unless explicitly allowed by the project rules.

### 10. Pixhawk Parameter Baseline
- `config/pixhawk_ida.param` is the current Pixhawk baseline that already works for manual driving on the real boat.
- Do not rewrite this baseline casually. Treat `SERVO1/3` functions, `1100/1500/1900` PWM ranges, RC5 start input, RC7 E-stop input, and RC8 mode input as the hardware reference until water tests prove a required change.
- The current baseline has `ARMING_CHECK=0`, `ARMING_REQUIRE=0`, and several Pixhawk-native failsafes disabled. That is acceptable only as a manual-test baseline; race deployment requires either a certified physical E-stop/contactor procedure or an explicit Pixhawk pre-arm/failsafe profile.
- `CRUISE_SPEED=2` and `WP_SPEED=2` can make P1 AUTO faster than the documented conservative mission profile. Confirm mission speed commands or a race param profile before autonomous water tests.

---

## 🔑 Critical Files
- `docker_workspace/src/usv_main.py`: Entry point for autonomous behavior.
- `docker_workspace/src/compliance_profile.py`: The single source of truth for settings.
- `host_scripts/system_start.sh`: The main entry point for the physical hardware.
- `sim/bin/run_sim_stack.sh`: The main entry point for the simulation environment.
- `docker_workspace/mission.json`: The mission configuration.
- `config/pixhawk_ida.param`: Current real Pixhawk manual-driving baseline.

---

## 🛡️ Safety & Compliance
- **Remote E-Stop**: Managed via RC (CH7) and STM32 power cut.
- **Fail-safe**: Dual-layer heartbeat (Link & Onboard) defined in `compliance_profile.py`.
- **Race Start**: In `race` mode, mission can only be started via RC switch (CH5 >= 1700).
- **Automation Limit**: `./sim/bin/run_sim_stack.sh --auto-test` is intentionally rejected. Motion and mission progression tests are manual operator actions.
- **Required Validators After Changes**:
  - `python3 -m py_compile docker_workspace/src/*.py`
  - `python3 host_scripts/check_compliance_static.py`
  - `python3 host_scripts/check_compliance_behavior.py`
  - `python3 host_scripts/check_compliance_race.py`

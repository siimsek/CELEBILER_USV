# 🚢 ÇELEBİLER USV - Project Context & Instructions

This document provides foundational context and instructions for the **ÇELEBİLER USV** project, a TEKNOFEST 2026 İDA (Unmanned Surface Vehicle) competition entry.

## 📁 Project Overview
The project implements an autonomous navigation and mission execution system for a USV. It features a dual-layer architecture: a **Host (Raspberry Pi 4)** running hardware-interfacing scripts and a **Docker Container (Ubuntu 22.04 + ROS 2 Humble)** hosting the core "brain," telemetry, and sensor processing modules.

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
- Follow the guidelines in `documents/module_boundaries.md`.
- Keep `usv_main.py` focused on orchestration; move heavy math to `nav_guidance.py`.
- Use `json_atomic.py` for file-based IPC to ensure data integrity.

### 3. Operating Modes (`USV_MODE`)
- **`test`**: Full visibility. MJPEG streams (5000) and Lidar maps (5001) are enabled.
- **`race`**: Competition mode. Restricted web access, specific safety triggers (RC CH5 start), and hardened logging.

### 4. Logging
- Logs are stored in the `logs/` directory at the project root.
- `logs/system/`: Core module logs (`usv_main.log`, `telemetry.log`).
- `logs/simulation/`: Gazebo and SITL logs.
- `logs/host/`: Startup and hardware logs.
- Use `runtime_debug_log.py` for rotating debug logs.

### 5. Mission Data
- Mission waypoints are defined in `docker_workspace/mission.json` as a flat list of `[lat, lon]` coordinates.

---

## 🔑 Critical Files
- `docker_workspace/src/usv_main.py`: Entry point for autonomous behavior.
- `docker_workspace/src/compliance_profile.py`: The single source of truth for settings.
- `host_scripts/system_start.sh`: The main entry point for the physical hardware.
- `sim/bin/run_sim_stack.sh`: The main entry point for the simulation environment.
- `docker_workspace/mission.json`: The mission configuration.

---

## 🛡️ Safety & Compliance
- **Remote E-Stop**: Managed via RC (CH7) and STM32 power cut.
- **Fail-safe**: Dual-layer heartbeat (Link & Onboard) defined in `compliance_profile.py`.
- **Race Start**: In `race` mode, mission can only be started via RC switch (CH5 >= 1700).

# 🚢 ÇELEBİLER USV - Autonomous Surface Vehicle

<div align="center">

![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-red.svg)
![ROS](https://img.shields.io/badge/ROS-2%20Humble-green.svg)
![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)

**TEKNOFEST 2026 İDA Competition Entry**

*An autonomous unmanned surface vehicle for navigation, obstacle avoidance, and mission execution.*

</div>

---

## 📖 Overview

This project implements a complete software stack for an Autonomous Surface Vehicle (USV) designed for the TEKNOFEST İDA (Unmanned Submarine/Surface Vehicle) competition. The system runs on a Raspberry Pi 4a inside Docker containers, integrating multiple sensors for autonomous navigation.

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      RASPBERRY PI 4 (HOST)                       │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────────┐ │
│  │ RPi Camera  │  │ RPLidar S2E  │  │  Pixhawk (MAVLink)      │ │
│  │ TCP :8888   │  │ UDP :2000    │  │  /dev/ttyACM*           │ │
│  └──────┬──────┘  └──────┬───────┘  └───────────┬─────────────┘ │
│         │                │                      │               │
├─────────┴────────────────┴──────────────────────┴───────────────┤
│                     DOCKER CONTAINER (ege_ros)                   │
│                     Ubuntu 22.04 + ROS 2 Humble                  │
├─────────────────────────────────────────────────────────────────┤
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐ │
│  │  cam.py    │  │lidar_map.py│  │telemetry.py│  │fusion_main │ │
│  │  Port:5000 │  │  Port:5001 │  │  Port:8080 │  │  (Brain)   │ │
│  │ 720p MJPEG │  │  SLAM Map  │  │ Dashboard  │  │ ROS2+MAV   │ │
│  └────────────┘  └────────────┘  └────────────┘  └────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## 🛠️ Technology Stack

| Category | Technology |
|----------|------------|
| **Hardware** | Raspberry Pi 4, RPLidar S2E, Pixhawk, Pi Camera |
| **OS** | Raspberry Pi OS (Host) / Ubuntu 22.04 (Docker) |
| **Middleware** | ROS 2 Humble Hawksbill |
| **Languages** | Python 3.10+, Bash |
| **Web Framework** | Flask (MJPEG streaming) |
| **Computer Vision** | OpenCV (Color detection, HUD overlay) |
| **Flight Control** | MAVLink via pymavlink |
| **Data Processing** | NumPy (Vectorized Lidar processing) |

## 📁 Project Structure

```
CELEBILER_USV/
├── host_scripts/
│   └── system_start.sh      # 🔑 Main startup script
├── docker_workspace/
│   ├── src/
│   │   ├── cam.py           # Camera processing & web stream (5000)
│   │   ├── lidar_map.py     # Lidar SLAM visualization (5001)
│   │   ├── telemetry.py     # Dashboard & sensor fusion (8080)
│   │   ├── fusion_main.py   # Autonomous navigation brain
│   │   ├── gps_mission.py   # GPS waypoint navigation
│   │   ├── autonomous_main.py  # Heading-hold autopilot
│   │   └── rc_test.py       # RC override testing
│   ├── scripts/
│   │   └── internal_start.sh  # Docker internal startup
│   └── logs/                # Telemetry CSV & video logs
├── config/                  # Configuration files
├── old/                     # Backup of legacy code
├── SYSTEM_MANIFEST.md       # Hardware/Software specs
└── DEVELOPMENT_RULES.md     # Coding standards
```

## 🚀 Quick Start

### 1. Clone the Repository
```bash
git clone https://github.com/celebiler/CELEBILER_USV.git
cd CELEBILER_USV
```

### 2. Start the System (on Raspberry Pi)
```bash
./host_scripts/system_start.sh
```

### 3. Access the Dashboard
Open a browser and navigate to:
- **Dashboard:** `http://<RPi_IP>:8080`
- **Camera Feed:** `http://<RPi_IP>:5000`
- **Lidar Map:** `http://<RPi_IP>:5001`

## 🌐 Web Interfaces

| Port | Service | Description |
|------|---------|-------------|
| `8080` | Telemetry Dashboard | Real-time GPS, Battery, Mode, Environment |
| `5000` | Camera Stream | 720p MJPEG with color detection overlay |
| `5001` | Lidar Map | Live 2D SLAM visualization |

## 🔧 Key Features

- **Zero-Latency Camera:** Direct socket MJPEG with automatic lag prevention
- **Vectorized Lidar:** NumPy-optimized point cloud processing
- **Simulation Mode:** Auto-fallback when hardware is disconnected
- **System Monitoring:** Live CPU, RAM, Temperature in dashboard
- **STM32 Integration:** Environmental sensors (Temp, Humidity, Rain)
- **Robust Startup:** Port cleanup, auto-reconnect, crash recovery

## 📜 License

This project is licensed under the MIT License.

## 👥 Team

**AKDENİZ ÜNİVERSİTESİ İDA TAKIMI**

---

<div align="center">
Made with ❤️ for TEKNOFEST 2026
</div>

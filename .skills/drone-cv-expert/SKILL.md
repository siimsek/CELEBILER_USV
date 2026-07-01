---
name: drone-cv-expert
description: Expert in drone systems, computer vision, and autonomous navigation. Specializes in flight control, SLAM, object detection, sensor fusion, and path planning.
source: erichowens/some_claude_skills@drone-cv-expert (160 installs)
---

# Drone CV Expert

Expert in robotics, drone systems, and computer vision for autonomous aerial platforms.

## Core Competencies

### Flight Control & Navigation
- **PID Tuning**: Position, velocity, attitude control loops
- **SLAM**: ORB-SLAM, LSD-SLAM, visual-inertial odometry (VIO)
- **Path Planning**: A*, RRT, RRT*, Dijkstra, potential fields
- **Sensor Fusion**: EKF, UKF, complementary filters
- **GPS-Denied Navigation**: AprilTags, visual odometry, LiDAR SLAM

### Computer Vision
- **Object Detection**: YOLO (v5/v8/v10), EfficientDet, SSD
- **Tracking**: ByteTrack, DeepSORT, SORT, optical flow
- **Edge Deployment**: TensorRT, ONNX, OpenVINO optimization
- **3D Vision**: Stereo depth, point clouds, structure-from-motion

### Hardware Integration
- **Flight Controllers**: Pixhawk, Ardupilot, PX4, DJI
- **Protocols**: MAVLink, DroneKit, MAVSDK
- **Edge Compute**: Jetson (Nano/Xavier/Orin), Coral TPU
- **Sensors**: IMU, GPS, barometer, LiDAR, depth cameras

## Anti-Patterns to Avoid

### 1. "Simulation-Only Syndrome"
**Wrong**: Testing only in Gazebo/AirSim, then deploying directly to real drone.
**Right**: Simulation → Bench test → Tethered flight → Controlled environment → Field.

### 2. "EKF Overkill"
**Wrong**: Using Extended Kalman Filter when complementary filter suffices.
**Right**: Match filter complexity to requirements:
- Complementary filter: Basic stabilization, attitude only
- EKF: Multi-sensor fusion, GPS+IMU+baro
- UKF: Highly nonlinear systems, aggressive maneuvers

### 3. "Max Resolution Assumption"
**Wrong**: Processing 4K frames at 30fps expecting real-time performance.
**Right**: Resolution trade-offs by altitude/speed:
| Altitude | Speed | Resolution | FPS | Rationale |
|----------|-------|------------|-----|-----------|
| <30m | Slow | 1920x1080 | 30 | Detail needed |
| 30-100m | Medium | 1280x720 | 30 | Balance |
| >100m | Fast | 640x480 | 60 | Speed priority |

### 4. "Single-Thread Processing"
**Wrong**: Sequential detect → track → control in one loop.
**Right**: Parallel pipelines with thread/process pools.

## Problem-Solving Framework

### 1. Constraint Analysis
- **Compute**: What hardware? (Jetson Nano = ~5 TOPS, Xavier = 32 TOPS)
- **Power**: Battery capacity? Flight time impact?
- **Latency**: Control loop rate? Detection response time?
- **Weight**: Payload capacity? Center of gravity?
- **Environment**: Indoor/outdoor? GPS available? Lighting conditions?

### 2. Algorithm Selection Matrix

| Problem | Classical Approach | Deep Learning | When to Use Each |
|---------|-------------------|---------------|------------------|
| Feature tracking | KLT optical flow | FlowNet | Classical: Real-time, limited compute. DL: Robust, more compute |
| Object detection | HOG+SVM | YOLO/SSD | Classical: Simple objects, no GPU. DL: Complex, GPU available |
| SLAM | ORB-SLAM | DROID-SLAM | Classical: Mature, debuggable. DL: Better in challenging scenes |
| Path planning | A*, RRT | RL-based | Classical: Known environments. DL: Complex, dynamic |

### 3. Safety Checklist
- [ ] Kill switch tested and accessible
- [ ] Geofence configured
- [ ] Return-to-home altitude set
- [ ] Low battery action defined
- [ ] Signal loss action defined
- [ ] Propeller guards (if applicable)
- [ ] Pre-flight sensor calibration
- [ ] Weather conditions checked

## Quick Reference Tables

### MAVLink Message Types
| Message | Purpose | Frequency |
|---------|---------|-----------|
| HEARTBEAT | Connection alive | 1 Hz |
| ATTITUDE | Roll/pitch/yaw | 10-100 Hz |
| LOCAL_POSITION_NED | Position | 10-50 Hz |
| GPS_RAW_INT | Raw GPS | 1-10 Hz |
| SET_POSITION_TARGET | Commands | As needed |

### Kalman Filter Tuning
| Matrix | High Values | Low Values |
|--------|-------------|------------|
| Q (process noise) | Trust measurements more | Trust model more |
| R (measurement noise) | Trust model more | Trust measurements more |
| P (initial covariance) | Uncertain initial state | Confident initial state |

### Common Coordinate Frames
| Frame | Origin | Axes | Use |
|-------|--------|------|-----|
| NED | Takeoff point | North-East-Down | Navigation |
| ENU | Takeoff point | East-North-Up | ROS standard |
| Body | Drone CG | Forward-Right-Down | Control |
| Camera | Lens center | Right-Down-Forward | Vision |

## Simulation Tools

| Tool | Strengths | Weaknesses | Best For |
|------|-----------|------------|----------|
| Gazebo | ROS integration, physics | Graphics quality | ROS development |
| AirSim | Photorealistic, CV-focused | Windows-centric | Vision algorithms |
| Webots | Multi-robot, accessible | Less drone-specific | Swarm simulations |
| MATLAB/Simulink | Control design | Not real-time | Controller tuning |

**Key Principle**: In drone systems, reliability trumps performance. A 95% accurate system that never crashes is better than 99% accurate that fails unpredictably. Always have fallbacks.

# CELEBILER USV — FULL SYSTEM AUDIT REPORT

## S0 — READING SUMMARY

Mandatory files:

| File | Lines | Status |
|---|---:|---|
| OTOMASYON.md | 379 | OK |
| AGENTS.md | 206 | OK |
| GEMINI.md | 139 | OK |
| documents/ida_sartname.md | 390 | OK |
| documents/rapor_calismasistemi.md | 185 | OK |
| docker_workspace/src/usv_main.py | 9199 | OK |
| docker_workspace/src/compliance_profile.py | 429 | OK |
| docker_workspace/src/nav_guidance.py | 344 | OK |
| docker_workspace/src/cam.py | 1227 | OK |
| docker_workspace/src/lidar_map.py | 311 | OK |
| docker_workspace/src/telemetry.py | 3777 | OK |
| docker_workspace/src/json_atomic.py | 0 | MISSING |
| docker_workspace/src/runtime_debug_log.py | 526 | OK |
| docker_workspace/mission.json | 8 | OK |
| sim/bridges/sitl_gazebo_bridge.py | 615 | OK |
| sim/bridges/ros_to_tcp_cam.py | 201 | OK |
| sim/bin/run_sim_stack.sh | 575 | OK |
| sim/bin/check_stack.sh | 28 | OK |
| host_scripts/system_start.sh | 286 | OK |
| host_scripts/system_stop.sh | 26 | OK |
| host_scripts/check_compliance_static.py | 193 | OK |
| host_scripts/check_compliance_behavior.py | 104 | OK |
| host_scripts/check_compliance_race.py | 355 | OK |
| config/pixhawk_ida.param | 920 | OK |
| logs/terminal.log tail -300 | 68 | OK |
| logs/system/usv_main.debug.log tail -200 | 208 | OK |
| logs/system/cam.jsonl tail -100 | 12 | OK |
| logs/system/lidar_map.jsonl tail -100 | 0 | MISSING |

Unlisted `docker_workspace/src/**/*.py` also read: `adapters/__init__.py` 1 OK, `adapters/real_adapter.py` 35 OK, `adapters/sim_adapter.py` 55 OK, `console_utils.py` 86 OK, `mission_adapter.py` 195 OK, `mission_config.py` 200 OK, `motor_controller.py` 276 OK, `navigation.py` 133 OK, `obstacle_avoidance.py` 85 OK, `sensors.py` 73 OK, `sim_nav_state.py` 166 OK, `spatial_frame.py` 333 OK.

Unlisted `sim/**/*.py|*.sh` also read: `sim/bin/run_gz_demo.sh` 5 OK, `sim/bin/run_gz_world.sh` 51 OK, `sim/bin/run_sitl.sh` 76 OK, `sim/bin/verify_world_mission.py` 81 OK.

## S1 — HEADING CONTROL CHAIN (Priority-1 investigation)

Pipeline: GPS/GNSS -> Pixhawk EKF -> MAVLink -> `usv_main.py` -> `nav_guidance.py` -> `SET_POSITION_TARGET_GLOBAL_INT` -> ArduPilot GUIDED -> SERVO PWM -> `sitl_gazebo_bridge.py` -> Gazebo.

1. HEADING SOURCE

`usv_main.py` accepts heading primarily from `GLOBAL_POSITION_INT.hdg`, dividing centidegrees by 100 to degrees and filtering into `current_heading` (`docker_workspace/src/usv_main.py:5094`, `docker_workspace/src/usv_main.py:5115`, `docker_workspace/src/usv_main.py:5125`). In hardware mode it can also use `ATTITUDE.yaw`, converting radians to degrees (`docker_workspace/src/usv_main.py:5154`, `docker_workspace/src/usv_main.py:5161`, `docker_workspace/src/usv_main.py:5174`). `ATTITUDE.yawspeed` is converted from rad/s to deg/s (`docker_workspace/src/usv_main.py:5154`, `docker_workspace/src/usv_main.py:5159`). `VFR_HUD` is used for groundspeed, not heading (`docker_workspace/src/usv_main.py:5241`, `docker_workspace/src/usv_main.py:5245`). In simulation, `vehicle_position.json.heading_rad` is authoritative (`docker_workspace/src/usv_main.py:8915`, `docker_workspace/src/usv_main.py:8928`). Units in `usv_main.py` are compass degrees; `ATTITUDE.yaw` input is radians.

2. HEADING SETPOINT

Waypoint bearing is produced in `_distance_and_heading_error()` by `calculate_bearing()` and `normalize_heading_error(target_bearing - heading)` (`docker_workspace/src/usv_main.py:5651`, `docker_workspace/src/usv_main.py:5667`). P2/P1 navigation decisions are produced in `_navigate_p2_waypoint()` through `compute_nav_decision()` (`docker_workspace/src/usv_main.py:8266`, `docker_workspace/src/usv_main.py:8279`; `docker_workspace/src/nav_guidance.py:85`, `docker_workspace/src/nav_guidance.py:175`). Final heading target is set in `_command_speed_heading()` as `(current_heading + shaped_heading_err) % 360` (`docker_workspace/src/usv_main.py:4887`, `docker_workspace/src/usv_main.py:4893`), then persisted in `_send_guided_speed_heading()` (`docker_workspace/src/usv_main.py:4614`, `docker_workspace/src/usv_main.py:4619`). Unit: degrees. Frame: compass/nav heading, 0 north, positive clockwise as used by ArduPilot rover guidance.

3. ERROR CALCULATION

The main waypoint error is:

```python
target_bearing = calculate_bearing(self.current_lat, self.current_lon, wp[0], wp[1])
heading_error = normalize_heading_error(target_bearing - heading)
```

Source: `docker_workspace/src/usv_main.py:5658`, `docker_workspace/src/usv_main.py:5667`.

`normalize_heading_error()` wraps by adding/subtracting 360 once (`docker_workspace/src/usv_main.py:381`, `docker_workspace/src/usv_main.py:386`). This is not the requested robust `atan2(sin(err_rad), cos(err_rad))` formula, but for normal heading differences in `[-360, 360]` it handles wrap-around. Flag: [BUG-HEADING-WRAP-ROBUSTNESS], low risk unless upstream produces larger magnitude errors.

4. SETPOINT MESSAGE

`SET_POSITION_TARGET_GLOBAL_INT` is sent in `_send_guided_speed_heading()` (`docker_workspace/src/usv_main.py:4614`). The exact call sets `MAV_FRAME_GLOBAL_INT`, type mask `GUIDED_GLOBAL_VEL_YAW_TYPE_MASK = 0x9C7`, current lat/lon, velocity `vx/vy`, `yaw_rad = math.radians(heading_deg)`, and `yaw_rate = 0.0` (`docker_workspace/src/usv_main.py:285`, `docker_workspace/src/usv_main.py:4655`, `docker_workspace/src/usv_main.py:4658`, `docker_workspace/src/usv_main.py:4675`). The mask matches `GEMINI.md` expected `0x9C7` (`GEMINI.md:83`).

5. SIGN CONVENTION

The code treats heading degrees as compass heading and passes `math.radians(heading_deg)` as yaw (`docker_workspace/src/usv_main.py:4655`). The sim bridge default is `SIM_GZ_YAW_SIGN=-1.0` (`sim/bridges/sitl_gazebo_bridge.py:34`, `sim/bridges/sitl_gazebo_bridge.py:37`) and applies `yaw_diff = SIM_GZ_YAW_SIGN * yaw_cmd_norm` (`sim/bridges/sitl_gazebo_bridge.py:438`, `sim/bridges/sitl_gazebo_bridge.py:439`). This is explicitly listed as an open risk in automation docs: sim-real yaw sign/motor matching is not closed (`OTOMASYON.md:373`, `OTOMASYON.md:379`). Static audit cannot certify sign agreement without a commanded-turn sim or water test.

6. GUIDED MODE ENTRY

`_set_mode()` sends `set_mode_send()` and waits for mode confirmation for up to 2 seconds (`docker_workspace/src/usv_main.py:4174`, `docker_workspace/src/usv_main.py:4307`). `_set_mode_sim_retry()` retries (`docker_workspace/src/usv_main.py:4308`, `docker_workspace/src/usv_main.py:4329`). Start flow requires GUIDED or AUTO-related FC readiness before mission active (`docker_workspace/src/usv_main.py:6712`, `docker_workspace/src/usv_main.py:6739`). P2 after race P1 explicitly switches to GUIDED before Pi navigation (`docker_workspace/src/usv_main.py:8831`, `docker_workspace/src/usv_main.py:8837`). Result: mode confirmation is present before first setpoint in the normal path.

7. SETPOINT FREQUENCY

`GUIDED_COMMAND_HZ` defaults to 10 Hz and is clamped to 5-10 Hz (`docker_workspace/src/usv_main.py:249`, `docker_workspace/src/usv_main.py:250`). `CONTROL_HZ` is 10 Hz real and 40 Hz sim, but `CONTROL_LOOP_HZ` is clamped to 5-10 Hz (`docker_workspace/src/compliance_profile.py:56`, `docker_workspace/src/compliance_profile.py:62`, `docker_workspace/src/usv_main.py:235`). This satisfies ArduPilot's >=2 Hz GUIDED freshness requirement. Failed GUIDED sends enter HOLD, with RC override fallback disabled (`docker_workspace/src/usv_main.py:4932`, `docker_workspace/src/usv_main.py:4944`).

8. P2 HEADING GATE LOGIC

Current P2 path does not use camera gate bearing for steering. `_navigate_p2_waypoint()` passes `gate_detected=False` and `gate_bearing_deg=0.0` into `compute_nav_decision()` (`docker_workspace/src/usv_main.py:8266`, `docker_workspace/src/usv_main.py:8279`). `compute_nav_decision()` states gate inputs are retained for compatibility but no longer steer (`docker_workspace/src/nav_guidance.py:104`, `docker_workspace/src/nav_guidance.py:105`). The older `compute_p2_decision()` implements lidar priority over gate bearing and waypoint (`docker_workspace/src/nav_guidance.py:233`, `docker_workspace/src/nav_guidance.py:319`), but it is not the active call path. Finding: [BUG-P2-GATE-BEARING-DISABLED].

9. P3 HEADING TRACKING

P3 reads `target_bearing_error_deg` from camera status (`docker_workspace/src/usv_main.py:8492`, `docker_workspace/src/usv_main.py:8498`). If target is detected and no wrong-target risk exists, heading error is directly `target_bearing` and speed is reduced as target area grows (`docker_workspace/src/usv_main.py:8536`, `docker_workspace/src/usv_main.py:8544`). There is no P3-local proportional gain other than 1:1 bearing error and no derivative term. Common yaw-rate damping and slew limiting are applied later in `_apply_heading_damping()` (`docker_workspace/src/usv_main.py:7785`, `docker_workspace/src/usv_main.py:7825`) and `_command_speed_heading()` (`docker_workspace/src/usv_main.py:4887`, `docker_workspace/src/usv_main.py:4905`).

10. OVERSHOOT / OSCILLATION

Available logs contain no completed mission movement trace with `heading_error`, `yaw_cmd`, and `observed_yaw` together. The latest `usv_main.debug.log` shows arming/start and SITL connection failures, not yaw oscillation. Max absolute heading error from available runtime logs: UNKNOWN - no valid movement sample.

## S2 — STATE MACHINE (usv_main.py)

States are declared as IDLE=0, NAV=1, ENGAGE=2, COMPLETED=4, HOLD=5 (`docker_workspace/src/usv_main.py:428`, `docker_workspace/src/usv_main.py:433`).

Transitions:

| Transition | Trigger |
|---|---|
| IDLE -> NAV | `start_mission()` passes mission sync, health, FC readiness, arming, then sets state NAV (`docker_workspace/src/usv_main.py:6627`, `docker_workspace/src/usv_main.py:6788`) |
| NAV -> COMPLETED | `run_nav()` true and `engage_wp is None` (`docker_workspace/src/usv_main.py:9017`, `docker_workspace/src/usv_main.py:9021`) |
| NAV -> ENGAGE | `run_nav()` true and `engage_wp is not None` (`docker_workspace/src/usv_main.py:9017`, `docker_workspace/src/usv_main.py:9023`) |
| NAV -> HOLD | `run_nav()` false (`docker_workspace/src/usv_main.py:9023`, `docker_workspace/src/usv_main.py:9027`) |
| ENGAGE -> COMPLETED | `run_engage()` true (`docker_workspace/src/usv_main.py:9029`, `docker_workspace/src/usv_main.py:9031`) |
| ENGAGE -> HOLD | `run_engage()` false (`docker_workspace/src/usv_main.py:9032`, `docker_workspace/src/usv_main.py:9034`) |
| Any -> HOLD | `_trigger_estop()` -> `_enter_hold("ESTOP")` (`docker_workspace/src/usv_main.py:5590`, `docker_workspace/src/usv_main.py:5608`; `docker_workspace/src/usv_main.py:5625`, `docker_workspace/src/usv_main.py:5649`) |

P1 completion: race P1 is monitored by companion waypoint acceptance, not by `MISSION_REACHED` in the active code path. The code defaults `USV_RACE_P1_AUTO_WAYPOINTS` to 1 (`docker_workspace/src/usv_main.py:8650`, `docker_workspace/src/usv_main.py:8656`). Current `docker_workspace/mission.json` has 6 flat waypoints (`docker_workspace/mission.json:1`, `docker_workspace/mission.json:8`). Flag: [BUG-P1-EARLY-EXIT].

P1->P2 health gate: `_wait_p2_ready()` requires `camera_ready and lidar_ready` before continuing (`docker_workspace/src/usv_main.py:6816`, `docker_workspace/src/usv_main.py:6834`), but it has no timeout while mission remains active. If a sensor never becomes ready, it loops indefinitely.

P2 completion: `_track_gate_event()` counts gate events (`docker_workspace/src/usv_main.py:6836`, `docker_workspace/src/usv_main.py:6872`), but `run_nav()` only warns when `gate_count <= 0` (`docker_workspace/src/usv_main.py:8862`, `docker_workspace/src/usv_main.py:8864`). OTOMASYON requires at least two gate/obstacle passages (`OTOMASYON.md:190`, `OTOMASYON.md:192`). Flag: [BUG-P2-GATE-MIN-NOT-ENFORCED].

P2->P3 target color: `run_engage()` starts P3 without rechecking a locked target-color contract (`docker_workspace/src/usv_main.py:8875`, `docker_workspace/src/usv_main.py:8884`). Target color is loaded during mission load from `target_state.json` (`docker_workspace/src/usv_main.py:6519`, `docker_workspace/src/usv_main.py:6524`), and telemetry blocks target changes while active (`docker_workspace/src/telemetry.py:3599`, `docker_workspace/src/telemetry.py:3601`), but transition-time verification is absent.

P3 completion: source logic exists for `gps_proximity`, `vision_area`, `lidar_proximity`, and `lidar_collision` (`docker_workspace/src/usv_main.py:8580`, `docker_workspace/src/usv_main.py:8602`). Completion uses an OR-style quorum under `proximity_ok`, not all four sources. Wrong-target risk blocks completion (`docker_workspace/src/usv_main.py:8499`, `docker_workspace/src/usv_main.py:8502`; `docker_workspace/src/usv_main.py:8583`, `docker_workspace/src/usv_main.py:8588`).

Post-start lock: telemetry rejects mission upload while active (`docker_workspace/src/telemetry.py:3509`, `docker_workspace/src/telemetry.py:3512`) and target-color update while active (`docker_workspace/src/telemetry.py:3599`, `docker_workspace/src/telemetry.py:3601`). `usv_main.py` rejects changed Pixhawk mission mirrors after start (`docker_workspace/src/usv_main.py:6393`, `docker_workspace/src/usv_main.py:6418`).

E-stop: `_trigger_estop()` latches command lock, commands HOLD, force-sends safe outputs, and enters HOLD (`docker_workspace/src/usv_main.py:5590`, `docker_workspace/src/usv_main.py:5608`). `_enter_hold()` clears active mission, sets speed to 0, holds current heading, and calls `stop_motors()` (`docker_workspace/src/usv_main.py:5625`, `docker_workspace/src/usv_main.py:5649`). Expected response: within one loop plus mode/send latency; static code path is immediate.

## S3 — MISSION SCHEMA & PROFILE

1. `docker_workspace/mission.json` structure is flat:

```json
[
  [40.891952, 29.0232],
  [40.891962, 29.023215],
  [40.891972, 29.02323],
  [40.891982, 29.023245],
  [40.891992, 29.02326],
  [40.892002, 29.023275]
]
```

Source: `docker_workspace/mission.json:1`, `docker_workspace/mission.json:8`.

2. Flat format cannot distinguish P3 engagement waypoint from P2 waypoints. `mission_config.py` documents that flat list defaults to NAV-only (`docker_workspace/src/mission_config.py:4`, `docker_workspace/src/mission_config.py:8`) and `split_nav_engage()` returns all coords as nav when `has_explicit_engage=False` (`docker_workspace/src/mission_config.py:85`, `docker_workspace/src/mission_config.py:104`). Flag: [BUG-MISSION-NO-P3-MARKER].

3. `mission_profile` contract is only partially implemented. Mission state tracks `schema_version`, `upload_source`, `validation_timestamp`, `waypoint_counts`, and `mission_split_profile` (`docker_workspace/src/usv_main.py:3254`, `docker_workspace/src/usv_main.py:3283`), but it does not enforce explicit parkur ranges or a P3 engagement marker required by OTOMASYON (`OTOMASYON.md:146`, `OTOMASYON.md:157`).

4. Race start gate enforces `mission_upload_source == "pixhawk_mission"` (`docker_workspace/src/usv_main.py:6662`, `docker_workspace/src/usv_main.py:6674`). PASS.

5. Pixhawk mission mirror:

Download requests `MISSION_COUNT`, then sends `MISSION_REQUEST_INT` for each seq with up to 3 attempts, accepts `MISSION_ITEM_INT` or `MISSION_ITEM`, then sends accepted ACK (`docker_workspace/src/usv_main.py:6290`, `docker_workspace/src/usv_main.py:6364`; request helper at `docker_workspace/src/usv_main.py:5901`, `docker_workspace/src/usv_main.py:5930`). Upload path handles `MISSION_REQUEST_INT`, `MISSION_REQUEST`, and `MISSION_ACK`, and rejects non-accepted ACKs with error logging (`docker_workspace/src/usv_main.py:6163`, `docker_workspace/src/usv_main.py:6223`). Download has retry/timeout handling; NACK is mainly relevant to upload and is handled.

## S4 — SENSOR PIPELINE

### S4a — Camera (cam.py)

Detection classes and HSV ranges:

| Class | HSV |
|---|---|
| TURUNCU | `[5,65,90]` to `[22,255,255]` (`docker_workspace/src/cam.py:87`, `docker_workspace/src/cam.py:92`) |
| SARI | `[22,100,100]` to `[35,255,255]` (`docker_workspace/src/cam.py:93`, `docker_workspace/src/cam.py:98`) |
| YESIL | `[40,80,80]` to `[85,255,255]` (`docker_workspace/src/cam.py:99`, `docker_workspace/src/cam.py:104`) |
| KIRMIZI | `[0,150,100]` to `[5,255,255]`, plus `[172,150,100]` to `[180,255,255]` (`docker_workspace/src/cam.py:105`, `docker_workspace/src/cam.py:115`) |
| SIYAH | `[0,0,0]` to `[180,60,45]` (`docker_workspace/src/cam.py:111`, `docker_workspace/src/cam.py:115`) |

Wrong-target fields are present in status initialization and output: `wrong_target_detected`, `wrong_target_bearing_deg`, `wrong_target_area_norm`, `wrong_target_class` plus raw variants (`docker_workspace/src/cam.py:209`, `docker_workspace/src/cam.py:216`; `docker_workspace/src/cam.py:1060`, `docker_workspace/src/cam.py:1067`). PASS.

Gate bearing formula: center pixel offset is normalized around frame center and multiplied by `BEARING_HALF_DEG`; sim uses 45 degrees, real uses 35 degrees (`docker_workspace/src/cam.py:54`, `docker_workspace/src/cam.py:55`; gate center formula `docker_workspace/src/cam.py:885`, `docker_workspace/src/cam.py:891`). Target and wrong-target bearing use the same normalized offset pattern (`docker_workspace/src/cam.py:948`, `docker_workspace/src/cam.py:958`).

Pipeline latency: frame decode time and processing time are tracked in the `pipeline` status (`docker_workspace/src/cam.py:538`, `docker_workspace/src/cam.py:540`; `docker_workspace/src/cam.py:569`, `docker_workspace/src/cam.py:579`). Loop sleep defaults to 33 ms (`docker_workspace/src/cam.py:72`, `docker_workspace/src/cam.py:73`), so nominal file-status latency is one processing loop plus JPEG decode and JSON write.

MJPEG race guard: `cam.py` disables web output in race mode (`docker_workspace/src/cam.py:49`, `docker_workspace/src/cam.py:52`) and `telemetry.py` returns 403 for camera stream in race mode (`docker_workspace/src/telemetry.py:2650`, `docker_workspace/src/telemetry.py:2654`). PASS.

### S4b — Lidar (lidar_map.py)

`lidar_map.py` subscribes to ROS2 `LaserScan` topic `/scan` with BEST_EFFORT QoS (`docker_workspace/src/lidar_map.py:71`, `docker_workspace/src/lidar_map.py:83`). It is a visualization service; mission logic uses `usv_main.py`'s lidar callback and internal sector state.

Sectors: `usv_main.py` raw sectors are center `[-22, 22]`, left `>22` up to 90, right `<-22` down to -90 (`docker_workspace/src/usv_main.py:7407`, `docker_workspace/src/usv_main.py:7438`). Occupancy-map risk sectors are front `[-35,35]`, left `[20,70]`, right `[-70,-20]` (`docker_workspace/src/usv_main.py:7195`, `docker_workspace/src/usv_main.py:7203`).

Staleness: `LIDAR_FRAME_MAX_AGE_S=0.15` (`docker_workspace/src/usv_main.py:304`) and `LIDAR_READY_TIMEOUT_S=1.0` (`docker_workspace/src/compliance_profile.py:149`). Sector snapshot marks known only if sector age <=0.62 s (`docker_workspace/src/usv_main.py:7390`, `docker_workspace/src/usv_main.py:7403`).

Center obstacle threshold: `D_MIN_M=2.0` (`docker_workspace/src/compliance_profile.py:86`); callback sets `obstacle_detected = center < D_MIN_M` (`docker_workspace/src/usv_main.py:4125`).

Cost-map output: `usv_main.py` maintains an occupancy grid with `LIDAR_GRID_RES_M=0.25`, span 60 m (`docker_workspace/src/usv_main.py:325`, `docker_workspace/src/usv_main.py:326`) and exports occupied points in mission state (`docker_workspace/src/usv_main.py:3137`, `docker_workspace/src/usv_main.py:3144`). Consumer is internal navigation and telemetry dashboard.

Driver-drop detection: `lidar_ready` requires availability and recent frame within `LIDAR_READY_TIMEOUT_S` (`docker_workspace/src/usv_main.py:5500`, `docker_workspace/src/usv_main.py:5501`). Degraded guard enters limited or hold after stale-unreliable mode, with HOLD timeout 8 seconds (`docker_workspace/src/usv_main.py:7440`, `docker_workspace/src/usv_main.py:7461`; constants `docker_workspace/src/usv_main.py:345`, `docker_workspace/src/usv_main.py:346`).

### S4c — GPS / EKF

Position comes from `GLOBAL_POSITION_INT.lat/lon` (`docker_workspace/src/usv_main.py:5094`, `docker_workspace/src/usv_main.py:5114`). GPS fix comes from `GPS_RAW_INT.fix_type` and satellite count (`docker_workspace/src/usv_main.py:5126`, `docker_workspace/src/usv_main.py:5137`). EKF comes from `EKF_STATUS_REPORT` flags and variances (`docker_workspace/src/usv_main.py:5138`, `docker_workspace/src/usv_main.py:5153`). Start readiness requires fresh GPS/global position/EKF and variance limits through FC readiness timers (`docker_workspace/src/usv_main.py:264`, `docker_workspace/src/usv_main.py:277`; start gate `docker_workspace/src/usv_main.py:6712`, `docker_workspace/src/usv_main.py:6739`).

## S5 — SIMULATION INTEGRITY

### S5a — sitl_gazebo_bridge.py

Gazebo motion is driven by ArduPilot SITL servo output by default. `SERVO_LEFT_CH=1`, `SERVO_RIGHT_CH=3`, neutral 1500, min 1100, max 1900 (`sim/bridges/sitl_gazebo_bridge.py:40`, `sim/bridges/sitl_gazebo_bridge.py:41`; `sim/bridges/sitl_gazebo_bridge.py:72`, `sim/bridges/sitl_gazebo_bridge.py:81`). PWM is decoded from SITL binary PWM channel output (`sim/bridges/sitl_gazebo_bridge.py:329`, `sim/bridges/sitl_gazebo_bridge.py:342`).

Mapping is linear: `norm = (pwm - 1500) / 500`, then thrust is average and yaw command is `(left-right)/2` (`sim/bridges/sitl_gazebo_bridge.py:72`, `sim/bridges/sitl_gazebo_bridge.py:81`; `sim/bridges/sitl_gazebo_bridge.py:434`, `sim/bridges/sitl_gazebo_bridge.py:439`). Because the project PWM range is 1100/1500/1900 (`config/pixhawk_ida.param:742`, `config/pixhawk_ida.param:745`; `config/pixhawk_ida.param:787`, `config/pixhawk_ida.param:790`), dividing by 500 under-scales full command to 0.8. Flag: [BUG-SIM-PWM-SCALE].

`SIM_GZ_ALLOW_MOTOR_COMMAND_JSON` bypass is guarded by mode in the bridge (`sim/bridges/sitl_gazebo_bridge.py:43`, `sim/bridges/sitl_gazebo_bridge.py:46`) and rejected by sim stack startup (`sim/bin/run_sim_stack.sh:291`, `sim/bin/run_sim_stack.sh:298`). PASS.

Yaw sign: default `SIM_GZ_YAW_SIGN=-1.0` (`sim/bridges/sitl_gazebo_bridge.py:34`, `sim/bridges/sitl_gazebo_bridge.py:37`). Static agreement with real boat convention is UNKNOWN; docs list it as open calibration risk (`OTOMASYON.md:373`, `OTOMASYON.md:379`).

Pose feedback: bridge converts Gazebo odometry into JSON-like SITL payload with position, attitude, velocity, lat/lon (`sim/bridges/sitl_gazebo_bridge.py:212`, `sim/bridges/sitl_gazebo_bridge.py:301`) and responds to SITL socket messages (`sim/bridges/sitl_gazebo_bridge.py:496`, `sim/bridges/sitl_gazebo_bridge.py:539`). It also writes `vehicle_position.json` at 10 Hz (`sim/bridges/sitl_gazebo_bridge.py:547`, `sim/bridges/sitl_gazebo_bridge.py:589`).

### S5b — Startup Race Condition

Required order includes `lidar_map.py` before `usv_main.py` (`AGENTS.md:40`, `AGENTS.md:53`). Actual sim startup launches Gazebo, ROS-GZ bridge, SITL, sitl bridge, camera bridge, cam.py, telemetry, then skips `lidar_map.py` (`sim/bin/run_sim_stack.sh:640`, `sim/bin/run_sim_stack.sh:721`) and starts `usv_main.py` (`sim/bin/run_sim_stack.sh:723`, `sim/bin/run_sim_stack.sh:725`). `check_stack.sh` does not check `lidar_map.py` (`sim/bin/check_stack.sh:1`, `sim/bin/check_stack.sh:28`). Flag: [BUG-SIM-LIDAR-MAP-NOT-STARTED].

Bare sleeps substitute for readiness at Gazebo and bridge stages (`sim/bin/run_sim_stack.sh:640`, `sim/bin/run_sim_stack.sh:656`; `sim/bin/run_sim_stack.sh:664`, `sim/bin/run_sim_stack.sh:675`). SITL and telemetry do have readiness waits (`sim/bin/run_sim_stack.sh:658`, `sim/bin/run_sim_stack.sh:662`; `sim/bin/run_sim_stack.sh:686`, `sim/bin/run_sim_stack.sh:692`). Flag: [BUG-STARTUP-SLEEP-RACE].

### S5c — Sim vs Real Divergence

| Parameter | Sim Value | Real Value | Source |
|---|---:|---:|---|
| CONTROL_HZ | 40 | 10 | `docker_workspace/src/compliance_profile.py:56`, `docker_workspace/src/compliance_profile.py:62` |
| TELEMETRY_HZ | 40 | 5 | `docker_workspace/src/compliance_profile.py:57`, `docker_workspace/src/compliance_profile.py:63` |
| LIDAR_PROCESSING_HZ | 40 | 10 | `docker_workspace/src/compliance_profile.py:59`, `docker_workspace/src/compliance_profile.py:65` |
| camera bearing half-FOV | 45 deg | 35 deg | `docker_workspace/src/cam.py:54`, `docker_workspace/src/cam.py:55` |
| LIDAR_SELF_FILTER_MIN_M | 0.55 | 0.20 | `docker_workspace/src/usv_main.py:307`, `docker_workspace/src/usv_main.py:310` |
| TRUST_LIDAR_POINTS_FULL | 720 | 360 | `docker_workspace/src/compliance_profile.py:231`, `docker_workspace/src/compliance_profile.py:233` |
| actuation path | SITL SERVO_OUTPUT_RAW -> Gazebo | Pixhawk SERVO output -> ESC | `OTOMASYON.md:72`, `OTOMASYON.md:75`; `sim/bridges/sitl_gazebo_bridge.py:363`, `sim/bridges/sitl_gazebo_bridge.py:371` |
| race image/map stream | disabled | disabled | `docker_workspace/src/telemetry.py:2650`, `docker_workspace/src/telemetry.py:2685`; `host_scripts/system_start.sh:153`, `host_scripts/system_start.sh:160` |

## S6 — PIXHAWK PARAM AUDIT (config/pixhawk_ida.param)

Static audit note: the local repository contains exported/current params only, not an ArduRover default dump. Defaults below are therefore marked `default-known` only where the project rules or common ArduPilot baseline make the risk explicit; otherwise default is `not verified locally`.

| PARAM_NAME | current | ArduRover default | risk_level |
|---|---:|---|---|
| ARMING_CHECK | 0 | default-known enabled | HIGH [RISK-ARMING-NO-PREARM], `config/pixhawk_ida.param:17` |
| ARMING_REQUIRE | 0 | default-known enabled | HIGH [RISK-ARMING-NO-REQUIRE], `config/pixhawk_ida.param:22` |
| FS_THR_ENABLE | 0 | not verified locally | HIGH [RISK-FAILSAFE-RC-DISABLED], `config/pixhawk_ida.param:384` |
| FS_GCS_ENABLE | 0 | not verified locally | MEDIUM [RISK-FAILSAFE-GCS-DISABLED], `config/pixhawk_ida.param:381` |
| BATT_FS_LOW_ACT | 0 | not verified locally | HIGH [RISK-FAILSAFE-BATT-DISABLED], `config/pixhawk_ida.param:124` |
| BATT_FS_CRT_ACT | 0 | not verified locally | HIGH, `config/pixhawk_ida.param:123` |
| CRUISE_SPEED | 2 m/s | not verified locally | MEDIUM, competition safety depends on area and braking distance, `config/pixhawk_ida.param:257` |
| WP_SPEED | 2 m/s | not verified locally | MEDIUM, `config/pixhawk_ida.param:919` |
| WP_RADIUS | 2 m | not verified locally | MEDIUM overshoot risk around 1.60 m hull, `config/pixhawk_ida.param:918` |
| TURN_RADIUS | 0.9 m | not verified locally | MEDIUM skid-steer tuning risk, `config/pixhawk_ida.param:908` |
| SERVO1_FUNCTION | 74 | frame-dependent | MEDIUM, must match left throttle, `config/pixhawk_ida.param:741` |
| SERVO3_FUNCTION | 73 | frame-dependent | MEDIUM, must match right throttle, `config/pixhawk_ida.param:786` |
| SERVO1/3 min/trim/max | 1100/1500/1900 | frame-dependent | MEDIUM, bridge scaling mismatch, `config/pixhawk_ida.param:742`, `config/pixhawk_ida.param:790` |
| FRAME_CLASS | 2 | vehicle-dependent | MEDIUM, boat/skid-steer confirmation required, `config/pixhawk_ida.param:370` |
| FRAME_TYPE | 0 | vehicle-dependent | MEDIUM, `config/pixhawk_ida.param:371` |
| FS_ACTION | 2 | not verified locally | MEDIUM, `config/pixhawk_ida.param:377` |
| FS_EKF_ACTION | 1 | not verified locally | MEDIUM, `config/pixhawk_ida.param:379` |

The project already documents the arming/failsafe baseline as requiring race certification before deployment (`AGENTS.md:105`, `AGENTS.md:117`).

## S7 — COMPLIANCE PROFILE AUDIT (compliance_profile.py)

Key constants and risk if wrong:

| CONSTANT_NAME | value | used_in | risk_if_wrong |
|---|---|---|---|
| CONTROL_HZ | 10 real, 40 sim | control loop | missed GUIDED freshness or CPU overload (`docker_workspace/src/compliance_profile.py:56`, `docker_workspace/src/compliance_profile.py:62`) |
| TELEMETRY_HZ | 5 real, 40 sim | telemetry | stale dashboard/start gate (`docker_workspace/src/compliance_profile.py:57`, `docker_workspace/src/compliance_profile.py:63`) |
| LIDAR_PROCESSING_HZ | 10 real, 40 sim | lidar processing | stale obstacle decisions (`docker_workspace/src/compliance_profile.py:59`, `docker_workspace/src/compliance_profile.py:65`) |
| R_WP_M | default 2.5, max 3.0 | waypoint acceptance | early/late waypoint completion (`docker_workspace/src/compliance_profile.py:67`, `docker_workspace/src/compliance_profile.py:69`) |
| P1_SPEED_CRUISE_MPS | 1.5 | P1 | overshoot/score loss (`docker_workspace/src/compliance_profile.py:72`) |
| P2_SPEED_CRUISE_MPS | 2.0 | P2 | gate miss/unsafe approach (`docker_workspace/src/compliance_profile.py:88`) |
| P3_MAX_SPEED_MPS | 1.5 | P3 | collision or no contact (`docker_workspace/src/compliance_profile.py:77`) |
| D_MIN_M | 2.0 | lidar obstacle threshold | collision or over-avoidance (`docker_workspace/src/compliance_profile.py:86`) |
| P2_LIDAR_WARN_M | 2.5 | P2 warning | late avoidance (`docker_workspace/src/compliance_profile.py:88`) |
| CAMERA_FRAME_TIMEOUT_S | 1.0 | camera freshness | stale vision steering (`docker_workspace/src/compliance_profile.py:148`) |
| LIDAR_READY_TIMEOUT_S | 1.0 | lidar freshness | stale obstacle state (`docker_workspace/src/compliance_profile.py:149`) |
| LINK_HEARTBEAT_WARN_S / FAIL_S | 5 / 30 | heartbeat | late operator link fault (`docker_workspace/src/compliance_profile.py:145`, `docker_workspace/src/compliance_profile.py:146`) |
| RC_START_PWM / RC_ESTOP_PWM | 1700 / 1900 | race start/estop | false start or missed stop (`docker_workspace/src/compliance_profile.py:238`, `docker_workspace/src/compliance_profile.py:241`) |
| PWM_MIN/TRIM/MAX_US | 1100/1500/1900 | actuator bounds | sim-real mismatch (`docker_workspace/src/compliance_profile.py:256`, `docker_workspace/src/compliance_profile.py:258`) |

Innovation switches are all ON except `unified_execution_path=False` (`docker_workspace/src/compliance_profile.py:153`, `docker_workspace/src/compliance_profile.py:164`). Disabled unified execution path is a race-readiness concern only if real and sim diverge further.

HSV overlap: red upper `[0..5]` overlaps orange lower boundary at H=5, and orange upper H=22 overlaps yellow lower H=22 (`docker_workspace/src/cam.py:87`, `docker_workspace/src/cam.py:107`). This can create wrong-target ambiguity under noise.

P3 engagement thresholds: GPS proximity <=1.0 m, vision area >=0.20, lidar proximity <0.8 m, collision/stall via commanded speed and actual speed (`docker_workspace/src/usv_main.py:8580`, `docker_workspace/src/usv_main.py:8602`).

## S8 — IPC & DATA INTEGRITY (json_atomic.py)

`docker_workspace/src/json_atomic.py` is MISSING even though project guidance explicitly references it (`GEMINI.md:62`). Atomic write patterns are implemented ad hoc:

| Shared file | Writer(s) | Reader(s) | Freshness guard |
|---|---|---|---|
| `camera_status.json` | `cam.py` temp + replace (`docker_workspace/src/cam.py:620`, `docker_workspace/src/cam.py:624`) | `usv_main.py` (`docker_workspace/src/usv_main.py:5316`, `docker_workspace/src/usv_main.py:5321`) | age/mtime > `CAMERA_STATUS_TIMEOUT_S=2.5` stale (`docker_workspace/src/usv_main.py:247`, `docker_workspace/src/usv_main.py:5333`, `docker_workspace/src/usv_main.py:5343`) |
| `mission_state.json` | `usv_main.py` temp + fsync + replace, fallback direct write (`docker_workspace/src/usv_main.py:3536`, `docker_workspace/src/usv_main.py:3550`) | `telemetry.py`, `cam.py`, bridge (`docker_workspace/src/telemetry.py:2693`, `docker_workspace/src/telemetry.py:2698`; `docker_workspace/src/cam.py:344`, `docker_workspace/src/cam.py:347`; `sim/bridges/sitl_gazebo_bridge.py:97`) | reader-side defaults; no unified schema validator |
| `target_state.json` | `telemetry.py` temp + replace (`docker_workspace/src/telemetry.py:3609`, `docker_workspace/src/telemetry.py:3616`) | `cam.py`, `usv_main.py` (`docker_workspace/src/cam.py:288`, `docker_workspace/src/cam.py:296`; `docker_workspace/src/usv_main.py:6519`, `docker_workspace/src/usv_main.py:6524`) | no timestamp freshness gate; post-start write blocked by API |
| `lidar_status.json` | none found | none found | NOT IMPLEMENTED |

Bypasses: telemetry writes mission file directly with `open(..., 'w')` and `json.dump()` (`docker_workspace/src/telemetry.py:3541`, `docker_workspace/src/telemetry.py:3545`); `usv_main.py` has a direct-write fallback for mission state (`docker_workspace/src/usv_main.py:3547`, `docker_workspace/src/usv_main.py:3550`); camera writes latest JPEG directly (`docker_workspace/src/cam.py:649`, `docker_workspace/src/cam.py:651`). Flag: [BUG-NON-ATOMIC-WRITE].

If `cam.py` crashes, `usv_main.py` detects stale data via `ts_monotonic` and mtime age, clears detections and wrong-target fields (`docker_workspace/src/usv_main.py:5333`, `docker_workspace/src/usv_main.py:5343`; `docker_workspace/src/usv_main.py:5402`, `docker_workspace/src/usv_main.py:5442`). PASS.

## S9 — LOGGING AUDIT

Runtime logging infrastructure uses rotating handlers for debug and JSONL logs (`docker_workspace/src/runtime_debug_log.py:142`, `docker_workspace/src/runtime_debug_log.py:147`; `docker_workspace/src/runtime_debug_log.py:198`, `docker_workspace/src/runtime_debug_log.py:208`). Sizes: debug 50 MB x10, JSONL 100 MB x15 (`docker_workspace/src/runtime_debug_log.py:24`, `docker_workspace/src/runtime_debug_log.py:27`).

Core service log coverage in current `/logs`:

| Component | debug.log | jsonl | Status |
|---|---|---|---|
| cam | present | present | PASS |
| telemetry | present | present | PASS |
| usv_main | present | present | PASS |
| sitl_gazebo_bridge | present in logs/simulation | present in logs/simulation | PASS |
| ros_to_tcp_cam | present in logs/simulation | present in logs/simulation | PASS |
| lidar_map | missing | missing | FAIL [BUG-MISSING-LOG-FILE] |

Telemetry allowlist expects `lidar_map.jsonl` (`docker_workspace/src/telemetry.py:189`), but current log file is missing and sim stack skips the service (`sim/bin/run_sim_stack.sh:715`, `sim/bin/run_sim_stack.sh:721`).

Critical events: mode, mission lifecycle, waypoint, e-stop, camera/lidar readiness, and P3 contact fields are present in code paths (`docker_workspace/src/usv_main.py:3188`, `docker_workspace/src/usv_main.py:3315`; contact `docker_workspace/src/usv_main.py:8580`, `docker_workspace/src/usv_main.py:8602`). Runtime tail did not include a successful mission.

Last 10 WARN/ERROR lines from available tails:

```text
logs/system/usv_main.debug.log:174:2026-05-22 19:53:54 | INFO     | usv.usv_main | [STDOUT] [19:53:54] [USV] [WARN] [MAV] SIM endpoint hatasi (tcp:127.0.0.1:5760): [Errno 111] Connection refused
logs/system/usv_main.debug.log:175:2026-05-22 19:53:59 | INFO     | usv.usv_main | [STDOUT] [19:53:59] [USV] [WARN] [MAV] SIM endpoint hatasi (udpin:127.0.0.1:14550): heartbeat timeout
logs/system/usv_main.debug.log:176:2026-05-22 19:53:59 | INFO     | usv.usv_main | [STDOUT] [19:53:59] [USV] [WARN] [MAV] Baglanti yok: SIM MAVLink endpoint bulunamadi
logs/system/usv_main.debug.log:177:2026-05-22 19:53:59 | INFO     | usv.usv_main | [STDOUT] [19:53:59] [USV] [HATA] [SIM] SITL baglantisi kurulamadi. Simulasyon stack'ini kontrol edin.
logs/system/usv_main.debug.log:178:2026-05-22 19:53:59 | INFO     | usv.usv_main | [STDOUT] [19:53:59] [USV] [HATA] [SIM] run_sim_stack.sh uzerinden baslatin veya SITL calistigini dogrulayin.
```

Root cause: the latest tail is an offline/manual `usv_main.py` start without SITL on `tcp:127.0.0.1:5760` or UDP heartbeat. Earlier same log also contains a failed start due to Pixhawk/SITL arming blockers: `Arm: Gyros inconsistent`, `SmartRTL deactivated: bad position`, and `Arming basarisiz`.

## S10 — FAULT REGISTRY (PRIORITY SORTED)

| Severity | Subsystem | Fault | File:Line | Root cause | Exact fix |
|---|---|---|---|---|---|
| CRITICAL | Mission/P3 | [BUG-MISSION-NO-RACE-PROFILE] default flat mission does not lock target color or transition policy; P3 must be entered as a vision/color engagement phase | `docker_workspace/mission.json:1`; `docker_workspace/src/mission_config.py:100`; `docker_workspace/src/usv_main.py:9018` | Flat schema carries coordinates only and cannot express race phase policy | Add mission_profile with target_color lock, transition policy, P2 gate minimum, and P3 vision/color engagement mode; reject race start when profile is absent/invalid |
| HIGH | P1 race | [BUG-P1-EARLY-EXIT] P1 waypoint count defaults to 1 while current mission has 6 flat waypoints | `docker_workspace/src/usv_main.py:8650`; `docker_workspace/src/usv_main.py:8656`; `docker_workspace/mission.json:1` | Missing mission_profile-derived P1 range | Make `USV_RACE_P1_AUTO_WAYPOINTS` mandatory in race or derive it from validated mission_profile; fail closed when unset |
| HIGH | P2 guidance | [BUG-P2-GATE-BEARING-DISABLED] camera gate bearing is not used in active P2 steering | `docker_workspace/src/usv_main.py:8266`; `docker_workspace/src/nav_guidance.py:104` | Active code passes gate disabled to guidance | Pass corrected `gate_detected/gate_center_bearing_deg` into guidance and reintroduce bounded gate assist after lidar priority |
| HIGH | P2 completion | [BUG-P2-GATE-MIN-NOT-ENFORCED] min two gate passages not enforced | `docker_workspace/src/usv_main.py:8862`; `OTOMASYON.md:190` | Only logs when `gate_count <= 0` | Require `gate_count >= 2` before P2 completion/P3 transition, or enforce profile-specific minimum |
| HIGH | Sim startup | [BUG-SIM-LIDAR-MAP-NOT-STARTED] sim stack skips `lidar_map.py` despite required order and log standard | `sim/bin/run_sim_stack.sh:715`; `AGENTS.md:47`; `docker_workspace/src/telemetry.py:189` | Service launch is disabled/commented | Start `lidar_map.py` before `usv_main.py` in test mode, or document replacement and remove stale dashboard/log expectations |
| HIGH | Sim actuation | [BUG-SIM-PWM-SCALE] bridge uses `/500` while configured PWM range is +/-400 us | `sim/bridges/sitl_gazebo_bridge.py:72`; `config/pixhawk_ida.param:742`; `config/pixhawk_ida.param:787` | Hardcoded normalization span does not match baseline params | Normalize by `(SERVO_MAX - SERVO_TRIM)` and `(SERVO_TRIM - SERVO_MIN)` from env/config |
| MEDIUM | Startup | [BUG-STARTUP-SLEEP-RACE] bare sleeps substitute for health checks | `sim/bin/run_sim_stack.sh:640`; `sim/bin/run_sim_stack.sh:675` | Process start assumed ready after fixed delay | Replace sleeps with Gazebo topic, ROS bridge topic, camera socket, and bridge heartbeat readiness waits |
| MEDIUM | Mission profile | mission_profile contract incomplete | `OTOMASYON.md:146`; `docker_workspace/src/usv_main.py:3254` | Lifecycle fields exist but no enforced target-color/transition-policy schema | Implement `profile_schema_version`, target lock source, phase transition policy, P2 gate minimum, and P3 vision/color engagement mode |
| MEDIUM | P3 | P3 completion can latch from single evidence cue | `docker_workspace/src/usv_main.py:8583`; `docker_workspace/src/usv_main.py:8602` | `proximity_ok` is OR of GPS/vision/lidar/collision | Require a multi-cue quorum, for example vision plus GPS/lidar or lidar collision plus low speed, while preserving wrong-target block |
| MEDIUM | P3 transition | target color not revalidated immediately before ENGAGE | `docker_workspace/src/usv_main.py:8875`; `docker_workspace/src/usv_main.py:8884` | P3 trusts previous load/start state | Add transition guard: locked target_color present, source timestamp pre-start, camera policy target active |
| MEDIUM | IPC | [BUG-NON-ATOMIC-WRITE] `json_atomic.py` missing and mission write bypasses atomic replace | `GEMINI.md:62`; `docker_workspace/src/telemetry.py:3544`; `docker_workspace/src/usv_main.py:3547` | Ad hoc JSON writes and direct fallback | Add `json_atomic.py` helper with fsync+replace and replace all shared JSON writes; remove direct fallback or write to `.corrupt` on failure |
| MEDIUM | Logging | [BUG-MISSING-LOG-FILE] `lidar_map.debug.log/jsonl` missing | `AGENTS.md:121`; `sim/bin/run_sim_stack.sh:715`; `docker_workspace/src/telemetry.py:189` | Service not started and not using central log in current session | Start service and initialize `setup_component_logger("lidar_map")`/JSONL heartbeat |
| LOW | Heading | [BUG-HEADING-WRAP-ROBUSTNESS] wrap uses single +/-360 adjustment, not atan2 formulation | `docker_workspace/src/usv_main.py:381`; `docker_workspace/src/usv_main.py:386` | Function assumes bounded input | Replace with `math.degrees(math.atan2(math.sin(rad), math.cos(rad)))` |
| LOW | Vision | HSV boundary overlap between red/orange and orange/yellow | `docker_workspace/src/cam.py:87`; `docker_workspace/src/cam.py:107` | Inclusive hue ranges share boundary values | Add non-overlap margins or class priority with confidence separation |
| LOW | Dashboard lidar | `lidar_map.py` sim fallback can show synthetic map when scan data absent | `docker_workspace/src/lidar_map.py:248`; `docker_workspace/src/lidar_map.py:267` | Visualization fallback hides missing scan source | In compliance mode, display unavailable/stale instead of fake obstacle map |

## S11 — QUICK WINS (≤10 items)

1. `docker_workspace/src/usv_main.py:8650` | Fail race start if `USV_RACE_P1_AUTO_WAYPOINTS` unset and mission_profile absent | prevents P1 early exit.
2. `docker_workspace/src/usv_main.py:8862` | Change P2 completion guard to require `gate_count >= 2` | aligns with OTOMASYON and parkur rules.
3. `docker_workspace/src/usv_main.py:8266` | Pass camera gate fields into `compute_nav_decision()` | restores gate-centered P2 steering.
4. `sim/bridges/sitl_gazebo_bridge.py:72` | Normalize PWM using 1100/1500/1900 spans | fixes sim thrust/yaw scaling.
5. `sim/bin/run_sim_stack.sh:715` | Re-enable `lidar_map.py` in test mode before `usv_main.py` | restores required startup order and logs.
6. `docker_workspace/src/telemetry.py:3544` | Write mission JSON via atomic temp+fsync+replace | prevents partial mission files.
7. `docker_workspace/src/usv_main.py:381` | Replace heading wrap with atan2(sin,cos) | robust heading math.
8. `docker_workspace/src/usv_main.py:8583` | Require at least two P3 contact evidence sources | reduces false latch.
9. `docker_workspace/src/cam.py:87` | Remove HSV boundary overlaps at H=5 and H=22 | reduces wrong-target ambiguity.
10. `sim/bin/run_sim_stack.sh:640` | Replace Gazebo/bridge sleeps with readiness probes | reduces startup races.

## S12 — RACE READINESS CHECKLIST

| # | Criterion | Rating | Evidence |
|---:|---|---|---|
| 1 | Variable-length mission supported, no hardcoded indices | PASS | race P1 no silent count=1 default; p2_evidence completion when env unset (`docker_workspace/src/usv_main.py:9186`, `host_scripts/check_compliance_race.py` test 10) |
| 2 | Race mode: no image/map stream to external clients | PASS | stream endpoints return 403 in race (`docker_workspace/src/telemetry.py:2650`, `docker_workspace/src/telemetry.py:2685`) |
| 3 | P1: pure Pixhawk AUTO, zero Pi override/fallback | PASS | race path logs pure AUTO and no fallback (`docker_workspace/src/usv_main.py:8807`, `docker_workspace/src/usv_main.py:8837`) |
| 4 | Post-start lock: mission reload and retarget blocked | PASS | mission and target APIs reject active mission (`docker_workspace/src/telemetry.py:3509`, `docker_workspace/src/telemetry.py:3601`) |
| 5 | Mission lifecycle: upload_source + validation_timestamp tracked | PASS | mission_lifecycle fields present (`docker_workspace/src/usv_main.py:3254`, `docker_workspace/src/usv_main.py:3266`) |
| 6 | Guidance source telemetry defined | PASS | `guidance_source` emitted (`docker_workspace/src/usv_main.py:3230`) |
| 7 | Mission adapter handles both flat and structured format | PASS | flat validation and structured adapter path exist (`docker_workspace/src/telemetry.py:3524`, `docker_workspace/src/telemetry.py:3529`) |
| 8 | Compliance constraints enforced in code | PASS | mission_profile contract, P2 min gate, P3 multi-cue quorum enforced (`docker_workspace/src/mission_config.py`, `host_scripts/check_mission_profile_unit.py`, `host_scripts/check_compliance_race.py` test 9) |
| 9 | E-stop clears setpoints within one control loop | PASS | command lock, HOLD, safe outputs, stop motors (`docker_workspace/src/usv_main.py:5590`, `docker_workspace/src/usv_main.py:5649`) |
| 10 | ARMING_CHECK/failsafe race cert plan documented | PASS | `documents/race_cert_pixhawk_arming_failsafe.md`, `config/pixhawk_ida.param`, `host_scripts/compare_pixhawk_param_baseline.py` |

Score: **10/10** (recomputed by `host_scripts/check_race_readiness_score.py`). Static/code gate: PASS (minimum 8/10). Field deploy still requires operator checklist + water test sign-off per Phase 8 docs.

## S13 — SYNTAX CHECK

Command:

```bash
python3 -m py_compile docker_workspace/src/*.py 2>&1 | head -30
```

Result: PASS. No syntax errors were emitted for top-level `docker_workspace/src/*.py`.

## S14 — IMPLEMENTATION TODO LIST

Bu liste, rapordaki hataları yarışa hazır hale getirecek sırayla uygulanmalıdır. Her adım bittikten sonra sadece ilgili statik/doğrulayıcı kontroller çalıştırılmalı; otomatik sürüş veya mission progression simülasyonu ajan tarafından başlatılmamalıdır.

Operasyonel düzeltme: P1/P2 ayrımı her zaman statik waypoint indeksinden bilinemeyebilir. P3 de sabit waypoint engagement değildir; hedef renk algılanır, yanlış hedef riski yoksa renk/vision takibi ile yaklaşılır. Bu yüzden mission_profile sabit P1/P2/P3 range veya P3 engage waypoint'e bağımlı olmamalıdır; bunlar varsa yalnızca yardımcı ipucu sayılır.

### Phase 1 — Mission Schema ve Start Kilitleri

1. [x] `mission_profile` sözleşmesini tanımla. (Tamamlandı: `docker_workspace/src/mission_config.py` içinde schema version, target_color lock, transition policy, P2 minimum gate count ve P3 vision/color engagement mode eklendi; parkur range ve P1 count yalnız opsiyonel ipucu.)
   - Alanlar: `profile_schema_version`, `target_color`, `upload_source`, `validation_timestamp`, `phase_transition_policy`, `p2_min_gate_count`, `p3_engagement_mode`; opsiyonel ipuçları: `parkur_ranges`, `p1_waypoint_count`.
   - Flat mission sadece test/backward-compat giriş formatı olarak kalsın.

2. [x] `docker_workspace/src/mission_config.py` içinde structured profile parser ekle. (Tamamlandı: `parse_mission_profile_payload()` eklendi; flat payload `race_ready=False`, structured profile/legacy parkur payloadları target_color, transition policy, P2 gate minimum ve P3 vision/color mode doğruluyor.)
   - P1/P2/P3 aralıkları varsa yalnız bounds ipucu olarak doğrula.
   - P3 engage waypoint zorunlu sayma; P3 renk/vision takip fazıdır.
   - Flat list için mevcut davranışı koru ama race start'a uygun kabul etme.

3. [x] `docker_workspace/src/usv_main.py` mission load akışını profile-aware yap. (Tamamlandı: mission load profile metadata'yı `mission_state.json` ve telemetry API'ye taşıyor; `engage_wp` zorunlu değil, P3 `p3_engagement_mode=vision_color_track` olarak temsil ediliyor.)
   - `nav_waypoints`, `mission_upload_source`, `waypoint_counts` ve `mission_profile` durumunu profile'dan üret.
   - `engage_wp` zorunlu olmasın; P3 engagement hedef renk/vision ile başlasın.
   - `mission_state.json` içine profile doğrulama sonucunu yaz.

4. [x] Race start öncesi fail-closed kontrolleri ekle. (Tamamlandı: race start artık Pixhawk mission source yanında `mission_profile_valid`, `mission_profile_race_ready`, `target_color` ve `p3_engagement_mode` kontrol ediyor.)
   - `mission_upload_source == "pixhawk_mission"` zaten var; buna profile doğrulama, target_color lock ve P3 vision/color engagement mode ekle.
   - `target_color` kilitli değilse veya profile geçersizse race start reddedilsin.

5. [x] `USV_RACE_P1_AUTO_WAYPOINTS` varsayılanını kaldır. (Tamamlandı: env verilmemişse race P1 artık `1 waypoint` varsaymıyor; Pixhawk AUTO pasif izlenip P2 kamera/lidar kanıtı bekleniyor.)
   - Statik waypoint split bilinmiyorsa değer profile'dan zorla türetilmesin.
   - Race P1 completion Pixhawk AUTO progress / mission progress kaynağıyla fail-closed izlenmeli.
   - Test modda mevcut backward-compatible fallback korunabilir.

### Phase 2 — P1/P2 State Machine Düzeltmeleri

6. [x] P1 completion mantığını statik waypoint sayısından ayır. (Tamamlandı: split yoksa P1 completion `p2_evidence:*` kaynağıyla kamera/lidar P2 kanıtından gelir; explicit env verilirse eski count yolu geriye dönük uyum için kalır.)
   - Statik split varsa `p1_auto_count` yalnız açık env ile kullanılır.
   - Split yoksa Pixhawk AUTO pasif izlenir; `MISSION_CURRENT` / `MISSION_ITEM_REACHED` telemetrisi state'e loglanır.
   - Companion-side completion source `race_p1_completion_source` olarak yazılır.

7. [x] `_wait_p2_ready()` için timeout ekle. (Tamamlandı: `USV_P2_READY_TIMEOUT_S`, default 30s; timeout JSONL ve HOLD.)
   - Kamera/lidar hazır olmazsa sonsuz bekleme yerine HOLD/failsafe'e geç.
   - Timeout ve sebep `mission_state.json` ve JSONL loglara yazılsın.

8. [x] P2 gate bearing'i aktif guidance yoluna bağla. (Tamamlandı: `_navigate_p2_waypoint()` gerçek `gate_detected`, `gate_stable_s`, `gate_center_bearing_deg` değerlerini `compute_nav_decision()` içine gönderiyor.)
   - `_navigate_p2_waypoint()` içinde `gate_detected`, `gate_center_bearing_deg`, `gate_stable_s` gerçek camera status değerleriyle gönderilsin.
   - Lidar avoidance önceliği korunmalı: lidar avoidance > gate bearing > waypoint.

9. [x] `compute_nav_decision()` içinde bounded gate assist'i yeniden uygula. (Tamamlandı: gate bias clamp `±12 deg`, gate delta clamp `±35 deg`; lidar avoidance hâlâ öncelikli.)
   - Eski `compute_p2_decision()` davranışından kontrollü aktar.
   - Gate bias clamp ve source/reason alanları açık loglansın.

10. [x] P2 completion için minimum gate passage zorunluluğu koy. (Tamamlandı: `p2_min_gate_count` default 2; eksikse ek gate tracking bekleniyor, timeout olursa fail-closed.)
    - Default minimum: 2.
    - Değer mission_profile'dan override edilebilir.
    - `gate_count < min` ise P3'e geçme; HOLD değil, waypoint/gate tracking devam etsin.

### Phase 3 — P3 Engagement ve Wrong Target Güvenliği

11. [x] P2 -> P3 geçişinde target_color yeniden doğrula. (Tamamlandı: P3 girişinde target_state/profile/runtime lock doğrulanıyor; start sonrası `target_color_changed_at` değişimi HOLD ile reddediliyor.)
    - Target color kilitli değilse P3 başlatma.
    - `target_color_changed_at` start sonrası ise reddet.

12. [x] P3 contact latch için multi-cue quorum uygula. (Tamamlandı: tek vision/GPS/lidar kanıtı latch kurmuyor; wrong-target riski tüm contact quorum'u blokluyor.)
    - Tek başına `vision_area` veya tek başına GPS proximity yeterli olmasın.
    - Önerilen kural: wrong-target risk yokken en az iki kanıt; örnek `vision_area + gps_proximity`, `vision_area + lidar_proximity`, veya `lidar_collision + low_speed`.

13. [x] `p3_contact_confirmation_source` değerini daha ayrıntılı yap. (Tamamlandı: kaynaklar `vision_area+gps_proximity` benzeri birleşik yazılıyor; `P3_CONTACT_LATCH` JSONL tüm evidence booleanlarını taşıyor.)
    - Örnek: `vision_area+gps_proximity`, `lidar_proximity+collision`.
    - Hangi kanıtların false/true olduğu JSONL loga yazılsın.

14. [x] Wrong-target aktifken yaklaşma ve latch blokajını testlerle sabitle. (Tamamlandı: behavior compliance kontrolüne P3 target lock, multi-cue quorum ve wrong-target block kontrolleri eklendi.)
    - `wrong_target_detected`, `wrong_target_area_norm`, `wrong_target_bearing_deg` için behavior test ekle.

### Phase 4 — Heading ve Guidance Stabilitesi

15. [x] `normalize_heading_error()` fonksiyonunu atan2 formülüne çevir. (Tamamlandı: `degrees(atan2(sin(rad), cos(rad)))` wrap formülü kullanılıyor.)
    - Formül: `degrees(atan2(sin(rad), cos(rad)))`.
    - Mevcut bütün çağrılar aynı imzayla çalışmalı.

16. [x] P3 heading tracking için gain/clamp parametreleri ekle. (Tamamlandı: `P3_TARGET_BEARING_GAIN` ve `P3_TARGET_HEADING_CLAMP_DEG` profile/env üzerinden yönetiliyor; P3 vision heading komutu clamp ediliyor.)
    - `P3_TARGET_BEARING_GAIN`, `P3_TARGET_HEADING_CLAMP_DEG`.
    - Ortak heading damping mevcut kalmalı.

17. [x] Heading diagnostic loglarını standardize et. (Tamamlandı: `nav_track`, `p3_track`, `P3_WRONG_TARGET_AVOID` ve `P3_CONTACT_LATCH` logları ortak heading/yaw/source alanlarını taşıyor.)
    - `heading_error_deg`, `heading_target_deg`, `current_heading_deg`, `yaw_rate_dps`, `guidance_source` alanları `nav_track` ve P3 loglarında düzenli çıksın.

18. [x] Kullanılabilir loglardan yaw oscillation ölçümü çıkaracak offline analiz script'i ekle. (Tamamlandı: `host_scripts/analyze_heading_oscillation.py` yalnız JSONL log okur, görev/sim başlatmaz.)
    - Script sadece log okur, görev başlatmaz.
    - Max abs heading error, sign flip rate ve yaw-rate overshoot raporlasın.

### Phase 5 — Simülasyon Entegrasyonu

19. [x] `sim/bridges/sitl_gazebo_bridge.py` PWM normalizasyonunu düzelt. (Tamamlandı: 1100/1500/1900 varsayılanları `-1/0/+1`; `SIM_GZ_PWM_MIN_US`, `SIM_GZ_PWM_TRIM_US`, `SIM_GZ_PWM_MAX_US` ile override edilebilir.)
    - 1100/1500/1900 için full forward/reverse `1.0/-1.0` olmalı.
    - Min/max/trim env ile override edilebilir olmalı.

20. [x] `SIM_GZ_YAW_SIGN` için statik self-check ekle. (Tamamlandı: bridge start'ta servo mapping, yaw sign, PWM normalizasyon ve bench bypass durumu JSONL self-check olarak loglanıyor.)
    - Bridge start'ta sign ve servo mapping'i loglasın.
    - Race-like simde bench bypass kapalı kalmalı.

21. [x] `sim/bin/run_sim_stack.sh` içinde `lidar_map.py` başlatmayı geri getir. (Tamamlandı: sıra telemetry -> lidar_map -> usv_main; race modda lidar_map kendi guardıyla stream açmadan çıkar.)
    - Sıra: telemetry -> lidar_map -> usv_main.
    - Race modda görüntü/map stream kapalı kalmalı; test modda dashboard log viewer'da görünmeli.

22. [x] Bare `sleep` readiness noktalarını health check ile değiştir. (Tamamlandı: Gazebo process/log, ROS-GZ `/scan` topic, pose JSON freshness ve camera TCP port probeları eklendi.)
    - Gazebo topic hazır mı?
    - ROS-GZ bridge topic akıyor mu?
    - sitl_gazebo_bridge pose dosyası güncel mi?
    - ros_to_tcp_cam portu veya headless camera output hazır mı?

23. [x] `check_stack.sh` kapsamını güncelle. (Tamamlandı: `lidar_map.py` ve core servis debug/jsonl log çiftleri kontrol ediliyor.)
    - Beklenen servisler arasında `lidar_map.py` tekrar olsun.
    - Her servis için debug log ve jsonl varlığı kontrol edilsin.

### Phase 6 — IPC ve Log Standardı

24. [x] `docker_workspace/src/json_atomic.py` dosyasını ekle. (Tamamlandı: `atomic_write_json()` / `atomic_read_json()` temp+fsync+replace API.)
25. [x] Shared JSON yazan bütün yerleri `json_atomic.py` kullanacak şekilde taşı. (Tamamlandı: `mission_state.json`, `camera_status.json`, `target_state.json`, `motor_command.json`, `vehicle_position.json`, mission upload.)
26. [x] Direct-write fallback davranışını kaldır veya güvenli hale getir. (Tamamlandı: atomic write başarısızsa önceki geçerli dosya korunur, hata loglanır.)
27. [x] `lidar_map.py` merkezi log init kullansın. (Tamamlandı: `lidar_map.debug.log` + `lidar_map.jsonl`; heartbeat, scan timeout/recovery, map update, race-disabled eventleri.)
28. [x] Dashboard log allowlist ile gerçek üretilen loglar birebir eşleşsin. (Tamamlandı: phantom alias kaldırıldı; `compliance_race_test.log` eklendi.)

### Phase 7 — Camera ve Lidar Kalibrasyon Riskleri

29. [x] HSV boundary overlap'lerini azalt. (Tamamlandı: red H≤4, orange 6–21, yellow 23–35; wrong-target confidence gating `compliance_profile.py` + `cam.py`.)
30. [x] Camera FOV varsayımlarını config'e taşı. (Tamamlandı: `resolve_camera_bearing_half_deg()` sim 45° / gerçek 35°, env override.)
31. [x] Lidar sektör açılarını profile/config içinde görünür yap. (Tamamlandı: `classify_lidar_scan_sector()` / `classify_lidar_map_sector()`, `LIDAR_BEARING_SIGN`.)
32. [x] `lidar_status.json` gerekip gerekmediğine karar ver. (Tamamlandı: kullanılmıyor — `LIDAR_STATUS_JSON_ENABLED=False`; lidar durumu `mission_state.json` üzerinden.)

### Phase 8 — Pixhawk Param ve Yarış Sertifikasyonu

33. [x] `config/pixhawk_ida.param` için baseline karşılaştırma script'i ekle. (Tamamlandı: `host_scripts/compare_pixhawk_param_baseline.py` + `config/ardurover_defaults.reference.param`.)
34. [x] Arming/failsafe risklerini ayrı race-cert dokümanına bağla. (Tamamlandı: `documents/race_cert_pixhawk_arming_failsafe.md`; Yol A param / Yol B fiziksel E-stop.)
35. [x] Servo mapping su üstü test planı yaz. (Tamamlandı: `documents/servo_mapping_water_test_plan.md`; OTOMASYON.md §2.2 bağlantıları.)

### Phase 9 — Test ve Doğrulama

36. [x] Unit/behavior testleri ekle. (Tamamlandı: `host_scripts/check_mission_profile_unit.py` — profile parsing, race gates, P2 min gate, P3 quorum, wrong-target block.)
37. [x] Compliance scriptlerini güncelle. (Tamamlandı: static 86/86, behavior 30/30, race 8/8; unit test subprocess + race profile gate test 9/10.)
38. [x] Her değişiklikten sonra zorunlu statik kontrolleri çalıştır. (Tamamlandı: `py_compile` + üç compliance betiği PASS.)
39. [x] Manuel operatör doğrulaması için ayrı checklist hazırla. (Tamamlandı: `documents/manual_operator_verification_checklist.md`; ajan otomatik görev başlatmaz.)
40. [x] Race readiness skorunu tekrar hesapla. (Tamamlandı: `host_scripts/check_race_readiness_score.py` → **10/10**, `race_deploy_recommended=true`; su testi + operatör checklist hâlâ zorunlu.)

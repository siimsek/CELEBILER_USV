# IDA Sim Diagnosis

Date: 2026-04-08

Reviewed first:
- `documents/ida_sartname.md`
- `host_scripts/check_compliance_static.py`
- `host_scripts/check_compliance_behavior.py`
- `old_source/docker_workspace/src/*.py`

Architecture summary:
- Gazebo world: `sim/worlds/water_world.sdf`
- Vehicle model: `sim/models/usv_model/model.sdf`
- SITL entry: `sim/bin/run_sitl.sh`
- Stack entry: `sim/bin/run_sim_stack.sh`
- Runtime bridge: `sim/bridges/sitl_gazebo_bridge.py`
- App/control nodes: `docker_workspace/src/usv_main.py`, `telemetry.py`, `lidar_map.py`, `cam.py`

Root causes found:
1. `usv_main.py` simulation path sent steer/throttle semantics on CH1/CH3 while the rest of the stack assumed differential left/right motor semantics.
2. Manual RC override detection in simulation re-read echoed RC override values and could preempt autonomy with its own injected commands.
3. LiDAR data itself was continuous, but telemetry UI refreshed a static JPG snapshot instead of binding the live MJPEG feed, creating a choppy perception of the scan.
4. Existing Gazebo actuation path used wheel-style / invalid motion assumptions (`DiffDrive` with dummy joints), so `/cmd_vel` did not produce reliable USV motion.
5. Auto-test motion check could report success from heading deltas alone without meaningful translation.

Implemented direction:
- Restore differential left/right motor command semantics in `usv_main.py`
- Ignore self-echoed RC overrides in simulation manual-override detection
- Replace invalid actuation chain with Gazebo `VelocityControl` on `base_link`
- Keep SITL servo output normalization with starboard reversal correction in bridge
- Switch telemetry LiDAR view to live stream proxy / fallback binding
- Tighten auto-test to require fresh pose updates and real translation

Validation:
- `python3 -m py_compile docker_workspace/src/*.py sim/bridges/*.py`
- `python3 host_scripts/check_compliance_static.py`
- `python3 host_scripts/check_compliance_behavior.py`
- `./sim/bin/run_sim_stack.sh --auto-test 45`
  Result: motion observed (`travel=1.446m`)

---

## Update: 2026-04-10

Reviewed first:
- `documents/ida_sartname.md`
- `documents/rapor_calismasistemi.md`
- `host_scripts/system_start.sh`
- `host_scripts/check_compliance_static.py`
- `host_scripts/check_compliance_behavior.py`
- `logs/system/lidar_map.debug.log`
- `logs/system/lidar_map.jsonl`
- `logs/system/usv_main.log`
- `logs/system/usv_main.jsonl`
- `logs/system/file3_local_map_index.csv`
- `logs/simulation/sitl_gazebo_bridge.debug.log`

Current root causes found from runtime logs:
1. `autonomy_health.lidar_point_count` can drop to `0` during transient invalid scans even while sector distances remain valid, so the dashboard flickers between nonzero and zero LiDAR health.
2. `lidar_map.py` redraws from only the current scan and clears the frame on empty-valid scans, so map view visually flickers despite continuous sensor flow.
3. Parkur-1 uses `front_min = min(left, center, right)` and reuses Parkur-2 avoidance logic, so a side-sector return around `2.8 m` keeps `in_warn=True` and hijacks waypoint tracking in an obstacle-free leg.
4. Mission start behavior was unstable because the differential turn sign in simulation was inverted and large heading corrections used reverse-producing pivot commands. In logs this showed as immediate sternward drift instead of rotating toward the waypoint.

Approved implementation scope:
- Hold last valid LiDAR samples briefly instead of dropping health/render to zero on a single bad scan.
- Keep Parkur-1 avoidance gated by center-sector hazard only; do not let side clutter override waypoint navigation.
- Keep mission RC override on direct skid channels (`CH1` left, `CH3` right) to match `telemetry.py` and `sitl_gazebo_bridge.py`.
- Invert the simulation turn sign and remove reverse-side bursts during large heading alignment by using a low forward bias instead of a pure reverse-capable pivot.

Follow-up diagnosis after freeze near waypoint:
1. Waypoint geometry in `sim/configs/mission_parkour_all.json` is internally consistent; freeze was not caused by a misplaced target. At the stuck pose the active target remained `P1/WP2`, about `2.6 m` away.
2. `usv_main.log` continued to emit non-neutral motor commands, but `sitl_gazebo_bridge.debug.log` showed SITL staying at `1500/1500`, so the breakage was between MAVLink autonomy commands and SITL JSON servo output.
3. Sim-only fallback was added: `usv_main.py` now mirrors its commanded left/right PWM into `sim/control/sim_motor_cmd.json`, and `sitl_gazebo_bridge.py` consumes that file only when SITL output is neutral, mission is active, and hold/e-stop/command-lock are all clear.
4. Validation after the patch showed bridge fallback activation and renewed forward progress (`vehicle_position.json` advanced from about `pos_y=0.43` to `pos_y=2.46` during the same mission) instead of freezing at neutral outputs.

Follow-up diagnosis after obstacle-avoidance retune:
1. Previous P1/P2 logic switched between pure waypoint tracking and pure escape heading, which created hard heading jumps and weak progress toward coordinates whenever LiDAR warning bands flickered.
2. `usv_main.py` was updated with a local planner that scores candidate headings against both target alignment and LiDAR corridor clearance, plus a path-based speed limiter tied to clearance and turn demand.
3. The planner now follows the target heading directly when the target corridor is clear, and only searches alternate headings when that corridor is actually constrained.
4. Validation in sim showed the boat continuing northward progress again (`vehicle_position.json` reached about `pos_y=3.68`) with lower mean absolute heading command than the first retune pass, although full parkur completion still needs more tuning against the current simulated obstacle field.

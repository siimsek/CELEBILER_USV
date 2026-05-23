# IDA Sim Diagnosis - Waypoint Heading Revision

Date: 2026-05-22

Approved implementation target:
- Make NAV waypoint guidance heading-first and deterministic.
- Preserve large waypoint bearing sign instead of clipping to small avoidance-biased values.
- Keep Pixhawk/ArduPilot as the low-level PWM owner; Pi sends GUIDED speed/yaw setpoints.
- Prevent persistent lidar/local-map residue from overriding a fresh waypoint turn unless current lidar confirms an immediate obstacle.
- Add diagnostics for heading phase, GUIDED N/E velocity components, and SITL servo-to-motor mapping.

Validation boundary:
- Agent may run static checks and Python validation.
- Manual driving/motion simulation remains operator-controlled.

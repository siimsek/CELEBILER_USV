"""
Pure guidance and control-allocation helpers for the twin-thruster USV.

The state machine keeps mission lifecycle, safety gates, and hardware I/O.
This module only turns already-validated sensor/nav inputs into stable
heading/speed requests and then maps those requests to left/right motor PWM.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from motor_controller import mix_twin_thrusters
from navigation import adaptive_l1_waypoint_request, clamp_heading_error, heading_first_waypoint_request


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


@dataclass(frozen=True)
class GuidanceDecision:
    speed_mps: float
    heading_error_deg: float
    mode: str
    reason: str
    avoidance_bias_deg: float = 0.0
    cross_track_error_m: float = 0.0
    nominal_heading_deg: float = 0.0
    gate_assist_bias_deg: float = 0.0
    limit_reason: str = "nominal"


@dataclass(frozen=True)
class MotorAllocation:
    left_pwm: int
    right_pwm: int
    surge_norm: float
    yaw_norm: float
    clipped: bool
    limit_reason: str


def _cross_track_from_heading(distance_m: float, heading_error_deg: float) -> float:
    # We do not have a full path segment here. This projects the current miss
    # against the target bearing into a lateral error that is still useful as a
    # live tuning diagnostic.
    return float(distance_m) * math.sin(math.radians(float(heading_error_deg)))


def _blend_with_avoidance(
    nominal_heading_error_deg: float,
    avoidance_bias_deg: float,
    obstacle_level: float,
    max_heading_error_deg: float = 95.0,
) -> float:
    nominal = float(nominal_heading_error_deg)
    avoidance = float(avoidance_bias_deg)
    level = clamp(float(obstacle_level), 0.0, 1.0)

    # Keep the mission heading alive while obstacle influence grows. This avoids
    # binary branch jumps that previously made the boat oscillate and lose the
    # waypoint/gate line after every transient obstacle blip.
    nominal_weight = 1.0 - (0.78 * level)
    combined = (nominal * nominal_weight) + avoidance
    return clamp(combined, -max_heading_error_deg, max_heading_error_deg)


def schedule_waypoint_speed(
    distance_m: float,
    cruise_speed_mps: float,
    approach_speed_mps: float,
    approach_window_m: float = 8.0,
) -> float:
    distance = max(0.0, float(distance_m))
    cruise = max(0.0, float(cruise_speed_mps))
    approach = max(0.0, float(approach_speed_mps))
    if distance >= float(approach_window_m):
        return cruise
    ratio = clamp(distance / max(float(approach_window_m), 0.1), 0.0, 1.0)
    return approach + ((cruise - approach) * ratio)


def compute_nav_decision(
    *,
    distance_m: float,
    gps_heading_error_deg: float,
    gate_detected: bool,
    gate_stable_s: float,
    gate_bearing_deg: float,
    base_speed_mps: float,
    avoidance_bias_deg: float,
    front_distance_m: float,
    center_distance_m: float,
    warn_distance_m: float,
    stop_distance_m: float,
    gate_stable_threshold_s: float,
    failsafe_slow_mps: float,
) -> GuidanceDecision:
    """
    Unified navigation decision for all waypoints.

    Waypoint coordinates are the primary objective. Gate vision applies a bounded
    assist around the waypoint heading. Lidar avoidance blends in only when a
    front threat is within the warn band; emergency stop distance keeps full avoid.
    """
    # Keep large waypoint-turn sign alive; final actuator shaping will cap speed/yaw.
    waypoint_heading = clamp_heading_error(float(gps_heading_error_deg))
    nominal_heading = waypoint_heading
    gate_assist_bias = 0.0
    mode = "nav_waypoint_track"
    reason = "gps_waypoint"
    limit_reason = "nominal"

    gate_ready = bool(gate_detected and gate_stable_s >= gate_stable_threshold_s)
    if gate_ready:
        gate_delta = clamp(float(gate_bearing_deg) - waypoint_heading, -35.0, 35.0)
        gate_assist_bias = clamp(gate_delta * 0.4, -12.0, 12.0)
        nominal_heading = clamp_heading_error(waypoint_heading + gate_assist_bias)
        mode = "nav_waypoint_track_gate_assist"
        reason = "waypoint_first_gate_assist"

    obstacle_level = 0.0
    if front_distance_m < warn_distance_m:
        obstacle_level = clamp(
            (warn_distance_m - front_distance_m) / max(warn_distance_m - stop_distance_m, 0.05),
            0.0,
            1.0,
        )

    waypoint_request = adaptive_l1_waypoint_request(
        distance_m=distance_m,
        heading_error_deg=nominal_heading,
        cruise_speed_mps=base_speed_mps,
        approach_speed_mps=min(float(base_speed_mps), float(failsafe_slow_mps)),
        current_speed_mps=base_speed_mps,
    )
    heading_error = clamp_heading_error(float(waypoint_request.heading_error_deg))
    speed = max(0.0, float(waypoint_request.speed_mps))
    if waypoint_request.phase == "TURN_TO_WAYPOINT":
        mode = "nav_turn_to_waypoint"
        reason = "heading_first"
        limit_reason = "heading_first"
    elif waypoint_request.phase == "WAYPOINT_REACHED":
        mode = "nav_waypoint_reached"
        reason = "acceptance_radius"
        limit_reason = "waypoint_reached"

    if front_distance_m < stop_distance_m:
        heading_error = clamp(float(avoidance_bias_deg), -95.0, 95.0)
        speed = min(speed, float(failsafe_slow_mps))
        mode = "nav_avoid"
        reason = "lidar_emergency"
        limit_reason = "failsafe_slow"
    elif front_distance_m < warn_distance_m:
        heading_error = _blend_with_avoidance(nominal_heading, avoidance_bias_deg, obstacle_level)
        speed = min(
            speed,
            max(float(failsafe_slow_mps), float(base_speed_mps) * (1.0 - (0.35 * obstacle_level))),
        )
        if gate_ready:
            mode = "nav_waypoint_track_gate_assist"
            reason = "waypoint_first_warn_blend"
        else:
            mode = "nav_waypoint_track"
            reason = "waypoint_first_warn_blend"
        limit_reason = "warn_blend"
    else:
        # No obstacle: bounded gate assist if available, otherwise pure waypoint bearing
        heading_error = float(nominal_heading)
        if gate_ready:
            speed = min(speed, max(float(failsafe_slow_mps), min(float(base_speed_mps), 0.9)))
            limit_reason = "gate_assist_cap"

    if center_distance_m < stop_distance_m and speed > float(failsafe_slow_mps):
        speed = float(failsafe_slow_mps)
        limit_reason = "center_sector_cap"

    return GuidanceDecision(
        speed_mps=float(speed),
        heading_error_deg=float(heading_error),
        mode=mode,
        reason=reason,
        avoidance_bias_deg=float(avoidance_bias_deg),
        cross_track_error_m=_cross_track_from_heading(distance_m, heading_error),
        nominal_heading_deg=float(nominal_heading),
        gate_assist_bias_deg=float(gate_assist_bias),
        limit_reason=limit_reason,
    )


# ---------------------------------------------------------------------------
# Deprecated shims — kept for backward compatibility. Use compute_nav_decision.
# ---------------------------------------------------------------------------

def compute_p1_decision(
    *,
    distance_m: float,
    gps_heading_error_deg: float,
    base_speed_mps: float,
    avoidance_bias_deg: float,
    center_distance_m: float,
    warn_distance_m: float,
    stop_distance_m: float,
    failsafe_slow_mps: float,
) -> GuidanceDecision:
    obstacle_level = 0.0
    if center_distance_m < warn_distance_m:
        obstacle_level = clamp(
            (warn_distance_m - center_distance_m) / max(warn_distance_m - stop_distance_m, 0.05),
            0.0,
            1.0,
        )

    nominal_heading = clamp(float(gps_heading_error_deg), -75.0, 75.0)
    heading_error = _blend_with_avoidance(nominal_heading, avoidance_bias_deg, obstacle_level)
    speed = max(0.0, float(base_speed_mps))
    mode = "p1_waypoint_los"
    reason = "los_nominal"
    limit_reason = "nominal"

    if center_distance_m < stop_distance_m:
        heading_error = clamp(float(avoidance_bias_deg), -95.0, 95.0)
        speed = min(speed, float(failsafe_slow_mps))
        mode = "p1_avoid"
        reason = "lidar_emergency"
        limit_reason = "failsafe_slow"
    elif center_distance_m < warn_distance_m:
        speed = min(speed, max(float(failsafe_slow_mps), float(base_speed_mps) * 0.85))
        mode = "p1_avoid"
        reason = "lidar_warning"
        limit_reason = "warn_speed_cap"

    return GuidanceDecision(
        speed_mps=float(speed),
        heading_error_deg=float(heading_error),
        mode=mode,
        reason=reason,
        avoidance_bias_deg=float(avoidance_bias_deg),
        cross_track_error_m=_cross_track_from_heading(distance_m, heading_error),
        nominal_heading_deg=float(nominal_heading),
        gate_assist_bias_deg=0.0,
        limit_reason=limit_reason,
    )


def compute_p2_decision(
    *,
    distance_m: float,
    gps_heading_error_deg: float,
    gate_detected: bool,
    gate_stable_s: float,
    gate_bearing_deg: float,
    base_speed_mps: float,
    avoidance_bias_deg: float,
    front_distance_m: float,
    center_distance_m: float,
    warn_distance_m: float,
    stop_distance_m: float,
    gate_stable_threshold_s: float,
    failsafe_slow_mps: float,
) -> GuidanceDecision:
    gate_ready = bool(gate_detected and gate_stable_s >= gate_stable_threshold_s)
    waypoint_heading = clamp(float(gps_heading_error_deg), -80.0, 80.0)
    gate_assist_bias = 0.0
    nominal_heading = waypoint_heading
    mode = "p2_waypoint_track"
    reason = "gps_waypoint"
    limit_reason = "nominal"

    if gate_ready:
        # Waypoint-first policy: the next mission coordinate remains the primary
        # objective. Gate vision only nudges that objective so the boat stays in
        # the buoy corridor without abandoning the leg to chase the image center.
        gate_delta = clamp(float(gate_bearing_deg) - waypoint_heading, -35.0, 35.0)
        gate_assist_bias = clamp(gate_delta * 0.4, -12.0, 12.0)
        nominal_heading = clamp(waypoint_heading + gate_assist_bias, -80.0, 80.0)
        mode = "p2_waypoint_track_gate_assist"
        reason = "waypoint_first_gate_assist"

    obstacle_level = 0.0
    if front_distance_m < warn_distance_m:
        obstacle_level = clamp(
            (warn_distance_m - front_distance_m) / max(warn_distance_m - stop_distance_m, 0.05),
            0.0,
            1.0,
        )

    heading_error = clamp(nominal_heading, -85.0, 85.0)
    speed = max(0.0, float(base_speed_mps))

    # P2 heading priority: obstacle > gate > waypoint, but blend instead of binary cutoff
    if front_distance_m < stop_distance_m:
        # Emergency: 100% avoidance
        heading_error = clamp(float(avoidance_bias_deg), -95.0, 95.0)
        speed = min(speed, float(failsafe_slow_mps))
        mode = "p2_avoid"
        reason = "lidar_emergency"
        limit_reason = "failsafe_slow"
    elif front_distance_m < warn_distance_m:
        # Warning: 60% avoidance, 40% navigation (waypoint or gate)
        heading_error = _blend_with_avoidance(nominal_heading, avoidance_bias_deg, obstacle_level)
        speed = min(speed, max(float(failsafe_slow_mps), float(base_speed_mps) * 0.85))
        mode = "p2_avoid"
        reason = "lidar_warning"
        limit_reason = "warn_speed_cap"
    elif gate_ready:
        # No obstacle: gate bearing preferred
        # "Preferred" here means bounded assist around waypoint heading, not
        # replacing the mission leg. This keeps the boat aimed at the next
        # waypoint while letting the camera recentre it inside the corridor.
        heading_error = clamp(float(nominal_heading), -80.0, 80.0)
        speed = min(speed, max(float(failsafe_slow_mps), min(float(base_speed_mps), 0.9)))
        limit_reason = "gate_assist_cap"
    else:
        # No obstacle, no gate: pure waypoint bearing
        heading_error = float(waypoint_heading)

    if center_distance_m < stop_distance_m and speed > float(failsafe_slow_mps):
        speed = float(failsafe_slow_mps)
        limit_reason = "center_sector_cap"

    return GuidanceDecision(
        speed_mps=float(speed),
        heading_error_deg=float(heading_error),
        mode=mode,
        reason=reason,
        avoidance_bias_deg=float(avoidance_bias_deg),
        cross_track_error_m=_cross_track_from_heading(distance_m, heading_error),
        nominal_heading_deg=float(nominal_heading),
        gate_assist_bias_deg=float(gate_assist_bias),
        limit_reason=limit_reason,
    )


def allocate_twin_thrusters(
    *,
    speed_mps: float,
    heading_error_deg: float,
    neutral_pwm: int = 1500,
    pwm_span: int = 400,
    max_speed_mps: float = 1.5,
) -> MotorAllocation:
    mix = mix_twin_thrusters(
        speed_mps=speed_mps,
        heading_error_deg=heading_error_deg,
        neutral_pwm=neutral_pwm,
        pwm_span=pwm_span,
        max_speed_mps=max_speed_mps,
    )
    return MotorAllocation(
        left_pwm=int(mix.left_pwm),
        right_pwm=int(mix.right_pwm),
        surge_norm=float(mix.surge_norm),
        yaw_norm=float(mix.yaw_norm),
        clipped=bool(mix.clipped),
        limit_reason=str(mix.limit_reason),
    )

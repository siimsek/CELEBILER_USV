"""
Pure guidance and control-allocation helpers for the twin-thruster USV.

The state machine keeps mission lifecycle, safety gates, and hardware I/O.
This module only turns already-validated sensor/nav inputs into stable
heading/speed requests and then maps those requests to left/right motor PWM.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


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

    Waypoint coordinates are always the nominal target. Vision/gate inputs are
    retained for compatibility but do not steer the boat in NAV.
    """
    # ±85: WP bearing + cross-track (Stanley) birlikte 80'i asabiliyor; ±80 kesinti lateral düzeltmeyi doyurganlıkta yiyordu
    waypoint_heading = clamp(float(gps_heading_error_deg), -85.0, 85.0)
    nominal_heading = waypoint_heading
    mode = "nav_waypoint_track"
    reason = "gps_waypoint"
    limit_reason = "nominal"

    obstacle_level = 0.0
    if front_distance_m < warn_distance_m:
        obstacle_level = clamp(
            (warn_distance_m - front_distance_m) / max(warn_distance_m - stop_distance_m, 0.05),
            0.0,
            1.0,
        )

    heading_error = clamp(nominal_heading, -85.0, 85.0)
    speed = max(0.0, float(base_speed_mps))

    if front_distance_m < stop_distance_m:
        # Emergency: 100% avoidance
        heading_error = clamp(float(avoidance_bias_deg), -95.0, 95.0)
        speed = min(speed, float(failsafe_slow_mps))
        mode = "nav_avoid"
        reason = "lidar_emergency"
        limit_reason = "failsafe_slow"
    elif front_distance_m < warn_distance_m:
        # Warning: çok yakında görev bearing'i neredeyse kapat — dubanın etrafından dönüş
        if obstacle_level >= 0.88:
            heading_error = clamp(float(avoidance_bias_deg), -95.0, 95.0)
        else:
            heading_error = _blend_with_avoidance(nominal_heading, avoidance_bias_deg, obstacle_level)
        speed = min(speed, max(float(failsafe_slow_mps), float(base_speed_mps) * 0.85))
        mode = "nav_avoid"
        reason = "lidar_warning"
        limit_reason = "warn_speed_cap"
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
        gate_assist_bias_deg=0.0,
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
    speed = float(speed_mps)
    heading_error = float(heading_error_deg)
    surge_norm = clamp(speed / max(float(max_speed_mps), 0.1), -1.0, 1.0)
    # /72 (eskiden /80): orta heading hatalarında diferansiyel dönüş biraz güçlenir (WP1’de önce buruna dönüş)
    yaw_norm = clamp(heading_error / 72.0, -1.0, 1.0)
    heading_abs = abs(heading_error)
    limit_reason = "nominal"

    # As turn demand grows we reserve more authority for yaw so the inner motor
    # does not get starved by a too-large forward command.
    if speed > 0.0:
        if heading_abs >= 75.0:
            surge_norm = min(surge_norm, 0.32)
            limit_reason = "hard_turn_speed_cap"
        elif heading_abs >= 50.0:
            surge_norm = min(surge_norm, 0.48)
            limit_reason = "medium_turn_speed_cap"
        elif heading_abs >= 25.0:
            surge_norm = min(surge_norm, 0.68)
            limit_reason = "soft_turn_speed_cap"
        elif heading_abs >= 10.0:
            # 10–25°: önceki davranışta ileri güdüm yüksek, yaw zayıf kalıyordu → “düz gidip geç dönüyor”
            surge_norm = min(surge_norm, 0.52)
            limit_reason = "align_turn_speed_cap"
        if 0.0 < surge_norm < 0.14:
            surge_norm = 0.14

    yaw_gain = 0.92 - (0.22 * abs(surge_norm))
    left_mix = surge_norm - (yaw_norm * yaw_gain)
    right_mix = surge_norm + (yaw_norm * yaw_gain)

    clipped = False
    max_mag = max(abs(left_mix), abs(right_mix), 1.0)
    if max_mag > 1.0:
        left_mix /= max_mag
        right_mix /= max_mag
        clipped = True
        limit_reason = "mix_normalized"

    left_pwm = int(round(int(neutral_pwm) + (left_mix * int(pwm_span))))
    right_pwm = int(round(int(neutral_pwm) + (right_mix * int(pwm_span))))
    left_pwm = int(clamp(left_pwm, 1100, 1900))
    right_pwm = int(clamp(right_pwm, 1100, 1900))
    # Narrow deadband so small heading corrections still produce differential thrust (WP1 approach).
    _pwm_dead = 8
    if abs(left_pwm - int(neutral_pwm)) < _pwm_dead:
        left_pwm = int(neutral_pwm)
    if abs(right_pwm - int(neutral_pwm)) < _pwm_dead:
        right_pwm = int(neutral_pwm)

    return MotorAllocation(
        left_pwm=left_pwm,
        right_pwm=right_pwm,
        surge_norm=float(surge_norm),
        yaw_norm=float(yaw_norm),
        clipped=bool(clipped),
        limit_reason=limit_reason,
    )

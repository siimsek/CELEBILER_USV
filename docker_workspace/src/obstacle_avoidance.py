"""
Small three-sector obstacle avoidance helper.

Lidar remains the primary safety sensor. Camera detections may bias guidance in
the caller, but they never make a blocked lidar sector safe.
"""

from __future__ import annotations

from dataclasses import dataclass

from compliance_profile import (
    OBSTACLE_TTC_DANGER_S,
    OBSTACLE_TTC_EMERGENCY_S,
    OBSTACLE_TTC_STOP_S,
    OBSTACLE_TTC_WARN_S,
)


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


@dataclass(frozen=True)
class AvoidanceDecision:
    active: bool
    state: str
    yaw_bias_deg: float
    speed_limit_mps: float | None
    escape_side: str
    reason: str


@dataclass(frozen=True)
class TTCSpeedProfile:
    active: bool
    level: str
    speed_factor: float
    speed_limit_mps: float | None
    reason: str


def speed_profile_for_ttc(base_speed_mps: float, min_ttc_s: float | None) -> TTCSpeedProfile:
    """Return conservative speed shaping for a dynamic obstacle TTC estimate."""
    try:
        ttc = float(min_ttc_s)
    except (TypeError, ValueError):
        ttc = 9999.0
    base = max(0.0, float(base_speed_mps))
    if ttc <= float(OBSTACLE_TTC_STOP_S):
        return TTCSpeedProfile(True, "stop", 0.0, 0.0, "ttc_stop")
    if ttc <= float(OBSTACLE_TTC_EMERGENCY_S):
        return TTCSpeedProfile(True, "emergency", 0.0, 0.0, "ttc_emergency")
    if ttc <= float(OBSTACLE_TTC_DANGER_S):
        limit = base * 0.25
        return TTCSpeedProfile(True, "danger", 0.25, limit, "ttc_danger")
    if ttc <= float(OBSTACLE_TTC_WARN_S):
        limit = base * 0.50
        return TTCSpeedProfile(True, "warn", 0.50, limit, "ttc_warn")
    return TTCSpeedProfile(False, "normal", 1.0, None, "ttc_clear")


def _finite_or_clear(value: float | None) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return 99.0
    if parsed <= 0.0:
        return 99.0
    return parsed


def decide_three_sector_avoidance(
    *,
    left_m: float | None,
    front_m: float | None,
    right_m: float | None,
    stop_distance_m: float,
    warn_distance_m: float,
    clear_hysteresis_m: float,
    previous_escape_side: str | None = None,
    failsafe_slow_mps: float = 0.3,
    max_yaw_bias_deg: float = 48.0,
) -> AvoidanceDecision:
    left = _finite_or_clear(left_m)
    front = _finite_or_clear(front_m)
    right = _finite_or_clear(right_m)
    stop = max(0.05, float(stop_distance_m))
    warn = max(stop + 0.05, float(warn_distance_m))
    clear = warn + max(0.0, float(clear_hysteresis_m))

    if front >= clear:
        return AvoidanceDecision(False, "clear", 0.0, None, "none", "front_clear")

    prev = str(previous_escape_side or "").lower()
    if prev in ("left", "right") and front < clear:
        escape_side = prev
    else:
        escape_side = "left" if left >= right else "right"

    side_sign = -1.0 if escape_side == "left" else 1.0
    if front <= stop:
        level = 1.0
        state = "blocked"
        speed_limit = 0.0
        reason = "front_stop"
    else:
        level = clamp((warn - front) / max(warn - stop, 0.05), 0.0, 1.0)
        state = "avoid"
        speed_limit = float(failsafe_slow_mps)
        reason = "front_warn"

    yaw_bias = side_sign * (18.0 + ((float(max_yaw_bias_deg) - 18.0) * level))
    return AvoidanceDecision(
        active=True,
        state=state,
        yaw_bias_deg=float(yaw_bias),
        speed_limit_mps=float(speed_limit),
        escape_side=escape_side,
        reason=reason,
    )

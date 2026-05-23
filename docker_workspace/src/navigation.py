"""
Minimal waypoint navigation helpers.

These functions are pure: no MAVLink, no files, no mission lifecycle. They turn
validated waypoint geometry into a heading-first speed/phase request.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from compliance_profile import (
    NAV_ALIGN_HEADING_DONE_DEG,
    NAV_ALIGN_CREEP_SPEED_MPS,
    NAV_HEADING_MAX_ERROR_DEG,
    NAV_STRICT_HEADING_FIRST,
    R_WP_M,
)


EARTH_RADIUS_M = 6_371_000.0


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def wrap_180(deg: float) -> float:
    return ((float(deg) + 180.0) % 360.0) - 180.0


def clamp_heading_error(deg: float, max_abs_deg: float = NAV_HEADING_MAX_ERROR_DEG) -> float:
    """Wrap heading error and preserve large waypoint-turn sign until the final command cap."""
    limit = max(1.0, min(179.0, abs(float(max_abs_deg))))
    return clamp(wrap_180(deg), -limit, limit)


def compass_heading_to_ne_velocity(speed_mps: float, heading_deg: float) -> tuple[float, float]:
    """Convert compass heading (0=N, 90=E) into MAVLink global N/E velocity components."""
    speed = max(0.0, float(speed_mps))
    heading_rad = math.radians(float(heading_deg) % 360.0)
    return float(speed * math.cos(heading_rad)), float(speed * math.sin(heading_rad))


def distance_bearing_m(
    current_lat: float,
    current_lon: float,
    target_lat: float,
    target_lon: float,
) -> tuple[float, float]:
    lat1 = math.radians(float(current_lat))
    lat2 = math.radians(float(target_lat))
    d_lat = math.radians(float(target_lat) - float(current_lat))
    d_lon = math.radians(float(target_lon) - float(current_lon))

    a = (
        math.sin(d_lat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * (math.sin(d_lon / 2.0) ** 2)
    )
    distance = 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))

    y = math.sin(d_lon) * math.cos(lat2)
    x = (math.cos(lat1) * math.sin(lat2)) - (
        math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
    )
    bearing = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    return float(distance), float(bearing)


@dataclass(frozen=True)
class WaypointRequest:
    speed_mps: float
    heading_error_deg: float
    phase: str
    reached: bool
    reason: str


def heading_first_waypoint_request(
    *,
    distance_m: float,
    heading_error_deg: float,
    cruise_speed_mps: float,
    approach_speed_mps: float,
    acceptance_radius_m: float = R_WP_M,
    slow_down_radius_m: float = 6.0,
    heading_threshold_deg: float = NAV_ALIGN_HEADING_DONE_DEG,
    creep_speed_mps: float = NAV_ALIGN_CREEP_SPEED_MPS,
) -> WaypointRequest:
    distance = max(0.0, float(distance_m))
    heading_error = clamp_heading_error(heading_error_deg)
    heading_abs = abs(heading_error)

    if distance <= float(acceptance_radius_m):
        return WaypointRequest(
            speed_mps=0.0,
            heading_error_deg=heading_error,
            phase="WAYPOINT_REACHED",
            reached=True,
            reason="acceptance_radius",
        )

    if heading_abs > float(heading_threshold_deg):
        if bool(NAV_STRICT_HEADING_FIRST):
            creep = max(0.0, min(float(creep_speed_mps), float(approach_speed_mps) * 0.5))
            if creep <= 0.0:
                return WaypointRequest(
                    speed_mps=0.0,
                    heading_error_deg=heading_error,
                    phase="TURN_TO_WAYPOINT",
                    reached=False,
                    reason="heading_first_yaw_only",
                )
        else:
            creep = max(0.0, min(float(creep_speed_mps), float(approach_speed_mps) * 0.5))
        return WaypointRequest(
            speed_mps=float(creep),
            heading_error_deg=heading_error,
            phase="TURN_TO_WAYPOINT",
            reached=False,
            reason="heading_first_creep",
        )

    slow_radius = max(float(slow_down_radius_m), float(acceptance_radius_m) + 0.1)
    cruise = max(0.0, float(cruise_speed_mps))
    approach = max(0.0, float(approach_speed_mps))
    if distance >= slow_radius:
        speed = cruise
    else:
        ratio = clamp(
            (distance - float(acceptance_radius_m)) / max(slow_radius - float(acceptance_radius_m), 0.1),
            0.0,
            1.0,
        )
        speed = approach + ((cruise - approach) * ratio)

    return WaypointRequest(
        speed_mps=float(speed),
        heading_error_deg=heading_error,
        phase="GO_TO_WAYPOINT",
        reached=False,
        reason="heading_aligned",
    )

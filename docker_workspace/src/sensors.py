"""
Shared sensor snapshot contracts and freshness guards.

Adapters should normalize sim and real inputs into these shapes before the
state machine consumes them.
"""

from __future__ import annotations

from dataclasses import dataclass
import time


@dataclass(frozen=True)
class NavSnapshot:
    lat: float
    lon: float
    heading_deg: float
    ground_speed_mps: float
    valid: bool
    source: str
    age_s: float


@dataclass(frozen=True)
class LidarSnapshot:
    left_m: float
    front_m: float
    right_m: float
    ready: bool
    quality_ready: bool
    point_count: int
    age_s: float
    drop_reason: str = "ok"


@dataclass(frozen=True)
class CameraSnapshot:
    ready: bool
    age_s: float
    target_detected: bool = False
    target_bearing_deg: float = 0.0
    gate_detected: bool = False
    gate_bearing_deg: float = 0.0


@dataclass(frozen=True)
class SensorSnapshot:
    nav: NavSnapshot
    lidar: LidarSnapshot
    camera: CameraSnapshot
    rc_link_active: bool
    estop_safe: bool
    mavlink_active: bool
    captured_monotonic: float


def is_fresh(age_s: float | None, timeout_s: float) -> bool:
    try:
        age = float(age_s)
    except (TypeError, ValueError):
        return False
    return 0.0 <= age <= float(timeout_s)


def monotonic_age(last_seen_monotonic: float | None) -> float:
    try:
        last_seen = float(last_seen_monotonic)
    except (TypeError, ValueError):
        return 999.0
    if last_seen <= 0.0:
        return 999.0
    return max(0.0, time.monotonic() - last_seen)

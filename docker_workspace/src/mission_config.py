"""
Mission coordinate schema and target-state helpers.

Unified mission contract:
- Waypoints are a flat ordered list: [[lat, lon], ...]
- split_nav_engage() separates nav waypoints from an optional explicit engage waypoint.
- Flat list defaults to NAV-only unless engage is explicitly declared.
- Target color is loaded separately from target_state.json.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any


MISSION_MIN_WAYPOINTS = 1  # At minimum: 1 nav waypoint
TARGET_STATE_FILE = os.environ.get("TARGET_STATE_FILE", f"{os.environ.get('CONTROL_DIR', '')}/target_state.json")

VALID_TARGET_COLORS = (
    "RED",
    "GREEN",
    "BLACK",
    "KIRMIZI_SANCAK",
    "YESIL_SANCAK",
    "SIYAH_HEDEF",
)

TARGET_COLOR_ALIASES = {
    "RED": "RED",
    "KIRMIZI": "RED",
    "KIRMIZI_SANCAK": "KIRMIZI_SANCAK",
    "GREEN": "GREEN",
    "YESIL": "GREEN",
    "YESIL_SANCAK": "YESIL_SANCAK",
    "BLACK": "BLACK",
    "SIYAH": "BLACK",
    "SIYAH_HEDEF": "SIYAH_HEDEF",
}


def normalize_target_color(raw: Any) -> str:
    token = str(raw or "").strip().upper()
    return TARGET_COLOR_ALIASES.get(token, token)


def validate_target_color(raw: Any) -> str:
    color = normalize_target_color(raw)
    if color not in VALID_TARGET_COLORS:
        raise ValueError(f"target_color '{raw}' must be one of {list(VALID_TARGET_COLORS)}")
    return color


def validate_coordinate_mission(data: Any) -> list[list[float]]:
    if not isinstance(data, list):
        raise ValueError("Mission must be a flat ordered JSON array of [lat, lon] pairs")

    coords: list[list[float]] = []
    for idx, wp in enumerate(data):
        coords.append(validate_coordinate_pair(wp, idx))

    if len(coords) < MISSION_MIN_WAYPOINTS:
        raise ValueError(
            f"Mission must have at least {MISSION_MIN_WAYPOINTS} waypoints "
            f"(nav waypoints); got {len(coords)}"
        )
    return coords


def validate_coordinate_pair(wp: Any, idx: int | str = "?") -> list[float]:
    if not isinstance(wp, list) or len(wp) != 2:
        raise ValueError(f"mission[{idx}] must be [lat, lon], got {wp}")
    lat, lon = wp
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        raise ValueError(f"mission[{idx}] coordinates must be numbers")
    if not (-90 <= lat <= 90):
        raise ValueError(f"mission[{idx}] latitude {lat} out of range [-90,90]")
    if not (-180 <= lon <= 180):
        raise ValueError(f"mission[{idx}] longitude {lon} out of range [-180,180]")
    return [float(lat), float(lon)]


def split_nav_engage(
    coords: list[list[float]],
    *,
    has_explicit_engage: bool = False,
) -> tuple[list[list[float]], list[float] | None]:
    """
    Split flat waypoint list into nav waypoints and engage (target) waypoint.

    Contract:
    - Default (flat mission): all coordinates are nav waypoints.
    - Explicit engage (structured mission): last coordinate is engage waypoint.
    - If empty: both empty/None
    """
    if len(coords) == 0:
        return [], None
    if not bool(has_explicit_engage):
        return list(coords), None
    if len(coords) == 1:
        return [], coords[0]
    return list(coords[:-1]), coords[-1]


def load_coordinate_mission_file(path: str) -> list[list[float]]:
    with open(path, "r", encoding="utf-8") as handle:
        return validate_coordinate_mission(json.load(handle))


def load_target_state(path: str | None = None) -> dict[str, Any]:
    from compliance_profile import CONTROL_DIR
    target_path = path or os.environ.get("TARGET_STATE_FILE", f"{CONTROL_DIR}/target_state.json")
    try:
        with open(target_path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except FileNotFoundError:
        return {}
    except (json.JSONDecodeError, OSError):
        return {}
    if not isinstance(payload, dict):
        return {}
    color = payload.get("target_color", "")
    if color:
        try:
            payload["target_color"] = validate_target_color(color)
        except ValueError:
            payload["target_color"] = ""
    return payload


def default_sim_mission(base_lat: float, base_lon: float) -> list[list[float]]:
    """
    Default simulation mission: flat NAV waypoint list.
    Explicit engage is optional and must come from structured payload.
    """
    meters_to_lat = 1.0 / 111320.0
    cos_lat = math.cos(math.radians(base_lat))
    meters_to_lon = 1.0 / (111320.0 * cos_lat) if abs(cos_lat) > 1e-6 else meters_to_lat

    def xy_to_gps(world_x_m: float, world_y_m: float) -> list[float]:
        return [
            round(base_lat + world_y_m * meters_to_lat, 7),
            round(base_lon + world_x_m * meters_to_lon, 7),
        ]

    return [
        xy_to_gps(2.5, 3.5),
        xy_to_gps(-2.5, 7.0),
        xy_to_gps(2.5, 10.5),
        xy_to_gps(0.0, 14.0),
        xy_to_gps(0.0, 27.0),
        xy_to_gps(-3.0, 29.8),
    ]


# ---------------------------------------------------------------------------
# Legacy compatibility shims — kept for backward compatibility with
# any external scripts that import these names. Do not use in new code.
# ---------------------------------------------------------------------------

def get_mission_split_profile(total_count: int) -> dict[str, Any]:
    """Deprecated: unified mission does not split by fixed counts."""
    nav_count = max(0, int(total_count))
    engage_count = 0
    return {
        "input_format": "flat_ordered",
        "allow_structured_legacy": False,
        "nav_count": nav_count,
        "engage_count": engage_count,
        "total_count": int(total_count),
        # legacy keys kept for dashboard/telemetry backward compat
        "p1_count": nav_count,
        "p2_count": 0,
        "p3_count": engage_count,
    }


def split_mission_waypoints(
    coords: list[list[float]],
    *,
    validate_lengths: bool = True,
) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
    """
    Deprecated shim: returns (nav_waypoints, [], [engage_wp]).

    Kept for backward compatibility with telemetry.py and mission_adapter.py
    that still reference split_mission_waypoints. New code should call
    split_nav_engage() directly.
    """
    nav, engage = split_nav_engage(coords)
    engage_list = [engage] if engage is not None else []
    return nav, [], engage_list


# Legacy constants referenced by old mission_adapter imports
MISSION_P1_MIN_WAYPOINTS = 1
MISSION_P2_MIN_WAYPOINTS = 0
MISSION_P3_MIN_WAYPOINTS = 1

"""
Mission coordinate schema, explicit parkur split profile, and target-state helpers.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any

from compliance_profile import (
    CONTROL_DIR,
    MISSION_ALLOW_STRUCTURED_LEGACY,
    MISSION_INPUT_FORMAT,
    MISSION_SPLIT_P2_COUNT,
    MISSION_SPLIT_P3_COUNT,
)


MISSION_P1_MIN_WAYPOINTS = 1
MISSION_P2_MIN_WAYPOINTS = 1
MISSION_P3_MIN_WAYPOINTS = 1
TARGET_STATE_FILE = os.environ.get("TARGET_STATE_FILE", f"{CONTROL_DIR}/target_state.json")

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

    split = get_mission_split_profile(len(coords))
    if split["p1_count"] < MISSION_P1_MIN_WAYPOINTS:
        raise ValueError(
            "Mission flat_ordered list is too short for configured split profile "
            f"(need at least {MISSION_P1_MIN_WAYPOINTS + MISSION_SPLIT_P2_COUNT + MISSION_SPLIT_P3_COUNT} waypoints: "
            f"P1>=1, P2={MISSION_SPLIT_P2_COUNT}, P3={MISSION_SPLIT_P3_COUNT}; got {len(coords)})"
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


def get_mission_split_profile(total_count: int) -> dict[str, Any]:
    p1_count = max(0, int(total_count) - MISSION_SPLIT_P2_COUNT - MISSION_SPLIT_P3_COUNT)
    return {
        "input_format": MISSION_INPUT_FORMAT,
        "allow_structured_legacy": bool(MISSION_ALLOW_STRUCTURED_LEGACY),
        "p1_min_count": MISSION_P1_MIN_WAYPOINTS,
        "p2_count": MISSION_SPLIT_P2_COUNT,
        "p3_count": MISSION_SPLIT_P3_COUNT,
        "p1_count": p1_count,
        "total_count": int(total_count),
    }


def split_mission_waypoints(coords: list[list[float]], *, validate_lengths: bool = True) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
    if validate_lengths:
        coords = validate_coordinate_mission(coords)
    total = len(coords)
    p1_end = total - MISSION_SPLIT_P2_COUNT - MISSION_SPLIT_P3_COUNT
    p2_end = total - MISSION_SPLIT_P3_COUNT
    p1 = coords[:p1_end]
    p2 = coords[p1_end:p2_end]
    p3 = coords[p2_end:]
    return p1, p2, p3


def load_coordinate_mission_file(path: str) -> list[list[float]]:
    with open(path, "r", encoding="utf-8") as handle:
        return validate_coordinate_mission(json.load(handle))


def load_target_state(path: str | None = None) -> dict[str, Any]:
    target_path = path or TARGET_STATE_FILE
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
    meters_to_lat = 1.0 / 111111.0
    cos_lat = math.cos(math.radians(base_lat))
    meters_to_lon = 1.0 / (111111.0 * cos_lat) if abs(cos_lat) > 1e-6 else meters_to_lat

    def xy_to_gps(world_x_m: float, world_y_m: float) -> list[float]:
        return [
            round(base_lat + world_y_m * meters_to_lat, 7),
            round(base_lon - world_x_m * meters_to_lon, 7),
        ]

    return [
        xy_to_gps(2.5, 3.5),
        xy_to_gps(-2.5, 7.0),
        xy_to_gps(2.5, 10.5),
        xy_to_gps(0.0, 14.0),
        xy_to_gps(0.0, 27.0),
        xy_to_gps(-3.0, 29.8),
    ]

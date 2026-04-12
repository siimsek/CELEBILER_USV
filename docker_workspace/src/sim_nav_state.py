"""
Shared simulation navigation state helpers.

Simulation adapters should read the same local pose source and convert it to
global coordinates consistently so mission logic and telemetry do not diverge.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Dict

from compliance_profile import CONTROL_DIR as DEFAULT_CONTROL_DIR


DEFAULT_SIM_HOME = (-35.363262, 149.165237, 584.0, 0.0)
DEFAULT_STALE_AFTER_S = 1.5


def parse_sim_home(raw: str | None = None) -> tuple[float, float, float, float]:
    text = (raw if raw is not None else os.environ.get("SIM_HOME", "")).strip()
    if not text:
        return DEFAULT_SIM_HOME
    try:
        parts = [token.strip() for token in text.split(",")]
        if len(parts) < 2:
            return DEFAULT_SIM_HOME
        lat = float(parts[0])
        lon = float(parts[1])
        alt = float(parts[2]) if len(parts) >= 3 else DEFAULT_SIM_HOME[2]
        heading = float(parts[3]) if len(parts) >= 4 else DEFAULT_SIM_HOME[3]
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return DEFAULT_SIM_HOME
        return lat, lon, alt, heading
    except (TypeError, ValueError):
        return DEFAULT_SIM_HOME


def local_xy_to_global(home_lat: float, home_lon: float, pos_x: float, pos_y: float) -> tuple[float, float]:
    cos_home = math.cos(math.radians(home_lat))
    meters_to_lon = 1.0 / (111320.0 * cos_home) if abs(cos_home) > 1e-6 else 1.0 / 111320.0
    lat = home_lat + (pos_y / 111320.0)
    lon = home_lon + (pos_x * meters_to_lon)
    return lat, lon


def _base_result(path: str) -> Dict[str, Any]:
    return {
        "path": path,
        "available": False,
        "valid": False,
        "fresh": False,
        "position_source": "none",
        "heading_source": "none",
        "reason": "unavailable",
        "source": "--",
        "lat": None,
        "lon": None,
        "heading_deg": None,
        "state_age_s": None,
        "ts_epoch": 0.0,
    }


def load_sim_nav_state(
    *,
    control_dir: str | None = None,
    stale_after_s: float = DEFAULT_STALE_AFTER_S,
) -> Dict[str, Any]:
    base_dir = control_dir or os.environ.get("CONTROL_DIR", DEFAULT_CONTROL_DIR)
    path = os.path.join(base_dir, "vehicle_position.json")
    result = _base_result(path)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except FileNotFoundError:
        result["reason"] = "file_missing"
        return result
    except json.JSONDecodeError:
        result["reason"] = "json_invalid"
        return result
    except Exception as exc:
        result["reason"] = f"read_error:{type(exc).__name__}"
        return result

    result["available"] = True
    result["source"] = str(payload.get("source", "--") or "--")

    try:
        pos_x = float(payload.get("pos_x", 0.0) or 0.0)
        pos_y = float(payload.get("pos_y", 0.0) or 0.0)
        heading_rad = float(payload.get("heading_rad", 0.0) or 0.0)
    except (TypeError, ValueError):
        result["reason"] = "pose_invalid"
        return result

    ts_epoch = float(payload.get("ts", payload.get("timestamp", 0.0)) or 0.0)
    age_s = max(0.0, time.time() - ts_epoch) if ts_epoch > 0.0 else None
    result["ts_epoch"] = ts_epoch
    result["state_age_s"] = age_s
    result["fresh"] = bool(age_s is not None and age_s <= float(stale_after_s))

    home_lat, home_lon, _, _ = parse_sim_home()
    lat, lon = local_xy_to_global(home_lat, home_lon, pos_x, pos_y)
    heading_deg = (math.degrees(heading_rad) + 360.0) % 360.0

    position_valid = (
        -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
        and (abs(lat) > 1e-6 or abs(lon) > 1e-6)
    )
    heading_valid = 0.0 <= heading_deg < 360.0

    result["lat"] = lat
    result["lon"] = lon
    result["heading_deg"] = heading_deg
    result["position_source"] = "sim_vehicle_position"
    result["heading_source"] = "sim_vehicle_position"
    result["valid"] = bool(position_valid and heading_valid and result["fresh"])
    if not position_valid:
        result["reason"] = "geo_invalid"
    elif not heading_valid:
        result["reason"] = "heading_invalid"
    elif not result["fresh"]:
        result["reason"] = "stale_pose"
    else:
        result["reason"] = "ok"
    return result

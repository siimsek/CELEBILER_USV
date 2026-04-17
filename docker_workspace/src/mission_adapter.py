"""
Mission format adapter: unified flat-ordered mission format.

Operational contract:
- Flat ordered list: [[lat, lon], ...]
- Last coordinate = engage waypoint (target duba)
- All others = nav waypoints (obstacle avoidance + gate navigation)

Legacy structured format (parkur1/parkur2/parkur3 dict) accepted only when
MISSION_ALLOW_STRUCTURED_LEGACY is set; converted to flat list on load.
"""

from __future__ import annotations

from typing import Any

from compliance_profile import MISSION_ALLOW_STRUCTURED_LEGACY, MISSION_INPUT_FORMAT
from mission_config import (
    MISSION_MIN_WAYPOINTS,
    VALID_TARGET_COLORS,
    normalize_target_color,
    split_nav_engage,
    validate_coordinate_mission,
    validate_coordinate_pair,
    validate_target_color,
    # Legacy shim (still needed for telemetry.py backward compat)
    split_mission_waypoints,
    get_mission_split_profile,
)
from runtime_debug_log import log_jsonl, setup_component_logger

_log = setup_component_logger("mission_adapter")

# Legacy aliases for mission_config names that external code may import from here
MISSION_P1_MIN_WAYPOINTS = 1
MISSION_P2_MIN_WAYPOINTS = 0
MISSION_P3_MIN_WAYPOINTS = 1


def validate_waypoint_array(waypoints: Any, array_name: str) -> list[list[float]]:
    """Validate a waypoint array (used for legacy structured mission parsing)."""
    if not isinstance(waypoints, list):
        raise ValueError(f"{array_name} must be array, got {type(waypoints).__name__}")
    if len(waypoints) == 0:
        raise ValueError(f"{array_name} cannot be empty")

    validated = []
    for i, wp in enumerate(waypoints):
        if not isinstance(wp, list) or len(wp) != 2:
            raise ValueError(f"{array_name}[{i}] must be [lat, lon], got {wp}")
        validated.append(validate_coordinate_pair(wp, f"{array_name}[{i}]"))

    return validated


def adapt_mission_to_structured(
    data: Any,
    strict: bool = True,
) -> dict[str, Any]:
    """
    Convert mission data to internal unified representation.

    Returns dict with keys:
    - 'nav_waypoints': list of [lat, lon] — intermediate navigation points
    - 'engage_wp':     [lat, lon] | None  — final target engage point
    - 'target_color':  str                — target duba color
    - '_adapter_source', '_mission_input_format', '_split_profile': metadata

    Legacy backward-compat keys also returned:
    - 'parkur1': nav_waypoints (alias)
    - 'parkur2': []
    - 'parkur3': [engage_wp] or []
    """
    if isinstance(data, list):
        return _adapt_flat_array(data, strict=strict)
    if isinstance(data, dict):
        return _adapt_structured_legacy(data, strict=strict)
    raise ValueError(f"Mission must be array or object, got {type(data).__name__}")


def _adapt_flat_array(flat_array: list, strict: bool = True) -> dict[str, Any]:
    """Convert canonical flat ordered mission input to unified representation."""
    total_waypoints = len(flat_array)
    _log.info("flat array received: %d waypoint(s)", total_waypoints)

    coords = validate_coordinate_mission(flat_array)
    nav_wps, engage_wp = split_nav_engage(coords)

    _log.info(
        "split applied: nav=%d engage=%s",
        len(nav_wps),
        "yes" if engage_wp is not None else "no",
    )
    log_jsonl(
        "mission_adapter",
        False,
        event="split",
        total_waypoints=total_waypoints,
        nav_count=len(nav_wps),
        has_engage=engage_wp is not None,
    )

    if strict and engage_wp is None:
        raise ValueError(
            f"Mission must have at least {MISSION_MIN_WAYPOINTS} waypoints "
            f"(nav + engage); got {total_waypoints}"
        )

    engage_list = [engage_wp] if engage_wp is not None else []
    split_profile = get_mission_split_profile(total_waypoints)

    return {
        # Primary unified keys
        "nav_waypoints": nav_wps,
        "engage_wp": engage_wp,
        # Legacy backward-compat keys
        "parkur1": nav_wps,
        "parkur2": [],
        "parkur3": engage_list,
        "target_color": "",
        "_adapter_source": "flat_array",
        "_mission_input_format": MISSION_INPUT_FORMAT,
        "_split_profile": split_profile,
    }


def _adapt_structured_legacy(data: dict, strict: bool = True) -> dict[str, Any]:
    """Validate legacy structured mission format (parkur1/2/3 dict) and flatten."""
    if not isinstance(data, dict):
        raise ValueError(f"Structured mission must be dict, got {type(data).__name__}")

    if strict and not MISSION_ALLOW_STRUCTURED_LEGACY:
        raise ValueError(
            "Structured mission payload disabled; "
            "send a flat ordered [lat, lon] JSON array."
        )

    required_fields = ["parkur1", "parkur3", "target_color"]
    missing = [f for f in required_fields if f not in data]
    if missing:
        raise ValueError(f"Missing required fields: {missing}")

    p1 = validate_waypoint_array(data.get("parkur1", []), "parkur1")
    p2 = validate_waypoint_array(data["parkur2"], "parkur2") if "parkur2" in data else []
    p3 = validate_waypoint_array(data["parkur3"], "parkur3") if data.get("parkur3") else []

    # Flatten all into a single ordered list and re-split
    all_coords = p1 + p2 + p3
    if not all_coords:
        raise ValueError("Structured mission contains no waypoints")

    target_color = validate_target_color(data["target_color"])
    nav_wps, engage_wp = split_nav_engage(all_coords)
    engage_list = [engage_wp] if engage_wp is not None else []
    total_count = len(all_coords)
    split_profile = get_mission_split_profile(total_count)

    _log.info(
        "structured legacy adapted: p1=%d p2=%d p3=%d → nav=%d engage=%s",
        len(p1), len(p2), len(p3), len(nav_wps), "yes" if engage_wp else "no",
    )

    return {
        # Primary unified keys
        "nav_waypoints": nav_wps,
        "engage_wp": engage_wp,
        # Legacy backward-compat keys
        "parkur1": nav_wps,
        "parkur2": [],
        "parkur3": engage_list,
        "target_color": target_color,
        "_adapter_source": "structured_legacy",
        "_mission_input_format": MISSION_INPUT_FORMAT,
        "_split_profile": {
            "input_format": "structured_legacy",
            "allow_structured_legacy": bool(MISSION_ALLOW_STRUCTURED_LEGACY),
            "nav_count": len(nav_wps),
            "engage_count": len(engage_list),
            "total_count": total_count,
            # Legacy keys
            "p1_count": len(p1),
            "p2_count": len(p2),
            "p3_count": len(p3),
        },
    }


__all__ = [
    "VALID_TARGET_COLORS",
    "normalize_target_color",
    "validate_target_color",
    "validate_waypoint_array",
    "adapt_mission_to_structured",
    "split_mission_waypoints",
    "get_mission_split_profile",
]

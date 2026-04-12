"""
Mission format adapter: keeps legacy structured parsing available behind an explicit
compatibility switch, while the operational contract remains a flat ordered waypoint list.
"""

from __future__ import annotations

from typing import Any

from compliance_profile import MISSION_ALLOW_STRUCTURED_LEGACY, MISSION_INPUT_FORMAT
from mission_config import (
    MISSION_P1_MIN_WAYPOINTS,
    MISSION_P2_MIN_WAYPOINTS,
    MISSION_P3_MIN_WAYPOINTS,
    VALID_TARGET_COLORS,
    get_mission_split_profile,
    normalize_target_color,
    split_mission_waypoints,
    validate_coordinate_mission,
    validate_coordinate_pair,
    validate_target_color,
)
from runtime_debug_log import log_jsonl, setup_component_logger

_log = setup_component_logger("mission_adapter")


def validate_waypoint_array(waypoints: Any, parkur_name: str) -> list[list[float]]:
    """Validate a legacy parkur waypoint array."""
    if not isinstance(waypoints, list):
        raise ValueError(f"{parkur_name} must be array, got {type(waypoints).__name__}")
    if len(waypoints) == 0:
        raise ValueError(f"{parkur_name} cannot be empty")

    validated = []
    for i, wp in enumerate(waypoints):
        if not isinstance(wp, list) or len(wp) != 2:
            raise ValueError(f"{parkur_name}[{i}] must be [lat, lon], got {wp}")
        validated.append(validate_coordinate_pair(wp, f"{parkur_name}[{i}]"))

    return validated


def adapt_mission_to_structured(
    data: Any,
    strict: bool = True
) -> dict[str, Any]:
    """
    Convert mission data to the internal structured parkur representation.

    Operational contract:
    - flat ordered list: [[lat, lon], ...]

    Legacy compatibility contract:
    - structured dict: {"parkur1": [...], "parkur2": [...], "parkur3": [...], "target_color": "..."}
    """
    if isinstance(data, list):
        return _adapt_flat_array_to_structured(data, strict=strict)
    if isinstance(data, dict):
        return _validate_and_return_structured(data, strict=strict)
    raise ValueError(f"Mission must be array or object, got {type(data).__name__}")


def _adapt_flat_array_to_structured(
    flat_array: list,
    strict: bool = True
) -> dict[str, Any]:
    """Convert canonical flat ordered mission input into structured parkur slices."""
    total_waypoints = len(flat_array)
    _log.info("flat array received: %d waypoint(s)", total_waypoints)

    coords = validate_coordinate_mission(flat_array)
    p1, p2, p3 = split_mission_waypoints(coords, validate_lengths=False)
    split_profile = get_mission_split_profile(len(coords))

    _log.info("split profile applied: P1=%d, P2=%d, P3=%d", len(p1), len(p2), len(p3))
    log_jsonl("mission_adapter", False, event="split",
              total_waypoints=total_waypoints,
              parkur1_count=len(p1),
              parkur2_count=len(p2),
              parkur3_count=len(p3),
              split_profile=split_profile)

    if strict:
        try:
            if len(p1) < MISSION_P1_MIN_WAYPOINTS:
                raise ValueError(f"P1 must have ≥{MISSION_P1_MIN_WAYPOINTS} waypoints, got {len(p1)}")
            if len(p2) < MISSION_P2_MIN_WAYPOINTS:
                raise ValueError(f"P2 must have ≥{MISSION_P2_MIN_WAYPOINTS} waypoints, got {len(p2)}")
            if len(p3) < MISSION_P3_MIN_WAYPOINTS:
                raise ValueError(f"P3 must have ≥{MISSION_P3_MIN_WAYPOINTS} waypoints, got {len(p3)}")
            _log.info("validation PASSED")
        except ValueError as e:
            _log.error("validation FAILED: %s", e)
            log_jsonl("mission_adapter", False, event="validation_error",
                      error=str(e), waypoint_count=total_waypoints)
            raise

    return {
        "parkur1": p1,
        "parkur2": p2,
        "parkur3": p3,
        "target_color": "",
        "_adapter_source": "flat_array",
        "_mission_input_format": MISSION_INPUT_FORMAT,
        "_split_profile": split_profile,
    }


def _validate_and_return_structured(
    data: dict,
    strict: bool = True
) -> dict[str, Any]:
    """Validate legacy structured mission format and return it when explicitly allowed."""
    if not isinstance(data, dict):
        raise ValueError(f"Structured mission must be dict, got {type(data).__name__}")

    if strict and not MISSION_ALLOW_STRUCTURED_LEGACY:
        raise ValueError(
            "Structured mission payload disabled; send a flat ordered JSON array and set target_color separately."
        )

    required_fields = ["parkur1", "parkur2", "parkur3", "target_color"]
    missing = [f for f in required_fields if f not in data]
    if missing:
        raise ValueError(f"Missing required fields: {missing}")

    validated = {}
    for parkur_name in ["parkur1", "parkur2", "parkur3"]:
        validated[parkur_name] = validate_waypoint_array(data[parkur_name], parkur_name)

    target_color = validate_target_color(data["target_color"])

    if strict:
        if len(validated["parkur1"]) < MISSION_P1_MIN_WAYPOINTS:
            raise ValueError(f"P1 must have ≥{MISSION_P1_MIN_WAYPOINTS} waypoints, got {len(validated['parkur1'])}")
        if len(validated["parkur2"]) < MISSION_P2_MIN_WAYPOINTS:
            raise ValueError(f"P2 must have ≥{MISSION_P2_MIN_WAYPOINTS} waypoints, got {len(validated['parkur2'])}")
        if len(validated["parkur3"]) < MISSION_P3_MIN_WAYPOINTS:
            raise ValueError(f"P3 must have ≥{MISSION_P3_MIN_WAYPOINTS} waypoints, got {len(validated['parkur3'])}")

    return {
        "parkur1": validated["parkur1"],
        "parkur2": validated["parkur2"],
        "parkur3": validated["parkur3"],
        "target_color": target_color,
        "_adapter_source": "structured_legacy",
        "_mission_input_format": MISSION_INPUT_FORMAT,
        "_split_profile": {
            "input_format": "structured_legacy",
            "allow_structured_legacy": bool(MISSION_ALLOW_STRUCTURED_LEGACY),
            "p1_count": len(validated["parkur1"]),
            "p2_count": len(validated["parkur2"]),
            "p3_count": len(validated["parkur3"]),
            "total_count": len(validated["parkur1"]) + len(validated["parkur2"]) + len(validated["parkur3"]),
        },
    }


__all__ = [
    "VALID_TARGET_COLORS",
    "normalize_target_color",
    "validate_target_color",
    "validate_waypoint_array",
    "adapt_mission_to_structured",
]

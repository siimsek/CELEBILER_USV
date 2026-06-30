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
import time
from typing import Any


MISSION_MIN_WAYPOINTS = 1  # At minimum: 1 nav waypoint
TARGET_STATE_FILE = os.environ.get("TARGET_STATE_FILE", f"{os.environ.get('CONTROL_DIR', '')}/target_state.json")
MISSION_PROFILE_SCHEMA_VERSION = 1
MISSION_PROFILE_REQUIRED_FIELDS = (
    "profile_schema_version",
    "target_color",
    "upload_source",
    "validation_timestamp",
    "phase_transition_policy",
    "p2_min_gate_count",
    "p3_engagement_mode",
)
MISSION_PROFILE_PARKUR_KEYS = ("p1", "p2", "p3")
MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT = 2
MISSION_PROFILE_DEFAULT_P3_ENGAGEMENT_MODE = "vision_color_track"
MISSION_PROFILE_ALLOWED_P3_ENGAGEMENT_MODES = (
    "vision_color_track",
    "vision_color_track_with_gps_fallback",
)
MISSION_PROFILE_DEFAULT_TRANSITION_POLICY = {
    "p1_completion_source": "pixhawk_auto_progress",
    "p2_completion_source": "gate_count_and_guidance",
    "p3_entry_source": "target_color_lock",
}

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


def mission_profile_contract() -> dict[str, Any]:
    """Return the canonical pre-start mission_profile contract."""
    return {
        "profile_schema_version": MISSION_PROFILE_SCHEMA_VERSION,
        "required_fields": list(MISSION_PROFILE_REQUIRED_FIELDS),
        "parkur_ranges": "optional range hints only; not required for race logic",
        "p1_waypoint_count": "optional int hint; do not use as the sole race split source",
        "p2_min_gate_count": f"int >= {MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT}",
        "p3_engage_waypoint": "not used; P3 is vision/color engagement, not a waypoint",
        "phase_transition_policy": dict(MISSION_PROFILE_DEFAULT_TRANSITION_POLICY),
        "p3_engagement_mode": list(MISSION_PROFILE_ALLOWED_P3_ENGAGEMENT_MODES),
        "target_color": list(VALID_TARGET_COLORS),
        "upload_source": "pixhawk_mission | structured_profile | local_flat_file",
        "validation_timestamp": "unix epoch seconds",
    }


def build_mission_profile(
    *,
    target_color: Any,
    upload_source: str,
    validation_timestamp: float,
    p2_min_gate_count: int = MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT,
    phase_transition_policy: dict[str, Any] | None = None,
    p3_engagement_mode: str = MISSION_PROFILE_DEFAULT_P3_ENGAGEMENT_MODE,
    parkur_ranges: dict[str, Any] | None = None,
    p1_waypoint_count: int | None = None,
) -> dict[str, Any]:
    """Build a normalized mission_profile payload without changing mission loading."""
    profile = {
        "profile_schema_version": MISSION_PROFILE_SCHEMA_VERSION,
        "target_color": validate_target_color(target_color),
        "upload_source": str(upload_source or "").strip() or "unknown",
        "validation_timestamp": float(validation_timestamp),
        "phase_transition_policy": dict(phase_transition_policy or MISSION_PROFILE_DEFAULT_TRANSITION_POLICY),
        "p2_min_gate_count": int(p2_min_gate_count),
        "p3_engagement_mode": str(p3_engagement_mode or MISSION_PROFILE_DEFAULT_P3_ENGAGEMENT_MODE),
    }
    if parkur_ranges is not None:
        profile["parkur_ranges"] = parkur_ranges
    if p1_waypoint_count is not None:
        profile["p1_waypoint_count"] = int(p1_waypoint_count)
    validate_mission_profile_contract(profile)
    return profile


def validate_mission_profile_contract(profile: Any) -> dict[str, Any]:
    """
    Validate the locked pre-start mission_profile contract.

    This is intentionally strict and independent from the legacy flat mission
    loader. Later race start gates should reject missions when this contract is
    absent or invalid.
    """
    if not isinstance(profile, dict):
        raise ValueError("mission_profile must be an object")

    missing = [field for field in MISSION_PROFILE_REQUIRED_FIELDS if field not in profile]
    if missing:
        raise ValueError(f"mission_profile missing required fields: {missing}")

    schema_version = int(profile.get("profile_schema_version"))
    if schema_version != MISSION_PROFILE_SCHEMA_VERSION:
        raise ValueError(
            f"mission_profile.profile_schema_version must be {MISSION_PROFILE_SCHEMA_VERSION}, "
            f"got {schema_version}"
        )

    target_color = validate_target_color(profile.get("target_color"))

    upload_source = str(profile.get("upload_source") or "").strip()
    if not upload_source:
        raise ValueError("mission_profile.upload_source must be non-empty")

    validation_timestamp = float(profile.get("validation_timestamp"))
    if validation_timestamp <= 0:
        raise ValueError("mission_profile.validation_timestamp must be > 0")

    policy = profile.get("phase_transition_policy")
    if not isinstance(policy, dict):
        raise ValueError("mission_profile.phase_transition_policy must be an object")
    normalized_policy = dict(MISSION_PROFILE_DEFAULT_TRANSITION_POLICY)
    normalized_policy.update({str(k): str(v) for k, v in policy.items() if v is not None})
    for key in MISSION_PROFILE_DEFAULT_TRANSITION_POLICY:
        if not str(normalized_policy.get(key, "")).strip():
            raise ValueError(f"mission_profile.phase_transition_policy.{key} must be non-empty")

    p2_min = int(profile.get("p2_min_gate_count"))
    if p2_min < MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT:
        raise ValueError(
            "mission_profile.p2_min_gate_count must be >= "
            f"{MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT}"
        )

    p3_mode = str(profile.get("p3_engagement_mode") or "").strip()
    if p3_mode not in MISSION_PROFILE_ALLOWED_P3_ENGAGEMENT_MODES:
        raise ValueError(
            "mission_profile.p3_engagement_mode must be one of "
            f"{list(MISSION_PROFILE_ALLOWED_P3_ENGAGEMENT_MODES)}"
        )

    normalized = dict(profile)
    normalized["profile_schema_version"] = schema_version
    normalized["target_color"] = target_color
    normalized["upload_source"] = upload_source
    normalized["validation_timestamp"] = validation_timestamp
    normalized["phase_transition_policy"] = normalized_policy
    normalized["p2_min_gate_count"] = p2_min
    normalized["p3_engagement_mode"] = p3_mode

    if "p1_waypoint_count" in normalized and normalized["p1_waypoint_count"] is not None:
        p1_count = int(normalized["p1_waypoint_count"])
        if p1_count < 1:
            raise ValueError("mission_profile.p1_waypoint_count must be >= 1 when provided")
        normalized["p1_waypoint_count"] = p1_count

    parkur_ranges = normalized.get("parkur_ranges")
    if parkur_ranges is not None:
        if not isinstance(parkur_ranges, dict):
            raise ValueError("mission_profile.parkur_ranges must be an object when provided")
        for key, rng in parkur_ranges.items():
            if not isinstance(rng, dict):
                raise ValueError(f"mission_profile.parkur_ranges.{key} must be an object")
            start_idx = rng.get("start_index")
            end_idx = rng.get("end_index")
            if not isinstance(start_idx, int) or not isinstance(end_idx, int):
                raise ValueError(f"mission_profile.parkur_ranges.{key} start_index/end_index must be int")
            if start_idx < 0 or end_idx < start_idx:
                raise ValueError(f"mission_profile.parkur_ranges.{key} has invalid bounds")
        normalized["parkur_ranges"] = {str(key): dict(value) for key, value in parkur_ranges.items()}

    normalized.pop("p3_engage_waypoint", None)
    return normalized


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


def _coordinate_pairs_equal(a: Any, b: Any, *, tol: float = 1e-7) -> bool:
    try:
        aa = validate_coordinate_pair(a, "coord_a")
        bb = validate_coordinate_pair(b, "coord_b")
    except ValueError:
        return False
    return abs(aa[0] - bb[0]) <= tol and abs(aa[1] - bb[1]) <= tol


def _normalize_optional_profile_ranges(
    parkur_ranges: dict[str, Any],
    total_count: int,
) -> dict[str, dict[str, int]]:
    if not isinstance(parkur_ranges, dict):
        raise ValueError("mission_profile.parkur_ranges must be an object")

    normalized: dict[str, dict[str, int]] = {}
    for key, rng in parkur_ranges.items():
        if not isinstance(rng, dict):
            raise ValueError(f"mission_profile.parkur_ranges.{key} must be an object")
        start_idx = int(rng.get("start_index"))
        end_idx = int(rng.get("end_index"))
        if end_idx < start_idx:
            raise ValueError(f"mission_profile.parkur_ranges.{key} has invalid bounds")
        if end_idx > int(total_count):
            raise ValueError(
                f"mission_profile.parkur_ranges.{key}.end_index {end_idx} exceeds waypoint count {total_count}"
            )
        normalized[str(key)] = {"start_index": start_idx, "end_index": end_idx}
    return normalized


def _slice_range(coords: list[list[float]], rng: dict[str, int]) -> list[list[float]]:
    return list(coords[int(rng["start_index"]): int(rng["end_index"])])


def _extract_profile_waypoints(payload: dict[str, Any]) -> list[list[float]]:
    for key in ("waypoints", "mission", "coordinates"):
        if key in payload:
            return validate_coordinate_mission(payload[key])
    return []


def parse_mission_profile_payload(
    payload: Any,
    *,
    upload_source: str = "structured_profile",
    validation_timestamp: float | None = None,
    race_required: bool = False,
) -> dict[str, Any]:
    """
    Parse mission input into a profile-aware mission description.

    Flat arrays keep the legacy nav-only behavior, but they are explicitly not
    race-ready because they cannot carry target color lock or transition policy.
    Structured profile payloads may include optional half-open range hints, but
    the runtime must not depend on static P1/P2/P3 waypoint splits. P3 is a
    vision/color engagement phase, not a waypoint engagement point.
    """
    ts = float(validation_timestamp if validation_timestamp is not None else time.time())

    if isinstance(payload, list):
        coords = validate_coordinate_mission(payload)
        if race_required:
            raise ValueError("flat mission array is not race-ready without mission_profile target_color and transition policy")
        nav_wps, engage_wp = split_nav_engage(coords, has_explicit_engage=False)
        return {
            "input_format": "flat_ordered",
            "race_ready": False,
            "flat_waypoints": coords,
            "nav_waypoints": nav_wps,
            "engage_wp": engage_wp,
            "parkur1": nav_wps,
            "parkur2": [],
            "parkur3": [],
            "target_color": "",
            "mission_profile": None,
            "mission_profile_valid": False,
            "profile_error": "flat mission has no mission_profile",
        }

    if not isinstance(payload, dict):
        raise ValueError(f"mission payload must be array or object, got {type(payload).__name__}")

    if "mission_profile" in payload:
        coords = _extract_profile_waypoints(payload)
        profile = validate_mission_profile_contract(payload["mission_profile"])
        parkur1: list[list[float]] = []
        parkur2: list[list[float]] = []
        parkur3: list[list[float]] = []
        if coords and "parkur_ranges" in profile:
            profile = dict(profile)
            profile["parkur_ranges"] = _normalize_optional_profile_ranges(profile["parkur_ranges"], len(coords))
            ranges = profile["parkur_ranges"]
            if "p1" in ranges:
                parkur1 = _slice_range(coords, ranges["p1"])
            if "p2" in ranges:
                parkur2 = _slice_range(coords, ranges["p2"])
            if "p3" in ranges:
                parkur3 = _slice_range(coords, ranges["p3"])
        nav_waypoints = (list(parkur1) + list(parkur2)) if (parkur1 or parkur2 or parkur3) else list(coords)
        return {
            "input_format": "structured_profile",
            "race_ready": True,
            "flat_waypoints": coords,
            "nav_waypoints": nav_waypoints,
            "engage_wp": None,
            "parkur1": parkur1,
            "parkur2": parkur2,
            "parkur3": parkur3,
            "p3_search_waypoints": parkur3,
            "target_color": profile["target_color"],
            "mission_profile": profile,
            "mission_profile_valid": True,
            "profile_error": "",
        }

    if {"parkur1", "target_color"}.issubset(payload.keys()):
        p1_wps = validate_coordinate_mission(payload.get("parkur1"))
        p2_raw = payload.get("parkur2", [])
        p2_wps = validate_coordinate_mission(p2_raw) if p2_raw else []
        p3_raw = payload.get("parkur3", [])
        p3_wps = validate_coordinate_mission(p3_raw) if p3_raw else []
        coords = p1_wps + p2_wps + p3_wps
        profile = build_mission_profile(
            p2_min_gate_count=int(payload.get("p2_min_gate_count", MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT)),
            target_color=payload.get("target_color"),
            upload_source=upload_source,
            validation_timestamp=ts,
            p1_waypoint_count=len(p1_wps) if p1_wps else None,
            p3_engagement_mode=str(payload.get("p3_engagement_mode", MISSION_PROFILE_DEFAULT_P3_ENGAGEMENT_MODE)),
        )
        return {
            "input_format": "structured_legacy_profile",
            "race_ready": True,
            "flat_waypoints": coords,
            "nav_waypoints": p1_wps + p2_wps,
            "engage_wp": None,
            "parkur1": p1_wps,
            "parkur2": p2_wps,
            "parkur3": p3_wps,
            "p3_search_waypoints": p3_wps,
            "target_color": profile["target_color"],
            "mission_profile": profile,
            "mission_profile_valid": True,
            "profile_error": "",
        }

    raise ValueError("structured mission object must contain mission_profile or parkur1/target_color")


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
        payload = json.load(handle)
    if isinstance(payload, dict):
        for key in ("waypoints", "mission", "coordinates"):
            if key in payload:
                return validate_coordinate_mission(payload[key])
    return validate_coordinate_mission(payload)


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

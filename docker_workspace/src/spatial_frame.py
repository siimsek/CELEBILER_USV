"""
Unified ENU spatial frame helpers for dashboard map, lidar world grid, and sim pose.

Contract:
- Map frame is local ENU with origin at SIM_HOME (sim) or first valid GPS/mission point (real).
- Gazebo vehicle_position.json uses x=north, y=west; ENU uses x=east, y=north.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any

from compliance_profile import CONTROL_DIR, MISSION_FILE_DEFAULT
from sim_nav_state import heading_rad_to_nav_deg, load_sim_nav_state, parse_sim_home


def gazebo_xy_to_spatial_enu(x_m: float, y_m: float) -> tuple[float, float]:
    """Convert Gazebo north/west coordinates to dashboard ENU east/north meters."""
    return -float(y_m), float(x_m)


def spatial_enu_to_gazebo_xy(east_m: float, north_m: float) -> tuple[float, float]:
    """Inverse of gazebo_xy_to_spatial_enu."""
    return float(north_m), -float(east_m)


def latlon_to_spatial_enu_m(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    lat_scale = 111320.0
    lon_scale = 111320.0 * math.cos(math.radians(float(origin_lat)))
    east_m = (float(lon) - float(origin_lon)) * lon_scale
    north_m = (float(lat) - float(origin_lat)) * lat_scale
    return float(east_m), float(north_m)


def nav_deg_to_heading_rad(heading_deg: float) -> float:
    return math.radians((-float(heading_deg) + 360.0) % 360.0)


def resolve_spatial_origin_candidates(
    *,
    cached_origin: dict[str, float | None] | None,
    state: dict[str, Any] | None,
    telemetry_lat: float | None,
    telemetry_lon: float | None,
    mission_file: str | None = None,
) -> tuple[float | None, float | None]:
    cache = cached_origin if isinstance(cached_origin, dict) else {}
    if cache.get("lat") is not None and cache.get("lon") is not None:
        return float(cache["lat"]), float(cache["lon"])

    if os.environ.get("USV_SIM") == "1":
        home_lat, home_lon, _, _ = parse_sim_home()
        return float(home_lat), float(home_lon)

    state = state if isinstance(state, dict) else {}
    candidates = [
        (telemetry_lat, telemetry_lon),
        (state.get("Lat"), state.get("Lon")),
    ]
    mission_path = str(mission_file or os.environ.get("MISSION_FILE", MISSION_FILE_DEFAULT))
    try:
        if os.path.exists(mission_path):
            with open(mission_path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            if isinstance(payload, list) and payload and isinstance(payload[0], (list, tuple)) and len(payload[0]) >= 2:
                candidates.append((payload[0][0], payload[0][1]))
    except Exception:
        pass

    for lat_raw, lon_raw in candidates:
        try:
            lat = float(lat_raw)
            lon = float(lon_raw)
        except (TypeError, ValueError):
            continue
        if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0 and (abs(lat) > 1e-6 or abs(lon) > 1e-6):
            return lat, lon
    return None, None


def read_sim_gazebo_pose() -> tuple[float, float, float]:
    try:
        pose_path = os.path.join(CONTROL_DIR, "vehicle_position.json")
        with open(pose_path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        return (
            float(payload.get("pos_x", 0.0) or 0.0),
            float(payload.get("pos_y", 0.0) or 0.0),
            float(payload.get("heading_rad", 0.0) or 0.0),
        )
    except Exception:
        return 0.0, 0.0, 0.0


def boat_spatial_enu_from_sim_pose() -> dict[str, float] | None:
    if os.environ.get("USV_SIM") != "1":
        return None
    pos_x, pos_y, heading_rad = read_sim_gazebo_pose()
    east_m, north_m = gazebo_xy_to_spatial_enu(pos_x, pos_y)
    return {
        "x_m": round(float(east_m), 3),
        "y_m": round(float(north_m), 3),
        "heading_deg": round(float(heading_rad_to_nav_deg(heading_rad)), 3),
    }


def boat_spatial_enu_from_latlon(
    lat: float,
    lon: float,
    heading_deg: float,
    origin_lat: float,
    origin_lon: float,
) -> dict[str, float]:
    east_m, north_m = latlon_to_spatial_enu_m(lat, lon, origin_lat, origin_lon)
    return {
        "x_m": round(float(east_m), 3),
        "y_m": round(float(north_m), 3),
        "heading_deg": round(float(heading_deg), 3),
    }


def resolve_boat_spatial_enu(
    *,
    origin_lat: float,
    origin_lon: float,
    telemetry_lat: float | None,
    telemetry_lon: float | None,
    telemetry_heading_deg: float | None,
) -> dict[str, float] | None:
    nav = load_sim_nav_state()
    if os.environ.get("USV_SIM") == "1" and bool(nav.get("valid")):
        try:
            lat = float(nav.get("lat"))
            lon = float(nav.get("lon"))
            heading_deg = float(nav.get("heading_deg"))
            return boat_spatial_enu_from_latlon(lat, lon, heading_deg, origin_lat, origin_lon)
        except (TypeError, ValueError):
            pass
    fallback = boat_spatial_enu_from_sim_pose()
    if isinstance(fallback, dict):
        return fallback
    try:
        if telemetry_lat is None or telemetry_lon is None:
            return None
        heading_deg = float(telemetry_heading_deg or 0.0)
        return boat_spatial_enu_from_latlon(
            float(telemetry_lat),
            float(telemetry_lon),
            heading_deg,
            origin_lat,
            origin_lon,
        )
    except (TypeError, ValueError):
        return None


def current_spatial_pose_enu(
    *,
    simulation_mode: bool,
    origin_lat: float,
    origin_lon: float,
    current_lat: float,
    current_lon: float,
    current_heading_deg: float,
) -> tuple[float, float, float]:
    """Return boat pose as (east_m, north_m, heading_rad) for lidar world mapping."""
    if simulation_mode or os.environ.get("USV_SIM") == "1":
        pos_x, pos_y, heading_rad = read_sim_gazebo_pose()
        nav = load_sim_nav_state()
        if bool(nav.get("valid")):
            try:
                east_m, north_m = latlon_to_spatial_enu_m(
                    float(nav.get("lat")),
                    float(nav.get("lon")),
                    origin_lat,
                    origin_lon,
                )
                return float(east_m), float(north_m), float(heading_rad)
            except (TypeError, ValueError):
                pass
        east_m, north_m = gazebo_xy_to_spatial_enu(pos_x, pos_y)
        return float(east_m), float(north_m), float(heading_rad)

    try:
        east_m, north_m = latlon_to_spatial_enu_m(current_lat, current_lon, origin_lat, origin_lon)
        heading_rad = math.radians(float(current_heading_deg) % 360.0)
        return float(east_m), float(north_m), float(heading_rad)
    except (TypeError, ValueError):
        return 0.0, 0.0, 0.0


def lidar_local_to_world_enu(
    x_local: float,
    y_local: float,
    east_m: float,
    north_m: float,
    heading_rad: float,
) -> tuple[float, float]:
    """Body lidar frame -> ENU using Gazebo body-yaw (heading_rad) convention."""
    c = math.cos(float(heading_rad))
    s = math.sin(float(heading_rad))
    de = -(float(x_local) * s) - (float(y_local) * c)
    dn = (float(x_local) * c) - (float(y_local) * s)
    return float(east_m) + float(de), float(north_m) + float(dn)


def lidar_local_to_world_enu_compass(
    x_local: float,
    y_local: float,
    east_m: float,
    north_m: float,
    heading_deg: float,
) -> tuple[float, float]:
    """Body lidar frame -> ENU using navigation compass heading (0=north, 90=east)."""
    h = math.radians(float(heading_deg) % 360.0)
    s = math.sin(h)
    c = math.cos(h)
    de = (float(x_local) * s) - (float(y_local) * c)
    dn = (float(x_local) * c) + (float(y_local) * s)
    return float(east_m) + float(de), float(north_m) + float(dn)


def world_enu_to_lidar_local(
    east_m: float,
    north_m: float,
    boat_east_m: float,
    boat_north_m: float,
    heading_rad: float,
) -> tuple[float, float]:
    de = float(east_m) - float(boat_east_m)
    dn = float(north_m) - float(boat_north_m)
    c = math.cos(float(heading_rad))
    s = math.sin(float(heading_rad))
    x_local = (-s * de) + (c * dn)
    y_local = (-c * de) - (s * dn)
    return float(x_local), float(y_local)


def world_enu_to_lidar_local_compass(
    east_m: float,
    north_m: float,
    boat_east_m: float,
    boat_north_m: float,
    heading_deg: float,
) -> tuple[float, float]:
    de = float(east_m) - float(boat_east_m)
    dn = float(north_m) - float(boat_north_m)
    h = math.radians(float(heading_deg) % 360.0)
    s = math.sin(h)
    c = math.cos(h)
    x_local = (dn * c) - (de * s)
    y_local = -(dn * s) - (de * c)
    return float(x_local), float(y_local)


def lidar_local_to_world_enu_gazebo(
    x_local: float,
    y_local: float,
    pos_x: float,
    pos_y: float,
    heading_rad: float,
) -> tuple[float, float]:
    c = math.cos(float(heading_rad))
    s = math.sin(float(heading_rad))
    x_gazebo = float(pos_x) + (float(x_local) * c) - (float(y_local) * s)
    y_gazebo = float(pos_y) + (float(x_local) * s) + (float(y_local) * c)
    return gazebo_xy_to_spatial_enu(x_gazebo, y_gazebo)


def compute_spatial_bounds(point_groups: list[list[tuple[float, float]]]) -> dict[str, float | None]:
    xs: list[float] = []
    ys: list[float] = []
    for group in point_groups:
        for x_raw, y_raw in group:
            try:
                xs.append(float(x_raw))
                ys.append(float(y_raw))
            except (TypeError, ValueError):
                continue
    if not xs or not ys:
        return {
            "min_east_m": None,
            "max_east_m": None,
            "min_north_m": None,
            "max_north_m": None,
        }
    return {
        "min_east_m": round(min(xs), 3),
        "max_east_m": round(max(xs), 3),
        "min_north_m": round(min(ys), 3),
        "max_north_m": round(max(ys), 3),
    }


def default_course_layout_path() -> Path:
    root = Path(__file__).resolve().parents[2]
    return root / "sim" / "configs" / "spatial_course_layout.json"


def load_course_static_features(path: Path | None = None) -> list[dict[str, Any]]:
    if os.environ.get("USV_SIM") != "1":
        return []
    layout_path = path or default_course_layout_path()
    if not layout_path.is_file():
        return []
    try:
        with open(layout_path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except Exception:
        return []
    features = payload.get("features") if isinstance(payload, dict) else None
    if not isinstance(features, list):
        return []
    out: list[dict[str, Any]] = []
    for item in features:
        if not isinstance(item, dict):
            continue
        try:
            out.append(
                {
                    "type": str(item.get("type", "unknown")),
                    "name": str(item.get("name", "")),
                    "x_m": round(float(item.get("x_m")), 3),
                    "y_m": round(float(item.get("y_m")), 3),
                }
            )
        except (TypeError, ValueError):
            continue
    return out

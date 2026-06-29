#!/usr/bin/env python3
"""Unit checks for unified spatial frame transforms."""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim_nav_state import parse_sim_home  # noqa: E402
from spatial_frame import (  # noqa: E402
    gazebo_xy_to_spatial_enu,
    latlon_to_spatial_enu_m,
    lidar_local_to_world_enu,
    lidar_local_to_world_enu_gazebo,
    load_course_static_features,
    world_enu_to_lidar_local,
)


def _mission_waypoints(payload):
    if isinstance(payload, list):
        return payload
    if isinstance(payload, dict):
        for key in ("waypoints", "mission", "coordinates"):
            if isinstance(payload.get(key), list):
                return payload[key]
    raise AssertionError("mission_parkour_all.json must contain waypoints")


def _assert_close(label: str, actual: float, expected: float, tol: float = 0.25) -> None:
    if abs(float(actual) - float(expected)) > tol:
        raise AssertionError(f"{label}: got {actual:.3f}, expected {expected:.3f} (tol={tol})")


def test_wp1_gps_to_enu() -> None:
    mission_path = ROOT / "sim" / "configs" / "mission_parkour_all.json"
    with open(mission_path, "r", encoding="utf-8") as handle:
        mission = _mission_waypoints(json.load(handle))
    wp1_lat, wp1_lon = float(mission[0][0]), float(mission[0][1])
    home_lat, home_lon, _, _ = parse_sim_home()
    east_m, north_m = latlon_to_spatial_enu_m(wp1_lat, wp1_lon, home_lat, home_lon)
    _assert_close("WP1 east_m", east_m, 2.5)
    _assert_close("WP1 north_m", north_m, 3.5)


def test_gazebo_marker_matches_wp1() -> None:
    east_m, north_m = gazebo_xy_to_spatial_enu(3.5, -2.5)
    _assert_close("gazebo east_m", east_m, 2.5)
    _assert_close("gazebo north_m", north_m, 3.5)


def test_lidar_local_world_round_trip() -> None:
    boat_east, boat_north, heading_rad = 2.5, 3.5, math.radians(30.0)
    lx, ly = 4.2, -1.1
    wx, wy = lidar_local_to_world_enu(lx, ly, boat_east, boat_north, heading_rad)
    lx2, ly2 = world_enu_to_lidar_local(wx, wy, boat_east, boat_north, heading_rad)
    _assert_close("round-trip x", lx2, lx, tol=0.01)
    _assert_close("round-trip y", ly2, ly, tol=0.01)


def test_lidar_enu_matches_gazebo_path() -> None:
    pos_x, pos_y, heading_rad = 3.5, -2.5, 0.42
    boat_east, boat_north = gazebo_xy_to_spatial_enu(pos_x, pos_y)
    lx, ly = 6.0, 1.5
    wx_gz, wy_gz = lidar_local_to_world_enu_gazebo(lx, ly, pos_x, pos_y, heading_rad)
    wx_en, wy_en = lidar_local_to_world_enu(lx, ly, boat_east, boat_north, heading_rad)
    _assert_close("gazebo vs enu east", wx_en, wx_gz, tol=0.01)
    _assert_close("gazebo vs enu north", wy_en, wy_gz, tol=0.01)


def test_forward_lidar_point_is_north_when_heading_zero() -> None:
    boat_east, boat_north = 0.0, 0.0
    wx, wy = lidar_local_to_world_enu(5.0, 0.0, boat_east, boat_north, 0.0)
    _assert_close("forward east", wx, 0.0, tol=0.01)
    _assert_close("forward north", wy, 5.0, tol=0.01)


def test_course_layout_loads_in_sim() -> None:
    os.environ["USV_SIM"] = "1"
    features = load_course_static_features()
    if len(features) < 10:
        raise AssertionError(f"expected sim course features, got {len(features)}")
    orange = [f for f in features if f.get("type") == "boundary_orange"]
    yellow = [f for f in features if f.get("type") == "obstacle_yellow"]
    if not orange or not yellow:
        raise AssertionError("course layout missing orange/yellow buoys")


def main() -> int:
    test_wp1_gps_to_enu()
    test_gazebo_marker_matches_wp1()
    test_lidar_local_world_round_trip()
    test_lidar_enu_matches_gazebo_path()
    test_forward_lidar_point_is_north_when_heading_zero()
    test_course_layout_loads_in_sim()
    print("check_spatial_frame_unit: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""water_world.sdf ile mission JSON / SIM_HOME xy_to_gps tutarliligini kontrol eder. Cikis 0=OK, 1=hata."""
from __future__ import annotations

import json
import math
import os
import sys

PROJ = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
MISSION = os.path.join(PROJ, "sim/configs/mission_parkour_all.json")
NAV_POINTS = [
    (2.5, 3.5),
    (-2.5, 7.0),
    (2.5, 10.5),
    (0.0, 14.0),
    (0.0, 27.0),
]
TARGET_POINTS = {
    "RED": (-3.0, 29.8),
    "GREEN": (0.0, 29.8),
    "BLACK": (3.0, 29.8),
}


def load_base():
    raw = os.environ.get("SIM_HOME", "-35.363262,149.165237,584,0").strip()
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    base_lat = float(parts[0])
    base_lon = float(parts[1])
    return base_lat, base_lon


def xy_to_gps(base_lat, base_lon, wx, wy):
    meters_to_lat = 1.0 / 111320.0
    cos_lat = math.cos(math.radians(base_lat))
    meters_to_lon = 1.0 / (111320.0 * cos_lat) if abs(cos_lat) > 1e-6 else meters_to_lat
    return [
        round(base_lat + wy * meters_to_lat, 7),
        round(base_lon + wx * meters_to_lon, 7),
    ]


def gps_to_xy(base_lat, base_lon, lat, lon):
    meters_to_lat = 1.0 / 111320.0
    cos_lat = math.cos(math.radians(base_lat))
    meters_to_lon = 1.0 / (111320.0 * cos_lat) if abs(cos_lat) > 1e-6 else meters_to_lat
    y = (lat - base_lat) / meters_to_lat
    x = (lon - base_lon) / meters_to_lon
    return x, y


def mission_waypoints(payload):
    if isinstance(payload, list):
        return payload
    if isinstance(payload, dict):
        for key in ("waypoints", "mission", "coordinates"):
            value = payload.get(key)
            if isinstance(value, list):
                return value
    return None


def mission_profile(payload):
    if isinstance(payload, dict) and isinstance(payload.get("mission_profile"), dict):
        return payload["mission_profile"]
    return {}


def main() -> int:
    base_lat, base_lon = load_base()
    with open(MISSION, encoding="utf-8") as f:
        payload = json.load(f)
    m = mission_waypoints(payload)
    profile = mission_profile(payload)

    ok = True
    eps = 0.03

    if not isinstance(m, list):
        print("[FAIL] mission JSON must be a flat array or contain waypoints")
        return 1
    if len(m) < len(NAV_POINTS):
        print(f"[FAIL] mission waypoint sayisi {len(m)} < beklenen nav {len(NAV_POINTS)}")
        return 1

    for i, ((wx, wy), wp) in enumerate(zip(NAV_POINTS, m[: len(NAV_POINTS)])):
        g = xy_to_gps(base_lat, base_lon, wx, wy)
        if g[0] != wp[0] or g[1] != wp[1]:
            print(f"[FAIL] wp{i+1}: xy_to_gps({wx},{wy})={g} mission={wp}")
            ok = False
        ix, iy = gps_to_xy(base_lat, base_lon, wp[0], wp[1])
        if math.hypot(ix - wx, iy - wy) > eps:
            print(f"[FAIL] wp{i+1}: GPS ters {ix:.4f},{iy:.4f} beklenen {wx},{wy}")
            ok = False

    extras = m[len(NAV_POINTS):]
    if extras:
        target_color = str(profile.get("target_color", "RED") or "RED").upper()
        target_xy = TARGET_POINTS.get(target_color)
        if target_xy is None:
            print(f"[FAIL] bilinmeyen target_color={target_color} icin P3 hedef dogrulanamiyor")
            ok = False
        else:
            expected = xy_to_gps(base_lat, base_lon, target_xy[0], target_xy[1])
            p3_wp = extras[0]
            if expected[0] != p3_wp[0] or expected[1] != p3_wp[1]:
                print(f"[FAIL] p3 target {target_color}: xy_to_gps{target_xy}={expected} mission={p3_wp}")
                ok = False
            ix, iy = gps_to_xy(base_lat, base_lon, p3_wp[0], p3_wp[1])
            if math.hypot(ix - target_xy[0], iy - target_xy[1]) > eps:
                print(f"[FAIL] p3 target GPS ters {ix:.4f},{iy:.4f} beklenen {target_xy}")
                ok = False
        if len(extras) > 1:
            print(f"[FAIL] mission beklenenden fazla P3 hedef noktasi iceriyor: {len(extras)}")
            ok = False

    if ok:
        print(f"[OK] mission <-> SDF metrik uyumu: {MISSION}")
        print(f"     SIM_HOME base: {base_lat}, {base_lon}")
        print(f"     NAV waypoint list SDF metriği ile hizali ({len(NAV_POINTS)} WP).")
        if extras:
            print(f"     P3 target waypoint hizali: {profile.get('target_color', 'RED')}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""water_world.sdf ile mission JSON / SIM_HOME xy_to_gps tutarliligini kontrol eder. Cikis 0=OK, 1=hata."""
from __future__ import annotations

import json
import math
import os
import sys

PROJ = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
MISSION = os.path.join(PROJ, "sim/configs/mission_parkour_all.json")
POINTS = [
    (2.5, 3.5),
    (-2.5, 7.0),
    (2.5, 10.5),
    (0.0, 14.0),
    (0.0, 27.0),
    (-3.0, 29.8),
]


def load_base():
    raw = os.environ.get("SIM_HOME", "-35.363262,149.165237,584,0").strip()
    parts = [p.strip() for p in raw.split(",") if p.strip()]
    base_lat = float(parts[0])
    base_lon = float(parts[1])
    return base_lat, base_lon


def xy_to_gps(base_lat, base_lon, wx, wy):
    meters_to_lat = 1.0 / 111111.0
    cos_lat = math.cos(math.radians(base_lat))
    meters_to_lon = 1.0 / (111111.0 * cos_lat) if abs(cos_lat) > 1e-6 else meters_to_lat
    return [
        round(base_lat + wy * meters_to_lat, 7),
        round(base_lon - wx * meters_to_lon, 7),
    ]


def gps_to_xy(base_lat, base_lon, lat, lon):
    meters_to_lat = 1.0 / 111111.0
    cos_lat = math.cos(math.radians(base_lat))
    meters_to_lon = 1.0 / (111111.0 * cos_lat) if abs(cos_lat) > 1e-6 else meters_to_lat
    y = (lat - base_lat) / meters_to_lat
    x = (base_lon - lon) / meters_to_lon
    return x, y


def main() -> int:
    base_lat, base_lon = load_base()
    with open(MISSION, encoding="utf-8") as f:
        m = json.load(f)

    ok = True
    eps = 0.03

    if not isinstance(m, list):
        print("[FAIL] mission JSON must be a flat array")
        return 1
    if len(m) != len(POINTS):
        print(f"[FAIL] mission waypoint sayisi {len(m)} != beklenen {len(POINTS)}")
        return 1

    for i, ((wx, wy), wp) in enumerate(zip(POINTS, m)):
        g = xy_to_gps(base_lat, base_lon, wx, wy)
        if g[0] != wp[0] or g[1] != wp[1]:
            print(f"[FAIL] wp{i+1}: xy_to_gps({wx},{wy})={g} mission={wp}")
            ok = False
        ix, iy = gps_to_xy(base_lat, base_lon, wp[0], wp[1])
        if math.hypot(ix - wx, iy - wy) > eps:
            print(f"[FAIL] wp{i+1}: GPS ters {ix:.4f},{iy:.4f} beklenen {wx},{wy}")
            ok = False

    if ok:
        print(f"[OK] mission <-> SDF metrik uyumu: {MISSION}")
        print(f"     SIM_HOME base: {base_lat}, {base_lon}")
        print("     Flat waypoint list SDF metriği ile hizali.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

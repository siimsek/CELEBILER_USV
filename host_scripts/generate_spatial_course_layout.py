#!/usr/bin/env python3
"""Generate sim/configs/spatial_course_layout.json from water_world.sdf buoy poses."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from spatial_frame import gazebo_xy_to_spatial_enu  # noqa: E402


def main() -> int:
    sdf_path = ROOT / "sim" / "worlds" / "water_world.sdf"
    out_path = ROOT / "sim" / "configs" / "spatial_course_layout.json"
    text = sdf_path.read_text(encoding="utf-8")
    pattern = re.compile(
        r'<include><uri>model://(?P<model>[^<]+)</uri><name>(?P<name>[^<]+)</name><pose>(?P<pose>[^<]+)</pose></include>'
    )
    features = []
    for match in pattern.finditer(text):
        model = str(match.group("model"))
        name = str(match.group("name"))
        parts = str(match.group("pose")).split()
        if len(parts) < 2:
            continue
        gx = float(parts[0])
        gy = float(parts[1])
        east_m, north_m = gazebo_xy_to_spatial_enu(gx, gy)
        if "boundary_buoy_orange" in model:
            feat_type = "boundary_orange"
        elif "obstacle_buoy_yellow" in model:
            feat_type = "obstacle_yellow"
        elif "target_buoy" in model:
            feat_type = "target_buoy"
        else:
            continue
        features.append(
            {
                "type": feat_type,
                "name": name,
                "x_m": round(float(east_m), 3),
                "y_m": round(float(north_m), 3),
            }
        )
    payload = {
        "source": "sim/worlds/water_world.sdf",
        "frame": "enu_local",
        "feature_count": len(features),
        "features": features,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"wrote {out_path} features={len(features)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

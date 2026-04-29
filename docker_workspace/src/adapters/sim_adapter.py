"""
Simulation adapter boundary.

The current runtime still hosts most I/O inside usv_main.py. This adapter keeps
the shared contract explicit for the next extraction step without changing the
sim-to-real protocol.
"""

from __future__ import annotations

from pathlib import Path
import json
import time

from sensors import CameraSnapshot, LidarSnapshot, NavSnapshot, SensorSnapshot


class SimAdapter:
    def __init__(self, control_dir: str) -> None:
        self.control_dir = Path(control_dir)

    def read_nav_snapshot(self) -> NavSnapshot:
        path = self.control_dir / "vehicle_position.json"
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            return NavSnapshot(
                lat=float(payload.get("lat", 0.0) or 0.0),
                lon=float(payload.get("lon", 0.0) or 0.0),
                heading_deg=float(payload.get("heading_deg", 0.0) or 0.0),
                ground_speed_mps=float(payload.get("speed_mps", 0.0) or 0.0),
                valid=True,
                source="sim_vehicle_position",
                age_s=0.0,
            )
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return NavSnapshot(0.0, 0.0, 0.0, 0.0, False, "sim_missing", 999.0)

    def build_snapshot(
        self,
        *,
        lidar: LidarSnapshot,
        camera: CameraSnapshot,
        rc_link_active: bool = True,
        estop_safe: bool = True,
        mavlink_active: bool = True,
    ) -> SensorSnapshot:
        return SensorSnapshot(
            nav=self.read_nav_snapshot(),
            lidar=lidar,
            camera=camera,
            rc_link_active=bool(rc_link_active),
            estop_safe=bool(estop_safe),
            mavlink_active=bool(mavlink_active),
            captured_monotonic=time.monotonic(),
        )

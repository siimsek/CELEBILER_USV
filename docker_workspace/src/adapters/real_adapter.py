"""
Real-hardware adapter boundary.

Real I/O is still performed by the existing MAVLink/ROS readers. This adapter
defines the same snapshot interface as the simulation adapter for incremental
extraction.
"""

from __future__ import annotations

import time

from sensors import CameraSnapshot, LidarSnapshot, NavSnapshot, SensorSnapshot


class RealAdapter:
    def build_snapshot(
        self,
        *,
        nav: NavSnapshot,
        lidar: LidarSnapshot,
        camera: CameraSnapshot,
        rc_link_active: bool,
        estop_safe: bool,
        mavlink_active: bool,
    ) -> SensorSnapshot:
        return SensorSnapshot(
            nav=nav,
            lidar=lidar,
            camera=camera,
            rc_link_active=bool(rc_link_active),
            estop_safe=bool(estop_safe),
            mavlink_active=bool(mavlink_active),
            captured_monotonic=time.monotonic(),
        )

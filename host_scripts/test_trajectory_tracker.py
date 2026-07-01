#!/usr/bin/env python3
"""Unit checks for dynamic obstacle tracking and TTC profiles."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from trajectory_tracker import DynamicObstacleTracker  # noqa: E402
from obstacle_avoidance import speed_profile_for_ttc  # noqa: E402


def main() -> int:
    tracker = DynamicObstacleTracker(match_gate_m=2.0)
    tracker.update([{"x_m": 5.0, "y_m": 0.0, "radius_m": 0.3}], now_s=10.0, boat_speed_mps=1.0)
    tracks = tracker.update([{"x_m": 4.0, "y_m": 0.0, "radius_m": 0.3}], now_s=11.0, boat_speed_mps=1.0)
    if len(tracks) != 1:
        raise AssertionError(f"expected one matched track, got {tracks}")
    track = tracks[0]
    if not track.vx_mps < -0.2:
        raise AssertionError(f"closing velocity not estimated: {track}")
    if track.ttc_s is None or track.ttc_s > 5.0:
        raise AssertionError(f"TTC not computed for closing obstacle: {track}")

    stop = speed_profile_for_ttc(1.0, 0.4)
    if stop.speed_limit_mps != 0.0 or stop.level != "stop":
        raise AssertionError(f"expected stop profile, got {stop}")
    warn = speed_profile_for_ttc(1.0, 4.0)
    if warn.level != "warn" or abs(float(warn.speed_limit_mps) - 0.5) > 1e-6:
        raise AssertionError(f"expected 50% warn profile, got {warn}")

    print("test_trajectory_tracker: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

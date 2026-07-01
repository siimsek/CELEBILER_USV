#!/usr/bin/env python3
"""Unit checks for advanced obstacle avoidance speed shaping."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from obstacle_avoidance import decide_three_sector_avoidance, speed_profile_for_ttc  # noqa: E402


def main() -> int:
    decision = decide_three_sector_avoidance(
        left_m=4.0,
        front_m=1.6,
        right_m=2.0,
        stop_distance_m=1.0,
        warn_distance_m=2.5,
        clear_hysteresis_m=0.4,
    )
    if not decision.active or decision.escape_side != "left" or decision.speed_limit_mps is None:
        raise AssertionError(f"expected left avoidance, got {decision}")

    clear = speed_profile_for_ttc(1.2, None)
    if clear.active or clear.speed_factor != 1.0:
        raise AssertionError(f"clear TTC should not limit speed: {clear}")
    danger = speed_profile_for_ttc(1.2, 2.0)
    if danger.level != "danger" or abs(float(danger.speed_limit_mps) - 0.3) > 1e-6:
        raise AssertionError(f"danger TTC should cap to 25%: {danger}")

    print("test_obstacle_avoidance_advanced: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

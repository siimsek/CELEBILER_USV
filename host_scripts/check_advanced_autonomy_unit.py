#!/usr/bin/env python3
"""Deterministic unit checks for advanced autonomy helpers.

This script does not start simulation or a mission. It only exercises pure
Python costmap/planner logic.
"""

from __future__ import annotations

import time
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from compliance_profile import PLANNER_STUCK_TIMEOUT_S  # noqa: E402
from compliance_profile import COSTMAP_WRONG_TARGET_COST  # noqa: E402
from local_costmap import RollingLocalCostmap  # noqa: E402
from local_planner import PredictiveLocalPlanner  # noqa: E402


def _assert(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def test_costmap_frame_center_obstacle() -> None:
    costmap = RollingLocalCostmap()
    costmap.update_lidar_points([(3.0, 0.0)], quality_ratio=1.0, now_mono=time.monotonic())
    snapshot = costmap.get_snapshot(now_mono=time.monotonic())
    sector = snapshot.sector
    _assert(2.5 <= sector.center_min_m <= 3.5, f"center obstacle not in center sector: {sector}")
    _assert(sector.left_min_m > 5.0, f"front obstacle leaked into left sector: {sector}")
    _assert(sector.right_min_m > 5.0, f"front obstacle leaked into right sector: {sector}")


def test_camera_bearing_right_maps_to_right_sector() -> None:
    costmap = RollingLocalCostmap()
    costmap.update_camera_detections(
        [{"bearing_deg": 25.0, "area_norm": 0.12, "class_name": "wrong_target", "is_wrong_target": True}],
        estimated_range_m=4.0,
        now_mono=time.monotonic(),
    )
    snapshot = costmap.get_snapshot(now_mono=time.monotonic())
    sector = snapshot.sector
    _assert(sector.right_min_m < 6.0, f"camera +right did not map to right sector: {sector}")
    cost = costmap.get_cost_at(4.0 * 0.906, -4.0 * 0.423)
    _assert(cost >= COSTMAP_WRONG_TARGET_COST, f"wrong target did not get high cost: {cost}")


def test_stale_decay_clears_faster_than_fresh_decay() -> None:
    fresh = RollingLocalCostmap(decay_factor=0.9, decay_interval_s=0.1)
    stale = RollingLocalCostmap(decay_factor=0.9, decay_interval_s=0.1)
    t0 = time.monotonic()
    fresh.update_lidar_points([(3.0, 0.0)], quality_ratio=1.0, now_mono=t0)
    stale.update_lidar_points([(3.0, 0.0)], quality_ratio=1.0, now_mono=t0)
    fresh.apply_decay(now_mono=t0 + 0.2)
    stale.apply_decay(now_mono=t0 + 4.0)
    fresh_cost = fresh.get_cost_at(3.0, 0.0)
    stale_cost = stale.get_cost_at(3.0, 0.0)
    _assert(stale_cost < fresh_cost, f"stale decay did not clear faster: stale={stale_cost} fresh={fresh_cost}")


def test_predictive_planner_avoids_center_obstacle() -> None:
    costmap = RollingLocalCostmap()
    costmap.update_lidar_points([(2.0, 0.0)], quality_ratio=1.0, now_mono=time.monotonic())
    snapshot = costmap.get_snapshot(now_mono=time.monotonic()).to_dict()
    planner = PredictiveLocalPlanner()
    decision = planner.decide(
        state="P2_MAPPING_AND_AVOIDANCE",
        current_heading_deg=0.0,
        global_heading_error_deg=0.0,
        base_speed_mps=1.0,
        current_speed_mps=0.2,
        progress_m=1.0,
        distance_m=10.0,
        costmap=costmap,
        costmap_snapshot=snapshot,
        fusion_state={"fusion_confidence": 1.0, "degraded_mode": False, "hold_recommended": False},
    )
    _assert(decision.candidate_count > 3, "planner did not generate candidates")
    _assert(decision.active_behavior in ("AVOID_OBSTACLE", "COLLISION_IMMINENT", "STOP_FOR_OBSTACLE", "DEGRADED_SENSOR"), f"unexpected behavior: {decision}")
    _assert(decision.rejected_collision > 0, "center obstacle did not reject any candidate")


def test_predictive_planner_clear_turn_is_not_obstacle_avoidance() -> None:
    costmap = RollingLocalCostmap()
    costmap.update_lidar_points([(10.0, 5.5)], quality_ratio=1.0, now_mono=time.monotonic())
    snapshot = costmap.get_snapshot(now_mono=time.monotonic()).to_dict()
    planner = PredictiveLocalPlanner()
    decision = planner.decide(
        state="P2_MAPPING_AND_AVOIDANCE",
        current_heading_deg=0.0,
        global_heading_error_deg=25.0,
        base_speed_mps=0.8,
        current_speed_mps=0.3,
        progress_m=1.0,
        distance_m=10.0,
        costmap=costmap,
        costmap_snapshot=snapshot,
        fusion_state={"fusion_confidence": 1.0, "degraded_mode": False, "hold_recommended": False},
    )
    _assert(decision.rejected_collision == 0, f"unexpected collision rejection: {decision}")
    _assert(decision.active_behavior == "FOLLOW_WAYPOINT", f"clear turn misclassified as avoidance: {decision}")


def test_predictive_planner_stuck_recovery() -> None:
    costmap = RollingLocalCostmap()
    costmap.update_lidar_points([(8.0, 4.5)], quality_ratio=1.0, now_mono=time.monotonic())
    snapshot = costmap.get_snapshot(now_mono=time.monotonic()).to_dict()
    planner = PredictiveLocalPlanner()
    planner._last_progress_m = 0.0
    planner._last_distance_m = 12.0
    planner._stuck_since = time.monotonic() - float(PLANNER_STUCK_TIMEOUT_S) - 0.2
    decision = planner.decide(
        state="P2_MAPPING_AND_AVOIDANCE",
        current_heading_deg=0.0,
        global_heading_error_deg=0.0,
        base_speed_mps=1.0,
        current_speed_mps=0.0,
        progress_m=0.0,
        distance_m=12.0,
        costmap=costmap,
        costmap_snapshot=snapshot,
        fusion_state={"fusion_confidence": 1.0, "degraded_mode": False, "hold_recommended": False},
    )
    _assert(decision.stuck_detected, "stuck was not detected")
    _assert(decision.active_behavior == "RECOVERY", f"expected RECOVERY, got {decision.active_behavior}")
    _assert(decision.recovery_action.startswith("switch_corridor_"), f"missing recovery action: {decision.recovery_action}")


def test_predictive_planner_does_not_hold_stale_opposite_turn() -> None:
    costmap = RollingLocalCostmap()
    costmap.update_lidar_points([(7.0, 4.5)], quality_ratio=1.0, now_mono=time.monotonic())
    snapshot = costmap.get_snapshot(now_mono=time.monotonic()).to_dict()
    planner = PredictiveLocalPlanner()
    planner._last_heading_error_deg = -70.0
    planner._last_corridor = "left"
    decision = planner.decide(
        state="P2_MAPPING_AND_AVOIDANCE",
        current_heading_deg=170.0,
        global_heading_error_deg=90.0,
        base_speed_mps=0.8,
        current_speed_mps=0.05,
        progress_m=1.0,
        distance_m=6.0,
        costmap=costmap,
        costmap_snapshot=snapshot,
        fusion_state={"fusion_confidence": 1.0, "degraded_mode": False, "hold_recommended": False},
    )
    _assert(decision.selected_heading_error_deg > 0.0, f"stale opposite turn persisted: {decision}")


def main() -> int:
    test_costmap_frame_center_obstacle()
    test_camera_bearing_right_maps_to_right_sector()
    test_stale_decay_clears_faster_than_fresh_decay()
    test_predictive_planner_avoids_center_obstacle()
    test_predictive_planner_clear_turn_is_not_obstacle_avoidance()
    test_predictive_planner_stuck_recovery()
    test_predictive_planner_does_not_hold_stale_opposite_turn()
    print("check_advanced_autonomy_unit: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

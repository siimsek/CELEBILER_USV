#!/usr/bin/env python3
"""Small deterministic checks for waypoint bearing and GUIDED velocity math."""

from __future__ import annotations

import math
import time
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from nav_guidance import compute_nav_decision  # noqa: E402
from navigation import compass_heading_to_ne_velocity, wrap_180  # noqa: E402
from motor_controller import mix_twin_thrusters  # noqa: E402
from usv_main import HEADING_WRONG_TURN_TRIGGER_S, USVStateMachine  # noqa: E402


def assert_close(actual: float, expected: float, eps: float = 1e-6) -> None:
    if abs(float(actual) - float(expected)) > eps:
        raise AssertionError(f"{actual!r} != {expected!r}")


def assert_sign(value: float, sign: int) -> None:
    if sign < 0 and not float(value) < 0.0:
        raise AssertionError(f"expected negative, got {value!r}")
    if sign > 0 and not float(value) > 0.0:
        raise AssertionError(f"expected positive, got {value!r}")


def main() -> int:
    assert_close(wrap_180(359.0 - 1.0), -2.0)
    assert_close(wrap_180(1.0 - 359.0), 2.0)

    vx, vy = compass_heading_to_ne_velocity(1.0, 0.0)
    assert_close(vx, 1.0)
    assert_close(vy, 0.0)
    vx, vy = compass_heading_to_ne_velocity(1.0, 90.0)
    assert_close(vx, 0.0, 1e-5)
    assert_close(vy, 1.0)
    vx, vy = compass_heading_to_ne_velocity(2.0, 180.0)
    assert_close(vx, -2.0, 1e-5)
    assert_close(vy, 0.0, 1e-5)

    # WP1 -> WP2 geometry from sim/configs/mission_parkour_all.json.
    # At WP1 acceptance the current heading was about 40 deg, and WP2 bearing is
    # about 309 deg, so the shortest commanded turn must stay negative.
    wp2_bearing_from_log = 308.883
    wp2_heading_at_transition = 40.101
    assert_close(wrap_180(wp2_bearing_from_log - wp2_heading_at_transition), -91.218, 1e-3)

    for err, sign in ((120.0, 1), (-120.0, -1), (170.0, 1), (-170.0, -1)):
        decision = compute_nav_decision(
            distance_m=8.0,
            gps_heading_error_deg=err,
            gate_detected=False,
            gate_stable_s=0.0,
            gate_bearing_deg=0.0,
            base_speed_mps=1.0,
            avoidance_bias_deg=0.0,
            front_distance_m=99.0,
            center_distance_m=99.0,
            warn_distance_m=2.5,
            stop_distance_m=2.0,
            gate_stable_threshold_s=1.0,
            failsafe_slow_mps=0.3,
        )
        assert_sign(decision.heading_error_deg, sign)
        if abs(decision.heading_error_deg) < 100.0:
            raise AssertionError(f"large heading was over-clipped: {decision.heading_error_deg}")
        if not math.isfinite(decision.speed_mps):
            raise AssertionError("speed is not finite")

    pos_mix = mix_twin_thrusters(speed_mps=0.4, heading_error_deg=20.0)
    if not pos_mix.right_pwm > pos_mix.left_pwm:
        raise AssertionError(f"positive heading error must drive right harder: {pos_mix}")
    neg_mix = mix_twin_thrusters(speed_mps=0.4, heading_error_deg=-20.0)
    if not neg_mix.left_pwm > neg_mix.right_pwm:
        raise AssertionError(f"negative heading error must drive left harder: {neg_mix}")
    straight_mix = mix_twin_thrusters(speed_mps=0.4, heading_error_deg=0.0)
    if straight_mix.left_pwm != straight_mix.right_pwm or straight_mix.left_pwm <= 1500:
        raise AssertionError(f"straight forward must drive both motors equally forward: {straight_mix}")

    far_obstacle = compute_nav_decision(
        distance_m=6.0,
        gps_heading_error_deg=-35.0,
        gate_detected=False,
        gate_stable_s=0.0,
        gate_bearing_deg=0.0,
        base_speed_mps=0.8,
        avoidance_bias_deg=65.0,
        front_distance_m=12.0,
        center_distance_m=12.0,
        warn_distance_m=2.5,
        stop_distance_m=2.0,
        gate_stable_threshold_s=1.0,
        failsafe_slow_mps=0.3,
    )
    assert_close(far_obstacle.heading_error_deg, -35.0)
    if far_obstacle.mode == "nav_avoid":
        raise AssertionError("far obstacle must not override waypoint heading")

    near_obstacle = compute_nav_decision(
        distance_m=6.0,
        gps_heading_error_deg=-20.0,
        gate_detected=False,
        gate_stable_s=0.0,
        gate_bearing_deg=0.0,
        base_speed_mps=0.8,
        avoidance_bias_deg=35.0,
        front_distance_m=1.85,
        center_distance_m=1.85,
        warn_distance_m=2.5,
        stop_distance_m=2.0,
        gate_stable_threshold_s=1.0,
        failsafe_slow_mps=0.3,
    )
    if near_obstacle.mode != "nav_avoid":
        raise AssertionError("near confirmed obstacle must enter bounded avoidance")
    if not -95.0 <= near_obstacle.heading_error_deg <= 95.0:
        raise AssertionError(f"avoidance heading must remain bounded: {near_obstacle}")

    usv = object.__new__(USVStateMachine)
    usv.current_yaw_rate_dps = 2.0
    usv._nav_align_mode = "advance"
    usv._wrong_turn_since = time.monotonic() - float(HEADING_WRONG_TURN_TRIGGER_S) - 0.1
    usv._wrong_turn_active = False
    usv._wrong_turn_count = 0
    usv._wrong_turn_last_log_ts = 0.0
    usv.heading_control_diagnostic = "nominal"
    err, speed, guarded = USVStateMachine._apply_wrong_turn_guard(usv, -8.0, 0.42)
    assert_close(err, -8.0)
    assert_close(speed, 0.42)
    if guarded:
        raise AssertionError("GUIDED wrong-turn diagnostic must not zero speed")
    if not str(usv.heading_control_diagnostic).startswith("wrong_turn_observed"):
        raise AssertionError("wrong-turn diagnostic was not exported")


    if USVStateMachine._allow_simple_avoidance_bias(True, False):
        raise AssertionError("simple avoidance must be blocked during turn priority")
    if not USVStateMachine._allow_simple_avoidance_bias(True, True):
        raise AssertionError("lidar emergency must allow simple avoidance during turn priority")
    if not USVStateMachine._allow_simple_avoidance_bias(False, False):
        raise AssertionError("simple avoidance must remain allowed outside turn priority")

    pivot_speed, pivot_reason = USVStateMachine._resolve_nav_align_pivot_speed(
        True,
        -94.0,
        2.0,
        0.45,
    )
    if pivot_speed != 0.0 or pivot_reason != "nav_align_pivot_hold":
        raise AssertionError(f"WP2-like turn must pivot in place near obstacle: {pivot_speed} {pivot_reason}")
    creep_speed, creep_reason = USVStateMachine._resolve_nav_align_pivot_speed(
        True,
        -94.0,
        99.0,
        0.45,
    )
    if creep_reason == "nav_align_yaw_only":
        if creep_speed != 0.0:
            raise AssertionError(f"large turn yaw-only must hold speed at zero: {creep_speed} {creep_reason}")
    elif creep_reason == "nav_align_turn_creep":
        if creep_speed <= 0.0:
            raise AssertionError(f"large turn optional creep must keep a bounded crawl: {creep_speed} {creep_reason}")
    else:
        raise AssertionError(f"large turn with clear front must pivot or optional creep: {creep_speed} {creep_reason}")

    if USVStateMachine._allow_local_minima_boost(True, -94.0):
        raise AssertionError("local minima boost must be suppressed during turn priority")
    if USVStateMachine._allow_local_minima_boost(False, -94.0, "align"):
        raise AssertionError("local minima boost must be suppressed during align mode")
    if USVStateMachine._allow_local_minima_boost(False, -94.0, "advance"):
        raise AssertionError("local minima boost must be suppressed during large heading error")
    if not USVStateMachine._allow_local_minima_boost(False, -5.0, "advance"):
        raise AssertionError("local minima boost may apply once heading is acquired")


    usv_ct = object.__new__(USVStateMachine)
    right_drift_corr = USVStateMachine._nav_cross_track_correction_deg(usv_ct, -2.0, 6.0)
    if not float(right_drift_corr) < 0.0:
        raise AssertionError(f"right-side drift must produce negative CT correction: {right_drift_corr}")

    align_heading, blend, align_active = USVStateMachine._apply_cross_track_to_heading(
        -94.0,
        -2.0,
        right_drift_corr,
        align_follow=True,
        in_warn=False,
        lidar_emergency=False,
    )
    if not align_active:
        raise AssertionError("align cross-track must activate on lateral offset during large turn")
    if abs(float(blend)) > 12.01:
        raise AssertionError(f"align cross-track blend must stay capped: {blend}")
    if not float(align_heading) < -94.0:
        raise AssertionError(f"right drift must pull heading left during align: {align_heading}")

    blocked_heading, blocked_blend, blocked_active = USVStateMachine._apply_cross_track_to_heading(
        -94.0,
        -2.0,
        right_drift_corr,
        align_follow=True,
        in_warn=True,
        lidar_emergency=False,
    )
    if blocked_active or blocked_blend != 0.0 or blocked_heading != -94.0:
        raise AssertionError("cross-track must stay off during lidar warn")

    small_ct_heading, small_blend, small_active = USVStateMachine._apply_cross_track_to_heading(
        -94.0,
        0.4,
        1.0,
        align_follow=True,
        in_warn=False,
        lidar_emergency=False,
    )
    if small_active or small_blend != 0.0:
        raise AssertionError("small lateral offset must not trigger align cross-track")

    from navigation import heading_first_waypoint_request  # noqa: E402

    strict_turn = heading_first_waypoint_request(
        distance_m=8.0,
        heading_error_deg=35.0,
        cruise_speed_mps=1.0,
        approach_speed_mps=0.8,
        creep_speed_mps=0.0,
    )
    if strict_turn.phase != "TURN_TO_WAYPOINT":
        raise AssertionError(f"expected turn phase, got {strict_turn}")
    if strict_turn.speed_mps != 0.0:
        raise AssertionError(f"strict misaligned turn must not surge: {strict_turn.speed_mps}")
    if strict_turn.reason != "heading_first_yaw_only":
        raise AssertionError(f"unexpected strict reason: {strict_turn.reason}")

    print("nav_guidance_unit: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

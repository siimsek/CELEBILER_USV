#!/usr/bin/env python3
"""Deterministic unit checks for mission profile, race gates, P2/P3 rules."""

from __future__ import annotations

import json
import os
import sys
import time
import traceback
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
REPORT_PATH = ROOT / "logs" / "compliance" / "mission_profile_unit_result.json"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from mission_config import (  # noqa: E402
    MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT,
    build_mission_profile,
    parse_mission_profile_payload,
)


def _fail(message: str) -> None:
    raise AssertionError(message)


def _write_report(*, success: bool, tests: int = 0, error: str = "") -> None:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {"success": success, "tests": tests, "error": error}
    REPORT_PATH.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def _load_usv_state_machine():
    from usv_main import USVStateMachine, normalize_heading_error  # noqa: WPS433

    return USVStateMachine, normalize_heading_error


def test_normalize_heading_wrap() -> None:
    _, normalize_heading_error = _load_usv_state_machine()
    assert abs(normalize_heading_error(359.0 - 1.0) - (-2.0)) < 1e-6
    assert abs(normalize_heading_error(1.0 - 359.0) - 2.0) < 1e-6


def test_flat_mission_not_race_ready() -> None:
    parsed = parse_mission_profile_payload([[40.89, 29.02], [40.891, 29.021]])
    if parsed.get("race_ready"):
        _fail("flat mission must not be race_ready")
    if parsed.get("mission_profile_valid"):
        _fail("flat mission must not have valid mission_profile")


def test_flat_mission_race_required_rejected() -> None:
    try:
        parse_mission_profile_payload([[40.89, 29.02]], race_required=True)
    except ValueError:
        return
    _fail("flat mission with race_required=True must raise ValueError")


def test_structured_profile_race_ready() -> None:
    payload = {
        "waypoints": [[40.89, 29.02], [40.891, 29.021]],
        "mission_profile": build_mission_profile(
            target_color="RED",
            upload_source="pixhawk_mission",
            validation_timestamp=time.time(),
        ),
    }
    parsed = parse_mission_profile_payload(payload, race_required=True)
    if not parsed.get("race_ready"):
        _fail("structured profile must be race_ready")
    if parsed.get("target_color") != "RED":
        _fail("structured profile must preserve target_color")
    if parsed.get("engage_wp") is not None:
        _fail("P3 must not require engage_wp waypoint")


def test_p2_min_gate_count_enforced() -> None:
    try:
        build_mission_profile(
            target_color="RED",
            upload_source="pixhawk_mission",
            validation_timestamp=time.time(),
            p2_min_gate_count=1,
        )
    except ValueError:
        return
    _fail(f"p2_min_gate_count < {MISSION_PROFILE_DEFAULT_P2_MIN_GATE_COUNT} must raise")


def test_p3_multi_cue_quorum() -> None:
    USVStateMachine, _ = _load_usv_state_machine()
    sm = USVStateMachine.__new__(USVStateMachine)
    sm.current_speed_mps = 0.5
    sm.engage_wp = [40.89, 29.02]

    ok_single, sources_single, _ = USVStateMachine._p3_contact_evidence(
        sm,
        target_detected=True,
        target_area=0.25,
        dist_m=5.0,
        lidar_center_m=99.0,
        lidar_ready=True,
        collision_stuck=False,
        wrong_target_contact_risk=False,
        wrong_target_detected=False,
        wrong_target_area=0.0,
        wrong_target_bearing=0.0,
        wrong_target_class="",
    )
    if ok_single:
        _fail("single vision_area must not satisfy P3 contact quorum")

    ok_dual, sources_dual, _ = USVStateMachine._p3_contact_evidence(
        sm,
        target_detected=True,
        target_area=0.25,
        dist_m=0.8,
        lidar_center_m=99.0,
        lidar_ready=True,
        collision_stuck=False,
        wrong_target_contact_risk=False,
        wrong_target_detected=False,
        wrong_target_area=0.0,
        wrong_target_bearing=0.0,
        wrong_target_class="",
    )
    if not ok_dual or "vision_area" not in sources_dual or "gps_proximity" not in sources_dual:
        _fail("vision_area + gps_proximity must satisfy quorum")


def test_p3_wrong_target_blocks_quorum() -> None:
    USVStateMachine, _ = _load_usv_state_machine()
    sm = USVStateMachine.__new__(USVStateMachine)
    sm.current_speed_mps = 0.1
    sm.engage_wp = [40.89, 29.02]

    ok, _, _ = USVStateMachine._p3_contact_evidence(
        sm,
        target_detected=True,
        target_area=0.30,
        dist_m=0.5,
        lidar_center_m=0.5,
        lidar_ready=True,
        collision_stuck=True,
        wrong_target_contact_risk=True,
        wrong_target_detected=True,
        wrong_target_area=0.12,
        wrong_target_bearing=5.0,
        wrong_target_class="YESIL_SANCAK",
    )
    if ok:
        _fail("wrong_target_contact_risk must block P3 contact quorum")


def test_race_start_fail_closed_snippets() -> None:
    usv_src = (SRC / "usv_main.py").read_text(encoding="utf-8")
    required = [
        "mission_profile_not_race_ready",
        "target_color_not_locked",
        "invalid_p3_engagement_mode",
        'mission_upload_source != "pixhawk_mission"',
        "race_p1_completion_source",
        "if USV_MODE == USV_MODE_RACE and not p1_auto_env",
    ]
    for snippet in required:
        if snippet not in usv_src:
            _fail(f"usv_main missing race gate snippet: {snippet}")


def main() -> int:
    tests = [
        test_flat_mission_not_race_ready,
        test_flat_mission_race_required_rejected,
        test_structured_profile_race_ready,
        test_p2_min_gate_count_enforced,
        test_normalize_heading_wrap,
        test_p3_multi_cue_quorum,
        test_p3_wrong_target_blocks_quorum,
        test_race_start_fail_closed_snippets,
    ]
    for test in tests:
        test()
    _write_report(success=True, tests=len(tests))
    return 0


if __name__ == "__main__":
    try:
        code = main()
    except AssertionError as exc:
        _write_report(success=False, error=str(exc))
        os._exit(1)
    except Exception:
        _write_report(success=False, error=traceback.format_exc())
        os._exit(1)
    os._exit(code)

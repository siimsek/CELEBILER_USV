#!/usr/bin/env python3
"""
Kontrol-1: Statik uyum kontrolu.
Rapor esik/frekans/mode ve endpoint kurallarinin kodda bulundugunu dogrular.
"""

from __future__ import annotations

import ast
import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
REPORT_DIR = Path(os.environ.get("COMPLIANCE_REPORT_DIR", str(ROOT / "logs" / "system" / "compliance")))
REPORT_MD = REPORT_DIR / "compliance_report.md"
REPORT_JSON = REPORT_DIR / "compliance_report.json"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def has(text: str, needle: str) -> bool:
    return needle in text


def main() -> int:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)

    profile_py = read(SRC / "compliance_profile.py")
    mission_cfg_py = read(SRC / "mission_config.py")
    mission_adapter_py = read(SRC / "mission_adapter.py")
    usv_py = read(SRC / "usv_main.py")
    cam_py = read(SRC / "cam.py")
    telem_py = read(SRC / "telemetry.py")
    sim_stack = read(ROOT / "sim" / "bin" / "run_sim_stack.sh")

    checks = {
        "mode_default_test": has(profile_py, "return USV_MODE_RACE if raw == USV_MODE_RACE else USV_MODE_TEST"),
        "threshold_r_wp": has(profile_py, "R_WP_M = 2.5"),
        "threshold_t_hold": has(profile_py, "T_HOLD_S = 2.0"),
        "threshold_d_min": has(profile_py, "D_MIN_M = 1.2"),
        "threshold_timeout": has(profile_py, "P3_TIMEOUT_S = 180"),
        "threshold_retry": has(profile_py, "P3_RETRY_S = 60"),
        "threshold_heartbeat_warn": has(profile_py, "HEARTBEAT_WARN_S = 5.0"),
        "threshold_heartbeat_fail": has(profile_py, "HEARTBEAT_FAIL_S = 30.0"),
        "mission_input_flat_profile": has(profile_py, "MISSION_INPUT_FORMAT = \"flat_ordered\"") and has(profile_py, "MISSION_SPLIT_P2_COUNT") and has(profile_py, "MISSION_SPLIT_P3_COUNT"),
        "mission_split_config_used": has(mission_cfg_py, "def get_mission_split_profile") and has(mission_cfg_py, "p1_end = total - MISSION_SPLIT_P2_COUNT - MISSION_SPLIT_P3_COUNT"),
        "legacy_structured_guarded": has(mission_adapter_py, "Structured mission payload disabled") and has(mission_adapter_py, "MISSION_ALLOW_STRUCTURED_LEGACY"),
        "telemetry_general_5hz": has(telem_py, "MAV_DATA_STREAM_ALL, GENERAL_TELEMETRY_HZ, 1"),
        "telemetry_rc_5hz": has(telem_py, "MAV_DATA_STREAM_RC_CHANNELS, GENERAL_TELEMETRY_HZ, 1"),
        "health_gate_ready": has(usv_py, "if not self.health_ready:"),
        "health_state_written": has(usv_py, "\"health_check\": {"),
        "manual_next_removed": not has(telem_py, "/api/next_parkur"),
        "auto_transition_enforced": has(usv_py, "otomatik gecis (kullanici girdisi kapali)"),
        "report_view_api": has(telem_py, "out['report_view'] = {"),
        "p1_mode_contract_visible": has(usv_py, "p1_mode = \"GUIDED\" if USV_MODE != USV_MODE_RACE else \"AUTO\"") and has(usv_py, "Pure Pixhawk AUTO mode"),
        "p2_ready_wait_visible": has(usv_py, "[WAIT] [P1->P2] Bekleniyor camera_ready="),
        "p2_guidance_priority_visible": has(usv_py, "Emergency: 100% avoidance") and has(usv_py, "gate bearing preferred") and has(usv_py, "pure waypoint bearing"),
        "sim_startup_blind_wait_removed": not has(sim_stack, "Waiting extra 12 sec for API services"),
        "sim_startup_api_probe": has(sim_stack, "camera_api_ready=True") and has(sim_stack, "probe_camera_endpoint"),
        "shared_sim_nav_helper": has(telem_py, "from sim_nav_state import load_sim_nav_state"),
        "sim_sensor_shortcut_enabled": has(usv_py, "self.camera_ready = True") and has(usv_py, "self.lidar_ready = True"),
        "objective_phase_written": has(usv_py, "def _get_objective_phase(self):") and has(usv_py, "\"objective_phase\": objective_phase"),
        "perception_policy_written": has(usv_py, "def _get_perception_policy(self):") and has(usv_py, "\"perception_policy\": perception_policy"),
        "camera_actionable_policy_visible": has(cam_py, "gate_actionable = bool(policy.get(\"gate\", False))") and has(cam_py, "yellow_actionable = bool(policy.get(\"yellow_obstacle\", False))") and has(cam_py, "target_actionable = bool(policy.get(\"target\", False))"),
        "p1_target_semantics_removed": has(usv_py, "esc_c = 0.0") and has(usv_py, "target-color semantics belong to later phases"),
        "p3_target_guidance_present": has(usv_py, "if target_detected:") and has(usv_py, "heading_err = target_bearing"),
        "mission_api_writes_flat": has(telem_py, "json.dump(flat_mission, f, indent=2)") and has(telem_py, "'input_format': MISSION_INPUT_FORMAT"),
        "runtime_loader_uses_target_state": has(usv_py, "load_target_state(TARGET_STATE_FILE)") and has(usv_py, "self.target_color = target_from_state or mission_target_color or self.target_color or \"RED\""),
        "mission_state_contract_visible": has(usv_py, "\"mission_input_format\": self.mission_input_format") and has(usv_py, "\"mission_split_profile\": dict(self.mission_split_profile)"),
        "gear_ui_removed": not has(telem_py, "gear_disp") and not has(telem_py, "<div class=\"label\">Gear</div>"),
        "gear_logic_removed": not has(telem_py, "input_gear") and not has(telem_py, "telemetry_data[\"Gear\"]") and not has(telem_py, "msg.chan6_raw"),
        "manual_control_signed_thrust": has(telem_py, "desired_throttle_pwm = 1500 + math.copysign") and has(telem_py, "self.motor_ctrl.update_inputs(msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw)"),
        "target_color_api_locked_post_start": has(telem_py, "Görev aktifken target_color güncellenemez.") and has(telem_py, "return jsonify({'error': 'Görev aktifken target_color güncellenemez.'}), 409"),
    }

    passed = sum(1 for ok in checks.values() if ok)
    total = len(checks)
    success = passed == total

    payload = {
        "control": "Kontrol-1",
        "passed": passed,
        "total": total,
        "success": success,
        "checks": checks,
    }
    REPORT_JSON.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    md_lines = [
        "# Compliance Static Check",
        "",
        f"- Result: {'PASS' if success else 'FAIL'}",
        f"- Score: {passed}/{total}",
        "",
        "| Check | Status |",
        "|---|---|",
    ]
    for name, ok in checks.items():
        md_lines.append(f"| {name} | {'PASS' if ok else 'FAIL'} |")
    REPORT_MD.write_text("\n".join(md_lines) + "\n", encoding="utf-8")

    print(json.dumps(payload, ensure_ascii=False))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())

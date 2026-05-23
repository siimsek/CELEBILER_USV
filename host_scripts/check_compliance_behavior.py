#!/usr/bin/env python3
"""
Kontrol-2: Davranissal uyum kontrolu (dry-run, bagimsiz).
API/akis davranislarini koddan dogrular:
- otomatik parkur gecisi
- race/start kurallari
- race goruntu aktarimi kapatma
- health/ready alanlarinin API'ye cikisi
"""

from __future__ import annotations

import json
import subprocess
import sys
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
REPORT_DIR = Path(os.environ.get("COMPLIANCE_REPORT_DIR", str(ROOT / "logs" / "system" / "compliance")))
REPORT_JSON = REPORT_DIR / "compliance_behavior_report.json"
REPORT_MD = REPORT_DIR / "compliance_behavior_report.md"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def has(text: str, needle: str) -> bool:
    return needle in text


def main() -> int:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)

    usv = read(SRC / "usv_main.py")
    telemetry = read(SRC / "telemetry.py")
    cam = read(SRC / "cam.py")
    lidar = read(SRC / "lidar_map.py")
    profile = read(SRC / "compliance_profile.py")
    sim_stack = read(ROOT / "sim" / "bin" / "run_sim_stack.sh")
    removed_decision_token = "m" + "oos"

    checks = {
        "parkur_transition_auto_only": has(usv, "state = self.STATE_ENGAGE") and has(usv, "otomatik gecis (kullanici girdisi kapali)"),
        "manual_next_flag_removed": ("FLAG_NEXT" not in usv) and ("/api/next_parkur" not in telemetry),
        "mission_start_ready_gate": has(usv, "HEALTH_CHECK gecmedi"),
        "race_start_api_blocked": has(telemetry, "if USV_MODE == USV_MODE_RACE"),
        "race_start_button_disabled": has(telemetry, "d.usv_mode === 'race'"),
        "dashboard_poll_5hz": has(telemetry, "setInterval(updateStats, 200);"),
        "api_includes_health": has(telemetry, "out['health_check'] ="),
        "api_includes_report_view": has(telemetry, "out['report_view'] ="),
        "camera_race_web_disabled": has(cam, "if RACE_MODE:") and has(cam, "Race modda sadece onboard isleme"),
        "lidar_race_exit": has(lidar, "YARIŞMA MODU - Harita yayını başlatılmıyor.") and has(lidar, "sys.exit(0)"),
        "nav_runs_in_auto": has(usv, "[NAV] WAYPOINT TAKIBI") and has(usv, "auto_result = self._run_nav_auto_mission()"),
        "p2_ready_gate_has_no_timeout_bypass": not has(usv, "Sensor bekleme zamani doldu") and has(usv, "mode=AUTO speed_cap<="),
        "p2_guidance_order_logged": has(usv, 'self._set_guidance_source("p2_avoid")') and has(usv, 'self._set_guidance_source("p2_gate")') and has(usv, 'self._set_guidance_source("p2_waypoint_fallback")'),
        "p3_wrong_target_avoidance_logged": has(usv, "P3_WRONG_TARGET_AVOID") and has(usv, "wrong_target_contact_risk") and has(cam, "wrong_target_detected"),
        "p3_contact_confirmation_exported": has(usv, "p3_contact_confirmation_source") and has(usv, '"p3_contact_confirmation_source":'),
        "p3_target_lock_verified_before_engage": has(usv, "def _validate_p3_target_lock") and has(usv, "p3_target_color_lock_rejected") and has(usv, "target_color_changed_after_start"),
        "p3_contact_requires_multi_cue_quorum": has(usv, "def _p3_contact_evidence") and has(usv, "primary_source_count") and has(usv, "P3_CONTACT_LATCH") and has(usv, "len(primary_sources) >= 2"),
        "p3_wrong_target_blocks_contact_latch": has(usv, 'if not evidence["wrong_target_contact_risk"]') and has(usv, "wrong_target_area_norm") and has(usv, "wrong_target_bearing_deg"),
        "startup_wait_is_probe_based": has(sim_stack, "probe_camera_endpoint") and not has(sim_stack, "sleep 12"),
        "shared_sim_nav_source_used": has(usv, "load_sim_nav_state(control_dir=CONTROL_DIR)") and has(telemetry, "load_sim_nav_state(control_dir=CONTROL_DIR)"),
        "sim_forced_sensor_ready_removed": not has(usv, "self.camera_ready = True") and not has(usv, "self.lidar_ready = True"),
        "nav_diagnostics_exported": has(usv, '"nav_position_source":') and has(telemetry, "out['nav_position_source']") and has(telemetry, "'nav_target_bearing_deg': out['nav_target_bearing_deg']"),
        "nav_invalid_blocks_heading_solution": has(usv, "if not self.nav_fix_valid:") and has(usv, "self._log_nav_invalid(target_lat, target_lon)"),
        "waypoint_accept_profile_aligned": has(usv, "arrival_radius = float(R_WP_M)") and has(usv, "hold_required_s = float(T_HOLD_S)"),
        "waypoint_reached_no_active_disarm": has(usv, "def _hold_waypoint_neutral") and has(usv, 'self._hold_waypoint_neutral("waypoint_reached")'),
        "external_decision_runtime_removed": removed_decision_token not in usv.lower() and not (SRC / f"{removed_decision_token}_decision_layer.py").exists(),
        "external_decision_telemetry_removed": removed_decision_token not in telemetry.lower() and removed_decision_token not in profile.lower(),
        "startup_external_gate_removed": removed_decision_token not in sim_stack.lower(),
        "telemetry_exposes_mp_gcs_heartbeat_age": has(telemetry, "out['mission_planner_gcs_age_s']"),
    }

    unit_script = ROOT / "host_scripts" / "check_mission_profile_unit.py"
    unit_ok = False
    if unit_script.is_file():
        unit_result = subprocess.run([sys.executable, str(unit_script)], cwd=str(ROOT), check=False)
        unit_ok = unit_result.returncode == 0
    checks["mission_profile_unit_tests_pass"] = unit_ok

    passed = sum(1 for ok in checks.values() if ok)
    total = len(checks)
    success = passed == total

    payload = {
        "control": "Kontrol-2",
        "passed": passed,
        "total": total,
        "success": success,
        "checks": checks,
    }
    REPORT_JSON.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    md_lines = [
        "# Compliance Behavior Check",
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

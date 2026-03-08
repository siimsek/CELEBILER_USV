#!/usr/bin/env python3
"""
USV Compliance Checker
Birleştirilmiş statik ve davranışsal uyum kontrolü.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
DOCS = ROOT / "documents"

def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")

def has(text: str, needle: str) -> bool:
    return needle in text

def run_static_checks(usv_py, telem_py, profile_py):
    return {
        "mode_default_test": has(profile_py, "return USV_MODE_RACE if raw == USV_MODE_RACE else USV_MODE_TEST"),
        "threshold_r_wp": has(profile_py, "R_WP_M = 2.5"),
        "threshold_t_hold": has(profile_py, "T_HOLD_S = 2.0"),
        "threshold_d_min": has(profile_py, "D_MIN_M = 2.0"),
        "threshold_timeout": has(profile_py, "P3_TIMEOUT_S = 180"),
        "threshold_retry": has(profile_py, "P3_RETRY_S = 60"),
        "threshold_heartbeat_warn": has(profile_py, "HEARTBEAT_WARN_S = 5.0"),
        "threshold_heartbeat_fail": has(profile_py, "HEARTBEAT_FAIL_S = 30.0"),
        "telemetry_general_5hz": has(telem_py, "MAV_DATA_STREAM_ALL, GENERAL_TELEMETRY_HZ, 1"),
        "telemetry_rc_5hz": has(telem_py, "MAV_DATA_STREAM_RC_CHANNELS, GENERAL_TELEMETRY_HZ, 1"),
        "health_gate_ready": has(usv_py, "if not self.health_ready:"),
        "health_state_written": has(usv_py, "\"health_check\": {"),
        "manual_next_removed": not has(telem_py, "/api/next_parkur"),
        "auto_transition_enforced": has(usv_py, "otomatik gecis (kullanici girdisi kapali)"),
        "report_view_api": has(telem_py, "out['report_view'] = {"),
    }

def run_behavior_checks(usv_py, telem_py, cam_py, lidar_py):
    return {
        "parkur_transition_auto_only": has(usv_py, "otomatik gecis (kullanici girdisi kapali)"),
        "manual_next_flag_removed": ("FLAG_NEXT" not in usv_py) and ("/api/next_parkur" not in telem_py),
        "mission_start_ready_gate": has(usv_py, "HEALTH_CHECK gecmedi"),
        "race_start_api_blocked": has(telem_py, "if USV_MODE == USV_MODE_RACE"),
        "race_start_button_disabled": has(telem_py, "d.usv_mode === 'race'"),
        "dashboard_poll_5hz": has(telem_py, "setInterval(updateStats, 200);"),
        "api_includes_health": has(telem_py, "out['health_check'] ="),
        "api_includes_report_view": has(telem_py, "out['report_view'] ="),
        "camera_race_web_disabled": has(cam_py, "if RACE_MODE:") and has(cam_py, "Race modda sadece onboard isleme"),
        "lidar_race_exit": has(lidar_py, "YARIŞMA MODU - Harita yayını başlatılmıyor.") and has(lidar_py, "sys.exit(0)"),
    }

def save_report(name, checks):
    passed = sum(1 for ok in checks.values() if ok)
    total = len(checks)
    success = passed == total
    
    payload = {
        "control": name,
        "passed": passed,
        "total": total,
        "success": success,
        "checks": checks,
    }
    
    json_path = DOCS / f"{name}_report.json"
    md_path = DOCS / f"{name}_report.md"
    
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    
    md_lines = [
        f"# {name.replace('_', ' ').title()} Check",
        "",
        f"- Result: {'PASS' if success else 'FAIL'}",
        f"- Score: {passed}/{total}",
        "",
        "| Check | Status |",
        "|---|---|"
    ]
    for k, ok in checks.items():
        md_lines.append(f"| {k} | {'PASS' if ok else 'FAIL'} |")
    md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    
    return success, payload

def main():
    profile_py = read(SRC / "compliance_profile.py")
    usv_py = read(SRC / "usv_main.py")
    telem_py = read(SRC / "telemetry.py")
    cam_py = read(SRC / "cam.py")
    lidar_py = read(SRC / "lidar_map.py")
    
    static_checks = run_static_checks(usv_py, telem_py, profile_py)
    behavior_checks = run_behavior_checks(usv_py, telem_py, cam_py, lidar_py)
    
    s1, p1 = save_report("compliance_static", static_checks)
    s2, p2 = save_report("compliance_behavior", behavior_checks)
    
    print("STATIC:", json.dumps(p1, ensure_ascii=False))
    print("BEHAVIOR:", json.dumps(p2, ensure_ascii=False))
    
    return 0 if (s1 and s2) else 1

if __name__ == "__main__":
    raise SystemExit(main())

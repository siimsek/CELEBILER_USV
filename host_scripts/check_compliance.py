#!/usr/bin/env python3
"""
USV Compliance Checker
Birleştirilmiş statik ve davranışsal uyum kontrolü.
"""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
REPORT_DIR = Path(os.environ.get("COMPLIANCE_REPORT_DIR", str(ROOT / "logs" / "system" / "compliance")))

def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")

def has(text: str, needle: str) -> bool:
    return needle in text

def run_static_checks(usv_py, telem_py, profile_py, cam_py):
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
        "mission_upload_api": has(telem_py, "@app.route('/api/mission', methods=['POST'])"),
        "target_color_api": has(telem_py, "@app.route('/api/target_color', methods=['POST'])"),
        "mission_validation": has(usv_py, "def validate_mission_schema") and has(usv_py, "def validate_parkur_waypoints") and has(usv_py, "def validate_target_color"),
        "command_lock_enforced": has(usv_py, "if self.command_lock:") and has(usv_py, "self._set_rc_override(1500, 1500)"),
        "camera_recording": has(cam_py, "def _check_mission_state") and has(cam_py, "_recording_active"),
    }

def check_mission_files():
    """Mission dosyalarının geçerli JSON ve schema'ya uygun olduğunu kontrol et."""
    checks = {}
    mission_files = [
        ROOT / "docker_workspace" / "mission.json",
        ROOT / "sim" / "configs" / "mission_parkour_all.json",
        ROOT / "sim" / "configs" / "mission_parkour1.json",
        ROOT / "sim" / "configs" / "mission_parkour2.json",
        ROOT / "sim" / "configs" / "mission_parkour3_red.json",
        ROOT / "sim" / "configs" / "mission_parkour3_green.json",
        ROOT / "sim" / "configs" / "mission_parkour3_black.json",
    ]
    
    for mission_file in mission_files:
        file_name = mission_file.name
        
        # Dosya varlığı kontrolü
        if not mission_file.exists():
            checks[f"file_exists_{file_name}"] = False
            continue
        
        # JSON validitesi kontrolü
        try:
            with open(mission_file, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, IOError) as e:
            checks[f"json_valid_{file_name}"] = False
            continue
        
        checks[f"json_valid_{file_name}"] = True
        
        # Schema kontrolü (zorunlu alanlar)
        required_fields = ["parkur1", "parkur2", "parkur3", "target_color"]
        has_all_fields = all(field in data for field in required_fields)
        checks[f"schema_complete_{file_name}"] = has_all_fields
        
        if not has_all_fields:
            continue
        
        # Parkur boş olmama kontrolü
        checks[f"parkur_not_empty_{file_name}"] = (
            len(data.get("parkur1", [])) > 0 and
            len(data.get("parkur2", [])) > 0 and
            len(data.get("parkur3", [])) > 0
        )
        
        # Parkur3 max 2 waypoint kontrolü
        checks[f"parkur3_max_2_{file_name}"] = len(data.get("parkur3", [])) <= 2
        
        # Geçerli renk kontrolü
        valid_colors = ["RED", "GREEN", "BLACK", "KIRMIZI_SANCAK", "YESIL_SANCAK", "SIYAH_HEDEF"]
        checks[f"target_color_valid_{file_name}"] = data.get("target_color", "") in valid_colors
    
    return checks


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
    
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    json_path = REPORT_DIR / f"{name}_report.json"
    md_path = REPORT_DIR / f"{name}_report.md"
    
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
    
    static_checks = run_static_checks(usv_py, telem_py, profile_py, cam_py)
    behavior_checks = run_behavior_checks(usv_py, telem_py, cam_py, lidar_py)
    mission_checks = check_mission_files()
    
    s1, p1 = save_report("compliance_static", static_checks)
    s2, p2 = save_report("compliance_behavior", behavior_checks)
    s3, p3 = save_report("compliance_mission", mission_checks)
    
    print("STATIC:", json.dumps(p1, ensure_ascii=False))
    print("BEHAVIOR:", json.dumps(p2, ensure_ascii=False))
    print("MISSION:", json.dumps(p3, ensure_ascii=False))
    
    return 0 if (s1 and s2 and s3) else 1

if __name__ == "__main__":
    raise SystemExit(main())

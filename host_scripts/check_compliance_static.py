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
    nav_guidance_py = read(SRC / "nav_guidance.py")
    cam_py = read(SRC / "cam.py")
    telem_py = read(SRC / "telemetry.py")
    console_utils_py = read(SRC / "console_utils.py")
    sim_stack = read(ROOT / "sim" / "bin" / "run_sim_stack.sh")
    check_stack = read(ROOT / "sim" / "bin" / "check_stack.sh")
    sim_bridge = read(ROOT / "sim" / "bridges" / "sitl_gazebo_bridge.py")
    system_start = read(ROOT / "host_scripts" / "system_start.sh")
    system_stop = read(ROOT / "host_scripts" / "system_stop.sh")
    internal_start = read(ROOT / "docker_workspace" / "scripts" / "internal_start.sh")
    compliance_wrapper = read(ROOT / "host_scripts" / "check_compliance.py")
    ida_sartname_doc = read(ROOT / "documents" / "ida_sartname.md")
    rapor_doc = read(ROOT / "documents" / "rapor_calismasistemi.md")
    readme_doc = read(ROOT / "README.md")

    extract_body = ""
    dl_body = ""
    _ext_start = usv_py.find("def _extract_mavlink_waypoint")
    _ext_end = usv_py.find("def _request_mission_item_int", _ext_start + 1)
    if _ext_start != -1 and _ext_end != -1:
        extract_body = usv_py[_ext_start:_ext_end]
    _dl_start = usv_py.find("def _download_pixhawk_mission")
    _dl_end = usv_py.find("def _apply_mission_coords", _dl_start + 1)
    if _dl_start != -1 and _dl_end != -1:
        dl_body = usv_py[_dl_start:_dl_end]
    _cleanup_start = sim_stack.find("cleanup() {")
    _cleanup_end = sim_stack.find("trap cleanup", _cleanup_start + 1)
    sim_cleanup_body = ""
    if _cleanup_start != -1 and _cleanup_end != -1:
        sim_cleanup_body = sim_stack[_cleanup_start:_cleanup_end]
    idx_no_nav_guard = usv_py.find('if str(self.pixhawk_mission_sync_error or "") == "no_nav_waypoints_extracted"')
    idx_sim_fb = usv_py.find("# Fallback: generate default mission")

    checks = {
        "mode_default_test": has(profile_py, "return USV_MODE_RACE if raw == USV_MODE_RACE else USV_MODE_TEST"),
        "threshold_r_wp": has(profile_py, "R_WP_M = max(0.5") or has(profile_py, "R_WP_M = 2.5"),
        "threshold_t_hold": has(profile_py, "T_HOLD_S = 2.0"),
        "threshold_d_min": has(profile_py, "D_MIN_M = 2.0"),
        "threshold_p2_lidar_warn": has(profile_py, "P2_LIDAR_WARN_M = 2.5"),
        "threshold_timeout": has(profile_py, "P3_TIMEOUT_S = 180"),
        "threshold_retry": has(profile_py, "P3_RETRY_S = 60"),
        "threshold_heartbeat_warn": has(profile_py, "HEARTBEAT_WARN_S = 5.0"),
        "threshold_heartbeat_fail": has(profile_py, "HEARTBEAT_FAIL_S = 30.0"),
        "mission_input_flat_profile": has(profile_py, "MISSION_INPUT_FORMAT = \"flat_ordered\"") and not has(profile_py, "MISSION_SPLIT_P2_COUNT = max"),
        "mission_split_config_used": has(mission_cfg_py, "def split_nav_engage") and has(mission_cfg_py, "def get_mission_split_profile"),
        "legacy_structured_guarded": has(mission_adapter_py, "Structured mission payload disabled") and has(mission_adapter_py, "MISSION_ALLOW_STRUCTURED_LEGACY"),
        "telemetry_general_5hz": has(telem_py, "MAV_DATA_STREAM_ALL, GENERAL_TELEMETRY_HZ, 1"),
        "telemetry_rc_5hz": has(telem_py, "MAV_DATA_STREAM_RC_CHANNELS, GENERAL_TELEMETRY_HZ, 1"),
        "health_gate_ready": has(usv_py, "if not self.health_ready:"),
        "health_state_written": has(usv_py, "\"health_check\": {"),
        "manual_next_removed": not has(telem_py, "/api/next_parkur"),
        "auto_transition_enforced": has(usv_py, "state = self.STATE_ENGAGE"),
        "report_view_api": has(telem_py, "out['report_view'] = {"),
        "nav_mode_contract_visible": has(usv_py, "nav_mode = \"GUIDED\" if USV_MODE != USV_MODE_RACE else \"AUTO\"") and has(usv_py, "Pure Pixhawk AUTO mode"),
        "nav_ready_wait_visible": has(usv_py, "[WAIT] [P1->P2] Bekleniyor camera_ready="),
        "p2_guidance_priority_visible": has(usv_py, "Emergency: 100% avoidance") and has(usv_py, "gate bearing preferred") and has(usv_py, "pure waypoint bearing"),
        "sim_startup_blind_wait_removed": not has(sim_stack, "Waiting extra 12 sec for API services"),
        "sim_startup_api_probe": has(sim_stack, "camera_api_ready=True") and has(sim_stack, "probe_camera_endpoint"),
        "sim_startup_probe_based_waits": has(sim_stack, "wait_for_process_alive") and has(sim_stack, "wait_for_ros_topic") and has(sim_stack, "wait_for_file_fresh") and has(sim_stack, "wait_for_tcp_port 127.0.0.1 8888"),
        "shared_sim_nav_helper": has(telem_py, "from sim_nav_state import load_sim_nav_state"),
        # Spec/behavior alignment: simulation must not hard-force sensors ready.
        "sim_sensor_shortcut_enabled": not has(usv_py, "self.camera_ready = True") and not has(usv_py, "self.lidar_ready = True"),
        "objective_phase_written": has(usv_py, "def _get_objective_phase(self):") and has(usv_py, "\"objective_phase\": objective_phase"),
        "perception_policy_written": has(usv_py, "def _get_perception_policy(self):") and has(usv_py, "\"perception_policy\": perception_policy"),
        "camera_actionable_policy_visible": has(cam_py, "gate_actionable = bool(policy.get(\"gate\", False))") and has(cam_py, "yellow_actionable = bool(policy.get(\"yellow_obstacle\", False))") and has(cam_py, "target_actionable = bool(policy.get(\"target\", False))"),
        "camera_wrong_target_contract_visible": has(cam_py, "wrong_target_detected") and has(cam_py, "wrong_target_bearing_deg") and has(cam_py, "wrong_target_area_norm"),
        "engage_semantics_removed_from_nav": has(usv_py, "esc_c = 0.0"),
        "engage_target_guidance_present": (has(usv_py, "if target_detected:") or has(usv_py, "elif target_detected:")) and has(usv_py, "P3_TARGET_BEARING_GAIN"),
        "heading_wrap_uses_atan2": has(usv_py, "def normalize_heading_error") and has(usv_py, "math.atan2(math.sin(err_rad), math.cos(err_rad))"),
        "p3_heading_gain_clamp_configured": has(profile_py, "P3_TARGET_BEARING_GAIN") and has(profile_py, "P3_TARGET_HEADING_CLAMP_DEG") and has(usv_py, "P3_TARGET_HEADING_CLAMP_DEG"),
        "heading_diagnostics_standardized": has(usv_py, "event=\"p3_track\"") and has(usv_py, "yaw_rate_dps=round(float(self.current_yaw_rate_dps") and has(usv_py, "guidance_source=str(self._get_guidance_source())"),
        "heading_oscillation_analyzer_present": (ROOT / "host_scripts" / "analyze_heading_oscillation.py").exists(),
        "nav_heading_first_guided_yaw_only": has(usv_py, "turn_phase=bool(turn_phase)") and has(usv_py, "body_forward_only=bool(body_forward_only)") and has(usv_py, "creep_cap = max(0.0, float(NAV_ALIGN_CREEP_SPEED_MPS))"),
        "nav_min_progress_only_when_aligned": has(usv_py, "allow_min_progress") and has(usv_py, "NAV_ALIGN_HEADING_DONE_DEG") and has(usv_py, '_nav_align_mode", "align")) == "advance"'),
        "nav_sim_turn_creep_default": has(profile_py, "def resolve_nav_turn_creep_speed_mps") and has(profile_py, 'default = "0.18" if os.environ.get("USV_SIM") == "1"'),
        "nav_turn_priority_until_heading_aligned": has(usv_py, "acquiring_heading = bool(self._nav_align_mode") and has(usv_py, "nav_turn_priority = bool(") and has(usv_py, "heading_not_aligned"),
        "nav_waypoint_first_warn_blend": has(nav_guidance_py, "waypoint_first_warn_blend") and has(nav_guidance_py, "warn_blend"),
        "sim_wrong_turn_guided_yaw_hold": has(usv_py, "wrong_turn_hold:sim_guided_yaw_only") and has(usv_py, "sim_guided_yaw_hold"),
        "engage_wrong_target_avoidance_present": has(usv_py, "P3_WRONG_TARGET_AVOID") and has(usv_py, "wrong_target_contact_risk") and has(usv_py, "p3_wrong_target_avoidance"),
        "json_atomic_module_present": (SRC / "json_atomic.py").exists(),
        "shared_json_uses_atomic_write": (
            has(usv_py, "from json_atomic import atomic_write_json")
            and has(cam_py, "from json_atomic import atomic_write_json")
            and has(telem_py, "atomic_write_json") and has(telem_py, "from json_atomic import")
            and has(sim_bridge, "from json_atomic import atomic_write_json")
            and not has(usv_py, "Fallback: direct write")
        ),
        "mission_api_atomic_write": has(telem_py, "atomic_write_json(mission_file") and has(telem_py, "atomic_write_json(TARGET_STATE_FILE"),
        "lidar_map_jsonl_events": has(read(SRC / "lidar_map.py"), 'event="lidar_heartbeat"') and has(read(SRC / "lidar_map.py"), 'event="scan_timeout"'),
        "camera_bearing_half_from_profile": has(profile_py, "resolve_camera_bearing_half_deg") and has(cam_py, "resolve_camera_bearing_half_deg"),
        "camera_hsv_non_overlapping": has(cam_py, "np.array([6, 65, 90])") and has(cam_py, "np.array([23, 100, 100])") and has(cam_py, "np.array([4, 255, 255])"),
        "camera_wrong_target_policy_configured": has(profile_py, "camera_wrong_target_contract") and has(cam_py, "wrong_target_policy"),
        "lidar_sector_config_centralized": has(profile_py, "classify_lidar_scan_sector") and has(profile_py, "classify_lidar_map_sector") and has(usv_py, "classify_lidar_scan_sector"),
        "lidar_status_json_not_used": has(profile_py, "LIDAR_STATUS_JSON_ENABLED = False"),
        "pixhawk_param_compare_script_present": (ROOT / "host_scripts" / "compare_pixhawk_param_baseline.py").exists(),
        "core_documents_present": (ROOT / "documents" / "ida_sartname.md").exists()
        and (ROOT / "documents" / "rapor_calismasistemi.md").exists()
        and (ROOT / "README.md").exists(),
        "doc_readme_pixhawk_baseline_safety": has(readme_doc, "config/pixhawk_ida.param")
        and has(readme_doc, "ARMING_CHECK=0")
        and has(readme_doc, "fiziksel E-stop/kontaktor"),
        "doc_readme_servo_mapping_guard": has(readme_doc, "SERVO1/3")
        and has(readme_doc, "1100/1500/1900")
        and has(readme_doc, "su ustu test"),
        "mission_profile_unit_script_present": (ROOT / "host_scripts" / "check_mission_profile_unit.py").exists(),
        "race_readiness_score_script_present": (ROOT / "host_scripts" / "check_race_readiness_score.py").exists(),
        "readme_operation_references_current_docs": has(readme_doc, "documents/rapor_calismasistemi.md")
        and has(readme_doc, "documents/ida_sartname.md")
        and has(readme_doc, "Kontrol sahipligi ve operasyon ozeti icin bu README"),
        "mission_api_writes_flat": has(telem_py, "'input_format': MISSION_INPUT_FORMAT") and has(telem_py, "atomic_write_json(mission_file"),
        "runtime_loader_uses_target_state": has(usv_py, "load_target_state(TARGET_STATE_FILE)") and has(usv_py, "self.target_color = target_from_state or mission_target_color or self.target_color or \"RED\""),
        "mission_state_contract_visible": has(usv_py, "\"mission_input_format\": self.mission_input_format") and has(usv_py, "\"mission_split_profile\": dict(self.mission_split_profile)"),
        "gear_ui_removed": not has(telem_py, "gear_disp") and not has(telem_py, "<div class=\"label\">Gear</div>"),
        "gear_logic_removed": not has(telem_py, "input_gear") and not has(telem_py, "telemetry_data[\"Gear\"]") and not has(telem_py, "msg.chan6_raw"),
        "manual_control_signed_thrust": has(telem_py, "desired_throttle_pwm = 1500 + math.copysign") and has(telem_py, "self.motor_ctrl.update_inputs(msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw)"),
        "target_color_api_locked_post_start": has(telem_py, "Görev aktifken target_color güncellenemez.") and has(telem_py, "return jsonify({'error': 'Görev aktifken target_color güncellenemez.'}), 409"),
        "unified_nav_state_present": has(usv_py, "STATE_NAV = 1") and has(usv_py, "STATE_ENGAGE = 2"),
        "split_nav_engage_used": has(usv_py, "split_nav_engage") and has(mission_cfg_py, "def split_nav_engage"),
        "race_rc_override_env_guard": has(usv_py, "and USV_MODE != USV_MODE_RACE") and has(usv_py, "AUTONOMY_ACTUATOR_MODE = \"rc_override_bench_test\""),
        "guided_failure_holds_no_rc_fallback": has(usv_py, "GUIDED_SETPOINT_SEND_FAILED") and not has(usv_py, "RC override fallback kullaniliyor"),
        "sim_bridge_sitl_servo_primary": has(sim_bridge, "Default is ArduPilot SITL servo output") and has(sim_bridge, "ALLOW_MOTOR_COMMAND_JSON_OVERRIDE") and has(sim_bridge, "and USV_MODE != \"race\""),
        "sim_bridge_pwm_normalization_calibrated": has(sim_bridge, "SIM_GZ_PWM_MIN_US") and has(sim_bridge, "piecewise_min_trim_max") and has(sim_bridge, "pwm_1100_norm"),
        "sim_bridge_yaw_self_check_logged": has(sim_bridge, "event=\"sim_bridge_self_check\"") and has(sim_bridge, "SIM_GZ_YAW_SIGN") and has(sim_bridge, "bench_json_override_allowed"),
        "deprecated_sim_bridge_shims_removed": not (ROOT / "sim" / "bridges" / "mavlink_to_gz_pose_simple.py").exists() and not (ROOT / "sim" / "bridges" / "gazebo_pose_updater.py").exists(),
        "sim_lidar_map_started_before_usv_main": (
            sim_stack.find("Starting lidar_map service") != -1
            and sim_stack.find("Starting usv_main state machine") != -1
            and sim_stack.find("Starting lidar_map service") < sim_stack.find("Starting usv_main state machine")
            and has(check_stack, "lidar_map.py")
        ),
        "sim_check_stack_log_pairs": has(check_stack, "report_log_pair") and has(check_stack, "sitl_gazebo_bridge") and has(check_stack, "lidar_map"),
        "race_preflight_rejects_pwm_bypass": has(system_start, "USV_USE_RC_OVERRIDE=1 yasak") and has(internal_start, "SIM_GZ_ALLOW_MOTOR_COMMAND_JSON=1 yasak") and has(sim_stack, "SIM_GZ_ALLOW_MOTOR_COMMAND_JSON=1 yasak"),
        "compliance_wrapper_delegates": has(compliance_wrapper, "check_compliance_static.py") and has(compliance_wrapper, "check_compliance_behavior.py") and has(compliance_wrapper, "check_compliance_race.py"),
        # PWM / kontrol yüzeyi: kanıt şartname + rapor + README (ayrı kontrat dosyası yok).
        "doc_ida_onboard_autonomy_stack": has(
            ida_sartname_doc,
            "Görüntü işleme, sensör işleme ve otonomi yazılımlarının tamamı İDA üzerinde çalışacaktır.",
        ),
        "doc_rapor_setpoint_to_esc_path": has(rapor_doc, "Raspberry Pi setpoint üretir")
        and has(rapor_doc, "thruster/ESC komutuna dönüştürür"),
        "doc_rapor_guided_set_position_target_int": has(rapor_doc, "SET_POSITION_TARGET_GLOBAL_INT"),
        # Nötr ve bantlar: README Motor/PWM bölümü (şartname µs tanımlamaz).
        "doc_readme_pwm_neutral_and_clamp": has(readme_doc, "PWM_NEUTRAL_US=1500")
        or (has(readme_doc, "1500") and has(readme_doc, "1100") and has(readme_doc, "1900")),
        "doc_readme_rc_override_not_main_autonomy": has(readme_doc, "RC override ana otonomi yolu degildir."),
        "load_mission_no_nav_guard_before_sim_fallback": idx_no_nav_guard != -1 and idx_sim_fb != -1 and idx_no_nav_guard < idx_sim_fb,
        "mavlink_mirror_extract_cmds_and_download_item_types": (
            "MAV_CMD_NAV_SPLINE_WAYPOINT" in extract_body
            and "MAV_CMD_NAV_WAYPOINT" in extract_body
            and 'type=["MISSION_ITEM_INT", "MISSION_ITEM"]' in dl_body
        ),
        "mavlink_mirror_empty_rejects_no_nav": bool(dl_body) and ("no_nav_waypoints_extracted" in dl_body),
        "nav_near_wp_turn_err_imported": has(profile_py, "NAV_NEAR_WP_TURN_ERR_DEG = 20.0")
        and has(usv_py, "NAV_NEAR_WP_TURN_ERR_DEG,")
        and has(usv_py, "float(NAV_NEAR_WP_TURN_ERR_DEG)"),
        "stop_and_cleanup_preserve_logs": not any(
            token in (system_stop + "\n" + sim_cleanup_body)
            for token in (
                "rm -rf \"$USV_LOG_ROOT\"",
                "rm -rf \"$ROOT_LOG_DIR\"",
                "rm -rf logs",
                "rm -f \"$ROOT_LOG_DIR/terminal.log\"",
                ": > \"$ROOT_LOG_DIR/terminal.log\"",
                ": > \"$TERMINAL_LOG_FILE\"",
            )
        ),
        "no_log_archive_policy": not has(sim_stack, "/archive/")
        and not has(system_start, "/archive/")
        and not has(sim_stack, "archive_previous_logs")
        and not has(system_start, "archive_existing_logs"),
        "sim_start_latest_session_logs_only": has(sim_stack, "prepare_latest_log_session")
        and has(sim_stack, "rm -rf \"$ROOT_LOG_DIR\"")
        and has(sim_stack, "loglar sadece son session icin /logs altinda tutulur"),
        "sim_terminal_line_buffering": has(sim_stack, "run_line_buffered")
        and has(sim_stack, "stdbuf -oL -eL")
        and has(sim_stack, "kill_if_running \"$pid\" KILL")
        and has(console_utils_py, "line_buffering=True")
        and has(console_utils_py, "write_through=True"),
        "system_start_latest_session_logs_only": has(system_start, "prepare_latest_log_session")
        and has(system_start, "rm -rf \"$LOG_DIR\"")
        and has(system_start, "yalnız son session /logs altında tutulacak"),
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

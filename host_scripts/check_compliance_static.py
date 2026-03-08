#!/usr/bin/env python3
"""
Kontrol-1: Statik uyum kontrolu.
Rapor esik/frekans/mode ve endpoint kurallarinin kodda bulundugunu dogrular.
"""

from __future__ import annotations

import ast
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
REPORT_MD = ROOT / "documents" / "compliance_report.md"
REPORT_JSON = ROOT / "documents" / "compliance_report.json"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def has(text: str, needle: str) -> bool:
    return needle in text


def main() -> int:
    profile_py = read(SRC / "compliance_profile.py")
    usv_py = read(SRC / "usv_main.py")
    telem_py = read(SRC / "telemetry.py")

    checks = {
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


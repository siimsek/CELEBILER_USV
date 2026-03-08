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
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
REPORT_JSON = ROOT / "documents" / "compliance_behavior_report.json"
REPORT_MD = ROOT / "documents" / "compliance_behavior_report.md"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def has(text: str, needle: str) -> bool:
    return needle in text


def main() -> int:
    usv = read(SRC / "usv_main.py")
    telemetry = read(SRC / "telemetry.py")
    cam = read(SRC / "cam.py")
    lidar = read(SRC / "lidar_map.py")

    checks = {
        "parkur_transition_auto_only": has(usv, "otomatik gecis (kullanici girdisi kapali)"),
        "manual_next_flag_removed": ("FLAG_NEXT" not in usv) and ("/api/next_parkur" not in telemetry),
        "mission_start_ready_gate": has(usv, "HEALTH_CHECK gecmedi"),
        "race_start_api_blocked": has(telemetry, "if USV_MODE == USV_MODE_RACE"),
        "race_start_button_disabled": has(telemetry, "d.usv_mode === 'race'"),
        "dashboard_poll_5hz": has(telemetry, "setInterval(updateStats, 200);"),
        "api_includes_health": has(telemetry, "out['health_check'] ="),
        "api_includes_report_view": has(telemetry, "out['report_view'] ="),
        "camera_race_web_disabled": has(cam, "if RACE_MODE:") and has(cam, "Race modda sadece onboard isleme"),
        "lidar_race_exit": has(lidar, "YARIŞMA MODU - Harita yayını başlatılmıyor.") and has(lidar, "sys.exit(0)"),
    }

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


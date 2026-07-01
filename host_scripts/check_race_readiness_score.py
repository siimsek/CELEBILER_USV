#!/usr/bin/env python3
"""Recompute race readiness score (10 criteria) from codebase evidence."""

from __future__ import annotations

import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
REPORT_DIR = Path(os.environ.get("COMPLIANCE_REPORT_DIR", str(ROOT / "logs" / "system" / "compliance")))
MIN_DEPLOY_SCORE = 8


def read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return ""


def has(text: str, needle: str) -> bool:
    return needle in text


def evaluate() -> dict:
    usv = read(SRC / "usv_main.py")
    telem = read(SRC / "telemetry.py")
    cam = read(SRC / "cam.py")
    mission_cfg = read(SRC / "mission_config.py")
    adapter = read(SRC / "mission_adapter.py")
    readme = read(ROOT / "README.md")
    rapor_doc = read(ROOT / "documents" / "rapor_calismasistemi.md")

    criteria = [
        {
            "id": 1,
            "name": "Variable-length mission, no P1 hardcode default",
            "pass": has(usv, "if USV_MODE == USV_MODE_RACE and not p1_auto_env")
            and has(usv, "race_p1_completion_source")
            and has(mission_cfg, "parse_mission_profile_payload"),
            "evidence": "P1 uses pixhawk progress when USV_RACE_P1_AUTO_WAYPOINTS unset",
        },
        {
            "id": 2,
            "name": "Race mode: no image/map stream",
            "pass": has(telem, "USV_MODE == USV_MODE_RACE") and has(telem, "403") and has(cam, "RACE_MODE"),
            "evidence": "telemetry 403 + cam race guard",
        },
        {
            "id": 3,
            "name": "P1: pure Pixhawk AUTO",
            "pass": has(usv, 'nav_mode = "GUIDED" if USV_MODE != USV_MODE_RACE else "AUTO"')
            and has(usv, "nav_pixhawk_auto"),
            "evidence": "run_nav race branch + nav_pixhawk_auto guidance",
        },
        {
            "id": 4,
            "name": "Post-start lock",
            "pass": has(telem, "Görev aktifken") and has(telem, "409"),
            "evidence": "mission/target API 409 when active",
        },
        {
            "id": 5,
            "name": "Mission lifecycle tracking",
            "pass": has(usv, "mission_lifecycle") and has(usv, "upload_source"),
            "evidence": "mission_state lifecycle fields",
        },
        {
            "id": 6,
            "name": "Guidance source telemetry",
            "pass": has(usv, "def _get_guidance_source") and has(usv, '"guidance_source": self._get_guidance_source()'),
            "evidence": "explicit guidance_source in state",
        },
        {
            "id": 7,
            "name": "Mission adapter flat + structured",
            "pass": has(adapter, "isinstance(data, list)") and has(adapter, "isinstance(data, dict)"),
            "evidence": "mission_adapter dual format",
        },
        {
            "id": 8,
            "name": "Compliance constraints in code",
            "pass": has(usv, "_wait_for_p2_gate_minimum")
            and has(mission_cfg, "p2_min_gate_count")
            and has(usv, "mission_profile_race_ready"),
            "evidence": "P2 gate min + mission_profile race gate",
        },
        {
            "id": 9,
            "name": "E-stop within one control loop",
            "pass": has(usv, "def _trigger_estop") and has(usv, "def _enter_hold"),
            "evidence": "estop -> hold path",
        },
        {
            "id": 10,
            "name": "Pixhawk baseline safety documented",
            "pass": (ROOT / "host_scripts" / "compare_pixhawk_param_baseline.py").is_file()
            and has(readme, "config/pixhawk_ida.param")
            and has(readme, "ARMING_CHECK=0")
            and has(readme, "fiziksel E-stop/kontaktor")
            and has(rapor_doc, "Fiziksel E-stop/kontaktör hattı"),
            "evidence": "README baseline safety note + param compare script + working-system doc",
        },
    ]

    passed = sum(1 for item in criteria if item["pass"])
    total = len(criteria)
    return {
        "score": passed,
        "total": total,
        "min_deploy_score": MIN_DEPLOY_SCORE,
        "race_deploy_recommended": passed >= MIN_DEPLOY_SCORE,
        "criteria": criteria,
        "note": "Code/static evidence only; water tests and operator procedures are still required for field deploy.",
    }


def main() -> int:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    report = evaluate()
    json_path = REPORT_DIR / "race_readiness_score.json"
    md_path = REPORT_DIR / "race_readiness_score.md"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    lines = [
        "# Race Readiness Score",
        "",
        f"- Score: **{report['score']}/{report['total']}**",
        f"- Minimum for static deploy gate: **{MIN_DEPLOY_SCORE}/10**",
        f"- Recommended (static): **{'YES' if report['race_deploy_recommended'] else 'NO'}**",
        "",
        "| # | Criterion | Status |",
        "|---:|---|---|",
    ]
    for item in report["criteria"]:
        lines.append(f"| {item['id']} | {item['name']} | {'PASS' if item['pass'] else 'FAIL'} |")
    lines.extend(["", f"Note: {report['note']}", ""])
    md_path.write_text("\n".join(lines), encoding="utf-8")

    print(json.dumps({
        "score": report["score"],
        "total": report["total"],
        "race_deploy_recommended": report["race_deploy_recommended"],
        "report_json": str(json_path),
        "report_md": str(md_path),
    }, ensure_ascii=False))
    return 0 if report["race_deploy_recommended"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

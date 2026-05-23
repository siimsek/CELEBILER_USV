#!/usr/bin/env python3
"""Compare Pixhawk param baseline against ArduRover defaults and race-risk registry.

Usage:
  python3 host_scripts/compare_pixhawk_param_baseline.py
  python3 host_scripts/compare_pixhawk_param_baseline.py \\
      --current config/pixhawk_ida.param \\
      --defaults config/ardurover_defaults.reference.param \\
      --strict

Outputs JSON + Markdown under logs/system/compliance/ by default.
Does not connect to Pixhawk or start the vehicle.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CURRENT = ROOT / "config" / "pixhawk_ida.param"
DEFAULT_DEFAULTS = ROOT / "config" / "ardurover_defaults.reference.param"
REPORT_DIR = Path(os.environ.get("COMPLIANCE_REPORT_DIR", str(ROOT / "logs" / "system" / "compliance")))

# Risk registry for params that matter to CELEBILER USV race readiness.
RISK_REGISTRY: dict[str, dict[str, str]] = {
    "ARMING_CHECK": {
        "risk_level": "HIGH",
        "risk_id": "RISK-ARMING-NO-PREARM",
        "default_hint": "1 (pre-arm checks enabled)",
        "race_note": "Manual-test baseline disables checks; race requires certified E-stop chain or re-enabled checks.",
        "race_profile_suggestion": "1 for race pre-arm, or keep 0 with signed physical E-stop cert (see documents/race_cert_pixhawk_arming_failsafe.md).",
    },
    "ARMING_REQUIRE": {
        "risk_level": "HIGH",
        "risk_id": "RISK-ARMING-NO-REQUIRE",
        "default_hint": "1 (require arming sequence)",
        "race_note": "Zero allows immediate arm without throttle-safe gating.",
        "race_profile_suggestion": "1 for race, or documented operator procedure if kept at 0.",
    },
    "FS_THR_ENABLE": {
        "risk_level": "HIGH",
        "risk_id": "RISK-FAILSAFE-RC-DISABLED",
        "default_hint": "1 (RC failsafe enabled)",
        "race_note": "Pixhawk RC-loss action disabled; Pi watchdog + physical E-stop must cover gap.",
        "race_profile_suggestion": "1 if RC link is primary safety; else certify Pi HOLD + contactor path.",
    },
    "FS_GCS_ENABLE": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-FAILSAFE-GCS-DISABLED",
        "default_hint": "1 (GCS heartbeat failsafe)",
        "race_note": "GCS failsafe off; acceptable if race forbids mission telecommand except upload.",
        "race_profile_suggestion": "Keep 0 for race if GCS link is monitoring-only; document in race cert.",
    },
    "BATT_FS_LOW_ACT": {
        "risk_level": "HIGH",
        "risk_id": "RISK-FAILSAFE-BATT-DISABLED",
        "default_hint": "1 (battery low action)",
        "race_note": "Low battery Pixhawk action disabled.",
        "race_profile_suggestion": "Enable low-battery HOLD/RTL or prove external power cut + operator abort.",
    },
    "BATT_FS_CRT_ACT": {
        "risk_level": "HIGH",
        "risk_id": "RISK-FAILSAFE-BATT-CRIT-DISABLED",
        "default_hint": "2 (critical battery action)",
        "race_note": "Critical battery Pixhawk action disabled.",
        "race_profile_suggestion": "Enable critical action or prove shore power / manual abort covers risk.",
    },
    "CRUISE_SPEED": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-CRUISE-SPEED",
        "default_hint": "vehicle dependent",
        "race_note": "P1 AUTO cruise ~2 m/s; validate braking distance for hull and course.",
        "race_profile_suggestion": "Match P1 mission speed to water test; do not raise without retest.",
    },
    "WP_SPEED": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-WP-SPEED",
        "default_hint": "vehicle dependent",
        "race_note": "Waypoint speed for AUTO legs.",
        "race_profile_suggestion": "Align with compliance_profile P1_SPEED_CRUISE_MPS after water validation.",
    },
    "WP_RADIUS": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-WP-RADIUS",
        "default_hint": "vehicle dependent",
        "race_note": "2 m radius on ~1.60 m hull may cause early/late WP acceptance.",
        "race_profile_suggestion": "Validate zigzag acceptance on water before race lock.",
    },
    "TURN_RADIUS": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-TURN-RADIUS",
        "default_hint": "vehicle dependent",
        "race_note": "Skid-steer turn geometry; wrong value causes overshoot or stall turns.",
        "race_profile_suggestion": "Calibrate from water test; update baseline only after signed test record.",
    },
    "SERVO1_FUNCTION": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-SERVO1-FUNCTION",
        "default_hint": "frame dependent (ThrottleLeft=74)",
        "race_note": "Must match left motor on real boat.",
        "race_profile_suggestion": "74 left throttle per documents/servo_mapping_water_test_plan.md sign-off.",
    },
    "SERVO3_FUNCTION": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-SERVO3-FUNCTION",
        "default_hint": "frame dependent (ThrottleRight=73)",
        "race_note": "Must match right motor on real boat.",
        "race_profile_suggestion": "73 right throttle per water test sign-off.",
    },
    "SERVO1_MIN": {"risk_level": "MEDIUM", "risk_id": "RISK-SERVO-PWM-RANGE", "default_hint": "1100", "race_note": "PWM contract min.", "race_profile_suggestion": "1100 (match sim bridge SIM_GZ_PWM_MIN_US)."},
    "SERVO1_TRIM": {"risk_level": "MEDIUM", "risk_id": "RISK-SERVO-PWM-RANGE", "default_hint": "1500", "race_note": "PWM neutral.", "race_profile_suggestion": "1500"},
    "SERVO1_MAX": {"risk_level": "MEDIUM", "risk_id": "RISK-SERVO-PWM-RANGE", "default_hint": "1900", "race_note": "PWM contract max.", "race_profile_suggestion": "1900 (match sim bridge SIM_GZ_PWM_MAX_US)."},
    "SERVO3_MIN": {"risk_level": "MEDIUM", "risk_id": "RISK-SERVO-PWM-RANGE", "default_hint": "1100", "race_note": "PWM contract min.", "race_profile_suggestion": "1100"},
    "SERVO3_TRIM": {"risk_level": "MEDIUM", "risk_id": "RISK-SERVO-PWM-RANGE", "default_hint": "1500", "race_note": "PWM neutral.", "race_profile_suggestion": "1500"},
    "SERVO3_MAX": {"risk_level": "MEDIUM", "risk_id": "RISK-SERVO-PWM-RANGE", "default_hint": "1900", "race_note": "PWM contract max.", "race_profile_suggestion": "1900"},
    "FRAME_CLASS": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-FRAME-CLASS",
        "default_hint": "2 (Rover/Boat class in baseline)",
        "race_note": "Boat/skid-steer frame must match wiring.",
        "race_profile_suggestion": "Keep verified FRAME_CLASS/TYPE from water test.",
    },
    "FRAME_TYPE": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-FRAME-TYPE",
        "default_hint": "0",
        "race_note": "Skid steering type must match motor layout.",
        "race_profile_suggestion": "Do not change without repeating servo mapping test.",
    },
    "FS_ACTION": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-FS-ACTION",
        "default_hint": "2 (RTL/HOLD class action)",
        "race_note": "Failsafe action when enabled.",
        "race_profile_suggestion": "Document chosen action in race cert; prefer HOLD near obstacles.",
    },
    "FS_EKF_ACTION": {
        "risk_level": "MEDIUM",
        "risk_id": "RISK-FS-EKF",
        "default_hint": "1",
        "race_note": "EKF failsafe behavior.",
        "race_profile_suggestion": "Verify GPS-denied / jump behavior on water if changed.",
    },
}


def parse_param_file(path: Path) -> dict[str, str]:
    params: dict[str, str] = {}
    if not path.is_file():
        raise FileNotFoundError(f"Param file not found: {path}")
    for line_no, raw in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("QGC") or line.startswith("FormatVersion"):
            continue
        if "," not in line:
            continue
        name, value = line.split(",", 1)
        name = name.strip()
        value = value.strip()
        if name:
            params[name] = value
    if not params:
        raise ValueError(f"No parameters parsed from {path} (line {line_no if 'line_no' in locals() else '?'})")
    return params


def _norm(value: str) -> str:
    text = str(value).strip()
    try:
        number = float(text)
        if number.is_integer():
            return str(int(number))
        return str(number)
    except ValueError:
        return text


def build_report(current_path: Path, defaults_path: Path | None) -> dict[str, Any]:
    current = parse_param_file(current_path)
    defaults = parse_param_file(defaults_path) if defaults_path and defaults_path.is_file() else {}

    audited: list[dict[str, Any]] = []
    non_default: list[dict[str, Any]] = []
    high_risk_open: list[str] = []

    for name, meta in sorted(RISK_REGISTRY.items()):
        cur = current.get(name)
        ref = defaults.get(name)
        entry = {
            "param": name,
            "current": cur,
            "reference_default": ref,
            "risk_level": meta["risk_level"],
            "risk_id": meta["risk_id"],
            "default_hint": meta["default_hint"],
            "race_note": meta["race_note"],
            "race_profile_suggestion": meta["race_profile_suggestion"],
            "differs_from_reference": bool(ref is not None and cur is not None and _norm(cur) != _norm(ref)),
            "missing_in_current": cur is None,
        }
        audited.append(entry)
        if entry["differs_from_reference"]:
            non_default.append(entry)
        if meta["risk_level"] == "HIGH" and cur is not None:
            high_risk_open.append(name)

    all_diffs = []
    if defaults:
        for name in sorted(set(current) | set(defaults)):
            if name not in RISK_REGISTRY:
                cur = current.get(name)
                ref = defaults.get(name)
                if cur is None or ref is None:
                    continue
                if _norm(cur) != _norm(ref):
                    all_diffs.append({"param": name, "current": cur, "reference_default": ref})

    race_profile = {
        "baseline_file": str(current_path),
        "reference_defaults_file": str(defaults_path) if defaults_path else None,
        "cert_documents": [
            "documents/race_cert_pixhawk_arming_failsafe.md",
            "documents/servo_mapping_water_test_plan.md",
        ],
        "recommendations": [
            "Complete servo mapping water test before changing SERVO* or FRAME_* params.",
            "Sign race arming/failsafe cert before race deploy with ARMING_CHECK=0 baseline.",
            "Re-run this script after any param or sim SITL profile change.",
        ],
        "audited_high_risk_params": high_risk_open,
    }

    return {
        "current_file": str(current_path),
        "defaults_file": str(defaults_path) if defaults_path else None,
        "param_count_current": len(current),
        "param_count_defaults": len(defaults),
        "audited_params": audited,
        "non_default_audited": non_default,
        "other_non_default_count": len(all_diffs),
        "other_non_default_sample": all_diffs[:40],
        "race_profile": race_profile,
        "strict_blockers": [
            item["param"]
            for item in audited
            if item["risk_level"] == "HIGH" and item["differs_from_reference"] and not item["missing_in_current"]
        ],
    }


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Pixhawk Param Baseline Comparison",
        "",
        f"- Current: `{report['current_file']}` ({report['param_count_current']} params)",
        f"- Reference defaults: `{report.get('defaults_file') or 'none'}`",
        "",
        "## Audited params (risk registry)",
        "",
        "| Param | Current | Ref default | Risk | Differs | Suggestion |",
        "|---|---:|---:|---|---|---|",
    ]
    for item in report["audited_params"]:
        lines.append(
            f"| {item['param']} | {item.get('current', '—')} | {item.get('reference_default', '—')} "
            f"| {item['risk_level']} | {item['differs_from_reference']} | {item['race_profile_suggestion']} |"
        )
    lines.extend(
        [
            "",
            "## Race profile notes",
            "",
        ]
    )
    for note in report["race_profile"]["recommendations"]:
        lines.append(f"- {note}")
    if report["strict_blockers"]:
        lines.extend(["", "## Strict blockers (HIGH + differs from reference)", ""])
        for name in report["strict_blockers"]:
            lines.append(f"- `{name}`")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare Pixhawk param baseline vs ArduRover reference defaults.")
    parser.add_argument("--current", type=Path, default=DEFAULT_CURRENT, help="Current Pixhawk param export")
    parser.add_argument(
        "--defaults",
        type=Path,
        default=DEFAULT_DEFAULTS,
        help="ArduRover default/reference param file (optional)",
    )
    parser.add_argument("--report-dir", type=Path, default=REPORT_DIR, help="Output directory for JSON/Markdown")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Exit 1 if any HIGH-risk audited param differs from reference defaults",
    )
    parser.add_argument("--json", action="store_true", help="Print JSON summary to stdout")
    args = parser.parse_args()

    defaults_path = args.defaults if args.defaults and args.defaults.is_file() else None
    report = build_report(args.current.resolve(), defaults_path.resolve() if defaults_path else None)

    args.report_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.report_dir / "pixhawk_param_baseline.json"
    md_path = args.report_dir / "pixhawk_param_baseline.md"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    md_path.write_text(render_markdown(report), encoding="utf-8")

    summary = {
        "success": not (args.strict and report["strict_blockers"]),
        "audited_count": len(report["audited_params"]),
        "non_default_audited_count": len(report["non_default_audited"]),
        "strict_blockers": report["strict_blockers"],
        "report_json": str(json_path),
        "report_md": str(md_path),
    }
    if args.json:
        print(json.dumps(summary, indent=2, ensure_ascii=False))
    else:
        print(json.dumps(summary, ensure_ascii=False))

    if args.strict and report["strict_blockers"]:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

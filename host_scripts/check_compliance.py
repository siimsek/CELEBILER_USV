#!/usr/bin/env python3
"""Compatibility wrapper for the current compliance suite.

Legacy inline checks drifted from the sim-first contract. Keep this entrypoint
for operators/CI, but delegate to the authoritative static, behavior and race
checkers so there is only one source of truth.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKS = (
    ROOT / "host_scripts" / "check_compliance_static.py",
    ROOT / "host_scripts" / "check_compliance_behavior.py",
    ROOT / "host_scripts" / "check_compliance_race.py",
    ROOT / "host_scripts" / "check_mission_profile_unit.py",
    ROOT / "host_scripts" / "check_race_readiness_score.py",
)


def main() -> int:
    failures: list[tuple[str, int]] = []
    for script in CHECKS:
        print(f"[COMPLIANCE] Running {script.name}...", flush=True)
        result = subprocess.run([sys.executable, str(script)], cwd=str(ROOT), check=False)
        if result.returncode != 0:
            failures.append((script.name, int(result.returncode)))

    if failures:
        for name, code in failures:
            print(f"[COMPLIANCE] FAIL {name} exit={code}", flush=True)
        return 1

    print("[COMPLIANCE] PASS static + behavior + race", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

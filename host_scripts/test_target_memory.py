#!/usr/bin/env python3
"""Unit checks for P3 target memory prediction."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from target_memory import TargetMemory  # noqa: E402


def main() -> int:
    memory = TargetMemory(memory_s=8.0, max_bearing_deg=45.0)
    live = memory.update(detected=True, bearing_deg=10.0, area_norm=0.08, confidence=0.9, now_s=100.0)
    if not live.detected or live.source != "vision_live":
        raise AssertionError(f"live target not accepted: {live}")
    memory.update(detected=True, bearing_deg=14.0, area_norm=0.08, confidence=0.9, now_s=101.0)
    remembered = memory.update(detected=False, bearing_deg=0.0, area_norm=0.0, confidence=0.0, now_s=104.0)
    if not remembered.active or remembered.detected:
        raise AssertionError(f"target memory did not stay active: {remembered}")
    if remembered.bearing_deg <= 14.0:
        raise AssertionError(f"bearing prediction did not advance: {remembered}")
    expired = memory.update(detected=False, bearing_deg=0.0, area_norm=0.0, confidence=0.0, now_s=112.0)
    if expired.active or expired.source != "search_spiral":
        raise AssertionError(f"memory did not expire into search: {expired}")

    print("test_target_memory: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

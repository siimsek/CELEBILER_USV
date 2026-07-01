#!/usr/bin/env python3
"""Unit checks for dependency-free Kalman helpers."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "docker_workspace" / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from kalman_filter import ExtendedKalmanFilter6D  # noqa: E402


def main() -> int:
    ekf = ExtendedKalmanFilter6D()
    first = ekf.update_position(0.0, 0.0)
    if not first.accepted or not ekf.initialized:
        raise AssertionError("initial GPS position was not accepted")
    ekf.predict(1.0)
    normal = ekf.update_position(0.6, 0.0, measurement_var=1.0)
    if not normal.accepted:
        raise AssertionError(f"normal GPS update rejected: {normal}")
    outlier = ekf.update_position(100.0, 100.0, measurement_var=1.0, chi2_threshold=9.21)
    if outlier.accepted:
        raise AssertionError(f"large outlier accepted: {outlier}")
    heading = ekf.update_heading(5.0)
    if not heading.accepted:
        raise AssertionError(f"heading update rejected: {heading}")

    print("test_kalman_filter: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

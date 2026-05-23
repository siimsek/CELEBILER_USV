#!/usr/bin/env python3
"""Offline heading/yaw oscillation summary from local JSONL logs."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG = ROOT / "logs" / "system" / "usv_main.jsonl"


def _as_float(value, default=None):
    try:
        if value is None:
            return default
        number = float(value)
        if math.isnan(number) or math.isinf(number):
            return default
        return number
    except (TypeError, ValueError):
        return default


def _iter_events(paths):
    for path in paths:
        if not path.exists():
            continue
        with path.open("r", encoding="utf-8", errors="replace") as handle:
            for line_no, line in enumerate(handle, start=1):
                line = line.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(payload, dict):
                    payload["_source_file"] = str(path)
                    payload["_source_line"] = line_no
                    yield payload


def analyze(paths):
    samples = []
    sign_flips = 0
    prev_sign = 0
    max_abs_heading = 0.0
    max_abs_yaw_rate = 0.0
    yaw_overshoot_count = 0
    event_counts = {}

    for event in _iter_events(paths):
        event_name = str(event.get("event", ""))
        heading_error = _as_float(event.get("heading_error_deg"))
        if heading_error is None:
            heading_error = _as_float(event.get("command_heading_err_deg"))
        yaw_rate = _as_float(event.get("yaw_rate_dps"))
        if yaw_rate is None:
            yaw_rate = _as_float(event.get("observed_yaw_rate_dps"))
        if heading_error is None and yaw_rate is None:
            continue

        event_counts[event_name or "unknown"] = event_counts.get(event_name or "unknown", 0) + 1
        if heading_error is not None:
            abs_heading = abs(heading_error)
            max_abs_heading = max(max_abs_heading, abs_heading)
            sign = 1 if heading_error > 0.5 else (-1 if heading_error < -0.5 else 0)
            if sign and prev_sign and sign != prev_sign:
                sign_flips += 1
            if sign:
                prev_sign = sign
            if yaw_rate is not None:
                yaw_sign = 1 if yaw_rate > 0.5 else (-1 if yaw_rate < -0.5 else 0)
                if sign and yaw_sign and sign != yaw_sign and abs_heading >= 10.0:
                    yaw_overshoot_count += 1
        if yaw_rate is not None:
            max_abs_yaw_rate = max(max_abs_yaw_rate, abs(yaw_rate))
        samples.append(event)

    sample_count = len(samples)
    sign_flip_rate = (sign_flips / max(1, sample_count - 1)) if sample_count > 1 else 0.0
    return {
        "sample_count": sample_count,
        "max_abs_heading_error_deg": round(max_abs_heading, 3),
        "sign_flip_count": sign_flips,
        "sign_flip_rate": round(sign_flip_rate, 4),
        "max_abs_yaw_rate_dps": round(max_abs_yaw_rate, 3),
        "yaw_rate_overshoot_count": yaw_overshoot_count,
        "event_counts": event_counts,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "logs",
        nargs="*",
        type=Path,
        default=[DEFAULT_LOG],
        help="JSONL log path(s); defaults to logs/system/usv_main.jsonl",
    )
    args = parser.parse_args()
    paths = args.logs or [DEFAULT_LOG]
    result = analyze(paths)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

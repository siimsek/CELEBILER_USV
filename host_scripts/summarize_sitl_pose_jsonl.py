#!/usr/bin/env python3
"""
sitl_gazebo_bridge JSONL özet: pose_stats ve set_pose hataları (titreme teşhisi).

Kullanım:
  python3 host_scripts/summarize_sitl_pose_jsonl.py [logs/simulation/sitl_gazebo_bridge.jsonl]
"""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "logs" / "simulation" / "sitl_gazebo_bridge.jsonl"
    if not path.is_file():
        print(f"Dosya yok: {path}")
        print("Sim çalıştırdıktan sonra USV_LOG_JSONL=1 ile jsonl üretilir.")
        sys.exit(1)

    events = Counter()
    last_stats = None
    errors = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except json.JSONDecodeError:
                continue
            ev = o.get("event", "")
            events[ev] += 1
            if ev == "pose_stats":
                last_stats = o
            if ev in ("set_pose_future_error", "set_pose_exception") or o.get("success") is False:
                if len(errors) < 20:
                    errors.append(o)

    print(f"Kaynak: {path}")
    print("Olay sayıları:", dict(events.most_common(30)))
    if last_stats:
        print(
            "Son pose_stats:",
            {k: last_stats[k] for k in last_stats if k in ("window_s", "n_set_pose_ok", "n_set_pose_fail", "n_set_pose_future_err", "n_set_pose_skipped_pending", "set_pose_ok_hz")},
        )
    if errors:
        print(f"Örnek hata/başarısız (max 20): {len(errors)} kayıt")
        for e in errors[:5]:
            print(" ", e)


if __name__ == "__main__":
    main()

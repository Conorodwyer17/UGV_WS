#!/usr/bin/env python3
"""
Summarize inspection_manager JSONL mission log (mission_log_path) for thesis / demos.

Each line is JSON: {"event": str, "data": dict, "t": float}

Usage:
  python3 ~/ugv_ws/scripts/generate_mission_report.py /path/to/mission.jsonl
  python3 ~/ugv_ws/scripts/generate_mission_report.py   # reads MISSION_LOG env or first *.jsonl in tire_inspection_photos/mission_logs
"""
from __future__ import annotations

import glob
import json
import os
import sys
from datetime import datetime


def _load_lines(path: str) -> list[dict]:
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def _default_log_path() -> str | None:
    env = os.environ.get("MISSION_LOG", "").strip()
    if env and os.path.isfile(os.path.expanduser(env)):
        return os.path.expanduser(env)
    base = os.path.expanduser(os.environ.get("UGV_WS", "~/ugv_ws"))
    pat = os.path.join(base, "tire_inspection_photos", "mission_logs", "*.jsonl")
    cand = sorted(glob.glob(pat), key=os.path.getmtime, reverse=True)
    return cand[0] if cand else None


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else _default_log_path()
    if not path or not os.path.isfile(path):
        print("Usage: generate_mission_report.py [mission.jsonl]", file=sys.stderr)
        print("Set MISSION_LOG or place JSONL under ~/ugv_ws/tire_inspection_photos/mission_logs/", file=sys.stderr)
        return 1

    rows = _load_lines(path)
    events = [r.get("event", "") for r in rows]
    photo = [r for r in rows if r.get("event") == "photo_captured"]
    nav_sent = [r for r in rows if r.get("event") == "nav_command_sent"]
    transitions = [r for r in rows if r.get("event") == "state_transition"]

    print("\n" + "=" * 60)
    print("TYRE INSPECTION MISSION LOG SUMMARY")
    print("=" * 60)
    print(f"File: {path}")
    print(f"Lines: {len(rows)}")
    print(f"Date (file mtime): {datetime.fromtimestamp(os.path.getmtime(path)).isoformat()}")
    print(f"\nEvent counts (top):")
    from collections import Counter

    for ev, n in Counter(events).most_common(20):
        print(f"  {ev}: {n}")
    print(f"\nphoto_captured events: {len(photo)}")
    print(f"nav_command_sent events: {len(nav_sent)}")
    print(f"state_transition events: {len(transitions)}")
    if photo:
        print("\nLast photo_captured data sample:")
        print(json.dumps(photo[-1].get("data", {}), indent=2)[:800])
    print("\n" + "=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

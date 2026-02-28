#!/usr/bin/env python3
"""SQLite persistence for inspection missions."""

import json
import os
import sqlite3
import uuid
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional


def _workspace_root() -> str:
    env_ws = os.environ.get("UGV_WS", "").strip()
    if env_ws and os.path.isdir(env_ws):
        return env_ws

    colcon_prefix = os.environ.get("COLCON_PREFIX_PATH", "").split(":")
    for prefix in colcon_prefix:
        prefix = prefix.strip()
        if not prefix:
            continue
        # .../<workspace>/install
        if os.path.basename(prefix) == "install":
            candidate = os.path.dirname(prefix)
            if os.path.isdir(candidate):
                return candidate

    fallback = "/home/conor/ugv_ws"
    if os.path.isdir(fallback):
        return fallback

    return os.path.abspath(os.getcwd())


class MissionStatePersistence:
    def __init__(self, db_path: Optional[str] = None) -> None:
        self.db_path = db_path or os.path.join(_workspace_root(), "research/state/inspection_missions.db")
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self._init_db()

    @staticmethod
    def _now() -> str:
        return datetime.now(timezone.utc).isoformat()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _init_db(self) -> None:
        with self._connect() as conn:
            conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS missions (
                  mission_id TEXT PRIMARY KEY,
                  object_id TEXT,
                  state TEXT,
                  allow_partial_success INTEGER,
                  created_at TEXT,
                  updated_at TEXT,
                  config_json TEXT,
                  failure_reason TEXT
                );
                CREATE TABLE IF NOT EXISTS tires (
                  mission_id TEXT,
                  tire_id TEXT,
                  order_index INTEGER,
                  status TEXT,
                  attempt_count INTEGER,
                  last_error TEXT,
                  photo_path TEXT,
                  photo_metadata_json TEXT,
                  updated_at TEXT,
                  PRIMARY KEY (mission_id, tire_id)
                );
                CREATE TABLE IF NOT EXISTS events (
                  id INTEGER PRIMARY KEY AUTOINCREMENT,
                  mission_id TEXT,
                  ts TEXT,
                  state TEXT,
                  event_type TEXT,
                  payload_json TEXT
                );
                """
            )

    def start_mission(self, object_id: str, config_json: str, allow_partial_success: bool = False) -> str:
        mission_id = f"mission_{uuid.uuid4().hex[:12]}"
        now = self._now()
        with self._connect() as conn:
            conn.execute(
                """
                INSERT INTO missions(mission_id, object_id, state, allow_partial_success, created_at, updated_at, config_json, failure_reason)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (mission_id, object_id, "DISCOVERY", 1 if allow_partial_success else 0, now, now, config_json, ""),
            )
            conn.execute(
                "INSERT INTO events(mission_id, ts, state, event_type, payload_json) VALUES (?, ?, ?, ?, ?)",
                (mission_id, now, "DISCOVERY", "mission_start", json.dumps({"object_id": object_id})),
            )
        return mission_id

    def add_tires(self, mission_id: str, tire_ids: List[str]) -> None:
        now = self._now()
        with self._connect() as conn:
            for idx, tire_id in enumerate(tire_ids):
                conn.execute(
                    """
                    INSERT OR REPLACE INTO tires(mission_id, tire_id, order_index, status, attempt_count, last_error, photo_path, photo_metadata_json, updated_at)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (mission_id, tire_id, idx, "pending", 0, "", "", "{}", now),
                )

    def set_mission_state(self, mission_id: str, state: str, failure_reason: str = "") -> None:
        now = self._now()
        with self._connect() as conn:
            conn.execute(
                "UPDATE missions SET state = ?, failure_reason = ?, updated_at = ? WHERE mission_id = ?",
                (state, failure_reason, now, mission_id),
            )
            conn.execute(
                "INSERT INTO events(mission_id, ts, state, event_type, payload_json) VALUES (?, ?, ?, ?, ?)",
                (mission_id, now, state, "state_transition", json.dumps({"failure_reason": failure_reason})),
            )

    def set_tire_status(self, mission_id: str, tire_id: str, status: str, **kwargs) -> None:
        now = self._now()
        attempt_increment = int(kwargs.get("attempt_increment", 0))
        last_error = str(kwargs.get("last_error", ""))
        photo_path = str(kwargs.get("photo_path", ""))
        photo_metadata = kwargs.get("photo_metadata", {})
        with self._connect() as conn:
            row = conn.execute(
                "SELECT attempt_count FROM tires WHERE mission_id = ? AND tire_id = ?",
                (mission_id, tire_id),
            ).fetchone()
            current_attempts = int(row["attempt_count"]) if row else 0
            next_attempts = current_attempts + attempt_increment
            conn.execute(
                """
                INSERT INTO tires(mission_id, tire_id, order_index, status, attempt_count, last_error, photo_path, photo_metadata_json, updated_at)
                VALUES (?, ?, COALESCE((SELECT order_index FROM tires WHERE mission_id = ? AND tire_id = ?), 9999), ?, ?, ?, ?, ?, ?)
                ON CONFLICT(mission_id, tire_id) DO UPDATE SET
                  status=excluded.status,
                  attempt_count=excluded.attempt_count,
                  last_error=excluded.last_error,
                  photo_path=excluded.photo_path,
                  photo_metadata_json=excluded.photo_metadata_json,
                  updated_at=excluded.updated_at
                """,
                (
                    mission_id,
                    tire_id,
                    mission_id,
                    tire_id,
                    status,
                    next_attempts,
                    last_error,
                    photo_path,
                    json.dumps(photo_metadata),
                    now,
                ),
            )
            conn.execute(
                "INSERT INTO events(mission_id, ts, state, event_type, payload_json) VALUES (?, ?, ?, ?, ?)",
                (
                    mission_id,
                    now,
                    status.upper(),
                    "tire_status",
                    json.dumps(
                        {
                            "tire_id": tire_id,
                            "status": status,
                            "attempt_count": next_attempts,
                            "last_error": last_error,
                            "photo_path": photo_path,
                        }
                    ),
                ),
            )

    def get_mission_state(self, mission_id: str) -> Optional[Dict[str, Any]]:
        with self._connect() as conn:
            m = conn.execute("SELECT * FROM missions WHERE mission_id = ?", (mission_id,)).fetchone()
            if m is None:
                return None
            tires = conn.execute("SELECT * FROM tires WHERE mission_id = ? ORDER BY order_index", (mission_id,)).fetchall()
            return {"mission": dict(m), "tires": [dict(t) for t in tires]}

    def latest_mission(self) -> Optional[Dict[str, Any]]:
        with self._connect() as conn:
            row = conn.execute("SELECT * FROM missions ORDER BY created_at DESC LIMIT 1").fetchone()
            return None if row is None else dict(row)

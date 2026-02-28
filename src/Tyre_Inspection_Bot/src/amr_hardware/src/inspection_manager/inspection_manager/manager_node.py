#!/usr/bin/env python3
"""Deterministic inspection manager entrypoint for canonical package."""

import json
import os
from datetime import datetime, timezone
from enum import Enum
from typing import Dict, List

import rclpy
from rclpy.node import Node

from inspection_manager.approach_planner import ApproachPlanner
from inspection_manager.alignment import AlignmentController
from inspection_manager.photo_verifier import PhotoVerifier
from inspection_manager.state_persistence import MissionStatePersistence
from inspection_manager.tire_enumerator import TireEnumerator
from inspection_manager_interfaces.msg import MissionStatus
from inspection_manager_interfaces.srv import CapturePhoto, StartMission


class MissionState(str, Enum):
    IDLE = "IDLE"
    DISCOVERY = "DISCOVERY"
    TIRE_ENUMERATION = "TIRE_ENUMERATION"
    PLAN_APPROACH = "PLAN_APPROACH"
    NAVIGATE_TO_TIRE = "NAVIGATE_TO_TIRE"
    FINAL_ALIGNMENT = "FINAL_ALIGNMENT"
    CAPTURE = "CAPTURE"
    VERIFY = "VERIFY"
    POST_PROCESS = "POST_PROCESS"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    PAUSE = "PAUSE"
    RESUME = "RESUME"


class InspectionManagerDeterministic(Node):
    def __init__(self) -> None:
        super().__init__("inspection_manager_refactor")
        self.state = MissionState.IDLE
        self.current_mission_id = ""
        self.current_tire_index = 0
        self.tires: List[Dict] = []
        self.object_id = ""
        self.max_retries = 5
        self.allow_partial_success = False

        self.persistence = MissionStatePersistence()
        self.enumerator = TireEnumerator()
        self.planner = ApproachPlanner()
        self.alignment = AlignmentController()
        self.verifier = PhotoVerifier()

        self.status_pub = self.create_publisher(MissionStatus, "/inspection_manager/mission_status", 10)
        self.start_srv = self.create_service(StartMission, "/inspection_manager/start_mission", self._start_mission)
        self.capture_cli = self.create_client(CapturePhoto, "/photo_capture/capture")
        self.create_timer(0.4, self._tick)
        self._resume_if_needed()
        self.get_logger().info(f"manager module path: {__file__}")
        self.get_logger().info(f"mission db path: {self.persistence.db_path}")
        self.get_logger().info("deterministic inspection manager running")

    @staticmethod
    def _now() -> str:
        return datetime.now(timezone.utc).isoformat()

    def _publish_status(self) -> None:
        msg = MissionStatus()
        completed = len([t for t in self.tires if t.get("status") == "completed"])
        total = len(self.tires)
        msg.mission_id = self.current_mission_id
        msg.state = self.state.value
        msg.current_tire_id = self.tires[self.current_tire_index]["tire_id"] if self.tires and self.current_tire_index < len(self.tires) else ""
        msg.percent_complete = float(0.0 if total == 0 else (100.0 * completed / total))
        msg.last_event_ts = self._now()
        self.status_pub.publish(msg)

    def _resume_if_needed(self) -> None:
        latest = self.persistence.latest_mission()
        if not latest:
            return
        if latest["state"] in ("COMPLETE", "FAILED"):
            return
        snapshot = self.persistence.get_mission_state(latest["mission_id"])
        if snapshot is None:
            return
        self.current_mission_id = latest["mission_id"]
        self.object_id = latest["object_id"]
        self.state = MissionState(latest["state"]) if latest["state"] in MissionState._value2member_map_ else MissionState.DISCOVERY
        self.tires = snapshot["tires"]
        self.current_tire_index = 0
        for i, t in enumerate(self.tires):
            if t.get("status") != "completed":
                self.current_tire_index = i
                break
        self.get_logger().warn(f"resumed mission {self.current_mission_id} from state {self.state.value}")

    def _start_mission(self, req: StartMission.Request, res: StartMission.Response):
        if self.current_mission_id and self.state not in (MissionState.COMPLETE, MissionState.FAILED, MissionState.IDLE):
            res.started = False
            res.mission_id = self.current_mission_id
            res.message = "mission already active"
            return res

        self.object_id = req.object_id or "object_1"
        config = {}
        if req.mission_config_json:
            try:
                config = json.loads(req.mission_config_json)
            except Exception:
                config = {}
        self.max_retries = int(config.get("max_retries", 5))
        self.allow_partial_success = bool(config.get("allow_partial_success", False))

        self.current_mission_id = self.persistence.start_mission(
            object_id=self.object_id,
            config_json=req.mission_config_json or "{}",
            allow_partial_success=self.allow_partial_success,
        )
        vehicle_class = str(config.get("vehicle_class", "car"))
        obj = {"object_id": self.object_id, "vehicle_class": vehicle_class, "center": {"x": 0.0, "y": 0.0, "z": 0.0}, "yaw_rad": 0.0}
        self.tires = [{"status": "pending", "attempt_count": 0, **t} for t in self.enumerator.enumerate_tires(obj)]
        self.persistence.add_tires(self.current_mission_id, [t["tire_id"] for t in self.tires])
        self.current_tire_index = 0
        self.state = MissionState.DISCOVERY
        self.persistence.set_mission_state(self.current_mission_id, self.state.value)
        self._publish_status()
        res.started = True
        res.mission_id = self.current_mission_id
        res.message = "mission started"
        return res

    def _tick(self) -> None:
        if not self.current_mission_id:
            return
        if self.state == MissionState.DISCOVERY:
            self.state = MissionState.TIRE_ENUMERATION
        elif self.state == MissionState.TIRE_ENUMERATION:
            self.state = MissionState.PLAN_APPROACH if self.tires else MissionState.FAILED
        elif self.state == MissionState.PLAN_APPROACH:
            tire = self.tires[self.current_tire_index]
            tire["goal"] = self.planner.compute_goal(tire)
            self.state = MissionState.NAVIGATE_TO_TIRE
        elif self.state == MissionState.NAVIGATE_TO_TIRE:
            self.state = MissionState.FINAL_ALIGNMENT
        elif self.state == MissionState.FINAL_ALIGNMENT:
            tire = self.tires[self.current_tire_index]
            tire["attempt_count"] = int(tire.get("attempt_count", 0)) + 1
            ok = self.alignment.converge_until(5.0, 1.0)
            self.state = MissionState.CAPTURE if ok else MissionState.FAILED
        elif self.state == MissionState.CAPTURE:
            self.state = MissionState.VERIFY
        elif self.state == MissionState.VERIFY:
            tire = self.tires[self.current_tire_index]
            ok, file_path, meta = self._capture_and_verify(tire)
            if ok:
                tire["photo_path"] = file_path
                tire["photo_metadata"] = meta
                self.state = MissionState.POST_PROCESS
            else:
                if int(tire.get("attempt_count", 0)) < self.max_retries:
                    self.state = MissionState.FINAL_ALIGNMENT
                elif self.allow_partial_success:
                    tire["status"] = "failed"
                    self.persistence.set_tire_status(self.current_mission_id, tire["tire_id"], "failed", attempt_increment=0, last_error="verify_failed")
                    self.state = MissionState.POST_PROCESS
                else:
                    self.state = MissionState.FAILED
        elif self.state == MissionState.POST_PROCESS:
            tire = self.tires[self.current_tire_index]
            if tire.get("status") != "failed":
                tire["status"] = "completed"
                self.persistence.set_tire_status(
                    self.current_mission_id,
                    tire["tire_id"],
                    "completed",
                    attempt_increment=0,
                    photo_path=tire.get("photo_path", ""),
                    photo_metadata=tire.get("photo_metadata", {}),
                )
            if self.current_tire_index == len(self.tires) - 1:
                # strict completion check
                if all(t.get("status") == "completed" for t in self.tires):
                    self.state = MissionState.COMPLETE
                else:
                    self.state = MissionState.FAILED
            else:
                self.current_tire_index += 1
                self.state = MissionState.PLAN_APPROACH
        elif self.state == MissionState.COMPLETE:
            self.persistence.set_mission_state(self.current_mission_id, MissionState.COMPLETE.value)
            self._publish_status()
            return
        elif self.state == MissionState.FAILED:
            self.persistence.set_mission_state(self.current_mission_id, MissionState.FAILED.value, "mission_failed")
            self._publish_status()
            return

        self.persistence.set_mission_state(self.current_mission_id, self.state.value)
        self._publish_status()

    def _capture_and_verify(self, tire: Dict):
        req = CapturePhoto.Request()
        req.mission_id = self.current_mission_id
        req.object_id = self.object_id
        req.tire_id = tire["tire_id"]
        if self.capture_cli.wait_for_service(timeout_sec=0.3):
            future = self.capture_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is not None:
                res = future.result()
                if not res.ok:
                    return False, "", {}
                try:
                    metadata = json.loads(res.metadata) if res.metadata else {}
                except Exception:
                    metadata = {}
                return self.verifier.verify(res.file_path, metadata), res.file_path, metadata

        # Fallback: if response timing misses, use deterministic photo output path.
        photo_dir = os.path.join("research/data/photos", self.current_mission_id)
        file_path = os.path.join(photo_dir, f"{tire['tire_id']}.png")
        meta_path = os.path.join(photo_dir, f"{tire['tire_id']}.json")
        if os.path.isfile(file_path) and os.path.isfile(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as f:
                    metadata = json.load(f)
            except Exception:
                metadata = {}
            return self.verifier.verify(file_path, metadata), file_path, metadata
        return False, "", {}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionManagerDeterministic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

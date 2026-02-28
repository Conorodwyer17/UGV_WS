#!/usr/bin/env python3
"""Deterministic mission manager skeleton."""

from enum import Enum
import json
import os
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from inspection_manager.alignment import AlignmentController
from inspection_manager.approach_planner import ApproachPlanner
from inspection_manager.photo_verifier import PhotoVerifier
from inspection_manager.state_persistence import MissionStatePersistence
from inspection_manager.tire_enumerator import TireEnumerator


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


class InspectionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("inspection_manager_refactor")
        self.state = MissionState.IDLE
        self.current_mission_id = ""
        self.current_tire_index = 0
        self.tires = []
        self.object_id = ""
        self.allow_partial_success = False

        db_path = "research/state/inspection_missions.db"
        self.persistence = MissionStatePersistence(db_path)
        self.enumerator = TireEnumerator()
        self.planner = ApproachPlanner()
        self.alignment = AlignmentController()
        self.verifier = PhotoVerifier()

        self.status_pub = self.create_publisher(String, "/inspection_manager/mission_status", 10)
        self.start_srv = self.create_service(Trigger, "/inspection_manager/start_mission", self._start_mission_cb)
        self.create_timer(0.5, self._tick)
        self.get_logger().info("inspection_manager deterministic core initialized")

    def _publish_status(self, detail: str = "") -> None:
        msg = String()
        completed = len([t for t in self.tires if t.get("status") == "completed"])
        total = len(self.tires)
        percent = 0.0 if total == 0 else (100.0 * completed / total)
        msg.data = json.dumps(
            {
                "mission_id": self.current_mission_id,
                "state": self.state.value,
                "current_tire": self.tires[self.current_tire_index]["tire_id"] if self.tires and self.current_tire_index < len(self.tires) else "",
                "percent_complete": percent,
                "detail": detail,
            }
        )
        self.status_pub.publish(msg)

    def _start_mission_cb(self, request: Trigger.Request, response: Trigger.Response):
        _ = request
        if self.state not in (MissionState.IDLE, MissionState.COMPLETE, MissionState.FAILED):
            response.success = False
            response.message = "mission already active"
            return response

        self.object_id = f"object_{uuid.uuid4().hex[:6]}"
        self.current_mission_id = self.persistence.start_mission(
            object_id=self.object_id,
            config_json=json.dumps({"max_retries": 5, "allow_partial_success": False}),
            allow_partial_success=self.allow_partial_success,
        )
        object_record = {
            "object_id": self.object_id,
            "vehicle_class": "car",
            "center": {"x": 0.0, "y": 0.0, "z": 0.0},
            "yaw_rad": 0.0,
            "axle_count": 2,
        }
        tire_candidates = self.enumerator.enumerate_tires(object_record)
        self.tires = [{"status": "pending", **t} for t in tire_candidates]
        self.persistence.add_tires(self.current_mission_id, [t["tire_id"] for t in self.tires])
        self.current_tire_index = 0
        self.state = MissionState.DISCOVERY
        self.persistence.set_mission_state(self.current_mission_id, self.state.value)
        self._publish_status("mission_started")
        response.success = True
        response.message = self.current_mission_id
        return response

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
            ok = self.alignment.converge_until(5.0, 1.0)
            self.state = MissionState.CAPTURE if ok else MissionState.FAILED
        elif self.state == MissionState.CAPTURE:
            # Simulated capture path for dry-runs
            fake_path = f"research/data/photos/{self.current_mission_id}/{self.tires[self.current_tire_index]['tire_id']}.png"
            os.makedirs(os.path.dirname(fake_path), exist_ok=True)
            if not os.path.isfile(fake_path):
                try:
                    import cv2
                    import numpy as np

                    rng = np.random.default_rng(42 + self.current_tire_index)
                    img = (rng.random((900, 1600)) * 255).astype(np.uint8)
                    cv2.circle(img, (800, 450), 260, 255, 10)
                    cv2.rectangle(img, (620, 300), (980, 600), 180, 6)
                    cv2.line(img, (500, 450), (1100, 450), 255, 3)
                    cv2.imwrite(fake_path, img, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
                except Exception:
                    with open(fake_path, "wb") as f:
                        f.write(b"\x00" * (60 * 1024))
            self.tires[self.current_tire_index]["photo_path"] = fake_path
            self.state = MissionState.VERIFY
        elif self.state == MissionState.VERIFY:
            tire = self.tires[self.current_tire_index]
            ok = self.verifier.verify(
                tire["photo_path"],
                {
                    "vehicle_id": self.object_id,
                    "tire_id": tire["tire_id"],
                    "timestamp": "simulated",
                    "pose": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "camera_intrinsics": {},
                    "projection_overlap": 0.8,
                },
            )
            self.state = MissionState.POST_PROCESS if ok else MissionState.FAILED
        elif self.state == MissionState.POST_PROCESS:
            tire = self.tires[self.current_tire_index]
            tire["status"] = "completed"
            self.persistence.set_tire_status(
                self.current_mission_id,
                tire["tire_id"],
                "completed",
                attempt_increment=1,
                photo_path=tire["photo_path"],
                photo_metadata={"vehicle_id": self.object_id, "tire_id": tire["tire_id"]},
            )
            if self.current_tire_index == len(self.tires) - 1:
                self.state = MissionState.COMPLETE
            else:
                self.current_tire_index += 1
                self.state = MissionState.PLAN_APPROACH
        elif self.state == MissionState.COMPLETE:
            self.persistence.set_mission_state(self.current_mission_id, MissionState.COMPLETE.value)
            self._publish_status("mission_complete")
            return
        elif self.state == MissionState.FAILED:
            self.persistence.set_mission_state(self.current_mission_id, MissionState.FAILED.value, "state_failure")
            self._publish_status("mission_failed")
            return

        self.persistence.set_mission_state(self.current_mission_id, self.state.value)
        self._publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


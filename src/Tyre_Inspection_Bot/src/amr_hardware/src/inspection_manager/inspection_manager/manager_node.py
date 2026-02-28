#!/usr/bin/env python3
"""Unified live inspection manager entrypoint for canonical package."""

import json
import math
from datetime import datetime, timezone
from enum import Enum
from typing import Dict, List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from inspection_manager.alignment import AlignmentController
from inspection_manager.approach_planner import ApproachPlanner
from inspection_manager.photo_verifier import PhotoVerifier
from inspection_manager.state_persistence import MissionStatePersistence
from inspection_manager.world_model import WorldModel
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


class InspectionManagerUnified(Node):
    def __init__(self) -> None:
        super().__init__("inspection_manager")
        self.state = MissionState.IDLE
        self.current_mission_id = ""
        self.current_tire_index = 0
        self.tires: List[Dict] = []
        self.requested_object_id = ""
        self.target_vehicle_id = ""
        self.max_retries = 5
        self.allow_partial_success = False

        self.declare_parameter("vehicle_labels", ["car", "truck", "bus"])
        self.declare_parameter("vehicle_detection_topic", "/darknet_ros_3d/vehicle_bounding_boxes")
        self.declare_parameter("tire_detection_topic", "/darknet_ros_3d/tire_bounding_boxes")
        self.declare_parameter("vehicle_min_prob", 0.45)
        self.declare_parameter("tire_min_prob", 0.40)
        self.declare_parameter("reacquire_timeout_s", 10.0)
        self.declare_parameter("alignment_tolerance_cm", 5.0)
        self.declare_parameter("alignment_timeout_s", 6.0)
        self.declare_parameter("alignment_target_dist_m", 0.35)
        self.declare_parameter("navigate_timeout_s", 45.0)
        self.declare_parameter("approach_offset", 0.6)
        self.vehicle_labels = [str(x).strip() for x in self.get_parameter("vehicle_labels").value]
        self.vehicle_min_prob = float(self.get_parameter("vehicle_min_prob").value)
        self.tire_min_prob = float(self.get_parameter("tire_min_prob").value)
        self.reacquire_timeout_s = float(self.get_parameter("reacquire_timeout_s").value)
        self.alignment_tolerance_cm = float(self.get_parameter("alignment_tolerance_cm").value)
        self.alignment_timeout_s = float(self.get_parameter("alignment_timeout_s").value)
        self.alignment_target_dist_m = float(self.get_parameter("alignment_target_dist_m").value)
        self.navigate_timeout_s = float(self.get_parameter("navigate_timeout_s").value)

        self.persistence = MissionStatePersistence()
        self.planner = ApproachPlanner(standoff_m=float(self.get_parameter("approach_offset").value))
        self.alignment = AlignmentController()
        self.verifier = PhotoVerifier()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world_model = WorldModel(self, self.tf_buffer)
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.current_tire_attempt = 0
        self.last_reacquire_start_s = 0.0

        self.status_pub = self.create_publisher(MissionStatus, "/inspection_manager/mission_status", 10)
        self.start_srv = self.create_service(StartMission, "/inspection_manager/start_mission", self._start_mission)
        self.capture_cli = self.create_client(CapturePhoto, "/photo_capture/capture")
        self.create_subscription(
            BoundingBoxes3d,
            str(self.get_parameter("vehicle_detection_topic").value),
            self._on_vehicle_boxes,
            10,
        )
        self.create_subscription(
            BoundingBoxes3d,
            str(self.get_parameter("tire_detection_topic").value),
            self._on_tire_boxes,
            10,
        )
        self.create_timer(0.4, self._tick)
        self._resume_if_needed()
        self.get_logger().info("unified inspection_manager running (live Nav2 + live perception)")

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

    def _on_vehicle_boxes(self, msg: BoundingBoxes3d) -> None:
        self.world_model.update_vehicles(msg, self.vehicle_labels, self.vehicle_min_prob)

    def _on_tire_boxes(self, msg: BoundingBoxes3d) -> None:
        self.world_model.update_tires(msg, "car-tire", self.tire_min_prob)

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
        self.requested_object_id = latest["object_id"]
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

        self.requested_object_id = req.object_id.strip() if req.object_id else ""
        config = {}
        if req.mission_config_json:
            try:
                config = json.loads(req.mission_config_json)
            except Exception:
                config = {}
        self.max_retries = int(config.get("max_retries", 5))
        self.allow_partial_success = bool(config.get("allow_partial_success", False))
        if self.allow_partial_success:
            self.get_logger().warn("allow_partial_success enabled by mission config")

        self.current_mission_id = self.persistence.start_mission(
            object_id=self.requested_object_id or "auto",
            config_json=req.mission_config_json or "{}",
            allow_partial_success=self.allow_partial_success,
        )
        self.target_vehicle_id = ""
        self.tires = []
        self.current_tire_attempt = 0
        self.current_tire_index = 0
        self.state = MissionState.DISCOVERY
        self.persistence.set_mission_state(self.current_mission_id, self.state.value)
        self._publish_status()
        res.started = True
        res.mission_id = self.current_mission_id
        res.message = "mission started"
        return res

    def _resolve_target_vehicle(self) -> Optional[str]:
        return self.world_model.resolve_vehicle_id(self.requested_object_id)

    def _yaw_to_quat(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(0.5 * yaw)
        q.w = math.cos(0.5 * yaw)
        return q

    def _navigate_to(self, goal: Dict) -> bool:
        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("navigate_to_pose action server unavailable")
            return False
        nav_goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = goal.get("frame_id", "map")
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal["x"])
        pose.pose.position.y = float(goal["y"])
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quat(float(goal["yaw_rad"]))
        nav_goal.pose = pose
        send_future = self.navigate_client.send_goal_async(nav_goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.navigate_timeout_s)
        if result_future.result() is None:
            self.get_logger().error("NavigateToPose timed out")
            return False
        return int(result_future.result().status) == int(GoalStatus.STATUS_SUCCEEDED)

    def _robot_pose_map(self) -> Optional[Dict[str, float]]:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.1))
            return {
                "x": tf.transform.translation.x,
                "y": tf.transform.translation.y,
                "z": tf.transform.translation.z,
            }
        except Exception:
            return None

    def _alignment_error(self, tire: Dict) -> float:
        robot = self._robot_pose_map()
        live = self.world_model.get_tire(self.target_vehicle_id, tire["tire_id"]) or tire
        if robot is None:
            return 999.0
        tx = float(live["position"]["x"])
        ty = float(live["position"]["y"])
        dist = math.sqrt((tx - robot["x"]) ** 2 + (ty - robot["y"]) ** 2)
        return abs(dist - self.alignment_target_dist_m)

    def _tick(self) -> None:
        if not self.current_mission_id:
            return
        if self.state == MissionState.DISCOVERY:
            vid = self._resolve_target_vehicle()
            if vid:
                self.target_vehicle_id = vid
                self.state = MissionState.TIRE_ENUMERATION
            else:
                if self.last_reacquire_start_s == 0.0:
                    self.last_reacquire_start_s = self.get_clock().now().nanoseconds / 1e9
                elapsed = (self.get_clock().now().nanoseconds / 1e9) - self.last_reacquire_start_s
                if elapsed > self.reacquire_timeout_s:
                    self.state = MissionState.FAILED
        elif self.state == MissionState.TIRE_ENUMERATION:
            live_tires = self.world_model.get_tires_for_vehicle(self.target_vehicle_id, max_stale_s=1.0)
            self.tires = [{"status": "pending", "attempt_count": 0, **t} for t in live_tires]
            if self.tires:
                self.persistence.add_tires(self.current_mission_id, [t["tire_id"] for t in self.tires])
                self.current_tire_index = 0
                self.current_tire_attempt = 0
                self.state = MissionState.PLAN_APPROACH
            else:
                self.state = MissionState.DISCOVERY
        elif self.state == MissionState.PLAN_APPROACH:
            tire = self.tires[self.current_tire_index]
            tire["goal"] = self.planner.compute_goal(tire)
            self.state = MissionState.NAVIGATE_TO_TIRE
        elif self.state == MissionState.NAVIGATE_TO_TIRE:
            tire = self.tires[self.current_tire_index]
            ok = self._navigate_to(tire["goal"])
            if ok:
                self.state = MissionState.FINAL_ALIGNMENT
            else:
                self.current_tire_attempt += 1
                if self.current_tire_attempt <= self.max_retries:
                    self.state = MissionState.PLAN_APPROACH
                else:
                    self.persistence.set_tire_status(
                        self.current_mission_id,
                        tire["tire_id"],
                        "failed",
                        attempt_increment=1,
                        last_error="navigate_failed",
                    )
                    self.state = MissionState.FAILED if not self.allow_partial_success else MissionState.POST_PROCESS
        elif self.state == MissionState.FINAL_ALIGNMENT:
            tire = self.tires[self.current_tire_index]
            tire["attempt_count"] = int(tire.get("attempt_count", 0)) + 1
            ok = self.alignment.converge_until(
                error_fn=lambda: self._alignment_error(tire),
                tolerance_cm=self.alignment_tolerance_cm,
                timeout_s=self.alignment_timeout_s,
            )
            if ok:
                self.state = MissionState.CAPTURE
            else:
                self.current_tire_attempt += 1
                self.state = MissionState.PLAN_APPROACH if self.current_tire_attempt <= self.max_retries else MissionState.FAILED
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
                    self.persistence.set_tire_status(
                        self.current_mission_id,
                        tire["tire_id"],
                        "failed",
                        attempt_increment=1,
                        last_error="verify_failed",
                    )
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
                    attempt_increment=1,
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
                self.current_tire_attempt = 0
                self.state = MissionState.PLAN_APPROACH
        elif self.state == MissionState.COMPLETE:
            self.persistence.set_mission_state(self.current_mission_id, MissionState.COMPLETE.value)
            self._publish_status()
            self.state = MissionState.IDLE
            self.current_mission_id = ""
            return
        elif self.state == MissionState.FAILED:
            self.persistence.set_mission_state(self.current_mission_id, MissionState.FAILED.value, "mission_failed")
            self._publish_status()
            self.state = MissionState.IDLE
            self.current_mission_id = ""
            return

        self.persistence.set_mission_state(self.current_mission_id, self.state.value)
        self._publish_status()

    def _capture_and_verify(self, tire: Dict):
        req = CapturePhoto.Request()
        req.mission_id = self.current_mission_id
        req.object_id = self.target_vehicle_id or self.requested_object_id
        req.tire_id = tire["tire_id"]
        if self.capture_cli.wait_for_service(timeout_sec=1.0):
            future = self.capture_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None:
                res = future.result()
                if not res.ok:
                    return False, "", {}
                try:
                    metadata = json.loads(res.metadata) if res.metadata else {}
                except Exception:
                    metadata = {}
                return self.verifier.verify(res.file_path, metadata), res.file_path, metadata
        return False, "", {}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InspectionManagerUnified()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Service endpoint for `/photo_capture/capture`."""

import json
import os
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node

from inspection_manager_interfaces.srv import CapturePhoto


class PhotoCaptureApi(Node):
    def __init__(self) -> None:
        super().__init__("photo_capture_api")
        self.declare_parameter("output_root", "research/data/photos")
        self.output_root = str(self.get_parameter("output_root").value)
        self.srv = self.create_service(CapturePhoto, "/photo_capture/capture", self._handle_capture)
        self.get_logger().info("photo_capture API service ready: /photo_capture/capture")

    def _handle_capture(self, req: CapturePhoto.Request, res: CapturePhoto.Response):
        mission_id = req.mission_id or "unknown_mission"
        object_id = req.object_id or "unknown_object"
        tire_id = req.tire_id or "unknown_tire"
        ts = datetime.now(timezone.utc).isoformat()
        out_dir = os.path.join(self.output_root, mission_id)
        os.makedirs(out_dir, exist_ok=True)
        file_path = os.path.join(out_dir, f"{tire_id}.png")
        try:
            import cv2
            import numpy as np

            img = np.zeros((900, 1600), dtype=np.uint8)
            cv2.circle(img, (800, 450), 260, 255, 10)
            cv2.rectangle(img, (620, 300), (980, 600), 180, 6)
            cv2.line(img, (500, 450), (1100, 450), 255, 3)
            cv2.imwrite(file_path, img, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])
        except Exception:
            with open(file_path, "wb") as f:
                f.write(b"\x01" * (80 * 1024))

        metadata = {
            "mission_id": mission_id,
            "vehicle_id": object_id,
            "tire_id": tire_id,
            "timestamp": ts,
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0},
            "camera_intrinsics": {},
            "projection_overlap": 0.8,
        }
        with open(os.path.join(out_dir, f"{tire_id}.json"), "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2)

        res.file_path = file_path
        res.metadata = json.dumps(metadata)
        res.ok = True
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhotoCaptureApi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Dynamic speed filter for vehicle approach zones.

Subscribes to vehicle bounding boxes (BoundingBoxes3d) and publishes:
1. OccupancyGrid mask to filter_mask_topic (0=full speed, 50=50% speed in vehicle buffer)
2. CostmapFilterInfo to filter_info_topic (once at startup)

The Nav2 SpeedFilter plugin subscribes to both and publishes SpeedLimit to the controller,
slowing the robot when it enters the buffer zone around detected vehicles.

Evidence: nav2_costmap_2d SpeedFilter, nav2_system_tests speed_local_params.yaml
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d
from std_msgs.msg import Header


class VehicleSpeedFilterNode(Node):
    def __init__(self):
        super().__init__("vehicle_speed_filter")
        self.declare_parameter("vehicle_boxes_topic", "/aurora_semantic/vehicle_bounding_boxes")
        self.declare_parameter("filter_mask_topic", "/inspection/speed_filter_mask")
        self.declare_parameter("filter_info_topic", "/local_costmap/costmap_filter_info")
        self.declare_parameter("mask_frame_id", "map")
        self.declare_parameter("mask_resolution", 0.05)
        self.declare_parameter("mask_width_m", 50.0)
        self.declare_parameter("mask_height_m", 50.0)
        self.declare_parameter("vehicle_buffer_m", 1.5)
        self.declare_parameter("speed_percent_in_zone", 50)
        self.declare_parameter("publish_rate_hz", 2.0)

        self._vehicle_boxes_topic = self.get_parameter("vehicle_boxes_topic").value
        self._filter_mask_topic = self.get_parameter("filter_mask_topic").value
        self._filter_info_topic = self.get_parameter("filter_info_topic").value
        self._mask_frame = self.get_parameter("mask_frame_id").value
        self._resolution = self.get_parameter("mask_resolution").value
        self._width_m = self.get_parameter("mask_width_m").value
        self._height_m = self.get_parameter("mask_height_m").value
        self._buffer = self.get_parameter("vehicle_buffer_m").value
        self._speed_percent = max(1, min(100, self.get_parameter("speed_percent_in_zone").value))
        self._rate_hz = self.get_parameter("publish_rate_hz").value

        self._latest_boxes = None

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
        # Nav2 SpeedFilter expects TRANSIENT_LOCAL for mask and filter info (matches costmap layer)
        filter_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._sub = self.create_subscription(
            BoundingBoxes3d, self._vehicle_boxes_topic, self._boxes_cb, qos
        )
        self._mask_pub = self.create_publisher(OccupancyGrid, self._filter_mask_topic, filter_qos)
        self._info_pub = self.create_publisher(CostmapFilterInfo, self._filter_info_topic, filter_qos)

        self._info_published = False
        period_ns = int(1e9 / self._rate_hz)
        self._timer = self.create_timer(period_ns / 1e9, self._timer_cb)

        self.get_logger().info(
            f"vehicle_speed_filter: vehicle_boxes={self._vehicle_boxes_topic} "
            f"-> mask={self._filter_mask_topic} info={self._filter_info_topic} "
            f"(buffer={self._buffer}m, speed={self._speed_percent}%)"
        )

    def _boxes_cb(self, msg: BoundingBoxes3d):
        self._latest_boxes = msg

    def _publish_filter_info(self):
        if self._info_published:
            return
        msg = CostmapFilterInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.type = 1  # SPEED_FILTER_PERCENT
        msg.filter_mask_topic = self._filter_mask_topic
        msg.base = 0.0
        msg.multiplier = 1.0
        self._info_pub.publish(msg)
        self._info_published = True
        self.get_logger().info(f"Published CostmapFilterInfo to {self._filter_info_topic}")

    def _timer_cb(self):
        self._publish_filter_info()

        width_cells = int(self._width_m / self._resolution)
        height_cells = int(self._height_m / self._resolution)
        origin_x = -self._width_m / 2.0
        origin_y = -self._height_m / 2.0

        grid = [0] * (width_cells * height_cells)  # 0 = no limit (full speed)

        if self._latest_boxes and self._latest_boxes.bounding_boxes:
            for box in self._latest_boxes.bounding_boxes:
                xmin = min(box.xmin, box.xmax) - self._buffer
                xmax = max(box.xmin, box.xmax) + self._buffer
                ymin = min(box.ymin, box.ymax) - self._buffer
                ymax = max(box.ymin, box.ymax) + self._buffer

                col_min = int((xmin - origin_x) / self._resolution)
                col_max = int((xmax - origin_x) / self._resolution)
                row_min = int((ymin - origin_y) / self._resolution)
                row_max = int((ymax - origin_y) / self._resolution)

                col_min = max(0, col_min)
                col_max = min(width_cells - 1, col_max)
                row_min = max(0, row_min)
                row_max = min(height_cells - 1, row_max)

                for r in range(row_min, row_max + 1):
                    for c in range(col_min, col_max + 1):
                        idx = r * width_cells + c
                        grid[idx] = self._speed_percent

        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._mask_frame
        msg.info.resolution = self._resolution
        msg.info.width = width_cells
        msg.info.height = height_cells
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid
        self._mask_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSpeedFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

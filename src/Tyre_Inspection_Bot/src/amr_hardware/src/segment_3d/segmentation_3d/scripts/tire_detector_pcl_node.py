#!/usr/bin/env python3
"""
Point-cloud tire fallback detector.

When YOLO misses tires (poor lighting, occlusion), use point-cloud clustering
within the vehicle ROI as a fallback. Subscribes to vehicle_boxes and point_cloud,
crops/filters/clusters, publishes BoundingBoxes3d to /tire_bounding_boxes_pcl_fallback.

Pipeline: crop to vehicle ROI -> height filter -> voxel downsample -> DBSCAN cluster
-> tire-like filter (size, extent) -> publish BoundingBoxes3d.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from sklearn.cluster import DBSCAN

try:
    from sensor_msgs_py import point_cloud2
except ImportError:
    point_cloud2 = None


def _pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract x,y,z from PointCloud2 as Nx3 float64 array."""
    if point_cloud2 is not None:
        try:
            pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            # read_points returns structured array; cannot cast to float64 directly
            if pts.size == 0:
                return np.zeros((0, 3), dtype=np.float64)
            return np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
        except (KeyError, TypeError, ValueError) as e:
            # Fallback: try read_points_numpy if fields are same type
            try:
                pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
                return pts.astype(np.float64) if pts.size > 0 else np.zeros((0, 3), dtype=np.float64)
            except Exception:
                pass
            # Last resort: manual parse
            pass
    # Fallback: manual parse for PointXYZ
    import struct
    point_step = msg.point_step
    row_step = msg.row_step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    xyz = []
    for i in range(0, len(data), point_step):
        if i + 12 <= len(data):
            x, y, z = struct.unpack_from("<fff", data, i)
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                xyz.append([x, y, z])
    return np.array(xyz, dtype=np.float64) if xyz else np.zeros((0, 3))


def _transform_points(points: np.ndarray, transform) -> np.ndarray:
    """Apply TF transform (geometry_msgs/TransformStamped) to Nx3 points."""
    if points.size == 0:
        return points
    t = transform.transform.translation
    r = transform.transform.rotation
    # Build rotation matrix from quaternion
    qx, qy, qz, qw = r.x, r.y, r.z, r.w
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])
    t_vec = np.array([t.x, t.y, t.z])
    return (points @ R.T) + t_vec


def _voxel_downsample(points: np.ndarray, leaf_size: float) -> np.ndarray:
    """Simple voxel downsampling: take centroid of each voxel."""
    if points.size == 0 or leaf_size <= 0:
        return points
    voxel_idx = np.floor(points / leaf_size).astype(np.int32)
    voxel_idx -= voxel_idx.min(axis=0)
    dims = voxel_idx.max(axis=0) + 1
    if np.prod(dims) > 10_000_000:
        return points  # Too many voxels, skip
    flat = np.ravel_multi_index(voxel_idx.T, dims)
    unique, inverse = np.unique(flat, return_inverse=True)
    out = np.zeros((len(unique), 3))
    np.add.at(out, inverse, points)
    counts = np.bincount(inverse, minlength=len(unique))
    out /= counts[:, np.newaxis]
    return out


class TireDetectorPclNode(Node):
    def __init__(self):
        super().__init__("tire_detector_pcl")
        self.declare_parameter("vehicle_boxes_topic", "/aurora_semantic/vehicle_bounding_boxes")
        self.declare_parameter("point_cloud_topic", "/segmentation_processor/registered_pointcloud")
        self.declare_parameter("output_topic", "/tire_bounding_boxes_pcl_fallback")
        self.declare_parameter("target_frame", "slamware_map")
        self.declare_parameter("roi_expansion_m", 0.2)
        self.declare_parameter("tire_z_min_m", 0.0)
        self.declare_parameter("tire_z_max_m", 0.6)
        self.declare_parameter("voxel_leaf_size_m", 0.015)
        self.declare_parameter("dbscan_eps", 0.08)
        self.declare_parameter("dbscan_min_points", 25)
        self.declare_parameter("tire_min_points", 30)
        self.declare_parameter("tire_max_points", 500)
        self.declare_parameter("tire_min_extent_m", 0.15)
        self.declare_parameter("tire_max_extent_m", 0.55)
        self.declare_parameter("process_rate_hz", 3.0)

        self._vehicle_boxes_topic = self.get_parameter("vehicle_boxes_topic").value
        self._point_cloud_topic = self.get_parameter("point_cloud_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._roi_expansion = self.get_parameter("roi_expansion_m").value
        self._tire_z_min = self.get_parameter("tire_z_min_m").value
        self._tire_z_max = self.get_parameter("tire_z_max_m").value
        self._voxel_leaf = self.get_parameter("voxel_leaf_size_m").value
        self._dbscan_eps = self.get_parameter("dbscan_eps").value
        self._dbscan_min = self.get_parameter("dbscan_min_points").value
        self._tire_min_pts = self.get_parameter("tire_min_points").value
        self._tire_max_pts = self.get_parameter("tire_max_points").value
        self._tire_min_ext = self.get_parameter("tire_min_extent_m").value
        self._tire_max_ext = self.get_parameter("tire_max_extent_m").value
        self._process_rate = self.get_parameter("process_rate_hz").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._latest_vehicle_boxes = None
        self._latest_pointcloud = None
        self._latest_pc_header = None

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)

        self._sub_boxes = self.create_subscription(
            BoundingBoxes3d, self._vehicle_boxes_topic, self._vehicle_boxes_cb, qos_rel
        )
        self._sub_pc = self.create_subscription(
            PointCloud2, self._point_cloud_topic, self._pointcloud_cb, qos_be
        )
        self._pub = self.create_publisher(BoundingBoxes3d, self._output_topic, 10)

        period_ns = int(1e9 / self._process_rate)
        self._timer = self.create_timer(period_ns / 1e9, self._process_cb)

        self.get_logger().info(
            f"tire_detector_pcl: vehicle_boxes={self._vehicle_boxes_topic} "
            f"point_cloud={self._point_cloud_topic} -> {self._output_topic}"
        )

    def _vehicle_boxes_cb(self, msg: BoundingBoxes3d):
        self._latest_vehicle_boxes = msg

    def _pointcloud_cb(self, msg: PointCloud2):
        self._latest_pointcloud = msg
        self._latest_pc_header = msg.header

    def _process_cb(self):
        if self._latest_vehicle_boxes is None or self._latest_pointcloud is None:
            return

        boxes_msg = self._latest_vehicle_boxes
        pc_msg = self._latest_pointcloud
        header = self._latest_pc_header

        if not boxes_msg.bounding_boxes:
            out = BoundingBoxes3d()
            out.header = header
            out.header.frame_id = self._target_frame
            out.bounding_boxes = []
            self._pub.publish(out)
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().debug(f"TF lookup failed: {e}")
            return

        points = _pointcloud2_to_xyz(pc_msg)
        if points.size == 0:
            out = BoundingBoxes3d()
            out.header = header
            out.header.frame_id = self._target_frame
            out.bounding_boxes = []
            self._pub.publish(out)
            return

        points_tf = _transform_points(points, transform)

        all_tire_boxes = []
        for box in boxes_msg.bounding_boxes:
            xmin = min(box.xmin, box.xmax) - self._roi_expansion
            xmax = max(box.xmin, box.xmax) + self._roi_expansion
            ymin = min(box.ymin, box.ymax) - self._roi_expansion
            ymax = max(box.ymin, box.ymax) + self._roi_expansion
            zmin = min(box.zmin, box.zmax) - 0.05
            zmax = max(box.zmin, box.zmax) + 0.2

            mask = (
                (points_tf[:, 0] >= xmin) & (points_tf[:, 0] <= xmax) &
                (points_tf[:, 1] >= ymin) & (points_tf[:, 1] <= ymax) &
                (points_tf[:, 2] >= self._tire_z_min) & (points_tf[:, 2] <= self._tire_z_max)
            )
            roi = points_tf[mask]
            if roi.shape[0] < self._dbscan_min:
                continue

            roi = _voxel_downsample(roi, self._voxel_leaf)
            if roi.shape[0] < self._dbscan_min:
                continue

            clustering = DBSCAN(eps=self._dbscan_eps, min_samples=self._dbscan_min)
            labels = clustering.fit_predict(roi)

            for lid in np.unique(labels):
                if lid < 0:
                    continue
                cluster_pts = roi[labels == lid]
                n = len(cluster_pts)
                if n < self._tire_min_pts or n > self._tire_max_pts:
                    continue
                x_ext = np.ptp(cluster_pts[:, 0])
                y_ext = np.ptp(cluster_pts[:, 1])
                ext_xy = max(x_ext, y_ext)
                if ext_xy < self._tire_min_ext or ext_xy > self._tire_max_ext:
                    continue
                cx, cy, cz = np.mean(cluster_pts, axis=0)
                half = ext_xy / 2.0
                half_z = max(0.05, np.ptp(cluster_pts[:, 2]) / 2.0)
                bb = BoundingBox3d()
                bb.object_name = "tire_pcl"
                bb.probability = 0.7
                bb.xmin = cx - half
                bb.xmax = cx + half
                bb.ymin = cy - half
                bb.ymax = cy + half
                bb.zmin = cz - half_z
                bb.zmax = cz + half_z
                all_tire_boxes.append(bb)

        out = BoundingBoxes3d()
        out.header = header
        out.header.frame_id = self._target_frame
        out.bounding_boxes = all_tire_boxes
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TireDetectorPclNode()
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

#!/usr/bin/env python3
"""
System verification script — run before or after launching the stack.
Checks: TF tree, required topics, CUDA, disk space.
Exit 0 if all checks pass, non-zero otherwise.

Simulation mode: publish fake vehicle and tire detections for state-machine testing
without a real car. Run with the stack up (./scripts/start_mission.sh) in another terminal.

Usage:
  python3 scripts/verify_system.py [--skip-ros]
  python3 scripts/verify_system.py --simulate [--duration N] [options]

  --skip-ros     Only run CUDA and disk checks (no ROS/topic/TF).
  --simulate     Publish fake vehicle/tire detections for testing.
  --duration N   Simulate for N seconds (default 60).
  --vehicle-count N   Number of vehicles (default 1).
  --tires-per-vehicle N   Tires per vehicle (default 4).
  --detection-rate R   Probability of publishing each cycle, 0.0-1.0 (default 1.0).
  --position-variance S   Std dev in m for tire position noise (default 0.05).
  --confidence-mean M   Mean confidence for detections (default 0.85).
  --confidence-std S   Std dev of confidence (default 0.05).
  --topic-vehicle T   Vehicle topic (default /aurora_semantic/vehicle_bounding_boxes).
                      Use /sim/vehicle_bounding_boxes when launched via full_bringup with sim_tyre_detections:=true.
  --topic-tire T   Tire topic (default /tire_bounding_boxes_merged).
                   Use /sim/tire_bounding_boxes_merged when launched via full_bringup with sim_tyre_detections:=true.
  --publish-vehicle   Publish synthetic vehicle boxes (default: true in simulate). Set false to use real Aurora vehicles only.
  --no-publish-vehicle   Same as --publish-vehicle=false.
  --vehicle-offset-x X   Vehicle center X in slamware_map (default 3.0).
  --vehicle-offset-y Y   Vehicle center Y (default 0.0).
  --vehicle-confidence C   Vehicle detection confidence (default 0.9).
  --tire-detection-rate R   Probability of publishing tire detections each cycle, 0.0-1.0 (default: same as --detection-rate).
  --tire-drop-probability P   Probability to drop one tire per cycle for occlusion test (default 0.0).
  --publish-objects-segment   Also publish ObjectsSegment for centroid_servo (requires TF).
  --log-level LEVEL   Log level: debug, info, warn, error (default: info).
"""

import argparse
import math
import os
import random
import sys
import time

# Disk thresholds (MB)
DISK_WARN_MB = 1024  # Warn if free < 1 GB
DISK_FAIL_MB = 500  # Fail if free < 500 MB


def check_cuda():
    """Verify CUDA is available and log device name."""
    try:
        import torch

        ok = torch.cuda.is_available()
        if ok:
            name = (
                torch.cuda.get_device_name(0)
                if torch.cuda.device_count()
                else "Unknown"
            )
            print(f"  CUDA: OK (device: {name})")
        else:
            print("  CUDA: FAIL (not available)")
        return ok
    except Exception as e:
        print(f"  CUDA: FAIL ({e})")
        return False


def check_disk(warn_mb=DISK_WARN_MB, fail_mb=DISK_FAIL_MB):
    """Check free space on workspace or root. Warn if < warn_mb, fail if < fail_mb."""
    try:
        ugv_ws = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
        path = ugv_ws if os.path.isdir(ugv_ws) else "/"
        stat = os.statvfs(path)
        free_mb = (stat.f_bavail * stat.f_frsize) // (1024 * 1024)
        if free_mb < fail_mb:
            print(f"  Disk: FAIL (free {free_mb} MB < {fail_mb} MB)")
            return False
        if free_mb < warn_mb:
            print(f"  Disk: WARN (free {free_mb} MB < {warn_mb} MB)")
        else:
            print(f"  Disk: OK (free {free_mb} MB)")
        return True
    except Exception as e:
        print(f"  Disk: FAIL ({e})")
        return False


def check_tf_and_topics(skip_ros=False):
    """Check TF tree and required topics using rclpy. Returns (tf_ok, topics_ok)."""
    if skip_ros:
        return True, True

    if not os.environ.get("ROS_DISTRO"):
        print("  ROS not sourced; skipping TF and topic checks.")
        return True, True

    try:
        import rclpy
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener
    except ImportError as e:
        print(f"  ROS/tf2 import failed: {e}; skipping TF and topic checks.")
        return True, True

    tf_ok = True
    topics_ok = True

    TF_PAIRS = [
        ("slamware_map", "map"),
        ("odom", "slamware_map"),
        ("base_link", "odom"),
        ("base_footprint", "base_link"),
        ("camera_left", "base_link"),
        ("camera_depth_optical_frame", "camera_left"),
    ]

    REQUIRED_TOPICS = [
        "/slamware_ros_sdk_server_node/map",
        "/slamware_ros_sdk_server_node/left_image_raw",
        "/aurora_semantic/vehicle_bounding_boxes",
        "/tire_bounding_boxes_merged",
    ]

    rclpy.init()
    node = Node("verify_system_node")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        time.sleep(0.5)
        for target, source in TF_PAIRS:
            try:
                buffer.lookup_transform(
                    target,
                    source,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1),
                )
                print(f"  TF {source} -> {target}: OK")
            except Exception as e:
                print(f"  TF {source} -> {target}: FAIL ({e})")
                tf_ok = False

        topic_names_and_types = node.get_topic_names_and_types()
        topic_names = [t[0] for t in topic_names_and_types]
        for topic in REQUIRED_TOPICS:
            if topic in topic_names:
                print(f"  Topic {topic}: OK")
            else:
                print(f"  Topic {topic}: FAIL (not found or no publisher)")
                topics_ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return tf_ok, topics_ok


def run_simulate(args) -> int:
    """Publish fake vehicle and tire detections for state-machine testing."""
    if not os.environ.get("ROS_DISTRO"):
        print(
            "ROS not sourced. Run: source /opt/ros/humble/setup.bash (or your distro)"
        )
        return 1

    try:
        import json
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
        from std_msgs.msg import Header, String

        if args.publish_objects_segment:
            from segmentation_msgs.msg import ObjectsSegment, ObjectSegment
            from tf2_ros import Buffer, TransformListener
            from tf2_ros import TransformException
    except ImportError as e:
        print(f"Import failed: {e}. Install: ros-humble-gb-visual-detection-3d-msgs")
        if args.publish_objects_segment:
            print(
                "  For --publish-objects-segment also need: segmentation_msgs, tf2_ros"
            )
        return 1

    rclpy.init()
    node = Node("verify_simulate_detections")

    log_level = getattr(args, "log_level", "info")
    level_map = {
        "debug": 10,
        "info": 20,
        "warn": 30,
        "error": 40,
    }
    node.get_logger().set_level(level_map.get(log_level, 20))

    # Explicit RELIABLE QoS to match inspection_manager subscription
    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )
    vehicle_pub = node.create_publisher(
        BoundingBoxes3d, args.topic_vehicle, qos_profile=qos
    )
    tire_pub = node.create_publisher(
        BoundingBoxes3d, args.topic_tire, qos_profile=qos
    )

    objects_segment_pub = None
    tf_buffer = None
    states_seen = set()
    mission_complete = False
    tires_captured = 0
    mission_start_time = None
    last_state = None
    transition_count = 0

    def on_runtime_diagnostics(msg):
        nonlocal mission_complete, tires_captured, mission_start_time, last_state, transition_count
        try:
            d = json.loads(msg.data) if msg.data else {}
            s = d.get("current_state") or d.get("state") or d.get("mission_state")
            if s:
                states_seen.add(str(s))
                if s != last_state:
                    mission_start_time = mission_start_time or time.time()
                    transition_count += 1
                last_state = s
            tires_captured = max(tires_captured, d.get("tires_captured", 0))
            if s == "DONE" or d.get("mission_complete") or d.get("all_tires_captured"):
                mission_complete = True
        except (json.JSONDecodeError, TypeError, AttributeError):
            pass

    node.create_subscription(
        String,
        "/inspection_manager/runtime_diagnostics",
        on_runtime_diagnostics,
        10,
    )

    if args.publish_objects_segment:
        objects_segment_pub = node.create_publisher(
            ObjectsSegment,
            "/ultralytics_tire/segmentation/objects_segment",
            qos_profile=qos,
        )
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)

    # Vehicle position (configurable via args)
    vehicle_cx = getattr(args, "vehicle_offset_x", 3.0)
    vehicle_cy = getattr(args, "vehicle_offset_y", 0.0)
    vehicle_cz = 0.75
    vehicle_extent_x = 2.25  # half-length
    vehicle_extent_y = 1.0  # half-width
    vehicle_extent_z = 0.75  # half-height
    publish_vehicle = getattr(args, "publish_vehicle", True)
    tire_detection_rate = getattr(args, "tire_detection_rate", args.detection_rate)
    tire_drop_probability = getattr(args, "tire_drop_probability", 0.0)
    vehicle_confidence = getattr(args, "vehicle_confidence", 0.9)

    # Tire positions relative to vehicle center; match vehicle_modeler (box corners with 0.12 inset)
    # Box extent 2.25 x 1.0; half_len=2.25, half_wid=1.0; inset 0.12 -> ±2.13, ±0.88
    tire_offsets = [
        (2.13, -0.88, 0.3),   # front left
        (2.13, 0.88, 0.3),   # front right
        (-2.13, -0.88, 0.3),  # rear left
        (-2.13, 0.88, 0.3),  # rear right
    ]

    def make_header():
        h = Header()
        h.stamp = node.get_clock().now().to_msg()
        h.frame_id = "slamware_map"
        return h

    def make_vehicle_box():
        b = BoundingBox3d()
        b.object_name = "car"
        b.probability = random.gauss(vehicle_confidence, args.confidence_std)
        b.probability = max(0.5, min(1.0, b.probability))
        b.xmin = vehicle_cx - vehicle_extent_x
        b.xmax = vehicle_cx + vehicle_extent_x
        b.ymin = vehicle_cy - vehicle_extent_y
        b.ymax = vehicle_cy + vehicle_extent_y
        b.zmin = vehicle_cz - vehicle_extent_z
        b.zmax = vehicle_cz + vehicle_extent_z
        return b

    def make_tire_box(x: float, y: float, z: float, conf: float):
        half = 0.15
        nx = x + random.gauss(0, args.position_variance)
        ny = y + random.gauss(0, args.position_variance)
        b = BoundingBox3d()
        b.object_name = "wheel"
        b.probability = max(0.5, min(1.0, conf))
        b.xmin = nx - half
        b.xmax = nx + half
        b.ymin = ny - half
        b.ymax = ny + half
        b.zmin = z - 0.1
        b.zmax = z + 0.1
        return b

    def publish_objects_segment():
        if objects_segment_pub is None or tf_buffer is None:
            return
        try:
            t = tf_buffer.lookup_transform(
                "camera_left",
                "slamware_map",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.2),
            )
        except (TransformException, Exception) as e:
            return

        # Camera intrinsics for 416x224 (Aurora depth resolution)
        fx, fy = 500.0, 500.0
        cx, cy = 208.0, 112.0

        def transform_point(tx, ty, tz):
            tr = t.transform.translation
            r = t.transform.rotation
            qx, qy, qz, qw = r.x, r.y, r.z, r.w
            R = [
                [
                    1 - 2 * (qy * qy + qz * qz),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx * qx + qz * qz),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx * qx + qy * qy),
                ],
            ]
            px = R[0][0] * tx + R[0][1] * ty + R[0][2] * tz + tr.x
            py = R[1][0] * tx + R[1][1] * ty + R[1][2] * tz + tr.y
            pz = R[2][0] * tx + R[2][1] * ty + R[2][2] * tz + tr.z
            return (px, py, pz)

        def project(px, py, pz):
            if pz <= 0.01:
                return None
            u = fx * px / pz + cx
            v = fy * py / pz + cy
            return (int(round(u)), int(round(v)))

        msg = ObjectsSegment()
        msg.header = make_header()
        msg.header.frame_id = "camera_left"

        for i, (dx, dy, dz) in enumerate(tire_offsets[: args.tires_per_vehicle]):
            tx = vehicle_cx + dx
            ty = vehicle_cy + dy
            tz = vehicle_cz - vehicle_extent_z + dz
            px, py, pz = transform_point(tx, ty, tz)
            uv = project(px, py, pz)
            if uv is None:
                continue
            u, v = uv
            if u < 0 or u >= 416 or v < 0 or v >= 224:
                continue
            radius = 15
            xi, yi = [], []
            for du in range(-radius, radius + 1):
                for dv in range(-radius, radius + 1):
                    if du * du + dv * dv <= radius * radius:
                        uu, vv = u + du, v + dv
                        if 0 <= uu < 416 and 0 <= vv < 224:
                            xi.append(uu)
                            yi.append(vv)
            if xi:
                obj = ObjectSegment()
                obj.header = msg.header
                obj.class_name = "wheel"
                obj.probability = random.gauss(
                    args.confidence_mean, args.confidence_std
                )
                obj.probability = max(0.5, min(1.0, obj.probability))
                obj.x_indices = xi
                obj.y_indices = yi
                msg.objects.append(obj)

        if msg.objects:
            objects_segment_pub.publish(msg)

    rate_hz = 5.0
    period_s = 1.0 / rate_hz
    start = time.time()
    cycle = 0
    last_heartbeat = start

    node.get_logger().info(
        f"Publishing simulated detections for {args.duration}s at {rate_hz} Hz"
    )
    node.get_logger().info(f"  {args.topic_vehicle} ({args.vehicle_count} vehicle(s))")
    node.get_logger().info(
        f"  {args.topic_tire} ({args.tires_per_vehicle} tires/vehicle)"
    )
    if objects_segment_pub:
        node.get_logger().info(
            "  /ultralytics_tire/segmentation/objects_segment (centroid_servo)"
        )
    sys.stdout.flush()
    sys.stderr.flush()

    try:
        while (time.time() - start) < args.duration:
            try:
                # Process callbacks (e.g. runtime_diagnostics)
                rclpy.spin_once(node, timeout_sec=0.05)

                if random.random() > args.detection_rate:
                    time.sleep(period_s)
                    continue

                stamp = make_header().stamp

                # Vehicle boxes (optional; when false, use real Aurora only)
                if publish_vehicle:
                    vb = BoundingBoxes3d()
                    vb.header.stamp = stamp
                    vb.header.frame_id = "slamware_map"
                    vb.bounding_boxes = [
                        make_vehicle_box() for _ in range(args.vehicle_count)
                    ]
                    vehicle_pub.publish(vb)

                # Tire boxes (all tires for first vehicle)
                if random.random() <= tire_detection_rate:
                    tires = []
                    for dx, dy, dz in tire_offsets[: args.tires_per_vehicle]:
                        conf = random.gauss(
                            args.confidence_mean, args.confidence_std
                        )
                        tires.append(
                            make_tire_box(
                                vehicle_cx + dx,
                                vehicle_cy + dy,
                                vehicle_cz - vehicle_extent_z + dz,
                                conf,
                            )
                        )
                    if (
                        tire_drop_probability > 0
                        and random.random() < tire_drop_probability
                    ):
                        if len(tires) > 1:
                            tires.pop(random.randint(0, len(tires) - 1))
                    elif args.detection_rate < 1.0 and random.random() < 0.1:
                        if len(tires) > 1:
                            tires.pop(random.randint(0, len(tires) - 1))

                    tb = BoundingBoxes3d()
                    tb.header.stamp = stamp
                    tb.header.frame_id = "slamware_map"
                    tb.bounding_boxes = tires
                    tire_pub.publish(tb)

                if objects_segment_pub:
                    publish_objects_segment()

                cycle += 1

                # Heartbeat every ~10 s to confirm loop is running
                now = time.time()
                if now - last_heartbeat >= 10.0:
                    node.get_logger().info(
                        f"Heartbeat: published {cycle} cycles in {now - start:.1f}s"
                    )
                    last_heartbeat = now

            except Exception as e:
                node.get_logger().error(
                    f"Exception in publish cycle {cycle}: {e}",
                    exc_info=True,
                )
                sys.stdout.flush()
                sys.stderr.flush()

            time.sleep(period_s)

    except KeyboardInterrupt:
        node.get_logger().info("Simulate stopped by user.")
    finally:
        mission_duration = (
            (time.time() - mission_start_time) if mission_start_time else 0
        )
        node.get_logger().info("=== Simulation Summary ===")
        node.get_logger().info(f"Published {cycle} cycles.")
        node.get_logger().info(f"Tires captured (from diagnostics): {tires_captured}")
        node.get_logger().info(f"Mission duration (s): {mission_duration:.1f}")
        node.get_logger().info(f"State transitions: {transition_count}")
        if states_seen:
            node.get_logger().info(f"States visited: {', '.join(sorted(states_seen))}")
        node.get_logger().info(
            f"Mission complete: {'yes' if mission_complete else 'no (or inspection_manager not running)'}"
        )
        node.destroy_node()
        rclpy.shutdown()

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Verify system (TF, topics, CUDA, disk) or simulate detections"
    )
    parser.add_argument(
        "--skip-ros", action="store_true", help="Only run CUDA and disk checks"
    )
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="Publish fake vehicle/tire detections for testing",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60,
        help="Simulate duration in seconds (default 60)",
    )
    parser.add_argument(
        "--vehicle-count",
        type=int,
        default=1,
        help="Number of vehicles to simulate (default 1)",
    )
    parser.add_argument(
        "--tires-per-vehicle", type=int, default=4, help="Tires per vehicle (default 4)"
    )
    parser.add_argument(
        "--detection-rate",
        type=float,
        default=1.0,
        help="Probability of publishing each cycle, 0.0-1.0 (default 1.0)",
    )
    parser.add_argument(
        "--position-variance",
        type=float,
        default=0.05,
        help="Std dev in m for tire position noise (default 0.05)",
    )
    parser.add_argument(
        "--confidence-mean",
        type=float,
        default=0.85,
        help="Mean confidence for detections (default 0.85)",
    )
    parser.add_argument(
        "--confidence-std",
        type=float,
        default=0.05,
        help="Std dev of confidence (default 0.05)",
    )
    parser.add_argument(
        "--topic-vehicle",
        type=str,
        default="/aurora_semantic/vehicle_bounding_boxes",
        help="Vehicle detection topic",
    )
    parser.add_argument(
        "--topic-tire",
        type=str,
        default="/tire_bounding_boxes_merged",
        help="Tire detection topic",
    )
    parser.add_argument(
        "--publish-vehicle",
        dest="publish_vehicle",
        action="store_true",
        default=True,
        help="Publish synthetic vehicle boxes (default)",
    )
    parser.add_argument(
        "--no-publish-vehicle",
        dest="publish_vehicle",
        action="store_false",
        help="Do not publish vehicle; use real Aurora only",
    )
    parser.add_argument(
        "--vehicle-offset-x",
        type=float,
        default=3.0,
        help="Vehicle center X in slamware_map (default 3.0)",
    )
    parser.add_argument(
        "--vehicle-offset-y",
        type=float,
        default=0.0,
        help="Vehicle center Y (default 0.0)",
    )
    parser.add_argument(
        "--vehicle-confidence",
        type=float,
        default=0.9,
        help="Vehicle detection confidence (default 0.9)",
    )
    parser.add_argument(
        "--tire-detection-rate",
        type=float,
        default=None,
        help="Tire publish probability per cycle (default: same as --detection-rate)",
    )
    parser.add_argument(
        "--tire-drop-probability",
        type=float,
        default=0.0,
        help="Probability to drop one tire per cycle for occlusion test (default 0)",
    )
    parser.add_argument(
        "--publish-objects-segment",
        action="store_true",
        help="Also publish ObjectsSegment for centroid_servo (requires TF)",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="info",
        choices=["debug", "info", "warn", "error"],
        help="Log level (default: info)",
    )
    args = parser.parse_args()
    if args.tire_detection_rate is None:
        args.tire_detection_rate = args.detection_rate

    if args.simulate:
        sys.exit(run_simulate(args))

    print("=== System verification ===")
    all_ok = True

    ok = check_cuda()
    all_ok = all_ok and ok

    ok = check_disk()
    all_ok = all_ok and ok

    tf_ok, topics_ok = check_tf_and_topics(skip_ros=args.skip_ros)
    all_ok = all_ok and tf_ok and topics_ok

    print("")
    if all_ok:
        print("System verification PASSED")
        sys.exit(0)
    else:
        print("System verification FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()

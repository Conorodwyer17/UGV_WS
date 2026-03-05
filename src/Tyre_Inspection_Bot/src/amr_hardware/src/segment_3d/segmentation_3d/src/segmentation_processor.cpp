#include "segmentation_processor.h"
#include "point_processor.hpp"
#include <rclcpp/executors.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <limits>
#include <numeric>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
// Canonical wheel class; accept tire/tyre aliases for backward compatibility
bool is_wheel_class(const std::string& class_lower) {
    return class_lower == "wheel" || class_lower == "tire" || class_lower == "tyre" ||
           class_lower == "car-tire" || class_lower == "car_tire" ||
           class_lower == "car-tyre" || class_lower == "car_tyre";
}
}  // namespace

SegmentationProcessor::SegmentationProcessor() : Node("segmentation_processor_node") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    initParams();

    // Callback group: isolate heavy segmentation work so heartbeat/timers can run in parallel (system_reaudit_report)
    segmentation_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = segmentation_callback_group_;
    segmentation_sub_ = this->create_subscription<segmentation_msgs::msg::ObjectsSegment>(
        input_segment_topic_, 10,
        std::bind(&SegmentationProcessor::segmentationCallback, this, std::placeholders::_1),
        sub_opts);
    
    // Use BEST_EFFORT QoS to match inspection manager subscriber
    rclcpp::QoS pub_qos_profile(100);
    pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    boxes3d_pub_ = this->create_publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>(
        output_bbx3d_topic_, pub_qos_profile);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/segmentation_processor/markers", 100);
    debug_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_pointcloud_topic_, 100);
    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        debug_markers_topic_, 100);
    
    // Use BEST_EFFORT QoS to match point cloud publisher (depth_image_proc uses BEST_EFFORT)
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    pointCloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, qos_profile,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            latest_pointcloud_ = msg;
        });

    // Publish empty boxes at 2 Hz so inspection_manager startup sees detection_topic alive
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SegmentationProcessor::heartbeatTimerCallback, this));
    
    RCLCPP_WARN(this->get_logger(), "SegmentationProcessor initialized");
}

void SegmentationProcessor::heartbeatTimerCallback() {
    // Only publish empty heartbeat when no recent detections (avoid overwriting real wheel boxes)
    auto elapsed = std::chrono::steady_clock::now() - last_detection_publish_time_;
    if (std::chrono::duration<double>(elapsed).count() < 2.0) {
        return;
    }
    gb_visual_detection_3d_msgs::msg::BoundingBoxes3d msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = working_frame_;
    boxes3d_pub_->publish(msg);
}

void SegmentationProcessor::initParams() {
    input_segment_topic_ = "/ultralytics/segmentation/objects_segment";
    output_bbx3d_topic_ = "/segmentation_processor/bounding_boxes_3d";
    pointcloud_topic_ = "/slamware_ros_sdk_server_node/point_cloud";  // Aurora SDK default
    debug_pointcloud_topic_ = "/segmentation_processor/debug_pointcloud";
    debug_markers_topic_ = "/segmentation_processor/debug_markers";
    working_frame_ = "map";
    mininum_detection_threshold_ = 0.5f;
    minimum_probability_ = 0.3f;
    min_valid_points_wheel_ = 3;
    wheel_cluster_tolerance_ = 0.18f;
    wheel_max_distance_m_ = 12.0f;

    this->declare_parameter("input_segment_topic", input_segment_topic_);
    this->declare_parameter("output_bbx3d_topic", output_bbx3d_topic_);
    this->declare_parameter("point_cloud_topic", pointcloud_topic_);
    this->declare_parameter("debug_pointcloud_topic", debug_pointcloud_topic_);
    this->declare_parameter("debug_markers_topic", debug_markers_topic_);
    this->declare_parameter("working_frame", working_frame_);
    this->declare_parameter("mininum_detection_threshold", mininum_detection_threshold_);
    this->declare_parameter("minimum_probability", minimum_probability_);
    this->declare_parameter("min_valid_points_wheel", min_valid_points_wheel_);
    this->declare_parameter("wheel_cluster_tolerance", wheel_cluster_tolerance_);
    this->declare_parameter("wheel_max_distance_m", wheel_max_distance_m_);
    this->declare_parameter("interested_classes", std::vector<std::string>{"car", "truck", "bus", "wheel", "tire", "tyre"});
    this->declare_parameter("min_valid_points", 5);
    this->declare_parameter("tf_max_age_ms", 250);
    this->declare_parameter("segment_image_width", 640);   // Aurora left_image_raw 640x480
    this->declare_parameter("segment_image_height", 480);
    this->declare_parameter("voxel_leaf_size", 0.0);  // 0 = adaptive; 0.02-0.05 for vehicles
    this->declare_parameter("pointcloud_max_age_s", 0.5);  // 0 = disabled; reject stale cloud (depth dropout defense)
    this->declare_parameter("sor_enabled", true);  // Statistical outlier removal (OpenNav pattern); reduces noisy 3D boxes
    this->declare_parameter("sor_nb_neighbors", 5);
    this->declare_parameter("sor_std_ratio", 2.0);

    this->get_parameter("input_segment_topic", input_segment_topic_);
    this->get_parameter("output_bbx3d_topic", output_bbx3d_topic_);
    this->get_parameter("point_cloud_topic", pointcloud_topic_);
    this->get_parameter("debug_pointcloud_topic", debug_pointcloud_topic_);
    this->get_parameter("debug_markers_topic", debug_markers_topic_);
    this->get_parameter("working_frame", working_frame_);
    this->get_parameter("mininum_detection_threshold", mininum_detection_threshold_);
    this->get_parameter("minimum_probability", minimum_probability_);
    this->get_parameter("min_valid_points_wheel", min_valid_points_wheel_);
    this->get_parameter("wheel_cluster_tolerance", wheel_cluster_tolerance_);
    this->get_parameter("wheel_max_distance_m", wheel_max_distance_m_);
    this->get_parameter("interested_classes", interested_classes_);
    this->get_parameter("min_valid_points", min_valid_points_);
    this->get_parameter("tf_max_age_ms", tf_max_age_ms_);
    this->get_parameter("segment_image_width", segment_image_width_);
    this->get_parameter("segment_image_height", segment_image_height_);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->get_parameter("pointcloud_max_age_s", pointcloud_max_age_s_);
    this->get_parameter("sor_enabled", sor_enabled_);
    this->get_parameter("sor_nb_neighbors", sor_nb_neighbors_);
    this->get_parameter("sor_std_ratio", sor_std_ratio_);
    last_tf_warn_time_ = rclcpp::Time(0);
    last_detection_publish_time_ = std::chrono::steady_clock::time_point{};

    RCLCPP_WARN(this->get_logger(),
        "Parameters initialized (interested_classes: %zu, min_valid_points: %d, min_valid_points_wheel: %d, segment_image: %dx%d)",
        interested_classes_.size(), min_valid_points_, min_valid_points_wheel_, segment_image_width_, segment_image_height_);
}

void SegmentationProcessor::segmentationCallback(const segmentation_msgs::msg::ObjectsSegment::SharedPtr msg) {
    rclcpp::Time segment_stamp = msg->header.stamp;

    // Use the latest pointcloud
    if (!latest_pointcloud_) {
        RCLCPP_WARN(this->get_logger(), "No PointCloud2 message available");
        return;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr closest_pointcloud_msg = latest_pointcloud_;

    // Reject stale pointcloud (depth dropout defense; failure_injection_defense_report #3)
    if (pointcloud_max_age_s_ > 0.0) {
        try {
            double age_s = (this->now() - rclcpp::Time(closest_pointcloud_msg->header.stamp)).seconds();
            if (age_s > pointcloud_max_age_s_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "pointcloud_stale: age %.2fs > max %.2fs; skipping",
                    age_s, pointcloud_max_age_s_);
                return;
            }
        } catch (const std::exception&) {
            // Clock mismatch; skip age check
        }
    }

    sensor_msgs::msg::PointCloud2 local_pointcloud;
    try {
        geometry_msgs::msg::TransformStamped transform;
        rclcpp::Time lookup_time = closest_pointcloud_msg->header.stamp;
        
        // Check if transform is available
        if (!tf_buffer_->canTransform(working_frame_, closest_pointcloud_msg->header.frame_id,
                                      lookup_time, rclcpp::Duration::from_seconds(5.0))) {
            RCLCPP_WARN(this->get_logger(), "No transform available from %s to %s",
                       closest_pointcloud_msg->header.frame_id.c_str(), working_frame_.c_str());
            return;
        }
        
        transform = tf_buffer_->lookupTransform(
            working_frame_, closest_pointcloud_msg->header.frame_id,
            lookup_time);

        // TF age check (skip if clock sources differ, e.g. Aurora system time vs ROS time)
        try {
            double age_ms = (this->now() - rclcpp::Time(transform.header.stamp)).seconds() * 1000.0;
            if (age_ms > static_cast<double>(tf_max_age_ms_)) {
                if ((this->now() - last_tf_warn_time_).seconds() > 5.0) {
                    RCLCPP_WARN(this->get_logger(),
                        "TF age %.0f ms > tf_max_age_ms %d; skipping frame",
                        age_ms, tf_max_age_ms_);
                    last_tf_warn_time_ = this->now();
                }
                return;
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(this->get_logger(), "TF age check skipped (clock mismatch): %s", e.what());
        }

        tf2::doTransform(*closest_pointcloud_msg, local_pointcloud, transform);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }

    // Support both PointXYZ (Aurora depth-based) and PointXYZRGB; use PointXYZ for compatibility
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    try {
        pcl::fromROSMsg(local_pointcloud, *cloud_pcl);
    } catch (std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting PointCloud2 to PCL: %s", e.what());
        return;
    }

    if (cloud_pcl->empty()) {
        RCLCPP_ERROR(this->get_logger(), "PointCloud is empty after conversion");
        return;
    }

    gb_visual_detection_3d_msgs::msg::BoundingBoxes3d boxes3d_msg;
    boxes3d_msg.header.stamp = local_pointcloud.header.stamp;
    boxes3d_msg.header.frame_id = working_frame_;

    pcl::PointCloud<pcl::PointXYZRGB> debug_cloud;

    for (const auto& object_segment : msg->objects) {
        // Class filter: only process interested classes (avoids false positives like "tv")
        if (!interested_classes_.empty()) {
            std::string cn = object_segment.class_name;
            std::transform(cn.begin(), cn.end(), cn.begin(), ::tolower);
            bool found = false;
            for (const auto& ic : interested_classes_) {
                std::string ic_lower = ic;
                std::transform(ic_lower.begin(), ic_lower.end(), ic_lower.begin(), ::tolower);
                if (cn == ic_lower) {
                    found = true;
                    break;
                }
            }
            if (!found) continue;
        }
        if (object_segment.probability < minimum_probability_) continue;
        calculate_boxes(local_pointcloud, cloud_pcl, object_segment, &boxes3d_msg);

        // Add points from the segment to the debug point cloud (segment coords are RGB image size, e.g. 640x480; point cloud is depth size, e.g. 416x224)
        const int pc_w = static_cast<int>(local_pointcloud.width);
        const int pc_h = static_cast<int>(local_pointcloud.height);
        for (size_t i = 0; i < object_segment.x_indices.size(); ++i) {
            int x_img = object_segment.x_indices[i];
            int y_img = object_segment.y_indices[i];
            int x = (segment_image_width_ > 0 && segment_image_height_ > 0)
                ? static_cast<int>(static_cast<float>(x_img) * pc_w / segment_image_width_)
                : x_img;
            int y = (segment_image_width_ > 0 && segment_image_height_ > 0)
                ? static_cast<int>(static_cast<float>(y_img) * pc_h / segment_image_height_)
                : y_img;
            if (x < 0) x = 0;
            if (x >= pc_w) x = pc_w - 1;
            if (y < 0) y = 0;
            if (y >= pc_h) y = pc_h - 1;

            int pcl_index = (y * pc_w) + x;
            const pcl::PointXYZ& point = cloud_pcl->at(pcl_index);
            if (!std::isnan(point.x)) {
                pcl::PointXYZRGB colored_point;
                colored_point.x = point.x;
                colored_point.y = point.y;
                colored_point.z = point.z;
                colored_point.r = static_cast<uint8_t>((i + 1) * 50 % 255);
                colored_point.g = static_cast<uint8_t>((i + 1) * 100 % 255);
                colored_point.b = static_cast<uint8_t>((i + 1) * 150 % 255);
                debug_cloud.push_back(colored_point);
            }
        }
    }

    // Always publish (even when empty) so inspection_manager startup sees detection_topic alive
    if (!boxes3d_msg.bounding_boxes.empty()) {
        last_detection_publish_time_ = std::chrono::steady_clock::now();
        publish_markers(boxes3d_msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu 3D bounding boxes", boxes3d_msg.bounding_boxes.size());
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "No 3D bounding boxes computed from %zu segmented objects", msg->objects.size());
    }
    boxes3d_pub_->publish(boxes3d_msg);
    
    publish_debug_pointcloud(debug_cloud);
    debug_markers_pub_->publish(center_markers_);
    center_markers_.markers.clear();
}

void SegmentationProcessor::publish_debug_pointcloud(const pcl::PointCloud<pcl::PointXYZRGB>& debug_cloud) {
    sensor_msgs::msg::PointCloud2 debug_cloud_msg;
    pcl::toROSMsg(debug_cloud, debug_cloud_msg);
    debug_cloud_msg.header.frame_id = working_frame_;
    debug_cloud_msg.header.stamp = this->now();
    debug_pointcloud_pub_->publish(debug_cloud_msg);
}

void SegmentationProcessor::calculate_boxes(const sensor_msgs::msg::PointCloud2& cloud_pc2,
                                            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_pcl,
                                            const segmentation_msgs::msg::ObjectSegment& object_segment,
                                            gb_visual_detection_3d_msgs::msg::BoundingBoxes3d* boxes) {
    // Step 1: Create a point cloud for the object mask
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    object_cloud->is_dense = false;

    // Step 2: Populate object point cloud with only valid points from the object mask
    // Segment indices are in RGB image space (e.g. 640x480); scale to point cloud (e.g. 416x224)
    const int pc_w = static_cast<int>(cloud_pc2.width);
    const int pc_h = static_cast<int>(cloud_pc2.height);
    int valid_points = 0;
    for (size_t i = 0; i < object_segment.x_indices.size(); ++i) {
        int x_img = object_segment.x_indices[i];
        int y_img = object_segment.y_indices[i];
        int x = (segment_image_width_ > 0 && segment_image_height_ > 0)
            ? static_cast<int>(static_cast<float>(x_img) * pc_w / segment_image_width_)
            : x_img;
        int y = (segment_image_width_ > 0 && segment_image_height_ > 0)
            ? static_cast<int>(static_cast<float>(y_img) * pc_h / segment_image_height_)
            : y_img;
        if (x < 0) x = 0;
        if (x >= pc_w) x = pc_w - 1;
        if (y < 0) y = 0;
        if (y >= pc_h) y = pc_h - 1;

        int pcl_index = (y * pc_w) + x;
        const pcl::PointXYZ& point = cloud_pcl->at(pcl_index);

        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
            continue;
        }

        pcl::PointXYZ valid_point;
        valid_point.x = point.x;
        valid_point.y = point.y;
        valid_point.z = point.z;
        object_cloud->points.push_back(valid_point);
        valid_points++;
    }

    int required_points = min_valid_points_;
    std::string class_lower = object_segment.class_name;
    std::transform(class_lower.begin(), class_lower.end(), class_lower.begin(), ::tolower);
    if (is_wheel_class(class_lower)) {
        required_points = min_valid_points_wheel_;
    }
    if (valid_points == 0 || valid_points < required_points) {
        RCLCPP_WARN(this->get_logger(), "Insufficient valid points for object %s: %d (min %d)",
            object_segment.class_name.c_str(), valid_points, required_points);
        return;
    }

    object_cloud->width = object_cloud->points.size();
    object_cloud->height = 1;
    RCLCPP_INFO(this->get_logger(), "Extracted %d valid points for object: %s", valid_points, object_segment.class_name.c_str());


    // Step 3-4: Segment the ground plane (skip for wheels: wheel face can be largest plane and would be removed)
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int number_of_planes = is_wheel_class(class_lower) ? 0 : 1;
    if (!segment_cloud(object_cloud, segmented_cloud, number_of_planes, voxel_leaf_size_)) {
        RCLCPP_WARN(this->get_logger(), "Failed to segment point cloud for object: %s", object_segment.class_name.c_str());
        return;
    }

    if (segmented_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Segmented cloud is empty for object: %s", object_segment.class_name.c_str());
        return;
    }

    // Step 5: Cluster the segmented cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    double cluster_tolerance = 0.1;  // default clustering tolerance
    int min_cluster_size = std::min(50, static_cast<int>(segmented_cloud->points.size() / 10));
    if (is_wheel_class(class_lower)) {
        cluster_tolerance = wheel_cluster_tolerance_;
        // Wheels: very relaxed clustering for sparse depth at distance (1-2 points can be valid)
        min_cluster_size = 1;
    }
    int max_cluster_size = 40000;
    if (!cluster_cloud(segmented_cloud, best_cluster, cluster_tolerance, min_cluster_size, max_cluster_size)) {
        if (is_wheel_class(class_lower) && !segmented_cloud->points.empty()) {
            // Fallback for wheels: use all points as cluster when clustering fails (sparse depth at distance)
            *best_cluster = *segmented_cloud;
            RCLCPP_DEBUG(this->get_logger(), "Wheel cluster fallback: using all %zu points", segmented_cloud->points.size());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to cluster point cloud for object: %s (segmented cloud had %zu points)",
                       object_segment.class_name.c_str(), segmented_cloud->points.size());
            return;
        }
    }

    if (best_cluster->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Best cluster is empty for object: %s", object_segment.class_name.c_str());
        return;
    }

    // Optional: Statistical outlier removal (OpenNav pattern) — reduces noisy 3D boxes from depth artifacts
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_for_bbox(best_cluster);
    if (sor_enabled_ && best_cluster->points.size() >= static_cast<size_t>(sor_nb_neighbors_ + 1)) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(best_cluster);
        sor.setMeanK(sor_nb_neighbors_);
        sor.setStddevMulThresh(sor_std_ratio_);
        sor.filter(*sor_cloud);
        if (!sor_cloud->points.empty()) {
            cluster_for_bbox = sor_cloud;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << " ****########****** Clustering processing  took " << duration.count()
              << " milliseconds." << std::endl;

    // Step 6: Compute 3D bounding box from the best cluster
    float maxx, minx, maxy, miny, maxz, minz;
    maxx = maxy = maxz = -std::numeric_limits<float>::max();
    minx = miny = minz = std::numeric_limits<float>::max();

    for (const auto& point : cluster_for_bbox->points) {
        if (std::isnan(point.x)) continue;

        maxx = std::max(point.x, maxx);
        maxy = std::max(point.y, maxy);
        maxz = std::max(point.z, maxz);
        minx = std::min(point.x, minx);
        miny = std::min(point.y, miny);
        minz = std::min(point.z, minz);
    }
    if (is_wheel_class(class_lower)) {
        float max_range = wheel_max_distance_m_;
        float max_xy = std::max(std::max(std::abs(maxx), std::abs(minx)), std::max(std::abs(maxy), std::abs(miny)));
        if (max_xy > max_range) {
            RCLCPP_WARN(this->get_logger(), "Wheel box exceeds max distance %.1fm (max_xy=%.2f), skipping",
                max_range, max_xy);
            return;
        }
    }

    // Step 7: Create and store the bounding box message
    gb_visual_detection_3d_msgs::msg::BoundingBox3d bbx_msg;
    // bbx_msg.class_ = object_segment.class_name;
    bbx_msg.object_name = object_segment.class_name;
    bbx_msg.probability = object_segment.probability;
    bbx_msg.xmin = minx;
    bbx_msg.xmax = maxx;
    bbx_msg.ymin = miny;
    bbx_msg.ymax = maxy;
    bbx_msg.zmin = minz;
    bbx_msg.zmax = maxz;

    boxes->bounding_boxes.push_back(bbx_msg);
}


pcl::PointXYZRGB SegmentationProcessor::compute_center_point(const sensor_msgs::msg::PointCloud2& cloud_pc2,
                                                             const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_pcl,
                                                             const segmentation_msgs::msg::ObjectSegment& object_segment) {
    if (object_segment.x_indices.empty() || object_segment.y_indices.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Object segment has no valid indices");
        return pcl::PointXYZRGB();
    }

    const int pc_w = static_cast<int>(cloud_pc2.width);
    const int pc_h = static_cast<int>(cloud_pc2.height);
    const float scale_x = (segment_image_width_ > 0) ? static_cast<float>(pc_w) / segment_image_width_ : 1.0f;
    const float scale_y = (segment_image_height_ > 0) ? static_cast<float>(pc_h) / segment_image_height_ : 1.0f;

    int center_x_img = std::accumulate(object_segment.x_indices.begin(), object_segment.x_indices.end(), 0) / object_segment.x_indices.size();
    int center_y_img = std::accumulate(object_segment.y_indices.begin(), object_segment.y_indices.end(), 0) / object_segment.y_indices.size();
    int center_x = static_cast<int>(center_x_img * scale_x);
    int center_y = static_cast<int>(center_y_img * scale_y);
    if (center_x < 0) center_x = 0;
    if (center_x >= pc_w) center_x = pc_w - 1;
    if (center_y < 0) center_y = 0;
    if (center_y >= pc_h) center_y = pc_h - 1;
    std::vector<int> xs = object_segment.x_indices;
    std::vector<int> ys = object_segment.y_indices;
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());

    int min_x = static_cast<int>(xs.front() * scale_x);
    int max_x = static_cast<int>(xs.back() * scale_x);
    int min_y = static_cast<int>(ys.front() * scale_y);
    int max_y = static_cast<int>(ys.back() * scale_y);
    if (min_x < 0) min_x = 0;
    if (max_x >= pc_w) max_x = pc_w - 1;
    if (min_y < 0) min_y = 0;
    if (max_y >= pc_h) max_y = pc_h - 1;

    int width = max_x - min_x;
    int height = max_y - min_y;

    // Convert num_samples to be in terms of height and width percentages
    int num_samples = 500; // Example number of samples
    float width_percentage = 0.12f;  // 55% of the bounding box width
    float height_percentage = 0.15f; // 35% of the bounding box height

    int width_step = static_cast<int>(width * width_percentage) / num_samples;
    int height_step = static_cast<int>(height * height_percentage) / num_samples;

    std::vector<pcl::PointXYZRGB> points;

    // Sampling in the region defined by the bounding box
    for (int i = -num_samples / 2; i <= num_samples / 2; ++i) {
        for (int j = -num_samples / 2; j <= num_samples / 2; ++j) {
            int new_x = center_x + i * width_step;
            int new_y = center_y + j * height_step;

            // Ensure the new points are within the image bounds
            if (new_x < 0 || new_x >= static_cast<int>(cloud_pc2.width) || new_y < 0 || new_y >= static_cast<int>(cloud_pc2.height)) {
                continue;
            }

            int pcl_index = (new_y * cloud_pc2.width) + new_x;
            const pcl::PointXYZ& pt = cloud_pcl->at(pcl_index);

            // Check if the point is valid
            if (!std::isnan(pt.x)) {
                pcl::PointXYZRGB point;
                point.x = pt.x;
                point.y = pt.y;
                point.z = pt.z;
                point.r = point.g = point.b = 128;
                points.push_back(point);
            }
        }
    }

    // If no valid points were found, return a default invalid point
    if (points.empty()) {
        return pcl::PointXYZRGB();
    }

    // Find the point with the minimum x value
    pcl::PointXYZRGB min_x_point = points[0];
    for (const auto& pt : points) {
        if (pt.x < min_x_point.x) {
            min_x_point = pt;
        }
    }
    return min_x_point;
}

void SegmentationProcessor::publish_markers(const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d& boxes) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < boxes.bounding_boxes.size(); ++i) {
        const auto& box = boxes.bounding_boxes[i];

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = boxes.header.frame_id;
        marker.header.stamp = boxes.header.stamp;
        marker.ns = "segmentation_3d";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = (box.xmax + box.xmin) / 2.0;
        marker.pose.position.y = (box.ymax + box.ymin) / 2.0;
        marker.pose.position.z = (box.zmax + box.zmin) / 2.0;
        marker.scale.x = (box.xmax - box.xmin);
        marker.scale.y = (box.ymax - box.ymin) ;
        marker.scale.z = box.zmax - box.zmin;

        marker.color.r = 1.0 - box.probability;
        marker.color.g = box.probability;
        marker.color.b = 0.0;
        marker.color.a = 0.6;

        marker_array.markers.push_back(marker);
    }

    markers_pub_->publish(marker_array);
}

void SegmentationProcessor::push_center_marker(const pcl::PointXYZRGB& center){

    static int i_ = 0;
     visualization_msgs::msg::Marker marker;
     marker.header.frame_id = working_frame_;
     marker.header.stamp = this->now();
     marker.ns = "segmentation_3d_CENTER";
     marker.id = i_++;
     marker.type = visualization_msgs::msg::Marker::CUBE;
     marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = center.x;
        marker.pose.position.y = center.y;
        marker.pose.position.z = center.z;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0;
        marker.color.g = 0.3;
        marker.color.b = 0.3;
        marker.color.a = 0.6;

        center_markers_.markers.push_back(marker);

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SegmentationProcessor>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);  // 2 threads: segmentation + heartbeat/pointcloud
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

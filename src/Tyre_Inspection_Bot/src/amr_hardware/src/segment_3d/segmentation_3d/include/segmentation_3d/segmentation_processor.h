#ifndef SEGMENTATION_PROCESSOR_H
#define SEGMENTATION_PROCESSOR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <segmentation_msgs/msg/objects_segment.hpp>
#include <segmentation_msgs/msg/object_segment.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

#include <gb_visual_detection_3d_msgs/msg/bounding_boxes3d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <memory>

class SegmentationProcessor : public rclcpp::Node {
public:
    SegmentationProcessor();
    void segmentationCallback(const segmentation_msgs::msg::ObjectsSegment::SharedPtr msg);
    void initParams();
    void publish_markers(const gb_visual_detection_3d_msgs::msg::BoundingBoxes3d& boxes);
    void calculate_boxes(const sensor_msgs::msg::PointCloud2& cloud_pc2,
                         const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_pcl,
                         const segmentation_msgs::msg::ObjectSegment& object_segment,
                         gb_visual_detection_3d_msgs::msg::BoundingBoxes3d* boxes);
    void publish_debug_pointcloud(const pcl::PointCloud<pcl::PointXYZRGB>& debug_cloud);
  
    pcl::PointXYZRGB compute_center_point(const sensor_msgs::msg::PointCloud2& cloud_pc2,
                                          const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_pcl,
                                          const segmentation_msgs::msg::ObjectSegment& object_segment);
    void push_center_marker(const pcl::PointXYZRGB& center);

private:
    rclcpp::Subscription<segmentation_msgs::msg::ObjectsSegment>::SharedPtr segmentation_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;

    rclcpp::Publisher<gb_visual_detection_3d_msgs::msg::BoundingBoxes3d>::SharedPtr boxes3d_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string input_segment_topic_;
    std::string output_bbx3d_topic_;
    std::string pointcloud_topic_;
    std::string debug_pointcloud_topic_;
    std::string debug_markers_topic_;
    std::string working_frame_;
    std::vector<std::string> interested_classes_;
    float mininum_detection_threshold_, minimum_probability_;
    int min_valid_points_;
    int min_valid_points_tire_;
    int tf_max_age_ms_;
    float tire_cluster_tolerance_;
    float tire_max_distance_m_;
    int segment_image_width_;   // RGB image size (e.g. 640 for Aurora left_image_raw)
    int segment_image_height_;  // e.g. 480; segment indices are in this space, point cloud may be 416x224
    rclcpp::Time last_tf_warn_time_;
    visualization_msgs::msg::MarkerArray center_markers_;
};

#endif // SEGMENTATION_PROCESSOR_H

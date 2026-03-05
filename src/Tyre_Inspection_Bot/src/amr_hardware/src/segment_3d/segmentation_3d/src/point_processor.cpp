#include "point_processor.hpp"
#include <pcl/filters/passthrough.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <algorithm>
#include <chrono>
#include <limits>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h> // Add this header for compute3DCentroid
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h> // filtering
#include <pcl/exceptions.h> // For PCLException
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <random>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
bool filter_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered,
                  double leaf_size)
{
    if (!cloud_in || cloud_in->points.empty())
    {
        std::cerr << "Input cloud is empty or null." << std::endl;
        return false;
    }

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);

    try
    {
        sor.filter(*cloud_filtered);
    }
    catch (const pcl::PCLException &e)
    {
        std::cerr << "Error during filtering: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void findMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_segmented)
{
    // Initialize variables to store min and max values
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();

    // Iterate through each point in the cloud
    for (auto &point : cloud_segmented->points)
    {
        // Update minimum and maximum values for each dimension
        if (point.x < minX)
            minX = point.x;
        if (point.y < minY)
            minY = point.y;
        if (point.z < minZ)
            minZ = point.z;
        if (point.x > maxX)
            maxX = point.x;
        if (point.y > maxY)
            maxY = point.y;
        if (point.z > maxZ)
            maxZ = point.z;
    }

    // Output the results
    // std::cout << "Minimum values:" << std::endl;
    // std::cout << "x: " << minX << std::endl;
    // std::cout << "y: " << minY << std::endl;
    // std::cout << "z: " << minZ << std::endl;

    // std::cout << "Maximum values:" << std::endl;
    // std::cout << "x: " << maxX << std::endl;
    // std::cout << "y: " << maxY << std::endl;
    // std::cout << "z: " << maxZ << std::endl;
}



// Main segmentation function incorporating iterative ground and wall plane
// removal
bool segment_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_segmented,
                   const int number_of_planes,
                   float voxel_leaf_size_param)
{
    if (!cloud_in || cloud_in->points.empty()) {
        std::cerr << "Input cloud is empty or null." << std::endl;
        return false;
    }

    // Step 0: Passthrough filter - drop far/high points before VoxelGrid (speeds clustering)
    // z > 2.0 m: above tire height; typical tire inspection: 0.3-2 m. Autoware-style preprocessing.
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-5.0f, 2.0f);
    pass.filter(*passthrough_cloud);
    if (passthrough_cloud->points.empty()) {
        *passthrough_cloud = *cloud_in;
    }
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_for_voxel = passthrough_cloud;

    // Step 1: Voxel filter before clustering (Autoware-style: downsample before EuclideanClusterExtraction)
    // Research: reduces point count significantly; 0.02-0.05m typical for vehicle-sized objects
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Calculate bounding box to determine appropriate leaf size
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_for_voxel, min_pt, max_pt);
    float cloud_size_x = max_pt.x - min_pt.x;
    float cloud_size_y = max_pt.y - min_pt.y;
    float cloud_size_z = max_pt.z - min_pt.z;
    float max_extent = std::max({cloud_size_x, cloud_size_y, cloud_size_z});
    
    // Configurable voxel_leaf_size > 0 overrides adaptive; 0 = use adaptive (best_practices_matrix)
    float leaf_size;
    if (voxel_leaf_size_param > 0.0f) {
        leaf_size = voxel_leaf_size_param;
    } else {
        // Avoid PCL integer overflow: max ~80 voxels per axis (80^3 = 512k safe)
        leaf_size = std::max(0.05f, max_extent / 80.0f);
        leaf_size = std::min(leaf_size, 0.2f);
        // Tuning: 0.02-0.05 typical for vehicle-sized objects (research: l2i, nav2_voxel_grid)
        // For large clouds (>20k): use more aggressive leaf size to bring count down (speeds clustering)
        size_t n = cloud_for_voxel->points.size();
        if (n > 20000) {
            leaf_size = std::max(leaf_size, 0.08f);  // Larger voxels = fewer points
        }
    }
    
    try {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud_for_voxel);
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.filter(*filtered_cloud);
        
        if (filtered_cloud->points.empty()) {
            *filtered_cloud = *cloud_for_voxel;
        }
    } catch (const pcl::PCLException& e) {
        std::cerr << "VoxelGrid error: " << e.what() << ". Using passthrough cloud." << std::endl;
        *filtered_cloud = *cloud_for_voxel;
    }

    // Step 2: Set up segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(300); // Set max iterations
    seg.setDistanceThreshold(0.0252); // Threshold for plane distance

    int planes_found = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud = filtered_cloud;

    // Step 3: Iterative plane segmentation and extraction
    while (planes_found < number_of_planes) {
        // Segment the largest plane in the current cloud
        seg.setInputCloud(remaining_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
            break; // Exit loop if no plane is found
        }

        // Extract the remaining points after removing the plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setInputCloud(remaining_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true); // Keep the points that are not in the plane
        extract.filter(*temp_cloud);

        remaining_cloud.swap(temp_cloud); // Efficiently swap the point clouds

        // Stop if the cloud becomes too small to continue
        if (remaining_cloud->points.size() < 700) {
            break;
        }

        planes_found++;
    }

    // Step 4: Set the final segmented cloud to the remaining points
    cloud_segmented.swap(remaining_cloud); // Swap is faster than assigning

    return true;
}


bool cluster_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster,
                   double cluster_tolerance,
                   int min_cluster_size,
                   int max_cluster_size)
{
    if (!cloud_in || cloud_in->points.empty()) {
        return false;
    }

    // Create a KdTree for the input cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);

    // Vector to store the indices of the points in each cluster
    std::vector<pcl::PointIndices> cluster_indices;

    // Perform Euclidean Cluster Extraction
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);

    // Initialize variables to track the largest cluster
    size_t largest_cluster_size = 0;
    pcl::PointIndices::Ptr largest_cluster_indices(new pcl::PointIndices);

    // Loop through each cluster to find the largest one
    for (const auto& indices : cluster_indices) {
        if (indices.indices.size() > largest_cluster_size) {
            largest_cluster_size = indices.indices.size();
            *largest_cluster_indices = indices;  // Store the indices of the largest cluster
        }
    }

    // Check if a cluster was found
    if (largest_cluster_size == 0) {
        return false; // No clusters found
    }

    // Extract the largest cluster from the input cloud
    best_cluster->points.reserve(largest_cluster_indices->indices.size());
    for (const auto& idx : largest_cluster_indices->indices) {
        best_cluster->points.push_back(cloud_in->points[idx]);
    }

    best_cluster->width = best_cluster->points.size();
    best_cluster->height = 1;
    best_cluster->is_dense = true;

    return true;
}

//
// Created by zlc on 2021/4/25.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>             // 体素滤波

#include "points_downsample.h"

#define MAX_MEASUREMENT_RANGE 120.0

ros::Publisher filter_points_pub;               // ROS发布消息

// Leaf size of VoxelGrid filter
static double voxel_leaf_size = 2.0;

static bool _output_log = false;
static std::ofstream ofs;
static std::string filename;

static std::string POINTS_TOPIC;
static double measurement_range = MAX_MEASUREMENT_RANGE;


static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ> scan;
    pcl::fromROSMsg(*input, scan);

    if (measurement_range != MAX_MEASUREMENT_RANGE)
    {
        scan = removePointByRange(scan, 0, measurement_range);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    sensor_msgs::PointCloud2 filtered_msg;

    // if voxel_leaf_size < 0.1 voxel_grid_filter

}








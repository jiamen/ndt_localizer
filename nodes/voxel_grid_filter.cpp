//
// Created by zlc on 2021/4/25.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>             // 体素滤波

#include "points_downsample.h"

#define MAX_MEASUREMENT_RANGE 120.0             // 最大的测量范围

ros::Publisher filtered_points_pub;             // ROS发布消息

// Leaf size of VoxelGrid filter
static double voxel_leaf_size = 2.0;

static bool _output_log = false;                // 是否保留输出日志
static std::ofstream ofs;                       // 文件输入输出操作
static std::string filename;                    // 文件名

static std::string POINTS_TOPIC;
static double measurement_range = MAX_MEASUREMENT_RANGE;


static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ> scan;
    pcl::fromROSMsg(*input, scan);          // 将ROS消息转换为PCL下的数据结构

    if (measurement_range != MAX_MEASUREMENT_RANGE)
    {
        scan = removePointByRange(scan, 0, measurement_range);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));         // 过滤前的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());    // 过滤后的点云

    // 要发布的ROS消息
    sensor_msgs::PointCloud2 filtered_msg;

    // 点云降采样
    // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL).
    if ( voxel_leaf_size >= 0.1 )
    {
        // Downsampling the velodyne scan using VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter.setInputCloud(scan_ptr);
        voxel_grid_filter.filter(*filtered_scan_ptr);
        pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);        // 将pcl点云转换成为filtered_msg
    }
    else
    {
        pcl::toROSMsg(*scan_ptr, filtered_msg);
    }

    filtered_msg.header = input->header;
    filtered_points_pub.publish(filtered_msg);
}


int main(int argc, char* *argv)
{
    ros::init(argc, argv, "voxel_grid_filter");

    ros::NodeHandle nh;                         // 句柄，  nh命名空间为/node_namespace，在launch文件中指定
    ros::NodeHandle private_nh("~");        // 句柄，nh命名空间为/node_namespace/voxel_grid_filter/

    private_nh.getParam("points_topic", POINTS_TOPIC);
    private_nh.getParam("output_log", _output_log);

    private_nh.param<double>("leaf_size", voxel_leaf_size, 2.0);
    ROS_INFO_STREAM("Voxel leaf size is: " << voxel_leaf_size);
    if (_output_log == true)
    {
        char buffer[80];
        std::time_t now = std::time(NULL);
        std::tm *pnow = std::localtime(&now);
        std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
        filename = "voxel_grid_filter_" + std::string(buffer) + ".csv";
        ofs.open(filename.c_str(), std::ios::app);
    }

    // Publisher    发布滤波后的点云
    filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

    // Subscribers  订阅点云数据
    ros::Subscriber scan_sub = nh.subscribe(POINTS_TOPIC, 10, scan_callback);

    ros::spin();

    return 0;
}





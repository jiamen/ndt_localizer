//
// Created by zlc on 2021/4/25.
//

#include "ndt.h"

NdtLocalizer::NdtLocalizer(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_)          // 注意这三个初始化
{
    key_value_stdmap_["state"] = "Initializing";
    init_params();

    // Publishers
    sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
    ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
    exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);
    transform_probability_pub_ = nh_.advertise<std_msgs::Float32>("transform_probability", 10);
    iteration_num_pub_ = nh_.advertise<std_msgs::Float32>("iteration_num", 10);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);

    // Subscribers
    initial_pose_sub_ = nh_.subscribe("initialpose", 100, &NdtLocalizer::callback_init_pose, this);
    map_points_sub_ = nh_.subscribe("points_map", 1, &NdtLocalizer::callback_pointsmap, this);
    sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &NdtLocalizer::callback_pointcloud, this);

    diagnostic_thread_ = std::thread(&NdtLocalizer::timer_diagnostic, this);
    diagnostic_thread_.detach();
}

NdtLocalizer::~NdtLocalizer() {}




int main(int argc, char* *argv)
{
    // launch 文件中指定的是
    ros::init(argc, argv, "ndt_localizer");     // node name

    ros::NodeHandle nh;                     // 句柄，  nh命名空间为/node_namespace，在launch文件中指定
    ros::NodeHandle private_nh("~");    // 句柄，nh命名空间为/node_namespace/ndt_localizer/

    NdtLocalizer ndt_localizer(nh, private_nh);

    ros::spin();

    return 0;
}



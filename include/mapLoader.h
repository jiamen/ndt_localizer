//
// Created by zlc on 2021/4/25.
//

#ifndef _NDT_LOCALIZER_MAPLOADER_H_
#define _NDT_LOCALIZER_MAPLOADER_H_

#include <pcl/point_cloud.h>            // 点云
#include <pcl/point_types.h>            // 点的类型
#include <pcl/filters/voxel_grid.h>     // 点云滤波
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>                    // ros相关头文件
#include <sensor_msgs/Imu.h>            // ros下的IMU传感器数据
#include <sensor_msgs/PointCloud2.h>    // ros下的点云数据
#include <vector>
#include <pcl_ros/transforms.h>


class MapLoader
{
public:
    ros::Publisher pc_map_pub_;
    std::vector<std::string> file_list_;

    MapLoader(ros::NodeHandle& nh);


private:
    float tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_;

    void init_tf_params(ros::NodeHandle& nh);
    sensor_msgs::PointCloud2 CreatePcd();
    sensor_msgs::PointCloud2 TransformMap(sensor_msgs::PointCloud2& in);

    void SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr);

};



#endif // _NDT_LOCALIZER_MAPLOADER_H_

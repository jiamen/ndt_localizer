//
// Created by zlc on 2021/4/25.
//

#ifndef _NDT_LOCALIZER_POINTS_DOWNSAMPLE_H_
#define _NDT_LOCALIZER_POINTS_DOWNSAMPLE_H_


static pcl::PointCloud<pcl::PointXYZ> removePointByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
    narrowed_scan.header = scan.header;

#if 1     //  This error handling should be detemind.
    if (min_range >= max_range)
    {
        ROS_ERROR_ONCE("min_range>=max_range @");
        return scan;
    }
#endif

    double square_min_range = min_range * min_range;        // 最小平方距离
    double square_max_range = max_range * max_range;        // 最大平方距离

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = scan.begin(); iter != scan.end(); ++ iter)
    {
        const pcl::PointXYZ& p = *iter;
        // p.x = iter->x;
        // p.y = iter->y;
        // p.z = iter->z;
        // p.intensity = iter->intensity;

        // 用来过滤距离车辆较近以及较远的点云数据
        double square_distance = p.x * p.x + p.y * p.y;
        if (square_min_range <= square_distance && square_distance <= square_max_range)
        {
            narrowed_scan.points.push_back(p);
        }
    }

    return narrowed_scan;
}


#endif // _NDT_LOCALIZER_POINTS_DOWNSAMPLE_H_

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
    // 一切使用预先构建的地图进行配准定位的方法都需要提供初始位姿，在工业界的实践中，这一初始姿态通常是通过gnss获得，
    // 本文中我们简化这一步，在Rviz中手动指定初始姿态，Rviz中设定的初始姿态会被默认发送至/initialpose topic上，
    // 在NdtLocalizer构造函数中，写这个subscriber监听该topic
    initial_pose_sub_ = nh_.subscribe("initialpose", 100, &NdtLocalizer::callback_init_pose, this);
    // 监听mapLoader节点发来的点云地图message
    map_points_sub_ = nh_.subscribe("points_map", 1, &NdtLocalizer::callback_pointsmap, this);
    sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &NdtLocalizer::callback_pointcloud, this);

    diagnostic_thread_ = std::thread(&NdtLocalizer::timer_diagnostic, this);
    diagnostic_thread_.detach();
}

NdtLocalizer::~NdtLocalizer() {}


void NdtLocalizer::timer_diagnostic()
{
    ros::Rate rate(100);
    while (ros::ok())
    {
        diagnostic_msgs::DiagnosticStatus diag_status_msg;      // 诊断信息
        diag_status_msg.name = "ndt_scan_mather";
        diag_status_msg.hardware_id = "";

        for (const auto & key_value : key_value_stdmap_)
        {
            diagnostic_msgs::KeyValue key_value_msg;
            key_value_msg.key = key_value.first;
            key_value_msg.value = key_value.second;
            diag_status_msg.values.push_back(key_value_msg);
        }

        diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
        diag_status_msg.message = "";

        if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing")
        {
            diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
            diag_status_msg.message += "Initializing State. ";
        }
        if (key_value_stdmap_.count("skipping_publish_num") &&
                std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1)
        {
            diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
            diag_status_msg.message += "skipping_publish_num > 1. ";
        }
        if (key_value_stdmap_.count("skipping_publish_num") &&
                std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5)
        {
            diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            diag_status_msg.message += "skipping_publish_num exceed limit. ";
        }

        diagnostic_msgs::DiagnosticArray diag_msg;              // 待发布的诊断信息
        diag_msg.header.stamp = ros::Time::now();
        diag_msg.status.push_back(diag_status_msg);

        diagnostics_pub_.publish(diag_msg);

        rate.sleep();
    }
}

// 初始位姿获取
void NdtLocalizer::callback_init_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial_pose_msg_ptr)
{
    // 初始位姿获取，如果Rviz上手动指定的初始位姿后，首先对坐标系进行统一，如果该pose是在地图坐标系，那么保存用于后续使用
    if (initial_pose_msg_ptr->header.frame_id == map_frame_)
    {
        initial_pose_cov_msg_ = *initial_pose_msg_ptr;
    }
    // 如果是其他坐标系，则先将该pose转换至地图坐标系，通过函数get_transform()获取变换关系
    else
    {
        // get TF from pose_frame to map_frame   得到坐标系之间的变换矩阵
        geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
        get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

        // transform pose_frame to map_frame      通过坐标系之间的变换矩阵将指定初始位姿进行变换，得到地图坐标系下的初始位姿
        geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(new geometry_msgs::PoseWithCovarianceStamped);
        tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
        initial_pose_cov_msg_ = *mapTF_initial_pose_msg_ptr;
    }

    // if click the initpose again, re init!
    init_pose_ = false;
}

// 初始化地图：NDT配准中的目标点云，SC-LEGO-LOAM构建的点云地图
void NdtLocalizer::callback_pointsmap(const sensor_msgs::PointCloud2::ConstPtr& map_points_msg_ptr)
{
    const auto trans_epsilon = ndt_.getTransformationEpsilon();
    const auto step_size = ndt_.getStepSize();
    const auto resolution = ndt_.getResolution();
    const auto max_iterations = ndt_.getMaximumIterations();

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;        // NDT配准
    // 设置变换的 ϵ（两个连续变换之间允许的最大差值），这是判断我们的优化过程是否已经收敛到最终解的阈值。
    ndt_new.setTransformationEpsilon(trans_epsilon);
    // 设置牛顿法优化的最大步长
    ndt_new.setStepSize(step_size);
    // 设置网格化时立方体的边长，网格大小设置在NDT中非常重要，太大会导致精度不高，太小导致内存过高，并且只有两幅点云相差不大的情况才能匹配。
    ndt_new.setResolution(resolution);
    // 优化的迭代次数，我们这里设置为35次，即当迭代次数达到35次或者收敛到阈值时，停止优化。
    ndt_new.setMaximumIterations(max_iterations);

    // 将ROS下的传感器信息转换为PCL下的点云数据结构
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
    // 设置ndt目标点云，即地图点云
    ndt_new.setInputTarget(map_points_ptr);

    // create Thread
    // detach
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());          // 点云配准,后面的单位阵是初始位姿
    // output_cloud: 存储filtered_cloud_ptr经过配准后的点云(由于filtered_cloud_ptr被极大的降采样了,因此这个数据没什么用)

    // swap
    ndt_map_mtx_.lock();
    ndt_ = ndt_new;
    ndt_map_mtx_.unlock();
}

// 回调函数，处理滤波后的点云
void NdtLocalizer::callback_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& sensor_points_sensorTF_msg_ptr)
{
    // 记录开始执行时间
    const auto exe_start_time = std::chrono::system_clock::now();

    // mutex Map
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);

    const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
    const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

    // 点云数据：将传感器点云数据PointCLoud2转换为PCL的PointCloud点云数据结构
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);

    // 变换矩阵：get TF base to sensor，上述点云是在base_link坐标系下，下面将数据投射到激光雷达坐标系下
    geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
    get_transform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);

    // 变换矩阵：仿射变换，将上面的geometry_msgs消息变换矩阵转换为Eigen变换矩阵
    const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
    const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

    // 用上面的Eigen变换矩阵，将车辆基础坐标系(base_link)转换到传感器坐标系下的点云数据(ouster)下
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);


    //=====================================至此开始配准====================================//
    // set input point cloud  设置输入点云
    ndt_.setInputSource(sensor_points_baselinkTF_ptr);

    if (nullptr == ndt_.getInputTarget())
    {
        ROS_WARN_STREAM_THROTTLE(1, "No MAP!");     // 没有地图，即没有获得解析后的地图
        return;
    }

    // align 配准对齐
    Eigen::Matrix4f initial_pose_matrix;
    if (!init_pose_)
    {
        Eigen::Affine3d initial_pose_affine;            // 用仿射变换过渡
        tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
        initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

        // for the first time, we don't know the pre_trans, so just use the init_trans, which means, the deltas trans for the second time is 0
        pre_trans_ = initial_pose_matrix;
        init_pose_ = true;
    }
    else
    {
        // use predicted pose as init guess (currently we only impl linear model)
        initial_pose_matrix = pre_trans_ * delta_trans_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const auto align_start_time = std::chrono::system_clock::now();     // 开始配准时间
    key_value_stdmap_["state"] = "Aligning";
    ndt_.align(*output_cloud, initial_pose_matrix);                 // 配准对齐
    key_value_stdmap_["state"] = "Sleeping";
    const auto align_end_time   = std::chrono::system_clock::now();     // 结束配准时间
    // 计算配准对齐花费的时间
    const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;

    // 得到最后的变换矩阵
    const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
    Eigen::Affine3d result_pose_affine;                                 // 还是借助仿射变换进行过渡
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time   = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

    const float transform_probability = ndt_.getTransformationProbability();
    const int iteration_num = ndt_.getFinalNumIteration();

    bool is_converged = true;
    static size_t skipping_publish_num = 0;
    if (iteration_num >= ndt_.getMaximumIterations() + 2 || transform_probability < converged_param_transform_probability_)
    {
        is_converged = false;
        ++ skipping_publish_num;
        std::cout << "Not Converged" << std::endl;
    }
    else
    {
        skipping_publish_num = 0;
    }

    // calculate the delta tf from pre_trans to current_trans
    delta_trans_ = pre_trans_.inverse() * result_pose_matrix;

    // 得到△T的平移矩阵
    Eigen::Vector3f delta_translation = delta_trans_.block<3, 1>(0,3);
    std::cout << "delta x: " << delta_translation(0)
              << "y: " << delta_translation(1)
              << "z: " << delta_translation(2) << std::endl;
    // 得到△T的旋转矩阵
    Eigen::Matrix3f delta_rotation_matrix = delta_trans_.block<3, 3>(0,0);
    Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2,1,0);
    std::cout << "delta yaw: " << delta_euler(0)
              << "pitch: " << delta_euler(1)
              << "roll: "  << delta_euler(2) << std::endl;

    pre_trans_ = result_pose_matrix;

    // publish 要发布的位姿信息
    geometry_msgs::PoseStamped result_pose_stamped_msg;
    result_pose_stamped_msg.header.stamp = sensor_ros_time;
    result_pose_stamped_msg.header.frame_id = map_frame_;
    result_pose_stamped_msg.pose  = result_pose_msg;

    if (is_converged)
    {
        ndt_pose_pub_.publish(result_pose_stamped_msg);
    }

    // publish tf(map frame to base frame)
    publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);

    // publish aligned point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);

    // 发布执行时间
    std_msgs::Float32 exe_time_msg;
    exe_time_msg.data = exe_time;
    exe_time_pub_.publish(exe_time_msg);

    // 发布变换矩阵可能性
    std_msgs::Float32 transform_probability_msg;
    transform_probability_msg.data = transform_probability;
    transform_probability_pub_.publish(transform_probability_msg);

    // 发布迭代次数
    std_msgs::Float32 iteration_num_msg;
    iteration_num_msg.data = iteration_num;
    iteration_num_pub_.publish(iteration_num_msg);

    key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
    key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
    key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
    key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

    std::cout << "------------------------------------------" << std::endl;
    std::cout << "align_time: " << align_time << "ms" << std::endl;
    std::cout << "exe_time: "   << exe_time   << "ms" << std::endl;
    std::cout << "trans_prob: " << transform_probability << std::endl;
    std::cout << "iter_num: "   << iteration_num << std::endl;
    std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
}


void NdtLocalizer::init_params()
{
    private_nh_.getParam("base_frame", base_frame_);
    ROS_INFO("base_frame_id: %s", base_frame_.c_str());     // string 转换为 字符串

    double trans_epsilon = ndt_.getTransformationEpsilon(); // 搜索的最小变化量
    double step_size = ndt_.getStepSize();                  // 搜索的步长
    double resolution = ndt_.getResolution();               // 目标点云的ND体素尺寸，单位为米
    int max_iterations = ndt_.getMaximumIterations();       // NDT配准的迭代次数

    private_nh_.getParam("trans_epsilon", trans_epsilon);
    private_nh_.getParam("step_size", step_size);
    private_nh_.getParam("resolution", resolution);
    private_nh_.getParam("max_iterations", max_iterations);

    map_frame_ = "map";

    ndt_.setTransformationEpsilon(trans_epsilon);   // 搜索的最小变化量
    ndt_.setStepSize(step_size);                    // 搜索的步长
    ndt_.setResolution(resolution);                 // 目标点云的ND体素尺寸，单位为米
    ndt_.setMaximumIterations(max_iterations);      // 使用牛顿法优化的迭代步长

    ROS_INFO("trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d", trans_epsilon, step_size, resolution, max_iterations);

    private_nh_.getParam("converged_param_transform_probability", converged_param_transform_probability_);
}


// 得到第三个参数，两个坐标系之间的变换矩阵transform_stamped_ptr
bool NdtLocalizer::get_transform(const std::string& target_frame, const std::string& source_frame,
                                 const geometry_msgs::TransformStamped::Ptr& transform_stamped_ptr)
{
    // 如果是源点云与地图点云，同一坐标系
    if (target_frame == source_frame)
    {
        transform_stamped_ptr->header.stamp = ros::Time::now();     // 现在时间赋值到变换时间戳
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return true;
    }

    try {
        // 代表从buffer中查询最近的两个坐标系间的变换。第三个参数ros::Time(0)代表从buffer中获取“最新可获取时间”的变换。
        // note：每一个listener都有一个buffer储存来自来自不同tf2广播者的坐标系变换关系。这些变换进入缓冲区需要一段很小的时间，所以第三个参数不应该为ros::Time::now(),一般使用ros::Time(0)就很好。
        // https://blog.csdn.net/qq_41986495/article/details/85274114
        *transform_stamped_ptr = tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

        transform_stamped_ptr->header.stamp = ros::Time::now();
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;           // 平移
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;              // 旋转用四元数表示
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 0.0;
        return false;
    }

    return true;
}

bool NdtLocalizer::get_transform(const std::string& target_frame, const std::string& source_frame,
                                 const geometry_msgs::TransformStamped::Ptr& transform_stamped_ptr,
                                 const ros::Time& time_stamp)
{
    if (target_frame == source_frame) {
        transform_stamped_ptr->header.stamp = time_stamp;
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return true;
    }

    try {
        *transform_stamped_ptr =
                tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp);
    } catch (tf2::TransformException & ex) {
        ROS_WARN("%s", ex.what());
        ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

        transform_stamped_ptr->header.stamp = time_stamp;
        transform_stamped_ptr->header.frame_id = target_frame;
        transform_stamped_ptr->child_frame_id = source_frame;
        transform_stamped_ptr->transform.translation.x = 0.0;
        transform_stamped_ptr->transform.translation.y = 0.0;
        transform_stamped_ptr->transform.translation.z = 0.0;
        transform_stamped_ptr->transform.rotation.x = 0.0;
        transform_stamped_ptr->transform.rotation.y = 0.0;
        transform_stamped_ptr->transform.rotation.z = 0.0;
        transform_stamped_ptr->transform.rotation.w = 1.0;
        return false;
    }
    return true;
}


void NdtLocalizer::publish_tf(const std::string& frame_id, const std::string& child_frame_id,
                              const geometry_msgs::PoseStamped& pose_msg)
{
    // 待发布的变换矩阵
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.header.stamp = pose_msg.header.stamp;

    // 平移向量
    transform_stamped.transform.translation.x = pose_msg.pose.position.x;
    transform_stamped.transform.translation.x = pose_msg.pose.position.x;
    transform_stamped.transform.translation.x = pose_msg.pose.position.x;

    // 旋转四元数
    tf2::Quaternion tf_quaternion;          // 变换坐标四元数
    tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
    transform_stamped.transform.rotation.x = tf_quaternion.x();
    transform_stamped.transform.rotation.y = tf_quaternion.y();
    transform_stamped.transform.rotation.z = tf_quaternion.z();
    transform_stamped.transform.rotation.w = tf_quaternion.w();

    tf2_broadcaster_.sendTransform(transform_stamped);
}


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



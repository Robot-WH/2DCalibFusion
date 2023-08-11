#include <glog/logging.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "slam2d_node.h"

// 使用PCL中点的数据结构 pcl::PointXYZ
typedef pcl::PointXYZ PointT;
// 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
typedef pcl::PointCloud<PointT> PointCloudT;

// 构造函数
RosWrapper::RosWrapper() : private_node_("~"), lastGetMapUpdateIndex(-100) {
    ROS_INFO_STREAM("\033[1;32m----> Hector SLAM started.\033[0m");
    // 参数初始化
    InitParams();

    laser_scan_subscriber_ = node_handle_.subscribe(primeLaserTopic_name_, 
        p_scan_subscriber_queue_size_, &RosWrapper::scanCallback, this); // 雷达数据处理
    wheel_odom_subscriber_ = node_handle_.subscribe(wheelOdomTopic_name_, 
        100, &RosWrapper::wheelOdomCallback, this);   // 轮速计数据处理
    imu_subscriber_ = node_handle_.subscribe(imuTopic_name_, 
        100, &RosWrapper::imuCallback, this);     // IMU数据处理 

    if (p_pub_odometry_) {
        odometryPublisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom_Fusion", 50);
    }
    wheelOdomDeadReckoningPublisher_ = 
        node_handle_.advertise<nav_msgs::Odometry>("odom_wheelDeadReckoning", 50);
    undistorted_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud>(
        "undistorted_pointcloud", 10, this);
    dynamic_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud>(
        "dynamic_laser_points", 10, this);
    stable_pointcloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud>(
        "stable_laser_points", 10, this);
    localmap_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud>(
        "local_map", 10, this);

    tf_base_to_odom_ = new tf::TransformBroadcaster();
    tf_laser_to_base_ = new tf::TransformBroadcaster();
    tf_laserOdom_to_odom_ = new tf::TransformBroadcaster();  
    tf_global_pose_ = new tf::TransformBroadcaster();  
    // 外参初始化
    ext_prime_laser_to_odom_.setZero(); 
    //订阅融合的里程计结果 
    ::util::DataDispatcher::GetInstance().Subscribe("fusionOdom", 
                                                                                                    &RosWrapper::FusionOdomResultCallback, 
                                                                                                    this, 
                                                                                                    5,
                                                                                                    true);  // 高优先级
    
    ::util::DataDispatcher::GetInstance().Subscribe("global_pose", 
                                                                                                &RosWrapper::GlobalPoseCallback, 
                                                                                                this, 
                                                                                                5,
                                                                                                true);  // 高优先级

    // 订阅激光和odom的外参回调
    ::util::DataDispatcher::GetInstance().Subscribe("lidarOdomExt", 
                                                                                                    &RosWrapper::lidarOdomExtransicCallback, 
                                                                                                    this, 
                                                                                                    5);
    // 轮速解算的回调
    ::util::DataDispatcher::GetInstance().Subscribe("WheelDeadReckoning", 
                                                                                                    &RosWrapper::wheelOdomDeadReckoningCallback, 
                                                                                                    this, 
                                                                                                    5);
    // 去除畸变的回调
    ::util::DataDispatcher::GetInstance().Subscribe("undistorted_pointcloud", 
                                                                                                    &RosWrapper::undistortedPointcloudCallback, 
                                                                                                    this, 
                                                                                                    5,
                                                                                                    true); // 高优先级  
    // 动态点
    ::util::DataDispatcher::GetInstance().Subscribe("dynamic_pointcloud", 
                                                                                                    &RosWrapper::dynamicPointsCallback, 
                                                                                                    this, 
                                                                                                    5,
                                                                                                    true); // 高优先级  
    // 静态点
    ::util::DataDispatcher::GetInstance().Subscribe("stable_pointcloud", 
                                                                                                    &RosWrapper::stablePointsCallback, 
                                                                                                    this, 
                                                                                                    5,
                                                                                                    true); // 高优先级                         
    // 局部地图
    ::util::DataDispatcher::GetInstance().Subscribe("local_map", 
                                                                                                    &RosWrapper::localMapCallback, 
                                                                                                    this, 
                                                                                                    5);                                                             
    std::string config_path = RosUtils::RosReadParam<std::string>(node_handle_, "ConfigPath");

    // 构建前端估计器  
    estimator_ = new Estimator2D::FrontEndEstimator(config_path);
    estimator_->SetMapUpdateMinDistDiff(p_map_update_distance_threshold_); // 0.4 m 
    estimator_->SetMapUpdateMinAngleDiff(p_map_update_angle_threshold_);   // 0.78  45度
    // 地图发布器初始化
    int mapLevels = estimator_->GetMapLevels();
    // mapLevels = 1; // 这里设置成只发布最高精度的地图，如果有其他需求，如进行路径规划等等需要多层地图时，注释本行。

    std::string mapTopic_ = "map";
    for (int i = 0; i < mapLevels; ++i) {
        mapPubContainer.push_back(MapPublisherContainer());
        std::string mapTopicStr(mapTopic_);

        if (i != 0) {
            mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
        }

        std::string mapMetaTopicStr(mapTopicStr);
        mapMetaTopicStr.append("_metadata");

        MapPublisherContainer& tmp = mapPubContainer[i];
        tmp.mapPublisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
        tmp.mapMetadataPublisher_ = node_handle_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

        setMapInfo(tmp.map_, estimator_->GetGridMap(i)); // 设置地图服务

        if (i == 0) {
            mapPubContainer[i].mapMetadataPublisher_.publish(tmp.map_.map.info);
        }
    }
    // 新建一个线程用来发布地图
    map_publish_thread_ = 
        new boost::thread(boost::bind(&RosWrapper::publishMapLoop, this, p_map_pub_period_));
    map_to_odom_.setIdentity();

}

RosWrapper::~RosWrapper() {
    delete estimator_;

    if (tf_base_to_odom_)
        delete tf_base_to_odom_;

    if (map_publish_thread_)
        delete map_publish_thread_;
}

// ros的参数初始化
void RosWrapper::InitParams() {
    private_node_.param("pub_map_baselink_tf", pub_map_to_baselink_tf_, true);
    private_node_.param("pub_map_odom_tf", p_pub_map_odom_transform_, false);
    private_node_.param("pub_odometry_topic", p_pub_odometry_, true);

    private_node_.param("scan_topic", primeLaserTopic_name_, std::string("laser_scan"));
    private_node_.param("wheel_topic", wheelOdomTopic_name_, std::string("wheel_odom"));
    private_node_.param("imu_topic", imuTopic_name_, std::string("imu"));
    private_node_.param("prime_laser_frame", primeLaserFrame_name_, std::string("primeLaser_link"));
    std::cout << "prime_laser_frame: " << primeLaserFrame_name_ << std::endl;
    private_node_.param("laser_odom_frame", laserOdomFrame_name_, std::string("laserOdom"));
    std::cout << "laser_odom_frame: " << laserOdomFrame_name_ << std::endl;
    private_node_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);
    private_node_.param("use_max_scan_range", p_use_max_scan_range_, 20.0);

    private_node_.param("map_frame", mapFrame_name_, std::string("map"));
    private_node_.param("odom_frame", odomFrame_name_, std::string("odom"));
    private_node_.param("base_frame", baseFrame_name_, std::string("base"));

    private_node_.param("output_timing", p_timing_output_, false);
    private_node_.param("map_pub_period", p_map_pub_period_, 2.0);

    private_node_.param("map_resolution", p_map_resolution_, 0.05);
    private_node_.param("map_size", p_map_size_, 2048);
    private_node_.param("map_start_x", p_map_start_x_, 0.5);
    private_node_.param("map_start_y", p_map_start_y_, 0.5);
    private_node_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

    private_node_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
    private_node_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.78);

    double tmp = 0.0;
    private_node_.param("laser_min_dist", tmp, 0.2);
    laser_min_dist_ = static_cast<float>(tmp);
    std::cout << "laser_min_dist: " << laser_min_dist_ << std::endl;

    private_node_.param("laser_max_dist", tmp, 30.0);
    laser_max_dist_ = static_cast<float>(tmp);
    std::cout << "laser_max_dist: " << laser_max_dist_ << std::endl;

    private_node_.param("laser_z_min_value", tmp, -1.0);
    p_laser_z_min_value_ = static_cast<float>(tmp);

    private_node_.param("laser_z_max_value", tmp, 1.0);
    p_laser_z_max_value_ = static_cast<float>(tmp);

    private_node_.param("laser_imaging_reversed", laser_info_.laser_imaging_reversed_, false);
    if (laser_info_.laser_imaging_reversed_) {
        std::cout << msa2d::color::GREEN << "激光雷达成像反向！" << msa2d::color::RESET << std::endl; 
    }
}

// 这个回调处理只输出位姿x，y，theta的里程计信息，这里会将odom位姿信息转化为速度和角速度
void RosWrapper::wheelOdomCallback(const nav_msgs::Odometry& odom_msg) {
}

void RosWrapper::imuCallback(const sensor_msgs::Imu& imu_msg) {
    static int i = 0; 
    static double last_time = -1;
    // 舍弃第一个数据  因为第一个数据有时会错乱  
    if (last_time < 0) {
        last_time = 0;
        return;  
    }

    double curr_time = imu_msg.header.stamp.toSec();

    if (curr_time <= last_time) {
        // std::cout << common::RED << "------------------------IMU时间戳混乱!--------------------------" << std::endl; 
        return;  
    }

    // i++;
    //std::cout << common::RED << "imu index: " << i << common::RESET << std::endl;

    msa2d::sensor::ImuData imu; 
    imu.time_stamp_ = curr_time;
    imu.acc_ = Eigen::Vector3d{imu_msg.linear_acceleration.x, 
                                                            imu_msg.linear_acceleration.y,
                                                            imu_msg.linear_acceleration.z};
    imu.gyro_ = Eigen::Vector3d{imu_msg.angular_velocity.x,
                                                                            imu_msg.angular_velocity.y,
                                                                            imu_msg.angular_velocity.z};      
    imu.orientation_ = Eigen::Quaterniond{imu_msg.orientation.w,
                                                                                imu_msg.orientation.x,    
                                                                                imu_msg.orientation.y,
                                                                                imu_msg.orientation.z};
    // float bias = 0;
    // if (i > 3000 && i < 6000) {
    //     bias = 0.1; 
    // } else if (i > 6000) {
    //     bias = 0.2; 
    // }   

    // imu.gyro_[2] -= bias;              

    // static int f = 0;
    // if (f <= 0) {
    //     util::SaveDataCsv<double>("", "true_bias.csv", {bias, curr_time}, {"bias", "time"});   
    //     f = 10;
    // }
    // f--;   

    estimator_->InputImu(imu);
}

/**
 * 激光数据处理回调函数，将ros数据格式转换为算法中的格式，并转换成地图尺度，交由slamProcessor处理。
 * 算法中所有的计算都是在地图尺度下进行。  
 */
void RosWrapper::scanCallback(const sensor_msgs::LaserScan &scan) {
    static double last_time = -1;
    // 舍弃第一个数据  因为第一个数据有时会错乱  
    if (last_time < 0) {
        last_time = 0;
        return;  
    }
    if (scan.header.stamp.toSec() <= last_time) {
        // std::cout << common::RED << "------------------------laser时间戳混乱!--------------------------" << std::endl; 
        return;  
    }
    last_time = scan.header.stamp.toSec();  

    ros::WallTime startTime = ros::WallTime::now();
    // 提取激光信息
    static bool scan_init = false;
    if (!scan_init) {
        // 提取激光的信息 
        laser_info_.angle_increment_ = scan.angle_increment;
        double angle_range = scan.angle_max - scan.angle_min; 
        if (angle_range < 6.28) {
            return;  
        }
        laser_info_.laser_num_ = angle_range / laser_info_.angle_increment_; 
        laser_info_.time_increment_ = scan.time_increment;
        laser_info_.laser_period_ = laser_info_.time_increment_ * laser_info_.laser_num_;  
        laser_info_.index_cos_.clear();
        laser_info_.index_sin_.clear();
        double angle = scan.angle_min; 

        for (unsigned int i = 0; i < scan.ranges.size(); i++) {
            laser_info_.index_cos_.push_back(cos(angle));
            laser_info_.index_sin_.push_back(sin(angle));
            angle += scan.angle_increment;
        }
        // 靠近起始和终止位置附近处的点不要  (+-25) 
        laser_info_.valid_ind_lower_ = 0.4363 / scan.angle_increment;
        laser_info_.valid_ind_upper_ = scan.ranges.size() - 1 - laser_info_.valid_ind_lower_; 

        scan_init = true; 
        std::cout << "激光点数量: " << laser_info_.laser_num_ << std::endl
        << "激光角度分辨率: " << laser_info_.angle_increment_ * 180 / 3.1415926<< std::endl 
        << "激光周期时间: " << laser_info_.laser_period_ << std::endl
        << "scan.angle_min: " << scan.angle_min << std::endl
        << "scan.angle_increment: " << scan.angle_increment << std::endl;
    } 
    // start_time_ = std::chrono::steady_clock::now();
    // 将 scan 转换成 点云格式
    msa2d::sensor::LaserScan::ptr laser_ptr(new msa2d::sensor::LaserScan());
    // 将雷达数据的点云格式 更改成 hector 内部的数据格式
    if (rosPointCloudToDataContainer(scan, *laser_ptr)) {
        // 进入扫描匹配与地图更新
        estimator_->InputLaser(laser_ptr);
    }
        
    // end_time_ = std::chrono::steady_clock::now();
    // time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    // std::cout << "数据转换与扫描匹配用时: " << time_used_.count() << " 秒。" << std::endl;

    // if (p_timing_output_) {
    //     ros::WallDuration duration = ros::WallTime::now() - startTime;
    //     ROS_INFO("Estimator2D Iter took: %f milliseconds", duration.toSec() * 1000.0f);
    // }
}

/**
 * @brief: 接收融合算法计算的结果 
 */
void RosWrapper::FusionOdomResultCallback(const msa2d::TimedPose2d& data) {
    // std::cout << common::YELLOW << "FusionOdom ----------------------------" 
    // << common:: RESET << std::endl;
    if (data.time_stamp_ < 0) return;  

    geometry_msgs::PoseWithCovarianceStamped pose_info = 
        RosUtils::GetPoseWithCovarianceStamped(data.pose_.vec(), estimator_->GetLastScanMatchCovariance(), 
                                                                                                ros::Time(data.time_stamp_), 
                                                                                                // ros::Time::now(),
                                                                                                odomFrame_name_); 
    // 发布tf
    tf::Transform primeLaser_to_odom_tf = RosUtils::GetTFTransform(ext_prime_laser_to_odom_); 
    tf_base_to_odom_->sendTransform(tf::StampedTransform(RosUtils::GetTFTransform(pose_info.pose.pose), 
                                                ros::Time(data.time_stamp_), 
                                                // ros::Time::now(),
                                                odomFrame_name_, 
                                                baseFrame_name_));
    tf_laserOdom_to_odom_->sendTransform(tf::StampedTransform(primeLaser_to_odom_tf, 
                                                ros::Time(data.time_stamp_), 
                                                // ros::Time::now(),
                                                odomFrame_name_, 
                                                laserOdomFrame_name_));                                            
    tf_laser_to_base_->sendTransform(tf::StampedTransform(primeLaser_to_odom_tf, 
                                                ros::Time(data.time_stamp_), 
                                                // ros::Time::now(),
                                                baseFrame_name_, 
                                                primeLaserFrame_name_));

    // tf_global_pose_->sendTransform(tf::StampedTransform(RosUtils::GetTFTransform(global_pose_msg_), 
    //                                 ros::Time(data.time_stamp_), 
    //                                 // ros::Time::now(),
    //                                 laserOdomFrame_name_, 
    //                                 "global_pose"));
    // 发布 odom topic
    if (p_pub_odometry_) {
        nav_msgs::Odometry tmp;
        tmp.pose = pose_info.pose;
        tmp.header = pose_info.header;
        // tmp.header.stamp = ros::Time::now();
        tmp.header.frame_id = odomFrame_name_;       // 基准坐标
        // tmp.child_frame_id = baseFrame_name_;
        odometryPublisher_.publish(tmp);
    }
}

void RosWrapper::GlobalPoseCallback(const msa2d::Pose2d& pose) {
        global_pose_msg_.position.x = pose.x();
        global_pose_msg_.position.y = pose.y();

        global_pose_msg_.orientation.w = cos(pose.yaw() * 0.5f);
        global_pose_msg_.orientation.z = sin(pose.yaw() * 0.5f);
}

void RosWrapper::lidarOdomExtransicCallback(const Eigen::Matrix<float, 6, 1>& ext) {
    ext_prime_laser_to_odom_ = ext;  
    std::cout << "Recieve lidar-odom extrinsic : " << ext_prime_laser_to_odom_.transpose() <<std::endl;
} 

// 原始wheel 航迹推算，调试用
void RosWrapper::wheelOdomDeadReckoningCallback(const msa2d::TimedPose2d& pose) {
    geometry_msgs::PoseWithCovarianceStamped pose_info = 
        RosUtils::GetPoseWithCovarianceStamped(Eigen::Vector3f{pose.pose_.x(), pose.pose_.y(), pose.pose_.yaw()}, 
                                                                                                Eigen::Matrix3f::Zero(), 
                                                                                                ros::Time(pose.time_stamp_), odomFrame_name_); 
    nav_msgs::Odometry tmp;
    tmp.pose = pose_info.pose;
    tmp.header.stamp = ros::Time(pose.time_stamp_);
    tmp.header.frame_id = odomFrame_name_;       // 基准坐标
    // tmp.child_frame_id = baseFrame_name_;
    wheelOdomDeadReckoningPublisher_.publish(tmp);
}

// 接收到去畸变的点云
void RosWrapper::undistortedPointcloudCallback(const msa2d::sensor::LaserScan& data) {
    // std::cout << msa2d::color::GREEN << "send undistortedPointcloud ----------------------------" 
    // << msa2d::color::RESET << std::endl;
    sensor_msgs::PointCloud pointcloud_msg;
    uint16_t size = data.pointcloud_.size(); 
    pointcloud_msg.points.reserve(size);
    for (uint16_t i = 0; i < size; ++i) {
        geometry_msgs::Point32 point; 
        point.x = data.pointcloud_[i].pos_[0];
        point.y = data.pointcloud_[i].pos_[1];
        point.z = 0;
        pointcloud_msg.points.push_back(point);
    }
    pointcloud_msg.header.stamp = ros::Time(data.end_time_); 
    // pointcloud_msg.header.stamp = ros::Time(data.start_time_); 
    // pointcloud_msg.header.stamp = ros::Time::now(); 
    // pointcloud_msg.header.frame_id = "global_pose";
    // pointcloud_msg.header.frame_id = "base_link";
    pointcloud_msg.header.frame_id = primeLaserFrame_name_;
    undistorted_pointcloud_publisher_.publish(pointcloud_msg);
}

void RosWrapper::dynamicPointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data) {
    sensor_msgs::PointCloud pointcloud_msg;
    uint16_t size = data.first.size(); 
    pointcloud_msg.points.reserve(size);

    for (uint16_t i = 0; i < size; ++i) {
        geometry_msgs::Point32 point; 
        point.x = data.first[i][0];
        point.y = data.first[i][1];
        point.z = 0;
        pointcloud_msg.points.push_back(point);
    }
    pointcloud_msg.header.stamp = ros::Time(data.second); 
    // pointcloud_msg.header.stamp = ros::Time::now(); 
    // pointcloud_msg.header.frame_id = primeLaserFrame_name_;
    pointcloud_msg.header.frame_id = laserOdomFrame_name_;
    dynamic_pointcloud_publisher_.publish(pointcloud_msg);
}

void RosWrapper::stablePointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data) {
    sensor_msgs::PointCloud pointcloud_msg;
    uint16_t size = data.first.size(); 
    pointcloud_msg.points.reserve(size);
    if (size == 0) {
        std::cout << msa2d::color::RED << "stablePoints empty !!! " << msa2d::color::RESET << std::endl;
        return;  
    }

    for (uint16_t i = 0; i < size; ++i) {
        geometry_msgs::Point32 point; 
        point.x = data.first[i][0];
        point.y = data.first[i][1];
        point.z = 0;
        pointcloud_msg.points.push_back(point);
    }
    pointcloud_msg.header.stamp = ros::Time(data.second); 
    // pointcloud_msg.header.stamp = ros::Time::now(); 
    // pointcloud_msg.header.frame_id = primeLaserFrame_name_;  
    pointcloud_msg.header.frame_id = laserOdomFrame_name_;
    stable_pointcloud_publisher_.publish(pointcloud_msg);
}

void RosWrapper::undeterminedPointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data) {

}

void RosWrapper::localMapCallback(const std::vector<Eigen::Vector2f>& data) {
    sensor_msgs::PointCloud pointcloud_msg;
    uint16_t size = data.size(); 
    pointcloud_msg.points.reserve(size);

    for (uint16_t i = 0; i < size; ++i) {
        geometry_msgs::Point32 point; 
        point.x = data[i][0];
        point.y = data[i][1];
        point.z = 0;
        pointcloud_msg.points.push_back(point);
    }
    pointcloud_msg.header.stamp = ros::Time::now(); 
    pointcloud_msg.header.frame_id = laserOdomFrame_name_;
    localmap_publisher_.publish(pointcloud_msg);
}

/**
 * @brief 将点云数据转换成Hector中雷达数据的格式
 * @param[out] laser 按照激光的角度从小到大排列
 */ 
// bool RosWrapper::rosPointCloudToDataContainer(const sensor_msgs::LaserScan& scan_msg, 
//                                                                                                                         msa2d::sensor::LaserPointCloud& laser) {
//     size_t size = scan_msg.ranges.size();
//     size_t last_index = size - 1;  
//     laser.pointcloud_.resize(size);
//     // std::cout << std::setprecision(15) << "laser.start_time_:" << laser.start_time_ <<std::endl;

//     for (uint16_t i = 0; i < size; i++) {
//         // 距离滤波 
//         if (!std::isfinite(scan_msg.ranges[i]) ||
//             scan_msg.ranges[i] < laser_min_dist_ ||
//             scan_msg.ranges[i] > laser_max_dist_) {
//             continue;
//         }
//         // 如果雷达点云的排列和旋转方向相反
//         if (laser_info_.laser_imaging_reversed_) {
//             laser.pointcloud_[last_index - i].pos_ = {scan_msg.ranges[i] * laser_info_.index_cos_[i],   // x
//                                                                                     scan_msg.ranges[i] * laser_info_.index_sin_[i]};   // y 
//             laser.pointcloud_[last_index - i].range_ = scan_msg.ranges[i];
//             laser.pointcloud_[last_index - i].index_ = last_index - i;
//             laser.pointcloud_[last_index - i].rel_time_ = (last_index - i) * laser_info_.time_increment_;                                                                          
//         } else {
//             laser.pointcloud_[i].pos_ = {scan_msg.ranges[i] * laser_info_.index_cos_[i],   // x
//                                                                         scan_msg.ranges[i] * laser_info_.index_sin_[i]};   // y 
//             laser.pointcloud_[i].range_ = scan_msg.ranges[i];
//             laser.pointcloud_[i].index_ = i;
//             laser.pointcloud_[i].rel_time_ = i * laser_info_.time_increment_;          
//         }
//     }

//     // if (laser_info_.laser_imaging_reversed_) {
//     //     laser.start_time_ = scan_msg.header.stamp.toSec() - laser.pointcloud_.back().rel_time_; 
//     //     laser.end_time_ = scan_msg.header.stamp.toSec();
//     // } else {
//     //     laser.start_time_ = scan_msg.header.stamp.toSec();
//     //     laser.end_time_ = laser.start_time_ + laser.pointcloud_.back().rel_time_; 
//     // }

//     laser.start_time_ = scan_msg.header.stamp.toSec();
//     laser.end_time_ = laser.start_time_ + laser.pointcloud_.back().rel_time_; 
    
//     laser.scan_period_ = laser_info_.laser_period_;  
//     return true;
// }


bool RosWrapper::rosPointCloudToDataContainer(const sensor_msgs::LaserScan& scan_msg, 
                                                                                                           msa2d::sensor::LaserScan& laser) {
    size_t size = scan_msg.ranges.size();
    size_t last_index = size - 1; 
    laser.pointcloud_.reserve(size);
    // std::cout << "size: " << size << std::endl;
    // std::cout << std::setprecision(15) << "laser.start_time_:" << laser.start_time_ <<std::endl;
    if (!laser_info_.laser_imaging_reversed_) {
        laser.start_time_ = scan_msg.header.stamp.toSec();

        for (uint16_t i = 0; i < size; i++) {
            // 距离滤波 
            if (!std::isfinite(scan_msg.ranges[i]) ||
                scan_msg.ranges[i] < laser_min_dist_ ||
                scan_msg.ranges[i] > laser_max_dist_) {
                continue;
            }
            msa2d::sensor::LaserPoint point;   
            point.pos_ = {scan_msg.ranges[i] * laser_info_.index_cos_[i],   // x
                                        scan_msg.ranges[i] * laser_info_.index_sin_[i]};   // y 
            point.range_ = scan_msg.ranges[i];
            point.index_ = i;
            point.rel_time_ = i * laser_info_.time_increment_; 
            laser.pointcloud_.push_back(std::move(point)); 
        }
        laser.end_time_ = laser.start_time_ + laser.pointcloud_.back().rel_time_;  
    } else {
        for (int i = last_index; i >= 0; --i) {
            // if (i > laser_info_.valid_ind_upper_) continue;
            // if (i < 2 * laser_info_.valid_ind_lower_) continue;
            // 距离滤波 
            if (!std::isfinite(scan_msg.ranges[i]) ||
                scan_msg.ranges[i] < laser_min_dist_ ||
                scan_msg.ranges[i] > laser_max_dist_) {
                continue;
            }
            msa2d::sensor::LaserPoint point;   
            point.pos_ = {scan_msg.ranges[i] * laser_info_.index_cos_[i],   // x
                                        scan_msg.ranges[i] * laser_info_.index_sin_[i]};   // y 
            point.range_ = scan_msg.ranges[i];
            point.index_ = last_index - i;
            point.rel_time_ = (last_index - i) * laser_info_.time_increment_; 
            laser.pointcloud_.push_back(std::move(point)); 
        }
        // laser.end_time_ = laser.start_time_;  
        // laser.start_time_ = scan_msg.header.stamp.toSec() - laser.pointcloud_.back().rel_time_;
        laser.start_time_ = scan_msg.header.stamp.toSec();
        laser.end_time_ = laser.start_time_ + laser.pointcloud_.back().rel_time_;  
    }
    laser.scan_period_ = laser_info_.laser_period_;  
    return true;
}

// 对ROS地图进行数据初始化与分配内存
void RosWrapper::setMapInfo(nav_msgs::GetMap::Response& map_, 
        const msa2d::map::OccGridMapBase* gridMap) {
    Eigen::Vector2f mapOrigin(gridMap->getGridMapBase().PosMapToWorldf(Eigen::Vector2f::Zero()));   // Map原点的world坐标

    map_.map.info.origin.position.x = mapOrigin.x();
    map_.map.info.origin.position.y = mapOrigin.y();
    map_.map.info.origin.orientation.w = 1.0;
    map_.map.info.resolution = gridMap->getGridMapBase().getGridResolution();
    map_.map.info.width = gridMap->getGridMapBase().getGridSizeX();
    map_.map.info.height = gridMap->getGridMapBase().getGridSizeY();

    map_.map.header.frame_id = laserOdomFrame_name_;    // 设置map的参考坐标系  
    // 分配内存空间
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

// 发布地图的线程
void RosWrapper::publishMapLoop(double map_pub_period) {
    ros::Rate r(1.0 / map_pub_period);
    while (ros::ok()) {
        ros::Time mapTime(ros::Time::now());
        //publishMap(mapPubContainer[2],estimator_->getGridMap(2), mapTime);
        //publishMap(mapPubContainer[1],estimator_->getGridMap(1), mapTime);
        // publishMap(mapPubContainer[2], estimator_->GetGridMap(2), mapTime, estimator_->GetMapMutex(2));
        publishMap(mapPubContainer[0], estimator_->GetGridMap(0), mapTime, estimator_->GetMapMutex(0));
        r.sleep();
    }
}

// 发布ROS地图
void RosWrapper::publishMap(MapPublisherContainer& mapPublisher,
                                  const msa2d::map::OccGridMapBase* gridMap,
                                  ros::Time timestamp, std::mutex* mapMutex) {
    nav_msgs::GetMap::Response& map_(mapPublisher.map_);
    //only update map if it changed
    if (lastGetMapUpdateIndex != gridMap->getGridMapBase().getUpdateIndex()) {
        int sizeX = gridMap->getGridMapBase().getGridSizeX();
        int sizeY = gridMap->getGridMapBase().getGridSizeY();
        int size = sizeX * sizeY;
        std::vector<int8_t> &data = map_.map.data;
        //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
        memset(&data[0], -1, sizeof(int8_t) * size);     

        mapMutex->lock();

        for (int i = 0; i < size; ++i) {
            // 空闲：0， 占据：100，未知：-1
            if (gridMap->isFree(i)) {
                data[i] = 0;
            } else if (gridMap->isOccupied(i)) {
                data[i] = 100;
            }
        }

        lastGetMapUpdateIndex = gridMap->getGridMapBase().getUpdateIndex();
        Eigen::Vector2f mapOrigin(gridMap->getGridMapBase().PosMapToWorldf(Eigen::Vector2f::Zero()));   // Map原点的world坐标

        map_.map.info.origin.position.x = mapOrigin.x();
        map_.map.info.origin.position.y = mapOrigin.y();

        mapMutex->unlock();
    }

    map_.map.header.stamp = timestamp;
    mapPublisher.mapPublisher_.publish(map_.map);
}

// void imuCallback(const int &t) {
//     std::cout << "callback imuCallback, sensor_msgs::Imu" << std::endl;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "fusion estimator");
    std::cout << "laser-imu-wheel fusion estimator, running in ARM..." << std::endl;
    
    char* home_dir = getenv("HOME");
    std::string home_dir_str(home_dir);
    std::string log_dir = home_dir_str + "/log";
    FLAGS_log_dir = log_dir;
    // FLAGS_logbufsecs = 5;   // 没隔5s写一次磁盘
    // FLAGS_max_log_size = 10; // 设置单个日志文件大小上限为 10MB
    google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::GLOG_INFO, "/home/lwh/log/info.log");
    // google::SetLogDestination(google::GLOG_WARNING, "/path/to/warning.log");
    // google::SetLogDestination(google::GLOG_ERROR, "/path/to/error.log");
    RosWrapper frontend_node;
    ros::spin();
    return (0);
}
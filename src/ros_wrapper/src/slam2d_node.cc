/*
 * Copyright 2021 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "slam2d_node.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/PointCloud2.h"

using namespace hectorslam; 

// 使用PCL中点的数据结构 pcl::PointXYZ
typedef pcl::PointXYZ PointT;
// 使用PCL中点云的数据结构 pcl::PointCloud<pcl::PointXYZ>
typedef pcl::PointCloud<PointT> PointCloudT;

// 构造函数
HectorMappingRos::HectorMappingRos() : private_node_("~"), 
                                                                                        lastGetMapUpdateIndex(-100) {
    ROS_INFO_STREAM("\033[1;32m----> Hector SLAM started.\033[0m");
    // 参数初始化
    InitParams();

    laser_scan_subscriber_ = node_handle_.subscribe(primeLaserTopic_name_, 
        p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this); // 雷达数据处理
    wheel_odom_subscriber_ = node_handle_.subscribe(wheelOdomTopic_name_, 
        100, &HectorMappingRos::wheelOdomCallback, this);
    imu_subscriber_ = node_handle_.subscribe(imuTopic_name_, 
        100, &HectorMappingRos::imuCallback, this);

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
    // 外参初始化
    ext_prime_laser_to_odom_.setZero(); 
    //订阅融合的里程计结果 
    util::DataDispatcher::GetInstance().Subscribe("fusionOdom", 
                                                                                                    &HectorMappingRos::FusionOdomResultCallback, 
                                                                                                    this, 
                                                                                                    5,
                                                                                                    true);  // 高优先级
    // 订阅激光和odom的外参回调
    util::DataDispatcher::GetInstance().Subscribe("lidarOdomExt", 
                                                                                                    &HectorMappingRos::lidarOdomExtransicCallback, 
                                                                                                    this, 
                                                                                                    5);
    // 轮速解算的回调
    util::DataDispatcher::GetInstance().Subscribe("WheelDeadReckoning", 
                                                                                                    &HectorMappingRos::wheelOdomDeadReckoningCallback, 
                                                                                                    this, 
                                                                                                    5);
    // 去除畸变的回调
    util::DataDispatcher::GetInstance().Subscribe("undistorted_pointcloud", 
                                                                                                    &HectorMappingRos::undistortedPointcloudCallback, 
                                                                                                    this, 
                                                                                                    5,
                                                                                                    true); // 高优先级  
    // 动态点
    util::DataDispatcher::GetInstance().Subscribe("dynamic_pointcloud", 
                                                                                                    &HectorMappingRos::dynamicPointsCallback, 
                                                                                                    this, 
                                                                                                    5,
                                                                                                    true); // 高优先级  
    // 静态点
    util::DataDispatcher::GetInstance().Subscribe("stable_pointcloud", 
                                                                                                    &HectorMappingRos::stablePointsCallback, 
                                                                                                    this, 
                                                                                                    5,
                                                                                                    true); // 高优先级                         
    // 局部地图
    util::DataDispatcher::GetInstance().Subscribe("local_map", 
                                                                                                    &HectorMappingRos::localMapCallback, 
                                                                                                    this, 
                                                                                                    5);                                                             
    std::string config_path = RosUtils::RosReadParam<std::string>(node_handle_, "ConfigPath");

    estimator_ = new FrontEndEstimator(config_path);

    estimator_->SetUpdateFactorFree(p_update_factor_free_);                // 0.4
    estimator_->SetUpdateFactorOccupied(p_update_factor_occupied_);        // 0.9
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
        new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));
    map_to_odom_.setIdentity();

    // util::Subscriber sub(&HectorMappingRos::imuCallback, this); 
    // sensor_msgs::Imu imu;
    // sub.Call(imu); 
}

HectorMappingRos::~HectorMappingRos() {
    delete estimator_;

    if (tf_base_to_odom_)
        delete tf_base_to_odom_;

    if (map_publish_thread_)
        delete map_publish_thread_;
}

// ros的参数初始化
void HectorMappingRos::InitParams() {
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

    private_node_.param("update_factor_free", p_update_factor_free_, 0.4);
    private_node_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

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
}

// 这个回调处理只输出位姿x，y，theta的里程计信息，这里会将odom位姿信息转化为速度和角速度
void HectorMappingRos::wheelOdomCallback(const nav_msgs::Odometry& odom_msg) {
    static double last_time = -1;
    // 舍弃第一个数据  因为第一个数据有时会错乱  
    if (last_time < 0) {
        last_time = 0;
        return;  
    }
    WheelOdom odom;
    double curr_time = odom_msg.header.stamp.toSec();

    if (curr_time <= last_time) {
        // std::cout << common::RED << "------------------------轮速计时间戳混乱!--------------------------" << std::endl; 
        return;  
    }
    static float last_x = 0;
    static float last_y = 0;
    static float last_yaw = 0;

    odom.pose_.SetTransform(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y);
    odom.pose_.SetRotation(odom_msg.pose.pose.orientation.x,
                                                            odom_msg.pose.pose.orientation.y,
                                                            odom_msg.pose.pose.orientation.z,
                                                            odom_msg.pose.pose.orientation.w); 

    if (last_time != 0) {
        double dt = curr_time - last_time; 
        odom.v_x_ = std::sqrt(std::pow((odom_msg.pose.pose.position.x - last_x) / dt, 2) + 
                                    std::pow((odom_msg.pose.pose.position.y - last_y) / dt, 2));
        odom.v_y_ = 0;
        float delta_angle = odom.pose_.yaw() - last_yaw; 
        if (delta_angle > M_PI) {
            delta_angle = delta_angle - 2 * M_PI; 
        }
        if (delta_angle < -M_PI) {
            delta_angle = 2 * M_PI + delta_angle; 
        }
        odom.omega_yaw_ = delta_angle / dt;
        odom.time_stamp_ = (curr_time + last_time) / 2;  
        // std::cout << common::YELLOW << "odom.time_stamp_: " 
        // << odom.time_stamp_ << common::RESET << std::endl;
        estimator_->InputWheelOdom(odom);
    }
    last_x = odom_msg.pose.pose.position.x;
    last_y = odom_msg.pose.pose.position.y;
    last_yaw = odom.pose_.yaw(); 
    last_time = curr_time;  
}

void HectorMappingRos::imuCallback(const sensor_msgs::Imu& imu_msg) {
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

    ImuData imu; 
    imu.time_stamp_ = curr_time;
    imu.acc_ = Eigen::Vector3d{imu_msg.linear_acceleration.x, 
                                                            imu_msg.linear_acceleration.y,
                                                            imu_msg.linear_acceleration.z};
    imu.angular_v_ = Eigen::Vector3d{imu_msg.angular_velocity.x,
                                                                            imu_msg.angular_velocity.y,
                                                                            imu_msg.angular_velocity.z};      
    // float bias = 0;
    // if (i > 3000 && i < 6000) {
    //     bias = 0.1; 
    // } else if (i > 6000) {
    //     bias = 0.2; 
    // }   

    // imu.angular_v_[2] -= bias;              

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
void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan &scan) {
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

        for (unsigned int i = 0; i < scan.ranges.size(); i++) {
            double angle = scan.angle_min + i * scan.angle_increment;
            laser_info_.index_cos_.push_back(cos(angle));
            laser_info_.index_sin_.push_back(sin(angle));
        }
        scan_init = true; 
        std::cout << "激光点数量: " << laser_info_.laser_num_ << std::endl
        << "激光角度分辨率: " << laser_info_.angle_increment_ * 180 / 3.1415926<< std::endl 
        << "激光周期时间: " << laser_info_.laser_period_<< std::endl;
    } 
    // start_time_ = std::chrono::steady_clock::now();
    // 将 scan 转换成 点云格式
    LaserPointCloud::ptr laser_ptr(new LaserPointCloud());
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
    //     ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec() * 1000.0f);
    // }
}

/**
 * @brief: 接收融合算法计算的结果 
 */
void HectorMappingRos::FusionOdomResultCallback(const TimedPose2d& data) {
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

void HectorMappingRos::lidarOdomExtransicCallback(const Eigen::Matrix<float, 6, 1>& ext) {
    ext_prime_laser_to_odom_ = ext;  
    std::cout << "Recieve lidar-odom extrinsic : " << ext_prime_laser_to_odom_.transpose() <<std::endl;
} 

// 原始wheel 航迹推算，调试用
void HectorMappingRos::wheelOdomDeadReckoningCallback(const TimedPose2d& pose) {
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
void HectorMappingRos::undistortedPointcloudCallback(const LaserPointCloud::Ptr& data) {
    // std::cout << common::GREEN << "send undistortedPointcloud ----------------------------" 
    // << common::RESET << std::endl;
    sensor_msgs::PointCloud pointcloud_msg;
    uint16_t size = data->pointcloud_.size(); 
    pointcloud_msg.points.reserve(size);

    for (uint16_t i = 0; i < size; ++i) {
        geometry_msgs::Point32 point; 
        point.x = data->pointcloud_[i].pos_[0];
        point.y = data->pointcloud_[i].pos_[1];
        point.z = 0;
        pointcloud_msg.points.push_back(point);
    }
    pointcloud_msg.header.stamp = ros::Time(data->end_time_); 
    // pointcloud_msg.header.stamp = ros::Time::now(); 
    pointcloud_msg.header.frame_id = primeLaserFrame_name_;
    undistorted_pointcloud_publisher_.publish(pointcloud_msg);
}

void HectorMappingRos::dynamicPointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data) {
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

void HectorMappingRos::stablePointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data) {
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
    stable_pointcloud_publisher_.publish(pointcloud_msg);
}

void HectorMappingRos::undeterminedPointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data) {

}

void HectorMappingRos::localMapCallback(const std::vector<Eigen::Vector2f>& data) {
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
bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::LaserScan& scan_msg, 
                                                                                                                        LaserPointCloud& laser) {
    size_t size = scan_msg.ranges.size();
    laser.pointcloud_.reserve(size);
    laser.start_time_ = scan_msg.header.stamp.toSec();
    // std::cout << std::setprecision(15) << "laser.start_time_:" << laser.start_time_ <<std::endl;

    for (uint16_t i = 0; i < size; i++) {
        // 距离滤波 
        if (!std::isfinite(scan_msg.ranges[i]) ||
            scan_msg.ranges[i] < laser_min_dist_ ||
            scan_msg.ranges[i] > laser_max_dist_) {
            continue;
        }
        LaserPoint point;   
        point.pos_ = {scan_msg.ranges[i] * laser_info_.index_cos_[i],   // x
                                    scan_msg.ranges[i] * laser_info_.index_sin_[i]};   // y 
        point.range_ = scan_msg.ranges[i];
        point.index_ = i;
        point.rel_time_ = i * laser_info_.time_increment_; 
        laser.pointcloud_.push_back(std::move(point)); 
    }
    laser.end_time_ = laser.start_time_ + laser.pointcloud_.back().rel_time_;  
    laser.scan_period_ = laser_info_.laser_period_;  
    return true;
}

// 对ROS地图进行数据初始化与分配内存
void HectorMappingRos::setMapInfo(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap) {
    Eigen::Vector2f mapOrigin(gridMap.getWorldCoords(Eigen::Vector2f::Zero()));   // Map原点的world坐标
    mapOrigin.array() -= gridMap.getCellLength() * 0.5f;

    map_.map.info.origin.position.x = mapOrigin.x();
    map_.map.info.origin.position.y = mapOrigin.y();
    map_.map.info.origin.orientation.w = 1.0;
    map_.map.info.resolution = gridMap.getCellLength();
    map_.map.info.width = gridMap.getSizeX();
    map_.map.info.height = gridMap.getSizeY();

    map_.map.header.frame_id = laserOdomFrame_name_;    // 设置map的参考坐标系  
    // 分配内存空间
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

// 发布地图的线程
void HectorMappingRos::publishMapLoop(double map_pub_period) {
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
void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher,
                                  const hectorslam::GridMap& gridMap,
                                  ros::Time timestamp, std::mutex* mapMutex) {
    nav_msgs::GetMap::Response& map_(mapPublisher.map_);
    //only update map if it changed
    if (lastGetMapUpdateIndex != gridMap.getUpdateIndex()) {
        int sizeX = gridMap.getSizeX();
        int sizeY = gridMap.getSizeY();
        int size = sizeX * sizeY;
        std::vector<int8_t> &data = map_.map.data;
        //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
        memset(&data[0], -1, sizeof(int8_t) * size);     

        mapMutex->lock();

        for (int i = 0; i < size; ++i) {
            // 空闲：0， 占据：100，未知：-1
            if (gridMap.isFree(i)) {
                data[i] = 0;
            } else if (gridMap.isOccupied(i)) {
                data[i] = 100;
            }
        }

        lastGetMapUpdateIndex = gridMap.getUpdateIndex();
        Eigen::Vector2f mapOrigin(gridMap.getWorldCoords(Eigen::Vector2f::Zero()));   // Map原点的world坐标
        mapOrigin.array() -= gridMap.getCellLength() * 0.5f;

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
    ros::init(argc, argv, "lesson4_hector_slam");
    HectorMappingRos hector_slam;
    ros::spin();
    return (0);
}
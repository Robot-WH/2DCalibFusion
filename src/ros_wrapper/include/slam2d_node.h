
#pragma once 

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/GetMap.h>
#include <laser_geometry/laser_geometry.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <boost/thread.hpp>
#include <chrono>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include "ros_util.h"
// #include "FrontEnd2D/Map/OccGridMapBase.h"
#include "FrontEnd2D/Estimator/estimator.hpp"
#include "msa2d/Sensor/LaserPointContainer.h"
#include "msa2d/Sensor/SensorData.hpp"
#include "msa2d/Sensor/point_cloud.hpp"
#include "FrontEnd2D/util/DataDispatcher.hpp"


class MapPublisherContainer {
public:
    ros::Publisher mapPublisher_;
    ros::Publisher mapMetadataPublisher_;
    nav_msgs::GetMap::Response map_;
    ros::ServiceServer dynamicMapServiceServer_;
};

class RosWrapper {
public:
    RosWrapper();
    ~RosWrapper();
    void scanRosCallback(const sensor_msgs::LaserScan &scan);
    void wheelOdomRosCallback(const nav_msgs::Odometry &odom);
    void imuRosCallback(const sensor_msgs::Imu &imu);
    void resetRosCallback(const std_msgs::Bool& flag);
    void FusionOdomResultCallback(const msa2d::TimedPose2d& data);
    void GlobalPoseCallback(const msa2d::Pose2d& pose);
    void lidarOdomExtransicCallback(const Eigen::Matrix<float, 6, 1>& ext);
    void wheelOdomDeadReckoningCallback(const msa2d::TimedPose2d& pose);
    void undistortedPointcloudCallback(const msa2d::sensor::LaserScan& data);
    void dynamicPointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data);
    void stablePointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data);
    void undeterminedPointsCallback(const std::pair<std::vector<Eigen::Vector2f>, double>& data);
    void localMapCallback(const std::vector<Eigen::Vector2f>& data);
    void publishMapLoop(double p_map_pub_period_);
    void ReadCamera();   

private:
    void InitParams();
    void setMapInfo(nav_msgs::GetMap::Response &map_, const msa2d::map::OccGridMapBase* gridMap);
    void publishMap(MapPublisherContainer &map_, const msa2d::map::OccGridMapBase* gridMap, 
        ros::Time timestamp, std::mutex* mapMutex);
    void publishTFLoop(double pub_period);
    bool rosPointCloudToDataContainer(const sensor_msgs::LaserScan &scan_msg, 
                                                                                msa2d::sensor::LaserScan& pointCloud);

    struct LaserInfo {
        std::vector<double> index_cos_; // 保存下来雷达各个角度的cos值
        std::vector<double> index_sin_; // 保存下来雷达各个角度的sin值
        float angle_increment_;  // 激光点角度增量  
        double time_increment_;  
        float laser_period_;   // 激光一帧的时间
        int laser_num_;   // 激光点的数量
        int valid_ind_lower_;   
        int valid_ind_upper_;
        bool laser_imaging_reversed_ = false;  // 激光雷达点云的排列和实际激光雷达旋转的方向相反
    }laser_info_;

    ros::NodeHandle node_handle_;  // ros中的句柄
    ros::NodeHandle private_node_; // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_;
    ros::Subscriber wheel_odom_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber reset_subscriber_;
    ros::Publisher odometryPublisher_;
    ros::Publisher wheelOdomDeadReckoningPublisher_;
    ros::Publisher undistorted_pointcloud_publisher_;    // 去畸变的点云
    ros::Publisher dynamic_pointcloud_publisher_;    // 动态点云
    ros::Publisher stable_pointcloud_publisher_;    // 稳定点云
    ros::Publisher localmap_publisher_;
    ros::Publisher camera_publisher_;

    std::vector<MapPublisherContainer> mapPubContainer;

    tf::TransformListener tf_;
    tf::TransformBroadcaster *tf_base_to_odom_;
    tf::TransformBroadcaster *tf_laser_to_base_;
    tf::TransformBroadcaster *tf_laserOdom_to_odom_;
    tf::TransformBroadcaster * tf_global_pose_;
    tf::Transform map_to_odom_;
    tf::StampedTransform laserTransform_;
    geometry_msgs::Pose global_pose_msg_;
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud laser_point_cloud_; // 点云格式的雷达数据

    boost::thread* map_publish_thread_;

    Estimator2D::FrontEndEstimator* estimator_;
    msa2d::sensor::LaserPointContainer laserScanContainer;

    int lastGetMapUpdateIndex;

    std::string baseFrame_name_; // base_frame
    std::string mapFrame_name_;  // map的frame
    std::string odomFrame_name_; // 里程计odom的frame
    std::string primeLaserFrame_name_; // 主激光的frame
    std::string laserOdomFrame_name_; // 主激光的frame
    std::string primeLaserTopic_name_; // 激光scan的topic
    std::string wheelOdomTopic_name_;  
    std::string imuTopic_name_;  

    // Parameters related to publishing the scanmatcher pose directly via tf
    bool pub_map_to_baselink_tf_;
    bool p_pub_map_odom_transform_;
    bool p_pub_odometry_;
    int p_scan_subscriber_queue_size_;
    double p_use_max_scan_range_;

    // 地图更新参数
    double p_update_factor_free_;
    double p_update_factor_occupied_;
    double p_map_update_distance_threshold_;
    double p_map_update_angle_threshold_;

    // map parameters --- resolution / size / init pose / map levels 
    double p_map_resolution_;
    int p_map_size_;
    double p_map_start_x_;
    double p_map_start_y_;
    int p_map_multi_res_levels_;
    double p_map_pub_period_;

    bool p_timing_output_;

    // laser data filter
    float laser_min_dist_;
    float laser_max_dist_;
    float p_laser_z_min_value_;
    float p_laser_z_max_value_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;
    // 外参
    Eigen::Matrix<float, 6, 1> ext_prime_laser_to_odom_;
};


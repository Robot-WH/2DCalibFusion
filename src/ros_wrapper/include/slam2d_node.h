
#pragma once 

#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"
#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/PointCloud2.h>
// pcl_ros
#include <pcl_ros/point_cloud.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "../../FrontEnd2D/include/Map/GridMap.h"
#include "../../FrontEnd2D/include/Estimator/estimator.hpp"
#include "../../FrontEnd2D/include/Sensor/LaserPointContainer.h"
#include "../../FrontEnd2D/include/Sensor/SensorData.hpp"
#include "../../FrontEnd2D/include/util/DataDispatcher.hpp"
#include "ros_util.h"

#include <boost/thread.hpp>
#include <chrono>

class MapPublisherContainer {
public:
    ros::Publisher mapPublisher_;
    ros::Publisher mapMetadataPublisher_;
    nav_msgs::GetMap::Response map_;
    ros::ServiceServer dynamicMapServiceServer_;
};

class HectorMappingRos {
public:
    HectorMappingRos();
    ~HectorMappingRos();
    void scanCallback(const sensor_msgs::LaserScan &scan);
    void wheelOdomCallback(const nav_msgs::Odometry &odom);
    void imuCallback(const sensor_msgs::Imu &imu);
    void FusionOdomResultCallback(const TimedPose2d& data);
    void lidarOdomExtransicCallback(const Eigen::Matrix<float, 6, 1>& ext);
    void wheelOdomDeadReckoningCallback(const TimedPose2d& pose);
    void undistortedPointcloudCallback(const hectorslam::LaserPointCloud::Ptr& data);
    void dynamicPointsCallback(const std::pair<std::vector<hectorslam::LaserPoint>, double>& data);
    void stablePointsCallback(const std::pair<std::vector<hectorslam::LaserPoint>, double>& data);
    void publishMapLoop(double p_map_pub_period_);

private:
    void InitParams();
    void setMapInfo(nav_msgs::GetMap::Response &map_, const hectorslam::GridMap &gridMap);
    void publishMap(MapPublisherContainer &map_, const hectorslam::GridMap &gridMap, 
        ros::Time timestamp, std::mutex* mapMutex);
    void publishTFLoop(double pub_period);
    bool rosPointCloudToDataContainer(const sensor_msgs::LaserScan &scan_msg, 
                                                                                hectorslam::LaserPointCloud& pointCloud);

    struct LaserInfo {
        std::vector<double> index_cos_; // ?????????????????????????????????cos???
        std::vector<double> index_sin_; // ?????????????????????????????????sin???
        float angle_increment_;  // ?????????????????????  
        double time_increment_;  
        float laser_period_;   // ?????????????????????
        int laser_num_;   // ??????????????????
    }laser_info_;

    ros::NodeHandle node_handle_;  // ros????????????
    ros::NodeHandle private_node_; // ros??????????????????
    ros::Subscriber laser_scan_subscriber_;
    ros::Subscriber wheel_odom_subscriber_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher odometryPublisher_;
    ros::Publisher wheelOdomDeadReckoningPublisher_;
    ros::Publisher undistorted_pointcloud_publisher_;    // ??????????????????
    ros::Publisher dynamic_pointcloud_publisher_;    // ????????????
    ros::Publisher stable_pointcloud_publisher_;    // ????????????

    std::vector<MapPublisherContainer> mapPubContainer;

    tf::TransformListener tf_;
    tf::TransformBroadcaster *tf_base_to_odom_;
    tf::TransformBroadcaster *tf_laser_to_base_;
    tf::TransformBroadcaster *tf_laserOdom_to_odom_;
    tf::Transform map_to_odom_;
    tf::StampedTransform laserTransform_;
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud laser_point_cloud_; // ???????????????????????????

    boost::thread *map_publish_thread_;

    hectorslam::FrontEndEstimator *estimator_;
    hectorslam::LaserPointContainer laserScanContainer;

    int lastGetMapUpdateIndex;

    // Hector??????????????????

    std::string baseFrame_name_; // base_frame
    std::string mapFrame_name_;  // map???frame
    std::string odomFrame_name_; // ?????????odom???frame
    std::string primeLaserFrame_name_; // ????????????frame
    std::string laserOdomFrame_name_; // ????????????frame
    std::string primeLaserTopic_name_; // ??????scan???topic
    std::string wheelOdomTopic_name_;  
    std::string imuTopic_name_;  

    // Parameters related to publishing the scanmatcher pose directly via tf
    bool pub_map_to_baselink_tf_;
    bool p_pub_map_odom_transform_;
    bool p_pub_odometry_;
    int p_scan_subscriber_queue_size_;
    double p_use_max_scan_range_;

    // ??????????????????
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
    // ??????
    Eigen::Matrix<float, 6, 1> ext_prime_laser_to_odom_;
};


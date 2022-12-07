
#pragma once 

#include <Eigen/Core>
#include "../utility.hpp"

namespace Slam2D {
struct ImuData {
    double time_stamp_ = 0;
    Eigen::Vector3d acc_;
    Eigen::Vector3d angular_v_;  
};

struct Odom {
    double time_stamp_ = 0;
    double v_x_ = 0, v_y_ = 0;  // 线速度
    double omega_yaw_ = 0;  // 角速度  
    double x_ = 0, y_ = 0;
    Eigen::Quaterniond orientation_; 
};

struct LaserPoint {
    double time_stamp_;  
    Eigen::Vector2d pos_;  // x, y 
    double range_; 
    uint16_t index_;  
};

template<typename _PointT>
struct LaserPointCloud {
    double time_stamp_; 
    std::vector<double> range_;
    std::vector<double> index_;
    pcl::PointCloud<_PointT> pointcloud_; 
};
}
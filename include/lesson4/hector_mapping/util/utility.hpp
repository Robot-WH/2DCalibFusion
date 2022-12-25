
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <queue>
#include <cmath>
#include <cassert>
#include <cstring>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>   // find_package() 要包含这个  
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

#include "../Type/Pose2d.hpp"
#include "../Sensor/SensorData.hpp"
namespace hectorslam {
namespace util {

/**
 * @brief 正规化角度
 * TODO static函数 public private权限不同有什么区别? 
 **/
static double NormalizationAngle(double &angle) {
    if (angle > M_PI)
        angle -= 2 * M_PI;
    else if (angle < -M_PI)
        angle += 2 * M_PI;

    return angle;
}

/**
 * @brief (x, y , theta) 转换为变换矩阵  
 **/
static Eigen::Matrix3d Vector2Transform(Eigen::Vector3d const& vec) {
    Eigen::Matrix3d T;
    T << cos(vec(2)), -sin(vec(2)), vec(0),
        sin(vec(2)), cos(vec(2)), vec(1),
        0, 0, 1;
    return T;
}

/**
 * @brief 对某一个点进行转换．
 * @return (x,y)
 **/
static Eigen::Vector2d TransformPoint(Eigen::Vector2d const& pt, Eigen::Matrix3d const& T) {
    Eigen::Vector3d tmp_pt(pt(0), pt(1), 1);     // x,y 
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0), tmp_pt(1));  
}

/**
 * @brief: 线性插值的模板函数
 * @details: 部分类型的插值需要进行特化  
 */    
template<typename _T>
static _T LinearInterpolate(const _T& front_value, const _T& back_value, double const& front_time, 
        double const& back_time, double const& time) {
    _T interpolation_v;
    float front_coeff = (back_time - time) / (back_time - front_time);
    float back_coeff = (time - front_time) / (back_time - front_time);
    interpolation_v = front_coeff * front_value + back_coeff * back_value;  
    return interpolation_v;  
} 

/**
 * @brief: 线性插值的模板函数
 * @details: 部分类型的插值需要进行特化  
 */    
template<>
Pose2d LinearInterpolate<Pose2d>(const Pose2d& front_value, const Pose2d& back_value, 
                                                                          double const& front_time, double const& back_time, 
                                                                          double const& time) {
    float front_coeff = (back_time - time) / (back_time - front_time);
    float back_coeff = (time - front_time) / (back_time - front_time);
    double x = front_coeff * front_value.x() + back_coeff * back_value.x();  
    double y = front_coeff * front_value.y() + back_coeff * back_value.y();  
    Eigen::Quaternionf orientation = front_value.orientation().slerp(back_coeff, back_value.orientation());
    return Pose2d(x, y, orientation);  
} 

/**
 * @brief: 线性插值的模板函数
 * @details: 轮速数据的特化  
 */    
template<>
WheelOdom LinearInterpolate<WheelOdom>(const WheelOdom& front_value, const WheelOdom& back_value, 
                                                                          double const& front_time, double const& back_time, 
                                                                          double const& time) {
    float front_coeff = (back_time - time) / (back_time - front_time);
    float back_coeff = (time - front_time) / (back_time - front_time);
    WheelOdom interpolate_data; 
    interpolate_data.v_x_ = front_coeff * front_value.v_x_ + back_coeff * back_value.v_x_;  
    interpolate_data.v_y_ = front_coeff * front_value.v_y_ + back_coeff * back_value.v_y_;  
    interpolate_data.omega_yaw_ = front_coeff * front_value.omega_yaw_ + back_coeff * back_value.omega_yaw_; 
    return interpolate_data;  
} 

/**
 * @brief: 线性插值的模板函数
 * @details: IMU数据的特化  
 */    
template<>
ImuData LinearInterpolate<ImuData>(const ImuData& front_value, const ImuData& back_value, 
                                                                          double const& front_time, double const& back_time, 
                                                                          double const& time) {
    float front_coeff = (back_time - time) / (back_time - front_time);
    float back_coeff = (time - front_time) / (back_time - front_time);
    ImuData interpolate_data; 
    interpolate_data.acc_ = front_coeff * front_value.acc_ + back_coeff * back_value.acc_;  
    interpolate_data.angular_v_ = front_coeff * front_value.angular_v_ + back_coeff * back_value.angular_v_;  
    return interpolate_data;  
} 

}
}

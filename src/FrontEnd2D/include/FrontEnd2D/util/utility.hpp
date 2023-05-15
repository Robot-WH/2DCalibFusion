
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <queue>
#include <cmath>
#include <cassert>
#include <cstring>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <iomanip>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <image_transport/image_transport.h>   // find_package() 要包含这个  
// #include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include "fileManager.hpp"
#include "msa2d/common/Pose2d.hpp"
#include "msa2d/Sensor/SensorData.hpp"

namespace Estimator2D {
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
msa2d::Pose2d LinearInterpolate<msa2d::Pose2d>(const msa2d::Pose2d& front_value, const msa2d::Pose2d& back_value, 
                                                                          double const& front_time, double const& back_time, 
                                                                          double const& time) {
    float front_coeff = (back_time - time) / (back_time - front_time);
    float back_coeff = (time - front_time) / (back_time - front_time);
    double x = front_coeff * front_value.x() + back_coeff * back_value.x();  
    double y = front_coeff * front_value.y() + back_coeff * back_value.y();  
    Eigen::Quaternionf orientation = front_value.orientation().slerp(back_coeff, back_value.orientation());
    return msa2d::Pose2d(x, y, orientation);  
} 

/**
 * @brief: 线性插值的模板函数
 * @details: 轮速数据的特化  
 */    
template<>
msa2d::sensor::WheelOdom LinearInterpolate<msa2d::sensor::WheelOdom>(
                                                                        const msa2d::sensor::WheelOdom& front_value, 
                                                                        const msa2d::sensor::WheelOdom& back_value, 
                                                                          double const& front_time, double const& back_time, 
                                                                          double const& time) {
    float front_coeff = (back_time - time) / (back_time - front_time);
    float back_coeff = (time - front_time) / (back_time - front_time);
    msa2d::sensor::WheelOdom interpolate_data; 
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
msa2d::sensor::ImuData LinearInterpolate<msa2d::sensor::ImuData>(
                                                                        const msa2d::sensor::ImuData& front_value, 
                                                                        const msa2d::sensor::ImuData& back_value, 
                                                                        double const& front_time, double const& back_time, 
                                                                        double const& time) {
    float front_coeff = (back_time - time) / (back_time - front_time);
    float back_coeff = (time - front_time) / (back_time - front_time);
    msa2d::sensor::ImuData interpolate_data; 
    interpolate_data.acc_ = front_coeff * front_value.acc_ + back_coeff * back_value.acc_;  
    interpolate_data.angular_v_ = front_coeff * front_value.angular_v_ + back_coeff * back_value.angular_v_;  
    return interpolate_data;  
} 

const std::string WORK_SPACE_PATH = "/home/lwh/code/2D_slam/src/2D-Calib-Fusion/data/"; 

/**
 * @brief 保存数据
 * 
 */
template<class _T>
static bool SaveDataCsv(std::string const& Directory_path, std::string const& file_name, 
                        std::vector<_T> const& data, std::vector<std::string> const& labels) {
    std::ofstream out;

    if (!boost::filesystem::exists(WORK_SPACE_PATH + Directory_path + file_name)) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + Directory_path))
            return false;
        if (!FileManager::CreateFile(out, WORK_SPACE_PATH + Directory_path + file_name))
            return false;
        // 写标签
        for(auto const& label:labels) {
           out << label << ',';  
        }
        out << std::endl;
    } else {
        out.open((WORK_SPACE_PATH + Directory_path + file_name).c_str(), std::ios::app);
    }

    for(auto const& n : data) {
       out << std::setprecision(15) << n << ',';
    }

    out << std::endl;
    return true;
}

template<class _T>
static bool SaveDataCsv(std::string const& Directory_path, std::string const& file_path, 
                        _T const& data, std::vector<std::string> const& labels) {
    static std::ofstream out;
    static bool is_file_created = false;

    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + Directory_path))
            return false;
        if (!FileManager::CreateFile(out, WORK_SPACE_PATH + file_path))
            return false;
        is_file_created = true;
        // 写标签
        for(auto const& label:labels) {
           out << label << ',';  
        }
        out << std::endl;
    }

    out << data << ',';

    out << std::endl;
    return true;
}

}
}

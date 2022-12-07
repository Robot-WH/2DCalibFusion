/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-22 20:41:14
 * @Description: 
 * @Others: 
 */

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

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "pose_2d.hpp"

class Utility {
  public:
    /**
     * @brief: 计算curr_pose 到 next_pose 的增量 
     * @details: delta_pose = curr_pose ^ -1 * next_pose
     * @return 结果 
     */    
    static Pose2d CalcIncremPose(nav_msgs::Odometry const& curr_odom, 
            nav_msgs::Odometry const& next_odom) {
        float x = curr_odom.pose.pose.position.x;
        float y = curr_odom.pose.pose.position.y;
        Eigen::Quaterniond orientation_q;  
        tf::Quaternion orientation;
        double roll, pitch, yaw;  
        tf::quaternionMsgToTF(curr_odom.pose.pose.orientation, orientation);
        orientation_q.w() = orientation.w();  
        orientation_q.x() = orientation.x();  
        orientation_q.y() = orientation.y();  
        orientation_q.z() = orientation.z();  
        orientation_q.normalize();  
        Pose2d curr_pose(x, y, orientation_q);
        // std::cout<<"laser odom curr x:  "<<x<<"y: "<<y<<", yaw: "
        // <<yaw<<", orientation: "<<orientation_q.coeffs().transpose()<<std::endl;
        x = next_odom.pose.pose.position.x;
        y = next_odom.pose.pose.position.y;
        tf::quaternionMsgToTF(next_odom.pose.pose.orientation, orientation);
        orientation_q.w() = orientation.w();  
        orientation_q.x() = orientation.x();  
        orientation_q.y() = orientation.y();  
        orientation_q.z() = orientation.z();  
        orientation_q.normalize();  
        Pose2d next_pose(x, y, orientation_q);
        // std::cout<<"laser odom next x:  "<<x<<"y: "<<y<<", yaw: "
        // <<yaw<<", orientation: "<<orientation_q.coeffs().transpose()<<std::endl;
        return curr_pose.inv() * next_pose;     // curr->next  
    }

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
     * @brief: 插值的模板函数
     * @details: 部分类型的插值需要进行特化  
     */    
    template<typename _T>
    static _T Interpolate(_T front_value, _T back_value, double const& front_time, 
            double const& back_time, double const& time) {
        _T interpolation_v;
        float front_coeff = (back_time - time) / (back_time - front_time);
        float back_coeff = (time - front_time) / (back_time - front_time);
        interpolation_v = front_coeff * front_value + back_coeff * back_value;  
        return interpolation_v;  
    } 
};

/**
 * @brief: 插值的模板函数
 * @details: 部分类型的插值需要进行特化  
 */    
template<>
Pose2d Utility::Interpolate<Pose2d>(Pose2d front_value, Pose2d back_value, double const& front_time, 
        double const& back_time, double const& time) {
    float front_coeff = (back_time - time) / (back_time - front_time);
    float back_coeff = (time - front_time) / (back_time - front_time);
    double x = front_coeff * front_value.GetX() + back_coeff * back_value.GetX();  
    double y = front_coeff * front_value.GetY() + back_coeff * back_value.GetY();  
    Eigen::Quaterniond orientation = front_value.GetOrientation().slerp(back_coeff, back_value.GetOrientation());
    return Pose2d(x, y, orientation);  
} 


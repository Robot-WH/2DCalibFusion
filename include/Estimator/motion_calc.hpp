/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-23 01:39:02
 * @Description: 
 * @Others: 
 */

#pragma once 

#include "pose_2d.hpp"
#include "sensor/sensor_data_type.hpp"

namespace Slam2D {

    static Odom OdomInterpolation(Odom const& front_odom, Odom const& back_odom, 
                                                                    double const& curr_time) {
        Odom interpolation_odom;
        float front_coeff = back_odom.time_stamp_ - curr_time
                                                / back_odom.time_stamp_ - front_odom.time_stamp_;
        float back_coeff = curr_time - front_odom.time_stamp_
                                                / back_odom.time_stamp_ - front_odom.time_stamp_;
        interpolation_odom.time_stamp_ = curr_time;
        interpolation_odom.v_x_ = front_coeff * front_odom.v_x_ + back_coeff * back_odom.v_x_;  
        interpolation_odom.v_y_ = front_coeff * front_odom.v_y_ + back_coeff * back_odom.v_y_;  
        interpolation_odom.x_ = front_coeff * front_odom.x_ + back_coeff * back_odom.x_;  
        interpolation_odom.y_ = front_coeff * front_odom.y_ + back_coeff * back_odom.y_;  
        interpolation_odom.omega_yaw_ = front_coeff * front_odom.omega_yaw_ 
                                                                                    + back_coeff * back_odom.omega_yaw_;                                                                     
        interpolation_odom.orientation_ = front_odom.orientation_.slerp(back_coeff, back_odom.orientation_);
        return interpolation_odom;  
    }

    /**
     * @brief: 使用odom信息恢复start_time-end_time之间的运动
     * @param start_time 
     * @param end_time
     * @return {*}
     */    
    static bool GetMotionFromOdom(double const& start_time, double const& end_time, 
            std::deque<Odom> const& wheel_odom_data, Pose2d &result) {
        if (start_time >= end_time) {
            ROS_WARN("GetMotionFromOdom() ERROR, start_time >= end_time !");
            return false;  
        }
        if (start_time < wheel_odom_data.front().time_stamp_ 
                || end_time > wheel_odom_data.back().time_stamp_ ) {
            ROS_WARN("GetMotionFromOdom() - start_time, end_time not in corret range !");
             return false;  
        }

        Pose2d start_pose, end_pose;  
        Odom last_odom;
        double target_time = start_time; 
        
        for (std::deque<Odom>::const_iterator it = wheel_odom_data.begin(); 
                it != wheel_odom_data.end(); it++) {
            if (it->time_stamp_ > target_time) {
                // 插值出当前的pose
                Odom odom_interpolate = OdomInterpolation(last_odom, *it, target_time);
                if (target_time == end_time) {
                    end_pose = Pose2d(odom_interpolate.x_, odom_interpolate.y_, odom_interpolate.orientation_); 
                    // std::cout<<"odom end_pose x: "<<odom_interpolate.x_<<" , y: "
                    // <<odom_interpolate.y_<<", theta: "<<eulerAngle[0]
                    // <<", orientation_: "<<odom_interpolate.orientation_.coeffs()<<std::endl;
                    break;  
                }
                start_pose = Pose2d(odom_interpolate.x_, odom_interpolate.y_, odom_interpolate.orientation_); 
                // std::cout<<"odom start_pose x: "<<odom_interpolate.x_
                //                 <<" , y: "<<odom_interpolate.y_
                //                 <<", theta: "<<eulerAngle[0]
                //                 <<", orientation_: "<<odom_interpolate.orientation_.coeffs()<<std::endl;
                target_time = end_time;
            }
            last_odom = *it;  
        }
        result = start_pose.inv() * end_pose;  
        return true;  
    }

    /**
     * @brief: 使用线速度和角速度信息恢复start_time-end_time之间的运动
     * @param start_time 
     * @param end_time
     * @return {*}
     */    
    static void GetMotionFromVelocityAndRotate(double const& start_time, double const& end_time)
    {
    }

}

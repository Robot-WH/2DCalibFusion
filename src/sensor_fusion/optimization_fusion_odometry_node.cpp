/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-14 23:33:35
 * @Description:  基于优化的融合激光里程计  
 * @Others:  使用ceres的2D滑动窗口优化
 *                          融合 wheel、imu、激光雷达  
 * 
 *  step:  1、初始化 - 外参初始化 ， imu bias 初始化， 轮速内参 
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include "calibration/2d_handeye_calibration.hpp"
#include "sensor/sensor_data_type.hpp"
#include "Estimator/motion_calc.hpp"
#include "utility.hpp"

using namespace Slam2D;  

constexpr bool PARAM_ESTIMATE = true;    // 外参估计 
constexpr bool EXTRINSIC_OPT = false;          // 外参在线优化 

class FusionOdometryNode
{
    public:

        FusionOdometryNode()
        {
            //node_handle = ros::NodeHandle("~");
            // \033[1;32m，\033[0m 终端显示成绿色
            ROS_INFO_STREAM("\033[1;32m----> optimization fusion node started.\033[0m");

            imu_subscriber_ = node_handle_.subscribe(
                "imu", 2000, &FusionOdometryNode::ImuCallback, this, ros::TransportHints().tcpNoDelay());
            
            wheel_odom_subscriber_ = node_handle_.subscribe(
                "odom", 2000, &FusionOdometryNode::OdomCallback, this, ros::TransportHints().tcpNoDelay());
            
            laser_odom_subscriber_ = node_handle_.subscribe(
                "odom_hector", 2000, &FusionOdometryNode::LaserOdomCallback, 
                this, ros::TransportHints().tcpNoDelay());

            odom_pub = node_handle_.advertise<nav_msgs::Odometry>(
                "raw_odom", 1, this);
        }

        void Estimator()
        {
            while(1)
            {
                {
                    std::unique_lock<std::mutex> laser_odom_lock(laser_odom_mut_);
                    if (laser_odom_msg_.size() >= 2)
                    {
                        // 提取当前处理的激光里程计数据
                        nav_msgs::Odometry curr_laser_odom = laser_odom_msg_.front();  
                        laser_odom_msg_.pop_front();
                        nav_msgs::Odometry next_laser_odom = laser_odom_msg_.front();  
                        laser_odom_lock.unlock();  
                        // 获取包围激光里程计数据的 odom 和 imu 数据  
                        // 轮速的数据需要包围laser
                        std::unique_lock<std::mutex> wheel_odom_lock(wheel_odom_mut_);
                        // 清除过早的数据 
                        while (!wheel_odom_msg_.empty())
                        {
                            if (wheel_odom_msg_.front().header.stamp.toSec() 
                                    > curr_laser_odom.header.stamp.toSec() - 0.03) {
                                break;
                            }
                            wheel_odom_msg_.pop_front();  
                        }
                        // 判断数据范围
                        if (wheel_odom_msg_.front().header.stamp.toSec() > curr_laser_odom.header.stamp.toSec())
                        {
                            wheel_odom_lock.unlock(); 
                            std::this_thread::sleep_for(std::chrono::milliseconds(20));
                            continue;  
                        }
                        // 如果odom的数据 暂时还没有覆盖laser  那么等待一会儿再处理 
                        if (wheel_odom_msg_.back().header.stamp.toSec() < next_laser_odom.header.stamp.toSec())
                        {
                            laser_odom_lock.lock(); 
                            laser_odom_msg_.push_front(curr_laser_odom); 
                            laser_odom_lock.unlock(); 
                            std::this_thread::sleep_for(std::chrono::milliseconds(20));
                            continue;  
                        }

                        nav_msgs::Odometry begin_wheel_odom;
                        bool is_begin = true; 

                        for (uint16_t i = 0; i < wheel_odom_msg_.size(); i++)
                        {
                            if (wheel_odom_msg_[i].header.stamp.toSec() <= curr_laser_odom.header.stamp.toSec())
                            {
                                begin_wheel_odom = wheel_odom_msg_[i]; 
                                continue;
                            }
                            
                            Odom wheel_odom;  
                            if (is_begin)
                            {
                                wheel_odom.time_stamp_ = begin_wheel_odom.header.stamp.toSec();

                                wheel_odom.v_x_ = begin_wheel_odom.twist.twist.linear.x;
                                wheel_odom.v_y_ = begin_wheel_odom.twist.twist.linear.y;
                                wheel_odom.omega_yaw_ = begin_wheel_odom.twist.twist.angular.z;

                                wheel_odom.x_ = begin_wheel_odom.pose.pose.position.x;
                                wheel_odom.y_ = begin_wheel_odom.pose.pose.position.y;  
                                wheel_odom.orientation_.w() = begin_wheel_odom.pose.pose.orientation.w;
                                wheel_odom.orientation_.x() = begin_wheel_odom.pose.pose.orientation.x;
                                wheel_odom.orientation_.y() = begin_wheel_odom.pose.pose.orientation.y;
                                wheel_odom.orientation_.z() = begin_wheel_odom.pose.pose.orientation.z;
                                wheel_odom_data_.push_back(wheel_odom);  
                                is_begin = false;
                            }

                            nav_msgs::Odometry curr_wheel_odom = wheel_odom_msg_[i]; 
                            wheel_odom.time_stamp_ = curr_wheel_odom.header.stamp.toSec();

                            wheel_odom.v_x_ = curr_wheel_odom.twist.twist.linear.x;
                            wheel_odom.v_y_ = curr_wheel_odom.twist.twist.linear.y;
                            wheel_odom.omega_yaw_ = curr_wheel_odom.twist.twist.angular.z;

                            wheel_odom.x_ = curr_wheel_odom.pose.pose.position.x;
                            wheel_odom.y_ = curr_wheel_odom.pose.pose.position.y;  
                            wheel_odom.orientation_.w() = begin_wheel_odom.pose.pose.orientation.w;
                            wheel_odom.orientation_.x() = begin_wheel_odom.pose.pose.orientation.x;
                            wheel_odom.orientation_.y() = begin_wheel_odom.pose.pose.orientation.y;
                            wheel_odom.orientation_.z() = begin_wheel_odom.pose.pose.orientation.z;
                            wheel_odom_data_.push_back(wheel_odom);  

                            if (wheel_odom.time_stamp_ > next_laser_odom.header.stamp.toSec())
                            {
                                break;
                            }   
                        }
                        wheel_odom_lock.unlock();  

                        // imu_mut_.lock();
                        // // 获取IMU数据 
                        // // 清除过早的数据 
                        // while (!imu_msg_.empty())
                        // {
                        //     if (imu_msg_.front().header.stamp.toSec() 
                        //             > curr_laser_odom.header.stamp.toSec() - 0.03) 
                        //     {
                        //         break;
                        //     }
                        //     imu_msg_.pop_front();  
                        // }
                        // // 判断数据范围
                        // if (imu_msg_.front().header.stamp.toSec() > curr_laser_odom.header.stamp.toSec())
                        // {
                        //     imu_mut_.unlock(); 
                        //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
                        //     continue;  
                        // }

                        // if (imu_msg_.back().header.stamp.toSec() < next_laser_odom.header.stamp.toSec())
                        // {
                        //     laser_odom_lock.lock(); 
                        //     laser_odom_msg_.push_front(curr_laser_odom); 
                        //     laser_odom_lock.unlock(); 
                        //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
                        //     continue;  
                        // }
                        // // 提取出wheel odom的数据 保存在wheel_odom_data_ 中 
                        // sensor_msgs::Imu begin_imu_data;
                        // is_begin = true; 
                        // for (uint16_t i = 0; i < imu_msg_.size(); i++)
                        // {
                        //     if (imu_msg_[i].header.stamp.toSec() <= curr_laser_odom.header.stamp.toSec())
                        //     {
                        //         begin_imu_data = imu_msg_[i]; 
                        //         continue;
                        //     }
                            
                        //     ImuData imu_odom;  
                        //     if (is_begin)
                        //     {
                        //         imu_odom.time_stamp_ = begin_imu_data.header.stamp.toSec();
                        //         imu_odom.omega_yaw_ = begin_imu_data.angular_velocity.z;  
                        //         imu_data_.push_back(imu_odom);  
                        //         is_begin = false;
                        //     }

                        //     sensor_msgs::Imu curr_imu_data = imu_msg_[i];
                        //     imu_odom.time_stamp_ = curr_imu_data.header.stamp.toSec();
                        //     imu_odom.omega_yaw_ = curr_imu_data.angular_velocity.z;  
                        //     imu_data_.push_back(imu_odom);  

                        //     if (imu_odom.time_stamp_ > next_laser_odom.header.stamp.toSec())
                        //     {
                        //         break;
                        //     }   
                        // }
                        
                        // imu_mut_.unlock();
                        // 求出激光里程计的位姿增量    即一次观测   
                        Pose2d delta_laser_pose = Utility::CalcIncremPose(curr_laser_odom, next_laser_odom);  

                        if (PARAM_ESTIMATE)  // 进行参数估计 
                        {
                            // 如果当前状态是 初始化 
                            if (status == INIT)
                            {
                                // 先初始化IMU角速度偏置    保持静止 数秒
                                // 接着进行激光odom外参初始化
                                Pose2d odom_delta_pose; 
                                // 求出轮速计的位姿增量  
                                if (GetMotionFromOdom(curr_laser_odom.header.stamp.toSec(), 
                                                                                next_laser_odom.header.stamp.toSec(), 
                                                                                wheel_odom_data_, odom_delta_pose))
                                {
                                    if (calib_.AddPose(delta_laser_pose, odom_delta_pose))
                                    {
                                        if (calib_.CalibMethod2())
                                        {
                                            ROS_INFO("calib done !!");
                                            calib_.GetCalibResult(extrinsic_lidar_odom_); 
                                            std::cout<<"extrinsic_lidar_odom_ x: "<<extrinsic_lidar_odom_.x_
                                            <<", y: "<<extrinsic_lidar_odom_.y_<<", yaw: "<<extrinsic_lidar_odom_.yaw_<<std::endl;
                                            status = ODOMETRY;  
                                        }
                                    }
                                }
                            }
                        }
                        // eskf 里程计运行
                        if (status == ODOMETRY)
                        {
                        }

                        //}
                        // else
                        // {
                        //     ROS_WARN("odom is not complete ! ");
                        //     // 如果odom 末尾数据  未覆盖住laser  
                        //     if (wheel_odom_msg_.front().header.stamp.toSec() <= curr_laser_odom.header.stamp.toSec())
                        //     {   
                        //         laser_odom_lock.lock();  
                        //         laser_odom_msg_.push_front(curr_laser_odom);  
                        //     }
                        // }

                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    
        // imu的回调函数
        void ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
        {
            std::lock_guard<std::mutex> lock(imu_mut_);
            imu_msg_.push_back(*imuMsg);
        }

        // odom的回调函数
        void OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg)
        {
            std::lock_guard<std::mutex> lock(wheel_odom_mut_);
            static bool is_start = true;
            static Pose2d start_pose;  
            static Eigen::Isometry3d start_pose_iso; 
            nav_msgs::Odometry odom_adjust;  

            Eigen::Vector2d P;
            Eigen::Vector3d P_3;
            P[0] = odometryMsg->pose.pose.position.x;
            P[1] = odometryMsg->pose.pose.position.y;  
            Eigen::Quaterniond orientation;
            orientation.w() = odometryMsg->pose.pose.orientation.w;
            orientation.x() = odometryMsg->pose.pose.orientation.x;
            orientation.y() = odometryMsg->pose.pose.orientation.y;
            orientation.z() = odometryMsg->pose.pose.orientation.z;

            if (is_start)
            {
                start_pose.x_ = P[0];
                start_pose.y_ = P[1];  
                start_pose.SetRotation(orientation);  
                std::cout<<"start_pose orientation: "<<orientation.coeffs().transpose()<<std::endl;
                std::cout<<"orientation.inv * ori: "<<(orientation.inverse() * orientation).coeffs().transpose()<<std::endl;
                start_pose_iso.translation() = Eigen::Vector3d(P[0], P[1], 0) ;
                start_pose_iso.linear() = orientation.toRotationMatrix();  
                //std::cout<<"start_pose x: "<<start_pose.x_<<" ,y: "<<start_pose.y_<<", theta: "<<start_pose.yaw_<<std::endl;
                std::cout<<"start_pose_iso: "<<start_pose_iso.matrix()<<std::endl;
                is_start = false; 
            }
            
            odom_adjust.header = odometryMsg->header;

            odom_adjust.twist.twist.linear.x = odometryMsg->twist.twist.linear.x;
            odom_adjust.twist.twist.linear.y = odometryMsg->twist.twist.linear.y;
            odom_adjust.twist.twist.angular.z = odometryMsg->twist.twist.angular.z;
            // 将坐标转换到起始坐标系 
            P = start_pose.inv() * P;  
            //P_3 = start_pose_iso.inverse() * Eigen::Vector3d(P[0], P[1], 0);
            orientation = start_pose.inv().orientation_ * orientation;
            std::cout<<"orientation: "<<orientation.coeffs().transpose()<<std::endl;
            //orientation = start_pose_iso.rotation().inverse() * orientation;

            Eigen::Isometry3d delta_T_1;
            delta_T_1.linear() = orientation.toRotationMatrix();
            delta_T_1.translation() = Eigen::Vector3d{P[0], P[1], 0};
            // 将原始轨迹做一个变换  模拟外参
            Eigen::Isometry3d T_1_2;
            // Eigen::AngleAxisd r(M_PI/3.6, Eigen::Vector3d(0, 0, 1));
            // T_1_2.linear() = r.toRotationMatrix();
            // T_1_2.translation() = Eigen::Vector3d(0.7,0.4, 0);
            Eigen::AngleAxisd r(0, Eigen::Vector3d(0, 0, 1));
            T_1_2.linear() = r.toRotationMatrix();
            T_1_2.translation() = Eigen::Vector3d(0.0,0.0, 0);

            Eigen::Isometry3d delta_T_2 = T_1_2.inverse() * delta_T_1 * T_1_2;
            // P[0] = delta_T_2.translation()[0];
            // P[1] = delta_T_2.translation()[1];
            // P[0] = P_3[0];
            // P[1] = P_3[1];
            //orientation = Eigen::Quaterniond(delta_T_2.rotation()); 

            odom_adjust.pose.pose.position.x = P[0];
            odom_adjust.pose.pose.position.y = P[1];  

            odom_adjust.pose.pose.orientation.w = orientation.w();
            odom_adjust.pose.pose.orientation.x= orientation.x();
            odom_adjust.pose.pose.orientation.y= orientation.y();
            odom_adjust.pose.pose.orientation.z= orientation.z();

            odom_adjust.header.frame_id = "odom";       // 基准坐标
            // 发布odom
            odom_pub.publish(odom_adjust); 
            wheel_odom_msg_.push_back(odom_adjust);
        }

        // 激光里程计的回调函数
        void LaserOdomCallback(const nav_msgs::Odometry::ConstPtr &laserOdomMsg)
        {
            std::lock_guard<std::mutex> lock(laser_odom_mut_);
            laser_odom_msg_.push_back(*laserOdomMsg);
            Eigen::Vector2d P;
            P[0] = laserOdomMsg->pose.pose.position.x;
            P[1] = laserOdomMsg->pose.pose.position.y;  
            std::cout<<"laserOdomMsg P: "<<P.transpose()<<std::endl;
        }

    private:
        enum WorkStatus
        {
            INIT = 0,    // 初始化状态
            PARAM_OPT, // 参数优化阶段， 滑动窗口优化  
            ODOMETRY // 里程计运行, 基于滤波器，可以在线优化参数   也可不优化   
        };
        WorkStatus status = INIT;   

        ros::NodeHandle node_handle_;  
        ros::Subscriber imu_subscriber_;
        ros::Subscriber wheel_odom_subscriber_;
        ros::Subscriber laser_odom_subscriber_;
        ros::Publisher odom_pub;

        std::deque<nav_msgs::Odometry> wheel_odom_msg_;
        std::deque<nav_msgs::Odometry> laser_odom_msg_;
        std::deque<sensor_msgs::Imu> imu_msg_;

        std::mutex imu_mut_, wheel_odom_mut_, laser_odom_mut_;  

        std::deque<Odom> wheel_odom_data_;
        std::deque<ImuData> imu_data_;

        HandEyeCalib2D calib_; 
        Pose2d extrinsic_lidar_odom_;       // lidar - odom的外参  
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optimization_fusion_odometry_node");
    FusionOdometryNode node;
    // std::thread estimator(&FusionOdometryNode::Estimator, &node); 
    ros::spin(); 
    return (0);
}




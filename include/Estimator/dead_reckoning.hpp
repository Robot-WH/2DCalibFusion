
/*
 * @Copyright(C):
 * @Author: lwh
 * @Description: 航迹推测  使用 IMU与odom
 * @Others: 
 */

#pragma once 

#include <eigen3/Eigen/Dense>
#include "sensor/sensor_data_type.hpp"
#include "../pose_2d.hpp"
#include "../utility.hpp"

namespace Slam2D {

/**
 * @brief: 输入imu角速度测量值，里程计速度测量值/位置测量
 */    
class DeadReckon2D {
    public:
        // 配置项
        struct Option {
            bool use_odom_motion_ = true;   
        };
        DeadReckon2D() {}
        DeadReckon2D(Option option) : option_(option) {
        }
        /**
         * @brief: 添加IMU观测
         * @details IMU用于更新角速度以及计算角度变化量, 角度变化量用于去除畸变 
         * @param bias yaw 角速度bias 
         */            
        void AddImuData(ImuData const& imu, double const& bias) {
            if (recorded_imu_rot_queue_.empty()) {
                last_imu_yaw_angular_v_ = imu.angular_v_[2] - bias; 
                recorded_imu_rot_queue_.emplace_back(imu.time_stamp_, 0); 
                cache_imu_for_odom_.emplace_back(imu.time_stamp_, last_imu_yaw_angular_v_);
                return;
            }
            ImuYawRot& last_rot = recorded_imu_rot_queue_.back();
            double dt = imu.time_stamp_ - last_rot.time_stamp_;
            double curr_rot = last_rot.yaw_rot_ + dt * 0.5 * (last_imu_yaw_angular_v_ + (imu.angular_v_[2] - bias));
            Utility::NormalizationAngle(curr_rot);
            recorded_imu_rot_queue_.emplace_back(imu.time_stamp_, curr_rot); 
            last_imu_yaw_angular_v_ = imu.angular_v_[2] - bias; 
            cache_imu_for_odom_.emplace_back(imu.time_stamp_, last_imu_yaw_angular_v_);
            // 保留 500ms 以内的数据
            while (recorded_imu_rot_queue_.back().time_stamp_ - 
                    recorded_imu_rot_queue_.front().time_stamp_ > 500) {
                recorded_imu_rot_queue_.pop_front(); 
            }
            while (cache_imu_for_odom_.back().time_stamp_ - 
                    cache_imu_for_odom_.front().time_stamp_ > 500) {
                cache_imu_for_odom_.pop_front(); 
            }
        }

        /**
         * @brief: 添加里程计观测，进行航迹推算
         * @details 使用odom的速度观测，旋转速度优先使用imu的观测
         *                     1、如果没有IMU角速度观测，则使用odom的角速度
         *                     2、若有IMU角速度数据，但是imu角速度数据距离当前时间太久(100ms)，则使用odom的角速度
         */            
        void AddOdometryData(Odom const& curr_odom) {
            cache_odom_.push_back(curr_odom); 
            if (cache_odom_.size() < 2) {
                return;
            }
            Odom& odom = cache_odom_.front();  
            cache_odom_.pop_front(); 
           
            static Pose2d last_odom_pose(odom.x_, odom.y_, odom.orientation_); 
            if (last_motion_.time_stamp_ == -1) {
                // 直接使用原始的odom数据初始化运动信息 
                last_motion_.time_stamp_ = odom.time_stamp_;
                last_motion_.linear_velocity_.x() = odom.v_x_;
                last_motion_.linear_velocity_.y() = odom.v_y_;
                last_motion_.angle_velocity_ = odom.omega_yaw_;
                return;
            }
            double angle_v;   // 角速度
            bool use_gyro_ = false;  
            // 若有IMU的角速度观测，则使用IMU角速度观测 代替odom角速度
            if (!cache_imu_for_odom_.empty()) {
                // 插值出本次 odom 时间戳处 旋转角速度
                if (cache_imu_for_odom_.back().time_stamp_ < odom.time_stamp_) {
                    cache_imu_for_odom_.clear();  
                } else {
                    ImuYawAngularVelocity before_imu;    // odom之前的一个imu数据
                    while (cache_imu_for_odom_.front().time_stamp_ <= odom.time_stamp_) {
                        before_imu = cache_imu_for_odom_.front();  
                        cache_imu_for_odom_.pop_front(); 
                    }
                    if (before_imu.time_stamp_ <= odom.time_stamp_ && before_imu.time_stamp_ > 0) {
                        ImuYawAngularVelocity& after_imu = cache_imu_for_odom_.front();   // odom后一个imu数据
                        // 进行插值 
                        odom.omega_yaw_ = Utility::Interpolate<double>(before_imu.yaw_angular_velocity_, 
                            after_imu.yaw_angular_velocity_, before_imu.time_stamp_, after_imu.time_stamp_, odom.time_stamp_); 
                        use_gyro_ = true;  
                    }
                }
            }
            
            double delta_time = odom.time_stamp_ - last_motion_.time_stamp_; 
            double linear_vx = 0, linear_vy = 0;  
            /************************ 基于差速运动学模型计算位姿****************************/
            // 不使用原始odom的速度信息  则通过位姿差计算速度，角速度优先使用IMU的
            if (!option_.use_odom_motion_) {
                Pose2d curr_pose(odom.x_, odom.y_, odom.orientation_); 
                Pose2d incre_pose = last_odom_pose.inv() * curr_pose;
                linear_vx = std::sqrt(std::pow(incre_pose.GetX() / delta_time, 2) + 
                                                        std::pow(incre_pose.GetY() / delta_time, 2));
                if (!use_gyro_) {
                    angle_v = incre_pose.GetYaw()  / delta_time;   
                } else {
                    angle_v = (odom.omega_yaw_ + last_motion_.angle_velocity_) / 2;   // 如果使用了IMU
                } 
                last_odom_pose = curr_pose; 
            } else {
                // 使用中值法
                linear_vx = (odom.v_x_ + last_motion_.linear_velocity_.x()) / 2;  
                angle_v = (odom.omega_yaw_ + last_motion_.angle_velocity_) / 2;  
            }
            // 差分运动学解算  
            double x = last_pose_.GetX() + 
                linear_vx * delta_time * cos(delta_time * angle_v / 2 + last_pose_.GetYaw());
            double y = last_pose_.GetY() + 
                linear_vx * delta_time * sin(delta_time * angle_v / 2 + last_pose_.GetYaw());   
            double yaw = last_pose_.GetYaw() + delta_time * angle_v; 
            last_pose_.SetTransform(x, y);
            last_pose_.SetRotation(yaw);  // 角度会进行规范化
            recorded_odom_pose_queue_.emplace_back(odom.time_stamp_, last_pose_); 
            // 更新当前运动信息    
            last_motion_.angle_velocity_ = odom.omega_yaw_;
            last_motion_.linear_velocity_.x() = linear_vx;
            last_motion_.linear_velocity_.y() = linear_vy;
            last_motion_.time_stamp_ = odom.time_stamp_;  
        }

        /**
         * @brief: 添加位姿观测 
         * @details: 匀速运动学模型计算 线速度和角速度 
         *                      1、没有IMU以及odom时，采用这里计算出来的匀速运动学模型进行预测
         *                      2、有IMU无odom，使用匀速运动模型的线速度，以及IMU的角速度
         *                      3、有IMU与odom，不使用匀速运动学模型
         */            
        void AddPoseData() {
            
        }

        /**
         * @brief: 推测时间time时的车体位姿
         * @details: 
         * @param undefined
         * @return {*}
         */        
        bool ExtrapolatorPose(double const& time, Pose2d &res) {
            // 有imu的数据   则使用 Imu的数据  插值出旋转 
            // if (recorded_imu_rot_queue_.empty()) {

            // }
            // 没有odom数据 ，则使用imu预测旋转，或匀速运动学模型预测
            if (recorded_odom_pose_queue_.empty()) {
                // 如果时间晚于 上一次pose的时间则 出错
            }
            if (time < recorded_odom_pose_queue_.front().first) {
                std::cout<<"time < recorded_odom_pose_queue_.front().first"<<std::endl;
                return false; 
            }
            // 如果时间戳 在odom 数据覆盖范围之外  则进行预测  否则就是插值  
            if (time > recorded_odom_pose_queue_.back().first) {
            }
            //std::cout<<"time: "<<time<<std::endl;
            // 插值 
            Pose2d before_pose, after_pose; 
            double before_time, after_time; 
            // 使用odom的pose 
            while(!recorded_odom_pose_queue_.empty()) {
                if (recorded_odom_pose_queue_.front().first >= time) {
                    after_pose = recorded_odom_pose_queue_.front().second;
                    after_time = recorded_odom_pose_queue_.front().first;
                    recorded_odom_pose_queue_.emplace_front(before_time, before_pose); 
                    //std::cout<<"after_time: "<<after_time<<std::endl;
                    break; 
                } else {
                    before_pose = recorded_odom_pose_queue_.front().second;
                    before_time = recorded_odom_pose_queue_.front().first;
                    //std::cout<<"before_time: "<<before_time<<std::endl;
                }
                recorded_odom_pose_queue_.pop_front();  
            }
            res = Utility::Interpolate<Pose2d>(before_pose, after_pose, before_time, after_time, time); 
            return true;  
        }

        /**
         * @brief: 获取航迹推算的当前位姿
         */        
        Pose2d GetDeadReckoningPose() const {
            return last_pose_; 
        }

    private:
        struct MotionInfo {
            double time_stamp_ = -1; 
            Eigen::Vector2d linear_velocity_;   // x，y方向
            double angle_velocity_;
        };
        struct ImuYawRot {
            ImuYawRot(double const& time_stamps, double const& yaw_rot) 
                :  time_stamp_(time_stamps), yaw_rot_(yaw_rot) {}
            double time_stamp_ = -1; 
            double yaw_rot_;
        };
        struct ImuYawAngularVelocity {
            ImuYawAngularVelocity() {}
            ImuYawAngularVelocity(double const& time_stamps, double const& yaw_angular_velocity) 
                :  time_stamp_(time_stamps), yaw_angular_velocity_(yaw_angular_velocity) {}
            double time_stamp_ = -1; 
            double yaw_angular_velocity_;
        };
        Option option_; 
        Pose2d last_pose_;   // 最新的pose 
        double last_imu_yaw_angular_v_;  // 最新的yaw角速度
        double b_i_ = 0.;    // imu的旋转bias 
        MotionInfo last_motion_;  
        std::deque<Odom> cache_odom_;   // odom的缓存 
        std::deque<ImuYawAngularVelocity> cache_imu_for_odom_;  
        std::deque<std::pair<double, Pose2d>> recorded_odom_pose_queue_;   // 记录里程计解算的位姿   用于激光去畸变以及匹配预测
        std::deque<ImuYawRot> recorded_imu_rot_queue_; 
}; // class
} // namespace 

#pragma once 
#include "common.hpp"
namespace hectorslam {
/**
 * @brief: 常规EKF估计框架
 */
class EstimatorEKF {
public: 
    /**
     * @brief: 初始化  即设置初始状态  
     */    
    void Init(const Pose2d& pose, const float& linear_velocity_measurement, 
            const float& angular_velocity_measurement, const double& time_stamp) {
        state_.X_[0] = pose.GetX();
        state_.X_[1] = pose.GetY();
        state_.X_[2] = pose.GetYaw();
        state_.cov_.setZero();  
        last_time_ = time_stamp;
        last_linear_velocity_ = linear_velocity_measurement;
        last_angular_velocity_ = angular_velocity_measurement; 
        last_posteriori_pose_ = pose;  
        last_pose_ = pose;  
        init_ = true;  
    }
    /**
     * @brief: 差分模型的预测
     * @param local_motion_increment 本次局部运动增量，在外部通过里程计/imu等解算得出
     * @param linear_velocity_measurement 线速度测量
     * @param angular_velocity_measurement 角速度测量值
     * @param noise_linear_velocity 线速度测量噪声
     * @param noise_angular_velocity 角速度测量噪声
     * @param time_stamp 测量的时间戳 
     * @return {*}
     */        
    void Predict(const Pose2d& local_motion_increment, const float& linear_velocity_measurement, 
            const float& angular_velocity_measurement, const float& noise_linear_velocity,
            const float& noise_angular_velocity, const double& time_stamp) {
        // 这个判断有两个作用 
        // 1、初始化前 last_time_ = DBL_MAX，因此，若未初始化，会直接退出
        // 2、自动忽略早于上一次处理时间的传感器数据 
        if (time_stamp <= last_time_) {
            return;  
        }
        // 更新预测均值
        // 实际上，下面式子并不完全相等，last_posteriori_pose_ 是上一帧雷达结束时的后验pose，
        // 而local_motion_increment 的起始参考位置是当前雷达起始坐标，然而当前雷达的起始坐标
        // 并不完全和上一帧雷达的结束坐标重合，因此，此处得到的预测Pose并不完全等于对应时刻的
        // 真实预测Pose,但是由于这个误差很小，因此这里忽略。
        Pose2d curr_pose = last_posteriori_pose_ * local_motion_increment;    
        state_.X_[0] = curr_pose.GetX(); 
        state_.X_[1] = curr_pose.GetY();
        state_.X_[2] = curr_pose.GetYaw();
        // 更新预测状态协方差 
        double dt = time_stamp - last_time_;
        double dt_2 = dt * dt;  
        float mid_linear_velocity = (linear_velocity_measurement + last_linear_velocity_) / 2; 
        float mid_anguler_velocity = (angular_velocity_measurement + last_angular_velocity_) / 2; 
        float curr_angular = last_pose_.GetYaw() + mid_anguler_velocity * dt / 2;
        Eigen::Matrix3f F;
        F << 1, 0, -mid_linear_velocity * dt * sin(curr_angular),
                    0, 1, mid_linear_velocity * dt * cos(curr_angular),
                    0, 0, 1;
        Eigen::Matrix<float, 3, 2> B;
        B << dt * cos(curr_angular),  -mid_linear_velocity * dt_2 * sin(curr_angular) / 2,
                    dt * sin(curr_angular), mid_linear_velocity * dt_2 * cos(curr_angular) / 2,
                    0, dt; 
        // 测量协方差
        Eigen::Matrix2f Q;
        Q << noise_linear_velocity, 0,
                    0, noise_angular_velocity; 
        state_.cov_ = F * state_.cov_ * F.transpose() + B * Q * B.transpose();  
        last_pose_ = curr_pose; 
        last_time_ = time_stamp; 
        last_linear_velocity_ = linear_velocity_measurement;
        last_angular_velocity_ = angular_velocity_measurement; 
        std::cout << "预测, " << "state: " << state_.X_.transpose() << ", time: " << std::setprecision(15)
        << last_time_ << std::endl;
        // std::cout << " cov: " << std::endl;
        // std::cout << state_.cov_ << std::endl;
    }
    /**
     * @brief: 观测校正
     * @details 位姿观测
     * @param obs 前后相对运动观测 
     * @param obs_cov 观测协方差
     * @param time_stamp 观测时间戳  
     */    
    void Correct(const Pose2d& obs, const Eigen::MatrixXf& obs_cov, const double& time_stamp) {
        std::cout << "校正，time_stamp: " << std::setprecision(15) << time_stamp << ", last_time_: "
            << last_time_ << std::endl;
        // 若观测与上次预测的时间戳相差不到1ms，认为观测有效
        if (std::fabs(time_stamp - last_time_) >= 1e-3) {
            throw "EKF correct time error!";
        }
        last_time_ = time_stamp;  
        std::cout << "obs: " << obs.ReadVec().transpose() << std::endl;
        Eigen::Matrix3f H; // 观测jacobian
        H.setIdentity(); 
        Eigen::Matrix3f C;  // 观测噪声jacobian
        C.setIdentity(); 
        Eigen::MatrixXf K = state_.cov_ * H.transpose() * 
                                                (H * state_.cov_ * H.transpose() + C * obs_cov * C.transpose()).inverse();
        state_.X_ = state_.X_ + K * (obs.ReadVec() - H * state_.X_);   // 后验均值
        Eigen::MatrixXf I_KH = Eigen::Matrix3f::Identity() - K * H;
        state_.cov_ = I_KH * state_.cov_ * I_KH.transpose() + K * C * obs_cov * C.transpose() * K.transpose();    // 后验方差
        last_posteriori_pose_.SetX(state_.X_[0]);
        last_posteriori_pose_.SetY(state_.X_[1]);
        last_posteriori_pose_.SetRotation(state_.X_[2]);
        last_pose_ = last_posteriori_pose_;
        std::cout << "校正完成, " << "pose: " << last_posteriori_pose_.ReadVec().transpose()  << std::endl;
        // std::cout << "cov: " << std::endl;
        // std::cout << state_.cov_ << std::endl;
    }

    /**
     * @brief: 读取pose的后验估计
     */    
    const Pose2d& ReadPosterioriPose() const {
        return last_posteriori_pose_;
    }

    bool is_init() {
        return init_;
    }
private:
    bool init_ = false;   
    State state_;  
    Pose2d last_posteriori_pose_;   // 上一次的后验pose
    Pose2d last_pose_;    
    double last_time_ = DBL_MAX; 
    double last_linear_velocity_ = 0;
    double last_angular_velocity_ = 0; 
};

/**
 * @brief: 以机器人坐标为中心的EKF估计框架
 */
class EstimatorRobocentricEKF {
public: 
    /**
     * @brief: 差分模型的预测
     * @param prior_mean 本次预测的2D pose均值，在外部通过里程计/imu等解算得出
     * @param linear_velocity_measurement 线速度测量
     * @param angular_velocity_measurement 角速度测量值
     * @param noise_linear_velocity 线速度测量噪声
     * @param noise_angular_velocity 角速度测量噪声
     * @param time_stamp 测量的时间戳 
     * @return {*}
     */        
    void Predict(const Pose2d& prior_mean, const float& linear_velocity_measurement, 
            const float& angular_velocity_measurement, const float& noise_linear_velocity,
            const float& noise_angular_velocity, const double& time_stamp) {
        // 本次EKF的第一次预测
        if (last_time_ < 0) {
            last_time_ = time_stamp;
            last_pose_ = prior_mean; 
            last_linear_velocity_ = linear_velocity_measurement;
            last_angular_velocity_ = angular_velocity_measurement; 
            return;
        }
        // 更新预测均值，在外部已经计算好了，直接拿来赋值就行
        state_.X_[0] = prior_mean.GetX();
        state_.X_[1] = prior_mean.GetY();
        state_.X_[2] = prior_mean.GetYaw();
        // 更新预测状态协方差 
        double dt = time_stamp - last_time_;
        double dt_2 = dt * dt;  
        float mid_linear_velocity = (linear_velocity_measurement + last_linear_velocity_) / 2; 
        float mid_anguler_velocity = (angular_velocity_measurement + last_angular_velocity_) / 2; 
        float curr_angular = last_pose_.GetYaw() + mid_anguler_velocity * dt / 2;
        Eigen::Matrix3f F;
        F << 1, 0, -mid_linear_velocity * dt * sin(curr_angular),
                    0, 1, mid_linear_velocity * dt * cos(curr_angular),
                    0, 0, 1;
        Eigen::Matrix<float, 3, 2> B;
        B << dt * cos(curr_angular),  -mid_linear_velocity * dt_2 * sin(curr_angular) / 2,
                    dt * sin(curr_angular), mid_linear_velocity * dt_2 * cos(curr_angular) / 2,
                    0, dt; 
        // 测量协方差
        Eigen::Matrix2f Q;
        Q << noise_linear_velocity, 0,
                    0, noise_angular_velocity; 
        state_.cov_ = F * state_.cov_ * F.transpose() + B * Q * B.transpose();  
        last_pose_ = prior_mean; 
    }
    /**
     * @brief: 观测校正
     * @details 位姿观测
     */    
    void Correct(const Pose2d& obs, const Eigen::MatrixXf& obs_cov) {
        Eigen::Matrix3f H; // 观测jacobian
        H.setIdentity(); 
        Eigen::Matrix3f C;  // 观测噪声jacobian
        C.setIdentity(); 
        Eigen::MatrixXf K = state_.cov_ * H.transpose() * 
                                                (H * state_.cov_ * H.transpose() + C * obs_cov * C.transpose()).inverse();
        state_.X_ = state_.X_ + K * (obs.ReadVec() - H * state_.X_);   // 后验均值
        Eigen::MatrixXf I_KH = Eigen::Matrix3f::Identity() - K * H;
        state_.cov_ = I_KH * state_.cov_ * I_KH.transpose() + K * C * obs_cov * C.transpose() * K.transpose();    // 后验方差
    }
private:
    State state_;  
    Pose2d last_pose_; 
    double last_time_ = -1; 
    double last_linear_velocity_ = 0;
    double last_angular_velocity_ = 0; 
};
}


#pragma once 
#include "common.hpp"
namespace Estimator2D {
/**
 * @brief: 常规EKF估计框架
 * @details 状态 位置 + 旋转 + 速度
 */
class EKFEstimatorBase {
public:
    EKFEstimatorBase() = default;
    virtual ~EKFEstimatorBase() = default; 
    EKFEstimatorBase(EKFEstimatorBase&&) = default;
    EKFEstimatorBase& operator=(EKFEstimatorBase&&) = default;
    EKFEstimatorBase(const EKFEstimatorBase&) = default;
    EKFEstimatorBase& operator=(const EKFEstimatorBase&) = default;

    EKFEstimatorBase(const msa2d::Pose2d& pose, const float& linear_velocity_measurement, 
            const float& angular_velocity_measurement, const double& time_stamp) {
        state_.cov_ = Eigen::Matrix<float, 5, 5>::Zero();  
        state_.X_ = Eigen::Matrix<float, 5, 1>::Zero();  
        state_.X_[0] = pose.x();    // px
        state_.X_[1] = pose.y();    // py
        state_.X_[2] = pose.yaw();   // theta
        state_.X_[3] = linear_velocity_measurement;  // vx
        state_.X_[4] = angular_velocity_measurement;  // omega
        last_state_ = state_; 
        prev_state_ = state_;
        last_time_ = time_stamp;
        last_linear_velocity_ = linear_velocity_measurement;
        last_angular_velocity_ = angular_velocity_measurement; 
        last_posteriori_pose_ = pose;  
        last_pose_ = pose;  
        std::cout << msa2d::color::GREEN << "EKFEstimatorBase init done!!" << std::endl;
    }
    /**
     * @brief: 没有任何传感器输入时的匀速运动学模型预测
     */    
    void Predict(const double& start_time, const double& end_time, DiffModelDeadReckoning& diff_model) {
        // 没有轮速数据则用运动学模型预测
        float filtered_linear_v = (prev_state_.X_[3] + last_state_.X_[3]) / 2;
        float filtered_rot_v = (prev_state_.X_[4] + last_state_.X_[4]) / 2;
        // 匀速运动学模型前向传播
        diff_model.Update(start_time, filtered_linear_v, filtered_rot_v); 
        diff_model.Update(end_time, filtered_linear_v, filtered_rot_v); 
        Predict(diff_model.ReadLastPose().pose_, filtered_linear_v, filtered_rot_v, 
            0.25, 5, end_time); 
    }

    /**
     * @brief 
     * 
     * @param angular_velocity_measurement 
     * @param noise_angular_velocity 
     * @param time_stamp 
     * @param diff_model 
     */
    virtual void Predict(const float& angular_velocity_measurement, const float& noise_angular_velocity, 
            const double& time_stamp, DiffModelDeadReckoning& diff_model) {}

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
    virtual void Predict(const msa2d::Pose2d& local_motion_increment, const float& linear_velocity_measurement, 
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
        msa2d::Pose2d curr_pose = last_posteriori_pose_ * local_motion_increment;    
        state_.X_[0] = curr_pose.x(); 
        state_.X_[1] = curr_pose.y();
        state_.X_[2] = curr_pose.yaw();
        state_.X_[3] = linear_velocity_measurement; 
        state_.X_[4] = angular_velocity_measurement; 
        // 更新预测状态协方差 
        double dt = time_stamp - last_time_;
        double dt_2 = dt * dt;  
        float mid_linear_velocity = (linear_velocity_measurement + last_linear_velocity_) / 2; 
        float mid_anguler_velocity = (angular_velocity_measurement + last_angular_velocity_) / 2; 
        float curr_angular = last_pose_.yaw() + mid_anguler_velocity * dt / 2;
        Eigen::Matrix<float, 5, 5> F;
        F.setZero();  
        F(0, 0) = 1; F(1, 1) = 1; F(2, 2) = 1;
        F(0, 2) = -mid_linear_velocity * dt * sin(curr_angular);
        F(1, 2) = mid_linear_velocity * dt * cos(curr_angular);       
        Eigen::Matrix<float, 5, 2> B;
        B << dt * cos(curr_angular),  -mid_linear_velocity * dt_2 * sin(curr_angular) / 2,
                    dt * sin(curr_angular), mid_linear_velocity * dt_2 * cos(curr_angular) / 2,
                    0, dt,
                    1, 0,
                    0, 1; 
        // 测量协方差
        Eigen::Matrix2f Q;
        Q << noise_linear_velocity, 0,
                    0, noise_angular_velocity; 
        state_.cov_ = F * state_.cov_ * F.transpose() + B * Q * B.transpose();  
        last_pose_ = curr_pose; 
        last_time_ = time_stamp; 
        last_linear_velocity_ = linear_velocity_measurement;
        last_angular_velocity_ = angular_velocity_measurement; 
        // std::cout << "预测, " << "state: " << state_.X_.transpose() << ", time: " << std::setprecision(15)
        // << last_time_ << std::endl;
        // std::cout << " cov: " << std::endl;
        // std::cout << state_.cov_ << std::endl;
    }

    /**
     * @brief EKF 校正
     * 
     * @param obs 
     * @param obs_cov 
     * @param time_stamp 
     */
    virtual void Correct(const msa2d::Pose2d& obs, const Eigen::MatrixXf& obs_cov, const double& time_stamp) {
        // std::cout << "校正，time_stamp: " << std::setprecision(15) << time_stamp << ", last_time_: "
        //     << last_time_ << std::endl;
        // 若观测与上次预测的时间戳相差不到1ms，认为观测有效
        if (std::fabs(time_stamp - last_time_) >= 1e-3) {
        }

        last_time_ = time_stamp;  
        // std::cout << "obs: " << obs.ReadVec().transpose() << std::endl;
        Eigen::Matrix<float, 3, 5> H; // 观测jacobian
        H.setZero(); 
        H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1;
        Eigen::Matrix3f C;  // 观测噪声jacobian
        C.setIdentity(); 
        Eigen::MatrixXf K = state_.cov_ * H.transpose() * 
                                                (H * state_.cov_ * H.transpose() + C * obs_cov * C.transpose()).inverse();
        Eigen::MatrixXf diff_obs = obs.vec() - H * state_.X_;
        // std::cout << "K: " << std::endl << K << std::endl;
        if (diff_obs(2, 0) > M_PI) {
            diff_obs(2, 0) = diff_obs(2, 0) - 2 * M_PI; 
        } else if (diff_obs(2, 0) < -M_PI) {
            diff_obs(2, 0) =  2 * M_PI + diff_obs(2, 0); 
        }
        // 修正量
        Eigen::VectorXf amendment = K * diff_obs;
        // std::cout << "amendment: "<< amendment.transpose() << std::endl;
        state_.X_ += amendment;   // 后验均值
        Eigen::MatrixXf I_KH = Eigen::Matrix<float, 5, 5>::Identity() - K * H;
        state_.cov_ = I_KH * state_.cov_ * I_KH.transpose() + 
                                    K * C * obs_cov * C.transpose() * K.transpose();    // 后验方差
        last_posteriori_pose_.SetX(state_.X_[0]);
        last_posteriori_pose_.SetY(state_.X_[1]);
        last_posteriori_pose_.SetRotation(state_.X_[2]);
        prev_state_ = last_state_;
        last_state_ = state_; 
        last_pose_ = last_posteriori_pose_;
        std::cout << "校正完成, " << "状态: " << state_.X_.transpose() << std::endl;
        // std::cout << "cov: " << std::endl;
        // std::cout << state_.cov_ << std::endl;
        // util::SaveDataCsv<double>("", "v.csv", {state_.X_[3], state_.X_[4]}, {"linear_v", "yaw_v"});
    }
    
    /**
     * @brief 
     * 
     * @return const State& 
     */
    const State& ReadState() const {
        return state_; 
    }

    /**
     * @brief: 读取pose的后验估计
     */    
    const msa2d::Pose2d& ReadPosterioriPose() const {
        return last_posteriori_pose_;
    }
protected:
    State prev_state_;   // 上一次的上一次校正后的状态
    State last_state_;   // 上一次校正后的状态
    State state_;   // 当前
    msa2d::Pose2d last_posteriori_pose_;   // 上一次的后验pose
    msa2d::Pose2d last_pose_;    
    double last_time_ = DBL_MAX; 
    double last_linear_velocity_ = 0;
    double last_angular_velocity_ = 0; 
}; 


/**
 * @brief: 融合IMU的EKF
 * @details 状态 位置 + 旋转 + 速度 + IMU bias 
 */
class EKFEstimatorPVQB : public EKFEstimatorBase {
public: 
    EKFEstimatorPVQB(const msa2d::Pose2d& pose, const float& linear_velocity_measurement, 
            const float& angular_velocity_measurement, const float& imu_bias, const float& n_bias, 
            const double& time_stamp) : n_bias_(n_bias) {
        state_.cov_ = Eigen::Matrix<float, 6, 6>::Zero();  
        state_.X_ = Eigen::Matrix<float, 6, 1>::Zero();  
        state_.X_[0] = pose.x();    // px
        state_.X_[1] = pose.y();    // py
        state_.X_[2] = pose.yaw();   // theta
        state_.X_[3] = linear_velocity_measurement;  // vx
        state_.X_[4] = angular_velocity_measurement;  // omega
        state_.X_[5] = imu_bias; 
        last_state_ = state_; 
        prev_state_ = state_;
        last_time_ = time_stamp;
        last_linear_velocity_ = linear_velocity_measurement;
        last_angular_velocity_ = angular_velocity_measurement;
        last_posteriori_pose_ = pose; 
        last_pose_ = pose;
        std::cout << msa2d::color::GREEN << "EKFEstimatorPVQB init done!!" << std::endl;
    }

    /**
     * @brief: 差分模型  有imu角速度信息  无线速度信息 
     * @param angular_velocity_measurement 角速度测量值 - 未去除bias的原始值
     * @param noise_angular_velocity 角速度测量噪声
     * @param time_stamp 测量的时间戳 
     * @param diff_model 航迹推算器  
     * @return {*}
     */        
    void Predict(const float& angular_velocity_measurement, const float& noise_angular_velocity, 
            const double& time_stamp, DiffModelDeadReckoning& diff_model) override {

        float un_bias_yaw_angular_v = angular_velocity_measurement - last_state_.X_[5]; // 取bias 
        // 没有线速度数据则用运动学模型预测
        float filtered_linear_v = (prev_state_.X_[3] + last_state_.X_[3]) / 2;
        diff_model.Update(time_stamp, filtered_linear_v, un_bias_yaw_angular_v); 
        // 自动忽略早于上一次处理时间的传感器数据 
        if (time_stamp <= last_time_) {
            return;  
        }
        
        Predict(diff_model.ReadLastPose().pose_, filtered_linear_v, 
            un_bias_yaw_angular_v, 0.25, noise_angular_velocity, time_stamp); 
    }

    /**
     * @brief: 差分模型  有速度信息和imu角速度信息
     * @param local_motion_increment 本次局部运动增量，在外部通过里程计/imu等解算得出
     * @param linear_velocity_measurement 线速度测量
     * @param angular_velocity_measurement 角速度测量值(去除bias)
     * @param noise_linear_velocity 线速度测量噪声
     * @param noise_angular_velocity 角速度测量噪声
     * @param time_stamp 测量的时间戳 
     * @return {*}
     */        
    void Predict(const msa2d::Pose2d& local_motion_increment, const float& linear_velocity_measurement, 
            const float& angular_velocity_measurement, const float& noise_linear_velocity,
            const float& noise_angular_velocity, const double& time_stamp) override {
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
        msa2d::Pose2d curr_pose = last_posteriori_pose_ * local_motion_increment;    
        state_.X_[0] = curr_pose.x(); 
        state_.X_[1] = curr_pose.y();
        state_.X_[2] = curr_pose.yaw();
        state_.X_[3] = linear_velocity_measurement; 
        state_.X_[4] = angular_velocity_measurement; 
        // 更新预测状态协方差 
        double dt = time_stamp - last_time_;
        double dt_2 = dt * dt;  
        float mid_linear_velocity = (linear_velocity_measurement + last_linear_velocity_) / 2; 
        float mid_anguler_velocity = (angular_velocity_measurement + last_angular_velocity_) / 2; 
        float curr_angular = last_pose_.yaw() + mid_anguler_velocity * dt / 2;
        Eigen::Matrix<float, 6, 6> F;
        F.setZero();  
        F(0, 0) = 1; F(1, 1) = 1; F(2, 2) = 1;
        F(2, 5) = -dt; F(4, 5) = -1; F(5, 5) = 1;
        F(0, 2) = -mid_linear_velocity * dt * sin(curr_angular);
        F(0, 5) = mid_linear_velocity * dt_2 * sin(curr_angular) / 2;
        F(1, 2) = mid_linear_velocity * dt * cos(curr_angular);       
        F(1, 5) = -mid_linear_velocity * dt_2 * cos(curr_angular) / 2;

        Eigen::Matrix<float, 6, 3> B;
        B.setZero();
        B(0, 0) = dt * cos(curr_angular);
        B(0, 1) = -mid_linear_velocity * dt_2 * sin(curr_angular) / 2;
        B(1, 0) = dt * sin(curr_angular);
        B(1, 1) = mid_linear_velocity * dt_2 * cos(curr_angular) / 2;
        B(2, 1) = dt;
        B(3, 0) = 1; B(4, 1) = 1; B(5, 2) = 1;
        // 测量协方差
        Eigen::Matrix3f Q;
        Q << noise_linear_velocity, 0, 0,
                    0, noise_angular_velocity, 0,
                    0, 0, dt * n_bias_; 
        state_.cov_ = F * state_.cov_ * F.transpose() + B * Q * B.transpose();  
        last_pose_ = curr_pose; 
        last_time_ = time_stamp; 
        last_linear_velocity_ = linear_velocity_measurement;
        last_angular_velocity_ = angular_velocity_measurement; 
        // std::cout << "预测, " << "state: " << state_.X_.transpose() << ", time: " << std::setprecision(15)
        // << last_time_ << std::endl;
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
    void Correct(const msa2d::Pose2d& obs, const Eigen::MatrixXf& obs_cov, const double& time_stamp) override {
        // std::cout << "校正，time_stamp: " << std::setprecision(15) << time_stamp << ", last_time_: "
        //     << last_time_ << std::endl;
        // 若观测与上次预测的时间戳相差不到1ms，认为观测有效
        if (std::fabs(time_stamp - last_time_) >= 1e-3) {
        }

        last_time_ = time_stamp;  
        // std::cout << "obs: " << obs.ReadVec().transpose() << std::endl;
        Eigen::Matrix<float, 3, 6> H; // 观测jacobian
        H.setZero(); 
        H(0, 0) = 1; H(1, 1) = 1; H(2, 2) = 1;
        Eigen::Matrix3f C;  // 观测噪声jacobian
        C.setIdentity(); 
        Eigen::MatrixXf K = state_.cov_ * H.transpose() * 
                                                (H * state_.cov_ * H.transpose() + C * obs_cov * C.transpose()).inverse();
        Eigen::MatrixXf diff_obs = obs.vec() - H * state_.X_;
        // std::cout << "K: " << std::endl << K << std::endl;
        if (diff_obs(2, 0) > M_PI) {
            diff_obs(2, 0) = diff_obs(2, 0) - 2 * M_PI; 
        } else if (diff_obs(2, 0) < -M_PI) {
            diff_obs(2, 0) =  2 * M_PI + diff_obs(2, 0); 
        }
        // 修正量
        Eigen::VectorXf amendment = K * diff_obs;
        // std::cout << "amendment: "<< amendment.transpose() << std::endl;
        state_.X_ += amendment;   // 后验均值
        Eigen::MatrixXf I_KH = Eigen::Matrix<float, 6, 6>::Identity() - K * H;
        state_.cov_ = I_KH * state_.cov_ * I_KH.transpose() + 
                                    K * C * obs_cov * C.transpose() * K.transpose();    // 后验方差
        last_posteriori_pose_.SetX(state_.X_[0]);
        last_posteriori_pose_.SetY(state_.X_[1]);
        last_posteriori_pose_.SetRotation(state_.X_[2]);
        prev_state_ = last_state_;
        last_state_ = state_; 
        last_pose_ = last_posteriori_pose_;
        std::cout << "校正完成, " << "状态: " << state_.X_.transpose() << std::endl;
        // std::cout << "cov: " << std::endl;
        // std::cout << state_.cov_ << std::endl;
        // util::SaveDataCsv<double>("", "bias.csv", {state_.X_[5], time_stamp}, {"bias", "time"});
    }
private:
    float n_bias_;   // imu bias随机游走噪声
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
    void Predict(const msa2d::Pose2d& prior_mean, const float& linear_velocity_measurement, 
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
        state_.X_[0] = prior_mean.x();
        state_.X_[1] = prior_mean.y();
        state_.X_[2] = prior_mean.yaw();
        // 更新预测状态协方差 
        double dt = time_stamp - last_time_;
        double dt_2 = dt * dt;  
        float mid_linear_velocity = (linear_velocity_measurement + last_linear_velocity_) / 2; 
        float mid_anguler_velocity = (angular_velocity_measurement + last_angular_velocity_) / 2; 
        float curr_angular = last_pose_.yaw() + mid_anguler_velocity * dt / 2;
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
    void Correct(const msa2d::Pose2d& obs, const Eigen::MatrixXf& obs_cov) {
        Eigen::Matrix3f H; // 观测jacobian
        H.setIdentity(); 
        Eigen::Matrix3f C;  // 观测噪声jacobian
        C.setIdentity(); 
        Eigen::MatrixXf K = state_.cov_ * H.transpose() * 
                                                (H * state_.cov_ * H.transpose() + C * obs_cov * C.transpose()).inverse();
        state_.X_ = state_.X_ + K * (obs.vec() - H * state_.X_);   // 后验均值
        Eigen::MatrixXf I_KH = Eigen::Matrix3f::Identity() - K * H;
        state_.cov_ = I_KH * state_.cov_ * I_KH.transpose() + K * C * obs_cov * C.transpose() * K.transpose();    // 后验方差
    }
private:
    State state_;  
    msa2d::Pose2d last_pose_; 
    double last_time_ = -1; 
    double last_linear_velocity_ = 0;
    double last_angular_velocity_ = 0; 
};
}

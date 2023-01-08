
#pragma once 
#include <deque>
#include "../../Type/Pose2d.hpp"
#include "../../Type/color.hpp"
#include "../../Sensor/SensorData.hpp"
namespace hectorslam {
class ImuInitializer2D {
public:
    ImuInitializer2D() {
    }
    /**
     * @brief: 支持静止初始化以及运动初始化
     * @return {*}
     */        
    bool Init(const Pose2d& pose_increm, const std::deque<ImuData>& data) {
        float rot_increm = pose_increm.yaw() * 180 / M_PI;   // 角度变化量
        // std::cout << "rot_increm: " << rot_increm << std::endl;
        // 角度变化量 < 0.1度() 则认为可以静止初始化  
        if (rot_increm < 1e-2) {
            // 静止初始化
            std::cout << common::YELLOW << "imu static init ...... " << common::RESET << std::endl;

            if (staticInit(data)) {
                std::cout << common::GREEN << "imu static init success...... " << common::RESET << std::endl;
                std::cout << common::GREEN << "imu yaw bias:  "<< yaw_bias_ << common::RESET << std::endl;
                return true;  
            }
        } else {
            std::cout << common::YELLOW << "imu motion init...... " << common::RESET << std::endl;
            // 运动初始化
            static_init_N_ = 1; 
            yaw_velocity_avg_ = 0;
            yaw_velocity_var_ = 0; 
        }
        return false;  
    }

    float GetYawBias() {
        return yaw_bias_;
    }
private:
    /**
     * @brief: 静止初始化
     * @details: 积累连续的20个数据即可进行初始化
     * @return 是否成功
     */        
    bool staticInit(const std::deque<ImuData>& packet) {

        for (const auto& data : packet) {
            if (static_init_N_ > 1) {
                yaw_velocity_var_ = yaw_velocity_var_ * (static_init_N_ - 2) / (static_init_N_ - 1)
                    + pow((data.angular_v_[2] - yaw_velocity_avg_), 2) / static_init_N_;  // 估计样本方差
            }
            yaw_velocity_avg_ += (data.angular_v_[2] - yaw_velocity_avg_) / static_init_N_;  // 估计样本均值
            ++static_init_N_;
        }

        if (static_init_N_ > 20) {
            // 如果方差足够小，则认为该均值估计可信
            if (yaw_velocity_var_ < 1e-6) {
                yaw_bias_ = yaw_velocity_avg_;
                return true; 
            }
        }

        return false; 
    }

    bool motionInit() {
        return false; 
    }
    
    uint16_t static_init_N_ = 1; 
    float yaw_velocity_avg_ = 0;
    float yaw_velocity_var_ = 0; 
    float yaw_bias_; 
};
} // namespace name

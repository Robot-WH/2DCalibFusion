
#pragma once 
#include <deque>
#include "msa2d/common/Pose2d.hpp"
#include "msa2d/common/color.hpp"
#include "msa2d/Sensor/SensorData.hpp"
namespace Estimator2D {
class ImuInitializer2D {
public:
    ImuInitializer2D() {
    }
    /**
     * @brief: 支持静止初始化以及运动初始化
     * @return {*}
     */        
    bool Init(const std::deque<msa2d::sensor::ImuData>& data) {
        // 静止初始化
        std::cout << msa2d::color::YELLOW << "imu static init ...... " << msa2d::color::RESET << std::endl;

        if (staticInit(data)) {
            std::cout << msa2d::color::GREEN << "imu static init success...... " << msa2d::color::RESET << std::endl;
            std::cout << msa2d::color::GREEN << "imu yaw bias:  "<< yaw_bias_ << msa2d::color::RESET << std::endl;
            return true;  
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
    bool staticInit(const std::deque<msa2d::sensor::ImuData>& packet) {

        for (const auto& data : packet) {
            if (static_init_N_ > 1) {
                yaw_velocity_var_ = yaw_velocity_var_ * (static_init_N_ - 2) / (static_init_N_ - 1)
                    + pow((data.angular_v_[2] - yaw_velocity_avg_), 2) / static_init_N_;  // 估计样本方差
            }
            yaw_velocity_avg_ += (data.angular_v_[2] - yaw_velocity_avg_) / static_init_N_;  // 估计样本均值
            ++static_init_N_;
        }

        if (static_init_N_ > 50) {
            std::cout << msa2d::color::YELLOW << "yaw_velocity_avg: " << yaw_velocity_avg_
                << ",yaw_velocity_var: " << yaw_velocity_var_ << msa2d::color::RESET << std::endl;
            // 如果方差和均值足够小，则认为该均值估计可信
            if (yaw_velocity_avg_ < 1e-1&& yaw_velocity_var_ < 1e-6) {
                yaw_bias_ = yaw_velocity_avg_;
                return true; 
            } else {
                // 重新估计 
                static_init_N_ = 1; 
                yaw_velocity_avg_ = 0;
                yaw_velocity_var_ = 0; 
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

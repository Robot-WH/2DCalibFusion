
#pragma once 
#include <eigen3/Eigen/Dense>
#include "../../Type/Pose2d.hpp"
#include "../Sensor/SensorData.hpp"
namespace hectorslam {
namespace Estimator {

// 状态 pos + 姿态 
struct State {
    Eigen::Vector2f pos_;    // 位置
    float yaw_;  // 姿态角  
    Eigen::Matrix3d cov_;  
};

class EstimatorEKF {
public: 
    /**
     * @brief: 
     * @details: 
     * @param prior_mean 预测状态的均值，在外部模块计算出来再传入 
     * @return {*}
     */        
    void Predict(const Pose2d& prior_mean, 
                                const float& linear_velocity, 
                                const float& angular_velocity,
                                const double& time_stamp) {
        // 更新预测状态协方差 
        
    }
private:
    State state_;  
};
}
}

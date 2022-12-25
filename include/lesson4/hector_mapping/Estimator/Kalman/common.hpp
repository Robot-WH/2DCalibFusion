
#pragma once 
#include <eigen3/Eigen/Dense>
#include "../motionModel.hpp"
#include "../../Type/Pose2d.hpp"
#include "../../Type/color.hpp"
#include "../../Sensor/SensorData.hpp"

namespace hectorslam {
// 状态 pos + 姿态 
struct State {
    Eigen::VectorXf X_;    // 位置 + 旋转 + 速度+imu bias 
    Eigen::MatrixXf cov_;  
};

}


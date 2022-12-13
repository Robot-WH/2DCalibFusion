
#pragma once 
#include <eigen3/Eigen/Dense>
#include "../../Type/Pose2d.hpp"
#include "../../Type/color.hpp"
#include "../../Sensor/SensorData.hpp"

namespace hectorslam {
// 状态 pos + 姿态 
struct State {
    Eigen::Vector3f X_;    // 位置 + 旋转
    Eigen::Matrix3f cov_;  
};
struct PoseObs {
    Pose2d obs_;

};

}


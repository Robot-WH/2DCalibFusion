
#pragma once 
#include <Eigen/Dense>
#include "../motionModel.hpp"
#include "../../util/utility.hpp"
#include "msa2d/common/Pose2d.hpp"
#include "msa2d/common/color.hpp"
#include "msa2d/Sensor/SensorData.hpp"

namespace Estimator2D {
// 状态 pos + 姿态 
struct State {
    Eigen::VectorXf X_;    // 位置 + 旋转 + 速度+imu bias 
    Eigen::MatrixXf cov_;  
};

}


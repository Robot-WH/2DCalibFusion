
#pragma once 

#include <deque>
#include "msa2d/common/Pose2d.hpp"

namespace Estimator2D {

using Path = std::deque<msa2d::TimedPose2d>; 

class DeadReckoningBase {
public:
    virtual ~DeadReckoningBase() {}
    void Reset() {
        path_.clear();
        last_pose_.time_stamp_ = -1; 
    }

    const msa2d::TimedPose2d& ReadLastPose() const {
        return last_pose_; 
    }

    Path& GetPath() {
        return path_; 
    }
protected:
    msa2d::TimedPose2d last_pose_;  
    Path path_;   // 轨迹
};

/**
 * @brief: 差分模型航迹推算
 */
class DiffModelDeadReckoning : public DeadReckoningBase {
public:
    DiffModelDeadReckoning(uint16_t path_capacity = 100) : path_capacity_(path_capacity) {
    }
    /**
     * @brief: 基于中值积分的航迹推算
     * @param time_stamp 输入数据的时间戳
     * @param linear_velocity 线速度测量值
     * @param angular_velocity 角速度测量值    单位 弧度 
     * @param last_pose
     * @return {*}
     */        
    void Update(const double& time_stamp, const float& linear_velocity, 
                                    const float& angular_velocity ) {
        // 非初始状态 
        if (last_pose_.time_stamp_ != -1) {
            double dt = time_stamp - last_pose_.time_stamp_;
            float mid_linear_velocity = (linear_velocity + last_linear_velocity_) / 2; 
            float mid_angular_velocity = (angular_velocity + last_angular_velocity_) / 2;
            float x = last_pose_.pose_.x() + mid_linear_velocity * dt 
                * cos(last_pose_.pose_.yaw() + mid_angular_velocity * dt / 2);
            float y = last_pose_.pose_.y() + mid_linear_velocity * dt 
                * sin(last_pose_.pose_.yaw() + mid_angular_velocity * dt / 2);
            last_pose_.pose_.SetTransform(x, y);
            last_pose_.pose_.SetRotation(last_pose_.pose_.yaw() + mid_angular_velocity * dt);
        }
        last_pose_.time_stamp_ = time_stamp;
        last_linear_velocity_ = linear_velocity;
        last_angular_velocity_ = angular_velocity;
        path_.push_back(last_pose_);
        if (path_.size() > path_capacity_) {
            path_.pop_front();  
        }
    }

private:
    float last_linear_velocity_ = -1; 
    float last_angular_velocity_ = -1;  
    uint16_t path_capacity_; // 轨迹能记录的最多pose数量
};
}
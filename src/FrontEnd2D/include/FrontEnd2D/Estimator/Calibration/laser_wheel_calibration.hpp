#pragma once 
#include <utility>
#include <queue>
#include <Eigen/Dense>
#include "msa2d/common/Pose2d.hpp"
#include "msa2d/common/color.hpp"
namespace Estimator2D {
// 用于优先队列 将旋转小的元素放到前面 
// 小顶堆
struct rotCmp {
    bool operator()(const std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>> &pose_r, 
                                    const std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>> &pose_l) {
        return (std::fabs(pose_l.second.first.yaw()) > std::fabs(pose_r.second.first.yaw()));   
    }
};

class laserWheelCalib {
public:
    struct Res {
        float rot_scale;   // 旋转尺度 
        float trans_scale;   // 线速度尺度 
        float x, y, theta;     // 外参
    };
    /**
     * @brief 
     * 
     * @param pose_laser 
     * @param pose_wheel 
     * @return true 
     * @return false 
     */
    bool AddData(const  msa2d::Pose2d& pose_laser, const  msa2d::Pose2d& pose_wheel) {
        // 要求一帧雷达数据间的旋转角度大于1度，才被认为是 有效的
        if (std::fabs(pose_laser.yaw()) < 0.0175) {
            return false;
        }

        std::cout << "wheel-laser calib AddData, pose_laser: " << pose_laser.vec().transpose()
            << ",pose_wheel: " << pose_wheel.vec().transpose() << std::endl;

        if (pose_storage_.size() < N_POSE) {
            priority_pose_.emplace(pose_storage_.size(), 
                std::make_pair(pose_laser, pose_wheel));  
            pose_storage_.emplace_back(pose_laser, pose_wheel); 
        } else {   // 数据量大于300    则滑动窗口 
            // maintain a min heap
            uint16_t pos = priority_pose_.top().first;  
            std::cout<<"num > thresh, remove top theta: "<< priority_pose_.top().second.first.yaw()<<std::endl;
            pose_storage_[pos] = std::make_pair(pose_laser, pose_wheel);  
            priority_pose_.pop();   
            priority_pose_.emplace(pos, std::make_pair(pose_laser, pose_wheel));
        }   
        std::cout << "pose_storage_.size(): " << pose_storage_.size() << std::endl;
        if (pose_storage_.size() >= 100) {
            // 标定轮速旋转角速度尺度
            if (!calib_rot_scale_done_) {
                if (calibRotScale()) {
                    calib_rot_scale_done_ = true;
                    std::cout << msa2d::color::GREEN << "轮速旋转尺度标定完成，结果：" << 
                        res_.rot_scale << msa2d::color::RESET << std::endl;
                }
            } 
            if (calib_rot_scale_done_ && calibTransScaleExtrinsic()) {
                return true; 
            }
        }

        return false;
    }

    /**
     * @brief Get the Param object
     * 
     * @return Res 
     */
    Res GetParam() const {
        return res_;
    }

private:
    /**
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    bool calibRotScale() {
        float sum_wheel_rot = 0;
        float sum_laser_rot = 0;
        // 初步计算尺度  
        for (const auto& pose_pair : pose_storage_) {
            sum_laser_rot += std::fabs(pose_pair.first.yaw());
            sum_wheel_rot += std::fabs(pose_pair.second.yaw());  
        }

        res_.rot_scale = sum_laser_rot / sum_wheel_rot;  
        std::cout << "初步尺度计算：" << res_.rot_scale << std::endl;
        // 对尺度进行优化   剔除外点
        sum_laser_rot = 0;
        sum_wheel_rot = 0; 
        float cnt = 0;
        // 进行校验     至少有超过95%的数据误差小与一定阈值
        for (const auto& pose_pair : pose_storage_) {
            float e = std::fabs(res_.rot_scale * std::fabs(pose_pair.second.yaw()) - std::fabs(pose_pair.first.yaw())) 
                                / std::fabs(pose_pair.first.yaw());
            // std::cout << "e: " << e << std::endl;
            // e 至少认为存在高斯噪声   方差为0.01  
            if (e < 0.1) {
                ++cnt;  
                sum_laser_rot += std::fabs(pose_pair.first.yaw());
                sum_wheel_rot += std::fabs(pose_pair.second.yaw());  
            }
        }

        float ratio = cnt / pose_storage_.size();
        std::cout << "ratio: " << ratio << std::endl;

        if (ratio > 0.85) {
            res_.rot_scale = sum_laser_rot / sum_wheel_rot;       // 最终的结果  
            return true;
        }

        return false;
    }

    /**
     * @brief: 最小二乘法 标定 平移尺度和外参  
     */        
    bool calibTransScaleExtrinsic() {
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(4, 4); 
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 1); 
        Eigen::MatrixXd A_k = Eigen::MatrixXd::Zero(2, 4); 
        Eigen::MatrixXd b_k = Eigen::MatrixXd::Zero(2, 1); 

        std::cout<<"标定外参和平移尺度..."<<std::endl;

        for (size_t i = 0; i < pose_storage_.size(); i++) {
            const msa2d::Pose2d &pose_laser = pose_storage_[i].first;
            const msa2d::Pose2d &pose_wheel = pose_storage_[i].second;

            A_k << cos(pose_laser.yaw()) - 1, -sin(pose_laser.yaw()), -pose_wheel.x(), pose_wheel.y(),
                            sin(pose_laser.yaw()),  cos(pose_laser.yaw()) - 1, -pose_wheel.y(), -pose_wheel.x();
            b_k << -pose_laser.x(), -pose_laser.y(); 
            M = M + A_k.transpose() * A_k; 
            B = B + A_k.transpose() * b_k;  
        }
        // 理论上  应该有唯一解     即矩阵M为满秩矩阵   
        // 
        // 通过计算条件数  
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(M);
        // 最大的奇异值 / 最小的奇异值
        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
        std::cout << "最小奇异值：" << svd.singularValues()(svd.singularValues().size() - 1) 
            << "条件数：" << cond << std::endl;
        
        if (cond > max_cond_number_) {
            std::cout << msa2d::color::RED<<"矩阵接近奇异，条件数： "<<cond << std::endl;
            return false;
        }
        // Ay = g --> y = inv(A)g; A square matrix;
        Eigen::VectorXd X = M.colPivHouseholderQr().solve(B);                // QR分解求解AX = B  
        res_.x = X[0];
        res_.y = X[1];
        res_.theta = std::atan2(X[3], X[2]);
        res_.trans_scale = X[3] / std::sin(res_.theta);
        return true;
    }

    Res res_; 
    const double EPSILON_R = 0.05;
    const double ROT_THRESH = 0.17;   // 1度 
    const size_t N_POSE = 200;              // 最多包含的pose个数   
    double rot_cov_thre_;
    // 对pose 按照质量进行排序  
    std::priority_queue<std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>>, 
        std::vector<std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>>>, rotCmp> priority_pose_;
    // pose 数据库中的序号  
    std::vector<std::pair<msa2d::Pose2d, msa2d::Pose2d>> pose_storage_;   

    Eigen::MatrixXd Q_;    // 计算旋转外参时的矩阵  
    Eigen::Quaterniond ext_q_result_;  
    Eigen::Vector2d ext_t_result_;  
    bool calib_rot_scale_done_ = false;

    bool calib_done_ = false; 
    msa2d::Pose2d primary_sensor_accum_pose_, sub_sensor_accum_pose_;
    int max_cond_number_ = 75;  
};
}
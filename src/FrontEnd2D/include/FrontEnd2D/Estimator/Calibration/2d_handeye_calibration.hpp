#pragma once 
#include <utility>
#include <queue>
#include <Eigen/Dense>
#include "msa2d/common/Pose2d.hpp"
#include "msa2d/common/color.hpp"
// 用于优先队列 将旋转小的元素放到前面 
// 小顶堆
struct rotCmp {
    bool operator()(const std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>> &pose_r, 
                                    const std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>> &pose_l) {
        return (std::fabs(pose_l.second.first.yaw()) > std::fabs(pose_r.second.first.yaw()));   
    }
};

/**
 * @brief: 2D POSE 的手眼标定
 */
class HandEyeCalib2D {
    private:
        const double EPSILON_R = 0.05;
        const double ROT_THRESH = 0.17;   // 1度 
        const size_t N_POSE = 200;              // 最多包含的pose个数   
        double rot_cov_thre_;
        // 对pose 按照质量进行排序  
        std::priority_queue<std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>>, 
            std::vector<std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>>>, rotCmp> priority_pose_;
        // pose 数据库中的序号  
        std::queue<std::pair<uint16_t, std::pair<msa2d::Pose2d, msa2d::Pose2d>>> new_pose_pair_;    
        std::vector<std::pair<msa2d::Pose2d, msa2d::Pose2d>> pose_storage_;   

        Eigen::MatrixXd Q_;    // 计算旋转外参时的矩阵  
        Eigen::Quaterniond ext_q_result_;  
        Eigen::Vector2d ext_t_result_;  
        bool calib_done_ = false; 
        msa2d::Pose2d primary_sensor_accum_pose_, sub_sensor_accum_pose_;
        int max_cond_number_ = 75;  

    public:  

        HandEyeCalib2D()
        {
            Q_ = Eigen::MatrixXd::Zero(N_POSE * 4, 4);
            pose_storage_.reserve(N_POSE);  
            rot_cov_thre_ = 0.25;  
        }

        // 添加一组pose数据     pose_laser 中装的是每个传感器的测量值  
        /**
         * @brief 
         * @param pose_primary 主传感器的pose
         * @param pose_sub 辅传感器pose 
         * @brief 添加一组多激光雷达帧间运动数据  
         * @return 本组运动是否合格   
         */
        bool AddPose(const  msa2d::Pose2d& pose_primary, const  msa2d::Pose2d& pose_sub)
        {        
            // 首先检查pose 
            bool check_pose = false;  
            if (!checkScrewMotion(pose_primary, pose_sub)) {
                // ROS_WARN("ScrewMotion error");
                check_pose = false;  
                return check_pose;
            }

            if (pose_storage_.size() < N_POSE) 
            {
                new_pose_pair_.emplace(pose_storage_.size(),     // 保存存储的位置 
                    std::make_pair(primary_sensor_accum_pose_, sub_sensor_accum_pose_));
                priority_pose_.emplace(pose_storage_.size(), 
                    std::make_pair(primary_sensor_accum_pose_, sub_sensor_accum_pose_));  
                pose_storage_.emplace_back(primary_sensor_accum_pose_, sub_sensor_accum_pose_); 
            } else {   // 数据量大于300    则滑动窗口 
                // maintain a min heap
                uint16_t pos = priority_pose_.top().first;  
                std::cout<<"num > 300, remove top theta: "<< priority_pose_.top().second.first.yaw()<<std::endl;
                pose_storage_[pos] = std::make_pair(primary_sensor_accum_pose_, sub_sensor_accum_pose_);  
                new_pose_pair_.emplace(pos, std::make_pair(primary_sensor_accum_pose_, sub_sensor_accum_pose_));
                priority_pose_.pop();   
                priority_pose_.emplace(pos, std::make_pair(primary_sensor_accum_pose_, sub_sensor_accum_pose_));
            }   

            primary_sensor_accum_pose_.SetIdentity();
            sub_sensor_accum_pose_.SetIdentity();  
            std::cout<<msa2d::color::GREEN<<"pose_storage_.size(): "<< pose_storage_.size()<<msa2d::color::RESET<<std::endl;
            if (pose_storage_.size() >= 10)
                check_pose = true;   
            return check_pose;  
        }

        /**
         * @brief: 拉格朗日乘子法  
         */        
        bool CalibMethod1()
        {
            Eigen::MatrixXd M = Eigen::MatrixXd::Zero(5, 5); 
            Eigen::MatrixXd A_k = Eigen::MatrixXd::Zero(2,5); 

            for (size_t i = 0; i < pose_storage_.size(); i++)
            {
                const msa2d::Pose2d &pose_primary = pose_storage_[i].first;
                const msa2d::Pose2d &pose_sub = pose_storage_[i].second;
                // AngleAxisd ang_axis_ref(pose_primary.q_);
                // AngleAxisd ang_axis_data(pose_sub.q_);
                // // 计算指向旋转轴方向的平移差值 
                // double t_dis = abs(pose_primary.t_.dot(ang_axis_ref.axis()) - pose_sub.t_.dot(ang_axis_data.axis()));
                // double huber = t_dis > 0.04 ? 0.04 / t_dis : 1.0;
                A_k << pose_primary.x(), cos(pose_primary.yaw()) - 1, -sin(pose_primary.yaw()), -pose_sub.x(), pose_sub.y(),
                               pose_primary.y(), sin(pose_primary.yaw()),  cos(pose_primary.yaw()) - 1, -pose_sub.y(), -pose_sub.x();
                M = M + A_k.transpose() * A_k; 
            }

            Eigen::VectorXd X = solveProblem(M);
            std::cout<<"solve X: "<<X.transpose()<<std::endl;
            return true;
        }

        /**
         * @brief: 最小二乘法 
         */        
        bool CalibMethod2()
        {
            Eigen::MatrixXd M = Eigen::MatrixXd::Zero(4, 4); 
            Eigen::MatrixXd B = Eigen::MatrixXd::Zero(4, 1); 

            Eigen::MatrixXd A_k = Eigen::MatrixXd::Zero(2, 4); 
            Eigen::MatrixXd b_k = Eigen::MatrixXd::Zero(2, 1); 

            std::cout<<"Calib !"<<std::endl;

            for (size_t i = 0; i < pose_storage_.size(); i++)
            {
                const msa2d::Pose2d &pose_primary = pose_storage_[i].first;
                const msa2d::Pose2d &pose_sub = pose_storage_[i].second;

                A_k << cos(pose_primary.yaw()) - 1, -sin(pose_primary.yaw()), -pose_sub.x(), pose_sub.y(),
                                sin(pose_primary.yaw()),  cos(pose_primary.yaw()) - 1, -pose_sub.y(), -pose_sub.x();
                b_k << -pose_primary.x(), -pose_primary.y(); 
                M = M + A_k.transpose() * A_k; 
                B = B + A_k.transpose() * b_k;  
            }
            // 理论上  应该有唯一解     即矩阵M为满秩矩阵   
            // 通过计算条件数  
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(M);
            // 最大的奇异值 / 最小的奇异值
            double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
            
            if (cond > max_cond_number_) {
                std::cout << msa2d::color::RED<<"矩阵接近奇异，条件数： "<<cond << std::endl;
                return false;
            }
            // Ay = g --> y = inv(A)g; A square matrix;
            Eigen::VectorXd X = M.colPivHouseholderQr().solve(B);                // QR分解求解AX = B  
          
            std::cout<<"solve X: "<<X.transpose()<<", tan(yaw): "<<X[3] / X[2]<<std::endl;
            return true;
        }

        bool GetCalibResult(msa2d::Pose2d &result) {
            if (calib_done_) {
                result.SetX(ext_t_result_[0]); 
                result.SetY(ext_t_result_[1]); 
                Eigen::Vector3d eulerAngle = ext_q_result_.toRotationMatrix().eulerAngles(2,1,0);    // 顺序 zyx 
                std::cout<<"ext_q_result_ eulerAngle: "<<eulerAngle.transpose()<<std::endl;
                result.SetRotation(eulerAngle[0]);
                return true;
            }
            return false;  
        }

    protected:

        /**
         * @brief: 罚函数法   
         * @details: TODO: 调试不成功 ！！！
         * @param {int const&} iterater
         * @param {MatrixXd const&} H
         * @param {MatrixXd const&} b
         * @return {*}
         */        
        Eigen::VectorXd SolvePenaltyFunction(int const& iterater, Eigen::MatrixXd const& H, 
                Eigen::MatrixXd const& b) {
            float lamda = 0.1;
            float beta = 5;
            float thresh = 0.05; 
            Eigen::MatrixXd W = Eigen::MatrixXd::Zero(4, 4); 
            W(2, 2) = 1;
            W(3, 3) = 1;

            for (int i = 0; i < iterater; i++) {
                // 求解  (ATA+ lamda * W) x = ATb  的最小而乘解
                Eigen::MatrixXd A = H + lamda * W;
                // Verify that A isn't singular
                // 即A是否接近奇异    否则 存在零空间，y 变化一点 ，造成X剧烈的变化
                // 通过计算条件数  
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
                // 最大的奇异值 / 最小的奇异值
                double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
                std::cout<<"A cond: "<<cond<<", A norm: "<<A.norm()<<std::endl;

                if (cond > max_cond_number_) {
                    std::cout << msa2d::color::RED<<"矩阵接近奇异，条件数： "<<cond << std::endl;
                    return Eigen::VectorXd(4);
                }
                // Ay = g --> y = inv(A)g; A square matrix;
                Eigen::VectorXd y = A.colPivHouseholderQr().solve(b);                // QR分解求解AX = B  
                // 求解是否满足可行域
                double v = y.transpose() * W * y;
                std::cout<<"v: "<<v<<std::endl;

                if (std::fabs(v - 1) < thresh) {
                    std::cout<<msa2d::color::GREEN<<"calib OK !!"<<std::endl;
                    return y;  
                }
                lamda *= beta;  
            }

            return Eigen::VectorXd(4);
        }

        /**
         * @brief: 求解带有约束的二次规划问题 
         * @details: 拉格朗日乘子法 
         * @param M 信息矩阵
         * @return 求解的解向量 
         */        
        Eigen::VectorXd solveProblem(Eigen::MatrixXd const& M) {   
            // step 1: 求解 lamda 使得 M + lamda * W 奇异 
            double m11 = M(0, 0);
            double m13 = M(0, 2);
            double m14 = M(0, 3);
            double m15 = M(0, 4);
            double m22 = M(1, 1);
            // double m25 = M(1, 4);
            double m34 = M(2, 3);
            double m35 = M(2, 4);
            double m44 = M(3, 3);
            // double m55 = M(4, 4);
            double a, b, c;

            a = m11 * pow(m22,2) - m22 * pow(m13,2);
            b = 2 * m13 * m22 * m35 * m15 - pow(m22,2) * pow(m15,2) - 2 * m11 * m22 * pow(m35, 2)
                + 2 * m13 * m22 * m34 * m14 - 2 * m22 * pow(m13,2) * m44 - pow(m22,2) * pow(m14,2)
                + 2 * m11 * pow(m22,2) * m44 + pow(m13,2) * pow(m35,2) - 2 * m11 * m22 * pow(m34,2)
                + pow(m13,2) * pow(m34,2);
            c = -2 * m13 * pow(m35, 3) * m15 - m22 * pow(m13,2) * pow(m44,2) + m11 * pow(m22,2) * pow(m44,2)
                + pow(m13,2) * pow(m35,2) * m44 + 2 * m13 * m22 * m34 * m14 * m44
                + pow(m13,2) * pow(m34,2) * m44 - 2 * m11 * m22 * pow(m34,2) * m44
                - 2 * m13 * pow(m34,3) * m14 - 2 * m11 * m22 * pow(m35,2) * m44
                + 2 * m11 * pow(m35,2) * pow(m34,2) + m22 * pow(m14,2) * pow(m35,2)
                - 2 * m13 * pow(m35,2) * m34 * m14 - 2 * m13 * pow(m34, 2) * m35 * m15
                + m11 * pow(m34,4) + m22 * pow(m15,2) * pow(m34,2)
                + m22 * pow(m35,2) * pow(m15,2) + m11 * pow(m35,4)
                - pow(m22,2) * pow(m14,2) * m44 + 2 * m13 * m22 * m35 * m15 * m44
                + m22 * pow(m34,2) * pow(m14,2) - pow(m22,2) * pow(m15,2) * m44;
            // 二次方程有实根
            if ((pow(b,2) - 4 * a * c) >= 0) {
                double r0 = (-b - sqrt(pow(b,2) - 4 * a * c)) / (2 * a);
                double r1 = (-b + sqrt(pow(b,2) - 4 * a * c)) / (2 * a);

                Eigen::MatrixXd W = Eigen::MatrixXd::Zero(5, 5);
                W(3,3) = 1;
                W(4,4) = 1;
                Eigen::VectorXd x0 = solveX(M, r0, W);
                Eigen::VectorXd x1 = solveX(M, r1, W);

                double e0 = calculateError(x0, M);
                double e1 = calculateError(x1, M);

                return e0 < e1 ? x0 : x1;
            }
            else {
                std::cout <<msa2d::color::RED<< "lamda 无零解, 问题无非零解！！！"<<msa2d::color::RESET << std::endl;
                return Eigen::VectorXd(5);
            }
        }

        /**
         * @brief: 通过最小二乘求解X   
         * @details: 
         * @return X 
         */        
        Eigen::VectorXd solveX(Eigen::MatrixXd const& M, double const& lambda, Eigen::MatrixXd const& W) {
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(5,5);
            Eigen::MatrixXd ATA = Eigen::MatrixXd::Zero(5,5);

            A = M + lambda * W;
            // 求解 AX = 0的最小二乘解    即 ATA 最小特征值对应的特征向量   
            ATA = A.transpose() * A;
            // 特征分解 
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(ATA);

            Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
            Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();
            // 获取最小特征值对应的特征向量 
            Eigen::VectorXd v0 = eigenvectors.col(eigenvalues.minCoeff());
            Eigen::Vector2d tmp_v = Eigen::Vector2d::Zero(2);
            tmp_v(0) = v0(3);
            tmp_v(1) = v0(4);
            double norm = tmp_v.norm();
            double coeff = (v0(0) >= 0 ? 1 : -1) / norm;
            v0 = coeff * v0;
            return v0;
        }

        double calculateError(const Eigen::VectorXd &x, const Eigen::MatrixXd &M) {
            double error;
            Eigen::VectorXd tmp = Eigen::VectorXd::Zero(x.rows());
            tmp = M * x;
            error = x.transpose() * tmp;    // xT *M * x 

            return error;
        }


        /**
         * @brief: 检查运动是否符合条件 
         * @details: 
         * @param {Isometry3d} &pose_primary
         * @param {Isometry3d} &pose_sub
         * @return {*}
         */            
        bool checkScrewMotion(const msa2d::Pose2d &pose_primary, const msa2d::Pose2d &pose_sub) {
            // 先检查当前数据是否正确
            //  检查旋转  
            //  刚性连接的传感器   旋转角度应该相同
            double r_dis = abs(pose_primary.yaw() - pose_sub.yaw());   
            // 表明误差太大  或 着本次运动的旋转太小
            //if ((r_dis > EPSILON_R) || abs(pose_primary.yaw_) < 0.01)
            if ((r_dis > EPSILON_R)) {
                std::cout<<"pose_primary.yaw_: "<<pose_primary.yaw()<<" ,pose_sub.yaw_"
                <<pose_sub.yaw()<<std::endl;
                // ROS_WARN("r_dis > EPSILON_R !!");
                primary_sensor_accum_pose_.SetIdentity();
                sub_sensor_accum_pose_.SetIdentity();  
                return false;
            }
            // 如果旋转不够的话   则进行累计直到旋转足够  
            primary_sensor_accum_pose_ = primary_sensor_accum_pose_ * pose_primary;
            sub_sensor_accum_pose_ = sub_sensor_accum_pose_ * pose_sub; 

            // std::cout<<"pose_primary x: "<<pose_primary.x()<<", y: "<<pose_primary.y()
            // <<", orientation_: "<<pose_primary.orientation().coeffs().transpose()<<std::endl;
            // std::cout<<"pose_sub x: "<<pose_sub.x()<<", y: "<<pose_sub.y()
            // <<", orientation_: "<<pose_sub.orientation().coeffs().transpose()<<std::endl;

            // 旋转累计到一定大小时
            if (std::fabs(primary_sensor_accum_pose_.yaw())> ROT_THRESH 
                    || std::fabs(sub_sensor_accum_pose_.yaw()) > ROT_THRESH) {
                // 位移太大则  不考虑 
                if (primary_sensor_accum_pose_.x() * primary_sensor_accum_pose_.x()
                        + primary_sensor_accum_pose_.y() * primary_sensor_accum_pose_.y() > 1) {
                    primary_sensor_accum_pose_.SetIdentity();
                    sub_sensor_accum_pose_.SetIdentity();  
                    std::cout<<msa2d::color::YELLOW<<"move too long"<<std::endl;
                    return false;
                }

                // std::cout<<"primary_sensor_accum_pose_ orientation_: "<<primary_sensor_accum_pose_.orientation().coeffs().transpose()<<std::endl;
                // std::cout<<"sub_sensor_accum_pose_ orientation_: "<<sub_sensor_accum_pose_.orientation().coeffs().transpose()<<std::endl;
                return true;  
            }
            return false;  
        }
};


#include <iomanip> 
#include "FrontEnd2D/Estimator/estimator.hpp"
#include "msa2d/ScanMatcher/fast_correlative_scan_matcher_2d.h"

namespace Estimator2D {

FrontEndEstimator::FrontEndEstimator(const std::string& config_path) {
    YAML::Node yaml = YAML::LoadFile(config_path);
    /* 构建初始地图 */
    msa2d::map::OccGridMapPyramid::Option grid_map_pyramid_option;
    grid_map_pyramid_option.bottom_resolution = 
        yaml["grid_map_pyramid"]["bottom_resolution"].as<float>();
    grid_map_pyramid_option.num_depth = 
        yaml["grid_map_pyramid"]["map_depth"].as<float>();
    grid_map_pyramid_option.map_sizeX = 
        yaml["grid_map_pyramid"]["map_size"]["x"].as<float>();   // 真实的物理尺寸  单位 m
    grid_map_pyramid_option.map_sizeY = 
        yaml["grid_map_pyramid"]["map_size"]["y"].as<float>();   // 真实的物理尺寸  单位 m
    grid_map_pyramid_option.grid_update_method = 
        yaml["grid_map_pyramid"]["grid_update_method"].as<std::string>();
    grid_map_pyramid_option.min_distance_to_boundary = 
        yaml["sensor"]["laser"]["valid_distance"].as<float>();

    grid_map_pyramid_ = new msa2d::map::OccGridMapPyramid(grid_map_pyramid_option);
    std::cout << "OccGridMapPyramid create done" << std::endl;
    msa2d::ScanMatcher::hectorScanMatcher::Option option; 
    hector_matcher_ = new msa2d::ScanMatcher::hectorScanMatcher(option);
    std::cout << "ScanMatcher create done" << std::endl;
    laser_pyramid_container_.resize(grid_map_pyramid_->getMapLevels());
    this->reset();
    /* 设置进行地图更新的位姿变化阈值 **/
    this->SetMapUpdateMinDistDiff(0.4f * 1.0f);
    this->SetMapUpdateMinAngleDiff(0.13f * 1.0f);
    // 读取laser-imu外参
    Eigen::Matrix<float, 6, 1> laser_odom_ext;
    laser_odom_ext.setZero();  
    prime_laser_upside_down_ = yaml["extrinsic"]["odom"]["prime_laser"]["upside_down"].as<bool>();
    
    if (prime_laser_upside_down_) {
        std::cout << "主激光雷达颠倒! "<< std::endl;
        laser_odom_ext(3, 0) = 3.1415926;     // 绕x轴转180度  
    }

    laser_odom_ext(0, 0) = yaml["extrinsic"]["odom"]["prime_laser"]["level"][0].as<float>();  // x
    laser_odom_ext(1, 0) = yaml["extrinsic"]["odom"]["prime_laser"]["level"][1].as<float>();  // y
    laser_odom_ext(5, 0) = yaml["extrinsic"]["odom"]["prime_laser"]["level"][2].as<float>();  // theta
    
    ::util::DataDispatcher::GetInstance().Publish("lidarOdomExt", laser_odom_ext);
    // primeLaserOdomExtrinsic_ = util::get3DTransformForState(laser_odom_ext);
    primeLaserOdomExtrinsic_ = msa2d::convert::getTransformForState(
        Eigen::Vector3f({laser_odom_ext(0, 0), laser_odom_ext(1, 0), laser_odom_ext(5, 0)}));
    // kalman set
    ekf_estimate_enable_ = yaml["kalman_filter"]["enable"].as<bool>(); 

    if (ekf_estimate_enable_) {
        std::cout << "使用卡尔曼滤波! "<< std::endl;
    } else {
        std::cout << "关闭卡尔曼滤波! "<< std::endl;
    }
    
    run_ = std::thread(&FrontEndEstimator::run, this); 
}

/**
 * @brief 
 * 
 * @param laser_ptr 
 */
void FrontEndEstimator::InputLaser(msa2d::sensor::LaserScan::ptr& laser_ptr) {
    laser_sm_.lock();   // 写锁
    laser_cache_.push_back(std::move(laser_ptr));
    laser_sm_.unlock(); 
}

/**
 * @brief 
 * 
 * @param data 
 */
void FrontEndEstimator::InputWheelOdom(msa2d::sensor::WheelOdom& data) {
    static DiffModelDeadReckoning diff_model; 
    wheel_sm_.lock();   // 写锁 
    wheelOdom_cache_.push_back(data);
    wheel_sm_.unlock();  
    // // 检测laser是否有数据， 若laser 没数据， 则退化为 odom/imu融合里程计
    // wheel_sm_.lock_shared();    // 读锁
    // if (laser_cache_.empty()) {
    //     if (wheelOdom_cache_.back().time_stamp_ - wheelOdom_cache_.front().time_stamp_ > 0.5) {
    //         model_ = MODEL::WHEEL_IMU;
    //         std::cout << "INFO: 没有激光数据，切换到wheel + imu 模式..." << std::endl; 
    //         // 进行初始化
    //     }
    // } else {
    //     if (wheelOdom_cache_.back().time_stamp_ - laser_cache_.back()->start_time_ > 0.5) {
    //         model_ = MODEL::WHEEL_IMU;
    //     }
    // }
    // wheel_sm_.unlock_shared();
    // // 若为imu/wheel融合 - wheel提供线速度，imu提供角速度形成组合里程计，此外在线估计陀螺仪bias
    // if (model_ == MODEL::WHEEL_IMU) {
    // }
}

/**
 * @brief 
 * 
 * @param data 
 */
void FrontEndEstimator::InputImu(msa2d::sensor::ImuData& data) {
    imu_sm_.lock();
    imu_cache_.push_back(std::move(data));
    imu_sm_.unlock();  
}

/**
 * @brief 估计器主线程
 */
void FrontEndEstimator::run() {
    uint16_t wait_time = 0;
    bool imu_used_ = false;
    bool wheel_used_ = false;  
    std::deque<msa2d::sensor::WheelOdom> wheel_odom_selected; 
    std::deque<msa2d::sensor::ImuData> imu_selected;  

    while (1) {
        std::shared_lock<std::shared_mutex> s_l(laser_sm_);   // 读锁
        if (laser_cache_.size()) {
            auto& curr_laser_ptr = laser_cache_.front();   // 之前的push_back并不会使该引用失效
            s_l.unlock(); 
            ++wait_time; 
            // 提取出包围laser的odom和imu数据 
            if (syncSensorData(curr_laser_ptr, wheel_odom_selected, imu_selected) || wait_time > 10) {
                // 自动检测传感器是否在线 
                // 当 wait_time > 10 认为有传感器离线了
                if (wait_time > 10) {
                    if (!wheelOdom_cache_.empty() &&
                            wheelOdom_cache_.back().time_stamp_ < curr_laser_ptr->end_time_) {
                        wheelOdom_cache_.clear();  
                        std::cout << msa2d::color:: RED << "---------------------wheel data loss !-----------------------" 
                            << msa2d::color::RESET << std::endl; 
                    }
                    if (!imu_cache_.empty() &&
                                imu_cache_.back().time_stamp_ < curr_laser_ptr->end_time_) {
                        imu_cache_.clear();  
                        std::cout << msa2d::color:: RED << "---------------------imu data loss !-----------------------" 
                            << msa2d::color::RESET << std::endl; 
                    }
                }
                wait_time = 0; 
                // 如果位姿丢失   需要重定位 
                // 重定位失败则新建一个轨迹  
                if (track_loss_) {
                    std::cout << msa2d::color::RED << "tracking loss !!!!" << 
                        msa2d::color::RESET << std::endl;
                    // 判断是否静止

                    // laser_sm_.lock();
                    // laser_cache_.pop_front();  
                    // laser_sm_.unlock();  
                    // wheel_odom_selected.clear(); 
                    // imu_selected.clear();  
                    // continue;  
                }

                static int global_match_time = 0;  
                if (global_match_time == 10) {
                    msa2d::ScanMatcher::FastCorrelativeScanMatcherOptions2D fcsm_option;
                    // msa2d::ScanMatcher::PrecomputationGridStack2D  pre_grid_stack(grid_map_pyramid_->getGridMap(0), fcsm_option);
                    
                    // for (int h = 0; h <= pre_grid_stack.max_depth(); ++h) {
                    // // for (int h = 0; h <= 0; ++h) {
                    //     cv::Mat map_img = pre_grid_stack.Get(h).ToImage();
                    //     cv::imshow("map_" + std::to_string(h), map_img);
                    //     cv::waitKey(10);  
                    // }
                    msa2d::ScanMatcher::FastCorrelativeScanMatcher2D global_matcher(grid_map_pyramid_->getGridMap(0), fcsm_option);
                    float score = 0.f;
                    msa2d::Pose2d global_pose_estimate;
                    global_matcher.MatchFullSubmap(curr_laser_ptr->pointcloud_, 0, score, global_pose_estimate); 
                    // std::cout << "score: " << score << ", global_pose_estimate x: " << global_pose_estimate.x()
                    //     << ", y: " << global_pose_estimate.y() << ",yaw: " << global_pose_estimate.yaw() << std::endl; 
                    global_pose_estimate.SetVec(
                        grid_map_pyramid_->getGridMap(0)->getGridMapBase().PoseMapToWorld(global_pose_estimate.vec()));
                    std::cout << "global_pose_estimate: " << global_pose_estimate.vec().transpose() << std::endl;
                    global_match_time = 0;  
                    ::util::DataDispatcher::GetInstance().Publish("global_pose", global_pose_estimate); 
                    ::util::DataDispatcher::GetInstance().Publish("undistorted_pointcloud", *curr_laser_ptr); 
                }
                global_match_time++; 
                /**
                 * @brief 进行EKF预测   
                 * aser-imu-wheel模式： imu预测+laser&wheel校正
                 * imu-laser模式：imu预测+laser校正
                 * wheel-laser模式：wheel预测+laser校正
                 * 纯laser模式：运动模型预测+laser校正
                 */
                DiffModelDeadReckoning diff_model;  // 差分模型
                bool use_motion_model_predict = true;  
            
                if (imu_selected.empty()) {
                    // 如果 有轮速数据，且odom与激光的外参已知  那么用轮速进行预测
                    if (!wheel_odom_selected.empty()) {
                        if (has_odomExtrinsicParam_) {
                            std::cout << msa2d::color::GREEN << "使用轮速计预测 ..." << msa2d::color::RESET << std::endl; 
                            use_motion_model_predict = false;  
                            // 使用轮速进行预测
                            for (uint16_t i = 0; i < wheel_odom_selected.size(); ++i) {
                                const auto& curr_data = wheel_odom_selected[i]; 
                                // 运动学前向传播
                                diff_model.Update(curr_data.time_stamp_, curr_data.v_x_, curr_data.omega_yaw_); 
                                // EKF预测
                                /**
                                 * @brief 对于轮速 需要考虑优化外参和不优化外参
                                 */
                                if (ekf_estimator_ != nullptr) {
                                    ekf_estimator_->Predict(diff_model.ReadLastPose().pose_, curr_data.v_x_, 
                                        curr_data.omega_yaw_, 0.0025, 1, curr_data.time_stamp_);    // 测量噪声 0.05m/s, 60度/s
                                }
                            }
                        }
                    }
                } else {
                    // 如果IMU初始化了
                    if (imu_initialized_) {
                        // 若使用EKF滤波器  
                        if (ekf_estimator_ != nullptr) {
                            std::cout << msa2d::color::GREEN << "EKF ------- IMU预测..." << msa2d::color::RESET << std::endl;
                        } else {
                            std::cout << msa2d::color::GREEN << "运动学 ------- IMU预测..." << msa2d::color::RESET << std::endl;
                        }
                        use_motion_model_predict = false;   
                        // 使用IMU的角速度
                        for (uint16_t i = 0; i < imu_selected.size(); ++i) {
                            const auto& curr_data = imu_selected[i]; 
                            // EKF预测
                            if (ekf_estimator_ != nullptr) {
                                ekf_estimator_->Predict(curr_data.angular_v_[2], 0.25, curr_data.time_stamp_, diff_model);  
                            } else {
                                float un_bias_yaw_angular_v = 
                                    imu_coeff_ * (curr_data.angular_v_[2] - imu_initializer_.GetYawBias());  // 去除bias 
                                // 不使用EKF时，直接用IMU估计运动(这里假设线速度为0，只考虑旋转)
                                diff_model.Update(curr_data.time_stamp_, 0, un_bias_yaw_angular_v); 
                            }
                        }      
                    } else {
                        std::cout << msa2d::color::YELLOW << "IMU未初始化 ..." << msa2d::color::RESET << std::endl;
                    }
                }
                // 在没有IMU以及轮速的情况下  会使用运动学模型预测
                if (use_motion_model_predict) {
                    if (ekf_estimator_ != nullptr) {
                        std::cout << msa2d::color::GREEN << "EKF ------- 先验运动学预测.." 
                            << msa2d::color::RESET << std::endl;
                        ekf_estimator_->Predict(curr_laser_ptr->start_time_, curr_laser_ptr->end_time_, diff_model);
                    }
                } 
                msa2d::Pose2d predict_incre_pose;   // 预测帧间的运动 
                predict_incre_pose = diff_model.ReadLastPose().pose_;
                // std::cout << msa2d::color::YELLOW << "predict_incre_pose: " << predict_incre_pose.vec().transpose()
                //     << msa2d::color::RESET << std::endl;
                if (imu_calib_) {
                    LaserUndistorted(*curr_laser_ptr, diff_model.GetPath());// 去除laser的畸变
                }
                msa2d::Pose2d estimated_odom_pose = last_fusionOdom_pose_.pose_ * predict_incre_pose;   // To<-last * Tlast<-curr
                // 将odom系下的预测位姿转换到laser odom下
                msa2d::Pose2d new_estimate_laserOdom_pose;
                poseOdomToPrimeLaserOdom(estimated_odom_pose, new_estimate_laserOdom_pose);
                // 基于栅格金字塔快速检测预测是否可靠，判断是否存在里程计打滑等现象
                // fastPredictFailureDetection(curr_laser_ptr, new_estimate_laserOdom_pose, grid_map_pyramid_->getGridMap(1)); 
                // std::cout << "odom predict: " << estimated_odom_pose.vec().transpose() << std::endl;
                // 将点云数据转换为激光多分辨率金字塔数据
                getLaserPyramidData(*curr_laser_ptr);     // 构造 laser_pyramid_container_ 
                // tt.toc("getGridPyramidData");
                TicToc tt; 
                new_estimate_laserOdom_pose.SetVec(hector_matcher_->Solve(new_estimate_laserOdom_pose.vec(), 
                    laser_pyramid_container_, *grid_map_pyramid_, lastScanMatchCov)); 
                std::cout << "laserOdom_pose: " << new_estimate_laserOdom_pose.vec().transpose() << std::endl;
                // 细匹配  ICP / NDT 
                tt.toc("match solve");
                // 将pose转换到odom系下
                posePrimeLaserToOdom(new_estimate_laserOdom_pose, estimated_odom_pose);
                std::cout << "Odom_pose: " << estimated_odom_pose.vec().transpose() << std::endl;
                // msa2d::Pose2d scan_matched_incre_pose = last_fusionOdom_pose_.pose_.inv() * estimated_odom_pose;
                // std::cout << msa2d::color::YELLOW << "scan_matched_incre_pose: " << scan_matched_incre_pose.vec().transpose()
                //     << msa2d::color::RESET << std::endl;
                // 校准IMU
                if (!imu_calib_) {
                    msa2d::Pose2d scan_matched_incre_pose = 
                        last_fusionOdom_pose_.pose_.inv() * estimated_odom_pose;
                    if (std::fabs(predict_incre_pose.yaw()) > 0.02) {
                        float t = predict_incre_pose.yaw() * scan_matched_incre_pose.yaw();
                        if (t < 0) {
                            imu_coeff_ = - imu_coeff_; 
                        }
                        imu_calib_ = true;  
                    }
                } 
          
                // 在IMU&odom初始化前，这里执行的是基于先验运动模型的EKF
                if (ekf_estimate_enable_ && ekf_estimator_ != nullptr) {
                    // 观测协方差矩阵
                    Eigen::Matrix3f obs_cov = Eigen::Matrix3f::Zero();
                    obs_cov(0, 0) = 0.0001;    // x方向方差   0.01 * 0.01
                    obs_cov(1, 1) = 0.0001;    // y 方向方差 0.01 * 0.01
                    obs_cov(2, 2) = 0.000003;    // yaw方差   0.1度
                    // 进行观测校正
                    ekf_estimator_->Correct(estimated_odom_pose, obs_cov, curr_laser_ptr->end_time_); 
                    estimated_odom_pose = ekf_estimator_->ReadPosterioriPose();
                    // 更新对应laser坐标
                    poseOdomToPrimeLaserOdom(estimated_odom_pose, new_estimate_laserOdom_pose);
                }
                // 系统初始化是指 联合imu和wheel的多传感器初始化
                // 系统初始化只要有imu或wheel两者之一就可以，系统初始化后，
                // 即使之后出现新的传感器数据，也不会重新初始化
                if (!system_initialized_) {
                    systemInit(new_estimate_laserOdom_pose, imu_selected, wheel_odom_selected, curr_laser_ptr->end_time_);
                }

                // 利用栅格金子塔进行点云分类
                std::pair<std::vector<Eigen::Vector2f>, double> dynamic_points_data;    // 动态点
                std::pair<std::vector<Eigen::Vector2f>, double>  stable_points_data;   // 静态点 
                std::pair<std::vector<Eigen::Vector2f>, double>  undetermined_points_data;  // 待定点
                dynamic_points_data.first.reserve(curr_laser_ptr->pointcloud_.size());
                dynamic_points_data.second = curr_laser_ptr->end_time_; 
                // dynamic_points_data.second = curr_laser_ptr->start_time_; 
                stable_points_data.first.reserve(curr_laser_ptr->pointcloud_.size());  
                stable_points_data.second = curr_laser_ptr->end_time_; 
                // stable_points_data.second = curr_laser_ptr->start_time_; 
                undetermined_points_data.first.reserve(curr_laser_ptr->pointcloud_.size());  
                undetermined_points_data.second = curr_laser_ptr->end_time_; 
                //  将激光点从激光坐标系转到laserOdom系 
                for (auto& point : curr_laser_ptr->pointcloud_.points()) {
                    point.pos_ = new_estimate_laserOdom_pose * point.pos_;      //  转到laserOdom系 
                }

                pointClassification(curr_laser_ptr, new_estimate_laserOdom_pose, grid_map_pyramid_->getGridMap(0), 
                    dynamic_points_data.first, stable_points_data.first, undetermined_points_data.first);

                last_fusionOdom_pose_.pose_ = estimated_odom_pose;
                last_fusionOdom_pose_.time_stamp_ = curr_laser_ptr->end_time_; 
                // last_fusionOdom_pose_.time_stamp_ = curr_laser_ptr->start_time_; 
                // 发布数据
                ::util::DataDispatcher::GetInstance().Publish("fusionOdom", last_fusionOdom_pose_); 
                ::util::DataDispatcher::GetInstance().Publish("dynamic_pointcloud", dynamic_points_data); 
                ::util::DataDispatcher::GetInstance().Publish("stable_pointcloud", stable_points_data); 
                ::util::DataDispatcher::GetInstance().Publish("local_map", local_map_.ReadLocalMap()); 

                // 判断位姿是否丢失
                if (!track_loss_ && !judgeTrackingLoss(dynamic_points_data.first, stable_points_data.first, 
                                                                undetermined_points_data.first)) {
                    /** 2.地图更新(last_map_updata_pose_初始化为一个很大的值，因此第一帧点云就会更新地图) **/
                    if (msa2d::module::poseDifferenceLargerThan(new_estimate_laserOdom_pose.vec(), last_map_updata_pose_.vec(), 
                            paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate)) { 
                        // 仅在位姿变化大于阈值 或者 map_without_matching为真 的时候进行地图更新
                        grid_map_pyramid_->updateByScan(laser_pyramid_container_, new_estimate_laserOdom_pose.vec());
                        grid_map_pyramid_->onMapUpdated();
                        local_map_.UpdateLocalMapForMotion(stable_points_data.first);
                        last_map_updata_pose_ = new_estimate_laserOdom_pose;
                    }
                }

                laser_sm_.lock();
                laser_cache_.pop_front();  
                laser_sm_.unlock();  
                wheel_odom_selected.clear(); 
                imu_selected.clear();  
                // tt.toc("map update");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

/**
 * @brief 
 * 
 * @param laser_ptr 
 * @param pose 
 * @param occGridMap 
 * @return true 
 * @return false 
 */
bool FrontEndEstimator::fastPredictFailureDetection(const msa2d::sensor::LaserScan::Ptr& laser_ptr, 
                                                                                                                const msa2d::Pose2d& pose, 
                                                                                                                const msa2d::map::OccGridMapBase* occGridMap) {
    uint16_t laser_num = laser_ptr->pointcloud_.size();
    float incre_step = 1.0;
    if (laser_num > 100) {
        incre_step = laser_num * 0.01;  
    } 
    // std::cout << "incre_step: " << incre_step << "laser_num: " << laser_num << std::endl;
    uint16_t unknow_point_num = 0;
    uint16_t inlier_point_num = 0;
    uint16_t outlier_point_num = 0;

    for (float curr_point = 0; curr_point < laser_num - 1; curr_point += incre_step ) {
        Eigen::Vector2f point_laserOdom_pos = pose * laser_ptr->pointcloud_[int(curr_point)].pos_;     
        Eigen::Vector2f point_gridmap_pos = occGridMap->getGridMapBase().PosWorldToMapf(point_laserOdom_pos);     // laserOdom -> map

        if (!occGridMap->getGridMapBase().pointOutOfMapBounds(point_gridmap_pos)) {
            if (occGridMap->isOccupied(round(point_gridmap_pos[0]), round(point_gridmap_pos[1]))) {
                inlier_point_num++;
            } else if (occGridMap->isFree(round(point_gridmap_pos[0]), round(point_gridmap_pos[1]))) {
                outlier_point_num++;
            } else {
                unknow_point_num++;
            }
        }
    }

    // std::cout << msa2d::color::YELLOW << "unknow_point_num: " << unknow_point_num << std::endl;
    // std::cout << msa2d::color::YELLOW << "inlier_point_num: " << inlier_point_num << std::endl;
    // std::cout << msa2d::color::YELLOW << "outlier_point_num: " << outlier_point_num << std::endl;
    return false; 
}

/**
 * @brief 
 * 
 * @param laser_ptr 
 * @param pose 
 * @param occGridMap 
 * @param dynamic_points 
 * @param stable_points 
 * @param undetermined_points 
 */
void FrontEndEstimator::pointClassification(const msa2d::sensor::LaserScan::Ptr& laser_ptr, 
                                                                                            const msa2d::Pose2d& pose, 
                                                                                            const msa2d::map::OccGridMapBase* occGridMap,
                                                                                            std::vector<Eigen::Vector2f>& dynamic_points,
                                                                                            std::vector<Eigen::Vector2f>& stable_points,
                                                                                            std::vector<Eigen::Vector2f>& undetermined_points) {
    for (auto& point : laser_ptr->pointcloud_) {
        Eigen::Vector2f point_gridmap_pos = occGridMap->getGridMapBase().PosWorldToMapf(point.pos_);     // laserOdom -> map
        // 超过范围的点一律丢掉
        if (occGridMap->getGridMapBase().pointOutOfMapBounds(point_gridmap_pos)) {
            continue;      
        }
        uint16_t grid_x = round(point_gridmap_pos[0]);
        uint16_t grid_y = round(point_gridmap_pos[1]); 

        if (occGridMap->isFree(grid_x, grid_y)) {
            if (occGridMap->isFree(grid_x + 1, grid_y) && 
                    occGridMap->isFree(grid_x - 1, grid_y) && 
                    occGridMap->isFree(grid_x, grid_y + 1) &&
                    occGridMap->isFree(grid_x, grid_y - 1)) {
                // 检测该栅格前后左右四个方向，如果都为空，则该点是动态点，若至少有一个非空，则直接忽略
                dynamic_points.push_back(point.pos_);  
                continue;  
            } 
        } 
        if (occGridMap->isOccupied(grid_x, grid_y)) {
            stable_points.push_back(point.pos_);  // 在一定距离内  且静态的点叫稳定点
        } else {
            undetermined_points.push_back(point.pos_);  
        }
    }
}

/**
 * @brief 判断位姿是否的丢失 
 *                  判断动态点和未知点是否突然增多，如果增加的比例大于阈值，那么认为位姿跟踪丢失 
 * @param dynamic_points 
 * @return true 位姿丢失
 * @return false  位姿良好
 */
bool FrontEndEstimator::judgeTrackingLoss(const std::vector<Eigen::Vector2f>& dynamic_points,
                                                                                            const std::vector<Eigen::Vector2f>& stable_points,
                                                                                            const std::vector<Eigen::Vector2f>& undetermined_points) {
    static float last_track_quality_factor = -1;
    float curr_track_quality_factor = (float)stable_points.size() / (dynamic_points.size() + undetermined_points.size()); 
    // std::cout << "dynamic_points size: " << dynamic_points.size() <<
    //     "stable_points size: " << stable_points.size() << 
    //     "undetermined_points size: " << undetermined_points.size() << std::endl;
    // 初始化
    if (last_track_quality_factor < 0) {
        if (stable_points.size() == 0) {
            return false;  
        }
        // 点的质量必须要良好才能初始化
        if (curr_track_quality_factor > 1) { 
            last_track_quality_factor = curr_track_quality_factor;
            return false;
        } else {
            track_loss_ = true;
            return true;  
        }
    }
    std::cout << "last_track_quality_factor: " << last_track_quality_factor << ",curr_track_quality_factor: "
        << curr_track_quality_factor << std::endl;
    // 质量
    if (curr_track_quality_factor < last_track_quality_factor * 0.1) {
        track_loss_ = true;  
        return true; 
    }
    last_track_quality_factor = curr_track_quality_factor;
    return false;
}

/**
 * @brief 根据运动信息去除激光点云的畸变  
 * 
 * @param laser 
 * @param motion_info 该激光帧时间范围内的运行信息  但是这是odom系下的运动信息
 */
void FrontEndEstimator::LaserUndistorted(msa2d::sensor::LaserScan& laser, 
                                                                                           Path& motion_info) {
    if (motion_info.size() == 0) {
        return;  
    }
    // 将odom系下的path转换到lidar系
    for (auto& it : motion_info) {
        poseOdomToPrimeLaserOdom(it.pose_, it.pose_);  
    }
    msa2d::Pose2d begin_pose = motion_info.back().pose_;     // T_begin<-end
    // 相邻的两个motion数据  靠近motion_info.back() 为 front ,靠近 motion_info.front() 的为 back 
    msa2d::Pose2d front_relate_begin_pose; // 相对与 激光最后一个点坐标系的pose 初始化为0 
    msa2d::Pose2d back_relate_begin_pose; // 相对与 激光最后一个点坐标系的pose 初始化为0 
    // 处理时path中pose的时间因该是相对于该帧起始时间的
    double part_front_time =  motion_info.back().time_stamp_ - motion_info.front().time_stamp_;
    double part_back_time = 0; 
    msa2d::Pose2d point_pose; // 每个激光点对应的雷达Pose 
    int16_t ptr = laser.pointcloud_.size() - 1;    // 从最后一个点往前进行处理 
    //   back -------------> front
    for (uint16_t i = motion_info.size() - 1; i >= 1; --i) {
        back_relate_begin_pose = begin_pose.inv() * motion_info[i - 1].pose_;  //T_end<-curr = T_end<-begin * T_begin<-curr
        part_back_time = motion_info[i - 1].time_stamp_ - motion_info.front().time_stamp_;

        while (ptr >= 0) {
            if (laser.pointcloud_[ptr].rel_time_ < part_back_time) {
                break;
            } 
            if (part_front_time - laser.pointcloud_[ptr].rel_time_ < 1e-3) {
                point_pose = front_relate_begin_pose;   // 该激光点时间与part_front_time接近，因此该激光点pose就近似为front_relate_begin_pose
            } else if (laser.pointcloud_[ptr].rel_time_ - part_back_time < 1e-3) {
                point_pose = back_relate_begin_pose;  // 该激光点时间与part_back_time接近，因此该激光点pose就近似为back_relate_begin_pose
            } else {
                // 插值
                point_pose = util::LinearInterpolate(front_relate_begin_pose, back_relate_begin_pose, 
                                                                                            part_front_time, part_back_time, laser.pointcloud_[ptr].rel_time_); 
            }
            // 将激光点观测转换到 最后一个点的坐标系   T_end<-curr * p_curr = p_end
            laser.pointcloud_.points()[ptr].pos_ = point_pose * laser.pointcloud_.points()[ptr].pos_;
            --ptr;
        }
        if (ptr < 0) break;  
        front_relate_begin_pose = back_relate_begin_pose; 
        part_front_time = part_back_time; 
    }
}

/**
 * @brief 系统初始话
 * 
 * @param laser_pose 
 * @param imu_selected 
 * @param wheel_odom_selected 
 * @param time_stamp 
 */
void FrontEndEstimator::systemInit(const msa2d::Pose2d& laser_pose, 
                                                                            const std::deque<msa2d::sensor::ImuData>& imu_selected,
                                                                            const std::deque<msa2d::sensor::WheelOdom>& wheel_odom_selected, 
                                                                            const double& time_stamp) {
    // 初始化
    static msa2d::Pose2d last_laser_pose = laser_pose; 
    msa2d::Pose2d pose_increm = last_laser_pose.inv() * laser_pose; 
    last_laser_pose = laser_pose; 
    // 有IMU数据 则首先初始化MU，即标定出 imu bias
    if (!imu_selected.empty() && !imu_initialized_) {
        // IMU初始化 
        if (imu_initializer_.Init(imu_selected)) {
            imu_initialized_ = true; 
        }
    } 

    if (!wheel_odom_selected.empty()) {
    } else {
        // 没有轮速且IMU初始化完成的情况下，那么可以确认进入LIO模式
        if (imu_initialized_) {
            system_initialized_ = true; 
            work_mode = MODE::lio; 
        }
    }

    if (ekf_estimate_enable_) {
        // LIO EKF估计器初始化
        if (work_mode == MODE::lio) {
            ekf_estimator_ = std::make_unique<EKFEstimatorPVQB>(
                laser_pose, 0, 0, imu_initializer_.GetYawBias(), 1e-4, time_stamp); 
        }
        // LWIO EKF估计器 初始化
        if (work_mode == MODE::lwio) {
        }
        // 如果之前IMU和轮速并为初始化成功，那么先初始化运动学模型的EKF进行使用
        if (ekf_estimator_ == nullptr) { 
            // 进行估计器初始化
            ekf_estimator_ = std::make_unique<EKFEstimatorBase>(laser_pose, 0, 0, time_stamp); 
        }
    }
}

/**
 * @brief 
 * 
 * @param laser_ptr 
 * @param wheelOdom_container 
 * @param imu_container 
 * @return true 
 * @return false 
 */
bool FrontEndEstimator::syncSensorData(const msa2d::sensor::LaserScan::Ptr& laser_ptr, 
                                                                                        std::deque<msa2d::sensor::WheelOdom>& wheelOdom_container,
                                                                                        std::deque<msa2d::sensor::ImuData>& imu_container) {
    bool wheel_extract_finish = true;
    bool imu_extract_finish = true; 
    // 提取轮速数据
    wheel_sm_.lock();
    if (!extractSensorData<msa2d::sensor::WheelOdom>(wheelOdom_cache_, wheelOdom_container,
            laser_ptr->start_time_, laser_ptr->end_time_)) {
        wheel_extract_finish = false; 
    }
    wheel_sm_.unlock();
    // 提取IMU数据
    imu_sm_.lock();
    if (!extractSensorData<msa2d::sensor::ImuData>(imu_cache_, imu_container,
            laser_ptr->start_time_, laser_ptr->end_time_)) {
        imu_extract_finish = false; 
    }
    imu_sm_.unlock();
    return wheel_extract_finish && imu_extract_finish; 
}

/**
 * @brief 
 * 
 * @tparam DataT_ 
 * @param data_cache 
 * @param extracted_container 
 * @param start_time 
 * @param end_time 
 * @return true 
 * @return false 
 */
template<class DataT_>
bool FrontEndEstimator::extractSensorData(std::deque<DataT_>& data_cache, 
                                                                                            std::deque<DataT_>& extracted_container,
                                                                                            const double& start_time, 
                                                                                            const double& end_time) {
    if (!data_cache.empty()) {
        if (data_cache.front().time_stamp_ <= start_time) {
            if (data_cache.back().time_stamp_ >= end_time) {
                // 添加第一个头数据 
                DataT_ begin_data;     // 时间戳 <= 激光起始时间戳的第一个数据

                while (data_cache.front().time_stamp_  <= start_time) {
                    begin_data = data_cache.front();
                    data_cache.pop_front();  
                }
                // 若这个begin_data 距离 激光起始时间太远了，那么需要进一步找begin_data
                if (start_time - begin_data.time_stamp_ > 1e-3) {
                    // 先直接看下一个数据 是否距离激光起始时间足够近
                    if (data_cache.front().time_stamp_ - start_time < 1e-3) {
                        begin_data = data_cache.front();
                        data_cache.pop_front();  
                    } else {
                        // 插值
                        begin_data = util::LinearInterpolate(begin_data, data_cache.front(), 
                                                                        begin_data.time_stamp_ , data_cache.front().time_stamp_, 
                                                                        start_time);
                    }
                }
                // 起始时间戳和激光第一个点的时间戳对齐  
                begin_data.time_stamp_ = start_time;  
                extracted_container.push_back(begin_data);   // 放置第一个数据
                // 添加中间段的数据
                auto data_ptr = data_cache.begin();  

                for (; data_ptr->time_stamp_ <= end_time; ++data_ptr) {
                    extracted_container.push_back(*data_ptr);
                }
                // 如果传感器最后一个数据的时间戳距离laser最后一个点的时间较远  那么向后寻找一个更接近的轮速数据
                if (end_time - extracted_container.back().time_stamp_ > 1e-3) {
                    if (data_ptr->time_stamp_ - end_time < 1e-3) {
                        extracted_container.push_back(*data_ptr); 
                    } else {
                        // 插值
                        auto end_data = util::LinearInterpolate(extracted_container.back(), *data_ptr, 
                            extracted_container.back().time_stamp_ , data_ptr->time_stamp_, 
                            end_time);
                        extracted_container.push_back(end_data); 
                    }
                }
                // 最后一个数据的时间戳和激光最后一个点的时间戳对齐 
                extracted_container.back().time_stamp_ = end_time; 
            } else {
                return false;  
            }
        } else {
            std::cout << msa2d::color::YELLOW << "warn: data_cache.front().time_stamp_ > start_time!" 
                << msa2d::color::RESET << std::endl;
        }
    }

    return true;  
}


/**
 * @brief 
 * 
 * @param pose_in_laser 
 * @param pose_in_world 
 */
void FrontEndEstimator::posePrimeLaserToOdom(const msa2d::Pose2d& pose_in_laser, 
                                                                                                            msa2d::Pose2d& pose_in_world) {
    pose_in_world.SetX(pose_in_laser.x());
    if (prime_laser_upside_down_) {
        pose_in_world.SetY(-pose_in_laser.y());
        pose_in_world.SetRotation(-pose_in_laser.yaw());
    } else {
        pose_in_world.SetY(pose_in_laser.y());
        pose_in_world.SetRotation(pose_in_laser.yaw());
    }
    // pose_in_laser = primeLaserOdomExtrinsic_ * pose_in_laser; 
}

/**
 * @brief 
 * 
 * @param pose_in_odom 
 * @param pose_in_laser 
 */
void FrontEndEstimator::poseOdomToPrimeLaserOdom(const msa2d::Pose2d& pose_in_odom, 
                                                                                                                        msa2d::Pose2d& pose_in_laser) {
    pose_in_laser.SetX(pose_in_odom.x());
    if (prime_laser_upside_down_) {
        pose_in_laser.SetY(-pose_in_odom.y());
        pose_in_laser.SetRotation(-pose_in_odom.yaw());
    } else {
        pose_in_laser.SetY(pose_in_odom.y());
        pose_in_laser.SetRotation(pose_in_odom.yaw());
    }
    // pose_in_laser = primeLaserOdomExtrinsic_ * pose_in_laser; 
}

/**
 * @brief 
 * 
 */
void FrontEndEstimator::reset() {
    last_map_updata_pose_.SetVec(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX));
    last_fusionOdom_pose_.pose_.SetVec(Eigen::Vector3f::Zero());
    //重置地图
    grid_map_pyramid_->reset();
}

/**
 * @brief 
 * 
 * @param pointcloud 
 */
void FrontEndEstimator::getLaserPyramidData(const msa2d::sensor::LaserScan& pointcloud) {
    // 第一层
    laser_pyramid_container_[0].clear(); 
    uint16_t num = pointcloud.pointcloud_.size(); 
    // std::cout << "getGridPyramidData size: " << num << std::endl;
    for (uint16_t i = 0; i < num; ++i) {
        laser_pyramid_container_[0].add(pointcloud.pointcloud_[i].pos_ * getScaleToMap());  
    }
    // 其他层  
    size_t size = grid_map_pyramid_->getMapLevels();
    for (int index = size - 1; index > 0; --index) {
        // 根据第0层地图层数，求取了在index层激光的数据
        laser_pyramid_container_[index].setFrom(laser_pyramid_container_[0], 
            static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));
    }
}
};
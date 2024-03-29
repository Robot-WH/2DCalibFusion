#include <iomanip> 
#include <assert.h>
// #include <glog/logging.h>
#include "FrontEnd2D/Estimator/estimator.hpp"
#include "msa2d/ScanMatcher/fast_correlative_scan_matcher_2d.h"
#include "msa2d/spdlog/spdlog.h"
namespace Estimator2D {
FrontEndEstimator::FrontEndEstimator(const std::string& config_path) {
    // google::InitGoogleLogging(nullptr);
    // char* home_dir = getenv("HOME");
    // std::string home_dir_str(home_dir);
    // std::string log_dir = home_dir_str + "/log";
    // std::cout << "lof file path: " << log_dir << std::endl;
    // FLAGS_log_dir = log_dir;
    // FLAGS_logtostderr = true;        // 输出到终端
    // FLAGS_alsologtostderr = true;    // 输出到终端和日志文件
    // google::SetLogFilenameExtension(".log"); // 设置日志文件的扩展名

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
    this->Reset();
    /* 设置进行地图更新的位姿变化阈值 **/
    this->SetMapUpdateMinDistDiff(0.4f);  // 
    this->SetMapUpdateMinAngleDiff(0.3491f);   // 20
    // 读取laser-imu外参
    Eigen::Matrix<float, 6, 1> laser_odom_ext;
    laser_odom_ext.setZero();  
    prime_laser_upside_down_ = yaml["extrinsic"]["odom"]["prime_laser"]["upside_down"].as<bool>();
    
    if (prime_laser_upside_down_) {
        std::cout << "主激光雷达颠倒!!! "<< std::endl;
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
    // 构造降采样滤波     0.01 res, 15是激光的范围 
    voxel_filter_.reset(new msa2d::filter::VoxelGridFilter(0.01, 15));    
    
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
 * @param data 
 */
void FrontEndEstimator::InputWheelOdom(msa2d::sensor::WheelOdom& data) {
    wheel_sm_.lock();   // 写锁 
    // std::cout << "wheel " << std::endl;
    wheelOdom_cache_.push_back(data);
    wheel_sm_.unlock();  
}

/**
 * @brief 
 * 
 * @param data 
 */
void FrontEndEstimator::InputImu(msa2d::sensor::ImuData& data) {
    imu_sm_.lock();
    std::cout << "imu InputImu, acc: " << data.acc_.norm() << std::endl;
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
                /**
                 * @brief 进行EKF预测   
                 * aser-imu-wheel模式： imu预测+laser&wheel校正
                 * imu-laser模式：imu预测+laser校正
                 * wheel-laser模式：wheel预测+laser校正
                 * 纯laser模式：运动模型预测+laser校正
                 */
                DiffModelDeadReckoning diff_model;  // 差分模型
                DiffModelDeadReckoning wheel_diff_model;  // 轮速差分模型   只在有轮速，且标定完成时使用(标定完成后只在雷达退化或高速时启用)
                bool use_motion_model_predict = true;  
                static msa2d::TimedPose2d last_wheel_pose;

                // 如果 有轮速数据，且odom与激光的外参已知  那么用轮速进行预测
                if (!wheel_odom_selected.empty()) {
                    // std::cout << msa2d::color::GREEN << "使用轮速计" << msa2d::color::RESET << std::endl; 
                    // 使用轮速进行预测
                    for (uint16_t i = 0; i < wheel_odom_selected.size(); ++i) {
                        auto& curr_data = wheel_odom_selected[i]; 
                        curr_data.omega_yaw_ *= laser_wheel_param_.rot_scale; 
                        // 如果有imu数据，且当前不出于轮速标定状态，用imu的角速度观测替换轮速计的角速度
                        if (!imu_selected.empty() && !wheel_calib_) {
                            msa2d::sensor::ImuData imu_data;
                            if (extractSensorData(imu_selected, imu_data, curr_data.time_stamp_)) {
                                curr_data.omega_yaw_ = imu_data.gyro_[2] - sensor_param_.imu_.gyro_bias_[2];  
                                // std::cout << "轮速  IMU" << std::endl;
                            } else {
                                // std::cout << "轮速获取IMU数据失败" << std::endl;
                            }
                        }
                        // 运动学前向传播
                        wheel_diff_model.Update(curr_data.time_stamp_, laser_wheel_param_.trans_scale * curr_data.v_x_, 
                                                                                curr_data.omega_yaw_); 
                        // // EKF预测
                        // /**
                        //  * @brief 对于轮速 需要考虑优化外参和不优化外参
                        //  */
                        // if (ekf_estimator_ != nullptr) {
                        //     ekf_estimator_->Predict(diff_model.ReadLastPose().pose_, curr_data.v_x_, 
                        //         curr_data.omega_yaw_, 0.0025, 1, curr_data.time_stamp_);    // 测量噪声 0.05m/s, 60度/s
                        // }
                    } 

                    last_wheel_pose.pose_ = last_wheel_pose.pose_ * wheel_diff_model.ReadLastPose().pose_;
                    last_wheel_pose.time_stamp_ = curr_laser_ptr->end_time_; 
                    ::util::DataDispatcher::GetInstance().Publish("WheelDeadReckoning", last_wheel_pose); 
                } 
                
                if (!imu_selected.empty()) {
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
                                ekf_estimator_->Predict(curr_data.gyro_[2], 0.25, curr_data.time_stamp_, diff_model);  
                            } else {
                                float un_bias_yaw_angular_v = 
                                    imu_coeff_ * (curr_data.gyro_[2] - sensor_param_.imu_.gyro_bias_[2]);  // 去除bias 
                                // 不使用EKF时，直接用IMU估计运动(这里假设线速度为0，只考虑旋转)
                                diff_model.Update(curr_data.time_stamp_, 0, un_bias_yaw_angular_v); 
                            }
                            // 估计姿态
                            simpleAttitudeEstimation(curr_data);
                            // 检查姿态变化  
                            if (i == imu_selected.size() - 1) {
                                variation_roll_ = attitude_roll_ - origin_roll_;
                                variation_pitch_ = attitude_pitch_ - origin_pitch_;
                                std::cout << "attitude_roll_: " << attitude_roll_ << ",attitude_pitch_: " << attitude_pitch_ << std::endl;
                                std::cout << "origin_roll_: " << origin_roll_ << ",origin_pitch_: " << origin_pitch_ << std::endl;
                                std::cout << "姿态变化，pitch：" << variation_pitch_ << ", roll: " << variation_roll_ << std::endl;
                            }
                        } 
                        std::cout << "预测完成" << std::endl;     
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
                /**
                 * @todo 地面校准 
                 */
                // 如果姿态变化超过15度，则跳过
                if (std::fabs(variation_roll_) < 0.2618 && std::fabs(variation_pitch_) < 0.2618) {
                    // 如果姿态变化超过了5度小与15度，那么需要将点云校正到水平
                    if (std::fabs(variation_roll_) > 0.0873 || std::fabs(variation_pitch_) > 0.0873) {
                        // std::cout << msa2d::color::YELLOW << "激光点云水平校正..." << msa2d::color::RESET << std::endl;
                        // laserLevelCorrection(*curr_laser_ptr); 
                    }
                    // 如果位姿丢失   需要重定位 
                    // 若连续10帧都重定位失败，那么新建一个轨迹  
                    if (track_loss_) {
                        static int relocalization_times = 0; 
                        std::cout << msa2d::color::RED << "tracking loss !!!!" << 
                            msa2d::color::RESET << std::endl;
                        // 进行重定位
                        msa2d::ScanMatcher::FastCorrelativeScanMatcherOptions2D fcsm_option;

                        msa2d::time::TicToc tt;
                        // static float avg_time = 0; 
                        static int N = 0; 
                        msa2d::ScanMatcher::FastCorrelativeScanMatcher2D global_matcher(grid_map_pyramid_->getGridMap(0), fcsm_option);
                        float score = 0.f;
                        msa2d::Pose2d global_pose_estimate;
                        if (global_matcher.MatchFullSubmap(curr_laser_ptr->pointcloud_, 0.9, score, global_pose_estimate)) {
                            float t = tt.toc("---------------------------------------global matcher: ");
                            // ++N;
                            // avg_time += (t - avg_time) / N;  // 估计样本均值
                            // std::cout << "avg_time: " << avg_time << std::endl;
                            // std::cout << "score: " << score << ", global_pose_estimate x: " << global_pose_estimate.x()
                            //     << ", y: " << global_pose_estimate.y() << ",yaw: " << global_pose_estimate.yaw() << std::endl; 
                            // global_pose_estimate.SetVec(
                            //     grid_map_pyramid_->getGridMap(0)->getGridMapBase().PoseMapToWorld(global_pose_estimate.vec()));
                            last_fusionOdom_pose_.pose_.SetVec(
                                grid_map_pyramid_->getGridMap(0)->getGridMapBase().PoseMapToWorld(global_pose_estimate.vec()));
                            
                            // std::cout << "global_pose_estimate: " << global_pose_estimate.vec().transpose() << std::endl;
                            // ::util::DataDispatcher::GetInstance().Publish("global_pose", global_pose_estimate); 
                            // ::util::DataDispatcher::GetInstance().Publish("undistorted_pointcloud", *curr_laser_ptr); 
                            spdlog::info("定位找回！");
                            relocalization_times = 0; 
                        } else {
                            spdlog::info("重定位失败！");
                            std::cout << msa2d::color::RED << "global matcher error" << msa2d::color::RESET << std::endl;
                            relocalization_times++;
                            // 如果连续几次重定位都失败的话   那么重置一次建图
                            if (relocalization_times > 5) {
                                reset();  
                                relocalization_times = 0;
                                track_loss_ = false;
                            }
                            continue;  
                        }

                        // laser_sm_.lock();
                        // laser_cache_.pop_front();  
                        // laser_sm_.unlock();  
                        // wheel_odom_selected.clear(); 
                        // imu_selected.clear();  
                        // continue;  
                    }

                    msa2d::Pose2d predict_incre_pose;   // 预测帧间的运动 
                    predict_incre_pose = diff_model.ReadLastPose().pose_;
                    std::cout << msa2d::color::YELLOW << "predict_incre_pose: " << predict_incre_pose.vec().transpose()
                        << msa2d::color::RESET << std::endl;
                    if (imu_calib_) {
                        LaserUndistorted(*curr_laser_ptr, diff_model.GetPath());  // 去除laser的畸变
                    }
                    // 体素降采样滤波
                    // std::cout << "ori size: " << curr_laser_ptr->pointcloud_.size() << std::endl;
                    // msa2d::time::TicToc tt;
                    voxel_filter_->Filter(curr_laser_ptr->pointcloud_); 
                    // tt.toc("filter ");
                    // std::cout << "filter size: " << curr_laser_ptr->pointcloud_.size() << std::endl;
                    // 如果是第一帧激光数据，则进行地图倾斜校正
                    if (map_correct_) {
                        map_correct_ = false; 
                    }
 
                    msa2d::Pose2d estimated_odom_pose = last_fusionOdom_pose_.pose_ * predict_incre_pose;   // To<-last * Tlast<-curr
                    // 将odom系下的预测位姿转换到laser 安装平面
                    msa2d::Pose2d new_estimate_laserOdom_pose;
                    odomPoseToLaserPlane(estimated_odom_pose, new_estimate_laserOdom_pose);
                    // 基于栅格金字塔快速检测预测是否可靠，判断是否存在里程计打滑等现象
                    // fastPredictFailureDetection(curr_laser_ptr, new_estimate_laserOdom_pose, grid_map_pyramid_->getGridMap(1)); 
                    std::cout << "odom predict: " << estimated_odom_pose.vec().transpose() << std::endl;
                    // 将点云数据转换为激光多分辨率金字塔数据
                    getLaserPyramidData(*curr_laser_ptr);     // 构造 laser_pyramid_container_ 
                    // tt.toc("getGridPyramidData");
                    // tt.tic(); 
                    new_estimate_laserOdom_pose.SetVec(hector_matcher_->Solve(new_estimate_laserOdom_pose.vec(), 
                        laser_pyramid_container_, *grid_map_pyramid_, lastScanMatchCov)); 
                    std::cout << "laserOdom_pose: " << new_estimate_laserOdom_pose.vec().transpose() << std::endl;
                    // 细匹配  ICP / NDT 
                    // tt.toc("match solve");
                    // 将pose转换到odom平面下
                    laserPoseToOdomPlane(new_estimate_laserOdom_pose, estimated_odom_pose);
                    // std::cout << "Odom_pose: " << estimated_odom_pose.vec().transpose() << std::endl;
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
                        odomPoseToLaserPlane(estimated_odom_pose, new_estimate_laserOdom_pose);
                    }
                    // 如果轮速需要标定
                    // if (wheel_calib_) {
                    //     msa2d::Pose2d scan_matched_incre_pose = 
                    //         last_fusionOdom_pose_.pose_.inv() * estimated_odom_pose;
                    //     if (laser_wheel_calib_.AddData(scan_matched_incre_pose, wheel_diff_model.ReadLastPose().pose_)) {
                    //         laser_wheel_param_ = laser_wheel_calib_.GetParam();  
                    //         wheel_calib_ = false;  
                    //         last_wheel_pose.pose_ = estimated_odom_pose;
                    //         std::cout << msa2d::color::GREEN << "激光-轮速标定完成！外参 x: " << laser_wheel_param_.x
                    //             << ", y: " << laser_wheel_param_.y << ",theta: " << laser_wheel_param_.theta << ", 尺度 rot_scale: " 
                    //             << laser_wheel_param_.rot_scale << ",trans_scale: " << laser_wheel_param_.trans_scale 
                    //             << msa2d::color::RESET << std::endl;
                    //     }  
                    // }
                    // 针对IMU  
                    if (!system_initialized_) {
                        systemInit(new_estimate_laserOdom_pose, imu_selected, curr_laser_ptr->end_time_);
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
                    // 动态目标检测 
                    dynamicObjectDetect(curr_laser_ptr, new_estimate_laserOdom_pose, grid_map_pyramid_->getGridMap(1), 
                        dynamic_points_data.first, stable_points_data.first, undetermined_points_data.first);

                    last_fusionOdom_pose_.pose_ = estimated_odom_pose;
                    last_fusionOdom_pose_.time_stamp_ = curr_laser_ptr->end_time_; 
                    // last_fusionOdom_pose_.time_stamp_ = curr_laser_ptr->start_time_; 
                    // 发布数据
                    ::util::DataDispatcher::GetInstance().Publish("fusionOdom", last_fusionOdom_pose_); 
                    // ::util::DataDispatcher::GetInstance().Publish("dynamic_pointcloud", dynamic_points_data); 
                    ::util::DataDispatcher::GetInstance().Publish("stable_pointcloud", stable_points_data); 
                    // ::util::DataDispatcher::GetInstance().Publish("local_map", local_map_.ReadLocalMap()); 
                    // ::util::DataDispatcher::GetInstance().Publish("undetermined_pointcloud", undetermined_points_data); 
                    // 判断位姿是否丢失
                    if (!judgeTrackingLoss(stable_points_data.first.size(), curr_laser_ptr->pointcloud_.size())) {
                        /** 2.地图更新(last_map_updata_pose_初始化为一个很大的值，因此第一帧点云就会更新地图) **/
                        if (msa2d::module::poseDifferenceLargerThan(new_estimate_laserOdom_pose.vec(), last_map_updata_pose_.vec(), 
                                paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate)) { 
                            // 仅在位姿变化大于阈值 或者 map_without_matching为真 的时候进行地图更新
                            grid_map_pyramid_->updateByScan(laser_pyramid_container_, new_estimate_laserOdom_pose.vec());
                            grid_map_pyramid_->onMapUpdated();
                            // local_map_.UpdateLocalMapForMotion(stable_points_data.first);
                            last_map_updata_pose_ = new_estimate_laserOdom_pose;
                        }
                    }
                } else {
                    spdlog::info("姿态倾斜过大！");
                    std::cout << "姿态倾斜过大！" << std::endl;
                    std::cout << "std::fabs(variation_roll_): " << std::fabs(variation_roll_) <<
                        ",std::fabs(variation_pitch_): " << std::fabs(variation_pitch_) << std::endl;
                }

                if (reset_flag_) {
                    reset();  
                    reset_flag_ = false;   
                }

                laser_sm_.lock();
                laser_cache_.pop_front();  
                laser_sm_.unlock();  
                wheel_odom_selected.clear(); 
                imu_selected.clear();  
                // tt.toc("map update");
                // std::cout << "本轮处理完成" << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}


void FrontEndEstimator::mapSlopeCorrect(const msa2d::sensor::LaserPointCloud& pointcloud) {
    Eigen::Vector2f line_front_point;
    float curr_line_tan_slope = -1;

    // for () {
    //     float tan_slope = (point.y - line_front_point.y) / (point.x- line_front_point.x);
    //     if (std::fabs(tan_slope - curr_line_tan_slope) < 0.01) {

    //     }
    // }
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
 * @brief 动态目标检测   
 * 
 * @param laser_ptr 
 * @param pose 
 * @param occGridMap 
 * @param dynamic_points 
 * @param stable_points 
 * @param undetermined_points 
 */
// void FrontEndEstimator::dynamicObjectDetect(const msa2d::sensor::LaserScan::Ptr& laser_ptr, 
//                                                                                             const msa2d::Pose2d& pose, 
//                                                                                             const msa2d::map::OccGridMapBase* occGridMap,
//                                                                                             std::vector<Eigen::Vector2f>& dynamic_points,
//                                                                                             std::vector<Eigen::Vector2f>& stable_points,
//                                                                                             std::vector<Eigen::Vector2f>& undetermined_points) {
//     std::vector<uint16_t> candidate_dynamic_points_index; 
//     candidate_dynamic_points_index.reserve(laser_ptr->pointcloud_.size());  
//     // 先提取出动态候选点  
//     for (uint16_t i = 0; i < laser_ptr->pointcloud_.size(); ++i) {
//         Eigen::Vector2f point_gridmap_pos = occGridMap->getGridMapBase().PosWorldToMapf(laser_ptr->pointcloud_[i].pos_);     // laserOdom -> map
//         // 超过范围的点一律丢掉
//         if (occGridMap->getGridMapBase().pointOutOfMapBounds(point_gridmap_pos)) {
//             continue;      
//         }
//         uint16_t grid_x = point_gridmap_pos[0];
//         uint16_t grid_y = point_gridmap_pos[1]; 
//         // 只要是非占用(周围领域也需要是非占用)，即空闲点和未知点，那么就认为是候选动态 
//         if (!occGridMap->isOccupied(grid_x, grid_y)) {
//             if (!occGridMap->isOccupied(grid_x + 1, grid_y) && 
//                     !occGridMap->isOccupied(grid_x - 1, grid_y) && 
//                     !occGridMap->isOccupied(grid_x, grid_y + 1) &&
//                     !occGridMap->isOccupied(grid_x, grid_y - 1) &&
//                     !occGridMap->isOccupied(grid_x + 1, grid_y + 1) &&
//                     !occGridMap->isOccupied(grid_x - 1, grid_y - 1) &&
//                     !occGridMap->isOccupied(grid_x + 1, grid_y - 1) &&
//                     !occGridMap->isOccupied(grid_x - 1, grid_y + 1)) {
//                 // 检测该栅格前后八领域，如果都不是占据点，则认为是可疑的动态候选
//                 candidate_dynamic_points_index.push_back(i);
//                 continue;  
//             }
//         }
//     }

//     // std::cout << "candidate_dynamic_points num: " << candidate_dynamic_points_index.size() << std::endl;
//     // 对于候选动态点进行可视检测   进一步分类为动态点和待定点  
//     std::vector<uint8_t> points_type_table;    // 标记点云的类型  
//     points_type_table.resize(laser_ptr->pointcloud_.size(), 0);  


    
//     std::vector<uint16_t> cluster_points; 
//     cluster_points.reserve(100);  
//     // 聚类   向两侧生长  
//     // 先向左 
//     int last_cluster_end_index = -1;

//     for (const uint16_t& i : candidate_dynamic_points_index) {
//         if (i <= last_cluster_end_index) continue; 
//         uint16_t point_index = i;
//         cluster_points.push_back(point_index);
//         // // 向左生长
//         // while (point_index > last_cluster_end_index + 1) {
//         //     const float& curr_range = laser_ptr->pointcloud_[point_index].range_;
//         //     const float& last_range = laser_ptr->pointcloud_[point_index - 1].range_;
//         //     float min_d = std::min(last_range, curr_range);
//         //     float max_d = std::max(last_range, curr_range);
//         //     float alpha = laser_ptr->pointcloud_[point_index].rel_angle_ - 
//         //                                 laser_ptr->pointcloud_[point_index - 1].rel_angle_;
//         //     assert(alpha > 0);
//         //     if (alpha > 0.1745) break;      // 前后两个点的角度间隔太远
//         //     float beta = std::atan2(min_d * std::sin(alpha), max_d - min_d * std::cos(alpha));  
//         //     if (beta < 0.3491) break;   // 如果夹角很小 < 20，说明不是一个类别的    本次聚类结束  
//         //     --point_index;
//         //     cluster_points.push_back(point_index);
//         // }

//         // // 向右生长
//         // point_index = i;
//         // while (point_index < laser_ptr->pointcloud_.size() - 1) {
//         //     const float& curr_range = laser_ptr->pointcloud_[point_index].range_;
//         //     const float& next_range = laser_ptr->pointcloud_[point_index + 1].range_;
//         //     float min_d = std::min(next_range, curr_range);
//         //     float max_d = std::max(next_range, curr_range);
//         //     float alpha = laser_ptr->pointcloud_[point_index + 1].rel_angle_ - 
//         //                                 laser_ptr->pointcloud_[point_index].rel_angle_;
//         //     assert(alpha > 0);
//         //     if (alpha > 0.1745) break;      // 前后两个点的角度距离太远
//         //     float beta = std::atan2(min_d * std::sin(alpha), max_d - min_d * std::cos(alpha));  
//         //     if (beta < 0.3491) break;   // 如果夹角很小，说明不是一个类别的    本次聚类结束  
//         //     ++point_index;
//         //     cluster_points.push_back(point_index);
//         // }

//         // std::cout << "cluster_points size: " << cluster_points.size() << std::endl;

//         last_cluster_end_index = point_index;   
//         // 聚类后 数量超过3的认为是动态
//         for (const auto& i : cluster_points) {
//             points_type_table[i] = 1;
//             if (cluster_points.size() > 0) {
//                 dynamic_points.push_back(laser_ptr->pointcloud_[i].pos_); 
//             } else {
//                 undetermined_points.push_back(laser_ptr->pointcloud_[i].pos_); 
//             }
//         }
//         cluster_points.clear();  
//     }

//     // 提取出稳定点 
//     for (uint16_t i = 0; i < points_type_table.size(); ++i) {
//         if (points_type_table[i] == 0) {
//             stable_points.push_back(laser_ptr->pointcloud_[i].pos_);  // 在一定距离内  且静态的点叫稳定点
//         }
//     }
// }

void FrontEndEstimator::dynamicObjectDetect(const msa2d::sensor::LaserScan::Ptr& laser_ptr, 
                                                                                            const msa2d::Pose2d& pose, 
                                                                                            const msa2d::map::OccGridMapBase* occGridMap,
                                                                                            std::vector<Eigen::Vector2f>& dynamic_points,
                                                                                            std::vector<Eigen::Vector2f>& stable_points,
                                                                                            std::vector<Eigen::Vector2f>& undetermined_points) {
    std::vector<uint16_t> candidate_dynamic_points_index; 
    candidate_dynamic_points_index.reserve(laser_ptr->pointcloud_.size());  
    // 先提取出动态候选点  
    for (uint16_t i = 0; i < laser_ptr->pointcloud_.size(); ++i) {
        Eigen::Vector2f point_gridmap_pos = occGridMap->getGridMapBase().PosWorldToMapf(laser_ptr->pointcloud_[i].pos_);     // laserOdom -> map
        // 超过范围的点一律丢掉
        if (occGridMap->getGridMapBase().pointOutOfMapBounds(point_gridmap_pos)) {
            continue;      
        }
        uint16_t grid_x = point_gridmap_pos[0];
        uint16_t grid_y = point_gridmap_pos[1]; 
        // 只要是非占用(周围领域也需要是非占用)，即空闲点和未知点，那么就认为是候选动态 
        if (!occGridMap->isOccupied(grid_x, grid_y)) {
            if (!occGridMap->isOccupied(grid_x + 1, grid_y) && 
                    !occGridMap->isOccupied(grid_x - 1, grid_y) && 
                    !occGridMap->isOccupied(grid_x, grid_y + 1) &&
                    !occGridMap->isOccupied(grid_x, grid_y - 1) &&
                    !occGridMap->isOccupied(grid_x + 1, grid_y + 1) &&
                    !occGridMap->isOccupied(grid_x - 1, grid_y - 1) &&
                    !occGridMap->isOccupied(grid_x + 1, grid_y - 1) &&
                    !occGridMap->isOccupied(grid_x - 1, grid_y + 1)) {
                // 检测该栅格前后八领域，如果都不是占据点，则认为是可疑的动态候选
                candidate_dynamic_points_index.push_back(i);
                continue;  
            } else {
                stable_points.push_back(laser_ptr->pointcloud_[i].pos_);  // 在一定距离内  且静态的点叫稳定点
            }
        } else {
            stable_points.push_back(laser_ptr->pointcloud_[i].pos_);  // 在一定距离内  且静态的点叫稳定点
        }
    }
}

/**
 * @brief 判断位姿是否的丢失 
 *                  通过稳定匹配点占总激光点数的比例进行判断  
 *                  定位丢失，则稳定匹配点占总激光点数的比例将会突然下降
 * @param dynamic_points 
 * @return true 位姿丢失
 * @return false  位姿良好
 */
bool FrontEndEstimator::judgeTrackingLoss(const uint16_t& good_matched_points_num,
                                                                                            const uint16_t& all_points_num) {
    // return false;
    float curr_track_quality_factor = 
        (float)good_matched_points_num / all_points_num; 

    if (track_loss_) {
        if (curr_track_quality_factor > 0.8) {
            track_loss_ = false;
            last_track_quality_factor_ = curr_track_quality_factor;  
            return true;  
        }
    }

    // 初始化
    if (last_track_quality_factor_ < 0) {
        if (good_matched_points_num == 0) {
            return false;  
        }
        // 点的质量必须要良好才能初始化
        if (curr_track_quality_factor > 0.8) { 
            last_track_quality_factor_ = curr_track_quality_factor;
            return false;    // 没有丢失位姿
        } else {
            return true;  // 丢失位姿
        }
    }
    // LOG(INFO) << "last_track_quality_factor_: " << last_track_quality_factor_ << ",curr_track_quality_factor: "
        // << curr_track_quality_factor;
    spdlog::info("last_track_quality_factor_: {}, curr_track_quality_factor: {}", 
        last_track_quality_factor_, curr_track_quality_factor);
    // 判断位姿丢失的条件 : 当前良好匹配点占比小于0.3 并且 这个占比前后变化巨大   
    if (curr_track_quality_factor < 0.6 && curr_track_quality_factor < last_track_quality_factor_ * 0.65) {
        track_loss_ = true;  
        last_track_quality_factor_ = -1;
        spdlog::info("定位丢失！");
        // LOG(INFO) << "tracking lost !!!!";
        return true; 
    }
    last_track_quality_factor_ = curr_track_quality_factor;
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
    // 将odom下的path转换到lidar平面
    for (auto& it : motion_info) {
        odomPoseToLaserPlane(it.pose_, it.pose_);  
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
 * @brief 
 * 
 * @param laser 
 * @param variation_pitch 
 * @param variation_roll 
 */
void FrontEndEstimator::laserLevelCorrection(msa2d::sensor::LaserScan& laser) {
    Eigen::Matrix3d rotationMatrix = origin_R_gb_.inverse() * sensor_param_.imu_.R_gb_;   // curr->origin

    for (auto& p : laser.pointcloud_) {
        Eigen::Vector3f p_local(p.pos_[0], p.pos_[1], 0);
        Eigen::Vector3f p_global = rotationMatrix.cast<float>() * p_local;
        p.pos_[0] = p_global[0];
        p.pos_[1] = p_global[1];
    }
}

/**
 * @brief 系统初始化
 * 
 * @param laser_pose 
 * @param imu_selected 
 * @param wheel_odom_selected 
 * @param time_stamp 
 */
void FrontEndEstimator::systemInit(const msa2d::Pose2d& laser_pose, 
                                                                            const std::deque<msa2d::sensor::ImuData>& imu_selected,
                                                                            const double& time_stamp) {
    // 初始化
    static msa2d::Pose2d last_laser_pose = laser_pose; 
    msa2d::Pose2d pose_increm = last_laser_pose.inv() * laser_pose; 
    last_laser_pose = laser_pose; 
    // 有IMU数据 则首先初始化MU，即标定出 imu bias
    if (!imu_selected.empty() && !imu_initialized_) {
        // IMU初始化 
        if (imu_tool_.Initialize(imu_selected, sensor_param_.imu_)) {
            origin_roll_ = atan2(sensor_param_.imu_.R_gb_(2, 1), sensor_param_.imu_.R_gb_(2, 2));
            origin_pitch_ = asin(-sensor_param_.imu_.R_gb_(2, 0)); 
            // 初始化时 放置的地面的倾斜度要小与3度  
            if (std::fabs(origin_roll_) < 0.0524 && std::fabs(origin_pitch_) < 0.0524) {
                imu_initialized_ = true; 
                system_initialized_ = true; 
                work_mode = MODE::lio; 
                origin_R_gb_ = sensor_param_.imu_.R_gb_;  
                std::cout << msa2d::color::GREEN << "IMU初始化成功, 角速度bias: " 
                    << sensor_param_.imu_.gyro_bias_.transpose() << ",加速度scale: " << sensor_param_.imu_.acc_scale_
                    << ", 姿态角 pitch: " << origin_pitch_
                    << ", roll: " << origin_roll_ << msa2d::color::RESET << std::endl;
            }
        }
    } 

    if (ekf_estimate_enable_) {
        // LIO EKF估计器初始化
        if (work_mode == MODE::lio) {
            ekf_estimator_ = std::make_unique<EKFEstimatorPVQB>(
                laser_pose, 0, 0, sensor_param_.imu_.gyro_bias_[2], 1e-4, time_stamp); 
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
    static bool wheel_extract_finish = false;
    static bool imu_extract_finish = false; 
    std::cout << "laser_ptr->start_time_: " << laser_ptr->start_time_ << ",laser_ptr->end_time_: " << laser_ptr->end_time_ << std::endl;
    // 提取轮速数据
    std::cout << "extract wheel ... " << wheel_extract_finish << std::endl;
    wheel_sm_.lock();
    if (!wheel_extract_finish && extractSensorData<msa2d::sensor::WheelOdom>(wheelOdom_cache_, 
            wheelOdom_container, laser_ptr->start_time_, laser_ptr->end_time_)) {
        wheel_extract_finish = true; 
    }
    wheel_sm_.unlock();
    // 提取IMU数据
    std::cout << "extract imu... " << imu_extract_finish << std::endl;
    imu_sm_.lock();
    if (!imu_extract_finish && extractSensorData<msa2d::sensor::ImuData>(imu_cache_, imu_container,
            laser_ptr->start_time_, laser_ptr->end_time_)) {
        imu_extract_finish = true; 
    }
    imu_sm_.unlock();

    if (wheel_extract_finish && imu_extract_finish) {
        wheel_extract_finish = false;
        imu_extract_finish = false;  
        return true;
    }

    return false; 
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
                std::cout << "extracted_container size: " << extracted_container.size() << std::endl;
            } else {
                std::cout << "data_cache.back().time_stamp_ < end_time" << std::endl;
                return false;  
            }
        } else {
            std::cout << msa2d::color::YELLOW << "warn: data_cache.front().time_stamp_ > start_time!" 
                << msa2d::color::RESET << std::endl;
            std::cout << std::setprecision(15) << "start_time: " << start_time << std::endl;
            std::cout << std::setprecision(15) << "data_cache.front().time_stamp_: " << data_cache.front().time_stamp_ << std::endl;
        }
    }

    return true;  
}

/**
 * @brief 从数据缓存中找出时间戳为 timestamp 的数据
 * 
 * @tparam DataT_ 
 * @param data_cache 
 * @param data 
 * @param timestamp 
 * @return true 
 * @return false 
 */
template<class DataT_>
bool FrontEndEstimator::extractSensorData(const std::deque<DataT_>& data_cache, 
                                                                                            DataT_& data, const double& timestamp) {
    if (data_cache.front().time_stamp_ > timestamp || data_cache.back().time_stamp_ < timestamp) {
        return false;
    }
    int i = 0;
    for (i = 0;  i < data_cache.size(); ++i) {
        if (data_cache[i].time_stamp_ >= timestamp) {
            break;  
        }
    }
    if (data_cache[i].time_stamp_ == timestamp) {
        data = data_cache[i];  
    } else {
        if (data_cache[i - 1].time_stamp_ + 0.002 > timestamp) {
            data = data_cache[i - 1];  
        } else {
            // 插值
            data = util::LinearInterpolate(data_cache[i - 1], data_cache[i], 
                                                                        data_cache[i - 1].time_stamp_ , data_cache[i].time_stamp_, 
                                                                        timestamp);
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
void FrontEndEstimator::laserPoseToOdomPlane(const msa2d::Pose2d& pose_in_laser, 
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
void FrontEndEstimator::odomPoseToLaserPlane(const msa2d::Pose2d& pose_in_odom, 
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
void FrontEndEstimator::Reset() {
    // std::cout << "FrontEndEstimator reset " << std::endl;
    reset_flag_ = true;  
}

void FrontEndEstimator::reset() {
    imu_initialized_ = false;  
    system_initialized_ = false; 
    last_attitude_predict_time_ = 0;   
    last_track_quality_factor_ = -1;
    last_map_updata_pose_.SetVec(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX));
    last_fusionOdom_pose_.pose_.SetVec(Eigen::Vector3f::Zero());
    ekf_estimator_.reset();  
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

/**
 * @brief 姿态估计
 * 
 */
void FrontEndEstimator::simpleAttitudeEstimation(const msa2d::sensor::ImuData& curr_data) {
    // static Eigen::Matrix3d test_G_R_I_;
    // 第一个IMU数据不计算，只记录时间信息
    if (last_attitude_predict_time_ == 0) {
        last_attitude_predict_time_ = curr_data.time_stamp_;
        last_attitude_update_time_ = last_attitude_predict_time_;  
        // test_G_R_I_ = sensor_param_.imu_.R_gb_; 
        return;  
    }
    if (curr_data.time_stamp_ <= last_attitude_predict_time_) {
        return;  
    }
    // std::cout << "curr_data.time_stamp_: " << curr_data.time_stamp_ << std::endl;
    // std::cout << "last_attitude_predict_time_: " << last_attitude_predict_time_ << std::endl;
    const Eigen::Vector3d unbiased_gyro = curr_data.gyro_ - sensor_param_.imu_.gyro_bias_;   // 去偏置 
    // std::cout << "unbiased_gyro: " << unbiased_gyro.transpose() << std::endl;
    const Eigen::Vector3d angle_vec = unbiased_gyro * (curr_data.time_stamp_ - last_attitude_predict_time_);
    // std::cout << "angle_vec: " << angle_vec.transpose() << std::endl;
    const double angle = angle_vec.norm();
    const Eigen::Vector3d axis = angle_vec / angle;
    Eigen::Matrix3d delta_rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    sensor_param_.imu_.R_gb_ = sensor_param_.imu_.R_gb_ * delta_rot;    // 旋转矩阵姿态更新  
    // test_G_R_I_ = test_G_R_I_ * delta_rot;  
    last_attitude_predict_time_ = curr_data.time_stamp_; 
    // 由陀螺仪观测出来的姿态角
    attitude_roll_ = atan2(sensor_param_.imu_.R_gb_(2, 1), sensor_param_.imu_.R_gb_(2, 2));
    attitude_pitch_ = asin(-sensor_param_.imu_.R_gb_(2, 0)); 
    // std::cout << "predict attitude_roll_: " << attitude_roll_ << ", attitude_pitch_:" << attitude_pitch_ << std::endl;
    /**
     * @brief 在线标定陀螺仪bias
     * 测试：使用在线标定，15min 角度最大漂移0.1rad
     * 不使用在线标定，15min角度最大漂移 0.3rad
     * @todo 为了节省计算量，采用了一种迭代的静止标定方法，局限是需要载体在静止时才会自动校准参数，
     *  改进点是如何实现高效的动态标定
     */
    imu_tool_.OnlineCalibGyroBias(unbiased_gyro, sensor_param_.imu_.gyro_bias_); 

    // float gravity_pitch = -std::asin(curr_data.acc_[0] / 9.81); 
    // float gravity_roll = std::atan2(curr_data.acc_[1], curr_data.acc_[2]); 

    // util::SaveDataCsv<double>("", "gravity_pitch.csv", 
    //                {gravity_pitch, curr_data.time_stamp_} , {"gravity_pitch", "time"});
    // util::SaveDataCsv<double>("", "gravity_roll.csv", 
    //                {gravity_roll, curr_data.time_stamp_} , {"gravity_roll", "time"});

    // float gyro_roll_ = atan2(sensor_param_.imu_.R_gb_(2, 1), sensor_param_.imu_.R_gb_(2, 2));
    // float gyro_pitch_ = asin(-sensor_param_.imu_.R_gb_(2, 0));

    // float ori_gyro_roll_ = atan2(test_G_R_I_(2, 1), test_G_R_I_(2, 2));
    // float ori_gyro_pitch_ = asin(-test_G_R_I_(2, 0));

    // util::SaveDataCsv<double>("", "gyro_roll_.csv", 
    //                {ori_gyro_roll_, curr_data.time_stamp_} , {"gyro_roll_", "time"});
    // util::SaveDataCsv<double>("", "gyro_pitch_.csv", 
    //                {ori_gyro_pitch_, curr_data.time_stamp_} , {"gyro_pitch_", "time"});

    // 重力校正
    Eigen::Vector3d acc = curr_data.acc_ * sensor_param_.imu_.acc_scale_;   
    float acc_v = acc.norm();
    // std::cout << "acc_v: " << acc_v << std::endl;
    // 加速度不能和重力相差太大  
    if (std::fabs(acc_v - 9.8) < 0.3 * 9.8) {
        double correct_delta_t = curr_data.time_stamp_ - last_attitude_update_time_;
        // 至少间隔0.1s
        if (correct_delta_t > 0.1) {
            // 计算权重因子
            float time_ratio = 1 - std::exp(-correct_delta_t / 0.5);     // 距离上一次校正的时间间隔越大   越接近1
            // std::cout << "correct_delta_t: " << correct_delta_t << ", time_ratio: " << time_ratio << std::endl;
            // 由重力观测出姿态角
            float gravity_pitch = -std::asin(acc[0] / 9.81); 
            float gravity_roll = std::atan2(acc[1], acc[2]); 
            // std::cout << "gravity_pitch: " << gravity_pitch << ", gravity_roll:" << gravity_roll << std::endl;
            // 重力解算出来的结果与陀螺仪解算出来的结果相差不能太大(5度)
            // 否则由与短时间更加信任陀螺仪，所以认为重力观测不准
            if (std::fabs(attitude_roll_ - gravity_roll) < 0.08 && std::fabs(attitude_pitch_ - gravity_pitch) < 0.08) {
                //  加权融合
                attitude_pitch_ = time_ratio * gravity_pitch + (1 - time_ratio) * attitude_pitch_;
                attitude_roll_ = time_ratio * gravity_roll + (1 - time_ratio) * attitude_roll_;
                // 重新更新旋转矩阵
                sensor_param_.imu_.R_gb_ = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
                                                        * Eigen::AngleAxisd(attitude_pitch_, Eigen::Vector3d::UnitY())
                                                        * Eigen::AngleAxisd(attitude_roll_, Eigen::Vector3d::UnitX());
                // std::cout << msa2d::color::GREEN << "std::fabs(gyro_roll_ - roll) < 0.08 && std::fabs(gyro_pitch_ - pitch)"
                //     << msa2d::color::RESET << std::endl;
                last_attitude_update_time_ = curr_data.time_stamp_; 
            } else {
                // std::cout << msa2d::color::RED << "------------------------------------------------------------"
                //     << msa2d::color::RESET << std::endl;
            }
        }
    } else {
        std::cout << msa2d::color::RED << "acc_v too large" << std::endl;
    }
    // Eigen::Matrix3d rotation_matrix = sensor_param_.imu_.R_gb_;
    // // float ori_roll_ = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2)) - sensor_param_.imu_.original_roll_;
    // // float ori_pitch_ = asin(-rotation_matrix(2, 0)) - sensor_param_.imu_.original_pitch_;
    // float ori_roll_ = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
    // float ori_pitch_ = asin(-rotation_matrix(2, 0));

    // util::SaveDataCsv<double>("", "fusion_pitch.csv", 
    //                 {ori_pitch_, curr_data.time_stamp_} , {"fusion_pitch", "time"});
    // util::SaveDataCsv<double>("", "fusion_roll.csv", 
    //                 {ori_roll_, curr_data.time_stamp_} , {"fusion_roll", "time"});           
}
};

#pragma once 

#include <yaml-cpp/yaml.h>
#include "Kalman/ekf_estimator.hpp"
#include "Initialize/2d_imu_initialize.hpp"
#include "motionModel.hpp"
#include "Calibration/2d_handeye_calibration.hpp"
#include "../Map/TimedSlidingLocalMap.hpp"
#include "../Map/GridMap.h"
#include "../Map/OccGridMapUtilConfig.h"
#include "../Sensor/LaserPointContainer.h"
#include "../Sensor/SensorData.hpp"
#include "../Lib/Algorithm/Pointcloud/laserUndistorted.hpp"
#include "../Lib/Algorithm/Pointcloud/ScanMatcher/hectorScanMatcher.hpp"
#include "../Lib/Algorithm/Pointcloud/ScanMatcher/imls_icp.h"

#include "../util/UtilFunctions.h"
#include "../util/DataDispatcher.hpp"
#include "../util/utility.hpp"
#include "../Type/color.hpp"
#include "../Type/Pose2d.hpp"

#include "GridMapPyramid.hpp"
#include "../tic_toc.h"

#include <float.h>
namespace hectorslam {

class FrontEndEstimator {
public:
    FrontEndEstimator(const std::string& config_path) {
        YAML::Node yaml = YAML::LoadFile(config_path);
        /* 构建初始地图 */
        GridMapPyramid::Option grid_map_pyramid_option;
        grid_map_pyramid_option.bottom_resolution = 
            yaml["grid_map_pyramid"]["bottom_resolution"].as<float>();
        grid_map_pyramid_option.num_depth = 
            yaml["grid_map_pyramid"]["map_depth"].as<float>();
        grid_map_pyramid_option.map_sizeX = 
            yaml["grid_map_pyramid"]["map_size"]["x"].as<float>();   // 真实的物理尺寸  单位 m
        grid_map_pyramid_option.map_sizeY = 
            yaml["grid_map_pyramid"]["map_size"]["y"].as<float>();   // 真实的物理尺寸  单位 m
        grid_map_pyramid_option.min_distance_to_boundary = 
            yaml["sensor"]["laser"]["valid_distance"].as<float>();

        grid_map_pyramid_ = new GridMapPyramid(grid_map_pyramid_option);
        hectorScanMatcher::Option option; 
        hector_matcher_ = new hectorScanMatcher(option);
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
        
        util::DataDispatcher::GetInstance().Publish("lidarOdomExt", laser_odom_ext);
        // primeLaserOdomExtrinsic_ = util::get3DTransformForState(laser_odom_ext);
        primeLaserOdomExtrinsic_ = util::getTransformForState(
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

    ~FrontEndEstimator() {
        delete grid_map_pyramid_;
    }

    void InputLaser(LaserPointCloud::ptr& laser_ptr) {
        laser_sm_.lock();   // 写锁
        laser_cache_.push_back(std::move(laser_ptr));
        laser_sm_.unlock(); 
    }

    void InputWheelOdom(WheelOdom& data) {
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

    void InputImu(ImuData& data) {
        imu_sm_.lock();
        imu_cache_.push_back(std::move(data));
        imu_sm_.unlock();  
    }

    // 设置概率、距离阈值参数
    void SetUpdateFactorFree(float free_factor) { grid_map_pyramid_->setUpdateFactorFree(free_factor); };
    void SetUpdateFactorOccupied(float occupied_factor) { grid_map_pyramid_->setUpdateFactorOccupied(occupied_factor); };
    void SetMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; };
    void SetMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; };

    // 获取地图层数
    int GetMapLevels() const { return grid_map_pyramid_->getMapLevels(); };
    // 上一次匹配到的位姿
    const TimedPose2d& GetLastFusionPose() { return last_fusionOdom_pose_; };
    // 获取指定图层地图的常量引用
    const GridMap& GetGridMap(int mapLevel = 0) const { return grid_map_pyramid_->getGridMap(mapLevel); };
    // 获取指定图层的锁
    std::mutex* GetMapMutex(int i) { return grid_map_pyramid_->getMapMutex(i); };
    // 上一次匹配的协方差
    const Eigen::Matrix3f& GetLastScanMatchCovariance() const { return lastScanMatchCov; };
protected:

    /**
     * @brief: 融合线程
     */    
    void run() {
        uint16_t wait_time = 0;
        bool imu_used_ = false;
        bool wheel_used_ = false;  
        std::deque<WheelOdom> wheel_odom_selected; 
        std::deque<ImuData> imu_selected;  
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
                            std::cout << common:: RED << "---------------------wheel data loss !-----------------------" 
                            << common::RESET << std::endl; 
                        }
                        if (!imu_cache_.empty() &&
                                 imu_cache_.back().time_stamp_ < curr_laser_ptr->end_time_) {
                            imu_cache_.clear();  
                            std::cout << common:: RED << "---------------------imu data loss !-----------------------" 
                            << common::RESET << std::endl; 
                        }
                    }
                    wait_time = 0; 
                    // 进行EKF预测   
                    // laser-imu-wheel模式： imu预测+laser&wheel校正
                    // imu-laser模式：imu预测+laser校正
                    // wheel-laser模式：wheel预测+laser校正
                    // 纯laser模式：运动模型预测+laser校正
                    DiffModelDeadReckoning diff_model;  // 差分模型
                    Pose2d predict_incre_pose;   // 预测帧间的运动 
                    bool use_motion_model_predict = true;  
                
                    if (imu_selected.empty()) {
                        // 如果 有轮速数据，且odom与激光的外参已知  那么用轮速进行预测
                        if (!wheel_odom_selected.empty()) {
                            if (has_odomExtrinsicParam_) {
                                std::cout << common::GREEN << "使用轮速计预测 ..." << common::RESET << std::endl; 
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
                        if (ekf_estimator_ != nullptr && imu_initialized_) {
                            // 有IMU数据执行IMU的预测
                            std::cout << common::GREEN << "使用IMU预测 ..." << common::RESET << std::endl;
                            use_motion_model_predict = false;   
                            // 使用IMU的角速度
                            for (uint16_t i = 0; i < imu_selected.size(); ++i) {
                                const auto& curr_data = imu_selected[i]; 
                                // EKF预测
                                ekf_estimator_->Predict(curr_data.angular_v_[2], 0.25, curr_data.time_stamp_, diff_model);  
                            }
                        } else {
                            std::cout << common::YELLOW << "IMU未初始化 ..." << common::RESET << std::endl;
                        }
                    }

                    if (use_motion_model_predict) {
                        if (ekf_estimator_ != nullptr) {
                            ekf_estimator_->Predict(curr_laser_ptr->start_time_, curr_laser_ptr->end_time_, diff_model);
                        }
                    } 

                    predict_incre_pose = diff_model.ReadLastPose().pose_;
                    // 去除laser的畸变
                    LaserUndistorted(curr_laser_ptr, diff_model.GetPath());
                    // 发布去畸变后的点云
                    // util::DataDispatcher::GetInstance().Publish("undistorted_pointcloud", curr_laser_ptr); 
                    Pose2d predict_odom_pose = last_fusionOdom_pose_.pose_ * predict_incre_pose;   // To<-last * Tlast<-curr
                    last_predictOdom_pose_.pose_ = last_predictOdom_pose_.pose_ * predict_incre_pose;
                    last_predictOdom_pose_.time_stamp_ = curr_laser_ptr->end_time_; 
                    // 发布轮速运动解算结果
                    // util::DataDispatcher::GetInstance().Publish("WheelDeadReckoning", last_predictOdom_pose_); 
                    // 将odom系下的预测位姿转换到laser odom下
                    Pose2d new_estimate_laserOdom_pose;
                    poseOdomToPrimeLaserOdom(predict_odom_pose, new_estimate_laserOdom_pose);
                    // 基于栅格金字塔快速检测预测是否可靠，判断是否存在里程计打滑等现象
                    // fastPredictFailureDetection(curr_laser_ptr, new_estimate_laserOdom_pose, grid_map_pyramid_->getGridMap(1)); 
                    // std::cout << "odom predict: " << predict_odom_pose.vec().transpose() << std::endl;
                    // 将点云数据转换为激光多分辨率金字塔数据
                    getLaserPyramidData(*curr_laser_ptr);  
                    // tt.toc("getGridPyramidData");
                    TicToc tt; 
                    //pointProjectionAndClassification(curr_laser_ptr, new_estimate_laserOdom_pose, grid_map_pyramid_->GetGridMap(2));
                    new_estimate_laserOdom_pose.SetVec(hector_matcher_->Solve(new_estimate_laserOdom_pose.vec(), 
                        laser_pyramid_container_, *grid_map_pyramid_, lastScanMatchCov)); 
                    // 细匹配  ICP / NDT 

                    tt.toc("match solve");
                    // tt.tic();
                    // 将pose转换到odom系下
                    Pose2d new_estimate_odom_pose; 
                    posePrimeLaserToOdom(new_estimate_laserOdom_pose, new_estimate_odom_pose);
                    // 在IMU&odom初始化前，这里执行的是基于先验运动模型的EKF
                    if (ekf_estimate_enable_ && ekf_estimator_ != nullptr) {
                        // 观测协方差矩阵
                        Eigen::Matrix3f obs_cov = Eigen::Matrix3f::Zero();
                        obs_cov(0, 0) = 0.0001;    // x方向方差   0.01 * 0.01
                        obs_cov(1, 1) = 0.0001;    // y 方向方差 0.01 * 0.01
                        obs_cov(2, 2) = 0.000003;    // yaw方差   0.1度
                        // 进行观测校正
                        ekf_estimator_->Correct(new_estimate_odom_pose, obs_cov, curr_laser_ptr->end_time_); 
                        new_estimate_odom_pose = ekf_estimator_->ReadPosterioriPose();
                        // 更新对应laser坐标
                        poseOdomToPrimeLaserOdom(new_estimate_odom_pose, new_estimate_laserOdom_pose);
                    }
                    // 利用栅格金子塔进行动态点云检测
                    std::pair<std::vector<Eigen::Vector2f>, double> dynamic_points_data;    // 动态点
                    std::pair<std::vector<Eigen::Vector2f>, double>  stable_points_data;   // 静态点 
                    std::pair<std::vector<Eigen::Vector2f>, double>  undetermined_points_data;  // 待定点
                    dynamic_points_data.first.reserve(curr_laser_ptr->pointcloud_.size());
                    dynamic_points_data.second = curr_laser_ptr->end_time_; 
                    stable_points_data.first.reserve(curr_laser_ptr->pointcloud_.size());  
                    stable_points_data.second = curr_laser_ptr->end_time_; 
                    undetermined_points_data.first.reserve(curr_laser_ptr->pointcloud_.size());  
                    undetermined_points_data.second = curr_laser_ptr->end_time_; 
                    pointProjectionAndClassification(curr_laser_ptr, new_estimate_laserOdom_pose, grid_map_pyramid_->getGridMap(2), 
                        dynamic_points_data.first, stable_points_data.first, undetermined_points_data.first);
                    // 系统初始化是指 联合imu和wheel的多传感器初始化
                    // 系统初始化只要有imu或wheel两者之一就可以，系统初始化后，
                    // 即使之后出现新的传感器数据，也不会重新初始化
                    if (!system_initialized_) {
                        systemInit(new_estimate_laserOdom_pose, imu_selected, wheel_odom_selected, curr_laser_ptr->end_time_);
                    }

                    last_fusionOdom_pose_.pose_ = new_estimate_odom_pose;
                    last_fusionOdom_pose_.time_stamp_ = curr_laser_ptr->end_time_; 
                    // std::cout << "odom correct : " << new_estimate_odom_pose.vec().transpose() << std::endl;
                    /** 2.地图更新(last_map_updata_pose_初始化为一个很大的值，因此第一帧点云就会更新地图) **/
                    if (util::poseDifferenceLargerThan(new_estimate_laserOdom_pose.vec(), last_map_updata_pose_.vec(), 
                            paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate)) { 
                        // 仅在位姿变化大于阈值 或者 map_without_matching为真 的时候进行地图更新
                        grid_map_pyramid_->updateByScan(laser_pyramid_container_, new_estimate_laserOdom_pose.vec());
                        grid_map_pyramid_->onMapUpdated();
                        local_map_.UpdateLocalMapForMotion(curr_laser_ptr->pointcloud_);
                        last_map_updata_pose_ = new_estimate_laserOdom_pose;
                    }
                    // 发布数据
                    util::DataDispatcher::GetInstance().Publish("fusionOdom", last_fusionOdom_pose_); 
                    util::DataDispatcher::GetInstance().Publish("dynamic_pointcloud", dynamic_points_data); 
                    util::DataDispatcher::GetInstance().Publish("stable_pointcloud", stable_points_data); 
                    util::DataDispatcher::GetInstance().Publish("local_map", local_map_.ReadLocalMap()); 

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
     * @brief: 快速预测失效检测
     * @details: 主要用于检测如轮子打滑，碰撞等预测失效的情况
     *                      具体算法：在当前激光点中，按照角度分布均匀抽取100个点进行检测，
     *                                              将这些点按照预测位姿投影到低分辨率(>10cm)的占据栅格中，计算占据/未占据的比率，
     *                                              若该比值过小，认为预测pose有误
     * @param laser_ptr 激光点云 已经去除畸变
     * @param pose 预测的位姿
     * @param occGridMap 栅格工具  
     * @return {*}
     */    
    bool fastPredictFailureDetection(const LaserPointCloud::Ptr& laser_ptr, 
                                                            const Pose2d& pose, 
                                                            const GridMap& occGridMap) {
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
            Eigen::Vector2f point_gridmap_pos = occGridMap.getMapCoords(point_laserOdom_pos);     // laserOdom -> map

            if (!occGridMap.pointOutOfMapBounds(point_gridmap_pos)) {
                if (occGridMap.isOccupied(round(point_gridmap_pos[0]), round(point_gridmap_pos[1]))) {
                    inlier_point_num++;
                } else if (occGridMap.isFree(round(point_gridmap_pos[0]), round(point_gridmap_pos[1]))) {
                    outlier_point_num++;
                } else {
                    unknow_point_num++;
                }
            }
        }

        std::cout << common::YELLOW << "unknow_point_num: " << unknow_point_num << std::endl;
        std::cout << common::YELLOW << "inlier_point_num: " << inlier_point_num << std::endl;
        std::cout << common::YELLOW << "outlier_point_num: " << outlier_point_num << std::endl;
        return false; 
    }

    /**
     * @brief: 激光点投影到世界坐标并且进行状态分类
     * @details 将点分为 ：动态点、稳定点、待定点 
     * @param pose 当前laser 在 laserOdom系下的位姿  
     * @param[out] dynamic_points 动态点   即击中空白栅格 且空白栅格的占据概率低与阈值
     * @param[out] stable_points 稳定点   即击中占据栅格
     * @param[out] undetermined_points 待定点  即击中未知栅格以及击中占据概率高与阈值的空白栅格的点
     * @return {*}
     */    
    void pointProjectionAndClassification(const LaserPointCloud::Ptr& laser_ptr, 
                                                            const Pose2d& pose, 
                                                            const GridMap& occGridMap,
                                                            std::vector<Eigen::Vector2f>& dynamic_points,
                                                            std::vector<Eigen::Vector2f>& stable_points,
                                                            std::vector<Eigen::Vector2f>& undetermined_points) {
        for (auto& point : laser_ptr->pointcloud_) {
            point.pos_ = pose * point.pos_;      //  转到laserOdom系 
            Eigen::Vector2f point_gridmap_pos = occGridMap.getMapCoords(point.pos_);     // laserOdom -> map
            // 超过范围的点一律丢掉
            if (occGridMap.pointOutOfMapBounds(point_gridmap_pos)) {
                continue;      
            }
            // 如果当前的激光点击中的占据栅格是空的，说明这是动态点或者不稳定点
            // if (occGridMap.isFree(round(point_gridmap_pos[0]), round(point_gridmap_pos[1]))) {
            //     dynamic_points.push_back(point);  
            // } 
            if (occGridMap.isOccupied(round(point_gridmap_pos[0]), round(point_gridmap_pos[1]))) {
                stable_points.push_back(point.pos_);     // 在一定距离内  且静态的点叫稳定点
            } else if (occGridMap.isFree(round(point_gridmap_pos[0]), round(point_gridmap_pos[1]))) {
                dynamic_points.push_back(point.pos_);  
            } else {
                undetermined_points.push_back(point.pos_);  
            }
        }
    }

    /**
     * @brief: 
     * @details: 
     * @return {*}
     */    
    void systemInit(const Pose2d& laser_pose, const std::deque<ImuData>& imu_selected,
            const std::deque<WheelOdom>& wheel_odom_selected, const double& time_stamp) {
        // 初始化
        static Pose2d last_laser_pose = laser_pose; 
        Pose2d pose_increm = last_laser_pose.inv() * laser_pose; 
        last_laser_pose = laser_pose; 
        // 先初始化imu bias
        // 然后laser 里程计 和 imu&wheel 标定外参
        if (!imu_selected.empty() && !imu_initialized_) {
            // IMU初始化 
            if (imu_initializer_.Init(pose_increm, imu_selected)) {
                imu_initialized_ = true; 
                system_initialized_ = true; // 
                ekf_estimator_ = std::make_unique<EKFEstimatorPVQB>(laser_pose, 0, 0, imu_initializer_.GetYawBias(), 1e-4, time_stamp); 
            }
        } else if (!wheel_odom_selected.empty()) {
            if (!has_odomExtrinsicParam_) {

            } else {
                system_initialized_ = true; 
                ekf_estimator_ = std::make_unique<EKFEstimatorBase>(laser_pose, 0, 0, time_stamp); 
                std::cout << common::GREEN << "系统初始化完毕，轮速-laser融合！" 
                << common::RESET << std::endl;
            }
        }
        // 如果之前imu和轮速数据没有初始化成功，那么这里也要给滤波器进行初始化，
        // 之后将执行融合运动学模型的EKF纯激光里程计
        if (ekf_estimate_enable_) {
            if (ekf_estimator_ == nullptr) { 
                // 进行估计器初始化
                ekf_estimator_ = std::make_unique<EKFEstimatorBase>(laser_pose, 0, 0, time_stamp); 
            }
        }
    }

    /**
     * @brief: 传感器的数据同步-提取出laser一帧时间戳内的其他sensor数据
     * @return 当前激光的数据同步是否完成；
     *                    true: 1、除了激光雷达数据外没有轮速和imu数据，那么直接返回true
     *                               2、成功提取出了包含laser帧的imu和轮速数据
     *                    false: 当存在除了激光雷达外的传感器数据，且该传感器最后数据的时间戳要早于
     *                                激光末尾时间戳，则返回false，延迟该激光帧的处理时间，等待其他传感器数据覆盖。
     */    
    bool syncSensorData(const LaserPointCloud::Ptr& laser_ptr, 
                                                std::deque<WheelOdom>& wheelOdom_container,
                                                std::deque<ImuData>& imu_container) {
        bool wheel_extract_finish = true;
        bool imu_extract_finish = true; 
        // 提取轮速数据
        wheel_sm_.lock();
        if (!extractSensorData<WheelOdom>(wheelOdom_cache_, wheelOdom_container,
                laser_ptr->start_time_, laser_ptr->end_time_)) {
            wheel_extract_finish = false; 
        }
        wheel_sm_.unlock();
        // 提取IMU数据
        imu_sm_.lock();
        if (!extractSensorData<ImuData>(imu_cache_, imu_container,
                laser_ptr->start_time_, laser_ptr->end_time_)) {
            imu_extract_finish = false; 
        }
        imu_sm_.unlock();
        return wheel_extract_finish && imu_extract_finish; 
    }

    /**
     * @brief: 从目标容器中提取出时间范围内的数据
     * @details: 处理后，data_cache 的第一个数据就要晚于 start_time 
     * @param data_cache 目标容器
     * @param extracted_container 提取数据放置的容器
     * @param start_time 最早时间
     * @param end_time 最晚时间
     * @return 提取是否完成 
     */    
    template<class DataT_>
    bool extractSensorData(std::deque<DataT_>& data_cache, std::deque<DataT_>& extracted_container,
            const double& start_time, const double& end_time) {
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
                    // 如果轮速最后一个数据的时间戳距离laser最后一个点的时间较远  那么向后寻找一个更接近的轮速数据
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
                    // 最后一个轮速数据的时间戳和激光最后一个点的时间戳对齐 
                    extracted_container.back().time_stamp_ = end_time; 
                } else {
                    return false;  
                }
            }
        }

        return true;  
    }

    // 激光雷达的运动转到Odom系
    void posePrimeLaserToOdom(const Pose2d& pose_in_laser, Pose2d& pose_in_world) {
        pose_in_world.SetX(pose_in_laser.x());
        if (prime_laser_upside_down_) {
            pose_in_world.SetY(-pose_in_laser.y());
            pose_in_world.SetRotation(-pose_in_laser.yaw());
        }
        // pose_in_laser = primeLaserOdomExtrinsic_ * pose_in_laser; 
    }

    // odom系的运动转到激光系 
    void poseOdomToPrimeLaserOdom(const Pose2d& pose_in_odom, Pose2d& pose_in_laser) {
        pose_in_laser.SetX(pose_in_odom.x());
        if (prime_laser_upside_down_) {
            pose_in_laser.SetY(-pose_in_odom.y());
            pose_in_laser.SetRotation(-pose_in_odom.yaw());
        }
        // pose_in_laser = primeLaserOdomExtrinsic_ * pose_in_laser; 
    }

    /** slam系统重置 **/
    void reset() {
        last_map_updata_pose_.SetVec(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX));
        last_fusionOdom_pose_.pose_.SetVec(Eigen::Vector3f::Zero());
        //重置地图
        grid_map_pyramid_->reset();
    }

    /**
     * @brief: 将当前的输入点云 转换成2D栅格金字塔数据 
     * @details: 转换的结果是多分辨率金字塔激光laser_pyramid_container_，由于对于点云匹配任务，
     *                      需要激光点在多分辨率栅格地图的具体位置，因此这里生成的多分辨率激光需要是浮点型的
     * @param pointcloud 输入点云，若有多激光的话，
     *                                            将其他激光的数据变换到主激光坐标系下再合成为一组数据。
     *                                             实际观测位置和该点云坐标原点重和  
     * @return {*}
     */    
    void getLaserPyramidData(const LaserPointCloud& pointcloud) {
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
            laser_pyramid_container_[index].setFrom(laser_pyramid_container_[0], static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));
        }
    }

    float getScaleToMap() const { return grid_map_pyramid_->getScaleToMap(); };    // 返回第 0层的scale  

private:

    GridMapPyramid* grid_map_pyramid_; // 地图接口对象--纯虚类进行
    PointcloudLocalMap local_map_;  
    hectorScanMatcher* hector_matcher_;
    std::vector<LaserPointContainer> laser_pyramid_container_;  /// 不同图层对应的激光数据
    
    bool prime_laser_upside_down_; // 主雷达颠倒 
    bool ekf_estimate_enable_ = true;    // 默认使用ekf估计  
    bool system_initialized_ = false;  
    bool imu_initialized_ = false;
    bool has_odomExtrinsicParam_ = true; 
    Eigen::Isometry2f primeLaserOdomExtrinsic_;  
    // Eigen::Isometry3f primeLaserOdomExtrinsic_;  
    Pose2d last_map_updata_pose_;
    TimedPose2d last_fusionOdom_pose_;
    TimedPose2d last_predictOdom_pose_;
    Eigen::Matrix3f lastScanMatchCov;
    float linear_v_ = 0;
    float rot_v_ = 0; 

    float paramMinDistanceDiffForMapUpdate;
    float paramMinAngleDiffForMapUpdate;

    std::deque<WheelOdom> wheelOdom_cache_;  
    std::deque<ImuData> imu_cache_;  
    std::deque<LaserPointCloud::Ptr> laser_cache_;   

    std::shared_mutex wheel_sm_;  
    std::shared_mutex imu_sm_;  
    std::shared_mutex laser_sm_;  

    std::unique_ptr<EKFEstimatorBase> ekf_estimator_; 
    ImuInitializer2D imu_initializer_; 

    std::thread run_;  
};
}

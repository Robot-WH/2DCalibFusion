
#ifndef _hectorslamprocessor_h__
#define _hectorslamprocessor_h__

#include <yaml-cpp/yaml.h>
#include "predictor.hpp"
#include "corrector.hpp"
#include "motionModel.hpp"
#include "../Map/GridMap.h"
#include "../Map/OccGridMapUtilConfig.h"
#include "../ScanMatcher/hectorScanMatcher.hpp"
#include "../Sensor/LaserPointContainer.h"
#include "../Sensor/SensorData.hpp"
#include "../Lib/Algorithm/Pointcloud/laserUndistorted.hpp"

#include "../util/UtilFunctions.h"
#include "../util/MapLockerInterface.h"
#include "../util/DataDispatcher.hpp"
#include "../util/utility.hpp"
#include "../Type/color.hpp"
#include "../Type/Pose2d.hpp"

#include "MapRepresentationInterface.h"
#include "MapRepMultiMap.h"
#include "../ScanMatcher/hectorScanMatcher_1.hpp"
#include "../tic_toc.h"

#include <float.h>

namespace hectorslam {
class FrontEndEstimator {
public:
    FrontEndEstimator(const std::string& config_path) {
        YAML::Node yaml = YAML::LoadFile(config_path);
        /* 构建初始地图 */
        MapRepMultiMap::Option grid_map_pyramid_option;
        grid_map_pyramid_option.bottomResolution = 
            yaml["grid_map_pyramid"]["bottom_resolution"].as<float>();
        grid_map_pyramid_option.numDepth = 
            yaml["grid_map_pyramid"]["map_depth"].as<float>();
        grid_map_pyramid_option.mapSizeX = 
            yaml["grid_map_pyramid"]["map_size"]["x"].as<float>();
        grid_map_pyramid_option.mapSizeY = 
            yaml["grid_map_pyramid"]["map_size"]["y"].as<float>();

        mapRep = new MapRepMultiMap(grid_map_pyramid_option);
        hectorScanMatcher::Option option; 
        hector_matcher_ = new hectorScanMatcher(option);
        dataContainers.resize(mapRep->GetMapLevels());
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
    }

    ~FrontEndEstimator() {
        delete mapRep;
    }

    void InputLaser(LaserPointCloud::ptr& laser_ptr) {
        TicToc tt; 
        laser_cache_.push_back(std::move(laser_ptr));
        if (laser_cache_.size() < 2) return;    // 为了保证imu/wheel的数据能彻底覆盖laser，故不处理最新帧
        auto& curr_laser_ptr = laser_cache_.front();  
        // 提取出包围laser的odom和imu数据 
        std::deque<WheelOdom> wheel_odom_selected; 
        std::deque<ImuData> imu_selected;  
        syncSensorData(curr_laser_ptr, wheel_odom_selected);  
        // 进行EKF预测   
        // laser-imu-wheel模式： imu预测+laser&wheel校正
        // imu-laser模式：imu预测+laser校正
        // wheel-laser模式：wheel预测+laser校正
        // 纯laser模式：运动模型预测+laser校正
        DiffModelDeadReckoning diff_model;  // 差分模型
        Pose2d predict_incre_pose;   // 预测帧间的运动 
        if (imu_selected.empty()) {
            if (wheel_odom_selected.empty()) {
                // 没有轮速数据则用运动学模型预测
            } else {
                // 使用轮速进行预测
                for (uint16_t i = 0; i < wheel_odom_selected.size(); ++i) {
                    const auto& curr_data = wheel_odom_selected[i]; 
                    // 运动学前向传播
                    diff_model.Update(curr_data.time_stamp_, curr_data.v_x_, curr_data.omega_yaw_); 
                    // EKF预测
                }
                predict_incre_pose = diff_model.ReadLastPose().pose_;
            }
        } else {
            // 有IMU数据执行IMU的预测
        }
        // 去除laser的畸变
        LaserUndistorted(curr_laser_ptr, diff_model.GetPath());
        // 发布去畸变后的点云
        util::DataDispatcher::GetInstance().Publish("undistorted_pointcloud", curr_laser_ptr); 
        Pose2d predict_odom_pose = last_fusionOdom_pose_.pose_ * predict_incre_pose;   // To<-last * Tlast<-curr
        std::cout << "odom predict: " << predict_odom_pose.Vec().transpose() << std::endl;
        
        // 将点云数据转换为栅格金字塔数据
        getGridPyramidData(*curr_laser_ptr);  
        // tt.toc("getGridPyramidData");
        // tt.tic();
        // 将odom系下的预测位姿转换到laser odom下
        Pose2d new_estimate_laserOdom_pose;
        poseOdomToPrimeLaserOdom(predict_odom_pose, new_estimate_laserOdom_pose);
        new_estimate_laserOdom_pose.Vec() = 
            hector_matcher_->Solve(new_estimate_laserOdom_pose.Vec(), dataContainers, *mapRep, lastScanMatchCov); 
        // tt.toc("match solve");
        // tt.tic();
        // 将pose转换到odom系下
        Pose2d new_estimate_odom_pose; 
        posePrimeLaserToOdom(new_estimate_laserOdom_pose, new_estimate_odom_pose);
        // 进行观测校正
        last_fusionOdom_pose_.pose_ = new_estimate_odom_pose;
        last_fusionOdom_pose_.time_stamp_ = curr_laser_ptr->end_time_; 
        std::cout << "odom correct : " << new_estimate_odom_pose.Vec().transpose() << std::endl;
        // 进行ESKF校正，若退化 ，则用wheelOdom数据插值进行校正

        /** 2.地图更新 **/
        if (util::poseDifferenceLargerThan(new_estimate_laserOdom_pose.Vec(), last_map_updata_pose_.Vec(), 
                paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate)) { 
            // 仅在位姿变化大于阈值 或者 map_without_matching为真 的时候进行地图更新
            mapRep->updateByScan(dataContainers, new_estimate_laserOdom_pose.Vec());
            mapRep->onMapUpdated();
            last_map_updata_pose_ = new_estimate_laserOdom_pose;
        }
        // 输出融合结果 
        laser_cache_.pop_front(); 
        // tt.toc("map update");
    }

    void InputWheelOdom(WheelOdom& data) {
        static DiffModelDeadReckoning diff_model; 
        wheel_mt_.lock();   // 写锁 
        wheelOdom_cache_.push_back(data);
        diff_model.Update(data.time_stamp_, data.v_x_, data.omega_yaw_);
        // 发布轮速运动解算结果
        util::DataDispatcher::GetInstance().Publish("WheelDeadReckoning", diff_model.ReadLastPose().pose_); 
        wheel_mt_.unlock();  
        // 检测laser是否有数据， 若laser 没数据， 则退化为 odom/imu融合里程计
        wheel_mt_.lock_shared();    // 读锁
        if (laser_cache_.empty()) {
            if (wheelOdom_cache_.back().time_stamp_ - wheelOdom_cache_.front().time_stamp_ > 0.5) {
                model_ = MODEL::WHEEL_IMU;
                std::cout << "INFO: 没有激光数据，切换到wheel + imu 模式..." << std::endl; 
                // 进行初始化
            }
        } else {
            if (wheelOdom_cache_.back().time_stamp_ - laser_cache_.back()->start_time_ > 0.5) {
                model_ = MODEL::WHEEL_IMU;
            }
        }
        wheel_mt_.unlock_shared();
        // 若为imu/wheel融合 - wheel提供线速度，imu提供角速度形成组合里程计，此外在线估计陀螺仪bias
        if (model_ == MODEL::WHEEL_IMU) {
        }
    }

    void InputImu(const ImuData& data) {
        imu_cache_.push_back(data);
    }

     // 设置概率、距离阈值参数
    void SetUpdateFactorFree(float free_factor) { mapRep->setUpdateFactorFree(free_factor); };
    void SetUpdateFactorOccupied(float occupied_factor) { mapRep->setUpdateFactorOccupied(occupied_factor); };
    void SetMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; };
    void SetMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; };

    /**
     * @brief: 提取出laser一帧时间戳内的其他sensor数据
     * @return {*}
     */    
    void syncSensorData(const LaserPointCloud::Ptr& laser_ptr, 
                                                std::deque<WheelOdom>& wheelOdom_container) {
        wheel_mt_.lock_shared();  
        if (!wheelOdom_cache_.empty()) {
            // 首先轮速计的数据要包围住laser帧 (1e-3 内的偏差可以接受)
            if (wheelOdom_cache_.front().time_stamp_ <= laser_ptr->start_time_ && 
                    wheelOdom_cache_.back().time_stamp_ >= laser_ptr->end_time_) {
                // 添加第一个头数据 
                wheel_mt_.unlock_shared();  
                wheel_mt_.lock();  
                WheelOdom begin_data;     // 时间戳 <= 激光起始时间戳的第一个数据
                while (wheelOdom_cache_.front().time_stamp_  <= laser_ptr->start_time_) {
                    begin_data = wheelOdom_cache_.front();
                    wheelOdom_cache_.pop_front();  
                }
                // 若这个begin_data 距离 激光起始时间太远了，那么需要进一步找begin_data
                if (laser_ptr->start_time_ - begin_data.time_stamp_ > 1e-3) {
                    // 先直接看下一个数据 是否距离激光起始时间足够近
                    if (wheelOdom_cache_.front().time_stamp_ - laser_ptr->start_time_ < 1e-3) {
                        begin_data = wheelOdom_cache_.front();
                        wheelOdom_cache_.pop_front();  
                    } else {
                        // 插值
                        begin_data = util::LinearInterpolate(begin_data, wheelOdom_cache_.front(), 
                                                                        begin_data.time_stamp_ , wheelOdom_cache_.front().time_stamp_, 
                                                                        laser_ptr->start_time_);
                    }
                }
                begin_data.time_stamp_ = laser_ptr->start_time_;
                wheel_mt_.unlock();  
                wheelOdom_container.push_back(begin_data);   // 放置第一个数据
                wheel_mt_.lock_shared();  
                // 添加中间段的数据
                auto wheelOdom_ptr = wheelOdom_cache_.begin();  
                for (; wheelOdom_ptr->time_stamp_ <= laser_ptr->end_time_; ++wheelOdom_ptr) {
                    wheelOdom_container.push_back(*wheelOdom_ptr);
                }
                // 如果轮速最后一个数据的时间戳距离laser最后一个点的时间较远  那么向后寻找一个更接近的轮速数据
                if (laser_ptr->end_time_ - wheelOdom_container.back().time_stamp_ > 1e-3) {
                    if (wheelOdom_ptr->time_stamp_ - laser_ptr->end_time_ < 1e-3) {
                        wheelOdom_container.push_back(*wheelOdom_ptr); 
                    } else {
                        // 插值
                        WheelOdom end_data = util::LinearInterpolate(wheelOdom_container.back(), *wheelOdom_ptr, 
                            wheelOdom_container.back().time_stamp_ , wheelOdom_ptr->time_stamp_, 
                            laser_ptr->end_time_);
                        wheelOdom_container.push_back(end_data); 
                    }
                }
                wheelOdom_container.back().time_stamp_ = laser_ptr->end_time_; 
                // std::cout << common::GREEN << "以匹配完成轮速和laser" << common::RESET << std::endl;
                // for (const auto& data : wheelOdom_container) {
                //     std::cout << common::GREEN << "time: " << data.time_stamp_ << common::RESET << std::endl;
                // }
                // std::cout << common::GREEN << "wheelOdom_cache_.front().time_stamp_: " 
                // << wheelOdom_cache_.front().time_stamp_ << common::RESET << std::endl;
            } else {
                // std::cout << common::RED << "轮速数据无法覆盖laser!!" << common::RESET << std::endl;
                // std::cout << std::setprecision(15) << "wheelOdom_cache_.front().time_stamp_:" << wheelOdom_cache_.front().time_stamp_ <<std::endl;
                // std::cout << std::setprecision(15) << "laser_ptr->start_time_:" << laser_ptr->start_time_ <<std::endl;
                // std::cout << std::setprecision(15) << "wheelOdom_cache_.back().time_stamp_:" << wheelOdom_cache_.back().time_stamp_ <<std::endl;
                // std::cout << std::setprecision(15) << "laser_ptr->end_time_:" << laser_ptr->end_time_ <<std::endl;
            }
        }
        wheel_mt_.unlock_shared();  
    }

    // 激光雷达的运动转到Odom系
    void posePrimeLaserToOdom(const Pose2d& pose_in_laser, Pose2d& pose_in_world) {
        pose_in_world.SetX(pose_in_laser.GetX());
        if (prime_laser_upside_down_) {
            pose_in_world.SetY(-pose_in_laser.GetY());
            pose_in_world.SetRotation(-pose_in_laser.GetYaw());
        }
        // pose_in_laser = primeLaserOdomExtrinsic_ * pose_in_laser; 
    }

    // odom系的运动转到激光系 
    void poseOdomToPrimeLaserOdom(const Pose2d& pose_in_odom, Pose2d& pose_in_laser) {
        pose_in_laser.SetX(pose_in_odom.GetX());
        if (prime_laser_upside_down_) {
            pose_in_laser.SetY(-pose_in_odom.GetY());
            pose_in_laser.SetRotation(-pose_in_odom.GetYaw());
        }
        // pose_in_laser = primeLaserOdomExtrinsic_ * pose_in_laser; 
    }

    /** slam系统重置 **/
    void reset() {
        last_map_updata_pose_.Vec() = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
        last_fusionOdom_pose_.pose_.Vec() = Eigen::Vector3f::Zero();
        //重置地图
        mapRep->reset();
    }

    /**
     * @brief: 将当前的输入点云 转换成2D栅格金字塔数据 
     * @param pointcloud 输入点云，若有多激光的话，
     *                                            将其他激光的数据变换到主激光坐标系下再合成为一组数据。
     *                                             实际观测位置和该点云坐标原点重和  
     * @return {*}
     */    
    void getGridPyramidData(const LaserPointCloud &pointcloud) {
        // 第一层
        dataContainers[0].clear(); 
        uint16_t num = pointcloud.pointcloud_.size(); 
        // std::cout << "getGridPyramidData size: " << num << std::endl;
        for (uint16_t i = 0; i < num; ++i) {
            dataContainers[0].add(pointcloud.pointcloud_[i].pos_ * getScaleToMap());    // 四舍五入   原点在grid中心  
        }
        // 其他层  
        size_t size = mapRep->GetMapLevels();
        for (int index = size - 1; index > 0; --index) {
            // 根据第0层地图层数，求取了在index层激光的数据
            dataContainers[index].setFrom(dataContainers[0], static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));
        }
    }

    // 上一次匹配到的位姿
    const TimedPose2d& getLastFusionPose() { return last_fusionOdom_pose_; };
    // 上一次匹配的协方差
    const Eigen::Matrix3f &getLastScanMatchCovariance() const { return lastScanMatchCov; };
    float getScaleToMap() const { return mapRep->getScaleToMap(); };    // 返回第 0层的scale  

    // 获取地图层数
    int GetMapLevels() const { return mapRep->GetMapLevels(); };
    // 获取指定图层地图的常量引用
    const GridMap &getGridMap(int mapLevel = 0) const { return mapRep->getGridMap(mapLevel); };
    // 给指定图层添加互斥锁
    void addMapMutex(int i, MapLockerInterface *mapMutex) { mapRep->addMapMutex(i, mapMutex); };
    // 获取指定图层的锁
    MapLockerInterface *getMapMutex(int i) { return mapRep->getMapMutex(i); };

private:
    enum class MODEL {   
        PURE_LASER = 1,  // 纯激光
        LASER_IMU, 
        LASER_WHEEL,
        WHEEL_IMU,
        LASER_WHEEL_IMU
    } model_;
    MapRepMultiMap *mapRep; // 地图接口对象--纯虚类
    hectorScanMatcher *hector_matcher_;
    std::vector<LaserPointContainer> dataContainers;  /// 不同图层对应的激光数据
    
    bool prime_laser_upside_down_; // 主雷达颠倒 
    Eigen::Isometry2f primeLaserOdomExtrinsic_;  
    // Eigen::Isometry3f primeLaserOdomExtrinsic_;  
    Pose2d last_map_updata_pose_;
    TimedPose2d last_fusionOdom_pose_;
    Eigen::Matrix3f lastScanMatchCov;

    float paramMinDistanceDiffForMapUpdate;
    float paramMinAngleDiffForMapUpdate;

    std::deque<WheelOdom> wheelOdom_cache_;  
    std::deque<ImuData> imu_cache_;  
    std::deque<LaserPointCloud::Ptr> laser_cache_;   

    std::shared_mutex wheel_mt_;  
};
}
#endif
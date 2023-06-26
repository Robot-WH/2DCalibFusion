
#pragma once 
#include <float.h>
#include <yaml-cpp/yaml.h>
#include "Kalman/ekf_estimator.hpp"
#include "Initialize/2d_imu_initialize.hpp"
#include "motionModel.hpp"
#include "Calibration/2d_handeye_calibration.hpp"
#include "../util/DataDispatcher.hpp"
#include "../util/utility.hpp"
#include "../tic_toc.h"
#include "msa2d/common/color.hpp"
#include "msa2d/common/Pose2d.hpp"
#include "msa2d/common/UtilFunctions.hpp"
#include "msa2d/Map/OccGridMapPyramid.h"
#include "msa2d/Map/TimedSlidingLocalMap.h"
// #include "../Map/OccGridMapUtilConfig.h"
#include "msa2d/Sensor/LaserPointContainer.h"
#include "msa2d/Sensor/SensorData.hpp"
#include "msa2d/Sensor/point_cloud.hpp"
#include "msa2d/ScanMatcher/hectorScanMatcher.h"

namespace Estimator2D {

class FrontEndEstimator {
public:
    FrontEndEstimator(const std::string& config_path);

    ~FrontEndEstimator() {
        delete grid_map_pyramid_;
    }

    void InputLaser(msa2d::sensor::LaserScan::ptr& laser_ptr);

    void InputWheelOdom(msa2d::sensor::WheelOdom& data);

    void InputImu(msa2d::sensor::ImuData& data);

    // 设置概率、距离阈值参数
    void SetMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; };
    void SetMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; };

    // 获取地图层数
    int GetMapLevels() const { return grid_map_pyramid_->getMapLevels(); };
    // 上一次匹配到的位姿
    const msa2d::TimedPose2d& GetLastFusionPose() { return last_fusionOdom_pose_; };
    // 获取指定图层地图的常量引用
    const msa2d::map::OccGridMapBase* GetGridMap(int mapLevel = 0) const { return grid_map_pyramid_->getGridMap(mapLevel); };
    // 获取指定图层的锁
    std::mutex* GetMapMutex(int i) { return grid_map_pyramid_->getMapMutex(i); };
    // 上一次匹配的协方差
    const Eigen::Matrix3f& GetLastScanMatchCovariance() const { return lastScanMatchCov; };
protected:

    /**
     * @brief: 融合线程
     */    
    void run();

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
    bool fastPredictFailureDetection(const msa2d::sensor::LaserScan::Ptr& laser_ptr, 
                                                            const msa2d::Pose2d& pose, 
                                                            const msa2d::map::OccGridMapBase* occGridMap);

    /**
     * @brief: 激光点分类
     * @details 将点分为 ：动态点、稳定点、待定点 
     * @param laser_ptr 转到了地图坐标系的激光点 
     * @param pose 当前laser 在 laserOdom系下的位姿  
     * @param[out] dynamic_points 动态点   即击中空白栅格 且空白栅格的占据概率低与阈值
     * @param[out] stable_points 稳定点   即击中占据栅格
     * @param[out] undetermined_points 待定点  即击中未知栅格以及击中占据概率高与阈值的空白栅格的点
     * @return {*}
     */    
    void pointClassification(const msa2d::sensor::LaserScan::Ptr& laser_ptr, 
                                                            const msa2d::Pose2d& pose, 
                                                            const msa2d::map::OccGridMapBase* occGridMap,
                                                            std::vector<Eigen::Vector2f>& dynamic_points,
                                                            std::vector<Eigen::Vector2f>& stable_points,
                                                            std::vector<Eigen::Vector2f>& undetermined_points);

    bool judgeTrackingLoss(const std::vector<Eigen::Vector2f>& dynamic_points,
                                                        const std::vector<Eigen::Vector2f>& stable_points,
                                                        const std::vector<Eigen::Vector2f>& undetermined_points);

    /**
     * @brief 激光畸变去除
     * 
     * @param laser 
     * @param motion_info 
     */
    void LaserUndistorted(msa2d::sensor::LaserScan& laser, Path& motion_info); 

    /**
     * @brief: 
     * @details: 
     * @return {*}
     */    
    void systemInit(const msa2d::Pose2d& laser_pose, const std::deque<msa2d::sensor::ImuData>& imu_selected,
            const std::deque<msa2d::sensor::WheelOdom>& wheel_odom_selected, const double& time_stamp);

    /**
     * @brief: 传感器的数据同步-提取出laser一帧时间戳内的其他sensor数据
     * @return 当前激光的数据同步是否完成；
     *                    true: 1、除了激光雷达数据外没有轮速和imu数据，那么直接返回true
     *                               2、成功提取出了包含laser帧的imu和轮速数据
     *                    false: 当存在除了激光雷达外的传感器数据，且该传感器最后数据的时间戳要早于
     *                                激光末尾时间戳，则返回false，延迟该激光帧的处理时间，等待其他传感器数据覆盖。
     */    
    bool syncSensorData(const msa2d::sensor::LaserScan::Ptr& laser_ptr, 
                                                std::deque<msa2d::sensor::WheelOdom>& wheelOdom_container,
                                                std::deque<msa2d::sensor::ImuData>& imu_container);
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
            const double& start_time, const double& end_time);

    void transformPathFromOdomToLaser(Path& motion_info);
    // 激光雷达的运动转到Odom系
    void posePrimeLaserToOdom(const msa2d::Pose2d& pose_in_laser, msa2d::Pose2d& pose_in_world);

    // odom系的运动转到激光系 
    void poseOdomToPrimeLaserOdom(const msa2d::Pose2d& pose_in_odom, msa2d::Pose2d& pose_in_laser);

    /** slam系统重置 **/
    void reset();

    /**
     * @brief: 将当前的输入点云 转换成2D栅格金字塔数据 
     * @details: 转换的结果是多分辨率金字塔激光laser_pyramid_container_，由于对于点云匹配任务，
     *                      需要激光点在多分辨率栅格地图的具体位置，因此这里生成的多分辨率激光需要是浮点型的
     * @param pointcloud 输入点云，若有多激光的话，
     *                                            将其他激光的数据变换到主激光坐标系下再合成为一组数据。
     *                                             实际观测位置和该点云坐标原点重和  
     * @return {*}
     */    
    void getLaserPyramidData(const msa2d::sensor::LaserScan& pointcloud);

    float getScaleToMap() const { return grid_map_pyramid_->getScaleToMap(); };    // 返回第 0层的scale  

private:
    enum class MODE {pure_lidar, lio, lwio};  // 三种模式  纯激光，lio-激光IMU融合，lwio-激光imu轮速融合 
    MODE work_mode = MODE::pure_lidar; 
    msa2d::map::OccGridMapPyramid* grid_map_pyramid_; // 地图接口对象--纯虚类进行
    msa2d::map::PointcloudLocalMap local_map_;  
    msa2d::ScanMatcher::hectorScanMatcher* hector_matcher_;
    std::vector<msa2d::sensor::LaserPointContainer> laser_pyramid_container_;  /// 不同图层对应的激光数据
    
    bool prime_laser_upside_down_; // 主雷达颠倒 
    bool ekf_estimate_enable_ = true;    // 默认使用ekf估计  
    bool system_initialized_ = false;  
    bool imu_initialized_ = false;
    bool has_odomExtrinsicParam_ = true; 
    bool imu_calib_ = false;  
    bool track_loss_ = false;    // 位姿丢失
    Eigen::Isometry2f primeLaserOdomExtrinsic_;  
    // Eigen::Isometry3f primeLaserOdomExtrinsic_;  
    msa2d::Pose2d last_map_updata_pose_;
    msa2d::TimedPose2d last_fusionOdom_pose_;
    msa2d::Pose2d last_lidarOdom_pose_;
    Eigen::Matrix3f lastScanMatchCov;
    float linear_v_ = 0;
    float rot_v_ = 0; 
    float imu_coeff_ = 1.0f; 
    float paramMinDistanceDiffForMapUpdate;
    float paramMinAngleDiffForMapUpdate;

    std::deque<msa2d::sensor::WheelOdom> wheelOdom_cache_;  
    std::deque<msa2d::sensor::ImuData> imu_cache_;  
    std::deque<msa2d::sensor::LaserScan::Ptr> laser_cache_;   

    std::shared_mutex wheel_sm_;  
    std::shared_mutex imu_sm_;  
    std::shared_mutex laser_sm_;  

    std::unique_ptr<EKFEstimatorBase> ekf_estimator_; 
    ImuInitializer2D imu_initializer_; 

    std::thread run_;  
};
}

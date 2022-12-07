/*
 * @Copyright(C): 
 * @FileName: 
 * @Author: lwh
 * @Description: 卡尔曼滤波融合激光、 IMU、轮速
 *                                  1、融合去畸变
 *                                  2、里程计在线外参标定，以及外参在线优化 
 *                                  3、反光板融合
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <deque>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include "calibration/2d_handeye_calibration.hpp"
#include "sensor/sensor_data_type.hpp"
#include "Estimator/motion_calc.hpp"
#include "utility.hpp"
#include "Estimator/dead_reckoning.hpp"

using namespace Slam2D;  

constexpr bool PARAM_ESTIMATE = true;  // 外参估计 
constexpr bool EXTRINSIC_OPT = false;   // 外参在线优化 

class FusionOdometryNode {
public:
    struct Option {
        float min_laser_range_ = -1;
        float max_laser_range_ = -1;  
    };
    FusionOdometryNode() {
        // \033[1;32m，\033[0m 终端显示成绿色
        ROS_INFO_STREAM("\033[1;32m----> optimization fusion node started.\033[0m");

        imu_subscriber_ = node_handle_.subscribe(
            "imu", 2000, &FusionOdometryNode::ImuCallback, this, ros::TransportHints().tcpNoDelay());
        
        wheel_odom_subscriber_ = node_handle_.subscribe(
            "odom", 2000, &FusionOdometryNode::OdomCallback, this, ros::TransportHints().tcpNoDelay());
        
        laser_subscriber_ = node_handle_.subscribe(
            "laser_scan", 5, &FusionOdometryNode::ScanCallback, this, ros::TransportHints().tcpNoDelay());

        odom_pub = node_handle_.advertise<nav_msgs::Odometry>(
            "dead_reckoning", 1, this);
        
        AccSigmaLimit_ = 0.002;
        Init();  
    }

    // 读参数 + 初始化
    void Init() {
        DeadReckon2D::Option option;
        option.use_odom_motion_ = false;  
        dead_reckon_ = DeadReckon2D(option); 
        // 激光参数
        option_.min_laser_range_ = 1;
        option_.max_laser_range_ = 20;
    }

    // imu的回调函数
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg) {
        std::lock_guard<std::mutex> lock(imu_mt_);
        ImuData imu; 
        imu.time_stamp_ = imuMsg->header.stamp.toSec();
        imu.acc_ = Eigen::Vector3d{imuMsg->linear_acceleration.x, 
                                                                imuMsg->linear_acceleration.y,
                                                                imuMsg->linear_acceleration.z};
        imu.angular_v_ = Eigen::Vector3d{imuMsg->angular_velocity.x,
                                                                                imuMsg->angular_velocity.y,
                                                                                imuMsg->angular_velocity.z};                      
        imu_buf_.push_back(imu);
        if (imu_init_) {
            dead_reckon_.AddImuData(imu, yaw_angular_velocity_bias_);  
        } else {
            // 初始化
            if (ImuInit(imu_buf_)) {
                imu_init_ = true;  
            }
        }
    }

    // imu初始化 - 静止情况下，计算出角速度的bias 
    bool ImuInit(std::deque<ImuData> &imu_data) {
        if (imu_data.size() < 100) {
            return false;
        }
        Eigen::Vector3d sigma_acc{0., 0., 0.};
        Eigen::Vector3d mean_acc{0., 0., 0.};
        Eigen::Vector3d mean_gyro{0., 0., 0.};
        uint16_t N = 0;
        // 计算均值
        for (auto const& imu : imu_data) {
            mean_gyro += (imu.angular_v_ - mean_gyro) / (N + 1);  
            mean_acc += (imu.acc_ - mean_acc) / (N + 1); 
            // cov_acc = cov_acc * (N / (N + 1)) + 
            //     (imu.acc_ - mean_acc).cwiseProduct(imu.acc_ - mean_acc) * N / ((N + 1) * (N + 1));
            N++; 
        }
        // 计算加速度方差   判定是否静止 
        for (auto const& imu : imu_data) {
            sigma_acc += (imu.acc_ - mean_acc).cwiseAbs2();
        }
        sigma_acc = (sigma_acc / imu_data.size()).cwiseSqrt(); 
        if (sigma_acc.norm() < AccSigmaLimit_) {
            yaw_angular_velocity_bias_ = mean_gyro[2]; 
            std::cout<<"imu yaw angular velocity bias init ok! is: "<<yaw_angular_velocity_bias_<<std::endl;
            return true;
        }
        imu_data.clear(); 
        return false; 
    }

    // odom的回调函数
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg) {
        std::lock_guard<std::mutex> lock(wheel_odom_mt_);
        Odom odom;
        odom.time_stamp_ = odometryMsg->header.stamp.toSec();

        odom.v_x_ = odometryMsg->twist.twist.linear.x;
        odom.v_y_ = odometryMsg->twist.twist.linear.y;
        odom.omega_yaw_ = odometryMsg->twist.twist.angular.z;

        odom.x_ = odometryMsg->pose.pose.position.x;
        odom.y_ = odometryMsg->pose.pose.position.y;
        odom.orientation_.w() = odometryMsg->pose.pose.orientation.w;
        odom.orientation_.x() = odometryMsg->pose.pose.orientation.x;
        odom.orientation_.y() = odometryMsg->pose.pose.orientation.y;
        odom.orientation_.z() = odometryMsg->pose.pose.orientation.z;
        odom_buf_.push_back(odom);
        dead_reckon_.AddOdometryData(odom);   // 进行航迹推算
        Pose2d odom_pose = dead_reckon_.GetDeadReckoningPose();
        // // 发布航迹推算的结果
        // nav_msgs::Odometry odom_msg;
        // odom_msg.header = odometryMsg->header;
        // odom_msg.pose.pose.position.x = odom_pose.x_;
        // odom_msg.pose.pose.position.y = odom_pose.y_;
        // odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose.yaw_);
        // odom_pub.publish(odom_msg); 
    }

    // 激光的回调函数
    // 1、第一个激光帧提取信息    2、将ros msg 转换为 xy 数据  
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg) {
        static bool is_first = true;
        if (is_first) {
            // 提取激光的信息 
            laser_info_.angle_increment_ = laserScanMsg->angle_increment;
            laser_info_.time_increment_ = laserScanMsg->time_increment;
            if (option_.min_laser_range_ == -1) {
                option_.min_laser_range_ = laserScanMsg->range_min;
            }
            if (option_.max_laser_range_ == -1) {
                option_.max_laser_range_ = laserScanMsg->range_max;
            }
            laser_info_.index_cos_.clear();
            laser_info_.index_sin_.clear();

            for (unsigned int i = 0; i < laserScanMsg->ranges.size(); i++) {
                double angle = laserScanMsg->angle_min + i * laserScanMsg->angle_increment;
                laser_info_.index_cos_.push_back(cos(angle));
                laser_info_.index_sin_.push_back(sin(angle));
            }
            is_first = false; 
        } 
        
        LaserTimedPointCloud pointcloud;
        pointcloud.start_timestamp_ = laserScanMsg->header.stamp.toSec();
        // 极坐标转换为笛卡尔坐标
        // 遍历每个激光点 求每个激光点相对于起始位置的变换 
        for (int i = 0; i < laserScanMsg->ranges.size(); i++) {
            // 如果是无效点，或距离超过范围的点，就跳过
            if (!std::isfinite(laserScanMsg->ranges[i]) ||
                laserScanMsg->ranges[i] < laserScanMsg->range_min ||
                laserScanMsg->ranges[i] > laserScanMsg->range_max) {
                continue;
            }
            LaserPoint point; 
            // 计算雷达数据的 x y 坐标
            point.range_ = laserScanMsg->ranges[i];
            point.pos_.x() =  laserScanMsg->ranges[i] * laser_info_.index_cos_[i];
            point.pos_.y() = laserScanMsg->ranges[i] * laser_info_.index_sin_[i];
            point.index_ = i; 
            pointcloud.points_.push_back(point); 
        }
        laser_buf_.emplace_back(std::move(pointcloud));
    }

    void Estimator() {
        while(1) {
            { 
                std::unique_lock<std::mutex> laser_lock(laser_mt_);
                // 进行数据同步  laser - imu - odom 
                // if (!laser_buf_.empty() || !odom_buf_.empty()) {
                //     uint8_t sensor_id = 0;
                //     double earliest_time = numeric_limits<double>::max();  
                //     // 找到最早的数据  
                //     if(!imu_buf.empty()) {   
                //         earliest_time = imu_buf.front()->timestamp;  
                //         sensor_id = imu;
                //     }
                //     if (!gnss_buf.empty()) {   
                //         // 如果gnss更早 
                //         if (earliest_time > gnss_buf.front()->timestamp) {
                //             earliest_time = gnss_buf.front()->timestamp;
                //             sensor_id = gnss;
                //         }
                //     }
                //     switch(sensor_id) {
                //         // 如果是imu数据执行 预测  
                //         case imu: {
                    
                //         }
                //         break;
                //         case gnss: {   

                //         }
                //         break;  
                //     }
                // }
        //             nav_msgs::Odometry curr_laser_odom = laser_odom_msg_.front();  
        //             laser_odom_msg_.pop_front();
        //             nav_msgs::Odometry next_laser_odom = laser_odom_msg_.front();  
        //             laser_lock.unlock();  
        //             // 获取包围激光里程计数据的 odom 和 imu 数据  
        //             // 轮速的数据需要包围laser
        //             std::unique_lock<std::mutex> wheel_odom_lock(wheel_odom_mt_);
        //             // 清除过早的数据 
        //             while (!odom_buf_.empty()) {
        //                 if (odom_buf_.front().header.stamp.toSec() 
        //                         > curr_laser_odom.header.stamp.toSec() - 0.03) {
        //                     break;
        //                 }
        //                 odom_buf_.pop_front();  
        //             }
        //             // 判断数据范围
        //             if (odom_buf_.front().header.stamp.toSec() > curr_laser_odom.header.stamp.toSec()) {
        //                 wheel_odom_lock.unlock(); 
        //                 std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //                 continue;  
        //             }
        //             // 如果odom的数据 暂时还没有覆盖laser  那么等待一会儿再处理 
        //             if (odom_buf_.back().header.stamp.toSec() < next_laser_odom.header.stamp.toSec()) {
        //                 laser_lock.lock(); 
        //                 laser_odom_msg_.push_front(curr_laser_odom); 
        //                 laser_lock.unlock(); 
        //                 std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //                 continue;  
        //             }

        //             nav_msgs::Odometry begin_wheel_odom;
        //             bool is_begin = true; 

        //             for (uint16_t i = 0; i < odom_buf_.size(); i++) {
        //                 if (odom_buf_[i].header.stamp.toSec() <= curr_laser_odom.header.stamp.toSec()) {
        //                     begin_wheel_odom = odom_buf_[i]; 
        //                     continue;
        //                 }
                        
        //                 Odom wheel_odom;  
        //                 if (is_begin) {
        //                     wheel_odom.time_stamp_ = begin_wheel_odom.header.stamp.toSec();

        //                     wheel_odom.v_x_ = begin_wheel_odom.twist.twist.linear.x;
        //                     wheel_odom.v_y_ = begin_wheel_odom.twist.twist.linear.y;
        //                     wheel_odom.omega_yaw_ = begin_wheel_odom.twist.twist.angular.z;

        //                     wheel_odom.x_ = begin_wheel_odom.pose.pose.position.x;
        //                     wheel_odom.y_ = begin_wheel_odom.pose.pose.position.y;  
        //                     wheel_odom.orientation_.w() = begin_wheel_odom.pose.pose.orientation.w;
        //                     wheel_odom.orientation_.x() = begin_wheel_odom.pose.pose.orientation.x;
        //                     wheel_odom.orientation_.y() = begin_wheel_odom.pose.pose.orientation.y;
        //                     wheel_odom.orientation_.z() = begin_wheel_odom.pose.pose.orientation.z;
        //                     odom_buf_.push_back(wheel_odom);  
        //                     is_begin = false;
        //                 }

        //                 nav_msgs::Odometry curr_wheel_odom = odom_buf_[i]; 
        //                 wheel_odom.time_stamp_ = curr_wheel_odom.header.stamp.toSec();

        //                 wheel_odom.v_x_ = curr_wheel_odom.twist.twist.linear.x;
        //                 wheel_odom.v_y_ = curr_wheel_odom.twist.twist.linear.y;
        //                 wheel_odom.omega_yaw_ = curr_wheel_odom.twist.twist.angular.z;

        //                 wheel_odom.x_ = curr_wheel_odom.pose.pose.position.x;
        //                 wheel_odom.y_ = curr_wheel_odom.pose.pose.position.y;  
        //                 wheel_odom.orientation_.w() = begin_wheel_odom.pose.pose.orientation.w;
        //                 wheel_odom.orientation_.x() = begin_wheel_odom.pose.pose.orientation.x;
        //                 wheel_odom.orientation_.y() = begin_wheel_odom.pose.pose.orientation.y;
        //                 wheel_odom.orientation_.z() = begin_wheel_odom.pose.pose.orientation.z;
        //                 odom_buf_.push_back(wheel_odom);  

        //                 if (wheel_odom.time_stamp_ > next_laser_odom.header.stamp.toSec()) {
        //                     break;
        //                 }   
        //             }
        //             wheel_odom_lock.unlock();  

        //             // imu_mt_.lock();
        //             // // 获取IMU数据 
        //             // // 清除过早的数据 
        //             // while (!imu_msg_.empty())
        //             // {
        //             //     if (imu_msg_.front().header.stamp.toSec() 
        //             //             > curr_laser_odom.header.stamp.toSec() - 0.03) 
        //             //     {
        //             //         break;
        //             //     }
        //             //     imu_msg_.pop_front();  
        //             // }
        //             // // 判断数据范围
        //             // if (imu_msg_.front().header.stamp.toSec() > curr_laser_odom.header.stamp.toSec())
        //             // {
        //             //     imu_mt_.unlock(); 
        //             //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //             //     continue;  
        //             // }

        //             // if (imu_msg_.back().header.stamp.toSec() < next_laser_odom.header.stamp.toSec())
        //             // {
        //             //     laser_lock.lock(); 
        //             //     laser_odom_msg_.push_front(curr_laser_odom); 
        //             //     laser_lock.unlock(); 
        //             //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //             //     continue;  
        //             // }
        //             // // 提取出wheel odom的数据 保存在wheel_odom_data_ 中 
        //             // sensor_msgs::Imu begin_imu_data;
        //             // is_begin = true; 
        //             // for (uint16_t i = 0; i < imu_msg_.size(); i++)
        //             // {
        //             //     if (imu_msg_[i].header.stamp.toSec() <= curr_laser_odom.header.stamp.toSec())
        //             //     {
        //             //         begin_imu_data = imu_msg_[i]; 
        //             //         continue;
        //             //     }
                        
        //             //     ImuData imu_odom;  
        //             //     if (is_begin)
        //             //     {
        //             //         imu_odom.time_stamp_ = begin_imu_data.header.stamp.toSec();
        //             //         imu_odom.omega_yaw_ = begin_imu_data.angular_velocity.z;  
        //             //         imu_buf_.push_back(imu_odom);  
        //             //         is_begin = false;
        //             //     }

        //             //     sensor_msgs::Imu curr_imu_data = imu_msg_[i];
        //             //     imu_odom.time_stamp_ = curr_imu_data.header.stamp.toSec();
        //             //     imu_odom.omega_yaw_ = curr_imu_data.angular_velocity.z;  
        //             //     imu_buf_.push_back(imu_odom);  

        //             //     if (imu_odom.time_stamp_ > next_laser_odom.header.stamp.toSec())
        //             //     {
        //             //         break;
        //             //     }   
        //             // }
                    
        //             // imu_mt_.unlock();
        //             // 求出激光里程计的位姿增量    即一次观测   
        //             Pose2d delta_laser_pose = Utility::CalcIncremPose(curr_laser_odom, next_laser_odom);  
        //             // 进行参数估计 
        //             if (PARAM_ESTIMATE) {
        //                 // 如果当前状态是 初始化 
        //                 if (status == INIT) {
        //                     // 先初始化IMU角速度偏置    保持静止 数秒
        //                     // 接着进行激光odom外参初始化
        //                     Pose2d odom_delta_pose; 
        //                     // 求出轮速计的位姿增量  
        //                     if (GetMotionFromOdom(curr_laser_odom.header.stamp.toSec(), 
        //                                                                     next_laser_odom.header.stamp.toSec(), 
        //                                                                     odom_buf_, odom_delta_pose)) {
        //                         if (calib_.AddPose(delta_laser_pose, odom_delta_pose)) {
        //                             if (calib_.CalibMethod2()) {
        //                                 ROS_INFO("calib done !!");
        //                                 calib_.GetCalibResult(extrinsic_lidar_odom_); 
        //                                 std::cout<<"extrinsic_lidar_odom_ x: "<<extrinsic_lidar_odom_.x_
        //                                 <<", y: "<<extrinsic_lidar_odom_.y_<<", yaw: "<<extrinsic_lidar_odom_.theta_<<std::endl;
        //                                 status = ODOMETRY;  
        //                             }
        //                         }
        //                     }
        //                 }
        //             }
        //             // eskf 里程计运行
        //             if (status == ODOMETRY) {
        //             }

            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

private:
    enum WorkStatus {
        INIT = 0,    // 初始化状态
        PARAM_OPT, // 参数优化阶段， 滑动窗口优化  
        ODOMETRY // 里程计运行, 基于滤波器，可以在线优化参数   也可不优化   
    };
    struct LaserInfo {
        std::vector<double> index_cos_; // 保存下来雷达各个角度的cos值
        std::vector<double> index_sin_; // 保存下来雷达各个角度的sin值
        float angle_increment_;  // 激光点角度增量  
        double time_increment_;  
        float laser_period_;   // 激光一帧的时间
        int laser_num_;   // 激光点的数量
    }laser_info_;

    WorkStatus status = INIT;   
    Option option_; 
    ros::NodeHandle node_handle_;  
    ros::Subscriber imu_subscriber_;
    ros::Subscriber wheel_odom_subscriber_;
    ros::Subscriber laser_subscriber_;
    ros::Publisher odom_pub;

    std::deque<nav_msgs::Odometry> laser_odom_msg_;

    std::mutex imu_mt_, wheel_odom_mt_, laser_mt_;  

    std::deque<Odom> odom_buf_;
    std::deque<ImuData> imu_buf_;
    std::deque<LaserTimedPointCloud> laser_buf_;  

    DeadReckon2D dead_reckon_; 
    HandEyeCalib2D calib_; 
    Pose2d extrinsic_lidar_odom_;       // lidar - odom的外参  

    bool imu_init_ = false;   // imu初始化 

    float AccSigmaLimit_; 
    double yaw_angular_velocity_bias_ = 0.; 
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "optimization_fusion_odometry_node");
    FusionOdometryNode node;
    // std::thread estimator(&FusionOdometryNode::Estimator, &node); 
    ros::spin(); 
    return (0);
}








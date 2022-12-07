/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-14 23:31:10
 * @Description: 使用 IMU、ODOM 去除 laser的畸变  
 * @Others: 
 */

#include <preprocess/laser_undistortion.hpp>

LidarUndistortion::LidarUndistortion() : private_node_("~")
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> lidar undistortion node started.\033[0m");

    imu_subscriber_ = node_handle_.subscribe(
        "imu", 2000, &LidarUndistortion::ImuCallback, this, ros::TransportHints().tcpNoDelay());
    
    odom_subscriber_ = node_handle_.subscribe(
        "odom", 2000, &LidarUndistortion::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    
    laser_scan_subscriber_ = node_handle_.subscribe(
        "laser_scan", 5, &LidarUndistortion::ScanCallback, this, ros::TransportHints().tcpNoDelay());

    corrected_pointcloud_publisher_ = node_handle_.advertise<PointCloudT>(
        "corrected_pointcloud", 1, this);

    first_scan_ = true;
    corrected_pointcloud_.reset(new PointCloudT());

    // 参数进行初始化与重置
    ResetParameters();
}
 
LidarUndistortion::~LidarUndistortion()
{
}

// 参数进行初始化与重置
void LidarUndistortion::ResetParameters()
{
    corrected_pointcloud_->points.clear();
    current_imu_index_ = 0;

    rot_container_.clear();

    odom_incre_x_ = 0.0;
    odom_incre_y_ = 0.0;
    odom_incre_z_ = 0.0;
}

// imu的回调函数
void LidarUndistortion::ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
{
    std::lock_guard<std::mutex> lock(imu_lock_);
    imu_queue_.push_back(*imuMsg);
}

// odom的回调函数
void LidarUndistortion::OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg)
{
    std::lock_guard<std::mutex> lock(odom_lock_);
    odom_queue_.push_back(*odometryMsg);
}

// scan的回调函数
void LidarUndistortion::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg)
{
    // 缓存雷达数据
    if (!CacheLaserScan(laserScanMsg))
        return;
    // 解析IMU数据  获取旋转 
    if (!ParseImuData()) {
        imu_valid_ = false; 
    } else {
        imu_valid_ = true; 
    }
    // 解析ODOM数据 ，获取 激光雷达帧间的平移数据    若没有IMU旋转数据 , 则这里还要提取旋转
    if (!ParseOdomData()) {
        odom_valid_ = false; 
    } else {
        odom_valid_ = true; 
    }
    
    if (!odom_valid_ && !imu_valid_)
        return;
    // 此时说明 imu 或者 odom 至少有一个具有覆盖laser的数据 
    // 对雷达数据的每一个点进行畸变的去除
    CorrectLaserScan();
    // 将较正过的雷达数据以PointCloud2的形式发布出去
    PublishCorrectedPointCloud();
    // 参数重置
    ResetParameters();
}

// 缓存雷达数据
bool LidarUndistortion::CacheLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg)
{   
    if (first_scan_)
    {
        first_scan_ = false;
        // 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
        CreateAngleCache(laserScanMsg);
        scan_count_ = laserScanMsg->ranges.size();
    }

    corrected_pointcloud_->points.resize(laserScanMsg->ranges.size());
    // 缓存雷达数据
    laser_queue_.push_back(*laserScanMsg);
    // 缓存两帧雷达数据，以防止imu或者odom的数据不能包含雷达数据
    if (laser_queue_.size() < 2)
        return false;
    // 取出队列中的第一个数据
    current_laserscan_ = laser_queue_.front();
    laser_queue_.pop_front();
    // 获取这帧雷达数据的起始，结束时间
    current_laserscan_header_ = current_laserscan_.header;
    current_scan_time_start_ = current_laserscan_header_.stamp.toSec(); // 认为ros中header的时间为这一帧雷达数据的起始时间
    current_scan_time_increment_ = current_laserscan_.time_increment;
    current_scan_time_end_ = current_scan_time_start_ + current_scan_time_increment_ * (scan_count_ - 1);
    
    return true;
}

// 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
void LidarUndistortion::CreateAngleCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    a_cos_.clear();
    a_sin_.clear();
    double angle;

    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
    {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        a_cos_.push_back(cos(angle));
        a_sin_.push_back(sin(angle));
    }
}

/**
 * @brief: 处理imu队列，以获取包含 当前帧雷达时间 的imu数据及转角
 * @details: 筛选出处于当前激光扫描帧时间范围内的全部IMU数据，并将第一个IMU的旋转设为0，
 *                      根据角速度测量求出其他IMU的旋转
 * @return {*}
 */
bool LidarUndistortion::ParseImuData()
{
    std::lock_guard<std::mutex> lock(imu_lock_);
    // imu数据队列的头尾的时间戳要在雷达数据的时间段外
    // 如果IMU数据不能完全包围激光帧的话   就无法将 激光点 转换到 起始坐标上  
    if (imu_queue_.empty()) {
        return false;
    }
    // 修剪imu的数据队列，直到imu的时间接近这帧点云的时间
    while (!imu_queue_.empty())
    {
        if (imu_queue_.front().header.stamp.toSec() < current_scan_time_start_ - 0.05)
            imu_queue_.pop_front();
        else
            break;
    }
    // IMU数据必须要前后覆盖住 激光数据  
    if (imu_queue_.front().header.stamp.toSec() > current_scan_time_start_ ||
        imu_queue_.back().header.stamp.toSec() < current_scan_time_end_)
    {
        ROS_WARN("imu not covered laser ! skip ---");
        return false;  
    }

    sensor_msgs::Imu curr_imu_msg;
    double current_imu_time, start_imu_time, time_diff;
    bool is_start = true;  

    for (int i = 0; i < (int)imu_queue_.size(); i++)
    {
        curr_imu_msg = imu_queue_[i];
        current_imu_time = curr_imu_msg.header.stamp.toSec();
        // 获取时间戳早于当前激光帧的最后IMU数据 
        if (current_imu_time < current_scan_time_start_)
        {
            start_imu_time = current_imu_time;
            continue;
        }

        RotData rot_data;  
        // 第一个IMU 数据  旋转设置为0
        if (is_start)
        {
            rot_data.rot_time_ = start_imu_time;
            rot_data.rot_x_ = 0;
            rot_data.rot_y_ = 0;
            rot_data.rot_z_ = 0;
            rot_container_.push_back(rot_data);  
        }
        // get angular velocity
        double angular_x, angular_y, angular_z;
        angular_x = curr_imu_msg.angular_velocity.x;
        angular_y = curr_imu_msg.angular_velocity.y;
        angular_z = curr_imu_msg.angular_velocity.z;
        // 对imu的角速度进行积分，当前帧的角度 = 上一帧的角度 + 当前帧角速度 * (当前帧imu的时间 - 上一帧imu的时间)
        double time_diff = current_imu_time - rot_container_.back().rot_time_;
        rot_data.rot_time_ = current_imu_time;
        rot_data.rot_x_ = rot_container_.back().rot_x_ + angular_x * time_diff;
        rot_data.rot_y_ = rot_container_.back().rot_y_ + angular_y * time_diff;
        rot_data.rot_z_ = rot_container_.back().rot_z_ + angular_z * time_diff;
        rot_container_.push_back(rot_data);  
        // imu时间比雷达结束时间晚，就退出
        if (current_imu_time > current_scan_time_end_) {
            break;
        }
    }
    return true;
}

// 修剪odom队列，以获取包含 当前帧雷达时间 的odom的平移距离
bool LidarUndistortion::ParseOdomData()
{
    std::lock_guard<std::mutex> lock(odom_lock_);

    if (odom_queue_.empty()) {
        return false;
    }
    // 修剪odom的数据队列，直到odom的时间接近这帧点云的时间
    while (!odom_queue_.empty())
    {
        if (odom_queue_.front().header.stamp.toSec() < current_scan_time_start_ - 0.05)
            odom_queue_.pop_front();
        else
            break;
    }
    // 保证一定包围 
    if (odom_queue_.front().header.stamp.toSec() > current_scan_time_start_ ||
        odom_queue_.back().header.stamp.toSec() < current_scan_time_end_)
    {
        ROS_WARN("odom not covered laser ! skip ---");
        return false;
    }
    // get start odometry at the beinning of the scan
    nav_msgs::Odometry start_odom_msg_, end_odom_msg_;
    double current_odom_time;
    bool is_start_odom = true; 

    for (int i = 0; i < (int)odom_queue_.size(); i++)
    {
        current_odom_time = odom_queue_[i].header.stamp.toSec();
        // 一定可以获得一个 在 当前帧左侧的ODOM数据 
        if (current_odom_time < current_scan_time_start_)
        {
            start_odom_msg_ = odom_queue_[i];
            continue;
        }
        // 如果IMU数据无效  那么需要从odom中提取旋转信息 
        if (!imu_valid_)
        {
            tf::Quaternion orientation;
            double roll, pitch, yaw; 
            RotData rot_data; 
            if (is_start_odom)
            {
                // 获取起始odom消息的位移与旋转
                tf::quaternionMsgToTF(start_odom_msg_.pose.pose.orientation, orientation);
                tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
                rot_data.rot_time_ = start_odom_msg_.header.stamp.toSec();
                rot_data.rot_x_ = roll;
                rot_data.rot_y_ = pitch;
                rot_data.rot_z_ = yaw;
                rot_container_.push_back(rot_data);  
                is_start_odom = false;
            }
            tf::quaternionMsgToTF(odom_queue_[i].pose.pose.orientation, orientation);
            tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
            rot_data.rot_time_ = current_odom_time;
            rot_data.rot_x_ = roll;
            rot_data.rot_y_ = pitch;
            rot_data.rot_z_ = yaw;
            rot_container_.push_back(rot_data);  
        }

        if (current_odom_time > current_scan_time_end_)
        {
            end_odom_msg_ = odom_queue_[i];
            break; 
        }
    }

    start_odom_time_ = start_odom_msg_.header.stamp.toSec();
    end_odom_time_ = end_odom_msg_.header.stamp.toSec();
    // 如果时间与激光帧起始和终止时间足够接近那么可以使用
    if (current_scan_time_start_ - start_odom_time_ > 0.03 
        || end_odom_time_ - current_scan_time_end_ > 0.03 )
    {
        return true;      // 否则直接return  不用平移量了  
    }

    tf::Quaternion orientation;
    double roll, pitch, yaw;
    // 获取起始odom消息的位移与旋转
    tf::quaternionMsgToTF(start_odom_msg_.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    Eigen::Affine3f transBegin = pcl::getTransformation(
        start_odom_msg_.pose.pose.position.x,
        start_odom_msg_.pose.pose.position.y,
        start_odom_msg_.pose.pose.position.z,
        roll, pitch, yaw);

    // 获取终止odom消息的位移与旋转
    tf::quaternionMsgToTF(end_odom_msg_.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    Eigen::Affine3f transEnd = pcl::getTransformation(
        end_odom_msg_.pose.pose.position.x,
        end_odom_msg_.pose.pose.position.y,
        end_odom_msg_.pose.pose.position.z,
        roll, pitch, yaw);
    // 求得这之间的变换
    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;
    // 通过　transBt　获取　odomIncreX等，一帧点云数据时间的odom变化量
    float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt,
                                      odom_incre_x_, odom_incre_y_, odom_incre_z_,
                                      rollIncre, pitchIncre, yawIncre);   //
    return true;
}

// 对雷达数据的每一个点进行畸变的去除
void LidarUndistortion::CorrectLaserScan()
{
    bool first_point_flag = true;
    double current_point_time = 0;
    double current_point_x = 0, current_point_y = 0, current_point_z = 1.0;

    Eigen::Affine3f transStartInverse, transFinal, transBt;
    float rotXCur = 0, rotYCur = 0, rotZCur = 0;
    float posXCur = 0, posYCur = 0, posZCur = 0;
    // 首先计算起始位置的变换
    ComputeRotation(current_scan_time_start_, &rotXCur, &rotYCur, &rotZCur);
    ComputePosition(current_scan_time_start_, &posXCur, &posYCur, &posZCur);
    transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
                                            rotXCur, rotYCur, rotZCur)).inverse();
    // 遍历每个激光点 求每个激光点相对于起始位置的变换 
    for (int i = 0; i < scan_count_; i++)
    {
        // 如果是无效点，就跳过
        if (!std::isfinite(current_laserscan_.ranges[i]) ||
            current_laserscan_.ranges[i] < current_laserscan_.range_min ||
            current_laserscan_.ranges[i] > current_laserscan_.range_max) 
        {
            continue;
        }
        // 畸变校正后的数据
        PointT &point_tmp = corrected_pointcloud_->points[i];
        current_point_time = current_scan_time_start_ + i * current_scan_time_increment_;
        // 计算雷达数据的 x y 坐标
        current_point_x = current_laserscan_.ranges[i] * a_cos_[i];
        current_point_y = current_laserscan_.ranges[i] * a_sin_[i];
        if (i == 0)
        {
            point_tmp.x = current_point_x;
            point_tmp.y = current_point_y;
            continue; 
        }
        // 求得当前点对应时刻 相对于current_scan_time_start_ 的平移与旋转
        ComputeRotation(current_point_time, &rotXCur, &rotYCur, &rotZCur);
        ComputePosition(current_point_time, &posXCur, &posYCur, &posZCur);
        // 当前点对应时刻 相对于current_scan_time_start_的平移与旋转
        transFinal = pcl::getTransformation(posXCur, posYCur, posZCur,
                                            rotXCur, rotYCur, rotZCur);
        // 雷达数据的第一个点对应时刻的激光雷达坐标系 到 雷达数据当前点对应时刻的激光雷达坐标系 间的坐标变换
        transBt = transStartInverse * transFinal;
        // 将当前点的坐标 加上 两个时刻坐标系间的坐标变换 
        // 得到 当前点在 雷达数据的第一个点对应时刻的激光雷达坐标系 下的坐标
        point_tmp.x = transBt(0, 0) * current_point_x + transBt(0, 1) * current_point_y 
                                        + transBt(0, 2) * current_point_z + transBt(0, 3);
        point_tmp.y = transBt(1, 0) * current_point_x + transBt(1, 1) * current_point_y 
                                        + transBt(1, 2) * current_point_z + transBt(1, 3);
        point_tmp.z = transBt(2, 0) * current_point_x + transBt(2, 1) * current_point_y 
                                        + transBt(2, 2) * current_point_z + transBt(2, 3);
    }
}

/**
 * @brief: 根据点云中某点的时间戳赋予其 通过插值 得到的旋转量
 * @details: 旋转的来源有两个 优先使用IMU， 没有IMU 则使用odom的旋转  
 * @param {double} pointTime
 * @param {float} *rotXCur
 * @param {float} *rotYCur
 * @param {float} *rotZCur
 * @return {*}
 */
bool LidarUndistortion::ComputeRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0;
    *rotYCur = 0;
    *rotZCur = 0;

    double rot_time_front;
    RotData rot_front, rot_back;

    /**
     * @todo  这个实现有点怪  需要改一下 
     */    
    if (rot_container_.empty()) 
        return false;      // 不太可能成立   但是还是要加上这一句 
    // rot_front, rot_back 因该将pointTime 夹在中间 
    rot_front = rot_container_.front();
    rot_container_.pop_front();
    rot_back = rot_container_.front(); 
    // 找到 前后包夹住pointTime的旋转信息
    while (rot_back.rot_time_ < pointTime)
    {
        rot_front = rot_container_.front();
        rot_container_.pop_front();
        rot_back = rot_container_.front(); 
    }

    rot_container_.push_front(rot_front);

    // 根据线性插值计算 pointTime 时刻的旋转
    double ratioBack = (pointTime - rot_front.rot_time_) 
                                                / (rot_back.rot_time_ - rot_front.rot_time_);
    double ratioFront = (rot_back.rot_time_ - pointTime) 
                                                / (rot_back.rot_time_ - rot_front.rot_time_);

    *rotXCur = rot_front.rot_x_ * ratioFront + rot_back.rot_x_ * ratioBack;
    *rotYCur = rot_front.rot_y_ * ratioFront + rot_back.rot_y_ * ratioBack;
    *rotZCur = rot_front.rot_z_ * ratioFront + rot_back.rot_z_ * ratioBack;
    return true;
}

// 根据点云中某点的时间戳赋予其 通过插值 得到的平移量
void LidarUndistortion::ComputePosition(double pointTime, float *posXCur, float *posYCur, float *posZCur)
{
    *posXCur = 0;
    *posYCur = 0;
    *posZCur = 0;
    // 根据线性插值计算 pointTime 时刻的平移
    double ratioFront = (pointTime - start_odom_time_) / (end_odom_time_ - start_odom_time_);

    *posXCur = odom_incre_x_ * ratioFront;
    *posYCur = odom_incre_y_ * ratioFront;
    *posZCur = odom_incre_z_ * ratioFront;
}

// 将较正过的雷达数据以PointCloud2的形式发布出去
void LidarUndistortion::PublishCorrectedPointCloud()
{
    // ROS_INFO("publish point cloud");
    corrected_pointcloud_->width = scan_count_;
    corrected_pointcloud_->height = 1;
    corrected_pointcloud_->is_dense = false; // contains nans
    // 将scan_msg的消息头 赋值到 PointCloudT的消息头
    pcl_conversions::toPCL(current_laserscan_header_, corrected_pointcloud_->header);
    // 由于ros中自动做了 pcl::PointCloud<PointT> 到 sensor_msgs/PointCloud2 的数据类型的转换
    // 所以这里直接发布 pcl::PointCloud<PointT> 即可
    corrected_pointcloud_publisher_.publish(corrected_pointcloud_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson5_lidar_undistortion");

    LidarUndistortion lidar_undistortion;

    // 开启3个线程同时工作
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::waitForShutdown();
    return (0);
}

#pragma once 
#include<cassert>
#include "../../../Sensor/LaserPointContainer.h"
#include "../../../Estimator/motionModel.hpp" 
#include "../../../util/utility.hpp" 

namespace hectorslam {

void LaserUndistorted(LaserPointCloud::Ptr& laser, const Path& motion_info) {
    if (motion_info.size() == 0) {
        return;  
    }
    int16_t ptr = laser->pointcloud_.size() - 1;    // 从最后一个点往前进行处理 
    Pose2d begin_pose = motion_info.back().pose_;     // T_begin<-end
    // 相邻的两个motion数据  靠近motion_info.back() 为 front ,靠近 motion_info.front() 的为 back 
    Pose2d front_relate_begin_pose; // 相对与 激光最后一个点坐标系的pose 初始化为0 
    Pose2d back_relate_begin_pose; // 相对与 激光最后一个点坐标系的pose 初始化为0 
    // 处理时path中pose的时间因该是相对于该帧起始时间的
    double part_front_time =  motion_info.back().time_stamp_ - motion_info.front().time_stamp_;
    double part_back_time = 0; 
    Pose2d point_pose; // 每个激光点对应的雷达Pose 

    for (uint16_t i = motion_info.size() - 1; i >= 1; --i) {
        back_relate_begin_pose = begin_pose.inv() * motion_info[i - 1].pose_;  //T_end<-curr = T_end<-begin * T_begin<-curr
        part_back_time = motion_info[i - 1].time_stamp_ - motion_info.front().time_stamp_;

        while (ptr >= 0) {
            if (laser->pointcloud_[ptr].rel_time_ < part_back_time) {
                break;
            } 
            if (part_front_time - laser->pointcloud_[ptr].rel_time_ < 1e-3) {
                point_pose = front_relate_begin_pose;   // 该激光点时间与part_front_time接近，因此该激光点pose就近似为front_relate_begin_pose
            } else if (laser->pointcloud_[ptr].rel_time_ - part_back_time < 1e-3) {
                point_pose = back_relate_begin_pose;  // 该激光点时间与part_back_time接近，因此该激光点pose就近似为back_relate_begin_pose
            } else {
                // 插值
                point_pose = util::LinearInterpolate(back_relate_begin_pose, front_relate_begin_pose, 
                                                                                            part_back_time, part_front_time, laser->pointcloud_[ptr].rel_time_); 
            }
            // 将激光点观测转换到 最后一个点的坐标系   T_end<-curr * p_curr = p_end
            laser->pointcloud_[ptr].pos_ = point_pose * laser->pointcloud_[ptr].pos_;
            --ptr;  
        }
        if (ptr < 0) break;  

        front_relate_begin_pose = back_relate_begin_pose; 
        part_front_time = part_back_time; 
    }
}

}
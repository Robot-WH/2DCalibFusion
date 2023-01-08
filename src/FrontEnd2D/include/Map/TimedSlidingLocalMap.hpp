
#pragma once

#include "PointcloudLocalMapBase.hpp"
#include <nabo/nabo.h>

namespace hectorslam {

class TimedSlidingLocalMap : public PointCloudLocalMapBase {
public:
    struct  Option {
        int window_size_ = 10; 
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    TimedSlidingLocalMap(Option option) : option_(option) {
        LOG(INFO) << "create TimedSlidingLocalMap, window size:" << option_.window_size_;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 由于发生了充足的运动，因此添加一帧数据到Local map   
     */            
    void UpdateLocalMapForMotion(const std::vector<LaserPoint>& curr_points) override {
        // if (frame->empty()) return;  
        // std::lock_guard<std::mutex> lock(local_map_mt_);
        // if (local_map_frame_container_.find(name) == local_map_frame_container_.end()) {
        //     Base::local_map_container_[name].reset(new pcl::PointCloud<_PointType>()); 
        //     kdtree_container_[name].reset(new pcl::KdTreeFLANN<_PointType>()); 
        // }
        // auto& frame_queue_ = local_map_frame_container_[name];
        // //TicToc tt;
        // // 更新滑动窗口      0.1ms   
        // if (frame_queue_.size() >= option_.window_size_) {  
        //     Base::full_ = true;   
        //     frame_queue_.pop_front();
        //     frame_queue_.push_back(frame);
        //     Base::local_map_container_[name]->clear();   
        //     // 更新submap  
        //     for (typename std::deque<PointCloudConstPtr>::const_iterator it = frame_queue_.begin(); 
        //         it != frame_queue_.end(); it++) {
        //             *Base::local_map_container_[name] += **it;   
        //     }
        // } else {  
        //     *Base::local_map_container_[name] += *frame;      
        //     frame_queue_.push_back(frame);   
        // }
        // // tt.toc("update localmap points ");
        // // tt.tic();
        // // 耗时 > 5ms  
        // if (option_.use_kdtree_search_) {
        //     kdtree_container_[name]->setInputCloud(Base::local_map_container_[name]);
        // }
        // //tt.toc("local map kdtree ");
        // // std::cout<<"map_name size: "
        // // <<Base::local_map_.second->size()<<std::endl;
        // return;  
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 由于长时间没有更新地图，因此添加一帧数据到Local map   
     * @details 此时直接将滑动窗口最末尾的数据移除
     */            
    void UpdateLocalMapForTime(const std::vector<LaserPoint>& curr_points) override {
        // if (frame->empty()) return;  
        // if (local_map_frame_container_.find(name) == local_map_frame_container_.end()) {
        //     Base::local_map_container_[name].reset(new pcl::PointCloud<_PointType>()); 
        //     kdtree_container_[name].reset(new pcl::KdTreeFLANN<_PointType>()); 
        // }
        // auto& frame_queue_ = local_map_frame_container_[name];
        // if (!frame_queue_.empty()) {
        //     frame_queue_.pop_back();
        // }
        // frame_queue_.push_back(frame);
        // // 更新submap  
        // Base::local_map_container_[name]->clear();   
        // for (typename std::deque<PointCloudConstPtr>::const_iterator it = frame_queue_.begin(); 
        //     it != frame_queue_.end(); it++) {
        //     *Base::local_map_container_[name] += **it;   
        // }
        // if (option_.use_kdtree_search_) {
        //     kdtree_container_[name]->setInputCloud(Base::local_map_container_[name]);
        // }
        // return;  
    }
    
    /**
     * @brief: 找到 point 的近邻点
     * @param name 点的标识名
     * @param num 需要的近邻点数量
     * @param max_range 近邻点的最大距离
     * @param[out] res 查找的结果 
     * @return 是否搜索到 num 个点 
     */            
    virtual bool GetNearlyNeighbor(LaserPoint const& point, uint16_t const& num, 
                                                                        double const& max_range, std::vector<LaserPoint>& res) const override {

    } 

protected:

    void createKDTreeUsingLocalMap() {
        target_kdtree_database_.resize(2,localmap_.size());

        for(int i = 0; i < localmap_.size();i++) {
            target_kdtree_database_(0,i) = localmap_[i].pos_[0];    // x
            target_kdtree_database_(1,i) = localmap_[i].pos_[1];    // y 
        }

        target_kdtree_ = Nabo::NNSearchD::createKDTreeLinearHeap(target_kdtree_database_);
    }

private:
    Option option_; 
    // kdtree内的点云数据
    Eigen::MatrixXd target_kdtree_database_;
    // kdtree 
    Nabo::NNSearchD* target_kdtree_;
    std::vector<LaserPoint> localmap_;
}; // class TimedSlidingLocalMap
}


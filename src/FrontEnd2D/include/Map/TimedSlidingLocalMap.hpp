
#pragma once
#include <mutex>
#include <nabo/nabo.h>
#include "../Sensor/SensorData.hpp"

namespace hectorslam {

class PointcloudLocalMap {
public:
    struct  Option {
        int window_size_ = 10; 
    };
    PointcloudLocalMap() = default; 
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PointcloudLocalMap(Option option) : option_(option) {
        LOG(INFO) << "create PointcloudLocalMap, window size:" << option_.window_size_;
        localmap_.reserve(option_.window_size_ * 1000); 
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 由于发生了充足的运动，因此添加一帧数据到Local map   
     */            
    void UpdateLocalMapForMotion(std::vector<Eigen::Vector2f>& curr_points) {
        if (curr_points.empty()) return;  
        //TicToc tt;
        // 更新滑动窗口      0.1ms   
        if (sliding_window_.size() >= option_.window_size_) {  
            sliding_window_.pop_front();
            sliding_window_.push_back(std::move(curr_points));
        } else {  
            sliding_window_.push_back(std::move(curr_points));   
        }
        // 进行降采样滤波
        localMapDownsampling(sliding_window_); 
        // tt.toc("update localmap points ");
        return;  
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 由于长时间没有更新地图，因此添加一帧数据到Local map   
     * @details 此时直接将滑动窗口最末尾的数据移除
     */            
    void UpdateLocalMapForTime(std::vector<Eigen::Vector2f>& curr_points) {
        if (curr_points.empty()) return;  
        if (sliding_window_.size() >= option_.window_size_) {
            sliding_window_.pop_back();
        }
        sliding_window_.push_back(std::move(curr_points));
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
                                                                        double const& max_range, std::vector<LaserPoint>& res) const {
    } 

    const std::vector<Eigen::Vector2f>& ReadLocalMap() const {
        return localmap_;
    }

protected:

    void localMapDownsampling(const std::deque<std::vector<Eigen::Vector2f>>& sliding_window) {
        localmap_.clear();
        for (const auto& frame : sliding_window) {
            localmap_.insert(localmap_.end(), frame.begin(), frame.end());  
        }
    }

    void createKDTreeFromLocalMap() {
        target_kdtree_database_.resize(2, localmap_.size());

        for(int i = 0; i < localmap_.size();i++) {
            target_kdtree_database_(0,i) = localmap_[i][0];    // x
            target_kdtree_database_(1,i) = localmap_[i][1];    // y 
        }

        target_kdtree_ = Nabo::NNSearchD::createKDTreeLinearHeap(target_kdtree_database_);
    }

private:
    Option option_; 
    // kdtree内的点云数据
    Eigen::MatrixXd target_kdtree_database_;
    // kdtree 
    Nabo::NNSearchD* target_kdtree_;
    std::deque<std::vector<Eigen::Vector2f>> sliding_window_;
    std::vector<Eigen::Vector2f> localmap_;
    std::mutex local_map_mt_; 
}; // class PointcloudLocalMap
}



#pragma once

#include "../Sensor/SensorData.hpp"

namespace hectorslam {

class PointCloudLocalMapBase {
public:
    using Ptr = std::shared_ptr<PointCloudLocalMapBase>; 
    using ConstPtr = std::shared_ptr<const PointCloudLocalMapBase>; 

    PointCloudLocalMapBase() {}

    virtual ~PointCloudLocalMapBase() {}
    virtual void UpdateLocalMapForMotion(const std::vector<LaserPoint>& curr_points) = 0; 
    virtual void UpdateLocalMapForTime(const std::vector<LaserPoint>& curr_points) = 0; 
    virtual bool GetNearlyNeighbor(LaserPoint const& point, uint16_t const& num, 
        double const& max_range, std::vector<LaserPoint>& res) const = 0;   // 搜索邻居  
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 获取名字为name 的local map 
     * @param name
     * @return 点云类型 
     */            
    void GetLocalMap() {
    }

protected:
    std::mutex local_map_mt_; 
};
}

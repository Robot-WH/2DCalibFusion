//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#pragma once 

#include "OccGridMapOperation.hpp"
#include "../Map/GridMap.h"
#include "../Map/OccGridMapUtilConfig.h"

namespace hectorslam {

/**
 * 金字塔（多分辨率）地图对象。
 */
class GridMapPyramid {
public:
    struct Option {
        float bottom_resolution;
        int map_sizeX;
        int map_sizeY;
        unsigned int num_depth;   // 金字塔的层数 
        float min_distance_to_boundary;
    }; 
    /**
     * 构建金字塔地图，第一层地图格子代表的物理尺寸最小，格子数最多，地图精度高；
     * 层数越高，格子代表的物理尺寸越大，格子数越少，精度越低。
     */
    GridMapPyramid(const Option& option) : option_(option) {
        std::cout<<"create grid pyramid Map, bottom_resolution:" << option.bottom_resolution 
        << " map length: " << option.map_sizeX << ", map width: " << option.map_sizeY 
        << ", num_depth: " << option.num_depth << ", min_distance_to_boundary: " 
        << option.min_distance_to_boundary << std::endl;
        Eigen::Vector2i map_grid_size(option.map_sizeX / option.bottom_resolution , 
            option.map_sizeY / option.bottom_resolution ); // 第一层地图栅格size
        // 地图原点在世界坐标系下的坐标
        map_in_world_.x() = - option.map_sizeX * 0.5;  
        map_in_world_.y() = - option.map_sizeY * 0.5;  
        float mapResolution = option.bottom_resolution;

        for (unsigned int i = 0; i < option.num_depth; ++i) {
            std::cout << "map layer: " << i << ", cellLength: " << mapResolution
            << " length:" << map_grid_size.x() << " ,width: " << map_grid_size.y() << "\n";
            /** 创建网格地图 **/
            GridMap* gridMap = new hectorslam::GridMap(
                mapResolution, map_grid_size, Eigen::Vector2f(map_in_world_.x(), map_in_world_.y()));
            /** 处理上面网格地图的一些工具 **/
            OccGridMapUtilConfig<GridMap>* gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
            /** 上述三个内容统一放在MapProContainer容器中 **/
            mapOperateContainer_.push_back(OccGridMapOperation(gridMap, gridMapUtil));

            map_grid_size /= 2;       // 地图格子行、列格子数减半
            mapResolution *= 2.0f; // 地图精度减半
        }
        // 输入的激光数据即对应首层地图,第一层的激光数据无需记录到dataContainers中，因此数据容器大小 = 金字塔层数 - 1 = 地图容器大小 - 1 。
        // dataContainers[i]对应mapContainer[i+1], 输入的数据dataContainer 对应 mapOperateContainer_[0]
        dataContainers.resize(option.num_depth - 1);
    }

    virtual ~GridMapPyramid() {
        unsigned int size = mapOperateContainer_.size();

        for (unsigned int i = 0; i < size; ++i) {
            mapOperateContainer_[i].cleanup();
        } /// 析构函数，需释放使用的动态内存
    }

    virtual void reset() {
        unsigned int size = mapOperateContainer_.size();

        for (unsigned int i = 0; i < size; ++i) {
            mapOperateContainer_[i].reset(); // 重置地图
        }
    }

    float getScaleToMap() const { return mapOperateContainer_[0].getScaleToMap(); } // 获取地图尺度 scale = 1.0 / map_resolution.

    int getMapLevels() const { return mapOperateContainer_.size(); }                                      // 获取地图总层数
    const GridMap& getGridMap(int mapLevel) const { return mapOperateContainer_[mapLevel].getGridMap(); }; // 获取指定层的地图

    std::mutex* getMapMutex(int i) {
        return mapOperateContainer_[i].getMapMutex(); /// 获取指定层的地图锁
    }

    void onMapUpdated() {
        /// 提示地图已经得到更新，cache中的临时数据无效
        unsigned int size = mapOperateContainer_.size();

        for (unsigned int i = 0; i < size; ++i) {
            mapOperateContainer_[i].resetCachedData();
        }
    }

    /**
     * 每层地图由当前scan与计算的位姿进行更新
     * @param dataContainer    第一层激光数据，其他层激光数据存在 dataContainers 中
     * @param laser_pose_in_world   当前帧的世界系下位姿
     */
    void updateByScan(const std::vector<LaserPointContainer>& data_containers, 
                                                            const Eigen::Vector3f &laser_pose_in_world) {
        // std::cout << "isCloseToBoundary()" << std::endl;
        // 判断地图是否需要移动
        if (isCloseToBoundary(laser_pose_in_world)) {
            std::cout << common::GREEN << "进入submap边界，submap进行移动" << common::RESET << std::endl;
            // 计算map移动后原点的世界坐标
            // map的中点移动到当前laser处
            Eigen::Vector2f new_map_pos_in_world{laser_pose_in_world[0] - option_.map_sizeX * 0.5f,
                                                                                                  laser_pose_in_world[1] - option_.map_sizeY * 0.5f}; 
            // 移动地图金字塔
            moveTo(new_map_pos_in_world); 
            map_in_world_ = new_map_pos_in_world;
        } 
        // std::cout << "isCloseToBoundary() done" << std::endl;
        unsigned int size = data_containers.size();
        for (unsigned int i = 0; i < size; ++i) {
            mapOperateContainer_[i].updateByScan(data_containers[i].dataPoints, laser_pose_in_world);
        }
        //std::cout << "\n";
    }

    /**
     * @brief: 检查是否接近地图金字塔边界  
     * @param {Vector3f} &laser_pose_in_world
     * @return {*}
     */    
    bool isCloseToBoundary(const Eigen::Vector3f& laser_pose_in_world) {
        Eigen::Vector2f laser_ref_map = {laser_pose_in_world[0] - map_in_world_[0], 
                                                                            laser_pose_in_world[1] - map_in_world_[1]};  
        if (laser_ref_map[0] < option_.min_distance_to_boundary || 
             laser_ref_map[0] > option_.map_sizeX - option_.min_distance_to_boundary ||
             laser_ref_map[1] < option_.min_distance_to_boundary || 
             laser_ref_map[1] > option_.map_sizeY - option_.min_distance_to_boundary) {
                return true;  
        }
        return false;   
    }

    /**
     * @brief: 将地图的移动到世界坐标系上一个新的位置
     * @param 
     * @return {*}
     */    
    void moveTo(const Eigen::Vector2f& new_map_pos_in_world) {
        for (uint16_t i = 0; i < mapOperateContainer_.size(); ++i) {
            mapOperateContainer_[i].moveTo(new_map_pos_in_world);
        }
    }

    /** 设置网格为free状态的概率 **/
    void setUpdateFactorFree(float free_factor) {
        size_t size = mapOperateContainer_.size();

        for (unsigned int i = 0; i < size; ++i) {
            GridMap &map = mapOperateContainer_[i].getGridMap();
            map.setUpdateFreeFactor(free_factor);
        }
    }
    
    /** 设置网格为occupied状态的概率 **/
    void setUpdateFactorOccupied(float occupied_factor) {
        size_t size = mapOperateContainer_.size();

        for (unsigned int i = 0; i < size; ++i) {
            GridMap& map = mapOperateContainer_[i].getGridMap();
            map.setUpdateOccupiedFactor(occupied_factor);
        }
    }

protected:
    Option option_; 
    Eigen::Vector2f map_in_world_;    // 地图原点在世界坐标系下的坐标
    std::vector<OccGridMapOperation> mapOperateContainer_; /// 不同图层的地图操作对象    层数越高  分辨率越低   
    std::vector<LaserPointContainer> dataContainers;  /// 不同图层对应的激光数据
};
} // namespace hectorslam

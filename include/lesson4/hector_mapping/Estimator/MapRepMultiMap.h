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

#ifndef _hectormaprepmultimap_h__
#define _hectormaprepmultimap_h__

#include "MapRepresentationInterface.h"
#include "MapProcContainer.h"

#include "../Map/GridMap.h"
#include "../Map/OccGridMapUtilConfig.h"
#include "../ScanMatcher/hectorScanMatcher.hpp"

namespace hectorslam {

/**
 * 金字塔（多分辨率）地图对象。
 */
class MapRepMultiMap : public MapOperationInterface {
public:
    struct Option {
        float bottomResolution;
        int mapSizeX;
        int mapSizeY;
        unsigned int numDepth;   // 金字塔的层数 
    }; 
    /**
     * 构建金字塔地图，第一层地图格子代表的物理尺寸最小，格子数最多，地图精度高；
     * 层数越高，格子代表的物理尺寸越大，格子数越少，精度越低。
     */
    MapRepMultiMap(const Option& option) {
        std::cout<<"create grid pyramid Map, bottomResolution:" << option.bottomResolution 
        << " mapSizeX: " << option.mapSizeX << ", mapSizeY: " << option.mapSizeY 
        << ", numDepth: " << option.numDepth << std::endl;
        Eigen::Vector2i map_grid_size(option.mapSizeX, option.mapSizeY); // 第一层地图大小

        float totalMapSizeX = option.bottomResolution * static_cast<float>(option.mapSizeX); // 实际物理尺寸范围
        float totalMapSizeY = option.bottomResolution * static_cast<float>(option.mapSizeY);
        // 中间的偏置  即初始时，world原点在map的中点 
        float mid_offset_x = totalMapSizeX * 0.5;              
        float mid_offset_y = totalMapSizeY * 0.5;
        scan_matcher_ =  new hectorslam::ScanMatcher<OccGridMapUtilConfig<GridMap>>();
        float mapResolution = option.bottomResolution;

        for (unsigned int i = 0; i < option.numDepth; ++i) {
            std::cout << "map layer: " << i << ", cellLength: " << mapResolution
            << " res x:" << map_grid_size.x() << " res y: " << map_grid_size.y() << "\n";
            /** 创建网格地图 **/
            GridMap *gridMap = new hectorslam::GridMap(
                mapResolution, map_grid_size, Eigen::Vector2f(mid_offset_x, mid_offset_y));
            /** 处理上面网格地图的一些工具 **/
            OccGridMapUtilConfig<GridMap> *gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
            /** 匹配工具 **/
            ScanMatcher<OccGridMapUtilConfig<GridMap>> *scanMatcher = new hectorslam::ScanMatcher<OccGridMapUtilConfig<GridMap>>();
            /** 上述三个内容统一放在MapProContainer容器中 **/
            mapContainer.push_back(MapProcContainer(gridMap, gridMapUtil, scanMatcher));

            map_grid_size /= 2;       // 地图格子行、列格子数减半
            mapResolution *= 2.0f; // 地图精度减半
        }
        // 输入的激光数据即对应首层地图,第一层的激光数据无需记录到dataContainers中，因此数据容器大小 = 金字塔层数 - 1 = 地图容器大小 - 1 。
        // dataContainers[i]对应mapContainer[i+1], 输入的数据dataContainer 对应 mapContainer[0]
        dataContainers.resize(option.numDepth - 1);
    }

    virtual ~MapRepMultiMap()
    {
        unsigned int size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            mapContainer[i].cleanup();
        } /// 析构函数，需释放使用的动态内存
    }

    virtual void reset()
    {
        unsigned int size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            mapContainer[i].reset(); // 重置地图
        }
    }

    virtual float getScaleToMap() const { return mapContainer[0].getScaleToMap(); }; // 获取地图尺度 scale = 1.0 / map_resolution.

    virtual int GetMapLevels() const { return mapContainer.size(); };                                      // 获取地图总层数
    virtual const GridMap &getGridMap(int mapLevel) const { return mapContainer[mapLevel].getGridMap(); }; // 获取指定层的地图
    OccGridMapUtilConfig<GridMap> &getOccGridMapUtil(int mapLevel) {return mapContainer[mapLevel].getOccGridMapUtil();};

    virtual void addMapMutex(int i, MapLockerInterface *mapMutex)
    {
        mapContainer[i].addMapMutex(mapMutex); /// 给每层地图添加互斥锁
    }

    MapLockerInterface *getMapMutex(int i)
    {
        return mapContainer[i].getMapMutex(); /// 获取指定层的地图锁
    }

    virtual void onMapUpdated() /// 提示地图已经得到更新，cache中的临时数据无效
    {
        unsigned int size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            mapContainer[i].resetCachedData();
        }
    }

    /**
     * 每层地图由当前scan与计算的位姿进行更新
     * @param dataContainer    第一层激光数据，其他层激光数据存在 dataContainers 中
     * @param robotPoseWorld   当前帧的世界系下位姿
     */
    virtual void updateByScan(const std::vector<LaserPointContainer> &data_containers, 
                                                            const Eigen::Vector3f &robotPoseWorld) {
        unsigned int size = data_containers.size();

        for (unsigned int i = 0; i < size; ++i) {
            mapContainer[i].updateByScan(data_containers[i], robotPoseWorld);
        }
        //std::cout << "\n";
    }

    /** 设置网格为free状态的概率 **/
    virtual void setUpdateFactorFree(float free_factor) {
        size_t size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i) {
            GridMap &map = mapContainer[i].getGridMap();
            map.setUpdateFreeFactor(free_factor);
        }
    }
    
    /** 设置网格为occupied状态的概率 **/
    virtual void setUpdateFactorOccupied(float occupied_factor) {
        size_t size = mapContainer.size();

        for (unsigned int i = 0; i < size; ++i) {
            GridMap &map = mapContainer[i].getGridMap();
            map.setUpdateOccupiedFactor(occupied_factor);
        }
    }

protected:
    std::vector<MapProcContainer> mapContainer; /// 不同图层的地图对象    层数越高  分辨率越低   
    std::vector<LaserPointContainer> dataContainers;  /// 不同图层对应的激光数据
    ScanMatcher<OccGridMapUtilConfig<GridMap>> *scan_matcher_; 
};
} // namespace hectorslam

#endif


#pragma once 

#include "GridMapBase.h"
#include "../Sensor/LaserPointContainer.h"
#include "../util/UtilFunctions.h"
#include <Eigen/Geometry>

namespace hectorslam {

template <typename _CellType, typename _GridFunctions>
class OccGridMapBase : public GridMapBase<_CellType> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @param {float} map_resolution
     * @param {Vector2i} &size 
     * @param map_in_world map坐标系原点在world坐标系的坐标
     */    
    OccGridMapBase(float map_resolution, const Eigen::Vector2i &size, 
        const Eigen::Vector2f& map_in_world) 
        : GridMapBase<_CellType>(map_resolution, size, map_in_world) {
    }

    virtual ~OccGridMapBase() {}

    void updateSetOccupied(int index) {
        grid_func_.updateSetOccupied(this->getCell(index));
    }

    void updateSetFree(int index) {
        grid_func_.updateSetFree(this->getCell(index));
    }

    void updateUnsetFree(int index) {
        grid_func_.updateUnsetFree(this->getCell(index));
    }

    float getGridProbabilityMap(int index) const {
        return grid_func_.getGridProbability(this->getCell(index));
    }

    bool isOccupied(int xMap, int yMap) const {
        return (this->getCell(xMap, yMap).isOccupied());
    }

    bool isFree(int xMap, int yMap) const {
        return (this->getCell(xMap, yMap).isFree());
    }

    bool isOccupied(int index) const {
        return (this->getCell(index).isOccupied());
    }

    bool isFree(int index) const {
        return (this->getCell(index).isFree());
    }

    float getObstacleThreshold() const {
        _CellType temp;
        temp.resetGridCell();
        return grid_func_.getGridProbability(temp);
    }

    void setUpdateFreeFactor(float factor) {
        grid_func_.setUpdateFreeFactor(factor);
    }

    void setUpdateOccupiedFactor(float factor) {
        grid_func_.setUpdateOccupiedFactor(factor);
    }

protected:
    _GridFunctions grid_func_;
};
} // namespace hectorslam

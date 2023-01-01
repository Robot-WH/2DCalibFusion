
#pragma once 

#include <Eigen/Geometry>
#include "../../../../Sensor/LaserPointContainer.h"
#include "../../../../util/UtilFunctions.h"
#include "../../../../Map/GridMap.h"
#include "../../../../Map/GridMapCacheArray.h"
#include "../../../../Estimator/MapRepMultiMap.h"

namespace hectorslam {

/**
 * @brief: hector-slam所使用的匹配算法
 * @details: 特点：多地图
 */
class hectorScanMatcher {
public:
    struct Option {
        int multi_map_num_ = 3;    // 使用的地图数量
        uint8_t lowest_layer_idx_; // 使用金字塔地图的最低层序号 
        uint8_t highest_layer_idx_; // 使用金字塔地图的最高层序号 
    };
    hectorScanMatcher(Option option) : option_(option) {
    }

    ~hectorScanMatcher() {
    }

    /**
     * 地图匹配，通过多分辨率地图求解当前激光帧的pose
     * @param beginEstimateWorld 世界坐标下先验位姿
     * @param dataContainers 激光观测金字塔数据
     * @param map 匹配金字塔地图  
     * @param covMatrix
     * @return  
     */
    virtual Eigen::Vector3f Solve(const Eigen::Vector3f& beginEstimateWorld, 
                                                                const std::vector<LaserPointContainer>& dataContainers, 
                                                                MapRepMultiMap& map, 
                                                                Eigen::Matrix3f& covMatrix) {
        // std::cout << "hectorScanMatcher::solve" <<std::endl;
        size_t size = map.GetMapLevels();
        Eigen::Vector3f tmp(beginEstimateWorld);
        /// coarse to fine 的pose求精过程，i层的求解结果作为i-1层的求解初始值。
        for (int index = size - 1; index >= 0; --index) {
            if (index == 0) {
                tmp = matchData(tmp, map.getGridMap(index), dataContainers[index], covMatrix, 5);
            } else {
                tmp = matchData(tmp, map.getGridMap(index), dataContainers[index], covMatrix, 3);
            }
        }
        return tmp;
    }

private:
  
    /**
     * 实际进行位姿估计的函数
     * @param beginEstimateWorld  位姿初值
     * @param grid_map  参与匹配的栅格地图 
     * @param dataContainer   激光数据    与  当前OccMap同分辨率  
     * @param covMatrix   协方差矩阵
     * @param maxIterations   最大迭代次数
     * @return
     */
    Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, 
                                                            const GridMap& grid_map, 
                                                            const LaserPointContainer& dataContainer, 
                                                            Eigen::Matrix3f& covMatrix, 
                                                            int maxIterations) {
        // 第一帧时，dataContainer为空 因此不会进行匹配                                                      
        if (dataContainer.getSize() != 0) {
            // beginEstimateWorld 为相对于世界坐标系的位姿 ，这里将世界坐标系的位姿转换为相对于当前OccMap的
            Eigen::Vector3f beginEstimateMap(grid_map.getMapCoordsPose(beginEstimateWorld));
            Eigen::Vector3f estimate(beginEstimateMap);
            // 2. 第一次迭代
            estimateTransformationGN(estimate, grid_map, dataContainer);
            int numIter = maxIterations;
            /** 3. 多次迭代求解 **/
            for (int i = 0; i < numIter; ++i) {
                estimateTransformationGN(estimate, grid_map, dataContainer);
            }
            // 角度正则化
            estimate[2] = util::normalize_angle(estimate[2]);
            covMatrix = Eigen::Matrix3f::Zero();
            // covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0).inverse());
            // covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0));
            // 使用Hessian矩阵近似协方差矩阵
            covMatrix = H;
            // 结果转换回物理坐标系下 -- 转换回实际尺度
            return grid_map.getWorldCoordsPose(estimate);
        }
        return beginEstimateWorld;
    }

protected:

    /**
     *  高斯牛顿估计位姿
     * @param estimate      相对于当前Map的位姿
     * @param gridMapUtil   网格地图相关计算工具
     * @param dataPoints    激光数据
     * @return  提示是否有解　－－－　貌似没用上
    */
    bool estimateTransformationGN(Eigen::Vector3f& estimate,
                                                                        const GridMap& grid_map,
                                                                        const LaserPointContainer& dataPoints) {
        /** 核心函数，计算H矩阵和dTr向量(ｂ列向量)---- occGridMapUtil.h 中 **/
        getCompleteHessianDerivs(estimate, grid_map, dataPoints, H, dTr);
        //std::cout << "\nH\n" << H  << "\n";
        //std::cout << "\ndTr\n" << dTr  << "\n";
        // 判断H是否可逆, 判断增量非0,避免无用计算
        if ((H(0, 0) != 0.0f) && (H(1, 1) != 0.0f)) {
            // 求解位姿增量
            Eigen::Vector3f searchDir(H.inverse() * dTr);
            // 角度增量不能太大
            if (searchDir[2] > 0.2f) {
                searchDir[2] = 0.2f;
                std::cout << "SearchDir angle change too large\n";
            } else if (searchDir[2] < -0.2f) {
                searchDir[2] = -0.2f;
                std::cout << "SearchDir angle change too large\n";
            }
            //　更新估计值 --- 结果在地图尺度下
            updateEstimatedPose(estimate, searchDir);
            return true;
        }
        return false;
    }

    void updateEstimatedPose(Eigen::Vector3f &estimate, const Eigen::Vector3f &change) {
        estimate += change;
    }

    /**
     * 使用当前pose投影dataPoints到地图，计算出 H 矩阵 b列向量， 理论部分详见Hector论文： 《A Flexible and Scalable SLAM System with Full 3D Motion Estimation》.
     * @param pose    地图系上的位姿   base -> map
     * @param grid_map 
     * @param dataPoints  已转换为地图尺度的激光点数据
     * @param H   需要计算的 H矩阵
     * @param dTr  需要计算的 g列向量
     */
    void getCompleteHessianDerivs(const Eigen::Vector3f& pose,
                                                                        const GridMap& grid_map,
                                                                        const LaserPointContainer& dataPoints,
                                                                        Eigen::Matrix3f& H,
                                                                        Eigen::Vector3f& dTr) {
        int size = dataPoints.getSize();
        // 获取变换矩阵
        Eigen::Isometry2f transform = 
            Eigen::Translation2f(pose[0], pose[1]) * Eigen::Rotation2Df(pose[2]);
        float sinRot = sin(pose[2]);
        float cosRot = cos(pose[2]);
        H = Eigen::Matrix3f::Zero();
        dTr = Eigen::Vector3f::Zero();
        // 按照公式计算H、b
        for (int i = 0; i < size; ++i) {
            // 地图尺度下的激光坐标系下的激光点坐标
            const Eigen::Vector2f &currPoint(dataPoints.getVecEntry(i));
            // 将激光点坐标转换到地图系上, 通过双线性插值计算栅格概率
            // transformedPointData[0]--通过插值得到的栅格值
            // transformedPointData[1]--栅格值x方向的梯度
            // transformedPointData[2]--栅格值y方向的梯度
            Eigen::Vector3f transformedPointData(interpMapValueWithDerivatives(grid_map, transform * currPoint));
            // 目标函数f(x)  (1-M(Pm))
            float funVal = 1.0f - transformedPointData[0];
            // 计算g列向量的 x 与 y 方向的值
            dTr[0] += transformedPointData[1] * funVal;
            dTr[1] += transformedPointData[2] * funVal;
            // 根据公式计算
            float rotDeriv = ((-sinRot * currPoint.x() - cosRot * currPoint.y()) * transformedPointData[1] +
                              (cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2]);
            // 计算g列向量的 角度 的值
            dTr[2] += rotDeriv * funVal;
            // 计算 hessian 矩阵
            H(0, 0) += util::sqr(transformedPointData[1]);
            H(1, 1) += util::sqr(transformedPointData[2]);
            H(2, 2) += util::sqr(rotDeriv);

            H(0, 1) += transformedPointData[1] * transformedPointData[2];
            H(0, 2) += transformedPointData[1] * rotDeriv;
            H(1, 2) += transformedPointData[2] * rotDeriv;
        }
        // H是对称矩阵，只算上三角就行， 减少计算量。
        H(1, 0) = H(0, 1);
        H(2, 0) = H(0, 2);
        H(2, 1) = H(1, 2);
    }

    /**
     * 双线性插值计算网格中任一点的得分（占据概率）以及该点处的梯度
     * @param coords  激光点地图坐标
     * @return ret(0) 是网格值 ， ret(1) 是栅格值在x方向的导数 ， ret(2)是栅格值在y方向的导数
     */
    Eigen::Vector3f interpMapValueWithDerivatives(const GridMap& grid_map, 
                                                                                                        const Eigen::Vector2f& coords) {
        // 检查coords坐标是否是在地图坐标范围内
        if (grid_map.pointOutOfMapBounds(coords)) {
            return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        }
        // 对坐标进行向下取整，即得到坐标(x0,y0)
        Eigen::Vector2i indMin(coords.cast<int>());
        // 得到双线性插值的因子
        Eigen::Vector2f factors(coords - indMin.cast<float>());
        // 获得地图的X方向最大边界
        int sizeX = grid_map.getSizeX();
        // 计算(x0, y0)点的网格索引值
        int index = indMin[1] * sizeX + indMin[0]; 
        Eigen::Vector4f intensities; /// 记录附近的四个格点的占据概率值
        // 下边这取4个点的栅格值，感觉就是导致hector大地图后计算变慢的原因
        /** 首先判断cache中该地图点在本次scan中是否被访问过，若有则直接取值；没有则立马计算概率值并更新到cache **/
        /** 这个cache的作用是，避免单次scan重复访问同一网格时带来的重复概率计算。地图更新后，网格logocc改变，cache数据就会无效。 **/
        /** 但是这种方式内存开销太大..相当于同时维护两份地图，使用 hash map 是不是会更合适些 **/
        intensities[0] = grid_map.getGridProbabilityMap(index); // 得到M(P00),P00(x0,y0)
        ++index;
        intensities[1] = grid_map.getGridProbabilityMap(index);
        index += sizeX - 1;
        intensities[2] = grid_map.getGridProbabilityMap(index);
        ++index;
        intensities[3] = grid_map.getGridProbabilityMap(index);

        float dx1 = intensities[0] - intensities[1]; // 求得(M(P00) - M(P10))的值
        float dx2 = intensities[2] - intensities[3]; // 求得(M(P01) - M(P11))的值
        float dy1 = intensities[0] - intensities[2]; // 求得(M(P00) - M(P01))的值
        float dy2 = intensities[1] - intensities[3]; // 求得(M(P10) - M(P11))的值
        // 得到双线性插值的因子
        float xFacInv = (1.0f - factors[0]); // 求得(x1-x)的值
        float yFacInv = (1.0f - factors[1]); // 求得(y1-y)的值
        // 计算网格值，计算梯度 --- 原版这里的dx、dy的计算有错误，已经改过来了
        return Eigen::Vector3f(
            ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
                ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),
            -((dx1 * yFacInv) + (dx2 * factors[1])),
            -((dy1 * xFacInv) + (dy2 * factors[0]))
            // -((dx1 * xFacInv) + (dx2 * factors[0])), // 改为： -((dx1 * yFacInv) + (dx2 * factors[1]))
            // -((dy1 * yFacInv) + (dy2 * factors[1]))  // 改为： -((dy1 * xFacInv) + (dy2 * factors[0]))
        );
    }

protected:
    Option option_;  
    Eigen::Vector3f dTr;
    Eigen::Matrix3f H;
};
}


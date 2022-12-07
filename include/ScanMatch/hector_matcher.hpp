
#pragma once 

#include <Eigen/Geometry>
#include "../scan/DataPointContainer.h"
#include "../util/UtilFunctions.h"

namespace Slam2D {

template <typename ConcreteOccGridMapUtil>
class HectorScanMatcher {
public:
    HectorScanMatcher()
    {
    }

    ~HectorScanMatcher()
    {
    }

    /**
     * 实际进行位姿估计的函数
     * @param beginEstimateWorld  位姿初值
     * @param gridMapUtil         网格地图工具,这里主要是用来做坐标变换
     * @param dataContainer       激光数据
     * @param covMatrix           协方差矩阵
     * @param maxIterations       最大迭代次数
     * @return
     */
    Eigen::Vector3f matchData(const Eigen::Vector3f &beginEstimateWorld,
                              ConcreteOccGridMapUtil &gridMapUtil, 
                              const DataContainer &dataContainer,
                              Eigen::Matrix3f &covMatrix, int maxIterations) {
        if (dataContainer.getSize() != 0)
        {

            // 1. 初始pose转换为地图尺度下的pose --仿射变换,包括位姿的尺度和偏移，旋转在 X Y 同尺度变化时保持不变
            Eigen::Vector3f beginEstimateMap(gridMapUtil.getMapCoordsPose(beginEstimateWorld));
            Eigen::Vector3f estimate(beginEstimateMap);

            // 2. 第一次迭代
            estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);

            int numIter = maxIterations;
            /** 3. 多次迭代求解 **/
            for (int i = 0; i < numIter; ++i) {
                estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);
            }

            // 角度正则化
            estimate[2] = util::normalize_angle(estimate[2]);

            covMatrix = Eigen::Matrix3f::Zero();
            //covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0).inverse());
            //covMatrix.block<2,2>(0,0) = (H.block<2,2>(0,0));

            // 使用Hessian矩阵近似协方差矩阵
            covMatrix = H;

            // 结果转换回物理坐标系下 -- 转换回实际尺度
            return gridMapUtil.getWorldCoordsPose(estimate);
        }

        return beginEstimateWorld;
    }

protected:
    /**
     *  高斯牛顿估计位姿
     * @param estimate      位姿初始值
     * @param gridMapUtil   网格地图相关计算工具
     * @param dataPoints    激光数据
     * @return  提示是否有解　－－－　貌似没用上
    */
    bool estimateTransformationLogLh(Eigen::Vector3f &estimate,
                                     ConcreteOccGridMapUtil &gridMapUtil,
                                     const DataContainer &dataPoints) {
        /** 核心函数，计算H矩阵和dTr向量(ｂ列向量)---- occGridMapUtil.h 中 **/
        gridMapUtil.getCompleteHessianDerivs(estimate, dataPoints, H, dTr);
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

    void updateEstimatedPose(Eigen::Vector3f &estimate, const Eigen::Vector3f &change)
    {
        estimate += change;
    }

protected:
    
    Eigen::Vector3f dTr;
    Eigen::Matrix3f H;
};
}


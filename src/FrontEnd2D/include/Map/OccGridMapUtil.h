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

#ifndef __OccGridMapUtil_h_
#define __OccGridMapUtil_h_

#include <cmath>
#include "../Sensor/LaserPointContainer.h"
#include "../util/UtilFunctions.h"

namespace hectorslam
{

/**
* @tparam ConcreteOccGridMap     -----> ??????MapContainer??? ???  GridMap
* @tparam ConcreteCacheMethod    -----> ?????? GridMapCacheArray
*/
template <typename ConcreteOccGridMap, typename ConcreteCacheMethod>
class OccGridMapUtil {
public:
    OccGridMapUtil(const ConcreteOccGridMap *gridMap)
        : concreteGridMap(gridMap), size(0) {
        mapObstacleThreshold = gridMap->getObstacleThreshold();
        cacheMethod.setMapSize(gridMap->getMapGridSize());
    }

    ~OccGridMapUtil() {
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** map??????Pose????????????????????????Pose  --- ????????????**/
    inline Eigen::Vector3f getWorldCoordsPose(const Eigen::Vector3f &mapPose) const { return concreteGridMap->getWorldCoordsPose(mapPose); };
    /** ???????????????Pose????????????????????????Pose **/
    inline Eigen::Vector3f getMapCoordsPose(const Eigen::Vector3f &worldPose) const { return concreteGridMap->getMapCoordsPose(worldPose); };
    /** ??????????????????????????????????????? **/
    inline Eigen::Vector2f getWorldCoordsPoint(const Eigen::Vector2f &mapPoint) const { return concreteGridMap->getWorldCoords(mapPoint); };

    float getUnfilteredGridPoint(int index) const {
        return (concreteGridMap->getGridProbabilityMap(index));
    }

    float getUnfilteredGridPoint(Eigen::Vector2i &gridCoords) const {
        return (concreteGridMap->getGridProbabilityMap(gridCoords.x(), gridCoords.y()));
    }

    /**
     * ?????????????????? pose ?????????????????????????????????????????????sampling???????????????????????????????????????????????????????????????????????? [20]
     * ????????????????????????Use a sampling based covariance estimate,sampling different pose estimates close to
     * scan matching pose and constructing a covariance from those.
     * ???????????????????????????????????????????????????????????????15????????????????????????
     * @param mapPose     ???????????????????????????pose
     * @param dataPoints  ??????????????????????????????
     * @return
     */
    // Eigen::Matrix3f getCovarianceForPose(const Eigen::Vector3f &mapPose, const DataContainer &dataPoints) {
    //     float deltaTransX = 1.5f;
    //     float deltaTransY = 1.5f;
    //     float deltaAng = 0.05f;

    //     float x = mapPose[0];
    //     float y = mapPose[1];
    //     float ang = mapPose[2];

    //     Eigen::Matrix<float, 3, 7> sigmaPoints;
    //     // ????????????  
    //     sigmaPoints.block<3, 1>(0, 0) = Eigen::Vector3f(x + deltaTransX, y, ang);
    //     sigmaPoints.block<3, 1>(0, 1) = Eigen::Vector3f(x - deltaTransX, y, ang);
    //     sigmaPoints.block<3, 1>(0, 2) = Eigen::Vector3f(x, y + deltaTransY, ang);
    //     sigmaPoints.block<3, 1>(0, 3) = Eigen::Vector3f(x, y - deltaTransY, ang);
    //     sigmaPoints.block<3, 1>(0, 4) = Eigen::Vector3f(x, y, ang + deltaAng);
    //     sigmaPoints.block<3, 1>(0, 5) = Eigen::Vector3f(x, y, ang - deltaAng);
    //     sigmaPoints.block<3, 1>(0, 6) = mapPose;

    //     Eigen::Matrix<float, 7, 1> likelihoods;
    //     // ??????????????????????????????
    //     likelihoods[0] = getLikelihoodForState(Eigen::Vector3f(x + deltaTransX, y, ang), dataPoints);
    //     likelihoods[1] = getLikelihoodForState(Eigen::Vector3f(x - deltaTransX, y, ang), dataPoints);
    //     likelihoods[2] = getLikelihoo123dForState(Eigen::Vector3f(x, y + deltaTransY, ang), dataPoints);
    //     likelihoods[3] = getLikelihoodForState(Eigen::Vector3f(x, y - deltaTransY, ang), dataPoints);
    //     likelihoods[4] = getLikelihoodForState(Eigen::Vector3f(x, y, ang + deltaAng), dataPoints);
    //     likelihoods[5] = getLikelihoodForState(Eigen::Vector3f(x, y, ang - deltaAng), dataPoints);
    //     likelihoods[6] = getLikelihoodForState(Eigen::Vector3f(x, y, ang), dataPoints);

    //     float invLhNormalizer = 1 / likelihoods.sum();

    //     std::cout << "\n lhs:\n"
    //               << likelihoods;

    //     Eigen::Vector3f mean(Eigen::Vector3f::Zero());

    //     for (int i = 0; i < 7; ++i) {
    //         mean += (sigmaPoints.block<3, 1>(0, i) * likelihoods[i]);
    //     }

    //     mean *= invLhNormalizer;

    //     Eigen::Matrix3f covMatrixMap(Eigen::Matrix3f::Zero());

    //     for (int i = 0; i < 7; ++i) {
    //         Eigen::Vector3f sigPointMinusMean(sigmaPoints.block<3, 1>(0, i) - mean);
    //         covMatrixMap += (likelihoods[i] * invLhNormalizer) * (sigPointMinusMean * (sigPointMinusMean.transpose()));
    //     }

    //     return covMatrixMap;

    //     //covMatrix.cwise() * invLhNormalizer;
    //     //transform = getTransformForState(Eigen::Vector3f(x-deltaTrans, y, ang);
    // }

    // /**
    //  * ???????????????????????????????????? --- ???????????????????????? ?????????????????????????????????x???y?????????????????????
    //  * ?????? x^theta y^theta ????????????????????? ??? theta^theta????????????????????????????????????????????????????????????
    //  * @param covMatMap
    //  * @return
    //  */
    // Eigen::Matrix3f getCovMatrixWorldCoords(const Eigen::Matrix3f &covMatMap) {

    //     //std::cout << "\nCovMap:\n" << covMatMap;

    //     Eigen::Matrix3f covMatWorld;

    //     float scaleTrans = concreteGridMap->getCellLength();
    //     float scaleTransSq = util::sqr(scaleTrans);

    //     covMatWorld(0, 0) = covMatMap(0, 0) * scaleTransSq;
    //     covMatWorld(1, 1) = covMatMap(1, 1) * scaleTransSq;

    //     covMatWorld(1, 0) = covMatMap(1, 0) * scaleTransSq;
    //     covMatWorld(0, 1) = covMatWorld(1, 0);

    //     covMatWorld(2, 0) = covMatMap(2, 0) * scaleTrans;
    //     covMatWorld(0, 2) = covMatWorld(2, 0);

    //     covMatWorld(2, 1) = covMatMap(2, 1) * scaleTrans;
    //     covMatWorld(1, 2) = covMatWorld(2, 1);

    //     covMatWorld(2, 2) = covMatMap(2, 2);

    //     return covMatWorld;
    // }

    // /**  ?????????????????????????????????????????????????????????????????????...??????????????????????????? **/
    // float getLikelihoodForState(const Eigen::Vector3f &state, const DataContainer &dataPoints) {
    //     float resid = getResidualForState(state, dataPoints);  // ????????? ?????????????????????   ?????????????????????
    //     return getLikelihoodForResidual(resid, dataPoints.getSize());   // ????????????????????????
    // }

    // float getLikelihoodForResidual(float residual, int numDataPoints) {
    //     float numDataPointsA = static_cast<int>(numDataPoints);
    //     float sizef = static_cast<float>(numDataPointsA);

    //     return 1 - (residual / sizef);
    // }

    // /**
    //  * @brief: ????????????????????????????????????
    //  * @param state ??????
    //  * @param {DataContainer} &dataPoints
    //  * @return {*}
    //  */    
    // float getResidualForState(const Eigen::Vector3f &state, const DataContainer &dataPoints) {
    //     int size = dataPoints.getSize();

    //     int stepSize = 1;
    //     float residual = 0.0f;

    //     Eigen::Isometry2f transform(getTransformForState(state));

    //     for (int i = 0; i < size; i += stepSize) {
    //         float funval = 1.0f - interpMapValue(transform * dataPoints.getVecEntry(i));    // ???????????? ???????????????
    //         residual += funval;
    //     }

    //     return residual;
    // }

    // /**
    //  * ????????????????????????????????????????????????????????????????????????????????????interpMapValueWithDerivatives()
    //  */
    // float interpMapValue(const Eigen::Vector2f &coords) {
    //     //check if coords are within map limits.
    //     if (concreteGridMap->pointOutOfMapBounds(coords)) {
    //         return 0.0f;
    //     }

    //     //map coords are alway positive, floor them by casting to int
    //     Eigen::Vector2i indMin(coords.cast<int>());

    //     //get factors for bilinear interpolation
    //     Eigen::Vector2f factors(coords - indMin.cast<float>());

    //     int sizeX = concreteGridMap->getSizeX();

    //     int index = indMin[1] * sizeX + indMin[0];

    //     // get grid values for the 4 grid points surrounding the current coords. Check cached data first, if not contained
    //     // filter gridPoint with gaussian and store in cache.
    //     if (!cacheMethod.containsCachedData(index, intensities[0]))
    //     {
    //         intensities[0] = getUnfilteredGridPoint(index);
    //         cacheMethod.cacheData(index, intensities[0]);
    //     }

    //     ++index;

    //     if (!cacheMethod.containsCachedData(index, intensities[1]))
    //     {
    //         intensities[1] = getUnfilteredGridPoint(index);
    //         cacheMethod.cacheData(index, intensities[1]);
    //     }

    //     index += sizeX - 1;

    //     if (!cacheMethod.containsCachedData(index, intensities[2]))
    //     {
    //         intensities[2] = getUnfilteredGridPoint(index);
    //         cacheMethod.cacheData(index, intensities[2]);
    //     }

    //     ++index;

    //     if (!cacheMethod.containsCachedData(index, intensities[3]))
    //     {
    //         intensities[3] = getUnfilteredGridPoint(index);
    //         cacheMethod.cacheData(index, intensities[3]);
    //     }

    //     float xFacInv = (1.0f - factors[0]);
    //     float yFacInv = (1.0f - factors[1]);

    //     return ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
    //            ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1]));
    // }

    /** ??????????????????**/
    Eigen::Isometry2f getTransformForState(const Eigen::Vector3f &transVector) const
    {
        return Eigen::Translation2f(transVector[0], transVector[1]) * Eigen::Rotation2Df(transVector[2]);
    }

    Eigen::Translation2f getTranslationForState(const Eigen::Vector3f &transVector) const
    {
        return Eigen::Translation2f(transVector[0], transVector[1]);
    }

    /** ??????cache?????????cache??????????????????cache?????????????????? **/
    void resetCachedData()
    {
        cacheMethod.resetCache();
    }

    /** ???????????????.. **/
    void resetSamplePoints()
    {
        samplePoints.clear();
    }

    const std::vector<Eigen::Vector3f> &getSamplePoints() const
    {
        return samplePoints;
    }

protected:
    Eigen::Vector4f intensities; /// ?????????????????????????????????????????????
    ConcreteCacheMethod cacheMethod; /// ?????? GridMapCacheArray
    const ConcreteOccGridMap *concreteGridMap; /// ?????? GridMap ---> OccGridMapBase
    std::vector<Eigen::Vector3f> samplePoints; // ????????????
    int size; // ???????
    float mapObstacleThreshold; //// ??????????????????
};
} // namespace hectorslam

#endif

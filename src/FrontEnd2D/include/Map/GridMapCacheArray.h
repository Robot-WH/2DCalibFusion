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

#ifndef __GridMapCacheArray_h_
#define __GridMapCacheArray_h_

#include <Eigen/Core>

struct CachedMapElement {
public:
    float val;
    int index;
};

/**
 * 网格缓存，类似于计算机的cache，保存最近访问过的网格的概率值（非logOddsVal值）。
 * 避免重复访问同一网格时，重复计算其概率值。（Cell中只保存了logOddsVal值）
 * Caches filtered grid map accesses in a two dimensional array of the same size as the map.
 */
class GridMapCacheArray {
public:
    /**
   * Constructor
   */
    GridMapCacheArray()
        : cacheArray_(0), arrayDimensions_(-1, -1) {
        currCacheIndex_ = 0;
    }

    /**
   * Destructor
   */
    ~GridMapCacheArray() {
        deleteCacheArray();
    }

    /**
   * Resets/deletes the cached data
   * 只有当  CachedMapElement.index == currCacheIndex_ 时，元素有效
   * 改变currCacheIndex的值，使当前cache内所有元素无效
   */
    void resetCache() {
        currCacheIndex_++;
    }

    /**
   * Checks wether cached data for coords is available. If this is the case, writes data into val.
   * @param coords The coordinates
   * @param val Reference to a float the data is written to if available
   * @return Indicates if cached data is available
   */
    bool containsCachedData(int index, float &val) {
        const CachedMapElement& elem(cacheArray_[index]);

        if (elem.index == currCacheIndex_) {
            val = elem.val;
            return true;
        } else {
            return false;
        }
    }

    /**
     * Caches float value val for coordinates coords.
     * @param coords The coordinates
     * @param val The value to be cached for coordinates.
     */
    void cacheData(int index, float val) {
        CachedMapElement &elem(cacheArray_[index]);
        elem.index = currCacheIndex_;
        elem.val = val;
    }

    /**
   * Sets the map size and resizes the cache array accordingly
   * @param sizeIn The map size.
   */
    void setMapSize(const Eigen::Vector2i &newDimensions) {
        setArraySize(newDimensions);
    }

protected:
    /**
   * Creates a cache array of size sizeIn.
   * @param sizeIn The size of the array
   */
    void createCacheArray(const Eigen::Vector2i& newDimensions) {
        arrayDimensions_ = newDimensions;

        int sizeX = arrayDimensions_[0];
        int sizeY = arrayDimensions_[1];

        int size = sizeX * sizeY;

        cacheArray_ = new CachedMapElement[size];

        for (int x = 0; x < size; ++x) {
            cacheArray_[x].index = -1;
        }
    }

    /**
   * Deletes the existing cache array.
   */
    void deleteCacheArray() {
        delete[] cacheArray_;
    }

    /**
     * Sets a new cache array size
     */
    void setArraySize(const Eigen::Vector2i &newDimensions) {
        if (this->arrayDimensions_ != newDimensions) {
            if (cacheArray_ != 0) {
                deleteCacheArray();
                cacheArray_ = 0;
            }
            createCacheArray(newDimensions);
        }
    }

protected:
    CachedMapElement* cacheArray_; ///< Array used for caching data.
    int currCacheIndex_;           ///< The cache iteration index value

    Eigen::Vector2i arrayDimensions_; ///< The size of the array
};

#endif

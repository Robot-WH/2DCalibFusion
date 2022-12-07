/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-22 16:51:55
 * @Description: 
 * @Others: 
 */
#pragma once 

namespace Slam2D {

    /**
     * @brief: 2d laser-轮速-IMU 估计器初始化  
     * @details: 估计 IMU 角速度 偏置 ， 估计 轮速-laser 外参以及轮速 -内参 初始值 
     */    
    class Initialize
    {
        public:

            /**
             * @brief: 初始化函数 
             * @details: 
             * @param extrinsic_calib 是否标定外参
             * @param intrinsic_calib 是否标定内参 
             * @return 初始化是否完成
             */            
            bool Init(bool extrinsic_calib = false, bool intrinsic_calib = false)
            {

            }
        private:
    };

}; 

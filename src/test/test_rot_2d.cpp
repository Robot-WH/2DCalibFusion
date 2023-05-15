/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-03-25 20:51:37
 * @Description: 
 * @Others: 
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <deque>
#include <mutex>
#include <thread>
#include <Eigen/Dense>

int main()
{
    // 验证 在2D转动中， 不同旋转的两个坐标系， 旋转同一个角度后，旋转矩阵变化相同
    Eigen::AngleAxisd A(0, Eigen::Vector3d(0, 0, 1));
    Eigen::AngleAxisd Ext(M_PI / 9, Eigen::Vector3d(0, 0, 1));   // 外参  

    Eigen::Quaterniond A_q(A);
    Eigen::Quaterniond Ext_q(Ext);

    Eigen::Matrix3d A_R = A.toRotationMatrix();
    Eigen::Matrix3d Ext_R = Ext_q.toRotationMatrix();    // 外参矩阵
    Eigen::Matrix3d B_R = A_R * Ext_R;  

    std::cout<<"A_R: "<<std::endl<< A_R.matrix()<<" B_R: "<<std::endl<<B_R.matrix()<<std::endl;

    // A顺时针转动一个角度
    Eigen::AngleAxisd A_(M_PI / 6, Eigen::Vector3d(0, 0, 1));   // 旋转后的A
    Eigen::Quaterniond A__q(A_);
    Eigen::Matrix3d A__R = A_.toRotationMatrix();
    // A的变化量
    Eigen::Matrix3d delta_A_R = A_R.transpose() * A__R; 

    std::cout<<"delta A_R: "<<std::endl<< delta_A_R.matrix()<<std::endl;

    Eigen::Matrix3d B__R = A__R * Ext_R; 
    // B的变化量
    Eigen::Matrix3d delta_B_R = B_R.transpose() * B__R; 
    std::cout<<"delta_B_R: "<<std::endl<< delta_B_R.matrix()<<std::endl;
    // 经验证  A，B的变化量相等 
    
    std::cout<<"delta A_R * RAB: "<<std::endl<< (delta_A_R*Ext_R).matrix()<<std::endl<<
    "RAB *  delta B_R: "<<std::endl<<(Ext_R*delta_B_R).matrix()<<std::endl;
    return 1;
}

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
#include "msa2d/common/Pose2d.hpp"

double normalizeAngle(double angle) {

    if (angle <= -M_PI) {
        angle += 2 * M_PI; // 将角度调整为[-PI, PI]的范围
    } else if (angle > M_PI) {
        angle -= 2 * M_PI; // 将角度调整为[-PI, PI]的范围
    }

    return angle;
}

double shortestPathInterpolateAngle(double startAngle, double endAngle, double ratio) {
    // 规范化起始角度和结束角度
    startAngle = normalizeAngle(startAngle);
    endAngle = normalizeAngle(endAngle);

    // 计算最短路径的插值角度
    double difference = endAngle - startAngle;

    if (difference > M_PI) {
        difference -= 2 * M_PI;
    } else if (difference < -M_PI) {
        difference += 2 * M_PI;
    }

    double interpolatedAngle = startAngle + ratio * difference;

    // 规范化插值角度
    interpolatedAngle = normalizeAngle(interpolatedAngle);

    return interpolatedAngle;
}

int main()
{
    // 验证 在2D转动中， 不同旋转的两个坐标系， 旋转同一个角度后，旋转矩阵变化相同
    // Eigen::AngleAxisd A(0, Eigen::Vector3d(0, 0, 1));
    // Eigen::AngleAxisd Ext(M_PI / 9, Eigen::Vector3d(0, 0, 1));   // 外参  

    // Eigen::Quaterniond A_q(A);
    // Eigen::Quaterniond Ext_q(Ext);

    // Eigen::Matrix3d A_R = A.toRotationMatrix();
    // Eigen::Matrix3d Ext_R = Ext_q.toRotationMatrix();    // 外参矩阵
    // Eigen::Matrix3d B_R = A_R * Ext_R;  

    // std::cout<<"A_R: "<<std::endl<< A_R.matrix()<<" B_R: "<<std::endl<<B_R.matrix()<<std::endl;

    // // A顺时针转动一个角度
    // Eigen::AngleAxisd A_(M_PI / 6, Eigen::Vector3d(0, 0, 1));   // 旋转后的A
    // Eigen::Quaterniond A__q(A_);
    // Eigen::Matrix3d A__R = A_.toRotationMatrix();
    // // A的变化量
    // Eigen::Matrix3d delta_A_R = A_R.transpose() * A__R; 

    // std::cout<<"delta A_R: "<<std::endl<< delta_A_R.matrix()<<std::endl;

    // Eigen::Matrix3d B__R = A__R * Ext_R; 
    // // B的变化量
    // Eigen::Matrix3d delta_B_R = B_R.transpose() * B__R; 
    // std::cout<<"delta_B_R: "<<std::endl<< delta_B_R.matrix()<<std::endl;
    // // 经验证  A，B的变化量相等 
    
    // std::cout<<"delta A_R * RAB: "<<std::endl<< (delta_A_R*Ext_R).matrix()<<std::endl<<
    // "RAB *  delta B_R: "<<std::endl<<(Ext_R*delta_B_R).matrix()<<std::endl;

    double startAngle = -1.57; // 起始角度
    double endAngle = 2.2; // 结束角度
    double ratio = 0.8; // 插值比例

    double interpolatedAngle = shortestPathInterpolateAngle(startAngle, endAngle, ratio);
    std::cout << "Interpolated angle: " << interpolatedAngle << std::endl;

    msa2d::Pose2d p1(1.1, 1.2, 7);
    msa2d::Pose2d p2(1.1, 1.2, 7);
    msa2d::Pose2d p3(1.11, 1.2, 7);

    if (p1 == p2) {
        std::cout << "p1 == p2" << std::endl;
    } 
    if (p1 == p3) {
        std::cout << "p1 == p3" << std::endl;
    } else {
        std::cout << "p1 != p3" << std::endl;
    }

    if (p1 != p3) {
        std::cout << "p1 != p3" << std::endl;
    }

    msa2d::Pose2d T1(0, 0, 0.36);
    Eigen::Vector2f p_1(3, 3);
    msa2d::Pose2d T2(0, 0, 0.36 + M_PI / 4);

    msa2d::Pose2d T21 = T2.inv() * T1; 
    Eigen::Vector2f p_2 = T21 * p_1;
    std::cout << "p_2： " << p_2.transpose() << std::endl;

    p_1 = T21.inv() * p_2;
    std::cout << "p_1： " << p_1.transpose() << std::endl;

    Eigen::Matrix2f R;
    R << 1, 2,
    3, 4;
    Eigen::Vector2f l(1, 1);
    std::cout << "v: " << R * l << std::endl;
    std::cout << "R: " << R << std::endl;

    return 1;
}

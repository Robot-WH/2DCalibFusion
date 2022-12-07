
// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef POSE_2D_HPP_
#define POSE_2D_HPP_

#include <Eigen/Core>

class Pose2d {
    public:
        Pose2d() {
            x_ = 0.0;
            y_ = 0.0;
            yaw_ = 0.0;
            orientation_.setIdentity(); 
        }
        
        Pose2d(double x, double y, double theta) : x_(x), y_(y), yaw_(theta) {
            NormAngle(yaw_);
            // yaw 转 四元数
            Eigen::AngleAxisd rollAngle(0,Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(0,Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw_,Eigen::Vector3d::UnitZ());
            orientation_ = rollAngle * pitchAngle * yawAngle;
        }

        Pose2d(double x, double y, Eigen::Quaterniond const& orientation) : x_(x), y_(y) {
            orientation_ = orientation.normalized();  
            Eigen::Vector3d eulerAngle=orientation_.matrix().eulerAngles(0,1,2);    // 分解为 0 <- 1 <- 2 顺序的欧拉角
            yaw_ = eulerAngle[2];  
        }

        void SetIdentity() {
            orientation_.setIdentity(); 
            x_ = 0.0;
            y_ = 0.0;
            yaw_ = 0.0;
        }

        void SetTransform(double const& x, double const& y) {
            x_ = x;
            y_ = y;
        }

        void SetRotation(Eigen::Quaterniond const& orientation) {
            orientation_ = orientation.normalized();  
            Eigen::Vector3d eulerAngle = orientation_.matrix().eulerAngles(0,1,2);    // 分解为 0 <- 1 <- 2 顺序的欧拉角
            yaw_ = eulerAngle[2];  
            NormAngle(yaw_); 
        }

        void SetRotation(double const& yaw) {
            yaw_ = yaw;
            NormAngle(yaw_); 
            Eigen::AngleAxisd rollAngle(0,Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(0,Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw_,Eigen::Vector3d::UnitZ());
            orientation_ = rollAngle * pitchAngle * yawAngle;
        }

        double GetX() {
            return x_;
        }

        double GetY() {
            return y_;
        }

        double GetYaw() {
            return yaw_;
        }

        Eigen::Quaterniond GetOrientation() {
            return orientation_; 
        }  

        // 重载pose的乘法   Pose2d (x,y,theta)  p1*p2   T1*T2  
        const Pose2d operator*(const Pose2d& p2) {  
            Pose2d p;
            Eigen::Matrix2d R;
            // 构造旋转矩阵
            R << cos(yaw_), -sin(yaw_),
            sin(yaw_), cos(yaw_);
            Eigen::Vector2d pt2(p2.x_, p2.y_);          
            Eigen::Vector2d pt = R * pt2 + Eigen::Vector2d(x_, y_);     // t = Rt + t 
            
            p.x_ = pt(0);
            p.y_ = pt(1);
            p.yaw_ = yaw_ + p2.yaw_;         // 这里注意  由于是2D平面  所以旋转可以转换为一个旋转向量 z*theta,  
                                                                        // z为z轴, 即绕着z轴旋转   因此两次旋转 R1*R2 的结果 可以看成绕着z轴连续旋转2次 , 因此角度直接相加 
            NormAngle(p.yaw_);                                       // 规范化 
            p.orientation_ = orientation_ * p2.orientation_;   // 四元数更新
            p.orientation_.normalize(); 
            return p;
        }  

        // 重载  T*P
        const Eigen::Vector2d operator*(const Eigen::Vector2d& p) {  
            Eigen::Matrix2d R;
            R << cos(yaw_), -sin(yaw_),
            sin(yaw_), cos(yaw_);
            Eigen::Vector2d t(x_, y_);
            return R * p + t;
        }  

        Pose2d inv() {
            double x = - ( cos(yaw_) * x_ + sin(yaw_) * y_);
            double y = - ( -sin(yaw_) * x_ + cos(yaw_) * y_);
            double theta = - yaw_;
            return Pose2d(x, y, theta);
        }

        // 对角度进行标准化     [-M_PI, M_PI]
        static void NormAngle(double& angle) {        
            if(angle >= M_PI)
                angle -= 2.0*M_PI;
            if(angle < -M_PI)
                angle += 2.0*M_PI;
        }

        double x_, y_, yaw_; 
        Eigen::Quaterniond orientation_;  
}; //class Pose2d

#endif


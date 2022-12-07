#pragma once 

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Core>

// using namespace geometry_msgs;  

class RosUtils {
public:
    // ros 读取参数   
    template <typename T>
    static T RosReadParam(ros::NodeHandle &n, std::string name) {
        T ans;
        if (n.getParam(name, ans)) {
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        } else {
            ROS_ERROR_STREAM("Failed to load " << name);
        }
        return ans;
    }

    static geometry_msgs::PoseStamped GetPoseStamped(const Eigen::Vector3f &pose, 
                                                                                   const ros::Time &stamp, 
                                                                                   const std::string &frame_id) {
        geometry_msgs::PoseStamped stampedPose; 
        stampedPose.header.stamp = stamp;
        stampedPose.header.frame_id = frame_id;

        stampedPose.pose.position.x = pose.x();
        stampedPose.pose.position.y = pose.y();

        stampedPose.pose.orientation.w = cos(pose.z() * 0.5f);
        stampedPose.pose.orientation.z = sin(pose.z() * 0.5f);
        return stampedPose;  
    }

    static tf::Transform GetTFTransform(const Eigen::Matrix<float, 6, 1>&pose) {
        tf::Transform poseTransform; 
        geometry_msgs::Pose pose_;
        pose_.position.x = pose(0, 0);
        pose_.position.y = pose(1, 0);
        pose_.position.z = pose(2, 0);

        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(pose(3, 0), Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pose(4, 0), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(pose(5, 0), Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond quaternion;
        quaternion=yawAngle * pitchAngle * rollAngle;

        pose_.orientation.w = quaternion.w();
        pose_.orientation.x = quaternion.x();
        pose_.orientation.y = quaternion.y();
        pose_.orientation.z = quaternion.z();

        //Fill tf tansform
        tf::poseMsgToTF(pose_, poseTransform);
        return poseTransform;  
    }

    static tf::Transform GetTFTransform(const geometry_msgs::Pose &pose) {
        tf::Transform poseTransform; 
        //Fill tf tansform
        tf::poseMsgToTF(pose, poseTransform);
        return poseTransform;  
    }

    static geometry_msgs::PoseWithCovarianceStamped 
    GetPoseWithCovarianceStamped(const Eigen::Vector3f& pose, 
                                                                        const Eigen::Matrix3f& poseCov,
                                                                        const ros::Time& stamp, 
                                                                        const std::string& frame_id) {
        //Fill stampedPose
        std_msgs::Header header;
        header.stamp = stamp;
        header.frame_id = frame_id;

        geometry_msgs::Pose pose_;
        pose_.position.x = pose.x();
        pose_.position.y = pose.y();

        pose_.orientation.w = cos(pose.z() * 0.5f);
        pose_.orientation.z = sin(pose.z() * 0.5f);
        
        geometry_msgs::PoseWithCovarianceStamped covPose; 
        covPose.header = header;
        covPose.pose.pose = pose_;
        // 6x6的矩阵
        boost::array<double, 36> &cov(covPose.pose.covariance);

        cov[0] = static_cast<double>(poseCov(0, 0));   // x-x
        cov[7] = static_cast<double>(poseCov(1, 1)); // y-y
        cov[35] = static_cast<double>(poseCov(2, 2)); // z-z

        double xyC = static_cast<double>(poseCov(0, 1));  // x-y
        cov[1] = xyC;
        cov[6] = xyC;

        double xaC = static_cast<double>(poseCov(0, 2)); // x-z
        cov[5] = xaC;
        cov[30] = xaC;

        double yaC = static_cast<double>(poseCov(1, 2)); // y-z
        cov[11] = yaC;
        cov[31] = yaC;
        return covPose; 
    }
};

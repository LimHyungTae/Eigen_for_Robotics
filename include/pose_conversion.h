#ifndef POSE_CONVERSION_H
#define POSE_CONVERSION_H

/** \cond */

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

/** \endcond */

namespace pose_conversion
{
    /**
     * @brief Rotation matrix / Transformation matrix of Eigen -> tf::Matrix3x3
     **/
    tf::Matrix3x3 eigenRot2RotMat(const Eigen::Matrix3d& eigenRotMat);
    tf::Matrix3x3 getRotMat(const Eigen::Matrix4d& eigenPose); //

    /**
     * @brief tf::Matrix3x3 -> Rotation matrix / Transformation matrix of Eigen
     **/
    Eigen::Matrix3d rotMat2EigenRot(const tf::Matrix3x3& rotMat); //
    void fillEigenPose(const tf::Matrix3x3& rotMat, Eigen::Matrix4d &eigenPose); //

    /**
     * @brief geometry_msgs/Pose <-> Transformation matrix of Eigen
     **/
    Eigen::Matrix4d geoPose2eigen(const geometry_msgs::Pose& geoPose);
    geometry_msgs::Pose eigen2geoPose(const Eigen::Matrix4d& eigenPose);

    /**
     * @brief xyzrpy <-> Transformation matrix of Eigen
     **/
    Eigen::VectorXd eigen2xyzrpy(const Eigen::Matrix4d& eigenPose);
    Eigen::Matrix4d xyzrpy2eigen(const double& x, const double& y, const double& z,
                                 const double& roll, const double& pitch, const double& yaw);
    Eigen::Matrix4d xyzrpy2eigen(const Eigen::VectorXd& xyzrpy);

    /**
     * @brief geometry_msgs/Pose <-> xyzrpy
     **/
    Eigen::VectorXd geoPose2xyzrpy(const geometry_msgs::Pose& geoPose);
    geometry_msgs::Pose xyzrpy2geoPose(const Eigen::VectorXd& xyzrpy);

}


#endif

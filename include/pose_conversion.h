#ifndef POSE_CONVERSION_H
#define POSE_CONVERSION_H

/** \cond */

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

/** \endcond */

namespace pose_conversion
{
    /**
     * @brief Rotation matrix / Transformation matrix of Eigen -> tf::Matrix3x3
     **/
    tf::Matrix3x3 eigenRot2RotMat(const Eigen::Matrix3f& eigenRotMat);
    tf::Matrix3x3 getRotMat(const Eigen::Matrix4f& eigenPose); //

    /**
     * @brief tf::Matrix3x3 -> Rotation matrix / Transformation matrix of Eigen
     **/
    Eigen::Matrix3f rotMat2EigenRot(const tf::Matrix3x3& rotMat); //
    void fillEigenPose(const tf::Matrix3x3& rotMat, Eigen::Matrix4f &eigenPose); //

    /**
     * @brief geometry_msgs/Pose <-> Transformation matrix of Eigen
     **/
    Eigen::Matrix4f geoPose2eigen(const geometry_msgs::Pose& geoPose);
    geometry_msgs::Pose eigen2geoPose(const Eigen::Matrix4f& eigenPose);

    /**
     * @brief xyzrpy <-> Transformation matrix of Eigen
     **/
    Eigen::VectorXf eigen2xyzrpy(const Eigen::Matrix4f& eigenPose);
    Eigen::Matrix4f xyzrpy2eigen(const float& x, const float& y, const float& z,
                                 const float& roll, const float& pitch, const float& yaw);
    Eigen::Matrix4f xyzrpy2eigen(const Eigen::VectorXf& xyzrpy);

    /**
     * @brief geometry_msgs/Pose <-> xyzrpy
     **/
    Eigen::VectorXf geoPose2xyzrpy(const geometry_msgs::Pose& geoPose);
    geometry_msgs::Pose xyzrpy2geoPose(const Eigen::VectorXf& xyzrpy);

}


#endif

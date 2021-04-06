#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
//#include <pose_conversion.h>
#include <utils/pose_conversion.h>

using namespace std;
using namespace Eigen;
// using namespace utils;
//using namespace unavlib;

int main(){
  double roll, pitch, yaw;

  geometry_msgs::Pose geoPoseInput; /** Test case 1. geoPose -> eigenPose / xyzrpy */

  geoPoseInput.position.x = 7.9;
  geoPoseInput.position.y = 6.5;
  geoPoseInput.position.z = 4.4;

  geoPoseInput.orientation.x = 0.062221;
  geoPoseInput.orientation.y = 0.062221;
  geoPoseInput.orientation.z = 0.777625;
  geoPoseInput.orientation.w = 0.6225425;

  Matrix4f eigenPoseInput; /** Test case 2. eigenPose -> geoPose / xyzrpy */
  eigenPoseInput << -0.6190476, 0.7824968, -0.0669241, 3.5,
                    -0.7619048, -0.6190476, -0.1904762, 4.2,
                    -0.1904762, -0.0669241,  0.9794080, 1.0,
                             0,          0,          0, 1.0;
  VectorXf xyzrpyInput(6); /** Test case 3. xyzrpy -> geoPose / eigenPose */
  xyzrpyInput << -4.2, 2.7, 3, 0.02, -1.2, 0.75;

  geometry_msgs::Pose geoPose;
  Matrix4f eigenPose = Matrix4f::Identity();
  VectorXf xyzrpy(6);

  /**
   * @brief geometry_msgs/Pose -> Eigen::Matrix4f
   */
  eigenPose = pose_conversion::geoPose2eigen(geoPoseInput);
  cout<<eigenPose<<endl;
  /**
   * @brief geometry_msgs/Pose -> xyzrpy
   */
  xyzrpy = pose_conversion::geoPose2xyzrpy(geoPoseInput);
  cout<<xyzrpy<<endl;

  cout<<"=================="<<endl;

  /**
   * @brief Eigen::Matrix4f -> geometry_msgs/Pose
   */
  geoPose = pose_conversion::eigen2geoPose(eigenPoseInput);
  cout<<geoPose.position.x<<", "<<geoPose.position.y<<", "<<geoPose.position.z<<endl;
  cout<<geoPose.orientation.x<<", "<<geoPose.orientation.y<<", "<<geoPose.orientation.z<<", "<<geoPose.orientation.w<<endl;


  /**
   * @brief Eigen::Matrix4f -> Eigen::VectorXf xyzrpy
   */
  xyzrpy = pose_conversion::eigen2xyzrpy(eigenPoseInput);
  cout<<xyzrpy<<endl;

  cout<<"=================="<<endl;

  /**
   * @brief xyzrpy -> Eigen::Matrix4f
   */
  eigenPose = pose_conversion::xyzrpy2eigen(xyzrpyInput);
  cout<<"After: "<<endl;
  cout<<eigenPose<<endl;
  /**
   * @brief xyzrpy  -> geometry_msgs/Pose
   */
  geoPose = pose_conversion::xyzrpy2geoPose(xyzrpyInput);
  cout<<geoPose.position.x<<", "<<geoPose.position.y<<", "<<geoPose.position.z<<endl;
  cout<<geoPose.orientation.x<<", "<<geoPose.orientation.y<<", "<<geoPose.orientation.z<<", "<<geoPose.orientation.w<<endl;

  cout<<"=================="<<endl;

//  tf::Matrix3x3 mat;
//  mat = pose_conversion::getRotMat(eigenPose);
//  cout<<mat[0][0]<<mat[0][1]<<mat[0][2]<<endl;
//  cout<<mat[1][0]<<mat[1][1]<<mat[1][2]<<endl;
//  cout<<mat[2][0]<<mat[2][1]<<mat[2][2]<<endl;
//  Matrix3f eigen_test;
//  eigen_test = pose_conversion::rotMat2EigenRot(mat);
//  cout<<eigen_test<<endl;

//  Matrix4f eigen4x4 = Matrix4f::Identity();
//  pose_conversion::fillEigenPose(mat, eigen4x4);
//  cout<<eigen4x4<<endl;
//  tf::Matrix3x3 mat2;
//  mat2 = pose_conversion::getRotMat(eigen4x4);
//  cout<<mat2[0][0]<<mat2[0][1]<<mat2[0][2]<<endl;
//  cout<<mat2[1][0]<<mat2[1][1]<<mat2[1][2]<<endl;
//  cout<<mat2[2][0]<<mat2[2][1]<<mat2[2][2]<<endl;

//  Matrix4d dddd;
//  dddd =pose_conversion::eigenf2eigend(eigen4x4);
//  cout<<dddd<<endl;
//  eigen4x4 = pose_conversion::eigend2eigenf(dddd);
//  cout<<eigen4x4<<endl;

  return 0;
}

#include "pose_conversion.h"
#include "utils.hpp"
using namespace std;
using namespace Eigen;

int main(){
    vector<nav_msgs::Odometry> odomBuf;
    loadOdom("/home/shapelim/catkin_ws/src/pose_conversion/materials/odom_poses.txt", odomBuf);

    nav_msgs::Odometry o = odomBuf[100];
    const auto & ori = o.pose.pose.orientation;
    const auto & posit = o.pose.pose.position;
    // Eigen:: w, x, y, z order
    // tf: x, y, z, w order
    Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
    cout << q.toRotationMatrix() << endl;
    Eigen::Matrix3d rotDouble = q.toRotationMatrix().cast<double>();

    Eigen::Vector3f p0;
    p0 << posit.x, posit.y, posit.z;

    Eigen::Vector3f p1(posit.x, posit.y, posit.z);

    cout << " ----------- " << endl;
    cout << p0.transpose() << endl;
    cout << p1.transpose() << endl;

    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf.topRightCorner(3, 1) = p0;

    cout << tf << endl;


  return 0;
}

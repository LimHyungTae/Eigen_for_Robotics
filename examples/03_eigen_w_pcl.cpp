#include "pose_conversion.h"
#include <iomanip>
#include "utils.hpp"

using namespace std;
using namespace Eigen;

void getTf4x4(const nav_msgs::Odometry o, Eigen::Matrix4f& tf4x4) {
    const auto &ori = o.pose.pose.orientation;
    const auto &posit = o.pose.pose.position;
    Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
    Eigen::Vector3f t(posit.x, posit.y, posit.z);

    tf4x4 = Eigen::Matrix4f::Identity();
    tf4x4.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf4x4.block<3, 1>(0, 3) = t;
}

Eigen::Affine3f tf2affine(const Eigen::Matrix4f& tf) {
    Eigen::Affine3f affineTf = Eigen::Affine3f::Identity();
    affineTf.translation() << tf.topRightCorner(3, 1);
    affineTf.rotate(tf.block<3, 3>(0, 0));
    return affineTf;
}

int main(){
    vector<nav_msgs::Odometry> odomBuf;
    loadOdom(ABS_FILE_PATH, odomBuf);

    int arbitraryNum = 100;
    nav_msgs::Odometry o = odomBuf[arbitraryNum];
    Eigen::Matrix4f tf4x4;
    getTf4x4(o, tf4x4);
    cout << tf4x4 << endl;

    pcl::PointXYZ srcPoint(20.0, 20.0, 5.0);
    // Ver. 1
    pcl::PointXYZ newPointNaive;
    newPointNaive.x = tf4x4(0,0) * srcPoint.x + tf4x4(0,1) * srcPoint.y + tf4x4(0,2) * srcPoint.z + tf4x4(0,3);
    newPointNaive.y = tf4x4(1,0) * srcPoint.x + tf4x4(1,1) * srcPoint.y + tf4x4(1,2) * srcPoint.z + tf4x4(1,3);
    newPointNaive.z = tf4x4(2,0) * srcPoint.x + tf4x4(2,1) * srcPoint.y + tf4x4(2,2) * srcPoint.z + tf4x4(2,3);
    cout << "Ver.1 : " << newPointNaive.x << ", " << newPointNaive.y << ", " << newPointNaive.z << endl;

    // Ver. 2 using `getVector3fMap()`
    Eigen::Vector3f newVec = tf4x4.block<3, 3>(0, 0) * srcPoint.getVector3fMap() + tf4x4.topRightCorner(3, 1);
    pcl::PointXYZ newPoint1(newVec(0), newVec(1), newVec(2));
    cout << "Ver.2 : " <<  newPoint1.x << ", " << newPoint1.y << ", " << newPoint1.z << endl;

    return 0;
}

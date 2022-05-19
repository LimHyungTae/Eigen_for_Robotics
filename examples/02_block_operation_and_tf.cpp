#include "pose_conversion.h"
#include <iomanip>
#include "utils.hpp"

using namespace std;
using namespace Eigen;

void getTf4x4WithBlock(const Eigen::Quaternionf &q, const Eigen::Vector3f &t, Eigen::Matrix4f &tf4x4) {
    tf4x4 = Eigen::Matrix4f::Identity();
    tf4x4.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf4x4.block<3, 1>(0, 3) = t;
}

void getTf4x4WithTopRightCorner(const Eigen::Quaternionf &q, const Eigen::Vector3f &t, Eigen::Matrix4f &tf4x4) {
    tf4x4 = Eigen::Matrix4f::Identity();
    tf4x4.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf4x4.topRightCorner(3, 1) = t;
}

int main(){
    vector<nav_msgs::Odometry> odomBuf;
    loadOdom(ABS_FILE_PATH, odomBuf);

    int arbitraryNum = 100;
    nav_msgs::Odometry o = odomBuf[arbitraryNum];
    const auto & ori = o.pose.pose.orientation;
    const auto & posit = o.pose.pose.position;

    // Eigen: w, x, y, z order
    // tf   : x, y, z, w order
    Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
    Eigen::Vector3f t(posit.x, posit.y, posit.z);

    // `head` and `tail` in Eigen Vector
    cout << t.transpose() << endl;
    cout << t.transpose().head(2) << endl;
    cout << t.transpose().tail(2) << endl;

    // Block operation to assign values to the transformation matrix in different ways
    // But, the results are same!
    Eigen::Matrix4f tf4x4WithBlock, tf4x4WithTopRightCorner;
    getTf4x4WithBlock(q, t, tf4x4WithBlock);
    getTf4x4WithTopRightCorner(q, t, tf4x4WithTopRightCorner);

    cout << "---------" << endl;
    cout << tf4x4WithBlock << endl;
    cout << " vs " <<endl;
    cout << tf4x4WithTopRightCorner << endl;

    // Additional block-wise operations
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf.bottomLeftCorner(1, 3) << 1, 2, 3;
    tf.topRightCorner(3, 1) << 4, 5, 6;
    cout << "---------" << endl;
    cout << tf << endl;
    tf.row(2) << 10, 20, 30, 40;
    tf.col(2) << -10, -20, -30, -40;
    cout << tf << endl;

    // Multiplication of transformation matrix
    Eigen::Vector3f point(20.0, 20.0, 5.0);
    Eigen::Vector4f pointHomogeneous;
    pointHomogeneous << point, 1;
    cout << "Before transformation:" << endl;
    cout << pointHomogeneous.transpose() << endl;

    // Transformed point
    Eigen::Vector4f transformed;
    transformed = tf4x4WithBlock * pointHomogeneous;

    cout << " After transformation:" << endl;
    cout << transformed.head(3).transpose() << endl;
    // Note that `Quaternionf` inherently rotate point by simple multiplication!!!!
    cout << (q * point + t).transpose() << endl;

    // Affine3f
    // IMPORTANT: `Eigen::Affine3f` must be initialized by identity!
    Eigen::Affine3f affineTf = Eigen::Affine3f::Identity();
    // Eigen::Matrix4f -> Eigen::Affine3f
    affineTf.translation() << tf4x4WithBlock.topRightCorner(3, 1);
    affineTf.rotate(tf4x4WithBlock.block<3, 3>(0, 0));
    cout << affineTf.matrix() << endl;
    cout << (affineTf * point).transpose() << endl;

    Eigen::Matrix3f rotByAffine = affineTf.rotation();
    Eigen::Vector3f transByAffine = affineTf.translation();
    cout << "Rotation: " << endl << rotByAffine << endl;
    cout << "Translation: " << transByAffine.transpose() << endl;

    // WARNING: `rotate()` actually rotate the matrix!
    // That is, it operates multiplication of rotation matrix, not just assigns the naive values!!!
    affineTf.rotate(tf4x4WithBlock.block<3, 3>(0, 0));
    cout << "Same with upper matrix?: " << endl;
    cout << affineTf.matrix() << endl;

    return 0;
}

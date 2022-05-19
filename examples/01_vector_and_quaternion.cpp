#include "pose_conversion.h"
#include <iomanip>
#include "utils.hpp"

using namespace std;
using namespace Eigen;

int main(){
    vector<nav_msgs::Odometry> odomBuf;
    loadOdom(ABS_FILE_PATH, odomBuf);

    int arbitraryNum = 100;
    nav_msgs::Odometry o = odomBuf[arbitraryNum];
    const auto & ori = o.pose.pose.orientation;
    const auto & posit = o.pose.pose.position;

    // For rotation <-> quaternion
    // Eigen:: w, x, y, z order
    // tf: x, y, z, w order
    Eigen::Quaterniond q(ori.w, ori.x, ori.y, ori.z);
    cout << q.toRotationMatrix() << endl;
    Eigen::Matrix3f rot = q.toRotationMatrix().cast<float>();
    Eigen::Quaternionf qFromRot(rot);
    cout << setw(20) << "Original: " << ori.w << ", " << ori.x << ", " << ori.y << ", " << ori.z << endl;
    cout << setw(20) << "Quat: " << q.coeffs().transpose() << endl;
    cout << setw(20) << "Quat from rot. mat: " << qFromRot.coeffs().transpose() << endl;

    // For Translation
    Eigen::Vector3f p0;
    p0 << posit.x, posit.y, posit.z;
    Eigen::Vector3f p1(posit.x, posit.y, posit.z);

    cout << " ----------- " << endl;
    cout << p0.transpose() << endl;
    cout << p1.transpose() << endl;

    cout << " ----------- " << endl;
    cout << setw(18) << "Matrix-like: "<< p1(0, 0) << ", " << p1(1, 0) << ", " << p1(2, 0) << endl;
    cout << setw(18) << "Vector-like: "<< p1(0) << ", " << p1(1) << ", " << p1(2) << endl;
    cout << setw(18) << "std::vector-like: " << p1[0] << ", " << p1[1] << ", " << p1[2] << endl;

    return 0;
}

#include "pose_conversion.h"
#include <iomanip>
#include "utils.hpp"
#include "opencv2/opencv.hpp"

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

void coutRotation(const string mode, float roll, float pitch, float yaw) {
    Matrix3f m;
    m = AngleAxisf(yaw, Vector3f::UnitZ())
        * AngleAxisf(pitch, Vector3f::UnitY())
        * AngleAxisf(roll, Vector3f::UnitX());
    cout << " ---------------- " << endl;
    cout << "Cout rot by " << mode << ": " << endl << m << endl;
    cout << " ---------------- " << endl;
}

int main(){
    vector<nav_msgs::Odometry> odomBuf;
    loadOdom(ABS_FILE_PATH, odomBuf);

    int startIdx = 0;
    nav_msgs::Odometry odomStart = odomBuf[startIdx];
    nav_msgs::Odometry odomEnd = odomBuf[odomBuf.size()-1];
    Eigen::Matrix4f tfStart, tfEnd;
    getTf4x4(odomStart, tfStart);
    getTf4x4(odomEnd, tfEnd);

    Eigen::Matrix4f relTf = tfStart.inverse() * tfEnd;
    Eigen::Matrix3f relRot = relTf.block<3, 3>(0, 0);
    cout << "Original rel. rotation:" << endl << relRot << endl;

    /**< 1. Estimation of rpy using Eigen */
    Eigen::Vector3f ea = relRot.eulerAngles(0, 1, 2);
    cout << "To Euler angles:" << endl;
    cout << "\033[1;31m" << ea.transpose() << "\033[0m" << endl;
    coutRotation("Eigen: ", ea(0), ea(1), ea(2));

    /**< 2. Estimation of rpy using ROS tf */
    Eigen::Quaternionf relQuat(relRot);
    // tf: x, y, z, w order!
    tf::Quaternion qTf(relQuat.x(), relQuat.y(), relQuat.z(), relQuat.w());
    tf::Matrix3x3 relRotTf(qTf);
    cout << "Tf rel. rotation: " << endl;
    for (int i = 0; i < 3; ++i) {
        cout << relRotTf[i][0] << " " << relRotTf[i][1] << " " << relRotTf[i][2] << endl;
    }
    double roll, pitch, yaw;
    relRotTf.getRPY(roll, pitch, yaw);
    cout << "\033[1;32m" << roll << ", " << pitch << ", " << yaw << "\033[0m" << endl;
    coutRotation("Tf", roll, pitch, yaw);

    relRotTf.getEulerYPR(yaw, pitch, roll);
    cout << "\033[1;34m" << roll << ", " << pitch << ", " << yaw << "\033[0m" << endl;

    /**< 3. Estimation of rpy using OpenCV */
    cv::Mat relRotCV = cv::Mat::eye(3, 3, CV_32FC1);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            relRotCV.at<float>(i, j) = relRot(i, j);
        }
    }
    cout << "OpenCV rel. rotation: " << endl;
    for (int i = 0; i < 3; ++i) {
        cout << relRotCV.at<float>(i, 0) << " " << relRotCV.at<float>(i, 1) << " " << relRotCV.at<float>(i, 2) << endl;
    }
    cv::Mat rot_vec;
    cv::Rodrigues(relRotCV,rot_vec);

    double rollCV, pitchCV, yawCV;
    rollCV = rot_vec.at<float>(0);
    pitchCV = rot_vec.at<float>(1);
    yawCV = rot_vec.at<float>(2);

    cout << "\033[1;33m" << rollCV << ", " << pitchCV << ", " << yawCV << "\033[0m" << endl;
    coutRotation("OpenCV", rollCV, pitchCV, yawCV);
    return 0;
}

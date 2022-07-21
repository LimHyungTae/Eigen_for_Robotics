#include "pose_conversion.h"
#include <iomanip>
#include "utils.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace Eigen;
// https://github.com/hku-mars/FAST_LIO/blob/520e69a9b718fad371fa5dba092bd884a2c66440/include/use-ikfom.hpp
vector<double> SO3ToEuler(const Eigen::Matrix3d &orient, string output_unit="rad")
{
    Eigen::Matrix<double, 3, 1> _ang;
    Eigen::Quaterniond q(orient);
    Eigen::Vector4d q_data = {q.x(), q.y(), q.z(), q.w()};
    //scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
    double sqw = q_data[3]*q_data[3];
    double sqx = q_data[0]*q_data[0];
    double sqy = q_data[1]*q_data[1];
    double sqz = q_data[2]*q_data[2];
    double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
    double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

    double conversion = output_unit == "rad"? 1: 57.3;
    if (test > 0.49999*unit) { // singularity at north pole

        _ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
        vector<double> euler_ang  = {_ang[0] * conversion, _ang[1] * conversion, _ang[2] * conversion};
        return euler_ang;
    }
    if (test < -0.49999*unit) { // singularity at south pole
        _ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
        vector<double> euler_ang = {_ang[0] * conversion, _ang[1] * conversion, _ang[2] * conversion};
        return euler_ang;
    }
    // https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
    // float roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
    // float pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
    // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
    _ang <<
         std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
            std::asin (2*test/unit),
            std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
    vector<double> euler_ang = {_ang[0] * conversion, _ang[1] * conversion, _ang[2] * conversion};
    // euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
    return euler_ang;
}

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
    cout << " ===============================" << endl;
    cout << "Original rel. rotation:" << endl << relRot << endl;
    cout << " ===============================" << endl;
    /**< Estimation of rpy using own code */
    vector<double> euler_ang = SO3ToEuler(relRot.cast<double>());
    cout << "\033[1;33mManually: " << euler_ang[0] << ", " << euler_ang[1] << ", " << euler_ang[2] << "\033[0m" << endl;
    cout << " ===============================" << endl;

    /**< 1. Estimation of rpy using Eigen */
    Eigen::Vector3f ea = relRot.eulerAngles(0, 1, 2);
    Eigen::Vector3f ea_zyx = relRot.eulerAngles(2, 1, 0);
    cout << "To Euler angles:" << endl;
    cout << "\033[1;31mEigen :" << endl;
    cout << ea.transpose() << endl;
    cout << ea_zyx.transpose() <<endl;
    coutRotation("Eigen-xyz: ", ea(0), ea(1), ea(2));
    coutRotation("Eigen-zyx: ", ea_zyx(2), ea_zyx(1), ea_zyx(0));
    cout << "\033[0m" << endl;

    /**< 2. Estimation of rpy using ROS tf */
    Eigen::Quaternionf relQuat(relRot);
    // Note: tf: x, y, z, w order!
    tf::Quaternion qTf(relQuat.x(), relQuat.y(), relQuat.z(), relQuat.w());
    tf::Matrix3x3 relRotTf(qTf);
//    cout << "Tf rel. rotation: " << endl;
//    for (int i = 0; i < 3; ++i) {
//        cout << relRotTf[i][0] << " " << relRotTf[i][1] << " " << relRotTf[i][2] << endl;
//    }
    double roll, pitch, yaw;
    relRotTf.getRPY(roll, pitch, yaw);
    cout << "\033[1;32mTF getRPY() => " << roll << ", " << pitch << ", " << yaw << "\033[0m" << endl;
    coutRotation("Tf", roll, pitch, yaw);

    relRotTf.getEulerYPR(yaw, pitch, roll);
    cout << "\033[1;34mTF getEulerRPY() => " << roll << ", " << pitch << ", " << yaw << "\033[0m" << endl;

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

    cout << "\033[1;33mOpenCV => " << rollCV << ", " << pitchCV << ", " << yawCV << "\033[0m" << endl;
    coutRotation("OpenCV", rollCV, pitchCV, yawCV);



    return 0;
}

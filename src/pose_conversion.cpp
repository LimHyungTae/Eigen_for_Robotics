#include <pose_conversion.h>

namespace pose_conversion
{
    //// there already exists, tf::matrixEigenToTF(const Eigen::Matrix3d &e, tf::Matrix3x3 &t) within tf_conversions/tf_eigen.h
    tf::Matrix3x3 eigenRot2RotMat(const Eigen::Matrix3d& eigenRotMat){
        tf::Matrix3x3 rotMat;
        // Eigen::Matrix3d rot = eigenRotMat.cast<double>();
        // rotMat.setValue(rot(0,0), rot(0,1), rot(0,2),
        //                 rot(1,0), rot(1,1), rot(1,2),
        //                 rot(2,0), rot(2,1), rot(2,2));
        tf::matrixEigenToTF(eigenRotMat, rotMat);
        return rotMat;
        ///// actually this func can be replaced by tf::matrixEigenToTF
    }

    //// there already exists, tf::matrixEigenToTF(const Eigen::Matrix3d &e, tf::Matrix3x3 &t) within tf_conversions/tf_eigen.h
     tf::Matrix3x3 getRotMat(const Eigen::Matrix4d& eigenPose){
        tf::Matrix3x3 rotMat;
        tf::matrixEigenToTF(eigenPose.block<3, 3>(0, 0), rotMat);
        // Eigen::Matrix4d pose = eigenPose.cast<double>();
        // rotMat.setValue(pose(0,0), pose(0,1), pose(0,2),
                        // pose(1,0), pose(1,1), pose(1,2),
                        // pose(2,0), pose(2,1), pose(2,2));
        return rotMat;
    }

    //// there already exists, tf::matrixTFToEigen(const tf::Matrix3x3 &t, Eigen::Matrix3d &e) within tf_conversions/tf_eigen.h
    Eigen::Matrix3d rotMat2EigenRot(const tf::Matrix3x3& rotMat){
        Eigen::Matrix3d eigenRotMat(3,3);
        tf::matrixTFToEigen(rotMat, eigenRotMat);
        // eigenRotMat(0,0) = rotMat[0][0];
        // eigenRotMat(0,1) = rotMat[0][1];
        // eigenRotMat(0,2) = rotMat[0][2];
        // eigenRotMat(1,0) = rotMat[1][0];
        // eigenRotMat(1,1) = rotMat[1][1];
        // eigenRotMat(1,2) = rotMat[1][2];
        // eigenRotMat(2,0) = rotMat[2][0];
        // eigenRotMat(2,1) = rotMat[2][1];
        // eigenRotMat(2,2) = rotMat[2][2];

        return eigenRotMat;
        ///// actually this func can be replaced by tf::matrixTFToEigen
    }

    void fillEigenPose(const tf::Matrix3x3& rotMat, Eigen::Matrix4d &eigenPose){
        Eigen::Matrix3d eigenRotMat = pose_conversion::rotMat2EigenRot(rotMat);
        eigenPose.block<3, 3>(0, 0) = eigenRotMat;
        // it seems that this function can be replaced by block slicing of Eigen, as Hyungtae already used in examples
    }

    Eigen::Matrix4d geoPose2eigen(const geometry_msgs::Pose& geoPose)
    {
        Eigen::Matrix4d eigenPose = Eigen::Matrix4d::Identity();
        eigenPose.block<3, 3>(0, 0) = Eigen::Quaterniond(geoPose.orientation.w, geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z).toRotationMatrix();
        //// care the order, Eigen::Quaternion uses wxyz
        // tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        // tf::Matrix3x3 rotMat(q);
        // pose_conversion::fillEigenPose(rotMat, eigenPose);
        eigenPose(0,3) = geoPose.position.x;
        eigenPose(1,3) = geoPose.position.y;
        eigenPose(2,3) = geoPose.position.z;

        return eigenPose;
    }

    geometry_msgs::Pose eigen2geoPose(const Eigen::Matrix4d& eigenPose)
    {
        geometry_msgs::Pose geoPose;

        // tf::Matrix3x3 m = pose_conversion::getRotMat(eigenPose);
        // tf::Quaternion q;
        // m.getRotation(q);
        // geoPose.orientation.x = q.getX();
        // geoPose.orientation.y = q.getY();
        // geoPose.orientation.z = q.getZ();
        // geoPose.orientation.w = q.getW();

        Eigen::Quaterniond q(eigenPose.block<3, 3>(0, 0)); //quat from rot directly
        geoPose.orientation.x = q.x();
        geoPose.orientation.y = q.y();
        geoPose.orientation.z = q.z();
        geoPose.orientation.w = q.w();

        geoPose.position.x = eigenPose(0,3);
        geoPose.position.y = eigenPose(1,3);
        geoPose.position.z = eigenPose(2,3);

        return geoPose;
    }

    Eigen::VectorXd eigen2xyzrpy(const Eigen::Matrix4d& eigenPose)
    {
        Eigen::VectorXd result(6);
        tf::Matrix3x3 rotMat = getRotMat(eigenPose);

        //// only if when deprecating getRotMat func
        // tf::Matrix3x3 rotMat;
        // tf::matrixEigenToTF(eigenPose.block<3, 3>(0, 0), rotMat);

        double r, p, y;
        rotMat.getRPY(r, p, y);

        result[0] = eigenPose(0, 3);
        result[1] = eigenPose(1, 3);
        result[2] = eigenPose(2, 3);
        result[3] = r;
        result[4] = p;
        result[5] = y;

        return result;
    }
    // Note that y in xyz and y in rpy are overlapped each other!
    Eigen::Matrix4d xyzrpy2eigen(const double& x, const double& y, const double& z,
                                 const double& roll, const double& pitch, const double& yaw)
    {
        Eigen::Matrix4d eigenPose = Eigen::Matrix4d::Identity();
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf::Matrix3x3 rotMat(q);

        pose_conversion::fillEigenPose(rotMat, eigenPose);
        //// only if when deprecating fillEigenPose func
        // Eigen::Matrix3d eigenRotMat(3,3);
        // tf::matrixTFToEigen(rotMat, eigenRotMat);
        // eigenPose.block<3, 3>(0, 0) = eigenRotMat;

        eigenPose(0,3) = x;
        eigenPose(1,3) = y;
        eigenPose(2,3) = z;

        return eigenPose;
    }

    Eigen::Matrix4d xyzrpy2eigen(const Eigen::VectorXd& xyzrpy)
    {
        Eigen::Matrix4d eigenPose = Eigen::Matrix4d::Identity();
        tf::Quaternion q;
        q.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
        tf::Matrix3x3 rotMat(q);

        pose_conversion::fillEigenPose(rotMat, eigenPose);
        //// only if when deprecating fillEigenPose func
        // Eigen::Matrix3d eigenRotMat(3,3);
        // tf::matrixTFToEigen(rotMat, eigenRotMat);
        // eigenPose.block<3, 3>(0, 0) = eigenRotMat;

        eigenPose(0,3) = xyzrpy[0];
        eigenPose(1,3) = xyzrpy[1];
        eigenPose(2,3) = xyzrpy[2];

        return eigenPose;
    }

    Eigen::VectorXd geoPose2xyzrpy(const geometry_msgs::Pose& geoPose)
    {
        Eigen::VectorXd xyzrpy(6);
        xyzrpy<<0, 0, 0, 0, 0, 0;

        tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);

        tf::Matrix3x3 rotMat(q);
        double r, p, y;
        rotMat.getRPY(r, p, y);


        xyzrpy[0] = geoPose.position.x;
        xyzrpy[1] = geoPose.position.y;
        xyzrpy[2] = geoPose.position.z;
        xyzrpy[3] = r;
        xyzrpy[4] = p;
        xyzrpy[5] = y;

        return xyzrpy;
    }

    geometry_msgs::Pose xyzrpy2geoPose(const Eigen::VectorXd& xyzrpy)
    {
      geometry_msgs::Pose geoPose;
      tf::Quaternion q;
      q.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
      geoPose.orientation.x = q.getX();
      geoPose.orientation.y = q.getY();
      geoPose.orientation.z = q.getZ();
      geoPose.orientation.w = q.getW();

      geoPose.position.x = xyzrpy[0];
      geoPose.position.y = xyzrpy[1];
      geoPose.position.z = xyzrpy[2];
      return geoPose;
    }
}


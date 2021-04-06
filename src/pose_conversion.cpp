#include <pose_conversion.h>

namespace pose_conversion
{
    Eigen::Matrix3d eigenf2eigend(const Eigen::Matrix3f& eigen3x3)
    {
      Eigen::Matrix3d result;
      for(int y = 0; y < 3; y++)
      {
        for(int x = 0; x < 3; x ++)
        {
          result(y,x) = eigen3x3(y,x);
        }
      }
      return result;
    }
    Eigen::Matrix4d eigenf2eigend(const Eigen::Matrix4f& eigen4x4)
    {
      Eigen::Matrix4d result;
      for(int y = 0; y < 4; y++)
      {
        for(int x = 0; x < 4; x ++)
        {
          result(y,x) = eigen4x4(y,x);
        }
      }
      return result;
    }

    Eigen::Matrix3f eigend2eigenf(const Eigen::Matrix3d& eigen3x3)
    {
      Eigen::Matrix3f result;
      for(int y = 0; y < 3; y++)
      {
        for(int x = 0; x < 3; x ++)
        {
          result(y,x) = eigen3x3(y,x);
        }
      }
      return result;
    }

    Eigen::Matrix4f eigend2eigenf(const Eigen::Matrix4d& eigen4x4)
    {
      Eigen::Matrix4f result;
      for(int y = 0; y < 4; y++)
      {
        for(int x = 0; x < 4; x ++)
        {
          result(y,x) = eigen4x4(y,x);
        }
      }
      return result;
    }

    tf::Matrix3x3 eigenRot2RotMat(const Eigen::Matrix3f& eigenRotMat){
        tf::Matrix3x3 rotMat;
        Eigen::Matrix3d rot = pose_conversion::eigenf2eigend(eigenRotMat);
        rotMat.setValue(rot(0,0), rot(0,1), rot(0,2),
                        rot(1,0), rot(1,1), rot(1,2),
                        rot(2,0), rot(2,1), rot(2,2));
        return rotMat;

    }

     tf::Matrix3x3 getRotMat(const Eigen::Matrix4f& eigenPose){
        tf::Matrix3x3 rotMat;
        Eigen::Matrix4d pose = pose_conversion::eigenf2eigend(eigenPose);
        rotMat.setValue(pose(0,0), pose(0,1), pose(0,2),
                        pose(1,0), pose(1,1), pose(1,2),
                        pose(2,0), pose(2,1), pose(2,2));
        return rotMat;

    }



    Eigen::Matrix3f rotMat2EigenRot(const tf::Matrix3x3& rotMat){
        Eigen::Matrix3f eigenRotMat(3,3);

        eigenRotMat(0,0) = rotMat[0][0];
        eigenRotMat(0,1) = rotMat[0][1];
        eigenRotMat(0,2) = rotMat[0][2];
        eigenRotMat(1,0) = rotMat[1][0];
        eigenRotMat(1,1) = rotMat[1][1];
        eigenRotMat(1,2) = rotMat[1][2];
        eigenRotMat(2,0) = rotMat[2][0];
        eigenRotMat(2,1) = rotMat[2][1];
        eigenRotMat(2,2) = rotMat[2][2];

        return eigenRotMat;
    }

    void fillEigenPose(const tf::Matrix3x3& rotMat, Eigen::Matrix4f &eigenPose){
        Eigen::Matrix3f eigenRotMat = pose_conversion::rotMat2EigenRot(rotMat);
        eigenPose.block<3, 3>(0, 0) = eigenRotMat;
    }

    Eigen::Matrix4f geoPose2eigen(const geometry_msgs::Pose& geoPose)
    {
        Eigen::Matrix4f eigenPose = Eigen::Matrix4f::Identity();
        tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        tf::Matrix3x3 rotMat(q);
        pose_conversion::fillEigenPose(rotMat, eigenPose);

        eigenPose(0,3) = geoPose.position.x;
        eigenPose(1,3) = geoPose.position.y;
        eigenPose(2,3) = geoPose.position.z;

        return eigenPose;
    }

    geometry_msgs::Pose eigen2geoPose(const Eigen::Matrix4f& eigenPose)
    {
        geometry_msgs::Pose geoPose;

        tf::Matrix3x3 m = pose_conversion::getRotMat(eigenPose);

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = eigenPose(0,3);
        geoPose.position.y = eigenPose(1,3);
        geoPose.position.z = eigenPose(2,3);

        return geoPose;
    }

    Eigen::VectorXf eigen2xyzrpy(const Eigen::Matrix4f& eigenPose)
    {
        Eigen::VectorXf result(6);
        tf::Matrix3x3 rotMat = getRotMat(eigenPose);

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
    Eigen::Matrix4f xyzrpy2eigen(const float& x, const float& y, const float& z,
                                 const float& roll, const float& pitch, const float& yaw)
    {
        Eigen::Matrix4f eigenPose = Eigen::Matrix4f::Identity();
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf::Matrix3x3 rotMat(q);
        pose_conversion::fillEigenPose(rotMat, eigenPose);
        eigenPose(0,3) = x;
        eigenPose(1,3) = y;
        eigenPose(2,3) = z;

        return eigenPose;
    }

    Eigen::Matrix4f xyzrpy2eigen(const Eigen::VectorXf& xyzrpy)
    {
        Eigen::Matrix4f eigenPose = Eigen::Matrix4f::Identity();
        tf::Quaternion q;
        q.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
        tf::Matrix3x3 rotMat(q);
        pose_conversion::fillEigenPose(rotMat, eigenPose);
        eigenPose(0,3) = xyzrpy[0];
        eigenPose(1,3) = xyzrpy[1];
        eigenPose(2,3) = xyzrpy[2];

        return eigenPose;
    }

    Eigen::VectorXf geoPose2xyzrpy(const geometry_msgs::Pose& geoPose)
    {
        Eigen::VectorXf xyzrpy(6);
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

    geometry_msgs::Pose xyzrpy2geoPose(const Eigen::VectorXf& xyzrpy)
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


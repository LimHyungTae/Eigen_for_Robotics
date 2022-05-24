//
// Created by shapelim on 22. 5. 18.
//

#ifndef POSE_CONVERSION_UTILS_HPP
#define POSE_CONVERSION_UTILS_HPP

#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/format.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Time.h"
#include "std_msgs/Int32.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <Eigen/Dense>

#define LARGE_ENOUGH 10000

using namespace std;

string ABS_FILE_PATH = "/home/mason/ws/paper_ws/src/Eigen_for_Robotics/materials/odom_poses.txt";

void loadOdom(const string &filePath, vector<nav_msgs::Odometry> &odomBuf){
    std::cout<< "Start loading odom..." << std::endl;
    odomBuf.reserve(LARGE_ENOUGH);
    string ts, x, y, z, qx, qy, qz, qw;
    string line;
    ifstream readFile(filePath);
    if (readFile.fail()) {
        throw std::invalid_argument("Address of ABS_FILE_PATH seems to be wrong. Check include/utils.hpp");
    }

    while(getline(readFile, line))   {
        nav_msgs::Odometry tmp;

        stringstream iss(line);
        getline(iss, ts, ' ');
        getline(iss, x, ' ');
        getline(iss, y, ' ');
        getline(iss, z, ' ');
        getline(iss, qx, ' ');
        getline(iss, qy, ' ');
        getline(iss, qz, ' ');
        getline(iss, qw, '\n');

        ros::Time ros_time(stod(ts));
        tmp.header.stamp = ros_time;
        tmp.pose.pose.position.x = stod(x);
        tmp.pose.pose.position.y = stod(y);
        tmp.pose.pose.position.z = stod(z);
        tmp.pose.pose.orientation.x = stod(qx);
        tmp.pose.pose.orientation.y = stod(qy);
        tmp.pose.pose.orientation.z = stod(qz);
        tmp.pose.pose.orientation.w = stod(qw);

        odomBuf.emplace_back(tmp);
    }

    readFile.close();
    std::cout<< "Load odom. complete" << std::endl;
    std::cout<< "odomBuf size :" << odomBuf.size() << std::endl;
}

#endif //POSE_CONVERSION_UTILS_HPP

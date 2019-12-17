cmake_minimum_required(VERSION 2.8.3)
project(utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(CMAKE_CXX_FLAGS "-std=c++11"
#)

set(CMAKE_CXX_FLAGS "-std=c++11"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
  message_generation
  pcl_conversions
  pcl_ros
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#catkin_package(
#  INCLUDE_DIRS
#  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kalman_filter
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

#find_package(OpenCV)
#include_directories(${OpenCV_INCLUDE_DIRS})

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

set(conversion_src
    src/utils/pose_conversion.h
    src/utils/pose_conversion.cpp
)

add_executable(pose_type_conversion src/main.cpp ${conversion_src})
target_link_libraries(pose_type_conversion ${catkin_LIBRARIES}) # ${OpenCV_LIBRARIES})


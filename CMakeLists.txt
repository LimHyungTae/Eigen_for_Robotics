cmake_minimum_required(VERSION 2.8.3)
project(eigen_for_robotics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


set(CMAKE_CXX_FLAGS "-std=c++11"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
  tf
  tf_conversions
  pcl_conversions
  pcl_ros
)

#find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src
)

catkin_package() #for using with catkin build

add_executable(01_vec_and_quat examples/01_vector_and_quaternion.cpp src/pose_conversion.cpp include/utils.hpp)
add_dependencies(01_vec_and_quat ${catkin_EXPORTED_TARGETS})
target_link_libraries(01_vec_and_quat ${catkin_LIBRARIES})

add_executable(02_block_and_tf examples/02_block_operation_and_tf.cpp src/pose_conversion.cpp include/utils.hpp)
add_dependencies(02_block_and_tf ${catkin_EXPORTED_TARGETS})
target_link_libraries(02_block_and_tf ${catkin_LIBRARIES})

add_executable(03_eigen_w_pcl examples/03_eigen_w_pcl.cpp src/pose_conversion.cpp include/utils.hpp)
add_dependencies(03_eigen_w_pcl ${catkin_EXPORTED_TARGETS})
target_link_libraries(03_eigen_w_pcl ${catkin_LIBRARIES})

add_executable(04_roll_pitch_yaw examples/04_roll_pitch_yaw.cpp src/pose_conversion.cpp include/utils.hpp)
add_dependencies(04_roll_pitch_yaw ${catkin_EXPORTED_TARGETS})
target_link_libraries(04_roll_pitch_yaw ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(05_pose_conversion examples/05_pose_conversion.cpp src/pose_conversion.cpp include/utils.hpp)
add_dependencies(05_pose_conversion ${catkin_EXPORTED_TARGETS})
target_link_libraries(05_pose_conversion ${catkin_LIBRARIES})

add_executable(06_calc_error examples/06_calc_ts_rot_error.cpp src/pose_conversion.cpp include/utils.hpp)
add_dependencies(06_calc_error ${catkin_EXPORTED_TARGETS})
target_link_libraries(06_calc_error ${catkin_LIBRARIES})
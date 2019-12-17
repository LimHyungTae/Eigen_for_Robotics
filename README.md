# Pose Conversion Package for ROS

Original Author: 임형태 (shapelim@kaist.ac.kr)
=======================


## pcl_conversions / pcl_ros에 Eigen이 깔려 있다!

How to run Launch file
-----

roslaunch iuslam_drb runall.launch


Folder type
-----

iu_* : Node for package

unavlib : Useful functions

isam-library : Gaussian Process



Prerequisites
-----
# 1. isam
```cpp
-0.217141 -0.960464  0.174239       7.9
 0.975949 -0.217141 0.0192987       6.5
0.0192987  0.174239  0.984514       4.4
        0         0         0         1
       7.9
       6.5
       4.4
  0.175166
-0.0192999
   1.78972

3.5, 4.2, 1
-0.0717496, -0.0717496, 0.89687, -0.430498
       3.5
       4.2
         1
-0.0682251
  0.191647
  -2.25311

After: 
  0.265133  -0.695141  -0.668194       -4.2
  0.246997   0.718837   -0.64982        2.7
  0.932039 0.00724667   0.362285          3
         0          0          0          1
-4.2, 2.7, 3
0.214482, -0.522355, 0.307537, 0.765875
==================
```
## How to Use

1. rosrun nodelet nodelet standalone velodyne_pointcloud/CloudNodelet
2. rosbag 

IUSLAM FOR DRB (LiDAR ICP matching & RGBD Feature Matching)
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
To install ISAM, you need these prerequisits.
**CMake (version 2.8 or higher), 
CHOLMOD (sparse matrix libraries by Tim Davis, University of Florida), 
Eigen (version 3), 

You have to install  **CMake, CHOLMOD, Eigen**
**CMake**is usally installed, you can check by this command at shell.
```
$ cmake -version
```
If installed, Version of **cmake** will be printed

You can install **CHOLMOD** by this command
```
$ sudo apt-get install libsuitesparse-dev
```

You can install **Eigen** by this command (usually **Eigen** installed when installing **CHOLMOD**)
```
$ sudo apt-get install libeigen3-dev
```

(optional)
```
$ sudo apt-get install libsdl1.2-dev doxygen graphviz
```

you can Download **isam**
```
$ svn co https://svn.csail.mit.edu/isam
```

Copy **Pose2d.cpp** and **Pose3d.cpp** into **isam/isamlib** folder.(if you didn't copy **Pose2d.cpp** and **Pose3d.cpp**,linking error will be occured.)
( **Pose2d.cpp** and **Pose3d.cpp** are in **isam-library** folder(in iuslam4))

move to folder which **isam** installed, and compile it using command.
```
$ make
```

After compile, you can run this command to check compile completed.
```
$ bin/isam -G data/sphere400.txt
```

move library on the system (To **/usr/local/include/isam**)
```
$ make install
```
## How to Use

1. rosrun nodelet nodelet standalone velodyne_pointcloud/CloudNodelet
2. rosbag 

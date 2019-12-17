# Pose Conversion Package for ROS

### ROS로 개발할시 type 변환을 용이하게 해주는 라이브러리

![trinity](./imgs/pose_conversion.png)

Original Author: 임형태 (shapelim@kaist.ac.kr)

원래 [연구실](http://urobot.kaist.ac.kr/) 내에서 공유하는 URL Navigation Library(unavlib)의 일부를 

ROS tf 공부할 겸 다시 정리했습니다. :smirk:

Special thanks to 김형진(hjkim86@kaist.ac.kr) and 송승원(sswan55@kaist.ac.kr)

---

ver 1.0. geometry_msgs/Pose <-> Eigen::Matrix4f <-> xyzrpy(by Eigen::VectorXf) 변환

---

### Dependency libraries

* Eigen (default version of ROS)
* pcl (default version of ROS)
* tf (default version of ROS)
---

### 사용해야하는 이유

ROS 상에서 로봇의 pose들은 [nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)나 [geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html)로 데이터를 제공하는데, 이 메세지를 C++ 상에서 활용하려면 Eigen의(python으로 치면 numpy 같은?) Matrix로 변환해서 사용하는 것이 편하다. 4x4 변환행렬(transformation matrix)로 pose를 포현하게 되면 상대적인 pose나 pose의 좌표계의 변환이 굉장히 용이해진다!

### 사용법 

1. Download this repository 
<pre><code>$ cd /home/$usr_name/catkin_ws/src</code></pre>
<pre><code>$ git clone https://github.com/LimHyungTae/pose_conversion.git</code></pre>

2. Build this ros code as follows.
<pre><code>$ cd /home/$usr_name/catkin_ws</code></pre>
<pre><code>$ catkin_make pose_conversion</code></pre>

Or if you use catkin-tools, then type below line on the command
<pre><code>$ catkin build pose_conversion</code></pre>

3. Rosrun example file
<pre><code>$ rosrun pose_conversion pose_type_conversion </code></pre>




Prerequisites
-----
## 테스트 및 검증

[Online 3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/)를 통해 여러 값들을 대입하여 라이브러리가 잘 작동하는 지 확인해보았다.
#### Eigen
![eigen2sth](./imgs/eigen2sth.png)

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
```
![geoPose2sth](./imgs/geoPose2sth.png)
```cpp
3.5, 4.2, 1
-0.0717496, -0.0717496, 0.89687, -0.430498
       3.5
       4.2
         1
-0.0682251
  0.191647
  -2.25311
```
![xyzrpy2sth](./imgs/xyzrpy2sth.png)
```cpp
After: 
  0.265133  -0.695141  -0.668194       -4.2
  0.246997   0.718837   -0.64982        2.7
  0.932039 0.00724667   0.362285          3
         0          0          0          1
-4.2, 2.7, 3
0.214482, -0.522355, 0.307537, 0.765875
```

### 다른 패키지에서 사용하는 법

![how_to_use](./imgs/how_to_use.png)

1. Download this repository 
<pre><code>$ cd /home/$usr_name/catkin_ws/src</code></pre>
<pre><code>$ git clone https://github.com/LimHyungTae/pose_conversion.git</code></pre>

2. Build this ros code as follows.
<pre><code>$ cd /home/$usr_name/catkin_ws</code></pre>
<pre><code>$ catkin_make pose_conversion</code></pre>

Or if you use catkin-tools, then type below line on the command
<pre><code>$ catkin build pose_conversion</code></pre>

3. Rosrun example file
<pre><code>$ rosrun pose_conversion pose_type_conversion </code></pre>

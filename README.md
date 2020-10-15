## A mobile robot visual SLAM system with enhanced semantics segmentation

 **Authors:**

[Feng Li,](https://github.com/leofer-victor) Wenfeng Chen, Weifeng Xu, Linqing Huang, Dan Li, Shuting Cai, Ming Yang, Xiaoming Xiong, Yuan Liu, Weijun Li

We presented a new mobile robot SLAM system that can works in dynamic indoor environments with robustly and high accuracy. Our system takes color images, depth images, and encoder data as input, and then performs enhanced semantic segmentation by image processing, it outputs the robot pose and establishes a color semantic octree map.

![Pipeline](./Pipeline.png)



## License


This SLAM system is released under a [GPLv3 license](https://github.com/zoeyuchao/DS-SLAM/blob/master/LICENSE).

This SLAM system allows personal and research use only. 

If you use our system in an academic work, please cite their publications as below:

Feng Li, Wenfeng Chen, Weifeng Xu, Linqing Huang, Dan Li, Shuting Cai, Ming
Yang, Xiaoming Xiong, Yuan Liu, Weijun Li. A mobile robot visual SLAM system
with enhanced semantics segmentation. IEEE Access, 2020, 8(1): 25442-25458,([WEB](https://ieeexplore.ieee.org/document/8974270)).

This project is a optimized SLAM system based on the famous ORB-SLAM2, DRE-SLAM, and DS-SLAM. The main frame came from DRE-SLAM. Compared to them, we fused and improved some threads including semantic segmentation thread, sub-OctoMap construction, and sub-OctoMap fusion. You can compare our project with the other three, the articles are as follows:

1. R. Mur-Artal and J. D. Tardos. Orb-slam2: An open-source slam system formonocular, stereo, and rgb-d cameras [J]. IEEE Transactions on Robotics. 2017, 33: 1255--1262.

2. D. Yang, S. Bi, W. Wang, C. Yuan, W. Wang, X. Qi, and Y. Cai. DRE-SLAM: Dynamic RGB-D Encoder SLAM for a Differential-Drive Robot [J]. Remote Sensing, 2019, 11(4): 380

3. C. Yu, Z. Liu, X.-J. Liu, F. Xie, Y. Yang, Q. Wei, and Q. Fei. Ds-slam: A semantic visual slam towards dynamic environments [C]. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2018, 1168--1174.

## Prerequisites

**1. Ubuntu 16.04**

**2. ROS Kinetic**

Follow the instructions in: <http://wiki.ros.org/kinetic/Installation/Ubuntu> 

**3. ROS Pacakge & Turtlebot3-waffle **

For details, please see https://www.ncnynl.com/category/Turtlebot3-waffle

**4. SegNet**

We adopt SegNet to provide pixel-wise semantic segmentation based on caffe in real-time. The root of Download and install instructions can be found at: https://github.com/TimoSaemann/caffe-segnet-cudnn5

**5. Ceres**

Follow the instructions in: <http://www.ceres-solver.org/installation.html>

## Build our SLAM

**1. Clone the repository**

```
cd ~/catkin_ws/src
git clone https://github.com/leofer-victor/tracking_slam.git
```

**2. Build Segnet**

```
cd ../caffe-segnet-cudnn5/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

**3. Build libsegmentation**

```
cd ../libsegmentation/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

**4. Build DBow2**

```
cd ../third_party/DBoW2
mkdir build
cd build
cmake ..
make -j4
```

**5. Build Sophus**

```
cd ../third_party/Sophus
mkdir build
cd build
cmake ..
make -j4
```

**6. Catkin_make**

```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Run

**1. Open a terminal and launch**

```
roslaunch tracking_slam tb3_test.launch
```

**2. Open a terminal and play rosbag**

```
rosbag play *.bag
```

**If you want to run on your own robot, please calibrate the parameter of your sensors, some code needs to be modified.**
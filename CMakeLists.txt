cmake_minimum_required(VERSION 2.8.3)
project(tracking_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #rospy
  std_msgs
  cv_bridge
  pcl_ros
  message_filters
  image_transport
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracking_slam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

#System needs some packages
find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(Ceres REQUIRED )
find_package(Sophus REQUIRED)
find_package(octomap REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	${Sophus_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/third_party/
	${Pangolin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/caffe-segnet-cudnn5/include 
	${PROJECT_SOURCE_DIR}/caffe-segnet-cudnn5/build/include
	/usr/local/include  
	/usr/local/cuda/include
	/usr/include  
	${PROJECT_SOURCE_DIR}/libsegmentation
)

## Declare a C++ library
 add_library(${PROJECT_NAME}_lib
    src/tracking_slam.cpp
    src/config.cpp
 	src/ORBextractor.cpp
 	src/camera.cpp
 	src/tracking.cpp
 	src/local_mapping.cpp
 	src/loop_closing.cpp
 	src/map.cpp
 	src/map_point.cpp
 	src/feature_detector.cpp
 	src/frame.cpp
 	src/keyframe.cpp
 	src/encoder_integration.cpp
 	src/common.cpp
 	src/optimizer.cpp
 	src/ros_puber.cpp
 	src/octomap_fusion.cpp
 	src/sub_octomap.cpp
 	src/sub_octomap_construction.cpp
	#fillhole
	src/fillhole.cpp
	###segment
	src/segment.cpp
	
	)

target_link_libraries(${PROJECT_NAME}_lib
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${PCL_LIBRARIES}
	${CERES_LIBRARIES}
	${Sophus_LIBRARIES}
	${PROJECT_SOURCE_DIR}/third_party/DBoW2/lib/libDBoW2.so
	${OCTOMAP_LIBRARIES}
	#segment
	${PROJECT_SOURCE_DIR}/caffe-segnet-cudnn5/build/lib/libcaffe.so  
	/usr/lib/x86_64-linux-gnu/libglog.so  
	-lboost_system
	${PROJECT_SOURCE_DIR}/libsegmentation/build/libsegmentation.so
)


 add_executable(tracking_slam_node node/tracking_slam_node.cpp)


 target_link_libraries(tracking_slam_node
	${PROJECT_NAME}_lib   
	${catkin_LIBRARIES}
)



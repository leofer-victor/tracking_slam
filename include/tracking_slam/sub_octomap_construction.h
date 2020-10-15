 /*--------------------------------------------------------------------------------------------------
 * A mobile robot visual SLAM system with enhanced semantics segmentation
 * Author(s):
 * Feng Li, Wenfeng Chen, Weifeng Xu, Linqing Huang, Dan Li, Shuting Cai, Ming Yang, Xiaoming Xiong, 
 * Yuan Liu, Weijun Li
 * Created by Feng Li @2019.10.1
 * The project was written by Feng Li, Wenfeng Chen, Weifeng Xu. 
 * We used improved turtlebot3-waffle to do this experiment.
 * -------------------------------------------------------------------------------------------------- 
 * This project is a optimized SLAM system based on the famous ORB-SLAM2, DRE-SLAM, and DS-SLAM. 
 * The main frame came from DRE-SLAM. Compared to them, we fused and improved some threads including 
 * semantic segmentation thread, sub-OctoMap construction, and sub-OctoMap fusion. 
 * --------------------------------------------------------------------------------------------------
 *　@articles:
 * 1. R. Mur-Artal and J. D. Tardos. Orb-slam2: An open-source slam system formonocular, stereo, and 
 * rgb-d cameras [J]. IEEE Transactions on Robotics. 2017, 33: 1255--1262.
 * 2. D. Yang, S. Bi, W. Wang, C. Yuan, W. Wang, X. Qi, and Y. Cai. DRE-SLAM: Dynamic RGB-D Encoder SLAM
 * for a Differential-Drive Robot [J]. Remote Sensing, 2019, 11(4): 380
 * 3. C. Yu, Z. Liu, X.-J. Liu, F. Xie, Y. Yang, Q. Wei, and Q. Fei. Ds-slam: A semantic visual slam 
 * towards dynamic environments [C]. IEEE/RSJ International Conference on Intelligent Robots and Systems 
 * (IROS), 2018, 1168--1174.
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2019 Feng Li <2111704234@mail2.gdut.edu.cn>
 * (ESELAB, School of automation, Guangdong University of Technology)
*/


#ifndef SUB_OCTOMAPPING_H
#define SUB_OCTOMAPPING_H

#include <tracking_slam/keyframe.h>
#include <tracking_slam/config.h>
#include <octomap/octomap.h>
#include <tracking_slam/sub_octomap.h>
#include <tracking_slam/octomap_fusion.h>

#include <thread>


namespace tracking_slam
{
class OctoMapFusion;	

class SubOctoMapConstruction{
public:
	SubOctoMapConstruction( OctoMapFusion* octomap_fusion, Config* cfg );
	
	// insert new frame to
	void insertKeyFrame(KeyFrame* kf);
	void processing();
	bool checkNewFrame();
	KeyFrame* getNewKeyFrame();
	
	// create points.
	//void createPointCloud( KeyFrame* kf, octomap::Pointcloud& point_cloud );
	void createPointCloud( KeyFrame* kf, octomap::Pointcloud& point_cloud , octomap::Pointcloud& point_cloud_color);
	void createCleanCloud( KeyFrame* kf, octomap::Pointcloud& point_cloud );
	void createCleanCloud_color_octree ( KeyFrame* kf, octomap::Pointcloud& point_cloud, octomap::Pointcloud& point_cloud_color );
	
	void insertAllData2OctoMapFusion();
private:
	OctoMapFusion* octomap_fusion_;
	Config* cfg_;
	
	// Thread.
	std::thread* sub_octomap_construction_thread_;
	std::queue<KeyFrame*> kfs_queue_;
	std::mutex mutex_kfs_queue_;
	
	// current sub map.
	SubOctomap* cur_sub_map_; 
	int nkf_passed_;
	
	// requestAllData
	std::mutex mutex_all_data_;
	
}; // class SubOctoMapConstruction
	
    
}//namespace


#endif
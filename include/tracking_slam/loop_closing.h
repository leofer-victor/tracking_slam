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

// This file is part of dre_slam - Dynamic RGB-D Encoder SLAM for Differential-Drive Robot.
//
// Copyright (C) 2019 Dongsheng Yang <ydsf16@buaa.edu.cn>
// (Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

// Tracking_slam is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H

#include <tracking_slam/camera.h>
#include <tracking_slam/config.h>
#include <tracking_slam/map.h>
#include <tracking_slam/optimizer.h>
#include <tracking_slam/vocabulary.h>
#include <tracking_slam/keyframe.h>
#include <tracking_slam/frame.h>
#include <thread>
#include <mutex>
#include <tracking_slam/sub_octomap_construction.h>
#include <tracking_slam/octomap_fusion.h>
#include <tracking_slam/ros_puber.h>


namespace tracking_slam
{
	
class LoopClosing
{
public:
	LoopClosing( SubOctoMapConstruction* sub_octomap_construction, OctoMapFusion* octomap_fusion, RosPuber* ros_puber, Optimizer* optimizer, Map* map, Camera* cam, Vocabulary* voc, Config* cfg);
	void insertKeyFrame(KeyFrame* kf); 
	
private:	
	bool checkLoop( KeyFrame* kf, KeyFrame*& lkf ); 
	bool computeTrans2KF(KeyFrame* rkf, KeyFrame* lkf, Sophus::SE2& delta_pose);
	int matchByBow(KeyFrame* pKF1, KeyFrame* pKF2, vector<MapPoint *> &vpMatches12);
	int matchNnearKfs(KeyFrame* kf,  std::vector<KeyFrame*>& lkfs, vector<MapPoint *> &vpMatches12);
	void processing();
	bool checkNewFrame();
	KeyFrame* getNewKeyFrame();
 
private:
	SubOctoMapConstruction* sub_octomap_construction_;
	OctoMapFusion* octomap_fusion_;
	RosPuber* ros_puber_;
	Optimizer* optimizer_;
	Map* map_;
	Camera* cam_;
	Vocabulary* voc_;
	Config* cfg_;
     
	long int last_loop_kfid_;
	std::thread* th_loop_;
	std::queue<KeyFrame*> kfs_queue_;
	std::mutex mutex_kfs_queue_;
}; //class LoopClosing


}//namespace



#endif
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

#ifndef MAP_POINT_H
#define MAP_POINT_H

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <tracking_slam/config.h>
#include <tracking_slam/keyframe.h>
#include <mutex>

namespace tracking_slam
{
class KeyFrame;    
    
class MapPoint{
public:
    MapPoint(const Eigen::Vector3d& ptw, const cv::Mat& des, Config* cfg_); 
	
    void setPosition( const Eigen::Vector3d& ptw);
    Eigen::Vector3d getPosition();
	
    void addObservation(KeyFrame* kf, int idx);
    cv::Mat& getDescriptor();
	
	// Graph 
 	std::map<KeyFrame*, int> ob_kfs_; 
 	KeyFrame* first_ob_kf_; 
	
	// Used for optimization.
	void cvtEigen2Double();
	void cvtDouble2Eigen();	
	double ptwd_[3]; 
	
	// used for match.
	bool is_selected_;
	int des_dist_; 
private:
    Eigen::Vector3d ptw_; 
    cv::Mat des_; 
    // Thread safe.
	std::mutex mutex_position_;
	std::mutex mutex_des_;

}; // class MapPoint
    
}//namespace



#endif
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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <sophus/se2.h>

#include <tracking_slam/config.h>
#include <tracking_slam/map.h>
#include <tracking_slam/frame.h>

namespace tracking_slam
{
    class Optimizer
    {
    public:
        Optimizer(Map* map, Config* cfg):map_(map), cfg_(cfg){}
    
        // Tracking
        void motionOnlyBA( Frame* frame, KeyFrame* ref_kf, EncoderIntegration& encoder_kf2f_);
        
        // Local Mapping. Sliding window local BA.
        void localBA(std::vector<KeyFrame*>& opt_kfs );
    
        // Loop closure.
        void motionOnlyBA(std::vector<cv::KeyPoint>& kps, std::vector<MapPoint*>& mpts, Sophus::SE2& pose);
        void poseGraphOptimization(  KeyFrame* loop_KF  );
    
    private:
        Config* cfg_;
        Map* map_;
        Eigen::Matrix2d sqrtMatrix2d(const Eigen::Matrix2d& TT);
        
        template <typename T>
        T sqrtMatrix(const T& TT);
        
        Eigen::Matrix3d sqrtMatrix3d(const Eigen::Matrix3d& TT);
    };//class
    
}//namespace


#endif
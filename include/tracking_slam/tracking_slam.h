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

#ifndef TRACKING_SLAM_H
#define TRACKING_SLAM_H

#include <tracking_slam/config.h>
#include <tracking_slam/vocabulary.h>
#include <tracking_slam/camera.h>
#include <tracking_slam/sub_octomap_construction.h>
#include <tracking_slam/octomap_fusion.h>
#include <tracking_slam/tracking.h>
#include <tracking_slam/local_mapping.h>
#include <tracking_slam/loop_closing.h>
#include <tracking_slam/optimizer.h>
#include <tracking_slam/segment.h>

namespace tracking_slam
{

    class TRACKING_SLAM{
    
        public:
             
            TRACKING_SLAM(  ros::NodeHandle& nh, Config* cfg, const std::string& orbvoc_dir, const string &prototxt_file, const string &pascal_caffemodel, const string &pascal_png );

            void addRGBDImage(const cv::Mat& rgb, const cv::Mat& depth, const double& timestamp);
            void addEncoder(const double& enc_l, const double& enc_r, const double& timestamp);
            
            // Save Results
            void saveMapPoints(const std::string& dir);
            void saveKeyFrames(const std::string& dir);
            void saveFrames(const std::string& dir);
            void saveOctoMap(const std::string& dir);

            
        private:
            Config* cfg_;
            Camera* cam_;
            
            Map* map_;
            Tracking* tracking_;
            LocalMapping* local_mapping_;
            LoopClosing* loop_closing_;
            SubOctoMapConstruction* sub_octomap_construction_;
            OctoMapFusion* octomap_fusion_;
            Optimizer* optimizer_;
            RosPuber* ros_puber_;
            
            Vocabulary* orb_voc_;
            
            //Segment
            Segment* segment_;
            const string prototxt_file_, pascal_caffemodel_, pascal_png_;

    }; // class TRACKING_SLAM
 
} ;// namespace tracking_slam

#endif

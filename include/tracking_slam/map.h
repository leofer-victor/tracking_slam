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

#ifndef MAP_H
#define MAP_H

#include <pcl/kdtree/kdtree_flann.h>
#include <tracking_slam/config.h>
#include <tracking_slam/map_point.h>
#include <tracking_slam/keyframe.h>

namespace tracking_slam
{
    class Map
    {
    public:
        Map(Config* cfg);
    
        void addMapPoint(MapPoint* mpt); 
        void addKeyFrame(KeyFrame* kf); 
        void updateKFKDTree(); 
        
        //Get last N keyframes for dynamic pixel culling and local sliding window BA.
        int getLastNKeyFrames( int N, bool keep_last, std::vector<KeyFrame*>& last_n_kfs);
        
        // Get Local MapPoints for RGB-D Encoder Tracking.
        // Keyframes are sorted by distance from near to far
        int getLocalKeyFrames( Frame* frame, double dist_th, double angle_th, std::vector<KeyFrame*>& local_kfs );
        int getLocalMappoints( Frame* frame, double dist_th, double angle_th, int max_n_mpts, std::set< MapPoint* >& local_mpts);
        
        // Get near KFs in dist and angle radius for loop closure. the near kfs must be befor N kfs.
        int getNearKeyFrames( KeyFrame* kf, double dist_th, double angle_th, int npassed_th, std::vector<KeyFrame*>& near_kfs );
        
        // Get n befor and n after kfs around a KF.
        int getNeighbourKeyFrames(KeyFrame* lkf, int n, std::vector< KeyFrame* >& kfs );
        
        // Get map data.
        int getKFsSize();
        void addFrame(Frame* frame);
        std::vector<Frame*> getAllFrames();
        std::set<MapPoint*> getAllMapPoints();
        std::map<long int, KeyFrame*> getAllKeyFrames();

        // For map update.
        std::mutex update_mutex_;
    
    private:
        Config* cfg_;
        std::set<MapPoint*> mpts_; // all map point.
        std::map<long int, KeyFrame*> kfs_; // all keyframes, with their kf id.
        pcl::KdTreeFLANN<pcl::PointXY> KDTree_kfs_; // KD-Tree of the 2D positions of the KFs.
        std::vector<Frame*> frames_; // all frames.
        std::mutex mutex_map_; // For map change.
    };//class
    
}//namespace



#endif
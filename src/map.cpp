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

#include <tracking_slam/map.h>
#include <tracking_slam/common.h>
#include <tracking_slam/keyframe.h>

namespace tracking_slam
{
    Map::Map ( Config* cfg ) :
    cfg_ ( cfg )
{} //  Map

void Map::addKeyFrame ( KeyFrame* kf )
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );
    kfs_[kf->kfid_] = kf;
    updateKFKDTree(); // update KD Tree
} // addKeyFrame

void Map::addMapPoint ( MapPoint* mpt )
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );
    mpts_.insert ( mpt ); //
} // addMapPoint

void Map::updateKFKDTree()
{
    // std::unique_lock<std::mutex> lock (mutex_map_);

    pcl::PointCloud<pcl::PointXY>::Ptr cloud ( new pcl::PointCloud<pcl::PointXY> );
    std::map<long int, KeyFrame*>::iterator it = kfs_.begin();
    for ( ; it != kfs_.end(); ++it ) {
        pcl::PointXY pt;
        Eigen::Vector2d& kfp = it->second ->getSE2Pose().translation();
        pt.x = kfp ( 0 );
        pt.y = kfp ( 1 );
        cloud->push_back ( pt );
    }

    KDTree_kfs_.setInputCloud ( cloud );
} // updateKFKDTree

int Map::getLastNKeyFrames ( int N, bool keep_last, std::vector< KeyFrame* >& last_n_kfs)
{
	std::unique_lock<std::mutex> lock (mutex_map_);
    std::map<long int, KeyFrame*>::reverse_iterator iter = kfs_.rbegin();
    while ( ( iter != kfs_.rend() ) && ( N > 0 ) ) {
        if ( ( !keep_last ) && ( iter == kfs_.rbegin() ) ) {
            ++N;
        } else {
            last_n_kfs.push_back ( iter->second );
        }
        --N;
		++iter;
    }
} // getLastNKeyFrames


int Map::getLocalKeyFrames ( Frame* frame, double dist_th, double angle_th, std::vector< KeyFrame* >& local_kfs )
{
    const Sophus::SE2& Twr = frame->getSE2Pose();

    pcl::PointXY search_point;
    search_point.x = Twr.translation() ( 0 );
    search_point.y = Twr.translation() ( 1 );
    double theta = Twr.so2().log();

    // Search by radiusSearch
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    int n_found =  KDTree_kfs_.radiusSearch ( search_point, dist_th, k_indices, k_sqr_distances );

    // sort by distance, small to large.
    for ( int i = 0; i < n_found; i ++ ) {
        int idx = k_indices.at ( i );
        std::map<long int, KeyFrame*>::iterator it = kfs_.find ( idx );
        double delta_th = normAngle ( it->second->getSE2Pose().so2().log() - theta );

        if ( fabs ( delta_th )  < angle_th ) {
            local_kfs.push_back ( it->second );
        }
    } // for all found kfs.

    return local_kfs.size();
} // getLocalKeyFrames


int Map::getLocalMappoints ( Frame* frame, double dist_th, double angle_th, int max_n_mpts, std::set< MapPoint* >& local_mpts )
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );

    std::vector<KeyFrame*> local_kfs;
    getLocalKeyFrames ( frame, dist_th, angle_th,  local_kfs );

    for ( size_t nk = 0; nk < local_kfs.size(); nk++ ) {
        KeyFrame* kf = local_kfs.at ( nk );
        for ( size_t np = 0; np < kf->mpts_.size(); np ++ ) {
            MapPoint* mpt = kf->mpts_.at ( np );
            if ( mpt != NULL ) {
                local_mpts.insert ( mpt );

                if ( local_mpts.size() >= max_n_mpts ) {
                    return local_mpts.size();
                }
            }
        }//for all mpts in one kf
    } // for all keyframes

    return local_mpts.size();
}


int Map::getNearKeyFrames ( KeyFrame* kf, double dist_th, double angle_th, int npassed_th, std::vector< KeyFrame* >& near_kfs )
{
	const Sophus::SE2& Twr = kf->getSE2Pose();
	
	pcl::PointXY search_point;
	search_point.x = Twr.translation() ( 0 );
	search_point.y = Twr.translation() ( 1 );
	double theta = Twr.so2().log();
	
	// Search by radiusSearch
	std::vector<int> k_indices;
	std::vector<float> k_sqr_distances;
	int n_found =  KDTree_kfs_.radiusSearch ( search_point, dist_th, k_indices, k_sqr_distances );
	
	for ( int i = 0; i < n_found; i ++ ) {
		int idx = k_indices.at ( i );
		std::map<long int, KeyFrame*>::iterator it = kfs_.find ( idx );
		
		if(( kf->kfid_ - it->second->kfid_) < npassed_th)
			continue;
		
		double delta_th = normAngle ( it->second->getSE2Pose().so2().log() - theta );
		
		if ( fabs ( delta_th )  < angle_th ) {
			near_kfs.push_back ( it->second );
		}
	} // for all found kfs.
	
	return near_kfs.size();
} // getNearKeyFrames


int Map::getNeighbourKeyFrames ( KeyFrame* lkf, int n, std::vector< KeyFrame* >& near_kfs )
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );

    int n_before;
    int n_after;
    if ( lkf->kfid_ < n ) {
        n_before = lkf->kfid_;
        n_after = 2 * n - n_before;
    } else {
        n_before = n;
        n_after = n;
    }

    long int start = lkf->kfid_ - n_before;

    std::map<long int, KeyFrame*>::iterator it = kfs_.find ( start );
    if ( it ==  kfs_.end() ) {
        return 0;
    }

    // collect keyframes
    for ( int cnt =0; ( it != kfs_.end() ) && ( cnt != 2*n +1 ); it ++, cnt ++ ) {
        near_kfs.push_back ( it->second );
    } // for

    return near_kfs.size();
} //  getNearKFs

int Map::getKFsSize()
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );
    return kfs_.size();
}

std::set<MapPoint*> Map::getAllMapPoints()
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );
    return mpts_;
}

std::map<long int, KeyFrame*> Map::getAllKeyFrames()
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );
    return kfs_;
}


void Map::addFrame ( Frame *frame )
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );
    frames_.push_back ( frame );
}

std::vector<Frame*> Map::getAllFrames()
{
    std::unique_lock<std::mutex> lock ( mutex_map_ );
    return frames_;
}
}//namespace tracking_slam

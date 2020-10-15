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

#include <tracking_slam/local_mapping.h>
#include <tracking_slam/run_timer.h>

namespace tracking_slam
{
    
	LocalMapping::LocalMapping (  LoopClosing* loop_closing, SubOctoMapConstruction* sub_OctoMap_construction, RosPuber* ros_puber, Optimizer* optimizer, Map* map, Camera* cam, Config* cfg ) :
	loop_closing_ ( loop_closing ), sub_OctoMap_construction_( sub_OctoMap_construction ), ros_puber_(ros_puber), optimizer_ ( optimizer ), map_ ( map ), cam_ ( cam ), cfg_ ( cfg )
{
    // start thread
    th_local_mapping_ = new std::thread ( &LocalMapping::processing, this );
	
} // local mapping

void LocalMapping::insertKeyFrame ( KeyFrame* kf )
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    kfs_queue_.push ( kf );
}

bool LocalMapping::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    return ( !kfs_queue_.empty() );
}

KeyFrame* LocalMapping::getNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    KeyFrame* kf = kfs_queue_.front();
    kfs_queue_.pop();
    return  kf;
}

void LocalMapping::processing()
{
    while ( true ) {

        if ( checkNewFrame() == true ) {
			
            KeyFrame* kf = getNewKeyFrame();

            kf->selectStaticFeatures();

            // add new mpts.
            int N = addNewMpts ( kf );

            // add visual edges.
            addVisualEdges ( kf );

            // Local optimization.
            
            if ( map_->getKFsSize() > 1.5 *  cfg_->sm_lm_window_size_ ) {
				
                std::vector<KeyFrame*> opt_kfs;
				map_->getLastNKeyFrames( cfg_->sm_lm_window_size_, true, opt_kfs );
                RunTimer t; t.start();
                optimizer_->localBA ( opt_kfs );
				t.stop();
				std::cout << "\nKeyFrame " << kf->kfid_ << " Local BA time: " << t.duration() << "\n\n";
            }

            // send keyframe to other thread.
            loop_closing_->insertKeyFrame ( kf );
			
			// send kf for octomap construction.
            sub_OctoMap_construction_->insertKeyFrame ( kf );
			
			// Publish KFs, and pose graph.
			ros_puber_->pubSparseMap( map_ );
        }
        usleep ( 5000 ); // sleep 5 ms
    } // while true.

} // processing


int LocalMapping::addNewMpts ( KeyFrame* kf )
{
    int N = 0;
    for ( size_t i = 0; i < kf->mpts_.size(); i ++ ) {

        // skip dynamic features.
        if ( kf->static_flags_.at ( i ) == false ) {
            continue;
        }

        MapPoint* mpt = kf->mpts_.at ( i );

        // If have a map point / add observation.
        if ( mpt != NULL ) {
            mpt->addObservation ( kf, i );
            continue;
        }

        // If no map point. created one.
        const cv::KeyPoint& kp = kf->kps_.at ( i );

        double& dp = kf->dps_.at ( i );
        if ( ( dp < cfg_->cam_dmin_ ) || ( dp > cfg_->cam_dmax_ ) ) {
            continue;
        }

        Eigen::Vector3d ptc = cam_ ->img2Cam ( Eigen::Vector2d ( kp.pt.x, kp.pt.y ), dp );
        Eigen::Vector3d ptw =kf->getSE3Pose() *  cfg_->Trc_ * ptc;
        MapPoint* nmpt = new MapPoint ( ptw, kf->des_.row ( i ) , cfg_ );
        kf->mpts_.at ( i ) = nmpt;
        nmpt->addObservation ( kf, i );
        map_->addMapPoint ( nmpt );

        N ++;
    }// for all mpts in kf

    return N;
} // addNewMpts


void LocalMapping::addVisualEdges ( KeyFrame* kf )
{
    std::vector<MapPoint*>& mpts = kf->mpts_;
    std::map<KeyFrame*, int> kfs_statistics;

    for ( MapPoint* mpt: mpts ) {
        if ( mpt == NULL ) {
            continue;
        }

        std::map<KeyFrame*, int>::iterator it;
        for ( it = mpt->ob_kfs_.begin(); it != mpt->ob_kfs_.end(); it ++ ) {
            KeyFrame* nkf = it->first;
            kfs_statistics[nkf] ++;
        } // for all ob kfs
    }//for all mpts

    std::map<KeyFrame*, int>::iterator it;
    for ( it = kfs_statistics.begin(); it != kfs_statistics.end(); it ++ ) {
        if ( it->first != kf && it->first != kf->getLastKeyFrameEdge() ) {
            if ( it->second > cfg_->sm_lm_cobv_th_ ) {
                kf->addVisualEdge ( it->first );
            }//
        } // if not last kf
    } // for all  kfs_statistics

} // addVisualEdges
}

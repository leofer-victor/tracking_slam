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

#include <tracking_slam/octomap_fusion.h>
#include <tracking_slam/run_timer.h>
#include <tracking_slam/sub_octomap_construction.h>

namespace tracking_slam
{
OctoMapFusion::OctoMapFusion ( RosPuber* ros_puber, Map* map, Config* cfg ) : ros_puber_(ros_puber), map_ ( map ), cfg_ ( cfg )
{
    th_octomap_fusion_ = new std::thread ( &OctoMapFusion::processing, this );
    full_map_ = new octomap::ColorOcTree ( cfg_->oc_voxel_size_ );
    full_map_->setOccupancyThres ( cfg_->oc_occ_th_ );
    full_map_->setProbHit ( cfg_->oc_prob_hit_ );
    full_map_->setProbMiss ( cfg_->oc_prob_miss_ );

    clamping_max_ = full_map_->getClampingThresMaxLog();
    clamping_min_ = full_map_->getClampingThresMinLog();
    
} // OctomapMerging

void OctoMapFusion::insertSubMap ( SubOctomap* submap )
{
    std::unique_lock<mutex> lock ( mutex_submaps_ );
    submaps_.push_back ( submap );
} // insertSubMap

void OctoMapFusion::insertOneScan2FullMapAndPub ( KeyFrame* kf, octomap::Pointcloud& point_cloud_c , octomap::Pointcloud& point_cloud_color )
{
    // Convert point cloud to world coordinate.
    octomap::Pointcloud point_cloud_w;
    octomap::Pointcloud point_cloud_w_color;
    Sophus::SE3 Twc = kf->getSE3Pose() * cfg_->Trc_;
    // cout<< "point_cloud_c.size = " << point_cloud_c.size() <<endl;
    // cout<< "point_cloud_color.size = " << point_cloud_color.size() <<endl;
    for ( size_t i = 0; i < point_cloud_c.size(); i ++ ) {
        octomap::point3d& pt = point_cloud_c[i];
        octomap::point3d& pt_color = point_cloud_color[i];
        Eigen::Vector3d ptc ( pt.x(), pt.y(), pt.z() );
        Eigen::Vector3d ptc_color ( pt_color.x(), pt_color.y(), pt_color.z() );
        Eigen::Vector3d ptw = Twc * ptc;
		
		// Delete error points. 
		if(ptw[2] > 6.0 || ptw[2] < -2.0)
			continue;
		
        point_cloud_w.push_back ( octomap::point3d ( ptw[0], ptw[1], ptw[2] ) );
        point_cloud_w_color.push_back(octomap::point3d(ptc_color[0], ptc_color[1],ptc_color[2]));

        
    }

    std::unique_lock<mutex> lock ( mutex_full_map_ );
    // update the map.
    Eigen::Vector3d& pt_o = Twc.translation();
    full_map_->insertPointCloud ( point_cloud_w, octomap::point3d ( pt_o[0],pt_o[1],pt_o[2] ), -1, true, true );
    full_map_->updateInnerOccupancy();
	for ( size_t i = 0; i < point_cloud_w_color.size(); i ++ ) {
        octomap::point3d& pt = point_cloud_w[i];
        octomap::point3d& pt_color = point_cloud_w_color[i];
        Eigen::Vector3d ptb ( pt.x(), pt.y(), pt.z() );
        Eigen::Vector3d ptb_color ( pt_color.x(), pt_color.y(), pt_color.z() );
        full_map_->integrateNodeColor(ptb[0], ptb[1], ptb[2], ptb_color[0], ptb_color[1],ptb_color[2]);
    }
	// publish OctoMap.
	ros_puber_->pubOctoMap(full_map_);
} // insertOneScan2FullMap


void OctoMapFusion::fusionAndPub()
{
    std::unique_lock<mutex> lock_sub ( mutex_submaps_ ); 
	std::list<SubOctomap*> tmp_submaps = submaps_;
	
	if ( tmp_submaps.size() == 0 ) {
        return;
    }

    // Create new tree.
    octomap::ColorOcTree* new_tree = new octomap::ColorOcTree ( cfg_->oc_voxel_size_ );
    new_tree->setOccupancyThres ( cfg_->oc_occ_th_ );
    new_tree->setProbHit ( cfg_->oc_prob_hit_ );
    new_tree->setProbMiss ( cfg_->oc_prob_miss_ );

    // insert sub tree to new tree.
	for ( SubOctomap* submap: tmp_submaps ) {
        insertSubMap2NewTree ( submap, new_tree );
    }

    std::unique_lock<mutex> lock_full ( mutex_full_map_ );
    delete full_map_;
    full_map_ = new_tree;

	// publish OctoMap.
	ros_puber_->pubOctoMap(full_map_);
} // fuse the sub octrees.


void OctoMapFusion::insertSubMap2NewTree ( SubOctomap* submap, octomap::ColorOcTree* new_tree )
{
    Sophus::SE3 Twc = submap->kf_base_->getSE3Pose() * cfg_->Trc_;
    transformTree ( submap->sub_octree_, Twc, new_tree );
} // insertSubMap2NewTree


void OctoMapFusion::transformTree ( octomap::ColorOcTree* src_tree, Sophus::SE3& Twc, octomap::ColorOcTree* dst_tree )
{
    for ( octomap::ColorOcTree::leaf_iterator it = src_tree->begin_leafs(); it != src_tree->end_leafs(); ++it ) {
        // src.
        Eigen::Vector3d pt_src ( it.getX(), it.getY(), it.getZ() );
		octomap::ColorOcTreeNode* node_src = src_tree->search ( pt_src ( 0 ), pt_src ( 1 ), pt_src ( 2 ) );
        if ( node_src == NULL ) {
            continue;
        }
        double prob_log = node_src->getLogOdds();

        // dest.
        Eigen::Vector3d pt_dst = Twc * pt_src;

		octomap::ColorOcTreeNode* node_dst = dst_tree->search ( pt_dst ( 0 ), pt_dst ( 1 ), pt_dst ( 2 ) );
        if ( node_dst == NULL ) {
            node_dst = dst_tree->updateNode ( pt_dst ( 0 ), pt_dst ( 1 ), pt_dst ( 2 ), true );
            node_dst->setColor(node_src->getColor());
			prob_log = std::max<double> ( std::min<double> ( prob_log, clamping_max_ ),  clamping_min_ );
		
            node_dst->setLogOdds ( prob_log );
        } else {
            double prob_log_dst = node_dst->getLogOdds();
            double sum_log = prob_log_dst + prob_log;
  
			sum_log = std::max<double> ( std::min<double> ( sum_log, clamping_max_ ),  clamping_min_ );
            node_dst->setLogOdds ( sum_log );
        }
    } // for all leafs.
    dst_tree->updateInnerOccupancy();
} // transformTree


void OctoMapFusion::processing()
{
    static long int kf_num = 0;
    while ( true ) {
        if ( getLoopFlag () == true ) {
			// fusion & publish octomap.
			fusionAndPub();
        }
        usleep ( 5000 ); // sleep 5 ms.
    }
} // processing


bool OctoMapFusion::getLoopFlag ()
{
    std::unique_lock<mutex> lock ( mutex_loop_flag_ );
    if ( loop_flag_ == true ) {
        loop_flag_ = false;
        return true;
    }
    return false;
}

void OctoMapFusion::setLoopFlag ()
{
    std::unique_lock<mutex> lock ( mutex_loop_flag_ );
    loop_flag_ = true;
}

void OctoMapFusion::saveOctoMap ( const string& dir )
{
	std::unique_lock<mutex> lock_full ( mutex_full_map_ );
	// full_map_->writeBinary( dir );
    full_map_->write( dir );
} // saveOctoMap
}

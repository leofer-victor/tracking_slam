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


#include <tracking_slam/sub_octomap.h>

namespace tracking_slam
{
SubOctomap::SubOctomap ( Config* cfg ) :cfg_ ( cfg )
{
    sub_octree_ = new octomap::ColorOcTree ( cfg->oc_voxel_size_ );
    sub_octree_->setOccupancyThres ( cfg->oc_occ_th_ );
    sub_octree_->setProbHit ( cfg->oc_prob_hit_ );
    sub_octree_->setProbMiss ( cfg->oc_prob_miss_ );
} // constructor

void SubOctomap::insertKeyFrame ( KeyFrame* kf, octomap::Pointcloud& point_cloud_c, octomap::Pointcloud& point_cloud_color )
{
    if ( kfs_.size() == 0 ) {
        kf_base_ = kf;
        
    } // the first is set as base.
    
    kfs_.insert ( kf );
	
    Sophus::SE3 Tcw_base = ( kf_base_->getSE3Pose() * cfg_->Trc_ ).inverse();
    Sophus::SE3 Twc = kf->getSE3Pose() * cfg_->Trc_;
    Sophus::SE3 Tbc = Tcw_base * Twc; // base to cur kf.

    // rotate the point cloud to base frame.
    octomap::Pointcloud point_cloud_b;
    octomap::Pointcloud point_cloud_b_color;
    for ( size_t i = 0; i < point_cloud_c.size(); i ++ ) {
        octomap::point3d& pt = point_cloud_c[i];
        octomap::point3d& pt_color = point_cloud_color[i];
        Eigen::Vector3d ptc ( pt.x(), pt.y(), pt.z() );
        Eigen::Vector3d ptc_color ( pt_color.x(), pt_color.y(), pt_color.z() );
        Eigen::Vector3d ptb = Tbc * ptc;
		
		// Delete error points.
		if(ptb[2] > 6.0 || ptb[2] < -2.0)
			continue;
        point_cloud_b.push_back ( octomap::point3d ( ptb[0], ptb[1], ptb[2] ) );
        point_cloud_b_color.push_back ( octomap::point3d ( ptc_color[0], ptc_color[1],ptc_color[2] ) );
    }

    // update the sub octree.
    Eigen::Vector3d& pt_o = Tbc.translation();
    sub_octree_->insertPointCloud ( point_cloud_b, octomap::point3d ( pt_o[0],pt_o[1],pt_o[2] ), -1, true, true );
    // add color
    for ( size_t i = 0; i < point_cloud_b_color.size(); i ++ ) {
        octomap::point3d& pt = point_cloud_b[i];
        octomap::point3d& pt_color = point_cloud_b_color[i];
        Eigen::Vector3d ptb ( pt.x(), pt.y(), pt.z() );
        Eigen::Vector3d ptb_color ( pt_color.x(), pt_color.y(), pt_color.z() );
        sub_octree_->integrateNodeColor(ptb[0], ptb[1], ptb[2], ptb_color[0], ptb_color[1],ptb_color[2]);
    }
    sub_octree_->updateInnerOccupancy();
} // insertKeyFrame


bool SubOctomap::isContainKeyframe ( KeyFrame* kf )
{
    std::set<KeyFrame*>::iterator iter = kfs_.find ( kf );
    if ( iter == kfs_.end() ) {
        return false;
    }
    return true;
} // isContainKeyframe
}

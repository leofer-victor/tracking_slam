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


#ifndef CONFIG_H
#define CONFIG_H

#include <opencv2/opencv.hpp>
#include <sophus/se3.h>

namespace tracking_slam{
class Config{
public:
	Config(const std::string& cfg_dir);
	
public:
	/**** Camera ****/
	std::string cam_rgb_topic_; 
	std::string cam_depth_topic_;
	
	double cam_fx_;
	double cam_fy_;
	double cam_cx_;
	double cam_cy_;
	double cam_k1_;
	double cam_k2_;
	double cam_p1_;
	double cam_p2_;
	double cam_k3_;
	int cam_height_;
	int cam_width_;	
	double cam_depth_factor_; 	// Depth scale factor.
	double cam_dmax_;			// Max depth value to be used.
	double cam_dmin_;			// Min depth value to be used.
	double cam_fps_;			// Camera FPs.

	/**** Robot intrinsic and extrinsic ****/
	std::string encoder_topic_;
	
	double odom_kl_; 	// left wheel factor
	double odom_kr_; 	// right wheel factor
	double odom_b_; 	// wheel space
	double odom_K_; 	// Noise factor.
	Sophus::SE3 Trc_; 	// Extrinsic parameter. Translation from the camera to the robot.


	/**** RGB-D Encoder Tracking ****/
	/** ORB feature **/
	int ret_ft_n_features_; // Number of ORB features per frame.
	float ret_ft_scale_factor_ =  1.2;
	int ret_ft_n_levels_ = 8;
	int ret_ft_init_th_ = 20;
	int ret_ft_min_th_ = 7;

	/** Tracking **/
	double ret_tk_dist_th_; // Local map search radius.
	double ret_tk_angle_th_; // Local map search angle.
	double ret_tk_max_local_mpts_ = 4000; // Max mappoints in the local map.
	double ret_tk_sigma_p_ = 0.03;  	  // (m)
	double ret_tk_db_; // The base threshold for erroneous match discard.
	double ret_tk_kd_; // The scale factor of the threshold for erroneous match discard.

	/** Keyframe decision **/
	// Condation 1. 
	double ret_kd_fps_factor_; 
	// Condation 2: 
	double ret_kd_dist_th_; 	// Max distance threshold.
	double ret_kd_angle_th_; 	// Max angle threshold.



	/**** Dynamic Pixels Culling ****/
	double dpc_ob_scale_up_factor_ = 1.2; 	// Scale-up factor of the bounding boxes
	std::vector<std::string> dpc_predef_dyn_obj_names_ = {"person", "bicycle", "car", "motorbike", "bird", "cat", "dog", "horse", "cow"}; 	// Names of the predefined dynamic objects.
	int dpc_n_near_kfs_; 				// Only using nearest dpc_max_nkf_passed_ KFs that are in the local map for dynamic cluster decision.
	int dpc_npts_per_cluster_; 			// Number of points per cluster.
	int dpc_n_sel_pts_per_cluster_; 	// Number of points per cluster to be selected for dynamic cluster decision.
	int dpc_search_square_size_;		// Size of the square search area.

	/**** Extend by depth parameters ****/
	double depth_thread_;
	double depth_thread_second_;
	int area_thread_;
	int blur_kernal_size_;
	int hight_limit_;
	/**** Sparse Mapping ****/
	/** Local Mapping **/
	int sm_lm_cobv_th_ = 20; 		// sp_lm_cobv_th_  a visual edge.
	int sm_lm_lba_niter = 50; 		// sp_lm_lba_niter iter.
	int sm_lm_window_size_; 		// local BA window size: sp_lm_window_size_ KFs.
	
	/** Loop Closure **/
	double sm_lc_search_dist_ = 8.0;
	double sm_lc_search_angle_ = 2.0;
	double sm_lc_loop_edge_weight_ = 90;
	double sm_lc_cov_edge_weight_ = 30;
	double sm_lc_encoder_edge_weight_ = 60;
	int sm_lc_pgop_niter_ = 120; 	// sp_lc_pgop_niter_ iter.


	/**** OctoMap Construction ****/
	double oc_voxel_size_; 	// Voxel size of the OctoMap (m).
	double oc_submap_size_; // Sub-OctoMap size (KFs)

	/** OctoMap parameters **/
	double oc_occ_th_;
	double oc_prob_hit_;
	double oc_prob_miss_; 
	
}; // class Config
} //namespace tracking_slam

#endif //CONFIG_H

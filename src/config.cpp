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


#include <tracking_slam/config.h>

namespace tracking_slam{
Config::Config ( const std::string& cfg_dir)
{

	cv::FileStorage fs ( cfg_dir, cv::FileStorage::READ );
	
	/**** Camera ****/
	fs["cam_rgb_topic_"] >> cam_rgb_topic_;
	fs["cam_depth_topic_"] >> cam_depth_topic_;
	
	fs["cam_fx_"] >> cam_fx_;
	fs["cam_fy_"] >> cam_fy_;
	fs["cam_cx_"] >> cam_cx_;
	fs["cam_cy_"] >> cam_cy_;
	fs["cam_k1_"] >> cam_k1_;
	fs["cam_k2_"] >> cam_k2_;
	fs["cam_p1_"] >> cam_p1_;
	fs["cam_p2_"] >> cam_p2_;
	fs["cam_k3_"] >> cam_k3_;
	fs["cam_height_"] >> cam_height_;
	fs["cam_width_"] >> cam_width_;	
	fs["cam_depth_factor_"] >> cam_depth_factor_; 	// Depth scale factor.
	fs["cam_dmax_"] >> cam_dmax_;	// Max depth value to be used.
	fs["cam_dmin_"] >> cam_dmin_;	// Min depth value to be used.
	fs["cam_fps_"] >> cam_fps_;		// Camera FPs.
	
	/**** Robot intrinsic and extrinsic ****/
	fs["encoder_topic_"] >> encoder_topic_;
	
	fs["odom_kl_"] >> odom_kl_; // left wheel factor
	fs["odom_kr_"] >> odom_kr_; // right wheel factor
	fs["odom_b_"] >> odom_b_; 	// wheel space
	fs["odom_K_"] >> odom_K_; 	// Noise factor.
	
	/** robot extrinsic **/
	cv::Mat cvTrc;
	fs["Trc_"] >> cvTrc;
	Eigen::Matrix3d R;
	R << cvTrc.at<double> ( 0, 0 ), cvTrc.at<double> ( 0, 1 ), cvTrc.at<double> ( 0, 2 ),
	cvTrc.at<double> ( 1, 0 ), cvTrc.at<double> ( 1, 1 ), cvTrc.at<double> ( 1, 2 ),
	cvTrc.at<double> ( 2, 0 ), cvTrc.at<double> ( 2, 1 ), cvTrc.at<double> ( 2, 2 );
	Eigen::Vector3d t;
	t << cvTrc.at<double> ( 0, 3 ), cvTrc.at<double> ( 1, 3 ), cvTrc.at<double> ( 2, 3 );
	Trc_ = Sophus::SE3 ( R, t );
	
	
	/**** RGB-D Encoder Tracking ****/
	/** ORB feature **/
	fs["ret_ft_n_features_"] >>  ret_ft_n_features_; // Number of ORB features per frame.

	/** Tracking **/
	fs["ret_tk_dist_th_"] >> ret_tk_dist_th_; // Local map search radius.
	fs["ret_tk_angle_th_"] >> ret_tk_angle_th_; // Local map search angle.
	fs["ret_tk_db_"] >> ret_tk_db_; // The base threshold for erroneous match discard.
	fs["ret_tk_kd_"] >> ret_tk_kd_; // The scale factor of the threshold for erroneous match discard.
	
	/** Keyframe decision **/
	// Condation 1. 
	fs["ret_kd_fps_factor_"] >> ret_kd_fps_factor_; 
	// Condation 2: 
	fs["ret_kd_dist_th_"] >> ret_kd_dist_th_;   // Max distance 
	fs["ret_kd_angle_th_"] >> ret_kd_angle_th_; // Max angle 
	
	
	
	/**** Dynamic Pixels Culling ****/
	fs["dpc_n_near_kfs_"] >> dpc_n_near_kfs_;
	fs["dpc_npts_per_cluster_"] >> dpc_npts_per_cluster_; // Number of points per cluster.
	fs["dpc_n_sel_pts_per_cluster_"] >> dpc_n_sel_pts_per_cluster_; // Number of points per cluster to be selected for dynamic cluster decision.
	fs["dpc_search_square_size_"] >> dpc_search_square_size_;
	
	/**** Extend by depth parameters ****/
	fs["depth_thread_"] >> depth_thread_;
	fs["depth_thread_second_"] >> depth_thread_second_;
	fs["area_thread_"] >> area_thread_;
	fs["blur_kernal_size_"] >> blur_kernal_size_;
	fs["hight_limit"] >> hight_limit_;
	/**** Sparse Mapping ****/
	/** Local Mapping **/
	fs["sm_lm_window_size_"] >> sm_lm_window_size_; // local BA window size: sp_lm_window_size_ KFs.
	
	/**** OctoMap Construction ****/
	fs["oc_voxel_size_"] >> oc_voxel_size_;   // Voxel size of the OctoMap (m).
	fs["oc_submap_size_"] >> oc_submap_size_; // Sub-OctoMap size (KFs)
	
	/** OctoMap parameters **/
	fs["oc_occ_th_"] >> oc_occ_th_;
	fs["oc_prob_hit_"] >> oc_prob_hit_;
	fs["oc_prob_miss_"] >> oc_prob_miss_;

	
	fs.release();
} // Config

} //namespace tracking_slam

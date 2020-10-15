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


#ifndef ROS_PUBER_H
#define ROS_PUBER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <tracking_slam/map.h>

namespace tracking_slam
{
    class RosPuber
    {
    public:
        RosPuber( ros::NodeHandle& nh );
	
        void pubCurrentFrame( Frame* frame );
        void pubDynamicPixelCullingResults( KeyFrame* kf);
        void pubSparseMap( Map* map);
        void pubOctoMap( octomap::ColorOcTree* octree );
        void pubSegnetImg(cv::Mat &img);
    
    private:
        ros::NodeHandle nh_;
	
        // Info of the current frame.
        ros::Publisher puber_robot_pose_; 
        image_transport::Publisher puber_img_match_;
        
        // Dynamic pixel detection results.
        image_transport::Publisher puber_dpc_img_objects_;
        image_transport::Publisher puber_dpc_img_clusters_;
        image_transport::Publisher puber_dpc_img_mask_;
        
        // KFs and pose graph. Sparse Map
        ros::Publisher puber_mappoints_;
        ros::Publisher puber_kfs_puber_; 
        ros::Publisher puber_encoder_graph_;        
        ros::Publisher puber_loop_graph_;        
        ros::Publisher puber_visual_graph_;
        
        
        // OctoMap.
        ros::Publisher puber_octomap_;

        
    }; // RosPuber
    
}//namespace


#endif
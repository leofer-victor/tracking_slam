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

#ifndef TRACKING_H
#define TRACKING_H

#include <mutex>
#include <thread>

#include <tracking_slam/run_timer.h>
#include <tracking_slam/optimizer.h>
#include <tracking_slam/map.h>
#include <tracking_slam/config.h>
#include <tracking_slam/camera.h>
#include <tracking_slam/frame.h>
#include <tracking_slam/encoder.h>
#include <tracking_slam/keyframe.h>
#include <tracking_slam/encoder_integration.h>
#include <tracking_slam/vocabulary.h>
#include <tracking_slam/ros_puber.h>
#include <tracking_slam/segment.h>

namespace tracking_slam
{
    class Tracking
    {
    public:
        Tracking(Segment* segment, RosPuber* ros_puber, Optimizer* optimizer, Map* map,  Camera* cam, Vocabulary* voc, Config* cfg);

        void addRGBD(const cv::Mat& rgb, const cv::Mat& depth, const double& timestamp);
        void addEncoder(const double& enc_l, const double& enc_r, const double& timestamp);
        double sum_time;
    
    private:
        void RGBDThread(); 
        void RGBDProcessing(); 

        bool initialization();
        int matchByProjection(const std::set<MapPoint*>& mpts, Frame* frame);

        bool checkNewFrame();
        Frame* getNewFrame();

        bool newKeyFrame();
        int discardOutliers(double th);
        void drawMatchedFeatures( cv::Mat& img_sum );
        
        double getSemiMajorAxisLength( const Eigen::Matrix2d& cov );
        

        bool inited_;

        //Segment
        Segment* segment_;
        
        RosPuber* ros_puber_;
        Optimizer* optimizer_;
        Map* map_;
        Camera* cam_;
        Vocabulary* voc_;
        Config* cfg_;
        FeatureDetector* feature_detector_;

        

        Frame* cur_frame_; // current frame

        std::mutex mutex_input_frames_;
        std::queue<Frame*> input_frames_; 
        std::thread* rgbd_thread_;

        long int last_frame_id_;

        // reference keyframe
        KeyFrame* ref_kf_;

        // raw encoder data.
        std::vector<Encoder> encoders_f2f_;

        // integrated encoder between the reference KF and the current frame.
        EncoderIntegration encoder_kf2f_;
    };//class
    
}//namespace


#endif
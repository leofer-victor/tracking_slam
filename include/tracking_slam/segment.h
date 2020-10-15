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

#ifndef SEGMENT_H
#define SEGMENT_H

#include <libsegmentation.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>
#include <condition_variable>
#include <tracking_slam/camera.h>
#include <tracking_slam/keyframe.h> 
#include <tracking_slam/local_mapping.h>
#include <tracking_slam/ros_puber.h>
#include <queue>
#include <thread>

#include <fstream>
#include <tracking_slam/fillhole.h>
#include <cmath>


namespace tracking_slam
{

class Tracking;
// class KeyFrame;
// class LocalMapping;
// class RosPuber;
class Segment
{

public:
    Segment(const string &pascal_prototxt, const string &pascal_caffemodel, const string &pascal_png, LocalMapping* local_mapping,RosPuber* ros_puber, Map* map, Camera* cam, Config* cfg );


    void SetTracker(Tracking* pTracker);
    void Predict();
    int no = -1;
    
    string ext = ".png";
    int conbase = 64, jinzhi=4;
    int labeldata[20]={32,8,40,2,34,10,42,16,48,24,56,18,50,26,58,4,36,12,44,6};
    // double depth_th = 0.008;
    cv::Mat label_colours;
    Classifier* classifier;
    bool isNewImgArrived();
    bool CheckFinish();
    void RequestFinish();
    void Initialize(const cv::Mat& img);
    cv::Mat mImg;
    cv::Mat mImgTemp;
    cv::Mat mImgSegment_color;
    cv::Mat mImgSegment_color_final;
    cv::Mat mImgSegment;
    cv::Mat mImgSegmentLatest;
    Tracking* mpTracker;
    std::mutex mMutexGetNewImg;
    std::mutex mMutexFinish;
    bool mbFinishRequested;
    void ProduceImgSegment();
    std::mutex mMutexNewImgSegment;
    std::condition_variable mbcvNewImgSegment;
    bool mbNewImgFlag;
    int mSkipIndex;
    double mSegmentTime;
    int imgIndex;
    // Paremeters for caffe
    string model_file;
    string trained_file;
    string LUT_file;
    void insertKeyFrame(KeyFrame* kf);
	bool checkNewFrame();
	KeyFrame* getNewKeyFrame();
    std::queue<KeyFrame*> passed_kfs_queue_;
    std::thread* mptSegment;
    void computePointCloud ( Camera* cam, const cv::Mat& depth, vector< Eigen::Vector2d >& pt2d, vector< Eigen::Vector3d >& pt3d );


    void drawResult( const cv::Mat& depth, cv::Mat& label, cv::Mat& color_mask);
    void drawDepth(const cv::Mat& depth, cv::Mat& blue_depth);
    void drawDepth1(const cv::Mat& depth, cv::Mat& blue_depth);
    // Paremeters for caffe


    ////for write image
    int index = 0;

    double sum_process;

private:
    

    LocalMapping* local_mapping_;
	RosPuber* ros_puber_;
	Map* map_;
	Camera* cam_;
	Config* cfg_;
    // thread.
	std::mutex mutex_kfs_queue_;
	std::queue<KeyFrame*> kfs_queue_; 



};

}

#endif

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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <tracking_slam/frame.h>
#include <tracking_slam/map_point.h>
#include <tracking_slam/encoder_integration.h>
#include <tracking_slam/vocabulary.h>
#include <mutex>

namespace tracking_slam
{
class MapPoint;
class Frame;

class KeyFrame
{
public:
    KeyFrame( Frame* frame , Vocabulary* voc);

    // pose
    void cvtEigen2Double();
    void cvtDouble2Eigen();
    void setPose(const Sophus::SE2& Twr);
    Sophus::SE2 getSE2Pose();
    Sophus::SE3 getSE3Pose();

    // Pose graph
    void addLastKeyFrameEdge(KeyFrame* kf);
    void addVisualEdge(KeyFrame* kf);
    
    std::set<KeyFrame*> getObKFs();
    void setObKFs(const std::set<KeyFrame*>& ob_kfs);

    void addLoopEdge(KeyFrame* kf, const Sophus::SE2& T);
    KeyFrame* getLastKeyFrameEdge();
    std::set<KeyFrame*> getVisualEdge();
    int getLoopEdge(std::vector<KeyFrame*>& loop_kfs, std::vector<Sophus::SE2>& Ts);

    // Optimization
    void calculateRelativePosition();
    void reCalculateMpts();
    void reCalculateSingleMpts();
    void freeMemory();
    void freeDepth();
    
    // select static features.
    void selectStaticFeatures();
    int colormap_[21][3] = 
    {{255, 255, 255},   //背景——白
    {255, 255, 255},   //飞机——酒红
    {0, 128, 0},   //自行车——墨绿
    {255, 255, 255},  //小鸟——暗黄
    {255, 255, 255},   //船——蓝紫
    {255, 255, 255},   //瓶子——紫色
    {255, 255, 255},   //公交车——暗青
    {255, 255, 255},   //汽车——灰色
    {255, 255, 255},   //猫——棕色
    {192, 0, 0},   //椅子——红色
    {255, 255, 255},   //牛——青绿
    {255, 255, 255},  //狗——褐色
    {255, 255, 255},  //马——深紫
    {255, 255, 255},   //摩托车——深粉红
    {255, 255, 255},  //人——青蓝色
    {192, 128, 128},   //棉羊——粉红
    //{0, 64, 0},   //沙发——深绿
    {255, 255, 255}, //沙发——深绿
    {255, 255, 255},   //桌子——巧克力色
    {255, 255, 255},  //盆栽——绿色
    {255, 255, 255},   //火车——亮绿色
    {0, 64, 128}    //电视机——深蓝
    };
    
    
public:
    // Number of all keyframes.
    static long int N_KFS;

    // Read only data.
    long int kfid_;
    long int fid_;
    double timestamp_;

    // The static KeyPoints and descriptors.
    std::vector<cv::KeyPoint> kps_;
    std::vector<bool> static_flags_;
    cv::Mat des_;
    
    Vocabulary* voc_;
    DBoW2::BowVector bow_vec_;
    DBoW2::FeatureVector feat_vec_;
    
    int n_features_;
    std::vector<double> dps_;
    std::vector<MapPoint*> mpts_; 
    
    std::vector<Eigen::Vector3d> pts_r_; 

    Camera* cam_;
    Config* cfg_;
    cv::Mat rgb_;
    cv::Mat depth_;
    
    // segmentation used.
    cv::Mat image_mask_;
    cv::Mat image_objects_;
    cv::Mat image_source_clusters_;
    cv::Mat image_static_clusters_;

    //leo
    cv::Mat image_label_;
    cv::Mat blue_depth_;
    std::vector<double> sum_time_;

    Sophus::SE2 Trr_; 
    Eigen::Matrix3d covrr_; 

    double Twrd_[3];
    
private:
    Sophus::SE2 Twr2_; // robot pose
    Sophus::SE3 Twr3_; // robot pose

    // Pose graph connections.
    KeyFrame* last_kf_; // The last frame.
    std::set<KeyFrame*> ob_kfs_; // Cov keyframes.
    std::vector<KeyFrame*> loop_kfs_; // Loop Frames.
    std::vector<Sophus::SE2> loop_delta_pose_;

    std::mutex mutex_pose_;
    std::mutex mutex_graph_;
    std::mutex mute_mpts_;

}; // class KeyFrame

}//namespace


#endif
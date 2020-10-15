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

#include "stdio.h"  
#include <iostream>  
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <list>
#include <map>
#include <stack>
#include <string>
#include <opencv2/core/core.hpp>      
#include <opencv2/highgui/highgui.hpp>    
#include "opencv2/imgproc/imgproc.hpp"  
#include <Eigen/Dense> 
#include <Eigen/Core>  
#pragma comment(lib,"opencv_core2410d.lib")                    
#pragma comment(lib,"opencv_highgui2410d.lib")                    
#pragma comment(lib,"opencv_imgproc2410d.lib")       
        
using namespace std;    
using namespace cv;   

void fillHole(const Mat srcBw, Mat &dstBw) ;
bool ascendSort(vector<cv::Point> a, vector<cv::Point> b) ;
bool descendSort(vector<cv::Point> a, vector<cv::Point> b) ;
void DelSmallAera(const Mat srcBw, Mat &dstBw, int aera) ;
void extend_by_depth(const cv::Mat& _depthImg, cv::Mat& _lableImg, const double depth_thread, const int hight_limit);
void extend_if_notedge(const cv::Mat& color_edge_threshold, const cv::Mat& _depthImg, cv::Mat& _lableImg, const double depth_thread, const int hight_limit);
void extend_by_depth_ROI(const cv::Mat& _depthImg, cv::Mat& _lableImg, const double depth_thread, const int hight_limit);

void extend_by_kmeans(const vector< vector< Eigen::Vector2d > >& clusters_2d, cv::Mat& _lableImg, const int cluster_min);
bool check_label(const int label);
void segmentPointCloudByKmeans ( const vector< Eigen::Vector2d >& pts2d, const vector< Eigen::Vector3d >& pts3d, 
                const int n_clusters, vector< vector< Eigen::Vector2d > >& clusters_2d, vector< vector< Eigen::Vector3d > >& clusters_3d );
void ClustersImage ( cv::Mat& io_img, const int width, const int height, const vector< vector< Eigen::Vector2d > >& clusters_2d, 
                const std::vector<uint8_t>& colors );
static std::vector<uint8_t> cluster_colors_ = {213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,165,0,200,
                83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,136,127,
                255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,81,181,
                103,58,183,171,71,188,236,64,122,239,83,80, 213,0,0,197,17,98,170,0,255,98,0,234,48,79,254,41,98,255,0,145,234,0,184,212,0,191,
                165,0,200,83,100,221,23,174,234,0,255,214,0,255,171,0,255,109,0,221,44,0,62,39,35,33,33,33,38,50,56,144,164,174,224,224,224,161,
                136,127,255,112,67,255,152,0,255,193,7,255,235,59,192,202,51,139,195,74,67,160,71,0,150,136,0,172,193,3,169,244,100,181,246,63,
                81,181,103,58,183,171,71,188,236,64,122,239,83,80};
int find_label_in_cluster(const std::vector<Eigen::Vector2d>& cluster_2d, cv::Mat& _lableImg);
void generate_kmeans_img(const vector< vector< Eigen::Vector2d > >& clusters_2d, cv::Mat& kmeans_img, const int cluster_min);
void extend_by_depth_and_kmeans(const cv::Mat& _depthImg, const cv::Mat& kmeans_img, cv::Mat& _lableImg, const double depth_thread, const int hight_limit);


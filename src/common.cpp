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

#include <tracking_slam/common.h>

namespace tracking_slam
{

double normAngle ( double angle )
{
    static double Two_PI = 2.0 * M_PI;
    
    if ( angle >= M_PI ) {
        angle -= Two_PI;
    }
    if ( angle < -M_PI ) {
        angle += Two_PI;
    }
    return angle;
}


int DescriptorDistance ( const cv::Mat &a, const cv::Mat &b )
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for ( int i=0; i<8; i++, pa++, pb++ ) {
        unsigned  int v = *pa ^ *pb;
        v = v - ( ( v >> 1 ) & 0x55555555 );
        v = ( v & 0x33333333 ) + ( ( v >> 2 ) & 0x33333333 );
        dist += ( ( ( v + ( v >> 4 ) ) & 0xF0F0F0F ) * 0x1010101 ) >> 24;
    }
    return dist;
}

std::vector<cv::Mat> CvMat2DescriptorVector ( const cv::Mat &Descriptors )
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve ( Descriptors.rows );
    for ( int j=0; j<Descriptors.rows; j++ ) {
        vDesc.push_back ( Descriptors.row ( j ) );
    }

    return vDesc;
}

Eigen::Matrix4d AngleAxisTrans2EigenT ( cv::Vec3d rvec, cv::Vec3d tvec )
{
    /* convert rotation angle to R matrix */
    cv::Mat R;
    cv::Rodrigues ( rvec, R );

    /* convert to eigen style */
    Eigen::Matrix4d T;
    T<<
     R.at<double> ( 0, 0 ), R.at<double> ( 0, 1 ), R.at<double> ( 0, 2 ), tvec[0],
          R.at<double> ( 1, 0 ), R.at<double> ( 1, 1 ), R.at<double> ( 1, 2 ), tvec[1],
          R.at<double> ( 2, 0 ), R.at<double> ( 2, 1 ), R.at<double> ( 2, 2 ), tvec[2],
          0.,0.,0.,1.;
    return T;
}

Sophus::SE2 EigenT2Pose2d ( Eigen::Matrix4d& T )
{
    double theta = atan2 ( T ( 1,0 ), T ( 0,0 ) );
    double x = T ( 0, 3 );
    double y = T ( 1, 3 );
    return Sophus::SE2 ( theta, Eigen::Vector2d ( x, y ) );
}

}//namespace
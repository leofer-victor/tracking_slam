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


#include <tracking_slam/segment.h>
#include <tracking_slam/run_timer.h>

#define SKIP_NUMBER 1

using namespace std;
namespace tracking_slam
{
class Frame;
Segment::Segment(const string &pascal_prototxt, const string &pascal_caffemodel, const string &pascal_png, LocalMapping* local_mapping,RosPuber* ros_puber, Map* map, Camera* cam, Config* cfg ) :local_mapping_ ( local_mapping ), ros_puber_ ( ros_puber ), map_ ( map ), cam_ ( cam ), cfg_ ( cfg ), mbFinishRequested(false),mSkipIndex(SKIP_NUMBER),mSegmentTime(0),imgIndex(0)
{

    model_file = pascal_prototxt;
    trained_file = pascal_caffemodel;
    LUT_file = pascal_png;
    label_colours = cv::imread(LUT_file,1);
    cv::cvtColor(label_colours, label_colours, CV_RGB2BGR);
    mImgSegmentLatest=cv::Mat(Camera::s_height,Camera::s_width,CV_8UC1);
    mbNewImgFlag=false;
    mptSegment =new thread(&tracking_slam::Segment::Predict, this);
}

void Segment::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

bool Segment::isNewImgArrived()
{
    unique_lock<mutex> lock(mMutexGetNewImg);
    if(mbNewImgFlag)
    {
        mbNewImgFlag=false;
        return true;
    }
    else
    return false;
}


bool Segment::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}
  
void Segment::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested=true;
}

void Segment::ProduceImgSegment()
{
    std::unique_lock <std::mutex> lock(mMutexNewImgSegment);
    mImgTemp=mImgSegmentLatest;
    mImgSegmentLatest=mImgSegment;
    mImgSegment=mImgTemp;
    //mpTracker->mbNewSegImgFlag=true;
}

void Segment::Predict()
{   
    classifier=new Classifier(model_file, trained_file);
    cout << "Load model ..."<<endl;

    
    
    while(true)
    {   
         if ( checkNewFrame() == true ) {
         KeyFrame* kf = getNewKeyFrame();
         RunTimer t;


         /**************depth image and rgb image preprocessing******************/

         cv::Mat depth_blur = kf->depth_.clone();
         //cv::medianBlur(depth_blur, kf->depth_, 5);
         drawDepth( kf->depth_, kf->image_objects_ );

		 t.start();
         mImgSegment=classifier->Predict(kf->rgb_, label_colours);   //predict

         /*delate small areas && repair tag*/
         mImgSegment_color = mImgSegment.clone();
         cv::Mat mImgSegment_backup = mImgSegment.clone();
         cv::Mat label_tmp;
         DelSmallAera(mImgSegment_backup,label_tmp,cfg_->area_thread_);       //delate some areas.
        //  /**************dilate label_tmp******************/
        //  cv::Mat dilate_element = cv::getStructuringElement(MORPH_CROSS, Size(2, 2));
        //  cv::dilate(label_tmp.clone(), label_tmp, dilate_element);
         
         /*          K-means          */
         ////std::vector<Eigen::Vector2d> pts2d;
         ////std::vector<Eigen::Vector3d> pts3d;
         ////computePointCloud(cam_,depth_downsample,pts2d,pts3d);
         ////int K_clusters = std::ceil ( ( double ) pts2d.size() / ( double ) cfg_->dpc_npts_per_cluster_ );
         // int K_clusters = 10;
         ////std::vector<std::vector<Eigen::Vector2d>> clusters_2d;
         ////std::vector<std::vector<Eigen::Vector3d>> clusters_3d;
         ////segmentPointCloudByKmeans ( pts2d, pts3d, K_clusters, clusters_2d, clusters_3d );
         ////cv::Mat kmeans ;
         //ClustersImage ( kmeans, depth_downsample.cols, depth_downsample.rows, clusters_2d, cluster_colors_ );
         /**************color image and depth image downsample**************** */
         cv::Mat color2gray;
         cv::Mat color_downsample;
         cv::Mat depth_downsample;
         cv::Mat color_edge;
         cv::Mat edge_x;
         cv::Mat edge_y;
         cv::Mat abs_x;
         cv::Mat abs_y;
         
         //cv::pyrDown(kf->rgb_, color_downsample, mImgSegment_backup.size());
         cv::resize(kf->rgb_, color_downsample, mImgSegment_backup.size());
         cv::resize(kf->depth_, depth_downsample, mImgSegment_backup.size());

         cv::Mat color_edge_threshold(mImgSegment_backup.size(), CV_8U);
         cv::Mat rgbEnhance = color_downsample.clone();
         cv::Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, 0, 5, 0, 0, -1, 0);
         cv::filter2D(rgbEnhance, color_downsample, CV_8UC3, kernel);
         cv::cvtColor(color_downsample, color2gray, cv::COLOR_BGR2GRAY);

         //x-scale
         cv::Sobel(color2gray, edge_x, CV_16S, 1, 0, 3, 1, 1,BORDER_DEFAULT);
         cv::convertScaleAbs(edge_x, abs_x);

         //y-scale
         cv::Sobel(color2gray, edge_y, CV_16S, 0, 1, 3, 1, 1,BORDER_DEFAULT);
         cv::convertScaleAbs(edge_y, abs_y);

         //merge
         cv::addWeighted(abs_x, 1.2, abs_y, 1.2, 0, color_edge);
         
         cv::convertScaleAbs(color_edge, color_edge_threshold, 1, 0);
         for (int u=0; u < color_edge_threshold.rows ; u++)
             {
                 //cout << "rows:" << u << endl;
                 for (int v=0; v < color_edge_threshold.cols; v++)
                     {   
                         if (color_edge_threshold.ptr(u)[v] > 150)
                         {
                             color_edge_threshold.ptr<uchar> (u)[v] = 255;
                         }
                         else
                         {
                             color_edge_threshold.ptr<uchar> (u)[v] = 0;
                         }
                    }
            }
         /****************************** */
        //  cout << "color_downsample.size():" << color_downsample.size() << endl;
        //  cout << "color2gray.size():" << color2gray.size() << endl;
        //  cout << "kf->depth_.size():" << kf->depth_.size() << endl;
        // cout << "depth_downsample.size():" << depth_downsample.size() << endl;
        // cout << "label_tmp.size():" << label_tmp.size() << endl;


         //cv::medianBlur(depth_blur, depth_downsample, 5);
         //extend_by_depth(depth_downsample,label_tmp,cfg_->depth_thread_, cfg_->hight_limit_);        
         
        extend_if_notedge(color_edge_threshold, depth_downsample, label_tmp, cfg_->depth_thread_, cfg_->hight_limit_);

        int left = 224;
        int right = 0;
        int up = 224;
        int down = 0;
        

        
         for (int u=0; u < label_tmp.rows ; u++)
             {
                 for (int v=0; v < label_tmp.cols; v++)
                     {   
                         if ((int) label_tmp.ptr<uchar>(u)[v] == 15 ) 
                         {
                             left = left < v ? left : v;
                             right = right > v ? right : v;
                             up = up < u ? up : u;
                             down = down > u ? down : u;
                         }
                    }
                    
            }

        if (left != 224 && right !=0 && up != 224 && down !=0)
        {
            
            if (left < 15)
            {
                left = 0;
            }
            if (up < 30)
            {
                up = 0;
            }
            if (right > 210)
            {
                right = 224;
            }
            if (down > 210)
            {
                down = 224;
            }

            cv::Mat depthROI = depth_downsample.clone();
            for (int mm = 0; mm < depthROI.rows; mm++)
            {
                for (int nn = 0; nn < depthROI.cols; nn++)
                {

                    if (nn < left - 10 || nn > right + 10 || mm < up - 50 || mm > down + 10)
                     {
                         depthROI.ptr<float> (mm)[nn] = 30.0f;
                     }
                }
            }
            
            extend_by_depth_ROI(depthROI, label_tmp, cfg_->depth_thread_second_, cfg_->hight_limit_);
        }
        

         for (int u=0; u < label_tmp.rows ; u++)
             {
                 for (int v=0; v < label_tmp.cols; v++)
                     {   
                         if ((int) label_tmp.ptr<uchar>(u)[v] == 15 ) 
                         {
                             depth_downsample.ptr<float> (u)[v] = 0.0f;
                         }
                    }
                    
            }

         
        //  cv::Mat f;
        //  cv::cvtColor(label_tmp,f, CV_GRAY2BGR);
        //  cv::namedWindow("label3");
        //  cv::imshow("label3", f);
        //  waitKey(0);
         cv::Mat dilate_element = cv::getStructuringElement(MORPH_RECT, Size(2, 2));
         cv::dilate(label_tmp.clone(), label_tmp, dilate_element);

         //Convert to color image
         cv::Mat tmp;
         cv::Mat final_label_color;
         cv::cvtColor(mImgSegment, mImgSegment_color, CV_GRAY2BGR);
         cv::cvtColor(label_tmp, tmp, CV_GRAY2BGR);
         LUT(mImgSegment_color, label_colours, mImgSegment_color_final);
         LUT(tmp, label_colours, final_label_color);
         cv::resize(mImgSegment, mImgSegment, cv::Size(Camera::s_width,Camera::s_height) );
         cv::resize(label_tmp, label_tmp, cv::Size(Camera::s_width,Camera::s_height) );
         cv::resize(mImgSegment_color_final, mImgSegment_color_final, cv::Size(Camera::s_width,Camera::s_height) );
         cv::resize(final_label_color, final_label_color, cv::Size(Camera::s_width,Camera::s_height) );
         imgIndex++;

         

         kf->image_label_ = label_tmp;           //Save the prediction results
         
         // no++;
         // stringstream s_no;
         // s_no<<no;
         // string number = s_no.str();
         // cv::imwrite(segment_path+number+ext,final_label_color);
         // cv::imwrite(reference_path+number+ext,mImgSegment_color_final);
         // cv::imwrite(rgb_path+number+ext,kf->depth_);
         // cv::imwrite(kmeans_path+number+ext,kmeans);
             
        
        /**************depth processing******************/
        
        //cv::GaussianBlur(depth_blur, kf->depth_, cv::Size(7, 7), 0, 0);
        //cv::medianBlur(depth_blur, kf->depth_, 5);

        //cout << "label_tmp.rows:" << label_tmp.rows << " " << "label_tmp.cols:" << label_tmp.cols << endl;
        int left_o = 960;
        int right_o = 0;
        int up_o = 540;
        int down_o = 0;
        for (int u=0; u < label_tmp.rows ; u++)
             {
                 for (int v=0; v < label_tmp.cols; v++)
                     {   
                         if ((int) label_tmp.ptr<uchar>(u)[v] == 15 ) 
                         {
                             kf->depth_.ptr<float> (u)[v] = 0.0f;
                             left_o = left_o < v ? left_o : v;
                             right_o = right_o > v ? right_o : v;
                             up_o = up_o < u ? up_o : u;
                             down_o = down_o > u ? down_o : u;
                         }
                    }
                    
            }
        

        cv::Mat depthROI_o = kf->depth_.clone();
        cv::Mat depth_process;
        cv::Mat depth_erode;
        cv::Mat erode_element = cv::getStructuringElement(MORPH_RECT, Size(10, 10));


        if (left_o != 960 && right_o !=0 && up_o != 540 && down_o !=0)
        {
            
            if (left_o < 30)
            {
                left_o = 0;
            }
            if (up_o < 50)
            {
                up_o = 0;
            }
            if (right_o > 900)
            {
                right_o = 959;
            }
            if (down_o > 520)
            {
                down_o = 539;
            }

            for (int u=0; u < depthROI_o.rows ; u++)
            {
             for (int v=0; v < depthROI_o.cols; v++)
                {   
                     if (v < left_o || v > right_o || u < up_o || u > down_o)
                     {
                         depthROI_o.ptr<float> (u)[v] = 8.0f;
                     }
                         
                }
                    
            }
  
        }
        else
        {
            for (int u=0; u < 50 ; u++)
            {
                for (int v=0; v < depthROI_o.cols; v++)
                {   
                     depthROI_o.ptr<float> (u)[v] = 0.0f;
                }  
            }
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        
        cv::threshold(depthROI_o, depth_process, 0.1, 8, THRESH_BINARY);

        //cv::erode(depth_process, depth_erode, erode_element);
        //cv::dilate(depth_erode.clone(), depth_erode,erode_element);
        cv::morphologyEx(depth_process, depth_erode, MORPH_OPEN, erode_element);

        // drawDepth(depth_erode, kf->blue_depth_);
        // cv::namedWindow("after");
        // cv::imshow("after", kf->blue_depth_);
        // waitKey(0);


        // drawDepth( kf->depth_, kf->image_source_clusters_);

        //cout << "kf->depth_.type():" << kf->depth_.type() << endl;

        //cout << "depth_erode.size():" << depth_erode.size() << endl;
        //cout << "depth_erode.channels():" << depth_erode.channels() << endl;

        for (int u=0; u < depth_erode.rows ; u++)
             {
                 for (int v=0; v < depth_erode.cols; v++)
                     {   
                         if ( depth_erode.ptr<float> (u)[v] == 0) 
                         {
                             kf->depth_.ptr<float> (u)[v] = 0.0f;
                         }
                    }
            }



        /**************depth processing_end******************/

        //kf->image_objects_ = final_label_color;
        //kf->image_mask_ = kf->rgb_;

        kf->image_source_clusters_ = mImgSegment_color_final;
        //kf->image_source_clusters_ = depth_process;
        //kf->image_source_clusters_ = kf->rgb_;

        //drawDepth(kf->depth_, kf->image_mask_);

        drawResult( kf->depth_, label_tmp, kf->image_mask_);
        t.stop();

        /**************write_image******************/
        char image_route_label[500];
        char image_route_depth[500];
        char image_route_rgb[500];
        char image_route_depth_label[500];
        char image_label[500];
        char image_edge[500];
        
        //This block can show the quality of pictures.
        //sprintf(image_route_label, "%s%d%s", "/home/leo/leo_slam/src/tracking_slam/results/pub_image/final_label/final_label", index, ".jpg");
        //sprintf(image_route_depth, "%s%d%s", "/home/leo/leo_slam/src/tracking_slam/results/pub_image/depth/depth", index, ".jpg");
        //sprintf(image_route_rgb, "%s%d%s", "/home/leo/leo_slam/src/tracking_slam/results/pub_image/rgb/rgb", index, ".jpg");
        //sprintf(image_route_depth_label, "%s%d%s", "/home/leo/leo_slam/src/tracking_slam/results/pub_image/depth_label/depth_label", index, ".jpg");
        //sprintf(image_label, "%s%d%s", "/home/leo/leo_slam/src/tracking_slam/results/pub_image/label/label", index, ".jpg");
        //sprintf(image_edge, "%s%d%s", "/home/leo/leo_slam/src/tracking_slam/results/pub_image/edge/edge", index++, ".jpg");

        //imwrite(image_route_label, final_label_color);
        //imwrite(image_route_depth, kf->image_objects_);
        //imwrite(image_route_rgb, kf->rgb_);
        //imwrite(image_route_depth_label, kf->image_mask_);
        //imwrite(image_label, kf->image_source_clusters_);
        //imwrite(image_edge, color_edge_threshold);
        /************************************************/

        /**************write_image_end******************/

		std::cout << "\nKeyFrame " << kf->kfid_ << " Processing time: " << t.duration() << "\n\n";

        sum_process += t.duration();
        std::cout << "\nKeyFrame " << kf->kfid_ << " Sum Processing time: " << sum_process << "\n\n";

        // Send to local mapping
        local_mapping_->insertKeyFrame ( kf );

        // publish results of the dynamic pixel culling.
        ros_puber_->pubDynamicPixelCullingResults ( kf );

        // free the depth image of the last Nth kf.
        passed_kfs_queue_.push ( kf );

        if ( passed_kfs_queue_.size() > ( cfg_->dpc_n_near_kfs_+1 ) ) {
            KeyFrame* pass_kf = passed_kfs_queue_.front();
            passed_kfs_queue_.pop();
            pass_kf->freeDepth();
            }
        } // if new KF come in.
        usleep(5000);
    }
    

    
}

bool Segment::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    return ( !kfs_queue_.empty() );
} // checkNewFrame

KeyFrame* Segment::getNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    KeyFrame* kf = kfs_queue_.front();
    kfs_queue_.pop();
    return  kf;
} // getNewKeyFrame

void Segment::insertKeyFrame ( KeyFrame* kf )
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    kfs_queue_.push ( kf );
} // insertKeyFrame

void Segment::computePointCloud ( Camera* cam_, const cv::Mat& depth, vector< Eigen::Vector2d >& pt2d, vector< Eigen::Vector3d >& pt3d )
{
    pt3d.clear();
    pt2d.clear();
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {
            float dp = depth.at<float> ( v,u );
            if ( ( dp > 0.1 ) && ( dp < 8.0 ) ) {
                Eigen::Vector3d ptc = cam_->img2Cam ( Eigen::Vector2d ( u, v ), dp );
                pt3d.push_back ( ptc );
                pt2d.push_back ( Eigen::Vector2d ( u, v ) );
            }
        } // for all pixels.
    } //  for all pixels.
} // computePointCloud

void Segment::drawResult( const cv::Mat& depth, cv::Mat& label, cv::Mat& color_mask)
{
    const cv::Scalar color_static ( 255, 0, 0 );
    const cv::Scalar color_dynamic ( 0, 0, 255 );
    const cv::Scalar color_nodepth ( 0, 0, 0 );

    color_mask = cv::Mat ( cfg_->cam_height_, cfg_->cam_width_, CV_8UC3, color_static );


    // drawNoDepth pixels
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {
            float dp = depth.at<float> ( v,u );

            // drawNoDepth pixels
            if ( ( dp < cfg_->cam_dmin_ ) || ( dp > cfg_->cam_dmax_ ) ) {
                color_mask.at<cv::Vec3b> ( v,u ) [0] = color_nodepth[0];
                color_mask.at<cv::Vec3b> ( v,u ) [1] = color_nodepth[1];
                color_mask.at<cv::Vec3b> ( v,u ) [2] = color_nodepth[2];
            }

            ///draw dynamic objects
            if ( (int) label.ptr<uchar>(v)[u] == 15 ) {
                color_mask.at<cv::Vec3b> ( v,u ) [0] = color_dynamic[0];
                color_mask.at<cv::Vec3b> ( v,u ) [1] = color_dynamic[1];
                color_mask.at<cv::Vec3b> ( v,u ) [2] = color_dynamic[2];
            }

        } // for all pixels.
    } //  for all pixels.

}//drawResult

void Segment::drawDepth(const cv::Mat& depth, cv::Mat& blue_depth)
    {
    const cv::Scalar color_static ( 255, 0, 0 );
    const cv::Scalar color_dynamic ( 0, 0, 255 );
    const cv::Scalar color_nodepth ( 0, 0, 0 );

    blue_depth = cv::Mat ( cfg_->cam_height_, cfg_->cam_width_, CV_8UC3, color_static );


    // drawNoDepth pixels
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {
            float dp = depth.at<float> ( v,u );

            // drawNoDepth pixels
            if ( ( dp < cfg_->cam_dmin_ ) || ( dp > cfg_->cam_dmax_ ) ) {
                blue_depth.at<cv::Vec3b> ( v,u ) [0] = color_nodepth[0];
                blue_depth.at<cv::Vec3b> ( v,u ) [1] = color_nodepth[1];
                blue_depth.at<cv::Vec3b> ( v,u ) [2] = color_nodepth[2];
            }
        }

    }
}//drawDepth

void Segment::drawDepth1(const cv::Mat& depth, cv::Mat& blue_depth)
    {
    const cv::Scalar color_static ( 255, 0, 0 );
    const cv::Scalar color_dynamic ( 0, 0, 255 );
    const cv::Scalar color_nodepth ( 0, 0, 0 );

    blue_depth = cv::Mat ( 224, 224, CV_8UC3, color_static );


    // drawNoDepth pixels
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {
            float dp = depth.at<float> ( v,u );

            // drawNoDepth pixels
            if (  (dp < cfg_->cam_dmin_ ) || ( dp > cfg_->cam_dmax_ ) ) {
                blue_depth.at<cv::Vec3b> ( v,u ) [0] = color_nodepth[0];
                blue_depth.at<cv::Vec3b> ( v,u ) [1] = color_nodepth[1];
                blue_depth.at<cv::Vec3b> ( v,u ) [2] = color_nodepth[2];
            }
        }

    }
}//drawDepth1



}//namespace

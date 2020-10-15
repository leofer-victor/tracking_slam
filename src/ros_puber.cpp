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

#include <tracking_slam/ros_puber.h>
#include <tracking_slam/frame.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <tracking_slam/run_timer.h>

namespace tracking_slam
{
RosPuber::RosPuber ( ros::NodeHandle& nh )
{
	
    // Current frame.
    puber_robot_pose_ = nh.advertise<geometry_msgs::PoseStamped> ( "tracking_slam/cur_robot_pose", 1, true );
    image_transport::ImageTransport it ( nh );
    puber_img_match_ = it.advertise ( "tracking_slam/cur_img_match", 1, true );

    // Dynamic pixel culling.
    puber_dpc_img_objects_ = it.advertise ( "tracking_slam/dpc_img_objects", 1, true );
    puber_dpc_img_clusters_ = it.advertise ( "tracking_slam/dpc_img_clusters", 1, true );
    puber_dpc_img_mask_ = it.advertise ( "tracking_slam/dpc_img_mask", 1, true );


    // KFs and pose graph. Sparse Map
    puber_mappoints_ = nh.advertise<sensor_msgs::PointCloud> ( "tracking_slam/mappoints", 1, true );
    puber_kfs_puber_ = nh.advertise<geometry_msgs::PoseArray> ( "tracking_slam/keyframes", 1, true );
    puber_encoder_graph_ = nh.advertise<visualization_msgs::Marker> ( "tracking_slam/encoder_graph", 1, true );
    puber_loop_graph_ = nh.advertise<visualization_msgs::Marker> ( "tracking_slam/loop_graph", 1, true );
    puber_visual_graph_ = nh.advertise<visualization_msgs::Marker> ( "tracking_slam/visual_graph", 1, true );

    // OctoMap
    puber_octomap_ = nh.advertise<octomap_msgs::Octomap> ( "tracking_slam/octomap", 1, true );


    //Sub-octoMap(Leo)
    ////puber_suboctomap_ = nh.advertise<octomap_msgs::Octomap> ( "tracking_slam/suboctomap", 1, true );


} // RosPuber

void RosPuber::pubCurrentFrame ( Frame* frame )
{
	RunTimer t; t.start();
	
    // time stamp
    ros::Time timestamp ( frame->timestamp_ );

    // show cur frame pose
    Sophus::SE2 cur_pose = frame->getSE2PoseRefKF();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = cur_pose.translation() ( 0 );
    pose.pose.position.y = cur_pose.translation() ( 1 );
    pose.pose.orientation = tf::createQuaternionMsgFromYaw ( cur_pose.so2().log() );
    pose.header.frame_id = "world";
    pose.header.stamp = timestamp;
    puber_robot_pose_.publish ( pose );

    // Pub TF.
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin ( tf::Vector3 ( cur_pose.translation() ( 0 ), cur_pose.translation() ( 1 ), 0.0 ) );
    tf::Quaternion q;
    q.setRPY ( 0, 0, cur_pose.so2().log() );
    transform.setRotation ( q );
    br.sendTransform ( tf::StampedTransform ( transform, timestamp, "world", "robot" ) );

    // Show image with matched keypoints.
    if ( !frame->img_match_.empty() ) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage ( std_msgs::Header(), "bgr8", frame->img_match_ ).toImageMsg();
        puber_img_match_.publish ( msg );
    }
    
    t.stop();
	//std::cout << "Pub frame time cost: " << t.duration() << "\n";
} // pubCurrentFrame


void RosPuber::pubDynamicPixelCullingResults ( KeyFrame* kf )
{
	RunTimer t; t.start();
	
    // puber object detection results.
    if ( !kf->image_objects_.empty() ) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage ( std_msgs::Header(), "bgr8", kf->image_objects_ ).toImageMsg();
        puber_dpc_img_objects_.publish ( msg );
    }

    // puber cluster results
    if ( !kf->image_source_clusters_.empty() ) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage ( std_msgs::Header(), "bgr8", kf->image_source_clusters_ ).toImageMsg();
        puber_dpc_img_clusters_.publish ( msg );
    }

    // puber mask results
    if ( !kf->image_mask_.empty() ) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage ( std_msgs::Header(), "bgr8", kf->image_mask_ ).toImageMsg();
        puber_dpc_img_mask_.publish ( msg );
    }
    
	t.stop();
	//std::cout << "Pub DPC time cost: " << t.duration() << "\n";
} // pubDynamicPixelCullingResults


void RosPuber::pubSparseMap ( Map* map )
{
	RunTimer t; t.start();
    /**** Publish Map Points. ****/
    std::set<MapPoint*> mpts = map->getAllMapPoints();
    if ( !mpts.empty() ) {
        std::vector<geometry_msgs::Point32> points;
        for ( MapPoint* mpt : mpts ) {
            geometry_msgs::Point32 pt;
            Eigen::Vector3d ptw = mpt->getPosition();
            pt.x = ptw ( 0 );
            pt.y = ptw ( 1 );
            pt.z = ptw ( 2 );
            points.push_back ( pt );
        } //  for all mpts

        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = "world";
        cloud.header.stamp = ros::Time::now();
        cloud.points = points;
        puber_mappoints_.publish ( cloud );
    } // if have mpts


    /**** Publish KFs ****/
    std::map<long int, KeyFrame*> kfs = map->getAllKeyFrames();

    geometry_msgs::PoseArray kfs_array;
    kfs_array.header.frame_id = "world";
    kfs_array.header.stamp = ros::Time::now();

    std::map<long int, KeyFrame*>::iterator kf_it;
    for ( kf_it = kfs.begin(); kf_it != kfs.end(); kf_it ++ ) {
        KeyFrame* kf = kf_it->second;
        geometry_msgs::Pose pose;
        Sophus::SE2 Twr = kf->getSE2Pose();
        pose.position.x = Twr.translation() ( 0 );
        pose.position.y = Twr.translation() ( 1 );
        pose.orientation = tf::createQuaternionMsgFromYaw ( Twr.so2().log() );
        kfs_array.poses.push_back ( pose );
    }
    puber_kfs_puber_.publish ( kfs_array );


    /**** Publish Encoder Graph ****/
    visualization_msgs::Marker marker_encoder;
    marker_encoder.header.frame_id = "world";
    marker_encoder.header.stamp = ros::Time();
    marker_encoder.ns = "tracking_slam";
    marker_encoder.id = 0;
    marker_encoder.type = visualization_msgs::Marker::LINE_LIST;
    marker_encoder.action = visualization_msgs::Marker::ADD;
    marker_encoder.color.a = 1; // Don't forget to set the alpha!
    marker_encoder.color.r = 0;
    marker_encoder.color.g = 0;
    marker_encoder.color.b = 0;
    marker_encoder.scale.x = 0.01;

    for ( kf_it = kfs.begin(); kf_it != kfs.end(); kf_it ++ ) {
        geometry_msgs::Point pt1, pt2;
        KeyFrame* kf1 = kf_it->second;
        KeyFrame* kf2 = kf1->getLastKeyFrameEdge();

        Sophus::SE2 kf1_Twr = kf1->getSE2Pose();
        Sophus::SE2 kf2_Twr = kf2->getSE2Pose();

        pt1.x = kf1_Twr.translation() ( 0 );
        pt1.y = kf1_Twr.translation() ( 1 );
        pt2.x = kf2_Twr.translation() ( 0 );
        pt2.y = kf2_Twr.translation() ( 1 );

        marker_encoder.points.push_back ( pt1 );
        marker_encoder.points.push_back ( pt2 );
    } // for all kfs
    puber_encoder_graph_.publish ( marker_encoder );

    /**** Publish Visual Graph ****/
    visualization_msgs::Marker marker_visual;
    marker_visual.header.frame_id = "world";
    marker_visual.header.stamp = ros::Time();
    marker_visual.ns = "tracking_slam";
    marker_visual.id = 1;
    marker_visual.type = visualization_msgs::Marker::LINE_LIST;
    marker_visual.action = visualization_msgs::Marker::ADD;
    marker_visual.color.a = 1; // Don't forget to set the alpha!
    marker_visual.color.r = 0;
    marker_visual.color.g = 1;
    marker_visual.color.b = 0;
    marker_visual.scale.x = 0.015;

    for ( kf_it = kfs.begin(); kf_it != kfs.end(); kf_it ++ ) {
        geometry_msgs::Point pt1, pt2;

        KeyFrame* kf1 = kf_it->second;

        Sophus::SE2 kf1_Twr = kf1->getSE2Pose();

        pt1.x = kf1_Twr.translation() ( 0 );
        pt1.y = kf1_Twr.translation() ( 1 );


        std::set<KeyFrame*> ob_kfs = kf1->getVisualEdge();
        for ( KeyFrame* okf : ob_kfs ) {
            Sophus::SE2 okf_Twr = okf->getSE2Pose();

            pt2.x =okf_Twr.translation() ( 0 );
            pt2.y =okf_Twr.translation() ( 1 );
            marker_visual.points.push_back ( pt1 );
            marker_visual.points.push_back ( pt2 );
        }

    } // for all kfs
    puber_visual_graph_.publish ( marker_visual );


    /**** Publish Loop Graph ****/
    visualization_msgs::Marker marker_loop;
    marker_loop.header.frame_id = "world";
    marker_loop.header.stamp = ros::Time();
    marker_loop.ns = "tracking_slam";
    marker_loop.id = 2;
    marker_loop.type = visualization_msgs::Marker::LINE_LIST;
    marker_loop.action = visualization_msgs::Marker::ADD;
    marker_loop.color.a = 1; // Don't forget to set the alpha!
    marker_loop.color.r = 1;
    marker_loop.color.g = 0;
    marker_loop.color.b = 0;
    marker_loop.scale.x = 0.02;

    for ( kf_it = kfs.begin(); kf_it != kfs.end(); kf_it ++ ) {
        geometry_msgs::Point pt1, pt2;

        KeyFrame* kf1 = kf_it->second;
        Sophus::SE2 kf1_Twr = kf1->getSE2Pose();

        pt1.x = kf1_Twr.translation() ( 0 );
        pt1.y = kf1_Twr.translation() ( 1 );

        std::vector<KeyFrame*> loop_edges;
        std::vector<Sophus::SE2> loop_delta_T;
        kf1->getLoopEdge ( loop_edges, loop_delta_T );
        for ( KeyFrame* okf : loop_edges ) {

            Sophus::SE2 okf_Twr = okf->getSE2Pose();
            pt2.x =okf_Twr.translation() ( 0 );
            pt2.y =okf_Twr.translation() ( 1 );
            marker_loop.points.push_back ( pt1 );
            marker_loop.points.push_back ( pt2 );
        }
    } // for all kfs
    puber_loop_graph_.publish ( marker_loop );
	
	
	t.stop();
	// std::cout << "Pub sparse map time cost: " << t.duration() << "\n";
} // pubSparseMap


void RosPuber::pubOctoMap ( octomap::ColorOcTree* octree )
{
	RunTimer t; t.start();
    if ( puber_octomap_.getNumSubscribers() == 0 ) {
        return;
    }

    octomap_msgs::Octomap map_msg;
    // octomap_msgs::binaryMapToMsg( *octree, map_msg );
    octomap_msgs::fullMapToMsg( *octree, map_msg);
    map_msg.header.frame_id = "world";
    map_msg.header.stamp = ros::Time::now();
    puber_octomap_.publish ( map_msg );
	octree->write("/home/leo/experiments/ours_octomap.ot");
	t.stop();
	// std::cout << "Puber OctoMap time cost: " << t.duration() << "\n";
} // pubOctoMap



}

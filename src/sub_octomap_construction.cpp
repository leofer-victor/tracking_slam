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


#include <tracking_slam/sub_octomap_construction.h>
#include <tracking_slam/run_timer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace tracking_slam
{

SubOctoMapConstruction::SubOctoMapConstruction ( OctoMapFusion* octomap_fusion, Config* cfg ) : octomap_fusion_ ( octomap_fusion ), cfg_ ( cfg )
{
    sub_octomap_construction_thread_ = new std::thread ( &SubOctoMapConstruction::processing, this );
    nkf_passed_ = 0;
} // SubOctomapping


void SubOctoMapConstruction::processing()
{
    while ( true ) {

        if ( checkNewFrame() == true ) {
            KeyFrame* kf = getNewKeyFrame();

			/*std::unique_lock<mutex> lock ( mutex_all_data_ );*/ 
			
            // construct sub map.
            if ( nkf_passed_ == 0 ) {
                cur_sub_map_ = new SubOctomap ( cfg_ );
            }

            // Construct 3D point cloud in the camera frame.
            octomap::Pointcloud point_cloud_c;
            octomap::Pointcloud point_cloud_color;
            //createCleanCloud ( kf, point_cloud_c ); // voxel_grid filtered
			createPointCloud( kf, point_cloud_c, point_cloud_color );
            //createCleanCloud_color_octree( kf, point_cloud_c, point_cloud_color );
			// Insert one scan to full map.
			//octomap_fusion_->insertOneScan2FullMapAndPub ( kf, point_cloud_c );
            octomap_fusion_->insertOneScan2FullMapAndPub ( kf, point_cloud_c , point_cloud_color);
             
			// Insert one scan to cur sub map.
            //cur_sub_map_->insertKeyFrame ( kf, point_cloud_c );
            cur_sub_map_->insertKeyFrame ( kf, point_cloud_c,  point_cloud_color);

            // Check if need to construct new submap.
            nkf_passed_ ++;
            if ( nkf_passed_ > cfg_->oc_submap_size_ ) {
                nkf_passed_ = 0;
                SubOctomap* new_sub_map = cur_sub_map_;
				
				// insert one submap to fullmap.
				octomap_fusion_->insertSubMap ( new_sub_map );
            } // if have to insert submap.

        } // if have new keyframe.
        usleep ( 5000 );
    }// while true.
} // processing new keyframe.


void SubOctoMapConstruction::insertKeyFrame ( KeyFrame* kf )
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    kfs_queue_.push ( kf );
}


bool SubOctoMapConstruction::checkNewFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    return ( !kfs_queue_.empty() );
} // checkNewFrame

KeyFrame* SubOctoMapConstruction::getNewKeyFrame()
{
    std::unique_lock<mutex> lock ( mutex_kfs_queue_ );
    KeyFrame* kf = kfs_queue_.front();
    kfs_queue_.pop();
    return  kf;
} // getNewKeyFrame


void SubOctoMapConstruction::createPointCloud ( KeyFrame* kf, octomap::Pointcloud& point_cloud, octomap::Pointcloud& point_cloud_color )
{
    cv::Mat& depth = kf->depth_;
    Camera* camera = kf->cam_;
    cv::Mat segpic = kf->image_label_;

    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {

            float dp = depth.at<float> ( v,u );
            if ( dp > cfg_->cam_dmin_ && dp < cfg_->cam_dmax_ ) {
                Eigen::Vector3d ptc = camera->img2Cam ( Eigen::Vector2d ( u, v ), dp );
                point_cloud.push_back ( ptc[0], ptc[1], ptc[2] );
                if(segpic.cols != 0)
                {   
                    //cout << "label = " << segpic.ptr<uchar>(v)[u] << endl;
                    int r = kf->colormap_[(int)segpic.ptr<uchar>(v)[u]][0];
                    int g = kf->colormap_[(int)segpic.ptr<uchar>(v)[u]][1];
                    int b = kf->colormap_[(int)segpic.ptr<uchar>(v)[u]][2];
                    // cout << "r = " << kf->colormap_[3][0] << ", g = " << kf->colormap_[3][1] << ", b = " << kf->colormap_[3][2] <<endl;
                    // cout << "r = " << r << ", g = " << g << ", b = " << b <<endl;
                    point_cloud_color.push_back(r,g,b);
                }
            } // if is good point.
        } // for all pixels.
    } //  for all pixels.

} // createPointCloud

void SubOctoMapConstruction::createCleanCloud_color_octree ( KeyFrame* kf, octomap::Pointcloud& point_cloud, octomap::Pointcloud& point_cloud_color )
{
    cv::Mat& depth = kf->depth_;
    Camera* camera = kf->cam_;
    cv::Mat segpic = kf->image_label_;

    // Create point cloud from the depth image.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ( new pcl::PointCloud<pcl::PointXYZ> );
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {

            float dp = depth.at<float> ( v,u );
            if ( dp > cfg_->cam_dmin_ && dp < cfg_->cam_dmax_ ) {
                Eigen::Vector3d ptc = camera->img2Cam ( Eigen::Vector2d ( u, v ), dp );

                cloud->points.push_back ( pcl::PointXYZ ( ptc[0], ptc[1], ptc[2] ) );
                if(segpic.cols != 0)
                {   
                    int r = kf->colormap_[(int)segpic.ptr<uchar>(v)[u]][0];
                    int g = kf->colormap_[(int)segpic.ptr<uchar>(v)[u]][1];
                    int b = kf->colormap_[(int)segpic.ptr<uchar>(v)[u]][2];
                    point_cloud_color.push_back(r,g,b);
                }

            } // if is good point.
        } // for all pixels.
    } //  for all pixels.


	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud ( cloud );
	sor.setLeafSize ( 0.25*cfg_->oc_voxel_size_, 0.25*cfg_->oc_voxel_size_, 0.25*cfg_->oc_voxel_size_ );
	sor.filter ( *cloud_filtered );

	// Assigned octomap cloud
	for ( size_t i = 0; i < cloud_filtered->size(); i ++ ) {
		pcl::PointXYZ pt = cloud_filtered->points.at ( i );
		point_cloud.push_back ( pt.x, pt.y, pt.z );
	} 
} // createCleanCloud_color_octree

void SubOctoMapConstruction::createCleanCloud ( KeyFrame* kf, octomap::Pointcloud& point_cloud )
{
    cv::Mat& depth = kf->depth_;
    Camera* camera = kf->cam_;

    // Create point cloud from the depth image.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ( new pcl::PointCloud<pcl::PointXYZ> );
    for ( int v = 0; v < depth.rows; v ++ ) {
        for ( int u = 0; u < depth.cols; u ++ ) {

            float dp = depth.at<float> ( v,u );
            if ( dp > cfg_->cam_dmin_ && dp < cfg_->cam_dmax_ ) {
                Eigen::Vector3d ptc = camera->img2Cam ( Eigen::Vector2d ( u, v ), dp );

                cloud->points.push_back ( pcl::PointXYZ ( ptc[0], ptc[1], ptc[2] ) );
            } // if is good point.
        } // for all pixels.
    } //  for all pixels.


	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud ( cloud );
	sor.setLeafSize ( 0.25*cfg_->oc_voxel_size_, 0.25*cfg_->oc_voxel_size_, 0.25*cfg_->oc_voxel_size_ );
	sor.filter ( *cloud_filtered );

	// Assigned octomap cloud
	for ( size_t i = 0; i < cloud_filtered->size(); i ++ ) {
		pcl::PointXYZ pt = cloud_filtered->points.at ( i );
		point_cloud.push_back ( pt.x, pt.y, pt.z );
	} 
} // createCleanCloud_octree

void SubOctoMapConstruction::insertAllData2OctoMapFusion()
{
// 	std::unique_lock<mutex> lock ( mutex_all_data_ );
	SubOctomap* new_sub_map = cur_sub_map_;
	if ( new_sub_map->kfs_.size() == 0 ) {
		return;
	}
	octomap_fusion_->insertSubMap ( new_sub_map );
	nkf_passed_ = 0;
} // requestAllData

}//namespace

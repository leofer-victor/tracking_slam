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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h> 
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tracking_slam/tracking_slam.h>
#include <sys/stat.h>
//turtlebot
#include </home/leo/catkin_ws/devel/include/turtlebot3_msgs/SensorState.h>


using namespace tracking_slam;

class SensorGrabber
{
public:
	SensorGrabber ( TRACKING_SLAM* slam ) :slam_ ( slam ) {}
	
	void grabRGBD ( const sensor_msgs::ImageConstPtr& msg_rgb,const sensor_msgs::ImageConstPtr& msg_depth ) {
		// Get images.
		cv_bridge::CvImageConstPtr cv_ptr_rgb = cv_bridge::toCvShare ( msg_rgb );
		cv_bridge::CvImageConstPtr cv_ptr_depth  = cv_bridge::toCvShare ( msg_depth );
		
		// Add RGB-D images.
		slam_->addRGBDImage ( cv_ptr_rgb->image, cv_ptr_depth->image, cv_ptr_rgb->header.stamp.toSec() );
		
	}// grabRGBD

    //void grabEncoder ( const geometry_msgs::QuaternionStamped::ConstPtr& en_ptr ) {
	void grabEncoder ( const turtlebot3_msgs::SensorState::ConstPtr& en_ptr ) {
		
        //Parameters from TB3 
		double enl = en_ptr->left_encoder;
		double enr = en_ptr->right_encoder;
		
		double  ts= en_ptr->header.stamp.toSec();
		
		// Check bad data.
		{
			
			if ( last_enl_ == 0 && last_enr_ == 0 ) {
				last_enl_ = enl;
				last_enr_ = enr;
				return;
			}
			
			double delta_enl = fabs ( enl - last_enl_ );
			double delta_enr = fabs ( enr - last_enr_ );
			
			const double delta_th = 4000;
			
			if ( delta_enl > delta_th || delta_enr > delta_th ) {
				std::cout << "\nJUMP\n";
				return;
			}
			
			last_enl_ = enl;
			last_enr_ = enr;
		}
		
		// Add encoder measurements.
		slam_->addEncoder ( enl, enr, ts );
	}// grabEncoder


private:
    TRACKING_SLAM* slam_;
    double last_enl_ = 0;
    double last_enr_ = 0;

}; //class SensorGrabber

int main ( int argc, char** argv)
{
    //Init ROS
    ros::init(argc, argv, "tracking_slam");
    ros::start();
    ros::NodeHandle nh;
    
    // Load parameters
	////std::string tracking_slam_cfg_dir, orbvoc_dir, yolov3_classes_dir, yolov3_model_dir, yolov3_weights_dir;
	std::string tracking_slam_cfg_dir, orbvoc_dir, prototxt_file, pascal_caffemodel, pascal_png;
	std::string results_dir;
	if ( ! nh.getParam ( "/tracking_slam_node/tracking_slam_cfg_dir", tracking_slam_cfg_dir ) ) {
		std::cout << "Read tracking_slam_cfg_dir failure !\n";
        return -1;
    }
    
    if ( ! nh.getParam ( "/tracking_slam_node/orbvoc_dir", orbvoc_dir ) ) {
		std::cout << "Read orbvoc_dir failure !\n";
		return -1;
		
	}
	
	////if ( ! nh.getParam ( "/tracking_slam_node/yolov3_classes_dir", yolov3_classes_dir ) ) {
		////std::cout << "Read yolov3_classes_dir failure !\n";
	if ( ! nh.getParam ( "/tracking_slam_node/prototxt_file", prototxt_file ) ) {
		std::cout << "Read prototxt_file failure !\n";
		return -1;
	}
	////if ( ! nh.getParam ( "/tracking_slam_node/yolov3_model_dir", yolov3_model_dir ) ) {
		////std::cout << "Read yolov3_model_dir failure !\n";
	if ( ! nh.getParam ( "/tracking_slam_node/pascal_caffemodel", pascal_caffemodel ) ) {
		std::cout << "Read pascal_caffemodel failure !\n";
		return -1;
	}
	////if ( ! nh.getParam ( "/tracking_slam_node/yolov3_weights_dir", yolov3_weights_dir ) ) {
		////std::cout << "Read yolov3_weights_dir failure !\n";
	if ( ! nh.getParam ( "/tracking_slam_node/pascal_png", pascal_png ) ) {
		std::cout << "Read pascal_png failure !\n";
		return -1;
	}
	
	if ( ! nh.getParam ( "/tracking_slam_node/results_dir", results_dir ) ) {
		std::cout << "Read results_dir failure !\n";
		return -1;
	}

	// Load system configure.
	Config* cfg = new Config( tracking_slam_cfg_dir );
	
	// Init SLAM system.
	TRACKING_SLAM slam( nh, cfg, orbvoc_dir, prototxt_file, pascal_caffemodel, pascal_png );
	
	// Sub topics.
	SensorGrabber sensors ( &slam );
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub ( nh, cfg->cam_rgb_topic_, 1 );
	message_filters::Subscriber<sensor_msgs::Image> depth_sub ( nh, cfg->cam_depth_topic_, 1 );
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
	message_filters::Synchronizer<sync_pol> sync ( sync_pol ( 5 ), rgb_sub,depth_sub );
	sync.registerCallback ( boost::bind ( &SensorGrabber::grabRGBD,&sensors,_1,_2 ) ); //
	ros::Subscriber encoder_sub = nh.subscribe ( cfg->encoder_topic_, 1, &SensorGrabber::grabEncoder,&sensors );
	
	std::cout << "\n\nTRACKING-SLAM Started\n\n";
	
    ros::spin();

    // System Stoped.
    std::cout << "\n\nTRACKING-SLAM Stoped\n\n";

    // Save results.
	mkdir(results_dir.c_str(), S_IRWXU ); // Make a new folder.
	slam.saveFrames( results_dir +"/pose_frames.txt" );
	slam.saveKeyFrames( results_dir +"/pose_keyframes.txt" );
	slam.saveOctoMap( results_dir +"/octomap.bt" );
	
	std::cout << "Results have been saved to \"" + results_dir + "\".\n\n\n\n";

    return 0;
}
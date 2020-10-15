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


#include <tracking_slam/fillhole.h> 
  
void fillHole(const Mat srcBw, Mat &dstBw)  
{  
    Size m_Size = srcBw.size();  
    Mat Temp=Mat::zeros(m_Size.height+2,m_Size.width+2,srcBw.type());//expend image  
    srcBw.copyTo(Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));  
  
    cv::floodFill(Temp, Point(0, 0), Scalar(255));  
  
    Mat cutImg;//cut expending image 
    Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);  
  
    dstBw = srcBw | (~cutImg);  
}  

//轮廓按照面积大小升序排序
bool ascendSort(vector<Point> a, vector<Point> b) {
	return a.size() < b.size();
 
}
 
//轮廓按照面积大小降序排序
bool descendSort(vector<Point> a, vector<Point> b) {
	return a.size() > b.size();
}

void DelSmallAera(const Mat srcBw, Mat &dstBw, int aera)
{
    cv::Mat srcImage = srcBw;
	cv::Mat thresholdImage;
	thresholdImage = srcImage.clone();
	//cv::threshold(srcImage, thresholdImage, 0, 255, CV_THRESH_OTSU + CV_THRESH_BINARY);
	for(int i=0;i<srcImage.rows;i++)
	{
		for(int j=0;j<srcImage.cols;j++)
			{
				if((int)srcImage.ptr<uchar>(i)[j]==15)
				{
					thresholdImage.ptr<uchar>(i)[j] = 0;
				}
					
				else
				{
					thresholdImage.ptr<uchar>(i)[j] = 255;
				}
					
			}
	}
	cv::Mat element = getStructuringElement(MORPH_ELLIPSE, Size(30, 30));  
    cv::erode(thresholdImage,thresholdImage,element);
	//Mat resultImage;
	//thresholdImage.copyTo(resultImage);
	vector< vector< cv::Point> > contours;  //Saving the information of contours.
	vector< vector< cv::Point> > contours2; //Saving the contours that the area is less than 100.
	vector<cv::Point> tempV;				//Temporary contours.
 
	cv::findContours(thresholdImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	//cv::Mat labels;
	//int N = connectedComponents(resultImage, labels, 8, CV_16U);
	//findContours(labels, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
 
	//The contours is in ascending order of area.
	sort(contours.begin(), contours.end(), ascendSort);
	vector<vector<Point> >::iterator itc = contours.begin();
	int i = 0;
	while (itc != contours.end())
	{
		//Get the rectangular boundary of the contours.
		Rect rect = boundingRect(*itc);
		int x = rect.x;
		int y = rect.y;
		int w = rect.width;
		int h = rect.height;
		if (itc->size() < aera)
		{
			//Put the contours that the area is less than 100 to "contours2".
			tempV.push_back(Point(x-1, y-1));
			tempV.push_back(Point(x-1, y+h+1));
			tempV.push_back(Point(x+w+1, y+h+1));
			tempV.push_back(Point(x+w+1, y-1));
			contours2.push_back(tempV);
			//contours2.push_back(*itc);
 
			//Delate the contours that the area is less than 100.
			cv::drawContours(srcImage, contours2, -1, Scalar(0), CV_FILLED);
		}
		tempV.clear();
		++itc;
	}
	dstBw =  srcImage;

}


bool check_label(const int label)
{
	// return (label == 2 || label == 9 || label == 15 || label == 20);
	return ( label == 15 );
}


void extend_by_depth(const cv::Mat& _depthImg, cv::Mat& _lableImg, const double depth_thread, const int hight_limit)
{
	if (_lableImg.empty() ||
		_lableImg.type() != CV_8UC1)
	{
		return ;
	}
 
	int rows = _lableImg.rows - 1 ;
	int cols = _lableImg.cols - 1 ;
	cv::Mat mask (_lableImg.size(),_lableImg.type(),cv::Scalar(0));

	for (int i = 1; i < rows-1-hight_limit; i++)
	{
		for (int j = 1; j < cols-1; j++)
		{	
			int data = (int)_lableImg.ptr<uchar>(i)[j];			//get the pixel's label
			if ( check_label(data) && (int)mask.ptr<uchar>(i)[j] == 0)	//check whether the label is 15 (representing person) and the pixel is under process
			{	
				float central_depth = (float)_depthImg.ptr<float>(i)[j];	//get the pixel's depth
				if (central_depth < 0.2 || central_depth > 8.0)				//remove uncertain data
					continue;
				std::stack<std::pair<int,int>> neighborPixels ;   
				neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>
				mask.ptr<uchar>(i)[j] = 1;							//the pixel was processed
				
				while (!neighborPixels.empty())						//depth first search
				{
					// cout << "There are " << neighborPixels.size() << " points wait to process!" << endl;
					// get the top pixel on the stack and label it with the same label
					std::pair<int,int> curPixel = neighborPixels.top() ;			//get the first pixel in the stack
					int curX = curPixel.first ;
					int curY = curPixel.second ;

					// get 4-neighbors depth
					float left_depth = (float)_depthImg.ptr<float>(curX)[curY-1];
					float right_depth = (float)_depthImg.ptr<float>(curX)[curY+1];
					float up_depth = (float)_depthImg.ptr<float>(curX-1)[curY];
					float down_depth = (float)_depthImg.ptr<float>(curX+1)[curY];
 
					// pop the top pixel
					neighborPixels.pop() ;
 
					// push the 4-neighbors (foreground pixels)
					//condition 0: pixel is under process
					//condition 1: pixel located in the limit area
					//condition 2: pixel's depth are proximate to the central_depth
					//condition 3: pixel's label is different from the start point
					if ( (int) mask.ptr<uchar>(curX)[curY-1] != 1 && curY-1 > hight_limit && (fabs(left_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX)[curY-1] != data )
					{// left pixel
						_lableImg.ptr<uchar>(curX)[curY-1] = data;
						neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
						mask.ptr<uchar>(curX)[curY-1] = 1;
					}
					if ( (int) mask.ptr<uchar>(curX)[curY+1] != 1 && curY+1 < cols-hight_limit && (fabs(right_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX)[curY+1] != data )
					{// right pixel
						_lableImg.ptr<uchar>(curX)[curY+1] = data;
						neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;
						mask.ptr<uchar>(curX)[curY+1] = 1;
					}
					if ( (int) mask.ptr<uchar>(curX-1)[curY] != 1 && curX-1 > hight_limit && (fabs(up_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX-1)[curY] != data )
					{// up pixel
						_lableImg.ptr<uchar>(curX-1)[curY] = data;
						neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;
						mask.ptr<uchar>(curX-1)[curY] = 1;
					}
					if ( (int) mask.ptr<uchar>(curX+1)[curY] != 1 && curX+1 < rows-hight_limit && (fabs(down_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX+1)[curY] != data )
					{// down pixel
						_lableImg.ptr<uchar>(curX+1)[curY] = data;
						neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;
						mask.ptr<uchar>(curX+1)[curY] = 1;
					}

				}		
			}
		}
	}
}

/////////////////1
void extend_by_depth_ROI(const cv::Mat& _depthImg, cv::Mat& _lableImg, const double depth_thread, const int hight_limit)
{
	if (_lableImg.empty() ||
		_lableImg.type() != CV_8UC1)
	{
		return ;
	}
 
	int rows = _lableImg.rows - 1 ;
	int cols = _lableImg.cols - 1 ;
	cv::Mat mask (_lableImg.size(),_lableImg.type(),cv::Scalar(0));

	for (int i = 1; i < rows-1-hight_limit; i++)
	{
		for (int j = 1; j < cols-1; j++)
		{	
			int data = (int)_lableImg.ptr<uchar>(i)[j];			//Get the pixel's label
			
			if ( check_label(data) && (int)mask.ptr<uchar>(i)[j] == 0 )	//Check whether the label is 15 (representing person) and the pixel is under process
			{	
				float central_depth = (float)_depthImg.ptr<float>(i)[j];	//Get the pixel's depth
				if (central_depth < 0.2 || central_depth > 8.0)				//Remove uncertain data
					continue;
				std::stack<std::pair<int,int>> neighborPixels ;   
				neighborPixels.push(std::pair<int,int>(i,j)) ;     // Pixel position: <i,j>
				mask.ptr<uchar>(i)[j] = 1;							//The pixel was processed
				
				while (!neighborPixels.empty())						//Depth first search
				{
					// cout << "There are " << neighborPixels.size() << " points wait to process!" << endl;
					// Get the top pixel on the stack and label it with the same label
					std::pair<int,int> curPixel = neighborPixels.top() ;			//Get the first pixel in the stack
					int curX = curPixel.first ;
					int curY = curPixel.second ;

					// Get 4-neighbors depth
					float left_depth = (float)_depthImg.ptr<float>(curX)[curY-1];
					float right_depth = (float)_depthImg.ptr<float>(curX)[curY+1];
					float up_depth = (float)_depthImg.ptr<float>(curX-1)[curY];
					float down_depth = (float)_depthImg.ptr<float>(curX+1)[curY];
 
					// Pop the top pixel
					neighborPixels.pop() ;
 
					// Push the 4-neighbors (foreground pixels)
					//Condition 0: pixel is under process
					//Condition 1: pixel located in the limit area
					//Condition 2: pixel's depth are proximate to the central_depth
					//Condition 3: pixel's label is different from the start point
					if ( (int) mask.ptr<uchar>(curX)[curY-1] != 1 && curY-1 > hight_limit && (fabs(left_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX)[curY-1] != data )
					{// left pixel
						_lableImg.ptr<uchar>(curX)[curY-1] = data;
						neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
						mask.ptr<uchar>(curX)[curY-1] = 1;
					}
					if ( (int) mask.ptr<uchar>(curX)[curY+1] != 1 && curY+1 < cols-hight_limit && (fabs(right_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX)[curY+1] != data )
					{// right pixel
						_lableImg.ptr<uchar>(curX)[curY+1] = data;
						neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;
						mask.ptr<uchar>(curX)[curY+1] = 1;
					}
					if ( (int) mask.ptr<uchar>(curX-1)[curY] != 1 && curX-1 > hight_limit && (fabs(up_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX-1)[curY] != data )
					{// up pixel
						_lableImg.ptr<uchar>(curX-1)[curY] = data;
						neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;
						mask.ptr<uchar>(curX-1)[curY] = 1;
					}
					if ( (int) mask.ptr<uchar>(curX+1)[curY] != 1 && curX+1 < rows-hight_limit && (fabs(down_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX+1)[curY] != data )
					{// down pixel
						_lableImg.ptr<uchar>(curX+1)[curY] = data;
						neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;
						mask.ptr<uchar>(curX+1)[curY] = 1;
					}

				}		
			}
		}
	}
}

//////////////////////////2
void extend_if_notedge(const cv::Mat& color_edge_threshold, const cv::Mat& _depthImg, cv::Mat& _lableImg, const double depth_thread, const int hight_limit)
{
	if (_lableImg.empty() || _lableImg.type() != CV_8UC1 || color_edge_threshold.empty() )
	{
		return ;
	}
 
	int rows = _lableImg.rows - 1 ;
	int cols = _lableImg.cols - 1 ;
	cv::Mat mask (_lableImg.size(),_lableImg.type(),cv::Scalar(0));

	for (int i = 1; i < rows-1-hight_limit; i++)
	{
		for (int j = 1; j < cols-1; j++)
		{	
			int data = (int)_lableImg.ptr<uchar>(i)[j];			//get the pixel's label
			if ( check_label(data) && (int)mask.ptr<uchar>(i)[j] == 0)	//check whether the label is 15 (representing person) and the pixel is under process
			{	
				float central_depth = (float)_depthImg.ptr<float>(i)[j];	//get the pixel's depth
				if (central_depth < 0.2 || central_depth > 8.0 || color_edge_threshold.ptr<uchar>(i)[j] == 255)				//remove uncertain data
					continue;
				std::stack<std::pair<int,int>> neighborPixels ;   
				neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>
				mask.ptr<uchar>(i)[j] = 1;							//the pixel was processed
				
				while (!neighborPixels.empty())						//depth first search
				{
					// cout << "There are " << neighborPixels.size() << " points wait to process!" << endl;
					// get the top pixel on the stack and label it with the same label
					std::pair<int,int> curPixel = neighborPixels.top() ;			//get the first pixel in the stack
					int curX = curPixel.first ;
					int curY = curPixel.second ;

					// get 4-neighbors depth
					float left_depth = (float)_depthImg.ptr<float>(curX)[curY-1];
					float right_depth = (float)_depthImg.ptr<float>(curX)[curY+1];
					float up_depth = (float)_depthImg.ptr<float>(curX-1)[curY];
					float down_depth = (float)_depthImg.ptr<float>(curX+1)[curY];
 
					// pop the top pixel
					neighborPixels.pop() ;
 
					// push the 4-neighbors (foreground pixels)
					//condition 0: pixel is under process
					//condition 1: pixel located in the limit area
					//condition 2: pixel's depth are proximate to the central_depth
					//condition 3: pixel's label is different from the start point
					if ( (int) mask.ptr<uchar>(curX)[curY-1] != 1 && curY-1 > hight_limit && (fabs(left_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX)[curY-1] != data )
					{// left pixel
						_lableImg.ptr<uchar>(curX)[curY-1] = data;
						neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
						mask.ptr<uchar>(curX)[curY-1] = 1;
						//_lableImg.ptr<uchar>(curX)[curY-1] = 15;
					}
					if ( (int) mask.ptr<uchar>(curX)[curY+1] != 1 && curY+1 < cols-hight_limit && (fabs(right_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX)[curY+1] != data )
					{// right pixel
						_lableImg.ptr<uchar>(curX)[curY+1] = data;
						neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;
						mask.ptr<uchar>(curX)[curY+1] = 1;
						//_lableImg.ptr<uchar>(curX)[curY+1] = 15;
					}
					if ( (int) mask.ptr<uchar>(curX-1)[curY] != 1 && curX-1 > hight_limit && (fabs(up_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX-1)[curY] != data  )
					{// up pixel
						_lableImg.ptr<uchar>(curX-1)[curY] = data;
						neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;
						mask.ptr<uchar>(curX-1)[curY] = 1;
						//_lableImg.ptr<uchar>(curX-1)[curY] = 15;
					}
					if ( (int) mask.ptr<uchar>(curX+1)[curY] != 1 && curX+1 < rows-hight_limit && (fabs(down_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX+1)[curY] != data)
					{// down pixel
						_lableImg.ptr<uchar>(curX+1)[curY] = data;
						neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;
						mask.ptr<uchar>(curX+1)[curY] = 1;
						//_lableImg.ptr<uchar>(curX+1)[curY] = 15;
					}

				}		
			}
		}
	}
}



void segmentPointCloudByKmeans ( const vector< Eigen::Vector2d >& pts2d, const vector< Eigen::Vector3d >& pts3d, const int n_clusters, vector< vector< Eigen::Vector2d > >& clusters_2d, vector< vector< Eigen::Vector3d > >& clusters_3d )
{
    // Convert
    cv::Mat points ( pts3d.size(), 3, CV_32F, cv::Scalar ( 0,0,0 ) );
    cv::Mat centers ( n_clusters, 1, points.type() );

    // Convert to opencv type
    for ( size_t i = 0; i < pts3d.size(); i ++ ) {
        const Eigen::Vector3d& ept = pts3d.at ( i );
        points.at<float> ( i, 0 ) = ept[0];
        points.at<float> ( i, 1 ) = ept[1];
        points.at<float> ( i, 2 ) = ept[2];
    } // for all points


    // Do Kmeans
    cv::Mat labels;
    cv::TermCriteria criteria = cv::TermCriteria ( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0 );
    cv::kmeans ( points, n_clusters, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers );

    // Collect clusters.
    clusters_2d.clear();
    clusters_3d.clear();

    clusters_2d.resize ( n_clusters );
    clusters_3d.resize ( n_clusters );

    for ( size_t i = 0; i < pts3d.size(); i ++ ) {
        int label_idx = labels.at<int> ( i, 0 );
        clusters_2d.at ( label_idx ).push_back ( pts2d.at ( i ) );
        clusters_3d.at ( label_idx ).push_back ( pts3d.at ( i ) );
    }

} // segmentPointCloudByKmeans

void ClustersImage ( cv::Mat& io_img, const int width, const int height, const vector< vector< Eigen::Vector2d > >& clusters_2d, const std::vector<uint8_t>& colors )
{
    io_img = cv::Mat ( height, width, CV_8UC3, cv::Scalar ( 0, 0, 0 ) );

    for ( size_t i = 0; i < clusters_2d.size(); i ++ ) {
        uint8_t r = colors.at ( i * 3 );
        uint8_t g = colors.at ( i * 3 + 1 );
        uint8_t b = colors.at ( i * 3 + 2 );

        // draw
        const std::vector<Eigen::Vector2d>& pts = clusters_2d.at ( i );
        for ( size_t j = 0; j < pts.size(); j ++ ) {
            const Eigen::Vector2d& pt = pts.at ( j );
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [0] = r;
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [1] = g;
            io_img.at<cv::Vec3b> ( pt[1], pt[0] ) [2] = b;
        }
    }
	
} // drawClustersOnImages

void extend_by_kmeans(const vector< vector< Eigen::Vector2d > >& clusters_2d, cv::Mat& _lableImg, const int cluster_min)
{
	for ( size_t i = 0; i < clusters_2d.size(); i ++ )
	{
		const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );
		int cluster_size = cluster_2d.size();
		if( cluster_size < cluster_min)
			continue;
		else
		{
			int label = find_label_in_cluster(cluster_2d,_lableImg);
			if(label == 15)
				for(int i = 0;i < cluster_size ; i++)
				{
					int x = cluster_2d.at(i).x();
					int y = cluster_2d.at(i).y();
					if(label != (int)_lableImg.ptr<uchar>(x)[y])
						_lableImg.ptr<uchar>(x)[y] = label;
				}
			
		}
	}
}

int find_label_in_cluster(const std::vector<Eigen::Vector2d>& cluster_2d, cv::Mat& _lableImg)
{
	srand((int)time(0));
	int count[5] ={0};			//count[*] = the number of  2,9,15,20,0 label in the cluster
	int cluster_size = cluster_2d.size();
	for(int i = 0;i < (int) ( cluster_size * 0.5 ); i++ )
		{
			int index = rand()%cluster_size;
			Eigen::Vector2d pixe = cluster_2d.at(index);
			int x = pixe.x();
			int y = pixe.y();
			// if ((int)_lableImg.ptr<uchar>(x)[y] == 2 )
			// 	count[0]++;
			// else
			// {
			// 	if ((int)_lableImg.ptr<uchar>(x)[y] == 9 )
			// 	count[1]++;
			// }
			switch ((int)_lableImg.ptr<uchar>(x)[y])
			{
				case 2:		count[0]++;	break;
				case 9: 	count[1]++;	break;
				case 15: 	count[2]++;	break;
				case 20: 	count[3]++;	break;
				default:	count[4]++;	break;
			}
			
		}
	int label_index = distance(count, max_element(count, count + sizeof(count)/sizeof(count[0])));
	if(label_index == 0)
		return 2;
	else if (label_index == 1)
		return 9;
	else if (label_index == 2)
		return 15;
	else if (label_index == 3)
		return 20;
	else return 0;
	
}


void generate_kmeans_img(const vector< vector< Eigen::Vector2d > >& clusters_2d, cv::Mat& kmeans_img, const int cluster_min)
{
	for ( size_t i = 0; i < clusters_2d.size(); i ++ )
	{
		const std::vector<Eigen::Vector2d>& cluster_2d = clusters_2d.at ( i );
		int cluster_size = cluster_2d.size();
		cout << "i = " << i << "size = " << cluster_size << endl;
		if( cluster_size < cluster_min)
			continue;
		else
		{
			for(int j = 0;j < cluster_size ; j++)
				{
					int x = cluster_2d.at(j).x();
					int y = cluster_2d.at(j).y();
					kmeans_img.ptr<uchar>(x)[y] = i;
				}
			
		}
	}
}

void extend_by_depth_and_kmeans(const cv::Mat& _depthImg, const cv::Mat& kmeans_img, cv::Mat& _lableImg, const double depth_thread, const int hight_limit)
{
	if (_lableImg.empty() ||
		_lableImg.type() != CV_8UC1)
	{
		return ;
	}
 
	int rows = _lableImg.rows - 1 ;
	int cols = _lableImg.cols - 1 ;
	cv::Mat mask (_lableImg.size(),_lableImg.type(),cv::Scalar(0));

	for (int i = 1; i < rows-1-hight_limit; i++)
	{
		for (int j = 1; j < cols-1; j++)
		{	
			int data = (int)_lableImg.ptr<uchar>(i)[j];			//get the pixel's label
			if ( check_label(data) && (int)mask.ptr<int>(i)[j] == 0)	//check whether the label is 15 (representing person) and the pixel is under process
			{	
				float central_depth = (float)_depthImg.ptr<float>(i)[j];	//get the pixel's depth
				if (central_depth < 0.2 || central_depth > 8.0)				//remove uncertain data
					continue;
				std::stack<std::pair<int,int>> neighborPixels ;   
				neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>
				mask.ptr<int>(i)[j] = 1;							//the pixel was processed
				
				while (!neighborPixels.empty())						//depth first search
				{
					// cout << "There are " << neighborPixels.size() << " points wait to process!" << endl;
					// get the top pixel on the stack and label it with the same label
					std::pair<int,int> curPixel = neighborPixels.top() ;			//get the first pixel in the stack
					int curX = curPixel.first ;
					int curY = curPixel.second ;

					// get 4-neighbors depth
					float left_depth = (float)_depthImg.ptr<float>(curX)[curY-1];
					float right_depth = (float)_depthImg.ptr<float>(curX)[curY+1];
					float up_depth = (float)_depthImg.ptr<float>(curX-1)[curY];
					float down_depth = (float)_depthImg.ptr<float>(curX+1)[curY];
					
					int central_kmeans_label = (int)kmeans_img.ptr<uchar>(curX)[curY];
					int left_kmeans_label = (int)kmeans_img.ptr<uchar>(curX)[curY-1];
					int right_kmeans_label = (int)kmeans_img.ptr<uchar>(curX)[curY+1];
					int up_kmeans_label = (int)kmeans_img.ptr<uchar>(curX-1)[curY];
					int down_kmeans_label = (int)kmeans_img.ptr<uchar>(curX+1)[curY];

					// pop the top pixel
					neighborPixels.pop() ;
 
					// push the 4-neighbors (foreground pixels)
					//condition 0: pixel is under process
					//condition 1: pixel located in the limit area
					//condition 2: pixel's depth are proximate to the central_depth
					//condition 3: pixel's label is different from the start point
					if (left_kmeans_label == central_kmeans_label || (int) mask.ptr<uchar>(curX)[curY-1] != 1 && curY-1 > hight_limit && (fabs(left_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX)[curY-1] != data )
					{// left pixel
						_lableImg.ptr<uchar>(curX)[curY-1] = data;
						neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
						mask.ptr<uchar>(curX)[curY-1] = 1;
					}
					if ( right_kmeans_label == central_kmeans_label || (int) mask.ptr<uchar>(curX)[curY+1] != 1 && curY+1 < cols-hight_limit && (fabs(right_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX)[curY+1] != data )
					{// right pixel
						_lableImg.ptr<uchar>(curX)[curY+1] = data;
						neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;
						mask.ptr<uchar>(curX)[curY+1] = 1;
					}
					if ( up_kmeans_label == central_kmeans_label || (int) mask.ptr<uchar>(curX-1)[curY] != 1 && curX-1 > hight_limit && (fabs(up_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX-1)[curY] != data )
					{// up pixel
						_lableImg.ptr<uchar>(curX-1)[curY] = data;
						neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;
						mask.ptr<uchar>(curX-1)[curY] = 1;
					}
					if ( down_kmeans_label == central_kmeans_label || (int) mask.ptr<uchar>(curX+1)[curY] != 1 && curX+1 < rows-hight_limit && (fabs(down_depth - central_depth) < depth_thread) && (int)_lableImg.ptr<uchar>(curX+1)[curY] != data )
					{// down pixel
						_lableImg.ptr<uchar>(curX+1)[curY] = data;
						neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;
						mask.ptr<uchar>(curX+1)[curY] = 1;
					}

				}		
			}
		}
	}
}
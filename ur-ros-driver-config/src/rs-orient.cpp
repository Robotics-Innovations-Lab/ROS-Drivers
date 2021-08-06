#include <opencv2/opencv.hpp>

#include <opencv2/calib3d/calib3d.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <stdio.h>

#include <librealsense2/rs.hpp>
#include <ros/ros.h>

#include <librealsense2/rsutil.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include <iostream>


using namespace cv;
using namespace std;

float orientation(Mat frame);
const int max_value_H = 360/2;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
//int high_H = max_value_H, high_S = max_value, high_V = max_value;
int high_H = 1000, high_S = max_value, high_V = max_value;
const auto window_name_final = "Display Final Image";
const auto window_name_original = "Display Original Image";
//const auto window_name_mask = "Display Mask Image";
//const auto window_name_edges = "Display Edges Image";
const auto window_capture_name = "Image";
//const auto window_detection_name = "Object Detection";
const auto window_screws_det="Final";
const auto window_edges="Edges";
const auto window_name_depth="Depth Window";
int flag_blue_button=0;
struct str{
    bool operator() ( Point a, Point b ){
        if ( a.y != b.y )
            return a.y < b.y;
        return a.x <= b.x ;
    }
} comp;

float key_position(Mat frame);
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
/*     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
          cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }*/
}


static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_name_depth, low_H);
}

static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_name_depth, high_H);
}
/*
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}
*/



// Define two align objects. One will be used to align
// to depth viewport and the other to color.
// Creating align object is an expensive operation
// that should not be performed in the main loop
rs2::align align_to_depth(RS2_STREAM_DEPTH);
rs2::align align_to_color(RS2_STREAM_COLOR);
rs2::colorizer c;                     // Helper to colorize depth images




int main(int argc, char **argv)
{
VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, Size(1920,1080));

rs2::decimation_filter dec;
// If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
// but you can also increase the following parameter to decimate depth more (reducing quality)
dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
// Define transformations from and to Disparity domain
rs2::disparity_transform depth2disparity;
rs2::disparity_transform disparity2depth(false);
// Define spatial filter (edge-preserving)
rs2::spatial_filter spat;
// Enable hole-filling
// Hole filling is an agressive heuristic and it gets the depth wrong many times
// However, this demo is not built to handle holes
// (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
// Define temporal filter
rs2::temporal_filter temp;


namedWindow(window_name_final, WINDOW_NORMAL);
//namedWindow(window_name_mask, WINDOW_NORMAL);
//namedWindow(window_name_edges, WINDOW_NORMAL);
namedWindow(window_name_original, WINDOW_NORMAL);
namedWindow(window_screws_det, WINDOW_NORMAL);
namedWindow(window_edges, WINDOW_NORMAL);
namedWindow(window_name_depth,WINDOW_NORMAL);
namedWindow("My Window", 1);
namedWindow("closed edges",WINDOW_NORMAL);

//namedWindow(window_detection_name, WINDOW_AUTOSIZE);
  ros::init(argc, argv,"talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("geometry_pose", 4);
  ros::Rate loop_rate(1000);



  int lowThreshold = 10;
  const int max_lowThreshold = 80;
  const int ratio = 3;
  const int kernel_size = 3;

  Mat kernel = Mat::ones(5,5, CV_32F); 
  cv::Mat frame, hsv, final1, mask, edges, gray, mask_filtered,cl_edge;
  std::vector<Vec3f>  circles;
  float button_coordinates[3];
  float button_pixel[2];
  
  rs2::pipeline pipe;
  rs2::config cfg;
    
  cfg.enable_stream(RS2_STREAM_DEPTH);
  cfg.enable_stream(RS2_STREAM_COLOR,RS2_FORMAT_BGR8);
    
  rs2::pipeline_profile profile=pipe.start(cfg);
  rs2::device dev = profile.get_device();
  rs2::depth_sensor ds = dev.query_sensors().front().as<rs2::depth_sensor>();
  float scale = ds.get_depth_scale();
  

  

   
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_name_depth, &low_H, 1000, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_name_depth, &high_H, 1000, on_high_H_thresh_trackbar);
    /*
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    */

  while(getWindowProperty(window_name_final, WND_PROP_AUTOSIZE) >= 0)
  {
	


  	rs2::frameset frameset = pipe.wait_for_frames();
 
  	frameset = align_to_color.process(frameset);
	// To make sure far-away objects are filtered proportionally
	// we try to switch to disparity domain
	frameset = frameset.apply_filter(depth2disparity);

	// Apply spatial filtering
	frameset = frameset.apply_filter(spat);

	// Apply temporal filtering
	frameset = frameset.apply_filter(temp);

	// If we are in disparity domain, switch back to depth
	frameset = frameset.apply_filter(disparity2depth);

  	auto color = frameset.get_color_frame(); 

	auto depth = frameset.get_depth_frame();
	
        auto colorized_depth = c.colorize(depth);

  	const int w = color.as<rs2::video_frame>().get_width();

  	const int h = color.as<rs2::video_frame>().get_height();

  	const int w_d = depth.as<rs2::video_frame>().get_width();

  	const int h_d = depth.as<rs2::video_frame>().get_height();
	


  	Mat frame(Size(w, h), CV_8UC3, (void*)color.get_data());
  	Mat depth_frame(Size(w_d, h_d), CV_8UC3, (void*)colorized_depth.get_data());

	imshow(window_name_original,frame);
	//imshow(window_name_depth,depth_frame);

	
	//uint16_t* data = (uint16_t*)frameset.get_data();
	//uint16_t pixel = data[0];
	//float meters = pixel * scale;
  	//Mat depth_metres(Size(w_d, h_d), CV_8UC3, (void*)depth.get_data());
	//Mat meters = depth_metres * scale;


        auto pf = depth.get_profile().as<rs2::video_stream_profile>();

        cv::Mat depth_metric_fp;
        cv::Mat depth_raw = cv::Mat(pf.height(), pf.width(), CV_16SC1, const_cast<void*>(depth.get_data()));
        depth_raw.convertTo(depth_metric_fp, CV_32FC1);
        depth_metric_fp *= scale*1000;

	cv::Mat edge_dist;

	cv::Mat depth_gray,depth_final,blt,cl_edge,dpt_edge;
	//Initialize m
	double minVal; 
	double maxVal; 
	Point minLoc; 
	Point maxLoc;
	video.write(frame);

	minMaxLoc( depth_metric_fp, &minVal, &maxVal, &minLoc, &maxLoc );
        //depth_gray =(depth_metric_fp/maxVal)*1000;
	//depth_gray=depth_metric_fp;
	//depth_gray.convertTo(depth_final, CV_8U);
	

	cv::inRange(depth_metric_fp,Scalar(low_H), Scalar(high_H), dpt_edge);
	
	
	//bilateralFilter(depth_final, blt, 15, 75, 75);  

  	//Canny( blt, dpt_edge, lowThreshold, lowThreshold*ratio, kernel_size );
  	//morphologyEx( dpt_edge, cl_edge, MORPH_CLOSE, kernel);

  	//Canny( depth_final, edge_dist, 100, 100*ratio, kernel_size );
	
	imshow(window_name_depth,dpt_edge);
	

	 //std::vector<cv::String> images;
         // Path of the folder containing checkerboard images

         // std::string path = "./images/*.png";

  	//frame = cv::imread(images[0]);

  	final1 = frame.clone();
  	cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  	cvtColor(frame, gray, COLOR_BGR2GRAY);
	
  
  	//cv::inRange(hsv, cv::Scalar(75, 50, 50),cv::Scalar(130, 255, 255), mask);
	//cv::inRange(hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), mask);
	cv::inRange(hsv, Scalar(94, 114, 135), Scalar(117,235, 255), mask);
  	morphologyEx( mask, mask_filtered, MORPH_OPEN, kernel);
  	Canny( mask, edges, lowThreshold, lowThreshold*ratio, kernel_size );
        //edges_filtered = edges.clone();
	//  cv::bitwise_and( hsv, frame, final1);

	// Apply Hough Transform

  	HoughCircles(edges, circles, HOUGH_GRADIENT, 1, edges.rows/16, 100, 10, 5, 25);

	// Draw detected circles
	Point center;

  	for(size_t i=0; i<circles.size(); i++) {

    		center=Point(cvRound(circles[i][0]), cvRound(circles[i][1]));

    		int radius = cvRound(circles[i][2]);

    		circle(final1, center, radius, Scalar(255, 255, 255), 2, 8, 0);

  		}

  	//std::cout<<"\nPoint x: "<<center.x<<" y "<<center.y<<"\n";
  	//imshow(window_name_mask, mask);
  	//imshow(window_name_edges, edges);
  	imshow(window_name_final,final1);
        //imshow(window_detection_name, mask);
  	//cv::imshow("mask_filtered", mask_filtered);

	//After getting pixel points need to coonvert get the coordinates from the points


	button_pixel[0] = static_cast<float>(center.x);

	button_pixel[1] = static_cast<float>(center.y);

	

	rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
	int key=cv::waitKey(30) & 0xFF;
	if(key == int('c'))
	{
		auto dist = depth.get_distance(static_cast<int>(button_pixel[0]), static_cast<int>(button_pixel[1]));
		rs2_deproject_pixel_to_point(button_coordinates, &intr, button_pixel, dist);
		cout<<"\nButton Coordinates x"<<button_coordinates[0]<<" y "<<button_coordinates[1]<<" z "<<button_coordinates[2]<<"\n";
			
		//float data[16]={-0.0334, -0.9940, 0.1041, 262.3557,-0.9994, 0.0342, 0.0056, 17.5711,-0.0092, -0.1038, -0.9946, 267.6186,0,0,0,1};
		//float data[16]={-0.0276, -0.9921, 0.1222, 256.2782,-0.9993, 0.0303, 0.0198, 14.8962,-0.0233, -0.1216, -0.9923, 268.9988,0,0,0,1};
		//float data[16]={-0.0275,-0.9927,0.1172,257.4999,-0.9995,0.0294,0.0147,15.3584,-0.0181,-0.1168,-0.9930,269.2863,0,0,0,1.0000};
		//float data[16]={-0.0425,-0.9953,0.0867,268.3707,-0.9988,0.0445,0.0211,31.8404,-0.0249,-0.0857,-0.9960,272.6322,0,0,0,1.0000};
		

		// Chessboard
		//float data[16]={ -0.0391 ,  -0.9936 ,   0.1063 , 260.3620,   -0.9992 ,   0.0399 ,   0.0055 ,  34.7831,   -0.0097 ,  -0.1060  , -0.9943 , 268.9369,   0   ,  0 ,  0 , 1.0000};
		
		float data[16]={-0.0747,   -0.9971,    0.0167,  295.4091,   -0.9967,    0.0752,    0.0313,   18.9754,   -0.0325,   -0.0143,   -0.9994,  266.7732,  0,  0,  0, 1.0000};
		float button_temp[4]={button_coordinates[0]*1000,button_coordinates[1]*1000,button_coordinates[2]*1000,1};


		float ori=orientation(frame);
		float val=key_position(frame);
		
		
		cv::Mat t=cv::Mat( 4,4, CV_32F, data);
		cv::Mat t_button=cv::Mat( 4,1, CV_32F, button_temp);
	        cv::Mat arr=t*t_button;
		cout<<arr<<"\n";

		
		if (ros::ok())
		{
			std_msgs::Int32MultiArray array;
			//Clear array
			array.data.clear();
			//for loop, pushing data in the size of the array
			for (int i = 0; i < 3; i++)
			{
				//assign array a random number between 0 and 255.
				//array.data.push_back(int(arr.at<float>(i,1)));
				int val=int(cvRound(arr.at<float>(i,0)));
				if(i==0)
				{;}
				else if(i==1)
				{val+=12;}
				else if(i==2)
				{val=-27;}

				array.data.push_back(val);
			}
			array.data.push_back(-27-int(cvRound(ori)));

			//Publish array
			pub.publish(array);
			//Let the world know
			ROS_INFO("I published something!");
			//Do this.
			ros::spinOnce();
			//Added a delay so not to spam
			sleep(2);
		}
		int key=cv::waitKey(0) & 0xFF;
		if(key)
		{
			flag_blue_button=1;
			continue;
		}
	}
	else if (key == int('q'))
	{
		std::cout<<"\nbye bye\n";
		break;
	}
	else if (key==int('i'))
	{		
		if (ros::ok())
		{
			std_msgs::Int32MultiArray array;
			//Clear array
			array.data.clear();
			//for loop, pushing data in the size of the array
			array.data.push_back(212);
			array.data.push_back(1);
			array.data.push_back(168);
			array.data.push_back(-27);

			//Publish array
			pub.publish(array);
			//Let the world know
			ROS_INFO("I published something!");
			//Do this.
			ros::spinOnce();
			//Added a delay so not to spam
			sleep(2);
		}
	}
	else if (key == int('m'))
	{
		cout<<"Matrix in m";
		//cout<<meters;
		int key=cv::waitKey(0) & 0xFF;
		if(key)
		{
			continue;
		}
	}


	//button coordinates will have the coordinates in array	
  }
  video.release();
  cv::destroyAllWindows();
  return 0;

  }





float orientation(Mat frame)
{

  	int lowThreshold = 100;
  	const int ratio = 3;
	const int kernel_size = 3;
	int ref_radius,radius=0;
	int num_of_circles=0;
	int size1_ind[2];
	int size2_ind[2];
  	Mat kernel = Mat::ones(5,5, CV_32F); 
  	cv::Mat hsv, final1, bilateral, edges, gray, closed_edges;
  	std::vector<Vec3f>  circles;
	std::vector<std::vector<Point>> circles_centers;





  	final1 = frame.clone();
  	//cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  	cvtColor(frame, gray, COLOR_BGR2GRAY);
	bilateralFilter(gray, bilateral, 15, 75, 75);  

  	Canny( bilateral, edges, lowThreshold, lowThreshold*ratio, kernel_size );
  	morphologyEx( edges, closed_edges, MORPH_CLOSE, kernel);
  	HoughCircles(edges, circles, HOUGH_GRADIENT, 1, edges.rows/16, 100, 15, 5, 20);
	
	num_of_circles=circles.size();
	std::cout<<"number of circles: "<<num_of_circles;
  	for(size_t i=0; i<circles.size(); i++) 
	{

    		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

    		radius = cvRound(circles[i][2]);
		std::cout<<"\nRadius : " <<radius;
    		circle(final1, center, radius, Scalar(255, 0, 255), 2);

  	}
	int temp[num_of_circles];
	
	for(int j=0; j<num_of_circles; j++)
		temp[j]=0;

	for(int j=0; j<num_of_circles; j++)
	{
			
		ref_radius = cvRound(circles[j][2]);
		for(int i=0; i<num_of_circles; i++)
		{
			radius=  cvRound(circles[i][2]);
			if(temp[i]==0 and i!=j)
			{
				if((radius<= ref_radius+1.5)and (radius>= ref_radius-1.5))
				{
					if(temp[j]!=0)
						temp[i]=temp[j];
					else
						temp[i] = j+1;
				}
			}
		}
	}
	std::cout<<"\nBitmap:";
	for(int j=0; j<num_of_circles; j++)
		std::cout<<temp[j];	

	int groups[4];	
	
	int flag=0;	
	int group_num=0;
  	std::vector<std::vector<int>> screws;

	for(int j=0; j<num_of_circles; j++)
	{
		if(temp[j]!=0)
			flag=1;
		std::vector<int> v;
		int k;
		for(k=0; k<group_num; k++)
		{
			if(temp[j]==groups[k])
			{	
				flag=0;
				break;
	
			}
		}
		if(flag)
		{
			groups[group_num]=temp[j];
			group_num++;
			v.push_back(j);
			screws.push_back(v);
		}
		else
		{
			screws[group_num-1].push_back(j);
	
		}	
	}
	int swap;
	std::vector<int> swap_vec;
	for(int i=0; i< group_num-1; i++)
	{
		for(int j=0; j<group_num -i -1; j++)
		{
			if(cvRound(circles[ temp[groups[j]] ][2]) > circles[ temp[groups[j + 1]] ][2])
			{
				//SWAP 	the groups array
				swap=groups[j];
				groups[j]= groups[j+1];
				groups[j+1]= swap;
				//swap the vectors storing screws of different sizes to arrange it in the order of its radii
				
				swap_vec = screws[j];
				screws[j] = screws[j+1];
				screws[j+1] = swap_vec;

			}
		}
	}
	for (int i = 0; i < screws.size(); i++)
	{
		std::cout << "\n";
		
		std::vector<Point> v;
        	for (int j = 0; j < screws[i].size(); j++)
            	{
    			Point center(cvRound(circles[screws[i][j]][0]), cvRound(circles[screws[i][j]][1]));
			v.push_back(center);	
			std::cout << screws[i][j] << " ";
    	
		}

		sort(v.begin(), v.end(),comp);
		circles_centers.push_back(v);
	}

	int order=1;
	int size=0;
	int thickness = 2;
	float angles[group_num];
	int cnt=0;
	for (int i = 0; i < screws.size(); i++)
	{
		std::cout << "\n";
		size=circles_centers[i].size();
		std::cout<<" size : "<<size<<" ";	
        	for (int j = 0; j < size; j++)
            	{
			std::cout << circles_centers[i][j].x << " "<<circles_centers[i][j].y<< "  : ";
			cv::putText(final1, std::to_string(order), Point(circles_centers[i][j].x +1, circles_centers[i][j].y+2), 
            				cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
			order++;
		}
		
		if(size>1)
		{

			
			line(final1, circles_centers[i][0], circles_centers[i][size-1], Scalar(255, 0, 0), thickness, LINE_8);
			angles[cnt]=atan2(circles_centers[i][0].y - circles_centers[i][size-1].y, 
					circles_centers[i][0].x - circles_centers[i][size-1].x);

			std::cout<<"\nangles = "<<angles[cnt] <<"   ";
			cnt++;
		}
	}
	float horizontal_angle= angles[cnt-1]*180/CV_PI;
	float vertical_angle = angles[0]*180/CV_PI;


	for(int i=0;i <cnt; i++)
		std::cout<<"angle in degrees"<<angles[i]*180/CV_PI<<"\n";

	float angle=horizontal_angle;
	

	if(angle<-90)
	{
	   angle=180+angle;
	}
	else if(angle>90)
	{
	   angle=180-angle;
	}
	
	//std::cout<<"\n"<<groups[0]<<groups[1];
	imshow(window_screws_det, final1);
  	imshow(window_edges, closed_edges);
	std::cout<<"angle"<<angle;

  	return angle;
}


float key_position(Mat frame)
{

  	int lowThreshold = 80;
  	const int ratio = 3;
	const int kernel_size = 3;
	int ref_radius,radius=0;
	int num_of_circles=0;
	float ori[2];
  	Mat kernel = Mat::ones(5,5, CV_32F); 
  	cv::Mat hsv, final1, bilateral, edges, gray, closed_edges;
  	std::vector<Vec3f>  circles;
	std::vector<std::vector<Point2f>> circles_centers;
	Point2f origin, key;
	float ori_position[3];
    

  	final1 = frame.clone();

  	cvtColor(frame, gray, COLOR_BGR2GRAY);
	bilateralFilter(gray, bilateral, 15, 75, 75);  

  	Canny( bilateral, edges, lowThreshold, lowThreshold*ratio, kernel_size );
  	morphologyEx( edges, closed_edges, MORPH_CLOSE, kernel);
  	HoughCircles(edges, circles, HOUGH_GRADIENT, 1, edges.rows/16, 100, 18, 5, 20);
	
	num_of_circles=circles.size();
	std::cout<<"number of circles: "<<num_of_circles;
  	for(size_t i=0; i<circles.size(); i++) 
	{

    		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

    		radius = cvRound(circles[i][2]);
		std::cout<<"\nRadius : " <<radius;
    		circle(final1, center, radius, Scalar(255, 0, 255), 2);

  	}
	int temp[num_of_circles];
	
	for(int j=0; j<num_of_circles; j++)
		temp[j]=0;

	for(int j=0; j<num_of_circles; j++)
	{
			
		ref_radius = cvRound(circles[j][2]);
		for(int i=0; i<num_of_circles; i++)
		{
			radius=  cvRound(circles[i][2]);
			if(temp[i]==0 and i!=j)
			{
				if((radius<= ref_radius+1.5)and (radius>= ref_radius-1.5))
				{
					if(temp[j]!=0)
						temp[i]=temp[j];
					else
						temp[i] = j+1;
				}
			}
		}
	}
	std::cout<<"\nBitmap:";
	for(int j=0; j<num_of_circles; j++)
		std::cout<<temp[j];	

	int groups[4];	
	
	int flag=0;	
	int group_num=0;
  	std::vector<std::vector<int>> screws;

	for(int j=0; j<num_of_circles; j++)
	{
		if(temp[j]!=0)
			flag=1;
		std::vector<int> v;
		int k;
		for(k=0; k<group_num; k++)
		{
			if(temp[j]==groups[k])
			{	
				flag=0;
				break;
	
			}
		}
		if(flag)
		{
			groups[group_num]=temp[j];
			group_num++;
			v.push_back(j);
			screws.push_back(v);
		}
		else
		{
			screws[group_num-1].push_back(j);
	
		}	
	}
	int swap;
	std::vector<int> swap_vec;
	for(int i=0; i< group_num-1; i++)
	{
		for(int j=0; j<group_num -i -1; j++)
		{
			if(cvRound(circles[ temp[groups[j]] ][2]) > circles[ temp[groups[j + 1]] ][2])
			{
				//SWAP 	the groups array
				swap=groups[j];
				groups[j]= groups[j+1];
				groups[j+1]= swap;
				//swap the vectors storing screws of different sizes to arrange it in the order of its radii
				
				swap_vec = screws[j];
				screws[j] = screws[j+1];
				screws[j+1] = swap_vec;

			}
		}
	}
	for (int i = 0; i < screws.size(); i++)
	{
		std::cout << "\n";
		
		std::vector<Point2f> v;
        	for (int j = 0; j < screws[i].size(); j++)
            	{
    			Point center(cvRound(circles[screws[i][j]][0]), cvRound(circles[screws[i][j]][1]));
			v.push_back(center);	
			std::cout << screws[i][j] << " ";
    	
		}

		sort(v.begin(), v.end(),comp);
		circles_centers.push_back(v);
	}

	int order=1;
	int size=0;
	int thickness = 2;
	float angles[group_num];
	Point2f p1, p2, p3, p4, tmp;
	int cnt=0;


	for (int i = 0; i < screws.size(); i++)
	{
		std::cout << "\n";
		size=circles_centers[i].size();
		std::cout<<" size : "<<size<<" ";	
        	for (int j = 0; j < size; j++)
            	{
			tmp=Point( circles_centers[i][j].x,  circles_centers[i][j].y  );
			std::cout << circles_centers[i][j].x << " "<<circles_centers[i][j].y<< "  : ";
			cv::putText(final1, std::to_string(order), Point(circles_centers[i][j].x +1, circles_centers[i][j].y+2), 
            				cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0), 2);
			if(j==0 and i==0)
				p1=tmp;
			if(j==1 and i==0)
				p2=tmp;
			if(j==0 and i==((screws.size())-1))
				p3=tmp;
			if(j==1 and i==((screws.size())-1))
				p4=tmp;
			order++;
		}
		
		if(size>1)
		{

			
			line(final1, circles_centers[i][0], circles_centers[i][size-1], Scalar(255, 0, 0), thickness, LINE_8);
			angles[cnt]=atan2(circles_centers[i][0].y - circles_centers[i][size-1].y, 
					circles_centers[i][0].x - circles_centers[i][size-1].x);

			std::cout<<"\nangles = "<<angles[cnt] <<"   ";
			cnt++;
		}
	}
	float horizontal_angle= angles[cnt-1]*180/CV_PI;
	float vertical_angle = angles[0]*180/CV_PI;
	
	float m1,m2, c1, c2, x0, y0;
   	Point2f x = p3 - p1;
    	Point2f d1 = p2 - p1;
    	Point2f d2 = p4 - p3;
	Point2f r;

	float cross = d1.x*d2.y - d1.y*d2.x;	
	if (std::abs(cross) < 1e-8)
		std::cout<<"\n Not intersecting!! \n";


	 double t1 = (x.x * d2.y - x.y * d2.x)/cross;
	 r = p1 + d1 * t1;
	origin =r;

	circle(final1, origin, 4, Scalar(0, 0, 255), 3);

	cout<<"\nORIGIN: x"<<origin.x<<" y "<< origin.y<<"\n";
	
	//Point2f rect1 = ()
	for(int i=0;i <cnt; i++)
		std::cout<<"angle in degrees"<<angles[i]*180/CV_PI<<"\n";

	
	setMouseCallback("My Window", CallBackFunc, NULL);

	//key=Point2f(1492.0, 708.0);
	//float key_angle = atan2((key.y - origin.y),(key.x - origin.x));
	//float dist_key = sqrt(pow(key.x - origin.x, 2) + pow(key.y - origin.y, 2));
	
	
	float prev_orientation = -6.76617;
	float key_angle =0.13334*180/CV_PI;

	//angle b/w key and horizontal from previous frame
	float dist_key = 617.106; 


	Point2f est_key;

	cout<<"\n current pose before if  :"<<horizontal_angle<<"\n";

	if(horizontal_angle <-90)
		horizontal_angle+= 180;

	if(horizontal_angle >90)
		horizontal_angle= 180 - horizontal_angle;


	float angle_shift = (horizontal_angle -  prev_orientation);
	cout<<"\nKey angle : "<<key_angle;
	cout<<"\n current pose :"<<horizontal_angle<<"\n";
	cout<<"\n angle shift "<<angle_shift<<"\n";

	
	est_key = Point2f(origin.x + dist_key*cos(CV_PI*(key_angle + angle_shift)/180),origin.y 
			+ dist_key*sin(CV_PI*(key_angle + angle_shift)/180));
	circle(final1, est_key, 6, Scalar(0, 255, 0), 5);

	
	cout<<"\nKey angle : "<<key_angle;
	cout<<"\nKey dist : "<<dist_key<<"\n";
	
	cv::imshow("My Window", final1);
  	cv::imshow("closed edges", closed_edges);

	
	
	return 0.0;
}

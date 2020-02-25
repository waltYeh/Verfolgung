#include "ros/ros.h"
#include "image_transport/image_transport.h"  
#include "sensor_msgs/image_encodings.h" 
#include <struck/roi_input.h>
#include <struck/track_output.h>

#include <cv_bridge/cv_bridge.h>

#include "Tracker.h"
#include "Config.h"

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

void inertial_filter_correct(float e, float dt, float *x, float w)
{
	float ewdt = e * w * dt;
	*x += ewdt;
}
void myrectangle(Mat& rMat, const FloatRect& rRect, const Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, Point(r.XMin(), r.YMin()), Point(r.XMax(), r.YMax()), rColour);
}
class StruckTrack
{
private:
	static const int kLiveBoxWidth = 100;
	static const int kLiveBoxHeight = 100;
	FloatRect initBB;
	FloatRect inputBB;
	bool statusRunning;
	bool doInitialise;
	bool paused;
	int fullwidth;
	int fullheight;
	float figurheight;
	string configPath;
	Config conf;
	Tracker tracker;
	ros::Publisher output_pub;
	ros::Subscriber input_sub;
	image_transport::ImageTransport it;
	image_transport::Subscriber sub;
//	float bb_bias[2];
	float bias_corr_w;
	float fig_dtct_w;
public:
	StruckTrack(ros::NodeHandle& nh, const string& configPath);
	~StruckTrack();
	void process(cv::Mat frameOrig);
	void inputCallback(const struck::roi_input msg)
	{
		static int cnt=0;
		cnt++;
		if(!statusRunning){
			if(msg.pose2.x - msg.pose1.x > 0 && msg.pose2.y - msg.pose1.y > 0){
				if(fullwidth!=0&&fullheight!=0){
					if(msg.pose1.y >= 0&&msg.pose1.x >= 0 && msg.pose2.x <= fullwidth){
						int pose1x_val = msg.pose1.x * fullwidth;
						int pose1y_val = msg.pose1.y * fullheight;
						int pose2x_val = msg.pose2.x * fullwidth;
						int pose2y_val = msg.pose2.y * fullheight;
						int rahm_w = pose2x_val - pose1x_val;
						int rahm_h = pose2y_val - pose1y_val;
						initBB = IntRect(pose1x_val+0.2*rahm_w, pose1y_val+0.2*rahm_h, rahm_w*0.6, rahm_h*0.6);
						//inputBB = IntRect(msg.pose1.x, msg.pose1.y, (msg.pose2.x - msg.pose1.x), (msg.pose2.y - msg.pose1.y));
						doInitialise = true;
						statusRunning = true;
					}
				}
			}
		}
		else{
			int pose1x_val = msg.pose1.x * fullwidth;
			int pose1y_val = msg.pose1.y * fullheight;
			int pose2x_val = msg.pose2.x * fullwidth;
			int pose2y_val = msg.pose2.y * fullheight;
			int rahm_w = pose2x_val - pose1x_val;
			int rahm_h = pose2y_val - pose1y_val;
			figurheight = msg.pose2.y - msg.pose1.y;
			float fig_x = pose1x_val+0.2*rahm_w;
			float fig_y = pose1y_val+0.2*rahm_h;
			float track_x = tracker.get_bb_x();
			float track_y = tracker.get_bb_y();
		//	tracker.GetBB();
			float corr_x = fig_x - track_x;
			float corr_y = fig_y - track_y;
			float bias_x = -corr_x * fig_dtct_w * fig_dtct_w;
			float bias_y = -corr_y * fig_dtct_w * fig_dtct_w;
			inertial_filter_correct(corr_x, 0.05, &track_x, fig_dtct_w); 
			inertial_filter_correct(corr_y, 0.05, &track_y, fig_dtct_w);
		//	bb_bias[0] += bias_x * 0.05 * bias_corr_w;
		//	bb_bias[1] += bias_y * 0.05 * bias_corr_w;
			tracker.CorrectBB(track_x, track_y);
			inputBB = IntRect(pose1x_val, pose1y_val, pose2x_val - pose1x_val, pose2y_val - pose1y_val);
//			myrectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
		}

	}
	void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg) 
	{ 
		cout<<"frame"<<endl; 
		cv_bridge::CvImagePtr cv_ptr;
		try{       
		     cv_ptr = cv_bridge::toCvCopy(tem_msg, enc::BGR8); 
		     cv::waitKey(1);
		}    
		catch (cv_bridge::Exception& e)    {
			ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());        
		}
	//	frameOrig = cv_ptr->image.clone();
		cout<<"frame"<<endl;
		fullwidth = cv_ptr->image.cols;
		fullheight = cv_ptr->image.rows;
		process(cv_ptr->image);
	} 

};
StruckTrack::StruckTrack(ros::NodeHandle& nh, const string& configPath)
:conf(configPath)
,tracker(conf)
,statusRunning(false)
,doInitialise(false)
,paused(false)
,it(nh)
{
	srand(conf.seed);
	output_pub = nh.advertise<struck::track_output>("track_output",5);
	input_sub = nh.subscribe("figure_roi",5,&StruckTrack::inputCallback,this);
	sub = it.subscribe("node_a",1,&StruckTrack::imageCallback,this);
//	bb_bias[0] = 0;
	bias_corr_w = 0.15f;
	fig_dtct_w = 2.0f;
//	initBB = IntRect(290, 140, 40, 60);
//	doInitialise = true;
//	statusRunning = true;
}
StruckTrack::~StruckTrack()
{

}
void StruckTrack::process(cv::Mat frame)
{
	float scaleW = 1.f;
	float scaleH = 1.f;
//	Mat frame;
//	resize(frameOrig, frame, Size(fullwidth, fullheight));
//	flip(frame, frame, 1);
	Mat result(fullheight, fullwidth, CV_8UC3);
	frame.copyTo(result);
	if (doInitialise){
		if (tracker.IsInitialised()){
			tracker.Reset();
		}
		else{
			tracker.Initialise(frame, initBB);
		}
		doInitialise = false;
	}
	else if (!tracker.IsInitialised()){
		myrectangle(result, initBB, CV_RGB(255, 255, 255));
	}
	if (tracker.IsInitialised()){
		tracker.Track(frame);
		if (!conf.quietMode && conf.debugMode)
		{
			tracker.Debug();
		}	
		myrectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
		myrectangle(result, inputBB, CV_RGB(255, 255, 255));
	}
	if (!conf.quietMode){
		//imshow("result", result);
		//int key = waitKey(paused ? 0 : 1);
	}
	struck::track_output output_msg;
	const FloatRect& bb = tracker.GetBB();
	output_msg.pose1.x = bb.XMin()/(float)fullwidth;
	output_msg.pose1.y = bb.YMin()/(float)fullheight;
	output_msg.pose2.x = (bb.XMin()+bb.Width())/(float)fullwidth;
	output_msg.pose2.y = (bb.YMin()+bb.Height())/(float)fullheight;
	output_msg.vel.x = 0;
	output_msg.vel.y = 0;
	output_msg.height = figurheight;
	output_pub.publish(output_msg);
}


int main(int argc, char* argv[])
{
	string configPath = argv[1];

//	cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
	ros::init(argc, argv, "simcontroller");
	ros::NodeHandle nh;
	StruckTrack st(nh,configPath);
	ros::Rate loop_rate(20);
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return EXIT_SUCCESS;
}



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

#include <opencv/cv.h>
#include <opencv/highgui.h>
namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;


static const int kLiveBoxWidth = 100;
static const int kLiveBoxHeight = 100;
FloatRect initBB;
FloatRect inputBB;
Mat frameOrig;
bool statusRunning = false;
bool doInitialise = false;
bool new_frame_comming = false;
void rectangle(Mat& rMat, const FloatRect& rRect, const Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, Point(r.XMin(), r.YMin()), Point(r.XMax(), r.YMax()), rColour);
}
void inputCallback(const struck::roi_input msg)
{
	if(!statusRunning){
		initBB = IntRect(msg.pose1.x, msg.pose1.y, msg.pose2.x - msg.pose1.x, msg.pose2.y - msg.pose1.y);
		inputBB = IntRect(msg.pose1.x, msg.pose1.y, msg.pose2.x - msg.pose1.x, msg.pose2.y - msg.pose1.y);
		doInitialise = true;
		statusRunning = true;;
	}
	else{
		inputBB = IntRect(msg.pose1.x, msg.pose1.y, msg.pose2.x - msg.pose1.x, msg.pose2.y - msg.pose1.y);
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg) 
{ 
  cout<<"frame"<<endl; 
  cv_bridge::CvImagePtr cv_ptr;
  try    
  {    
      /*转化成CVImage*/    
       cv_ptr = cv_bridge::toCvCopy(tem_msg, enc::BGR8); 
         
 //    cv::imshow("C_IN_WINDOW",cv_ptr->image);  
       cv::waitKey(1);
         
  }    

  catch (cv_bridge::Exception& e)    
  {    
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());        
  }

  frameOrig = cv_ptr->image.clone();
  new_frame_comming = true;
  cout<<"frame"<<endl;
} 
int main(int argc, char* argv[])
{
	string configPath = "/home/walt/catkin_ws/src/struck/config.txt";

	Config conf(configPath);
	cout << conf << endl;
	
	if (conf.features.size() == 0)
	{
		cout << "error: no features specified in config" << endl;
		return EXIT_FAILURE;
	}
	
	ofstream outFile;
	if (conf.resultsPath != "")
	{
		outFile.open(conf.resultsPath.c_str(), ios::out);
		if (!outFile)
		{
			cout << "error: could not open results file: " << conf.resultsPath << endl;
			return EXIT_FAILURE;
		}
	}
	
	int startFrame = -1;
	int endFrame = -1;
	
	string imgFormat;
	float scaleW = 1.f;
	float scaleH = 1.f;


	startFrame = 0;
	endFrame = INT_MAX;


//	initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2, conf.frameHeight/2-kLiveBoxHeight/2, kLiveBoxWidth, kLiveBoxHeight);
	Tracker tracker(conf);
//	conf.quietMode  = true;

	cv::namedWindow("result", CV_WINDOW_AUTOSIZE);
	cout<<"Window"<<endl;
	Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	bool paused = false;
	
	srand(conf.seed);


	ros::init(argc, argv, "simcontroller");
	ros::NodeHandle ntruck;
	ros::Publisher output_pub = ntruck.advertise<struck::track_output>("output",5);
	ros::Subscriber input_sub = ntruck.subscribe("input",5,inputCallback);
	image_transport::ImageTransport it(ntruck);
	
	image_transport::Subscriber sub = it.subscribe("node_a",1,imageCallback);
	int frameInd = 0;
	ros::Rate loop_rate(20);
	cout<<"Window"<<endl;
	while (ros::ok()){

		if(new_frame_comming){

			new_frame_comming = false;
			ros::Time last_timet = ros::Time::now();
			double last_time = last_timet.toSec();
			++frameInd;
			cout << "loop " <<frameInd<< endl;
	/*		if(frameInd == 100){
				cout << "now start" << endl;
				doInitialise = true;
			}else if(frameInd == 10)
				cout << "3" <<endl;
			else if(frameInd == 40)
				cout << "2" <<endl;
			else if(frameInd == 70)
				cout << "1" <<endl;*/
			Mat frame;
			resize(frameOrig, frame, Size(conf.frameWidth, conf.frameHeight));
			flip(frame, frame, 1);
			frame.copyTo(result);
			if (doInitialise)
			{
				if (tracker.IsInitialised())
				{
					tracker.Reset();
				}
				else
				{
					tracker.Initialise(frame, initBB);
				}
				doInitialise = false;
			}
			else if (!tracker.IsInitialised())
			{
				rectangle(result, initBB, CV_RGB(255, 255, 255));
			}
			if (tracker.IsInitialised())
			{
				tracker.Track(frame);
				
				if (!conf.quietMode && conf.debugMode)
				{
					tracker.Debug();
				}
				
				rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
				
				if (outFile)
				{
					const FloatRect& bb = tracker.GetBB();
					outFile << bb.XMin()/scaleW << "," << bb.YMin()/scaleH << "," << bb.Width()/scaleW << "," << bb.Height()/scaleH << endl;
				}
			}
			if (!conf.quietMode)
			{
				imshow("result", result);
				int key = waitKey(paused ? 0 : 1);
			}
			ros::Time runtime = ros::Time::now();
			double secs = runtime.toSec()-last_time;
		//	cout<<secs<<endl;
			struck::track_output output_msg;
			const FloatRect& bb = tracker.GetBB();
			output_msg.pose1.x = bb.XMin()/scaleW;
			output_msg.pose1.y = bb.YMin()/scaleH;
			output_msg.pose2.x = bb.XMin()/scaleW+bb.Width()/scaleW;
			output_msg.pose2.y = bb.YMin()/scaleH+bb.Height()/scaleH;
			output_msg.vel.x = 0;
			output_msg.vel.y = 0;
			output_pub.publish(output_msg);
			
		}
		ros::spinOnce();
			loop_rate.sleep();
		
	}
	if (outFile.is_open())
	{
		outFile.close();
	}
	
	return EXIT_SUCCESS;
}


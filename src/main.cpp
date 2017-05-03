/* 
 * Struck: Structured Output Tracking with Kernels
 * 
 * Code to accompany the paper:
 *   Struck: Structured Output Tracking with Kernels
 *   Sam Hare, Amir Saffari, Philip H. S. Torr
 *   International Conference on Computer Vision (ICCV), 2011
 * 
 * Copyright (C) 2011 Sam Hare, Oxford Brookes University, Oxford, UK
 * 
 * This file is part of Struck.
 * 
 * Struck is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Struck is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Struck.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */


#if 0
#include "ros/ros.h"

 
#include "Tracker.h"
#include "Config.h"

#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

static const int kLiveBoxWidth = 100;
static const int kLiveBoxHeight = 100;

void rectangle(Mat& rMat, const FloatRect& rRect, const Scalar& rColour)
{
	IntRect r(rRect);
	rectangle(rMat, Point(r.XMin(), r.YMin()), Point(r.XMax(), r.YMax()), rColour);
}

int main(int argc, char* argv[])
{
	// read config file
	string configPath = "config.txt";
	if (argc > 1)
	{
		configPath = argv[1];
	}
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
	
	// if no sequence specified then use the camera
	bool useCamera = (conf.sequenceName == "");
	
	VideoCapture cap;
	
	int startFrame = -1;
	int endFrame = -1;
	FloatRect initBB;
	string imgFormat;
	float scaleW = 1.f;
	float scaleH = 1.f;
	
	if (useCamera)
	{
		if (!cap.open(0))
		{
			cout << "error: could not start camera capture" << endl;
			return EXIT_FAILURE;
		}
		startFrame = 0;
		endFrame = INT_MAX;
		Mat tmp;
		cap >> tmp;
		scaleW = (float)conf.frameWidth/tmp.cols;
		scaleH = (float)conf.frameHeight/tmp.rows;

		initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2, conf.frameHeight/2-kLiveBoxHeight/2, kLiveBoxWidth, kLiveBoxHeight);
		cout << "press 'i' to initialise tracker" << endl;
	}
	else
	{
		// parse frames file
		string framesFilePath = conf.sequenceBasePath+"/"+conf.sequenceName+"/"+conf.sequenceName+"_frames.txt";
		ifstream framesFile(framesFilePath.c_str(), ios::in);
		if (!framesFile)
		{
			cout << "error: could not open sequence frames file: " << framesFilePath << endl;
			return EXIT_FAILURE;
		}
		string framesLine;
		getline(framesFile, framesLine);
		sscanf(framesLine.c_str(), "%d,%d", &startFrame, &endFrame);
		if (framesFile.fail() || startFrame == -1 || endFrame == -1)
		{
			cout << "error: could not parse sequence frames file" << endl;
			return EXIT_FAILURE;
		}
		
		imgFormat = conf.sequenceBasePath+"/"+conf.sequenceName+"/imgs/img%05d.png";
		
		// read first frame to get size
		char imgPath[256];
		sprintf(imgPath, imgFormat.c_str(), startFrame);
		Mat tmp = cv::imread(imgPath, 0);
		scaleW = (float)conf.frameWidth/tmp.cols;
		scaleH = (float)conf.frameHeight/tmp.rows;
		
		// read init box from ground truth file
		string gtFilePath = conf.sequenceBasePath+"/"+conf.sequenceName+"/"+conf.sequenceName+"_gt.txt";
		ifstream gtFile(gtFilePath.c_str(), ios::in);
		if (!gtFile)
		{
			cout << "error: could not open sequence gt file: " << gtFilePath << endl;
			return EXIT_FAILURE;
		}
		string gtLine;
		getline(gtFile, gtLine);
		float xmin = -1.f;
		float ymin = -1.f;
		float width = -1.f;
		float height = -1.f;
		sscanf(gtLine.c_str(), "%f,%f,%f,%f", &xmin, &ymin, &width, &height);
		if (gtFile.fail() || xmin < 0.f || ymin < 0.f || width < 0.f || height < 0.f)
		{
			cout << "error: could not parse sequence gt file" << endl;
			return EXIT_FAILURE;
		}
		initBB = FloatRect(xmin*scaleW, ymin*scaleH, width*scaleW, height*scaleH);
	}
	
	
	
	Tracker tracker(conf);
	if (!conf.quietMode)
	{
		namedWindow("result");
	}
	
	Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	bool paused = false;
	bool doInitialise = false;
	srand(conf.seed);
	for (int frameInd = startFrame; frameInd <= endFrame; ++frameInd)
	{
		if(frameInd == 100){
			cout << "now start" << endl;
			doInitialise = true;
		}else if(frameInd == 10)
			cout << "3" <<endl;
		else if(frameInd == 40)
			cout << "2" <<endl;
		else if(frameInd == 70)
			cout << "1" <<endl;
		Mat frame;
		if (useCamera)
		{
			Mat frameOrig;
			cap >> frameOrig;
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
		}
		else
		{			
			char imgPath[256];
			sprintf(imgPath, imgFormat.c_str(), frameInd);
			Mat frameOrig = cv::imread(imgPath, 0);
			if (frameOrig.empty())
			{
				cout << "error: could not read frame: " << imgPath << endl;
				return EXIT_FAILURE;
			}
			resize(frameOrig, frame, Size(conf.frameWidth, conf.frameHeight));
			cvtColor(frame, result, CV_GRAY2RGB);
		
			if (frameInd == startFrame)
			{
				tracker.Initialise(frame, initBB);
			}
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
			if (key != -1)
			{
				if (key == 27 || key == 113) // esc q
				{
					break;
				}
				else if (key == 112) // p
				{
					paused = !paused;
				}
				else if (key == 105 && useCamera)
				{
					doInitialise = true;
				}
			}
			if (conf.debugMode && frameInd == endFrame)
			{
				cout << "\n\nend of sequence, press any key to exit" << endl;
				waitKey();
			}
		}
	}
	
	if (outFile.is_open())
	{
		outFile.close();
	}
	
	return EXIT_SUCCESS;
}
#else

#include "ros/ros.h"
#include <struck/roi_input.h>
#include <struck/track_output.h>


#include "Tracker.h"
#include "Config.h"

#include <iostream>
#include <fstream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;


static const int kLiveBoxWidth = 100;
static const int kLiveBoxHeight = 100;
FloatRect initBB;
FloatRect inputBB;
bool statusRunning = false;
bool doInitialise = false;
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
int main(int argc, char* argv[])
{
	string configPath = "config.txt";
	if (argc > 1)
	{
		configPath = argv[1];
	}
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
	VideoCapture cap;
	
	int startFrame = -1;
	int endFrame = -1;
	
	string imgFormat;
	float scaleW = 1.f;
	float scaleH = 1.f;

	if (!cap.open(0))
	{
		cout << "error: could not start camera capture" << endl;
		return EXIT_FAILURE;
	}
	startFrame = 0;
	endFrame = INT_MAX;
	Mat tmp;
	cap >> tmp;
	scaleW = (float)conf.frameWidth/tmp.cols;
	scaleH = (float)conf.frameHeight/tmp.rows;

//	initBB = IntRect(conf.frameWidth/2-kLiveBoxWidth/2, conf.frameHeight/2-kLiveBoxHeight/2, kLiveBoxWidth, kLiveBoxHeight);
	Tracker tracker(conf);
//	conf.quietMode  = true;
	if (!conf.quietMode){
		namedWindow("result");
	}
	Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	bool paused = false;
	
	srand(conf.seed);


	ros::init(argc, argv, "simcontroller");
	ros::NodeHandle n;
	ros::Publisher output_pub = n.advertise<struck::track_output>("output",5);
	ros::Subscriber input_sub = n.subscribe("input",5,inputCallback);

	int frameInd = 0;
	ros::Rate loop_rate(20);
	
	while (ros::ok()){
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
		Mat frameOrig;
		cap >> frameOrig;
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
		ros::spinOnce();
		loop_rate.sleep();
		
	}
	if (outFile.is_open())
	{
		outFile.close();
	}
	
	return EXIT_SUCCESS;
}

#endif
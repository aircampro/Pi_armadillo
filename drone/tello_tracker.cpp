// ==================================================================
//
// Simple example of openCV tracker on video from a DJI Tello drone
// uses https://github.com/herrnamenlos123/tello library for DJI Drone
//
// ==================================================================
#include "link_opencv.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/highgui.hpp>
#include "tello.hpp"
#include <iostream>
#include <opencv2/imgcodecs.hpp>
/*
     https://github.com/yuukicammy/opencv_tracker_performance_test/blob/master/opencv_tracker/dev/src/main_simple_opencv_tracking.cpp
*/
static cv::Mat Image;
static cv::Rect2d BBox;
static std::string WindowName;
static bool Paused;
static bool SelectObject = false;
static bool StartSelection = false;

static const char* Keys =
{ "{@tracker_algorithm | | tracker algorithm }"
"{video_name       |   | path to video name        }"
"{tello_cam        | 0 | use the dji tello cam stream  }"
"{help h usage| |print this message   }"
};

static void Help(void)
{
	std::cout << "\nThis example shows the functionality of \"Long-term optical tracking API\""
		"-- pause video [p] and draw a bounding box around the target to start the tracker\n"
		"Call:\n"
		"./tracker <tracker_algorithm> <video_name>\n"
		<< std::endl;

	std::cout << "\n\nHot Keys: \n"
		"\tq - quit the program\n"
		"\tp - pause video\n";
}

static void OnMouse(int event, int x, int y, int, void*)
{
	if (!SelectObject)
	{
		switch (event)
		{
		case cv::EVENT_LBUTTONDOWN:
			//set origin of the bounding box
			StartSelection = true;
			BBox.x = x;
			BBox.y = y;
			break;
		case cv::EVENT_LBUTTONUP:
			//sei with and height of the bounding box
			BBox.width = std::abs(x - BBox.x);
			BBox.height = std::abs(y - BBox.y);
			Paused = false;
			SelectObject = true;
			break;
		case cv::EVENT_MOUSEMOVE:
			if (StartSelection)
			{
				//draw the bounding box
				cv::Mat currentFrame;
				Image.copyTo(currentFrame);
				cv::rectangle(currentFrame, cv::Point(BBox.x, BBox.y), cv::Point(x, y), cv::Scalar(255, 0, 0), 2, 1);
				cv::imshow(WindowName, currentFrame);
			}
			break;
		}
	}
}

bool CheckTrackerAlgType(cv::String& tracker_algorithm)
{
	if (tracker_algorithm == "BOOSTING" ||
		tracker_algorithm == "MIL" ||
		tracker_algorithm == "TLD" ||
		tracker_algorithm == "MEDIANFLOW" ||
		tracker_algorithm == "KCF")
	{
		std::cout << "Tracker Algorithm Type: " << tracker_algorithm << std::endl;
	}
	else{
		CV_Error(cv::Error::StsError, "Unsupported algorithm type " + tracker_algorithm + " is specified.");
	}
	return true;
}

/*
    for OpenCV 3.0 era
    ref :- https://github.com/opencv/opencv_contrib/commit/3ac9e242549a85572739afd232559d2e4baa7e68

void AXplusB5(cv::InputArray A_, cv::InputArray X_, cv::InputArray B_, cv::OutputArray dest_)
{
    Mat A = A_.getMat();
    Mat X = X_.getMat();
    Mat B = B_.getMat();

    Mat dest = A*X + B;
    dest.copyTo(dest_);
}

void cvMatToOutArray(cv::Mat& A_, cv::OutputArray dest_)
{
    cv::Mat dest = A_;
    cv::dest.copyTo(dest_);
}

as defined in imgproc.hpp
http://www5d.biglobe.ne.jp/~noocyte/Programming/OpenCV.html

void convert_image(cv::InputArray A_, cv::OutputArray B_ int code) {
    switch (code) {
		case 1:
		cv::cvtColor(A_, B_, COLOR_YUV420p2RGB);                      // YUV420p to COLOR_YUV420p2RGB other outputs are 
		break;
		
		case 2:
		cv::cvtColor(A_, B_, COLOR_YUV420sp2RGB);                    // YUV420sp
		break;
		
		case 3:
		cv::cvtColor(A_, B_, COLOR_RGB2YUV_I420);                    // BGR to COLOR_RGB2YUV_I420 
		break;		
		
		case 4:
		cv::cvtColor(A_, B_, COLOR_YUV2BGRA_YUY2);		            // to COLOR_YUV2BGRA_YUY2
		break;
		
	}
    
}
inline cv::Ptr<cv::Tracker> createTrackerByName(cv::String name)
{
    cv::Ptr<cv::Tracker> tracker;

    if (name == "KCF")
        tracker = cv::TrackerKCF::create();
    else if (name == "TLD")
        tracker = cv::TrackerTLD::create();
    else if (name == "BOOSTING")
        tracker = cv::TrackerBoosting::create();
    else if (name == "MEDIAN_FLOW")
        tracker = cv::TrackerMedianFlow::create();
    else if (name == "MIL")
        tracker = cv::TrackerMIL::create();
    else if (name == "GOTURN")
        tracker = cv::TrackerGOTURN::create();
    else
        CV_Error(cv::Error::StsBadArg, "Invalid tracking algorithm name\n");

    return tracker;
}
*/
int main(int argc, char** argv)
{
	cv::CommandLineParser parser(argc, argv, Keys);

	cv::String tracker_algorithm = parser.get<cv::String>(0);
	//cv::String video_name = parser.get<cv::String>(1);
        cv::String video_name = parser.get<cv::String>("video_name");
        bool use_tello_stream = parser.has("tello_cam");
	
	parser.about("OpenCV Tracker API Test");
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}

	if (tracker_algorithm.empty() || !parser.check())
	{
		parser.printErrors();
		return -1;
	}

    // enable the tello drone and enable the video camera stream
	//
	if (use_tello_stream==1) 
	{
            cv::VideoCapture capture{"udp://0.0.0.0:11111", cv::CAP_FFMPEG};
            Tello tello;
            if (!tello.connect()) return 0;
            tello.enable_video_stream();
	}
	CheckTrackerAlgType(tracker_algorithm);

	//open the capture
	cv::VideoCapture cap;
	cv::Mat frame;
        if (!video_name.empty()) {
	    cap.open(video_name);   	
	    if (!cap.isOpened())
	    {
		   Help();
                   std::cout << "***Could not initialize capturing...***\n";
		   std::cout << "Current parameter's value: \n";
		   parser.printMessage();
		   return -1;
	    }
	    //get the first frame
	    cap >> frame;
	} else if (use_tello_stream==1) {
	     capture >> frame;                              // we are now getting the video capture from the dji drone
	} else {
	     std::cout << "\033[31m No video file or tello_cam 1 argument supplied\n \033[0m" << std::endl;
	     return -2;
	}

	Paused = true;
	WindowName = "Tracking API: " + tracker_algorithm;
	cv::namedWindow(WindowName, 0);
	cv::setMouseCallback(WindowName, OnMouse, 0);

	//instantiates the specific Tracker
	// Ptr<Tracker> tracker = TrackerMIL::create();
	// Ptr<Tracker> tracker = TrackerTLD::create();
	// Ptr<Tracker> tracker = TrackerBoosting::create();
	// Ptr<Tracker> tracker = TrackerGOTURN::create()
	// Ptr<Tracker> tracker = TrackerKCF::create();
	// Ptr<Tracker> tracker = cv::TrackerMedianFlow::create();
	//
	cv::Ptr<cv::Tracker> tracker = cv::Tracker::create(tracker_algorithm);
	if (tracker == NULL)
	{
		std::cout << "***Error in the instantiation of the tracker...***\n";
		return -1;
	}

	std::cout << "\n\nHot Keys: \n"
		"\tq - quit the program\n"
		"\tp - pause video\n";

	frame.copyTo(Image);
	cv::imshow(WindowName, Image);
	bool initialized = false;
	while (true)
	{
		if (!Paused)
		{
			cap >> frame;
			frame.copyTo(Image);

			if (SelectObject && !initialized)
			{
				//initializes the tracker
				if (tracker->init(Image, BBox))
				{
					initialized = true;
				}
			}
			else if (initialized)
			{
				//updates the tracker
				if (tracker->update(frame, BBox))
				{
					cv::rectangle(Image, BBox, cv::Scalar(0, 0, 255), 2, 1);
				}
				else{
					SelectObject = initialized = false;
				}
			}
			imshow(WindowName, Image);
		}
		char c = static_cast<char>(cv::waitKey(2));
		if (c == 'q')
		    break;
		if (c == 'p')
		    Paused = !Paused;
	}

	return 0;
}

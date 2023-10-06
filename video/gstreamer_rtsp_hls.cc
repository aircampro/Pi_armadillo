// RTSP HLS Streaming of video example
//
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
//Comment these two out
/*#include "detector.h"
#include "poseEstimation.h" */
using namespace cv;
int main() 
{
//VideoCapture cap(0, cv::CAP_MSMF);  //Notebook camera input
VideoCapture cap("C:/www/town.avi", cv::CAP_MSMF); // video file input
VideoWriter writer; 
// Write this string to one line to be sure!!
writer.open("appsrc ! videoconvert ! videoscale ! video/x-raw,width=640,height=480
            ! x264enc speed-preset=veryfast tune=zerolatency bitrate=800 !
             rtspclientsink location=rtsp://localhost:8554/mystream ",
              0, 20, Size(640, 480), true);
// Comment this line out
Detector dec = Detector();
Mat img;
for (;;)
   {
      if (!cap.isOpened())
      {
     std::cout << "Video Capture Fail" << std::endl;
      break;
   }
     cap.read(img); 
     // Comment detector lines out
     /* dec.ProcessFrame(img); 
     dec.detect();
     Mat proc = dec.getResult();*/
    
     cv::resize(img, img, Size(640, 480));
     cv::imshow("raw", img);
     writer << img;
     cv::waitKey(25);
    }
}
// For tutorial go to
// https://funvision.blogspot.com
// this is optical flow using openCV
//
#include <iostream>
#include <fstream>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudabgsegm.hpp>
#include <opencv2/cudaobjdetect.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/cudaarithm.hpp"
#include <omp.h>
#include <chrono>

using namespace std;
using namespace cv;
using namespace cv::cuda; 

int main()
{

    Mat frame0;
    Mat frame1;
    // https://funvision.blogspot.com
    Ptr<cuda::FarnebackOpticalFlow> farn = cuda::FarnebackOpticalFlow::create();
    //Capture camera
    VideoCapture cap(0);

    for (;;) {
        Mat image;
        cap >> image;
    
        cv::cvtColor(image, frame0, cv::COLOR_BGR2GRAY);

        if (frame1.empty()) {
            frame0.copyTo(frame1);
        }
        else {
            Mat flow;
            //Put Mat into GpuMat
            GpuMat GpuImg0(frame0);
            GpuMat GpuImg1(frame1);
            //Prepare space for output
            GpuMat gflow(frame0.size(), CV_32FC2);
            // chrono time to calculate the the needed time to compute and
            // draw the optical flow result
            std::chrono::steady_clock::time_point begin = 
                         std::chrono::steady_clock::now(); 
            // Calculate optical flow
            farn->calc(GpuImg0, GpuImg1, gflow);
            // GpuMat to Mat
            gflow.download(flow);

                for (int y = 0; y < image.rows - 1; y += 10) {
                    for (int x = 0; x < image.cols - 1; x += 10) {
                        // get the flow from y, x position * 10 for better visibility
                        const Point2f flowatxy = flow.at<Point2f>(y, x) * 5;
                        // draw line at flow direction
                        line(image, Point(x, y), Point(cvRound(x + flowatxy.x),
                         cvRound(y + flowatxy.y)), Scalar(0, 255, 0), 2);
                        // draw initial point  https://funvision.blogspot.com
                        circle(image, Point(x, y), 1, Scalar(0, 0, 255), -1);
                    }
                }

            // end - begin time to calculate compute farneback opt flow + draw
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::cout << "Time difference = " << 
              std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() 
               << "[ms]" << std::endl;
            // Display result  https://funvision.blogspot.com
            imshow("Display window", image);
            waitKey(25);
            // Save frame0 to frame1 to for next round
            // https://funvision.blogspot.com
            frame0.copyTo(frame1);
        }
    }
}
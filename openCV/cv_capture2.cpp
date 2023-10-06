/*

    OpenCV 2.0 using USB webcam
	
*/
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/core.hpp>

int main(void)
{
	const char* windowName = "Image";

	cv::VideoCapture cap(0);
	if(!cap.isOpened())                           // open stream．
	{
		return -1;
	}
    int total = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);   // get the total frame count
	
	while (1)
	{
		cv::Mat img;
		cap >> img;
		cv::imshow(windowName, img);             // show the image captured
		int key = cv::waitKey(1);
		int n = (int)cap.get(cv::CAP_PROP_POS_FRAMES); // get the position of the frame
		std::cout << "\r Frame number " << std::setw(4) << n << '/' << total << std::flush
				
		if (key == 113)                          // ascii 113 sent
		{
			break;                               // drop out loop．
		}
	}
	cv::destroyAllWindows();                     // kill the window

	return(0);
}
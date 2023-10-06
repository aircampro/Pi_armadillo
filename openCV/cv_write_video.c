#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
	cv::VideoCapture video("input.mp4"); 

	if(!video.isOpened()){ 
		 
		 std::cout << "video.error" << std::endl;
		 return -1;
	}
	
	cv::Mat frame,gray,canny; 
	
	int fourcc = cv::VideoWriter::fourcc('M','P','4','V');
	
	double fps = video.get(cv::CAP_PROP_FPS); 
	
	int width = video.get(cv::CAP_PROP_FRAME_WIDTH); 
	int height = video.get(cv::CAP_PROP_FRAME_HEIGHT); 
	
	cv::VideoWriter output("output.mp4", fourcc, fps, cv::Size(width, height),0); 
	
	while(video.read(frame)){ 
		
		cvtColor(frame, gray, cv::COLOR_RGB2GRAY); 
	
		Canny(gray,canny, 50, 200); 
		
		output << canny; 
		
		int key = cv::waitKey(1);
		if(key == 'q'){
			
			cv::destroyWindow("canny");
			video.release();
			break; 
		}
	}
	
    return 0;
}
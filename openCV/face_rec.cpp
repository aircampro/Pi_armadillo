#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#ifdef _DEBUG
#pragma comment(lib, "opencv_world430d.lib")
#else
#pragma comment(lib, "opencv_world430.lib")
#endif

int main()
{
	const char* windowName = "Image";

	cv::VideoCapture cap(0);
	if (!cap.isOpened())
	{
		return -1;
	}

	// use this known dataset
	std::string cascade_path = "C:\\opencv\\sources\\samples\\winrt\\FaceDetection\\FaceDetection\\Assets\\haarcascade_frontalface_alt.xml";
	cv::CascadeClassifier cascade;
	cascade.load(cascade_path);

	while (1)
	{
		cv::Mat img;
		cap >> img;
		cv::Mat gray;
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

		std::vector<cv::Rect> faces;
		cascade.detectMultiScale(gray, faces, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));
		for (auto face : faces) {
			cv::rectangle(img, face, cv::Scalar(0, 0, 255), 2);
		}

		cv::imshow(windowName, img);          

		int key = cv::waitKey(1);
		if (key == 113)
		{
			break;
		}
	}
	cv::destroyAllWindows();
	return 0;
}
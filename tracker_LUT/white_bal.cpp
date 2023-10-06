#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/xphoto/white_balance.hpp>
/*
     white balance correction
	 compile : g++-8 -std=c++17 -o white_bal white_bal.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`
	 use : sudo ./white_bal -src=test1.jpg -th=0.3
*/
int main(int argc, const char* argv[]) {

	// ./out -src=[image path] -th=0.3
	cv::String keys = "{src||}""{th||}";
	cv::CommandLineParser parser(argc, argv, keys);
	cv::String src_path = parser.get<cv::String>("src");
	double th = parser.get<double>("th");

	std::cout << src_path << std::endl;

	cv::Mat src = cv::imread(src_path);
	cv::Mat dst;

    // Instance Generation
	cv::Ptr<cv::xphoto::GrayworldWB> obj = cv::xphoto::createGrayworldWB();

	// Saturation value: default 0.9
	std::cout << obj->getSaturationThreshold() << std::endl;

	// Set Saturation value
	obj->setSaturationThreshold(th);

	obj->balanceWhite(src, dst);

	cv::imwrite("./output.png", dst);

   return 0;
}
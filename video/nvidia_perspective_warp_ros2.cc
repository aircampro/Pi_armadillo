/*
MIT License
Copyright (c) 2021 MACNICA Inc. & aircampro
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 *	A perspective warp demonstration program using NVIDIA VPI
 *  This corrected display has been published as a ROS2 node
 
 refs:-

 https://qiita.com/sugimaro/items/85938bde53ebb92db22f
 https://qiita.com/porizou1/items/060dfdf6ba33cd60b365

 */
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <signal.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/alphamat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/aruco.hpp>

#include <vpi/OpenCVInterop.hpp>
#include <vpi/Context.h>
#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/PerspectiveWarp.h>
#include <vpi/algo/Rescale.h>

#define RET_OK	0
#define RET_ERR	-1

#define DEBUG_PRINT(str) \
	std::cout << __FILE__ << "(" << __LINE__ << "): " << str << std::endl

#define CHECK_STATUS(STMT)									  \
	do														  \
	{														  \
		VPIStatus status = (STMT);							  \
		if (status != VPI_SUCCESS)							  \
		{													  \
			char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH]; 	  \
			vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
			std::ostringstream ss;							  \
			ss << vpiStatusGetName(status) << ": " << buffer; \
			throw std::runtime_error(ss.str()); 			  \
		}													  \
	} while (0);
	
#define USB_PORT 0                                 # port where you connected the camera e.g. /dev/video0

// Utility function to wrap a cv::Mat into a VPIImage
static VPIImage ToVPIImage(VPIImage image, const cv::Mat &frame)
{
    if (image == nullptr)
    {
        // Ceate a VPIImage that wraps the frame
        CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(frame, 0, &image));
    }
    else
    {
        // reuse existing VPIImage wrapper to wrap the new frame.
        CHECK_STATUS(vpiImageSetWrappedOpenCVMat(image, frame));
    }
    return image;
}

static int openCamera(cv::VideoCapture& cap)
{
	cap.open(USB_PORT, cv::CAP_V4L2);
	if (!cap.isOpened()) {
		throw std::runtime_error("Unable to open camera: ");
	}
	std::cout << "Backend API: " << cap.getBackendName() << std::endl;
	
	cap.set(cv::CAP_PROP_FRAME_WIDTH, (double)cap_wt);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, (double)cap_ht);
	cap.set(cv::CAP_PROP_FPS, 30.0);
	cap.set(cv::CAP_PROP_BUFFERSIZE, (double)3);
	
	/* Get the actual configurations */
	cap_wt = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
	cap_ht = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	int fps = cap.get(cv::CAP_PROP_FPS);
	int bufSize = (int)cap.get(cv::CAP_PROP_BUFFERSIZE);
	std::cout << "Frame width :" << cap_wt << std::endl;
	std::cout << "Frame height:" << cap_ht << std::endl;
	std::cout << "FPS         :" << fps << std::endl;
	std::cout << "Buffer Size :" << bufSize << std::endl;

	return (RET_OK);
}

int getOutPoints(std::vector<std::vector<cv::Point2f>>& corners,std::vector<int>& ids, cv::Point2f *ptDst)
{
	int ret = RET_OK;

	for (int id = 0;id < 4;id++) {
		int idx = -1;
		for (int i = 0;i < 4;i++) {
			if (ids[i] == id) {
				idx = i;
				break;
			}
		}
		if (idx < 0) {
			std::cerr << "id not found" << std::endl;
			ret = RET_ERR;
			break;
		}

		ptDst[id] = cv::Point(corners[idx][id].x, corners[idx][id].y);
	}

	return (ret);
}

bool LoopFlag = true;
void signalHandler(int sig)
{
	std::cout << "Aborted." << std::endl;
	LoopFlag = false;
}

namespace usb_cam_node
{

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher(rclcpp::NodeOptions options);
  ~ImagePublisher();
  int cap_ht = 500;
  int cap_wt = 500;
  int vidWidth = 500;	
  int vidHeight = 500;
  std::string vid_name = "video_cap_file.mp4";
 
private:
  void publishImagePoll();
  void cleanup();
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap;
  VPIContext ctx = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  cv::Mat cvFrame;
  cv::VideoCapture invid;
  cv::Mat frame, frameCopy, warpFrame; 
  cv::Mat image(cap_ht, cap_wt, CV_8UC3, cv::Scalar(0, 0, 255));
  VPIImage imgInput, imgOutput, imgTemp; 
  VPIPayload warp;  
  VPIPerspectiveTransform xform;
  VPIImage imgVid = nullptr;
  VPIImage imgDisp = nullptr;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
};

ImagePublisher::ImagePublisher(rclcpp::NodeOptions options) : Node("image_publisher", options)
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 30), std::bind(&ImagePublisher::publishImagePoll, this));
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

  auto cret = openCamera(cap);
  if(!cap.isOpened() || (cret != RET_OK)){
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera ");
    rclcpp::shutdown();
  }
  try {  
		/* Open the video file to be warpped */
		if (!invid.open(vid_name)) {
			std::string msg = "Can't open the video file: ";
			msg.append("video_cap_file.mp4");
			throw std::runtime_error(msg);
		}
		vidWidth = (int)invid.get(cv::CAP_PROP_FRAME_WIDTH);
		vidHeight = (int)invid.get(cv::CAP_PROP_FRAME_HEIGHT);
		std::cout << vidWidth << ", " << vidHeight << std::endl;

		/* Create the context */
		CHECK_STATUS(vpiContextCreate(0, &ctx));

		/* Activate it. From now on all created objects will be owned by it. */
		CHECK_STATUS(vpiContextSetCurrent(ctx));

		/* Create the stream for the given backend */
		VPIStream stream;
		CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_CUDA, &stream));
		CHECK_STATUS(vpiImageCreate(cap_wt, cap_ht,	VPI_IMAGE_FORMAT_NV12_ER, 0, &imgInput)	);
		CHECK_STATUS(vpiImageCreate(cap_wt, cap_ht, VPI_IMAGE_FORMAT_NV12_ER, 0, &imgOutput	));
		CHECK_STATUS(vpiImageCreate(vidWidth, vidHeight, VPI_IMAGE_FORMAT_NV12_ER, 0, &imgTemp));

		/* Create a Perspective Warp payload */
		CHECK_STATUS(vpiCreatePerspectiveWarp(VPI_BACKEND_CUDA, &warp));

		memset(&xform, 0, sizeof(xform));

		detectorParams = cv::aruco::DetectorParameters::create();
		int dictionaryId = 0;
		dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

		std::cout << "Start grabbing" << std::endl
			<< "Press any key to terminate" << std::endl;

		if (signal(SIGINT, signalHandler) == SIG_ERR) {
			std::string msg = "Can't bind signal handler SIGINT: ";
			throw std::runtime_error(msg);
		}
		if (signal(SIGUSR1, signalHandler) == SIG_ERR) {
			std::string msg = "Can't bind signal handler SIGUSR1: ";
			throw std::runtime_error(msg);
		}
		if (signal(SIGUSR2, signalHandler) == SIG_ERR) {
			std::string msg = "Can't bind signal handler SIGUSR2: ";
			throw std::runtime_error(msg);
		}
	catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
		std::string msg = e.what();
		throw std::runtime_error(msg);
    }
}

ImagePublisher::~ImagePublisher()
{
}

void ImagePublisher::cleanup()
{
	// Clean up VPI
	vpiContextDestroy(ctx);
	std::cout << "Context destoried." << std::endl;
	
	/* Release the camera device */
	cap.release();
	invid.release();
	std::cout << "Video capture released." << std::endl;
}

void ImagePublisher::publishImagePoll()
{
	try {
        if (LoopFlag == true) {
			/* Capture a frame */
			cap.read(frame);
			if (frame.empty()) {
				std::cerr << "ERROR! blank frame grabbed" << std::endl;
				return;
			}

			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners, rejected;
			cv::aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams, rejected);

			frame.copyTo(frameCopy);
			if (ids.size() > 0 && ids.size() < 4) {	
			    cv::aruco::drawDetectedMarkers(frameCopy, corners, ids);
			}

			cv::Point2f ptSrc[4] = {
				cv::Point2f(0.0, 0.0),
				cv::Point2f((float)args.capWidth, 0.0),
				cv::Point2f((float)args.capWidth, (float)args.capHeight),
				cv::Point2f(0.0, (float)args.capHeight)
			};
			cv::Point2f ptDst[4];
			cv::Point pt[4];
			if (ids.size() == 4) {
				if(getOutPoints(corners, ids, ptDst) != RET_OK) {
					/* Display the captured frame */
					cv::imshow("Capture", frameCopy);
					return;
				}
				for (int id = 0;id < 4;id++) {
					pt[id] = ptDst[id];
				}

				cv::fillConvexPoly(
					frameCopy, pt, 4, cv::Scalar(0, 0, 0)
				);

				cv::Mat tmtrx = cv::getPerspectiveTransform(ptSrc, ptDst);

				bool ret = invid.read(cvFrame);
				if (!ret) {
					//invid.release();
					return;
				}

				imgVid = ToVPIImage(imgVid, cvFrame);
				imgDisp = ToVPIImage(imgDisp, frame);

				// First convert it to NV12 using CUDA
				CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, imgVid, imgTemp, NULL));

				// Rescale
				CHECK_STATUS(vpiSubmitRescale(stream, VPI_BACKEND_CUDA, imgTemp, imgInput, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0	));

				for (int i = 0;i < 3;i++) {
					for (int j = 0;j < 3;j++) {
							xform[i][j] = tmtrx.at<double>(i, j);
					}
				}

				// Do perspective warp using the backend passed in the command line.
				CHECK_STATUS(vpiSubmitPerspectiveWarp(stream, 0, warp, imgInput, xform, imgOutput, VPI_INTERP_LINEAR, VPI_BORDER_ZERO, 0	));

				// Convert output back to BGR using CUDA
				CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, imgOutput, imgDisp, NULL));
				CHECK_STATUS(vpiStreamSync(stream));

				// Now add it to the output video stream
				VPIImageData imgdata;
				CHECK_STATUS(vpiImageLock(imgDisp, VPI_LOCK_READ, &imgdata));
				CHECK_STATUS(vpiImageDataExportOpenCVMat(imgdata, &warpFrame));
				CHECK_STATUS(vpiImageUnlock(imgDisp));
				cv::add(warpFrame, frameCopy, frameCopy);
				cv::imshow("Capture", frameCopy);
			}
			else {
				/* Display the captured frame */
				cv::imshow("Capture", frameCopy);
			}

            // publish the corrected image to ROS
            sensor_msgs::msg::Image ros_img;
            cv_bridge::CvImage cv_img;
            cv_img.encoding = "bgr8"; 
            frameCopy >> cv_img.image;
            cv_img.header.stamp = this->now();
            cv_img.toImageMsg(ros_img);
            image_pub_->publish(std::move(ros_img));
		} else {
           ImagePublisher::cleanup();	
           ret = RET_OK;		   
		}
	}
	catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
        ret = RET_ERR;
    }
  }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam_node::ImagePublisher)

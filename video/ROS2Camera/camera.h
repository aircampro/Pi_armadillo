//#pragma once
#ifndef __CameraROS
#define __CameraROS

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

#include <thread>
#include <mutex>
#include <unistd.h>
#include <cmath>
#include <format>

cv::Mat morphology(const cv::Mat& image, const int morphology_mode, const int number_of_times) {
    cv::Mat result_image;
    int op = -1;
    if (morphology_mode == 0) {
        op = cv::MORPH_OPEN;
    } else if (morphology_mode == 1) {
        op = cv::MORPH_CLOSE;
    } else if (morphology_mode == 2) {
        op = cv::MORPH_GRADIENT;
    } else if (morphology_mode == 3) {
        op = cv::MORPH_TOPHAT;
    } else if (morphology_mode == 4) {
        op = cv::MORPH_BLACKHAT;
    }

    cv::morphologyEx(
            image,
            result_image,
            op,  
            cv::Mat(),
            cv::Point(-1, -1),
            number_of_times,  
            cv::BORDER_CONSTANT,
            cv::morphologyDefaultBorderValue());

    return result_image;
}

//
// Camera Class
//
class Camera
{
  Camera(const Camera &c);
  Camera &operator=(const Camera &w);

protected:

  unsigned char *buffer;
  cv::Mat cam_frame;
  std::thread thr;
  std::mutex mtx;
  bool run;
  unsigned int seqc = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  virtual void capture() {};

public:

  Camera()
  {
    buffer = nullptr;
    run = false;
  }

  virtual ~Camera()
  {
  }

  // starts the camera capture 
  void start( int ft )
  {

    // set state bit
    run = true;
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

    // run the capture as a thread
    thr = std::thread([this](){ this->capture(ft); });
  }

  // stops capture
  void stop()
  {
    if (run)
    {
      mtx.lock();
      run = false;
      mtx.unlock();
      thr.join();
    }
  }

  //int getWidth() const
  //{
  //  return width;
  //}

  //int getHeight() const
  //{
  //  return height;
  //}

  // Ovrvision Pro 
  virtual void increaseExposure() {};

  // Ovrvision Pro 
  virtual void decreaseExposure() {};

  // Ovrvision Pro 
  virtual void increaseGain() {};

  // Ovrvision Pro 
  virtual void decreaseGain() {};

  void 
  // transmit the grabbed frame to ROS2
  void transmit(time ts, int option)
  {
	std::string s1,s2;
    cv::Mat dst1;
    if (mtx.try_lock())
    {
      if (buffer)
      {
        // publish the grabbed image to ROS
        sensor_msgs::msg::Image ros_img;
        cv_bridge::CvImage cv_img;
        cv_img.encoding = "bgr8"; 
		switch (option) {
			case 0:
	        cv::blur(cam_frame, dst1, cv::Size(7, 7));
            dst1 >> cv_img.image;
			break;
			case 1:			
	        cv::medianBlur(cam_frame, dst1, 7);
            dst1 >> cv_img.image;
			break;
			case 2:
	        cv::GaussianBlur(cam_frame, dst1, cv::Size(7, 7), 0.0);
            dst1 >> cv_img.image;
			break;
            case 3:
	        cv::Laplacian(cam_frame, dst1, -1);
            dst1 >> cv_img.image;
			break;
            case 4:			
	        cv::Canny(cam_frame, dst1, 60.0, 150.0);
            dst1 >> cv_img.image;
			break;
            case 5:	
	        cv::threshold(cam_frame, dst1, 140, 255, cv::THRESH_BINARY);
            dst1 >> cv_img.image;
 	        break;
			case 6:
	        cv::threshold(cam_frame, dst1, 0, 255, cv::THRESH_OTSU);
            dst1 >> cv_img.image;
			break;
            case 7:
			const int times = 10;
            const cv::Mat morphology_image = morphology(cam_frame, cv::MORPH_OPEN, times);
            morphology_image >> cv_img.image;
			break;		
            case 8:
			const int times = 10;
            const cv::Mat morphology_image = morphology(cam_frame, cv::MORPH_CLOSE, times);
            morphology_image >> cv_img.image;
			break;
            case 9:
			const int times = 10;
            const cv::Mat morphology_image = morphology(cam_frame, cv::MORPH_GRADIENT, times);
            morphology_image >> cv_img.image;
			break;	
            case 10:
			const int times = 10;
            const cv::Mat morphology_image = morphology(cam_frame, cv::MORPH_TOPHAT, times);
            morphology_image >> cv_img.image;
			break;
            case 11:
			const int times = 10;
            const cv::Mat morphology_image = morphology(cam_frame, cv::MORPH_BLACKHAT, times);
            morphology_image >> cv_img.image;
			break;			
			default:
            cam_frame >> cv_img.image;
			break;
		}
        cv_img.header.seq = seqc++;
        cv_img.header.stamp = ts;
        cv_img.toImageMsg(ros_img);
        image_pub_->publish(std::move(ros_img));
        buffer = nullptr;
      }
      mtx.unlock();
    }
  }
};
#endif

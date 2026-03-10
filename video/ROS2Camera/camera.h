//#pragma once
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

cv::Mat convert_grayscale(const cv::Mat& image) {
    const int grayscale_channel_num = 1;
    if (image.channels() == grayscale_channel_num) {
        return image.clone();
    }
    cv::Mat gray_image;
    cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    return gray_image;
}

cv::Mat convert_colorimage(const cv::Mat& image) {
    const int colorimage_channel_num = 3;
    if (image.channels() == colorimage_channel_num) {
        return image.clone();
    }
    cv::Mat color_image;
    cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
    return color_image;
}

cv::Point2f get_moment_point(const cv::Mat& image) {
    cv::Mat image_gray = convert_grayscale(image);
    const cv::Moments image_moment = cv::moments(image_gray, false);
    const cv::Point2f moment_center = cv::Point2f(
            static_cast<float>(image_moment.m10 / image_moment.m00),
            static_cast<float>(image_moment.m01 / image_moment.m00));
    return moment_center;
}

cv::Mat draw_dot(const cv::Mat image, const cv::Point center, const float radius, const cv::Scalar color) {
    cv::Mat drawed_image = convert_colorimage(image);
    const int thickness = -1;
    cv::circle(drawed_image, center, static_cast<int>(radius), color, thickness, CV_MSA);
    return drawed_image;
}

cv::Mat adjust_brightness(const cv::Mat& image, const float brightness_bias) {
    cv::Mat colored_image = convert_colorimage(image);
    cv::Mat brightness_adjusted_image = colored_image + cv::Scalar(brightness_bias, brightness_bias, brightness_bias);
    return brightness_adjusted_image;
}

cv::Mat extract_by_color(const cv::Mat& image, const int extract_value[6], const int cv_color_code) {
    cv::Mat object_img = convert_colorimage(image);
    cv::Mat extracted_image;
    cv::Mat colored_image;

    cv::Mat lut = cv::Mat(256, 1, CV_8UC3);
    int lower[3] = {extract_value[0], extract_value[2], extract_value[4]};
    int upper[3] = {extract_value[1], extract_value[3], extract_value[5]};

    cv::cvtColor(object_img, colored_image, cv_color_code);

    for (int i = 0; i < 256; i++) {
        for (int k = 0; k < 3; k++) {
            if (lower[k] <= upper[k]) {
                if ((lower[k] <= i) && (i <= upper[k])) {
                    lut.data[i * lut.step + k] = 255;
                } else {
                    lut.data[i * lut.step + k] = 0;
                }
            } else {
                if ((i <= upper[k]) || (lower[k] <= i)) {
                    lut.data[i * lut.step + k] = 255;
                } else {
                    lut.data[i * lut.step + k] = 0;
                }
            }
        }
    }

    cv::LUT(colored_image, lut, colored_image);
    std::vector<cv::Mat> planes;
    cv::split(colored_image, planes);

    cv::Mat mask_image;
    cv::bitwise_and(planes[0], planes[1], mask_image);
    cv::bitwise_and(mask_image, planes[2], mask_image);

    object_img.copyTo(extracted_image, mask_image);
    return extracted_image;
}

cv::Mat adjust_contrast(const cv::Mat& image, const float contrast_gain) {
    cv::Mat contrast_adjusted_image = contrast_gain * image;
    return contrast_adjusted_image;
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
            case 12:
            const cv::Point2f moment_point = get_moment_point(cam_frame);
            const cv::Mat drawed_image = draw_dot(cam_frame, moment_point, 20, cv::Scalar(0, 0, 255));
            drawed_image >> cv_img.image;
			break;	
            case 13:
            const float brightness_bias = -25.f;
            const cv::Mat adjusted_image = adjust_brightness(cam_frame, brightness_bias);
            adjusted_image >> cv_img.image;
            break;	
            case 14:
            const float brightness_bias = 25.f;
            const cv::Mat adjusted_image = adjust_brightness(cam_frame, brightness_bias);
            adjusted_image >> cv_img.image;
            break;
			case 15:
            const int extract_value[6] = {40, 100, 0, 255, 0, 180};
            const cv::Mat color_extracted_image = extract_by_color(cam_frame, extract_value, cv::COLOR_BGR2HSV);
            color_extracted_image >> cv_img.image;
            break;	
            case 16:
            const float contrast_gain = 1.8f;
            const cv::Mat adjusted_image = adjust_contrast(cam_frame, contrast_gain);	
            adjusted_image >> cv_img.image;
            break;
            case 17:
            const float contrast_gain = 0.2f;
            const cv::Mat adjusted_image = adjust_contrast(cam_frame, contrast_gain);	
            adjusted_image >> cv_img.image;
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

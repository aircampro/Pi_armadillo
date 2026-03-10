#ifndef __CameraRealSen
#define __CameraRealSen

#include "Camera.h"
// for Relasense camera we are using the robot integrated control software "Klubo"
// https://chitose-robotics.com/product
//
#include "crewbo/crewbo.h"
#include "project_lib/vision_assistant.h"

cv::Mat crop_depth_image_by_depth_value(const cv::Mat& depth_image, const float far_value, const float near_value) {
    cv::Mat result_depth_image = depth_image.clone();
    for (int i = 0; i < depth_image.cols; i++) {
        for (int j = 0; j < depth_image.rows; j++) {
            const double distance = static_cast<double>(depth_image.at<float>(cv::Point(i, j)));
            if (distance > far_value || distance < near_value) {
                result_depth_image.at<float>(cv::Point(i, j)) = 0.f;
            }
        }
    }
    return result_depth_image;
}

// Realsense camera  
class CamRs
  : public Camera
{
  crewbo::camera::RealSense camera;
  cv::Mat frame;
  cv::Mat cropped_depth_image;
  cv::Mat colorized_image;
  int req_fps;
  
  // initialize the camera
  bool init()
  {
    frame = camera.fetchDepthFrame_();	
    buffer = frame.data;
    return true;
  }

  virtual void capture(float depth_near_value = 600.f, float depth_far_value = 1600.f)
  {
    mtx.lock();
    while (run)
    {
      if (!buffer)                                                                 // buffer is cleared each time it is transmitted
      {
		try {
              frame = camera.fetchDepthFrame_();
              buffer = frame.data;
              cropped_depth_image = crop_depth_image_by_depth_value(frame, depth_far_value, depth_near_value);
              colorized_image = crewbo::camera::RealSense::colorizeDepthImage_(cropped_depth_image);
		      cam_frame = colorized_image.clone();
              continue;
           }
	    catch (std::exception &e) {
              continue;
        }
      }

      mtx.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds((1/req_fps)*1000));                    // set to frame rate
      mtx.lock();
    }
    mtx.unlock();
  }

public:

  CamRs() {}

  virtual ~CamRs()
  {
    stop();
  }

  bool open(int fps = 60)
  {
	req_fps = fps;
    return init();
  }

  bool open(int fps = 60)
  {
	req_fps = fps;
    return init();
  }
};
#endif
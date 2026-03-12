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
class CamRsScan
  : public Camera
{
  crewbo::camera::RealSense camera;
  cv::Mat frame;
  cv::Mat cropped_depth_image;
  cv::Mat colorized_image;
  int req_fps;
  float depth_far_value = 600.0f;
  float depth_near_value = 300.0f;  
  std::vector<float> vdist = {0.0f, 200.0f, 400.0f, 600.0f, 800.0f, 1000.0f, 1200.0f, 1400.0f, 1600.0f, 1800.0f, 2000.0f };
  int indx = 0;                                                             // start idx
  int z = 0;
  int dir = 0;
	
  // initialize the camera
  bool init()
  {
    frame = camera.fetchDepthFrame_();	
    buffer = frame.data;
	indx = vdist.size() - 1;
    return true;
  }

  virtual void capture(int no_of_repeat_frames)
  {
    mtx.lock();

    while (run)
    {
      if (!buffer)                                                                 // buffer is cleared each time it is transmitted
      {
		try {
              frame = camera.fetchDepthFrame_();
              buffer = frame.data;
			  depth_far_value = vdist.at(indx);
			  depth_near_value = vdist.at(indx-1);
              cropped_depth_image = crop_depth_image_by_depth_value(frame, depth_far_value, depth_near_value);
              colorized_image = crewbo::camera::RealSense::colorizeDepthImage_(cropped_depth_image);
		      cam_frame = colorized_image.clone();
	          z++;
			  /* cycle from back to front then back */
              if (z >= no_of_repeat_frames) {
                if (dir == 0) {
                    indx--;            
                } else {
                    indx++;
                }

                if ((indx <= 0) || (indx > (int)(vdist.size()) - 1)) {
                    if (dir == 0) {
                        indx = 1;
                        dir = 1;
                    } else {
                        indx = v.size() - 1;
                        dir = 0;
                    }
                }
		        z = 0;
              }	
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

  CamRsScan() {}

  virtual ~CamRsScan()
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

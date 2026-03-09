#ifndef __CameraOpenCV
#define __CameraOpenCV

#include "Camera.h"

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/legacy/constants_c.h>
#if defined(_WIN32)
#  define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
#  if defined(_DEBUG)
#    define CV_EXT_STR "d.lib"
#  else
#    define CV_EXT_STR ".lib"
#  endif
#  pragma comment(lib, "opencv_core" CV_VERSION_STR CV_EXT_STR)
#  pragma comment(lib, "opencv_highgui" CV_VERSION_STR CV_EXT_STR)
#  pragma comment(lib, "opencv_videoio" CV_VERSION_STR CV_EXT_STR)
#endif

cv::Mat convert_colorimage(const cv::Mat& image) {
    const int colorimage_channel_num = 3;
    if (image.channels() == colorimage_channel_num) {
        return image.clone();
    }
    cv::Mat color_image;
    cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
    return color_image;
}

cv::Mat set_reference_text(const cv::Mat& image, double message) {
    cv::Mat image_with_message;
    int font_style = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 1.0;
    int bar_color = 255;
    int font_thickness = 1;
    int bar_height = 60;
    std::string message1{std::format("frame time {}", message)};
	
    cv::Mat message_bar = cv::Mat::ones(bar_height, image.cols, CV_8UC1) * bar_color;
    cv::Size text_size = cv::getTextSize(message1, font_style, font_scale, font_thickness, 0);
    cv::Point message_start_point((int)(message_bar.cols - text_size.width) / 2, (int)(message_bar.rows * 0.6));
    cv::putText(message_bar, message1, message_start_point, font_style, font_scale, cv::Scalar(0, 0, 0), font_thickness);

    std::vector<cv::Mat> concat_images{convert_colorimage(image), convert_colorimage(message_bar)};
    cv::vconcat(concat_images, image_with_message);
    return image_with_message;
}

// OpenCV 
class CamCv
  : public Camera
{
  cv::VideoCapture camera;
  cv::Mat frame;
  double frameTime;
  int exposure, gain, contrast, brightness;
  int req_fps;
  
  // initialize the camera
  bool init(int initial_width, int initial_height, int initial_fps)
  {
    if (initial_width > 0) camera.set(CV_CAP_PROP_FRAME_WIDTH, initial_width);
    if (initial_height > 0) camera.set(CV_CAP_PROP_FRAME_HEIGHT, initial_height);
    if (initial_fps > 0) camera.set(CV_CAP_PROP_FPS, initial_fps);

    if (camera.grab())
    {
      frameTime = 0.0;
      width = static_cast<int>(camera.get(CV_CAP_PROP_FRAME_WIDTH));
      height = static_cast<int>(camera.get(CV_CAP_PROP_FRAME_HEIGHT));
      if (width == 0) width = initial_width;
      if (height == 0) height = initial_height;
      gain = static_cast<int>(camera.get(CV_CAP_PROP_GAIN));
      exposure = static_cast<int>(camera.get(CV_CAP_PROP_EXPOSURE) * 10.0);
	  contrast = static_cast<int>(camera.get(CV_CAP_PROP_CONTRAST));
	  brightness = static_cast<int>(camera.get(CV_CAP_PROP_BRIGHTNESS));	  
	  std::cout << "camera exposure : " << exposure << std::endl;
	  std::cout << "camera gain : " << gain << std::endl;
	  std::cout << "camera contrast : " << contrast << std::endl;	
	  std::cout << "camera brightness : " << brightness << std::endl;		  
      camera.retrieve(frame, 3);
      buffer = frame.data;
      return true;
    }
    return false;
  }

  virtual void capture(int ft)
  {
    mtx.lock();
    while (run)
    {
      if (!buffer)                                                                 // buffer is cleared each time it is transmitted
      {
		try {
           if (camera.grab())
           {
             frameTime = camera.get(CV_CAP_PROP_POS_MSEC) * 0.001;
             camera.retrieve(frame, 3);
             buffer = frame.data;
			 if (ft == 0) {                                                        // without the frame time
		         cam_frame = frame.clone();
             } else if (ft == 1) {
			     const cv::Mat image_with_text = set_reference_text(frame, frameTime);
		         cam_frame = image_with_text.clone();
			 } else if (ft == 2) {
				std::string text{std::format("frame time {}", frameTime)};
                int fontFace = cv::FONT_HERSHEY_SCRIPT_COMPLEX;
                double fontScale = 1.2;
                int thickness = 3;
                cv::putText(frame, text, cv::Point(20,20), fontFace, fontScale, cv::Scalar::all(250), thickness, CV_AA);
                cam_frame = frame.clone();
			 }
             continue;
           }
		}
	    catch (std::exception &e) {
            continue;
        }
      }

      mtx.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds((1/req_fps)*1000));                    // set to 100 Hz
      mtx.lock();
    }
	camera.release();
    mtx.unlock();
  }

public:

  CamCv() {}

  virtual ~CamCv()
  {
    stop();
  }

  bool open(int device, int width = 0, int height = 0, int fps = 0)
  {
	req_fps = fps;
    camera.open(device);
    if (camera.isOpened() && init(width, height, fps)) return true;
    return false;
  }

  bool open(const std::string &file, int width = 0, int height = 0, int fps = 0)
  {
	req_fps = fps;
    camera.open(file);
    if (camera.isOpened() && init(width, height, fps)) return true;
    return false;
  }

  virtual void increaseExposure()
  {
    if (camera.isOpened()) camera.set(CV_CAP_PROP_EXPOSURE, ++exposure * 0.1);
  }

  virtual void decreaseExposure()
  {
    if (camera.isOpened()) camera.set(CV_CAP_PROP_EXPOSURE, --exposure * 0.1);
  }

  virtual void increaseGain()
  {
    if (camera.isOpened()) camera.set(CV_CAP_PROP_GAIN, ++gain);
  }

  virtual void decreaseGain()
  {
    if (camera.isOpened()) camera.set(CV_CAP_PROP_GAIN, --gain);
  }
};
#endif
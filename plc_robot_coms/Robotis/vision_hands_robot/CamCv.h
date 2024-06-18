//#pragma once
#ifndef __CameraOpenCV
#define __CameraOpenCV

#include <iostream>
#include <chrono>

//
// FrameGrabber
// OpenCV Control of Camera or video file input
//

// include camera interface library which is the control wrapper to this class
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

// uncomment below if you want to write the video output to file
// #define __VideoWriteActive_
#ifdef __VideoWriteActive_
enum video_out_codec_e {
  xvid,
  divx,
  h264,
  wmv2,
  wmv8,
  wmv9,
  vp9,
  mp4,
  dib,
  no_of_video_out_codecs
};
const video_out_codec_e video_out_format = video_out_codec_e::xvid;        // this is codec we are using for output
#endif

// uncomment to use ms rather than seconds for frame times
// #define _use_ms

// OpenCV frame grabber class
class CamCv
  : public Camera
{
    // define a OpenCV video capture device 
    cv::VideoCapture camera;

    // if you defined to write the raw video file to disk
#ifdef __VideoWriteActive_
    std::string out_file_name;
    int fourcc;

	// set the output stream codec and file name
    switch (video_out_format) {
	    case video_out_codec_e::xvid:
		{
	        fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');	// Xvid  / file extension is .avi
	        std::string out_file_name = "video_out.avi";			
		}
		break;

	    case video_out_codec_e::divx:
		{
	        fourcc = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');	// DivX  / .avi
	        std::string out_file_name = "video_out.avi";			
		}
		break;
		
	    case video_out_codec_e::h264:
		{
	        fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');	// H.264 / .wmv
	        std::string out_file_name = "video_out.wmv";			
		}
		break;

	    case video_out_codec_e::wmv2:
		{
	        fourcc = cv::VideoWriter::fourcc('W', 'M', 'V', '2');	// WMV8  / .wmv
	        std::string out_file_name = "video_out.wmv";			
		}
		break;

	    case video_out_codec_e::wmv9:
		{
	        fourcc = cv::VideoWriter::fourcc('W', 'M', 'V', '3');	// WMV9  / .wmv
	        std::string out_file_name = "video_out.wmv";			
		}
		break;

	    case video_out_codec_e::wmv8:
		{
	        fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');	// ISO MPEG-4 / .mp4
	        std::string out_file_name = "video_out.mp4";			
		}
		break;

	    case video_out_codec_e::vp9:
		{
	        fourcc = cv::VideoWriter::fourcc('V', 'P', '9', '0');	// VP9   / .avi
	        std::string out_file_name = "video_out.avi";			
		}
		break;
		
	    case video_out_codec_e::mp4:
		{
	        fourcc = cv::VideoWriter::fourcc('M', 'P', '4', '3');	// MS MPEG-4  / .avi
	        std::string out_file_name = "video_out.avi";			
		}
		break;

	    case video_out_codec_e::dib:
		{
	        fourcc = cv::VideoWriter::fourcc('D', 'I', 'B', ' ');	// RGB  / .avi
	        std::string out_file_name = "video_out.avi";			
		}
		break;
		
		default:
		break;
		
    }
	
	cv::VideoWriter writer;
#endif

  // OpenCV matrix
  cv::Mat frame;

  // define the frame times in seconds and milliseconds
  double frameTime_s;
  double frameTime_ms;

  // exposure and gain
  int exposure, gain;

  // initialise
  bool init(int initial_width, int initial_height, int initial_fps)
  {
    // set width and frame height and fps
    if (initial_width > 0) camera.set(CV_CAP_PROP_FRAME_WIDTH, initial_width);
    if (initial_height > 0) camera.set(CV_CAP_PROP_FRAME_HEIGHT, initial_height);
    if (initial_fps > 0) camera.set(CV_CAP_PROP_FPS, initial_fps);

    // grab frame
    if (camera.grab())
    {
      // reset tick counters
      auto start = std::chrono::system_clock::now();
      auto end = std::chrono::system_clock::now();

      // reset frame times
      frameTime_s = 0.0;
      frameTime_ms = 0.0;
	  
      // get the width and height of the frame
      width = camera.get(CV_CAP_PROP_FRAME_WIDTH);
      height = camera.get(CV_CAP_PROP_FRAME_HEIGHT);

      // macOS if no reply then use what we tryed to set it to
      if (width == 0) width = initial_width;
      if (height == 0) height = initial_height;

      // intialise the gain and exposure to that of the camera
      gain = camera.get(CV_CAP_PROP_GAIN);
      exposure = camera.get(CV_CAP_PROP_EXPOSURE) * 10;

      // retrieve frame
      camera.retrieve(frame, 3);

      // fill buffer with byte data from the frame
      buffer = frame.data;

      // write to a cam_frame for analysis
      cam_frame = frame.clone();

      // write the frame to the output stream if defined		  
#ifdef __VideoWriteActive_
      fps = camera.get(cv::CAP_PROP_FPS);	
	  writer.open(out_file_name, fourcc, fps, cv::Size(width, height));
      if (writer.isOpened() == true) {
         writer << frame;
	  }
#endif

      // success
      return true;
    }

    // error failed to grab
    return false;
  }

  // cature frame
  virtual void capture()
  {
    // lock mutex
    mtx.lock();

    // initialise start clock clock times
    auto start = std::chrono::system_clock::now();
	
    // do forever 
    while (run)
    {
   	  
      // the buffer is cleared every time we process the frame using yolo so we need a new frame and the timer has expired so grab a new grab frame
#ifdef _use_ms
      // Calculate the difference (end - start) in milliseconds
	  auto end = std::chrono::system_clock::now();
      std::chrono::duration<double, std::milli> elapsed_ms = end - start; 
      if (!buffer && elapsed_ms.count >= frameTime_ms)
#else
      // Calculate the difference (end - start) in seconds
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_s = end - start;
      if (!buffer && elapsed_s.count >= frameTime_s)
#endif
      {
        // grab frame
        if (camera.grab())
        {
          // get frame time
          frameTime_s = camera.get(CV_CAP_PROP_POS_MSEC) * 0.001;
          frameTime_ms = camera.get(CV_CAP_PROP_POS_MSEC);
          auto start = std::chrono::system_clock::now();
	
          // retrive frame into openCV matrix
          camera.retrieve(frame, 3);

          // fill buffer
          buffer = frame.data;
		  
		  // write to a duplicate openCV cam_frame for analysis
		  cam_frame = frame.clone();

#ifdef __VideoWriteActive_
         if (writer.isOpened() == true) {
             writer << frame;
	     }
#endif

          // continue
          continue;
        }

        // Rewind movie file if frame cannot be acquired
        if (camera.set(CV_CAP_PROP_POS_FRAMES, 0.0))
        {
          // reset the elapsed time
          auto start = std::chrono::system_clock::now();
          auto end = std::chrono::system_clock::now();

          // reset frame time
          frameTime_s = 0.0;
          frameTime_ms = 0.0;
		  
          // advance to next frame
          continue;
        }
      }

      // If the frame cannot be cut out, release the lock
      mtx.unlock();

      // After waiting a little while for other threads to access the resource
      std::this_thread::sleep_for(std::chrono::milliseconds(10L));

      // lock mutex
      mtx.lock();
    }
#ifdef __VideoWriteActive_
	if (writer.isOpened() == true) {
        writer.release();
    }
#endif			
    // unlock mutex
    mtx.unlock();
  }

public:

  // create class instance
  CamCv() {}

  // delete the class instance
  virtual ~CamCv()
  {
    // stop capture
    stop();
  }

  // opens a camera device 
  bool open(int device, int width = 0, int height = 0, int fps = 0)
  {
    // open the device
    camera.open(device);

    // if open set the width height and frames per second
    if (camera.isOpened() && init(width, height, fps)) return true;

    // return false if unable to open the device
    return false;
  }

  // opens a video file
  bool open(const std::string &file, int width = 0, int height = 0, int fps = 0)
  {
    // open file
    camera.open(file);

    // if open set the width height and frames per second
    if (camera.isOpened() && init(width, height, fps)) return true;

    // return false if unable to open
    return false;
  }

  // increase the exposure
  virtual void increaseExposure()
  {
    if (camera.isOpened()) camera.set(CV_CAP_PROP_EXPOSURE, ++exposure * 0.1);
  }

  // decrease the exposure
  virtual void decreaseExposure()
  {
    if (camera.isOpened()) camera.set(CV_CAP_PROP_EXPOSURE, --exposure * 0.1);
  }

  // increase gain
  virtual void increaseGain()
  {
    if (camera.isOpened()) camera.set(CV_CAP_PROP_GAIN, ++gain);
  }

  // decrease gain
  virtual void decreaseGain()
  {
    if (camera.isOpened()) camera.set(CV_CAP_PROP_GAIN, --gain);
  }
};
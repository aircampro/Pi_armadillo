//#pragma once
#ifndef __CameraYolo
#define __CameraYolo

//
// Class for performing yolo object classification on camera frames for robotic control
//

// threading and mutex lock/unlocking between threads
#include <thread>
#include <mutex>

// class for object recognition using yolo
#include "yolo_engine.cpp"

// define the choice of yolo v5 or v8
#define DRAWCYCLE_YOLO8 1                            // when set uses yolo8 model 
#define DRAWCYCLE_YOLO5 0                            // when set uses yolo5 model 

// uncomment below if you want to write the video output to file
// #define __YoloWriteActive_
#ifdef __YoloWriteActive_
enum yolo_out_codec_e {
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
const yolo_out_codec_e yolo_out_format = yolo_out_codec_e::h264;            // choose the codec for the output video
#endif

// uncomment to view the yolo video output 
//#define __YoloView_

//
// Class for controlling the camera with vision classification via yolo
//
class Camera
{
  // Block copy constructor
  Camera(const Camera &c);

  // block assignment
  Camera &operator=(const Camera &w);

protected:

  // byte buffer representing the frame
  unsigned char *buffer;
  
  cv::Mat cam_frame;

  // frame properties
  int width, height;
  double fps;

  // Mutex
  std::thread thr;

  // mutex for threads
  std::mutex mtx;

  // running state
  bool run;

  // indicates the openCV vision is running
  int ai_running;
  
  // yolo objects
  //pt_YoloObjectDetection = new YoloObjectDetection;
  YoloObjectDetection* pt_YoloObjectDetection = nullptr;		//サムネイル表示の時のAIクラス 複数のカメラでAIを共有、管理クラスからポインタをコピー
  //YoloAIParametors yp;
  int wave_hands;
  
#ifdef __YoloWriteActive_
    std::string yo_file_name;
    int yo_fourcc;

	// set the output stream
    switch (yolo_out_format) {
	    case yolo_out_codec_e::xvid:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');	// Xvid  /  .avi
	        std::string yo_file_name = "yolo_out.avi";			
		}
		break;

	    case yolo_out_codec_e::divx:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');	// DivX  / .avi
	        std::string yo_file_name = "yolo_out.avi";			
		}
		break;
		
	    case yolo_out_codec_e::h264:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');	// H.264 / .wmv
	        std::string yo_file_name = "yolo_out.wmv";			
		}
		break;

	    case yolo_out_codec_e::wmv2:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('W', 'M', 'V', '2');	// WMV8  / .wmv
	        std::string yo_file_name = "yolo_out.wmv";			
		}
		break;

	    case yolo_out_codec_e::wmv9:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('W', 'M', 'V', '3');	// WMV9  / .wmv
	        std::string yo_file_name = "yolo_out.wmv";			
		}
		break;

	    case yolo_out_codec_e::wmv8:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');	// ISO MPEG-4 / .mp4
	        std::string yo_file_name = "yolo_out.mp4";			
		}
		break;

	    case yolo_out_codec_e::vp9:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('V', 'P', '9', '0');	// VP9   / .avi
	        std::string yo_file_name = "yolo_out.avi";			
		}
		break;
		
	    case yolo_out_codec_e::mp4:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('M', 'P', '4', '3');	// MS MPEG-4  / .avi
	        std::string yo_file_name = "yolo_out.avi";			
		}
		break;

	    case yolo_out_codec_e::dib:
		{
	        yo_fourcc = cv::VideoWriter::fourcc('D', 'I', 'B', ' ');	// RGB  / .avi
	        std::string yo_file_name = "yolo_out.avi";			
		}
		break;
		
		default:
		break;
		
    }
	
	cv::VideoWriter yo_writer;
#endif
		
  // capture the frame
  virtual void capture() {};

public:

  // constructor
  Camera()
  {
    // Note that the image has not yet been acquired
    buffer = nullptr;

    // Record that the thread is stopped
    run = false;
	
	ai_running = 0;
  }

  // destructor
  virtual ~Camera()
  {
	// for yolo
	if (pt_YoloObjectDetection != nullptr)
    {
        delete pt_YoloObjectDetection;
        pt_YoloObjectDetection = nullptr;
    }
#ifdef __YoloWriteActive_
	if (yo_writer.isOpened() == true) {
        yo_writer.release();
    }
#endif
  }

  // starts the camera capture and initialises the yolo model
  void start( int DrawCycleMode, int wid, int ht )
  {
	pt_YoloObjectDetection = new YoloObjectDetection;                               // malloc memory for the yolo object
	if (DrawCycleMode == DRAWCYCLE_YOLO8)
    {
        pt_YoloObjectDetection->YP.yolo_version = YOLOV8;
        pt_YoloObjectDetection->YP.onnx_file_name = DEFAULT_ONNX_FILE_PATH;
        pt_YoloObjectDetection->YP.names_file_name = DEFAULT_NAMES_FILE_PATH;
    }
    else                                                                             //YOLOV5
    {
        pt_YoloObjectDetection->YP.yolo_version = YOLOV5;
        pt_YoloObjectDetection->YP.onnx_file_name = DEFAULT_ONNX_FILE_PATH_YOLOV5;
        pt_YoloObjectDetection->YP.names_file_name = DEFAULT_NAMES_FILE_PATH_YOLOV5;
    }

    pt_YoloObjectDetection->YP.input_width = wid;
    pt_YoloObjectDetection->YP.input_height = ht;
    pt_YoloObjectDetection->YP.score_threshold = (float)DEFAULT_SCORE_THRESHOLD;
    pt_YoloObjectDetection->YP.nms_threshold = (float)DEFAULT_NMS_THRESHOLD;
    pt_YoloObjectDetection->YP.confidence_thresgold = (float)DEFAULT_CONF_THRESHOLD;
    ai_running = pt_YoloObjectDetection->init_yolo(pt_YoloObjectDetection->YP, true, false);
	
    run = true;                                                                // set state bit

    thr = std::thread([this](){ this->capture(); });                          // run the capture as a thread
#ifdef __YoloWriteActive_
	yo_writer.open(yo_file_name, yo_fourcc, fps, cv::Size(width, height));
#endif
  }

  // stop the frame grabber and unload the yolo model
  void stop()
  {

    // If the capture thread is running
    if (run)
    {
      // lock the mutex
      mtx.lock();

      // set the global frame grabber flag to false
      run = false;

      // unloack the mutex
      mtx.unlock();

#ifdef __YoloWriteActive_
	if (yo_writer.isOpened() == true) {
        yo_writer.release();
    }
#endif
      // join the threads
      thr.join();
    }
	
	// for yolo unload the model
	if (pt_YoloObjectDetection != nullptr)                // memory malloced to yolo object
    {
        delete pt_YoloObjectDetection;                    // free the this memory by deleting the object
        pt_YoloObjectDetection = nullptr;                 // set the pointer back to null
    }
	ai_running = 0;
	
  }

  // get frame width
  int getWidth() const
  {
    return width;
  }

  // get frame height
  int getHeight() const
  {
    return height;
  }

  // get frame rate in frames per second
  double getFPS() const
  {
    return fps;
  }
  
  // increase Exposure
  virtual void increaseExposure() {};

  // decrease Exposure
  virtual void decreaseExposure() {};

  // increase Gain
  virtual void increaseGain() {};

  // decrease Gain
  virtual void decreaseGain() {};

  // process the frame which was grabbed
  void transmit()
  {
	std::string s1,s2;
    // try to aquire the mutex (when frame grabber is not active)
    if (mtx.try_lock())
    {
      // buffer has been filled by frame farbber
      if (buffer)
      {

        // perform yolo object detection on the camera feed
		if (ai_running == 1) {
            pt_YoloObjectDetection->_pre_process(cam_frame);
            Mat out_frame = pt_YoloObjectDetection->_post_process(true, cam_frame, s1, s2);
		    std::cout << "wave hand state = " << pt_YoloObjectDetection->wave_hand_state << std::endl;
			wave_hands = pt_YoloObjectDetection->wave_hand_state;
#ifdef __YoloWriteActive_
            if (yo_writer.isOpened() == true) {
                yo_writer << out_frame;
	        }
#endif
#ifdef __YoloView_
		    if (out_frame.empty() != true) {
		        cv::imshow("yolo frame", out_frame);
		    }
#endif		
        // Record data transfer completion
        buffer = nullptr;
      }

      // Unlock 
      mtx.unlock();
    }
  }
};
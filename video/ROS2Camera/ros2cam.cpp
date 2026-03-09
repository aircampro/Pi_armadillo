//	
// send openCV camera to ROS2 channel
//
#include "CamCv.h"
#include <signal.h>

std::string capture_input("test_video.mp4");
int capture_width(500);
int capture_height(500);
int capture_fps(0);
bool run_flg = true;

void signalHandler(int sig)
{
	std::cout << "Aborted." << std::endl;
	run_flg = false;
}

int main(int argc, const char* argv[])
{

  CamCv camera;
  cv::String keys = "{src||}""{height||}""{width||}""{fps||}";
  cv::CommandLineParser parser(argc, argv, keys);
  std::String src_path = static_cast<std::string>(parser.get<cv::String>("src"));
  capture_width = static_cast<int>(parser.get<cv::int>("width"));
  capture_height = static_cast<int>(parser.get<cv::int>("height"));
  capture_fps = static_cast<int>(parser.get<cv::int>("fps"));

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
		
  try { 
    int port_no = stoi(src_path);  
    if (!camera.open(port_no, capture_width, capture_height, capture_fps))
    {
      throw std::runtime_error("Can't open capture device.");
    }
  }
  catch (std::exception &e) {
    if (!camera.open(src_path, capture_width, capture_height, capture_fps))
    {
      throw std::runtime_error("Can't open capture device.");
    }
  }
  camera.start();

  while (run_flg)
  {
    camera.transmit();
  }
}
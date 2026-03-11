//	
// run the realsense depth scanner
//
#include "CamCv.h"
#include <signal.h>

bool run_flg = true;

void signalHandler(int sig)
{
	std::cout << "Aborted." << std::endl;
	run_flg = false;
}

int main(int argc, const char* argv[])
{

  CamCv camera;
  cv::String keys = "{no_rpt_frames||}""{fps||}";
  cv::CommandLineParser parser(argc, argv, keys);
  int no_of_rpt_frms = static_cast<int>(parser.get<cv::int>("width"));
  int capture_fps = static_cast<int>(parser.get<cv::int>("fps"));

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
    if (camera.open(capture_fps))
    {
      throw std::runtime_error("Can't open capture device.");
    }
  }
  catch (std::exception &e) {
     srd::cout << "error with realsense camera") << std::endl;
  }
  camera.start(no_of_rpt_frms);

  while (run_flg)
  {
    camera.transmit();
  }
}
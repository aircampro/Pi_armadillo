// ---------------------------------------------------------------------------------------------
// main.cpp
// uses openCV to get the video stream and then applies the openCL function on the grabbed frame
//
// to get openCL
// linux - https://wiki.archlinuxjp.org/index.php/GPGPU
// windows - intel https://software.intel.com/en-us/intel-opencl
// macOS - https://www.khronos.org/registry/OpenCL/
//
// ---------------------------------------------------------------------------------------------
/* openCL install like this
$ git clone https://github.com/KhronosGroup/OpenCL-CLHPP.git $ cd OpenCL-CLHPP/
$ mkdir build
$ cd build
$ cmake -DBUILD_EXAMPLES=0 -DBUILD_TESTS=0 ..
$ make
$ ls include/CL/
cl.hpp cl2.hpp
$ cd docs
$ doxygen
*/
#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>

#include <csignal>
#include <unistd.h>
#include <exception>

#include <stdio.h>
#include <malloc.h>

#define __CL_ENABLE_EXCEPTIONS
#include <CL/cl.hpp>

#include "opencv2/opencv.hpp"
#include "opencl.h"

using namespace std;

volatile sig_atomic_t eflag = 0;
volatile int sn = 0;

void signalHandler(int signum) {
    eflag = 1;                                                         // stop the frame grabber and process loop
    sn = signum;                                                       // save the signum to print it 
    // exit(signum); 
    // Re-register signal handler functions
    signal(SIGUSR1, SignalHandler);	
    signal(SIGUSR2, SignalHandler);	
    signal(SIGINT, signalHandler);
    signal(SIGQUIT, signalHandler);
    signal(SIGTSTP, signalHandler);
    signal(SIGTERM, signalHandler);
    signal(SIGKILL, signalHandler);
    signal(SIGSTOP, signalHandler);
    signal(SIGFPE, signalHandler);
    signal(SIGSEGV, signalHandler);
    signal(SIGABRT, signalHandler);	
}

int main(int argc, char **argv)
{

	cv::VideoCapture cap(0);
	cv::Mat frame;
	cv::Mat result(480, 640, CV_8UC1);
    int platf = 0;
	int devi = 0;

    // handle signal interrupts to ensure we are releasing memory
    /*
    SIGINT	Interrupts from the keyboard	Press Ctrl + C
    SIGQUIT	Cancel by keyboard	Press Ctrl+" or Ctrl+'4'
    SIGTSTP	Temporarily stopped input from terminal (tty)	Press Ctrl + Z
    SIGTERM	End	(Command) kill pid
    SIGKILL	Force quit	(Command) kill -KILL pid
    SIGSTOP	Pause kill -s SIGSTOP pid
    */
    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGINT \n";			
    }
    if (signal(SIGQUIT, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGQUIT \n";	
    }
    if (signal(SIGTSTP, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGTSTP \n";	
    }
    if (signal(SIGTERM, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGTERM \n";
    }
    if (signal(SIGKILL, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGKILL \n";
    }
    if (signal(SIGSTOP, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGSTOP \n";
    }
	
    // prog errors
    if (signal(SIGFPE, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGINT \n";			
    }
    if (signal(SIGSEGV, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGSEGV \n";	
    }
	// user abort
    if (signal(SIGABRT, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGABRT \n";	
    }
	
    // user defined signals 
    if (signal(SIGUSR1, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGUSR1 \n";			
    }
    if (signal(SIGUSR2, signalHandler) == SIG_ERR) {
        std::cout << "Interrupt signal failed to bind for SIGUSR2 \n";	
    }	

    // show available platforms and devices and choose the one we are going to use
    try {
        std::vector<cl::Platform> platforms;
        cl::Platform::get(&platforms);
        if (platforms.size() == 0) {
            std::cerr << "No platform found." << std::endl;
            return 1;
        }
        int i = 0;
        for (auto& platform : platforms) {
            std::cout << "Platform #" << i << ":" << std::endl;
            std::cout << "  Profile: " << platform.getInfo<CL_PLATFORM_PROFILE>() << std::endl;
            std::cout << "  Name: " << platform.getInfo<CL_PLATFORM_NAME>() << std::endl;
            std::cout << "  Vendor: " << platform.getInfo<CL_PLATFORM_VENDOR>() << std::endl;
            std::cout << "  Extensions: " << platform.getInfo<CL_PLATFORM_EXTENSIONS>() << std::endl;
            std::vector<cl::Device> devices;
            platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);
            int j = 0;
            for (auto& device : devices) {
                {
                    cl_device_type deviceType = device.getInfo<CL_DEVICE_TYPE>();
                    std::string deviceTypeStr = deviceType == CL_DEVICE_TYPE_CPU ? "CPU"
                        : deviceType == CL_DEVICE_TYPE_GPU ? "GPU"
                        : deviceType == CL_DEVICE_TYPE_ACCELERATOR ? "Accelerator"
                        : "unknown";
                    std::cout << "  Device #" << j << " (" << deviceTypeStr << "):" << std::endl;
                }
                std::cout << "    Name: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;
                std::cout << "    Vendor: " << device.getInfo<CL_DEVICE_VENDOR>() << std::endl;
                std::cout << "    Device Version: " << device.getInfo<CL_DEVICE_VERSION>() << std::endl;
                std::cout << "    Driver Version: " << device.getInfo<CL_DRIVER_VERSION>() << std::endl;
                std::cout << "    Extensions: " << device.getInfo<CL_DEVICE_EXTENSIONS>() << std::endl;
                std::cout << "    Max Compute Units: " << device.getInfo<CL_DEVICE_MAX_COMPUTE_UNITS>() << std::endl;
                std::cout << "    Preferred Vector Width (Float): " << device.getInfo<CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT>() << std::endl;
                std::cout << "    Preferred Vector Width (Double): " << device.getInfo<CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE>() << std::endl;
                ++j;
            }
            ++i;
        }
        cout << "Choose the platform number you want";
        cin >> platf;
        cout << "platform chosen is " << platf <<"\n";
        cout << "Choose the device number you want";
        cin >> devi;
        cout << "device chosen is " << platf <<"\n";
    } catch (cl::Error const& ex) {
        std::cerr << "OpenCL Error: " << ex.what() << " (code " << ex.err() << ")" << std::endl;
        return 1;
    } catch (std::exception const& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
	
	OpenCL::initialize(0, 0, platf, devi);                   // initialise the openCL platform and device
	
	if (!cap.isOpened())                                     // couldnt open video stream then throw SIGABRT to go to clean up
		throw std::exception();

	while (!eflag)
	{
		do                                                   // wait until frame ready then grab it and proceed
		{
			cap >> frame;
		} while (frame.empty());

		OpenCL::detectLine(result.data, frame.data);         // apply line segment detection from the openCL Kernel
		cv::imshow("test", result);                          // show the result
		unsigned char key = cv::waitKey(1);
		if (key == '\x1b')                                   // key '\x1b' exits
			break;
	}
	OpenCL::release();                                      // free the memory
	if (g_set_gs == 1) {
        free(g_sourceString);
		g_sourceString = NULL;
    }      
	if (sn != 0) {
        std::cout << "Interrupt signal (" << sn << ") received. prog has cleaned up\n";	
    }
    return 0;
}
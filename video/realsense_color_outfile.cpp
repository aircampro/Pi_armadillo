// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 RealSense, Inc. All Rights Reserved.
//
// write video file output from color frame
// get x,y,z for a given pixel position x,y and send over OSC to Unreal Engine
//
// $ g++ -std=c++11 -o realsense_color_outfile realsense_color_outfile.cpp `pkg-config --cflags opencv` `pkg-config --cflags realsense2` `pkg-config --libs realsense2` `pkg-config --libs opencv`
//

/*
CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
project(realsense_color_outfile CXX)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall")

find_package(PkgConfig)
pkg_check_modules(RealSense2 REQUIRED realsense2)
pkg_check_modules(OpenCV REQUIRED opencv)
include_directories(${RealSense2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${RealSense2_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

add_executable(realsense_color_outfile realsense_color_outfile.cpp)
target_link_libraries(realsense_color_outfile ${RealSense2_LIBRARIES} ${OpenCV_LIBRARIES})
*/

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#if _DEBUG
#pragma comment(lib, "opencv_world470d.lib")
#else
#pragma comment(lib, "opencv_world470")
#endif

#include <iostream>
#include <memory>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
#include "opencv2/highgui/highgui.hpp"

//---------------------------------------------------------------------------------------
//  Finally, the coordinates of the target are distributed to UE4 (Unreal Engine) by OSC
//	ref:- https://note.com/prty/n/n4549edcef3e4?creator_urlname=prty
//----------------------------------------------------------------------------------------
#include "OscReceivedElements.h"
#include "OscPacketListener.h"
#include "OscOutboundPacketStream.h"
#include "UdpSocket.h"
#include "IpEndpointName.h"
void sendOSCMessage(int id, float x, float y, float z)
{
    // Set IPAddress and Port
    const std::string ipAddress = "127.0.0.1";
	const int port = 8000;
 
	UdpTransmitSocket transmitSocket(IpEndpointName(ipAddress.c_str(), port));
	//Buffer
	char buffer[6144];
	osc::OutboundPacketStream p(buffer, 6144);
	p << osc::BeginBundleImmediate
		// Head
		<< osc::BeginMessage("/position") << id << x << y << z << osc::EndMessage
		<< osc::EndBundle;
	transmitSocket.Send(p.Data(), p.Size());
}

bool run_prog = true;

void signal_callback_handler(int signum)
{
   std::cout << "Caught signal " << signum << std::endl;
   run_prog = false;
}

void signal_cont_handler(int signum)
{
   std::cout << "Continue signal " << signum << std::endl;
   run_prog = true;
}

int main(int argc, char * argv[]) try
{
    // declare handlers for clean-up 
    signal(SIGINT, signal_callback_handler);                     // ctl-calculate
    signal(SIGILL, signal_callback_handler);                     
    signal(SIGQUIT, signal_callback_handler);                    
    signal(SIGHUP, signal_callback_handler);                
    signal(SIGTRAP, signal_callback_handler);           
    signal(SIGABRT, signal_callback_handler);               
    signal(SIGBUS, signal_callback_handler);              
    signal(SIGFPE, signal_callback_handler);                   
    signal(SIGUSR1, signal_callback_handler);                   // user kills
    signal(SIGUSR2, signal_callback_handler);  
    signal(SIGSEGV, signal_callback_handler);  
    signal(SIGPIPE, signal_callback_handler);  
    signal(SIGTERM, signal_callback_handler);  
    signal(SIGSTKFLT, signal_callback_handler);  
    signal(SIGSTOP, signal_callback_handler);  
    signal(SIGTSTP, signal_callback_handler);  
    signal(SIGXCPU, signal_callback_handler); 
    signal(SIGSYS, signal_callback_handler);  
    signal(SIGPWR, signal_callback_handler); 
    signal(SIGCONT, signal_cont_handler); 

    int    fourcc, width, height;
    double fps;
    width = 1920;	
    height = 1080;	
    fps = 30.0;					
    fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');	
    VideoWriter Videowriter("rs_output.mp4", fourcc, fps * 2, cv::Size(width, height), true);

    int    fourcc2, width2, height2;
    double fps2;
    width2 = 1280;	
    height2 = 720;	
    fps2 = 30.0;					
    fourcc2 = cv::VideoWriter::fourcc('m', 'p', '4', 'v');	
    VideoWriter Videowriter2("rs_depth.mp4", fourcc2, fps2 * 2, cv::Size(width2, height2), true);

    int    fourcc3, width3, height3;
    double fps3;
    width3 = 832;	
    height3 = 468;	
    fps3 = 30.0;					
    fourcc3 = cv::VideoWriter::fourcc('m', 'p', '4', 'v');	
    VideoWriter Videowriter3("rs_combined.mp4", fourcc3, fps3 * 2, cv::Size(width3, height3), true);
	
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, width2, height2, RS2_FORMAT_Z16, fps2);
	
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with configuration
    auto profile = pipe.start(cfg);
    auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto depth_intr = depth_stream.get_intrinsics();

    while ((run_prog == true) && (cv::waitKey(1) == -1))
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        rs2::align align(RS2_STREAM_COLOR);
        auto aligned_frames = align.process(frames);

        //auto color = frames.get_color_frame();
        auto color = aligned_frames.first(RS2_STREAM_COLOR);
	
        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color)
            color = frames.get_infrared_frame();

        // get color frame and write out to the video file
        // pc.map_to(color);
        cv::Mat colorf(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        Videowriter << colorf;

        // get depth frame and write out to the video file
        rs2::colorizer color_map;
        auto depth_frame = color_map(aligned_frames.get_depth_frame());
        cv::Mat depthf(cv::Size(width2, height2), CV_8UC3, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        Videowriter2 << depthf;

        // re-size colot frame and combine with depth show and spool to file
        cv::Mat colorf(cv::Size(width2, height2), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat disp(colorf.rows + depthf.rows, std::max(colorf.cols, depthf.cols), CV_8UC3);
        colorf.copyTo(cv::Mat(disp, cv::Rect(0, 0, colorf.cols, colorf.rows)));
        depthf.copyTo(cv::Mat(disp, cv::Rect(0, colorf.rows, depthf.cols, depthf.rows)));
        cv::resize(disp, disp, cv::Size(), 0.65, 0.65);
        cv::namedWindow("disp", cv::WINDOW_AUTOSIZE);
        cv::imshow("disp", disp);	
        Videowriter3 << disp;
		
        // Generate the pointcloud and texture mappings from the depth frame
		// auto depth = frames.get_depth_frame();
        // points = pc.calculate(depth);

        // get 3d-coordinate info from specified pixel position. e.g. 600, 500 then send it over OSC 
        float pixel[2] = {600.0, 500.0};
        float depth = depth_frame.get_distance(pixel[0], pixel[1]);
        float point[3];
        rs2_deproject_pixel_to_point(point, &depth_intr, pixel, depth);
		sendOSCMessage(1, point[0], point[1], point[2]);
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

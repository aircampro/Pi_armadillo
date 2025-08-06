
/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ref:- https://github.com/HesaiTechnology/Pandar40_SDK/tree/master
 * ACP modified the lidar callback to print the structure information
 *
 *****************************************************************************/

#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include <iostream>
#include <csignal>
#ifdef __linux__ 
    #include <signal.h>
#elif _WIN32
    #include <windows.h>
#else

#endif

volatile int g_a = 0;

#ifdef __linux__ 
void sig_handler(int signo) 
{
  std::cout << "received signal: " << signo << "\n";
  g_a = 1;
}
#elif _WIN32
BOOL WINAPI consoleCtrlHandler(DWORD ctrlType) {
   switch (ctrlType) {
      case CTRL_C_EVENT:
      std::cout << "Ctrl+C signal received." << std::endl;
	  g_a = 1;
      return TRUE;
      case CTRL_BREAK_EVENT:
      std::cout << "Ctrl+Break signal received." << std::endl;
	  g_a = 1;
      return TRUE;
      default:
      return FALSE;
    }
}
#else

#endif

void gpsCallback(int timestamp) {
  printf("gps: %d", timestamp);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
  printf("Frame timestamp: %lf,\n", timestamp);
  printf("point_size: %ld,\n",cld->points.size());
  printf("width: %ld,\n",cld->width);
  printf("height: %ld,\n",cld->height);
  printf("dense: %ld,\n",static_cast<int>(cld->is_dense));
  std::cout << "sensor origin (xyz): [" << cld->sensor_origin_.x () << ", " << cld->sensor_origin_.y () << ", " << cld->sensor_origin_.z () << "]" >> std::endl; 
  std::cout << "orientation (xyzw): [" << cld->sensor_orientation_.x () << ", " << cld->sensor_orientation_.y () << ", " << cld->sensor_orientation_.z () << ", " << cld->sensor_orientation_.w () << "]" << std::endl;
}

int main(int argc, char** argv) {
  // PandarGeneralSDK pandarGeneral(std::string("192.168.1.201"), 2368, 10110, \
  //     lidarCallback, gpsCallback, 0, 0, 1, std::string("Pandar40P"));
  /* example panda40
      HesaiLidarSDK pandarGeneral(
	8080				// lidar data port //, 
	8308				// gps data port //, 
        0                               // start angle//,
	std::string("correction.csv")	// calibration file of lidar //, 
	lidarCallback 			// point cloud data call back //, 
	gpsCallback 			// gps data callback //, 
	HESAI_LIDAR_RAW_DATA_STRCUT_SINGLE_RETURN// Return Mode: Single Return data structure //, 
	40				// laser counter //, 
	HESAI_LIDAR_PCL_DATA_TYPE_REDUCED// pcl data alignment //
	);
   */
   /* pandora
     HesaiLidarSDK pandarGeneral(
    std::string("192.168.20.51"), 
    9870, 
    std::string("calibration.yml"), 
    cameraCallback,
    2368, 10110, 0,
    std::string("correction.csv"),
    lidarCallback, gpsCallback,
    HESAI_LIDAR_RAW_DATA_STRCUT_DUAL_RETURN, 40, HESAI_LIDAR_PCL_DATA_TYPE_REDUCED);
	*/
   
#ifdef __linux__ 
  signal(SIGUSR1, sig_handler);
  signal(SIGUSR2, sig_handler);
  signal(SIGINT, sig_handler);
#elif _WIN32  
  if (!SetConsoleCtrlHandler(consoleCtrlHandler, TRUE)) {
      std::cerr << "Error setting console control handler." << std::endl;
      return 1;
  }
#else

#endif
	
  PandarGeneralSDK pandarGeneral(std::string("/path/to/pcapFile"), \
  lidarCallback, 0, 0, 1, std::string("PandarXT-16"));
  std::string filePath = "/path/to/correctionFile";
  std::ifstream fin(filePath);
  int length = 0;
  std::string strlidarCalibration;
  fin.seekg(0, std::ios::end);
  length = fin.tellg();
  fin.seekg(0, std::ios::beg);
  char *buffer = new char[length];
  fin.read(buffer, length);
  fin.close();
  strlidarCalibration = buffer;
  pandarGeneral.LoadLidarCorrectionFile(strlidarCalibration);

  pandarGeneral.Start();

  while (g_a == 0) {
    sleep(100);
  }
  
  delete[] buffer;
  return 0;
}
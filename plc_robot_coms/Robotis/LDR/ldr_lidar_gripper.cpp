/**
 * @file ldr_lidar_gripper.cpp
 * @author LDRobot (marketing1@ldrobot.com) and aircampro
 * for LDROBOT driver libraries https://github.com/ldrobotSensorTeam/ldlidar_stl_ros/tree/master
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * lidar sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @lidar copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. Air Cam Pro Ltd and Dyanmixel Robotis All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 
 we are using the robotis RH-P12-RN(A) gripper to grab the object seen in range and release when 
 in another range of the object
 https://emanual.robotis.com/docs/en/platform/rh_p12_rna/
 YM080-230-R099-RH is used in froward and reverse directions 
 https://emanual.robotis.com/docs/en/dxl/y/ym080-230-r099-rh/
 */
#include <limits>
#include "ros_api.h"
#include "ldlidar_driver.h"

// if the intensity is above this then accept the reading
#define INTENSITY_ENOUGH 50
// range to see to grab the object
#define MIN_RANGE_GRAB 0.050f
#define MAX_RANGE_GRAB 0.1000f
// range to see to drop the object
#define MIN_RANGE_RELEASE 5.000f
#define MAX_RANGE_RELEASE 5.500f
// time step of movement
#define MOVE_DURATION 500                     // motor on move period ns
#define MOVE_DURATION_STEP MOVE_DURATION*2    // motor off duration
#define MOVE_DURATION_TOTAL MOVE_DURATION*20   // total distance catch

#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"                                   // Uses DYNAMIXEL SDK library

// Robotis RH-P12-RN(A) gripper hand
#define HAND_ID 1
#define HAND_POSITION_CTL_REG 564
#define HAND_POSITION_FB_REG 580
#define HAND_OPEN 0
#define HAND_CLOSED 740
#define HAND_TORQ_REG 512

// shutdown control for hand
#define SHUTDWN_CTL_REG 63
#define HARDWARE_STAT_REG 518                 // manual also suggests 70
#define HW_ALERT_WORD 0x80
#define SHUTDWN_ON_IN_VOLT 1
#define SHUTDWN_ON_OVRHEAT 1<<2
#define SHUTDWN_ON_MTR_ENC 1<<3
#define SHUTDWN_ON_SHOCK 1<<4
#define SHUTDWN_ON_OVERLOAD 1<<5
#define ALL_SHUTDWN_ACTIVE (SHUTDWN_ON_IN_VOLT | SHUTDWN_ON_OVRHEAT | SHUTDWN_ON_MTR_ENC | SHUTDWN_ON_SHOCK | SHUTDWN_ON_OVERLOAD)

// external port controls
#define P1_MODE 56
#define P2_MODE 57
#define P3_MODE 58
#define P4_MODE 59
#define P11_DATA 600
#define P12_DATA 601
#define P21_DATA 602
#define P22_DATA 603
#define P31_DATA 604
#define P32_DATA 605
#define P41_DATA 606
#define P42_DATA 607
#define ANI 0
#define DOT 1
#define DINPU 2
#define DINPD 3

// Robotis YM080-230-R099-RH fwd/rev drive
#define DRIVE_ID 2
#define DRIVE_MODE_CTL_REG 32
#define DRIVE_FWD(s) (s&0xFFFE) 
#define DRIVE_RVS(s) (s|0x1)
#define DRIVE_FWD_BIT 0
#define DRIVE_RVS_BIT 1
#define DRIVE_VELO_CTL_REG 528
#define DRIVE_VELO_SP 3000                                 // 30 rpm
#define DRIVE_TORQ_CTL_REG 512
#define DXL_MOVING_STATUS_THRESHOLD 20

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_VELO_STATUS_THRESHOLD     20                    // Dynamixel moving status threshold

#define TORQ_AFTER_RST_DLY 50000                            // reset bind time after reboot

#define OBJECT_FROM_REGION 1
#define OBJECT_TO_REGION 4

// the motion and exception steps for the robot
typedef enum
{
    move_fwd,
    close_hand,
    wait_for_cls,
    move_rev,
    open_hand,
    wait_for_open,
    shutdwn_actv,
    shutdwn_release,
    manually_control,
    stop_drive,
    fail_needs_reset
} MotionSteps_e;

// for exception handlers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
// can also be 
// #include <signal.h>
// #include <stdexcept>
#define NO_SIGHANDLER 0                              // dont define signal handler (you have no manual mode)
#define USE_SIGNAL 1                                 // use signal - maybe be deprecated for many
#define USE_SIGACTION 2                              // use sigaction 
#define SIGNAL_HANDLER USE_SIGACTION

int ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, LaserScanSetting& setting, ros::Publisher& lidarpub);
uint64_t GetSystemTimeStamp(void);
#ifdef (SIGNAL_HANDLER == USE_SIGNAL) || (SIGNAL_HANDLER == USE_SIGACTION)
static void SignalHandler(int signo);
#endif

int grab_the_object = MotionSteps_e::move_fwd;
int saved_step = 0;
int stop_running = 0;

int main(int argc, char **argv) {

#ifdef (SIGNAL_HANDLER == USE_SIGNAL) 
  signal(SIGINT, SignalHandler);
  signal(SIGCONT, SignalHandler);
  signal(SIGUSR1, SignalHandler);
  signal(SIGUSR2, SignalHandler);
  signal(SIGTSTP, SignalHandler);
#elif (SIGNAL_HANDLER == USE_SIGACTION)
  // set up signal handler
  struct sigaction act;
  memset(&act, 0, sizeof act);
  act.sa_handler = SignalHandler;
  
  sigemptyset(&act.sa_mask);               /* Clear mask.． */  
  sigaddset(&act.sa_mask, SIGINT);
  act.sa_flags = SA_RESTART;               /* Compatible with BSD (magic).． */
  /* Experiment: Clear the configuration once the signal handler is called */
  /* act.sa_flags = action.sa_flags | SA_ONESHOT; */ 

  if (sigaction(SIGINT, &act, NULL)<0) {
    perror("SIGINT sigaction()");
    exit(EXIT_FAILURE);
  }
  if (sigaction(SIGCONT, &act, NULL)<0) {
    perror("SIGCONT sigaction()");
    exit(EXIT_FAILURE);
  }
  if (sigaction(SIGUSR1, &act, NULL)<0) {
    perror("SIGUSR1 sigaction()");
    exit(EXIT_FAILURE);
  }
  if (sigaction(SIGUSR2, &act, NULL)<0) {
    perror("SIGUSR2 sigaction()");
    exit(EXIT_FAILURE);
  }
  if (sigaction(SIGTSTP, &act, NULL)<0) {
    perror("SIGTSTP sigaction()");
    exit(EXIT_FAILURE);
  }  
#endif

  // initialise ROS
  ros::init(argc, argv, "ldldiar_publisher");
  ros::NodeHandle nh;                                          // create a ROS Node
  ros::NodeHandle nh_private("~");
  std::string product_name;
  std::string topic_name;
  std::string port_name;
  int serial_port_baudrate;
  LaserScanSetting setting;
  ldlidar::LDType type_name;
  int lidar_grab = 0;
  
  nh_private.getParam("product_name", product_name);
  nh_private.getParam("topic_name", topic_name);
  nh_private.param("frame_id", setting.frame_id, std::string("base_laser"));
  nh_private.getParam("port_name", port_name);
  nh_private.param("port_baudrate", serial_port_baudrate, int(230400));
  nh_private.param("laser_scan_dir", setting.laser_scan_dir, bool(true));
  nh_private.param("enable_angle_crop_func", setting.enable_angle_crop_func, bool(false));
  nh_private.param("angle_crop_min", setting.angle_crop_min, double(0.0));
  nh_private.param("angle_crop_max", setting.angle_crop_max, double(0.0));

  // initialise the lidar class
  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();

  ROS_INFO("LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());
  ROS_INFO("ROS params input:");
  ROS_INFO("<product_name>: %s", product_name.c_str());
  ROS_INFO("<topic_name>: %s", topic_name.c_str());
  ROS_INFO("<frame_id>: %s", setting.frame_id.c_str());
  ROS_INFO("<port_name>: %s", port_name.c_str());
  ROS_INFO("<port_baudrate>: %d", serial_port_baudrate);
  ROS_INFO("<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  ROS_INFO("<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  ROS_INFO("<angle_crop_min>: %f", setting.angle_crop_min);
  ROS_INFO("<angle_crop_max>: %f", setting.angle_crop_max);

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06; 
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else {
    ROS_ERROR("Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); 

  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    ROS_INFO("ldlidar node start is success");
  } else {
    ROS_ERROR("ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (ldlidarnode->WaitLidarCommConnect(3000)) {
    ROS_INFO("ldlidar communication is normal.");
  } else {
    ROS_ERROR("ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }
 
  ros::Publisher lidar_pub =  nh.advertise<sensor_msgs::LaserScan>(topic_name, 10);  // create a ROS topic

  // connect to the drive and the hand
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);

  // Initialize PacketHandler Structs
  packetHandler();

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 100;             // Present position
  int32_t shutdown_state = 0;
  
  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return 0;
  }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return 0;
  }

  // Enable Dynamixel Torque for hand and drive
  write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HAND_TORQ_REG, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  else
  {
    printf("Dynamixel 1 has been successfully connected \n");
  }
  write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_TORQ_CTL_REG, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  else
  {
    printf("Dynamixel 2 has been successfully connected \n");
  }

  // set conditions which will shutdown the hand if active 
  write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, SHUTDWN_CTL_REG, ALL_SHUTDWN_ACTIVE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }

  // set up the external DOT port
  write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, P1_MODE, DOT);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  
  ros::Rate r(10); //10hz
  ldlidar::Points2D laser_scan_points;
  double lidar_scan_freq;
  ROS_INFO("Publish topic message:ldlidar scan data .");
  uint64_t st_tm = GetSystemTimeStamp(void);
  uint64_t act_tm = 0;
  uint64_t rs_st_tm = GetSystemTimeStamp(void);
  uint64_t rs_act_tm = 0;

  // initialize the sequence to the current operating condition
  // Read present position
  dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HAND_POSITION_FB_REG);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  // hand is closed
  if ((abs(HAND_CLOSE - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD))
  {	
      // set the drive to forward directions
      write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_MODE_CTL_REG, DRIVE_RVS_BIT);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
          printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
         printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }	
      grab_the_object = MotionSteps_e::move_rev;	
  } else if ((abs(HAND_OPEN - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD))             // hand is open
      // set the drive to forward directions
      write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_MODE_CTL_REG, DRIVE_FWD_BIT);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
          printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
         printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }	
      grab_the_object = MotionSteps_e::move_fwd;
  }	else {                                                                                     // hand is intermediate
      grab_the_object = MotionSteps_e::close_hand;
  }	  

 // -------------- main loop continuous -----------------------
 try
 {  
   while (((ros::ok())&&(stop_running==0))&&ldlidar::LDLidarDriver::IsOk()) {
	  
    // read the lidar and publish to the ROS topic
    switch (ldlidarnode->GetLaserScanData(laser_scan_points, 1500)){
      case ldlidar::LidarStatus::NORMAL: 
        ldlidarnode->GetLidarScanFreq(lidar_scan_freq);
	// publish to ROS and check if we need to close the grabber
        lidar_grab = ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting, lidar_pub);
        if ((grab_the_object==MotionSteps_e::move_fwd) && (lidar_grab == OBJECT_FROM_REGION)) {
            grab_the_object = lidar_grab;   
        else if ((grab_the_object==MotionSteps_e::move_rev) && (lidar_grab == OBJECT_TO_REGION)) {		
            grab_the_object = lidar_grab;
        }			
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        ROS_ERROR("get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }
		
	// move the robot and grab using the grabber between the 2 lidar ranges
    switch (grab_the_object){
      // move fotward MotionSteps_e::move_fwd
      case MotionSteps_e::move_fwd:
          act_tm = GetSystemTimeStamp(void);
          if ((act_tm - st_tm) > MOVE_DURATION) {
	    // stop motor zero velocity
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_VELO_CTL_REG, 0);
            if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
               printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
            }
            else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
               printRxPacketError(PROTOCOL_VERSION, dxl_error);
            }
            if ((act_tm - st_tm) > MOVE_DURATION_TOTAL) {	        // it times out wihout the lidar seeing the mark
		        grab_the_object=MotionSteps_e::fail_needs_reset;                                  // go to a alarm with reset step (eanble manual control)
            } else if ((act_tm - st_tm) > MOVE_DURATION_STEP) {				
		        st_tm = GetSystemTimeStamp(void);                   // reset the time and iterate another step
            }
	} else {
            // reverse direction at velocity specified for the time duration of movement in a step
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_VELO_CTL_REG, DRIVE_VELO_SP);
            if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
               printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
            }
            else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
               printRxPacketError(PROTOCOL_VERSION, dxl_error);
            }			
        }
        break;
      // set the hand position to close MotionSteps_e::close_hand
      case MotionSteps_e::close_hand: 
        write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HAND_POSITION_CTL_REG, HAND_CLOSE);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
           printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
           printRxPacketError(PROTOCOL_VERSION, dxl_error);
        }
	grab_the_object=MotionSteps_e::wait_for_cls;
        break;
      // wait until the posiion feedback shows closed MotionSteps_e::wait_for_cls
      case MotionSteps_e::wait_for_cls:
        if ((abs(HAND_CLOSE - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD))
        {
           // Read present position
          dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HAND_POSITION_FB_REG);
          if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
          {
             printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
          }
          else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
          {
             printRxPacketError(PROTOCOL_VERSION, dxl_error);
          }
          printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", HAND_ID, HAND_CLOSE, dxl_present_position);
           // Read shutdown status if we shutdown it needs reboot and manual interaction
          shutdown_state = read4ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HARDWARE_STAT_REG);
          if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
          {
             printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
          }
          else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
          {
             printRxPacketError(PROTOCOL_VERSION, dxl_error);
          }
          if (((shutdown_state & HW_ALERT_WORD) & ALL_SHUTDWN_ACTIVE) > 0) {
              saved_step = grab_the_object;
              grab_the_object = MotionSteps_e::shutdwn_actv;			  
          }			  
        } else {
          st_tm = GetSystemTimeStamp(void);
          grab_the_object=MotionSteps_e::move_rev;
           // revese dirction of drive
          write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_MODE_CTL_REG, DRIVE_RVS_BIT);
          if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
          {
             printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
          }
          else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
          {
             printRxPacketError(PROTOCOL_VERSION, dxl_error);
          }
        }
        break;
	  // move back we wait until the lidar jumps us to the next step -- if it doesnt see the stop stop after exceeding the time MotionSteps_e::move_rev
      case MotionSteps_e::move_rev:
        act_tm = GetSystemTimeStamp(void);
        if ((act_tm - st_tm) > MOVE_DURATION) {
            // stop motor velocity
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_VELO_CTL_REG, 0);
            if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
               printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
            }
            else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
               printRxPacketError(PROTOCOL_VERSION, dxl_error);
            }
            if ((act_tm - st_tm) > MOVE_DURATION_TOTAL) {	        // it times out wihout the lidar seeing the mark
               grab_the_object=MotionSteps_e::fail_needs_reset;    // go to a alarm with reset step (eanble manual control)
            } else if ((act_tm - st_tm) > MOVE_DURATION_STEP) {				
                st_tm = GetSystemTimeStamp(void);                   // reset the time and iterate another step
            }
       } else {
            // reverse direction at velocity specified for the time duration of movement in a step
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_VELO_CTL_REG, DRIVE_VELO_SP);
            if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
               printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
            }
            else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
               printRxPacketError(PROTOCOL_VERSION, dxl_error);
            }			
        }
	break;
      // set the hand to open MotionSteps_e::open_hand
      case MotionSteps_e::open_hand:
        write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HAND_POSITION_CTL_REG, HAND_OPEN);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
           printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
           printRxPacketError(PROTOCOL_VERSION, dxl_error);
        }
        grab_the_object=MotionSteps_e::wait_for_open;
        break;
      // wait until the posiion feedback shows open MotionSteps_e::wait_for_open
      case MotionSteps_e::wait_for_open:
        if ((abs(HAND_OPEN - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD))
        {
           // Read present position
          dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HAND_POSITION_FB_REG);
          if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
          {
             printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
          }
          else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
          {
             printRxPacketError(PROTOCOL_VERSION, dxl_error);
          }
          printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", HAND_ID, HAND_CLOSE, dxl_present_position);
           // Read shutdown status if we shutdown it needs reboot and manual interaction
          shutdown_state = read4ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HARDWARE_STAT_REG);
          if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
          {
             printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
          }
          else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
          {
             printRxPacketError(PROTOCOL_VERSION, dxl_error);
          }
          if (((shutdown_state & HW_ALERT_WORD) & ALL_SHUTDWN_ACTIVE) > 0) {
              saved_step = grab_the_object;
              grab_the_object = MotionSteps_e::shutdwn_actv;			  
          }
        } else {
          st_tm = GetSystemTimeStamp(void);
          grab_the_object=MotionSteps_e::move_fwd;
	  // set the drive to forward directions
          write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_MODE_CTL_REG, DRIVE_FWD_BIT);
          if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
          {
             printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
          }
          else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
          {
             printRxPacketError(PROTOCOL_VERSION, dxl_error);
          }	
        }		  
        break;
      // on timeout we need manual interaction --- alarm step ---
      case MotionSteps_e::fail_needs_reset:
        // wait for reset we timed out on lidar signal
        std::cout << "timed out on moving to target" << std::endl;
        std::cout << "enter step to advance to when ready" << std::endl;
        int my_input;
        std:cin << my_input;
        grab_the_object = my_input;
        break;
      // allow manual control via signal interrupt
      case MotionSteps_e::manually_control:
          std::cout << "enter step to advance to when ready...." << std::endl;
          int my_input;
          std:cin << my_input;
          grab_the_object = my_input;
          break;
      case MotionSteps_e::stop_drive:
        // stop motor velocity
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DRIVE_ID, DRIVE_VELO_CTL_REG, 0);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
           printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
           printRxPacketError(PROTOCOL_VERSION, dxl_error);
        }
        std::cout << "enter step to advance to when ready...." << std::endl;
        int my_input;
        std:cin << my_input;
        grab_the_object = my_input;
        break;		
      // drive detected shutdown exception step action
      case MotionSteps_e::shutdwn_actv:
        // re-enable torque on the drive after reboot (perhaps consider power-off contact)
        // power a light to show state via external DOT 1
        write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, P11_DATA, 1);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
           printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
           printRxPacketError(PROTOCOL_VERSION, dxl_error);
        }
        std::cout << "timed out on moving to target press any key to continue and return back...." << std::endl;
        int my_input;
        std:cin << my_input;
        // release the output and after a time torque the drive
        rs_st_tm = GetSystemTimeStamp(void);
        write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, P11_DATA, 0);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
           printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
           printRxPacketError(PROTOCOL_VERSION, dxl_error);
        }
        grab_the_object = MotionSteps_e::shutdwn_release;
        break;
      case MotionSteps_e::shutdwn_release:		
         rs_act_tm = GetSystemTimeStamp(void);
         if ((rs_act_tm - rs_st_tm) > TORQ_AFTER_RST_DLY) {
            write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HAND_TORQ_REG, TORQUE_ENABLE);
            if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
               printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
            }
            else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
            {
               printRxPacketError(PROTOCOL_VERSION, dxl_error);
            }	
            grab_the_object = saved_step;                                     // return the step we branched out of
	  }
          break;
      default:
        std::cout << "invalid step encountered enter correct step to continue...." << std::endl;
        int my_input;
        std:cin << my_input;
        grab_the_object = my_input;
        break;
    }
    r.sleep();
   }
 catch(std::exception &exception)
 {
    std::cerr << std::endl << "EXCEPTION : " << exception.what() << std::endl;
 }
  // stop the lidar and free the memory
  ldlidarnode->Stop();
  delete ldlidarnode;
  ldlidarnode = nullptr;

  // untorque the drive
  write1ByteTxRx(port_num, PROTOCOL_VERSION, HAND_ID, HAND_TORQ_REG, TORQUE_DISABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
      printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
      printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
		
  return EXIT_SUCCESS;
}

int ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, LaserScanSetting& setting, ros::Publisher& lidarpub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  float scan_time;
  ros::Time start_scan_time;
  static ros::Time end_scan_time;
  static bool first_scan = true;
  int in_range = 0;                                  // nothing is in range
  
  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }

  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  range_min = 0.02;
  range_max = 12;
  int beam_size = static_cast<int>(src.size());
  angle_increment = (angle_max - angle_min) / (float)(beam_size -1);

  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    sensor_msgs::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    if (beam_size <= 1) {
      output.time_increment = 0;
    } else {
      output.time_increment = scan_time / (float)(beam_size - 1);
    }
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    for (auto point : src) {
      float range = point.distance / 1000.f;                // distance unit transform to meters
      float intensity = point.intensity;                    // laser receive intensity 
      float dir_angle = point.angle;

      if (setting.enable_angle_crop_func) {                 // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle);            // Lidar angle unit form degree transform to radian
      int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
	  	  
      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }
	  
      if (index < beam_size) {
        if (index < 0) {
          ROS_ERROR("[ldrobot] error index: %d, beam_size: %d, angle: %f, angle_min: %f, angle_increment: %f", 
              index, beam_size, angle, angle_min, angle_increment);
        } else {
          if ((( range >= MIN_RANGE_GRAB ) && ( range <= MAX_RANGE_GRAB )) && (intensity >= INTENSITY_ENOUGH)) {
              in_range = OBJECT_FROM_REGION;                      // we are in of range of the object
          } else if (((( range >= MIN_RANGE_RELEASE ) && ( range <= MAX_RANGE_RELEASE )) && (intensity >= INTENSITY_ENOUGH)) && (in_range != OBJECT_FROM_REGION)) {
              in_range = OBJECT_TO_REGION;                        // we are out of range of the object in a drop region
          }	
        }

        if (setting.laser_scan_dir) {
          int index_anticlockwise = beam_size - index - 1;
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index_anticlockwise])) {
            output.ranges[index_anticlockwise] = range;
          } else { // Otherwise, only when the distance is less than the current
                    //   value, it can be re assigned
            if (range < output.ranges[index_anticlockwise]) {
                output.ranges[index_anticlockwise] = range;
            }
          }
          output.intensities[index_anticlockwise] = intensity;
        } else {
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index])) {
            output.ranges[index] = range;
          } else { // Otherwise, only when the distance is less than the current
                  //   value, it can be re assigned
            if (range < output.ranges[index]) {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = intensity;
        }
      }
    }
    lidarpub.publish(output);
    end_scan_time = start_scan_time;
  } 
  return in_range;
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
  std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

#ifdef (SIGNAL_HANDLER == USE_SIGNAL) 
static void SignalHandler(int signo)
{
    printf(" ... Caught the signal number (%d)\n", signo);
	switch(signo) {
        case SIGINT:
            printf("Interrupt pressed!\n");
            //Re-register signal handler functions
            signal(SIGINT, SignalHandler);
            break;
        case SIGTSTP:
            printf("We have a break.\n");
            stop_running = 1;
            break;
        case SIGSEGV: 
            printf("We got a seg fault.\n");
            exit(EXIT_FAILURE);
            break;
        case SIGUSR1:                                                        // sets the robot to manual control (kill -SIGNAL1 <pid>
            saved_step = grab_the_object;
            grab_the_object = MotionSteps_e::manually_control;
            //Re-register signal handler functions
            signal(SIGUSR1, SignalHandler);
            break;
        case SIGCONT:
            grab_the_object = saved_step;
            //Re-register signal handler functions
            signal(SIGCONT, SignalHandler);
            break;	
        case SIGUSR2:                                                        // reset to move forward 
            saved_step = grab_the_object;
            grab_the_object = MotionSteps_e::stop_drive;
            //Re-register signal handler functions
            signal(SIGUSR2, SignalHandler);
            break;				
        default:
            printf("I don't know what to do for this exception.\n");
            break;
    }
}
#elif (SIGNAL_HANDLER == USE_SIGACTION)
// this handles signals such as shutdown or a kill -SIGNAL1 <pid> to put us in manaul operation mode
static void SignalHandler(int signo)
{
    printf(" ... Caught the signal number (%d)\n", signo);
	switch(signo) {
        case SIGINT:
            printf("Interrupt pressed!\n");
            break;
        case SIGTSTP:
            printf("We have a break.\n");
            exit(EXIT_SUCCESS);
            break;
        case SIGSEGV: 
            printf("We got a seg fault.\n");
            exit(EXIT_FAILURE);
            break;
        case SIGUSR1:                                                        // sets the robot to manual control (kill -SIGNAL1 <pid>
            saved_step = grab_the_object;
            grab_the_object = MotionSteps_e::manually_control;
            break;
        case SIGUSR2:
            saved_step = grab_the_object;
            grab_the_object = MotionSteps_e::stop_drive;
            break;
        case SIGCONT:
            grab_the_object = saved_step;
            break;			
        default:
            printf("I don't know what to do for this exception.\n");
            break;
    }
}
#endif

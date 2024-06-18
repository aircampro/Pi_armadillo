// --------------------------------------------------------------------------------------------------------------------------
//    Waving Hands Robot using a robotis protocol 2.0 SDK
//    analysing for example a Ricoh theta S 360 camera feed with yolov5 or yolov8 for object classifacation
//    looks for rabbits or birds in the field and waves hands
//
// -------------------------------------------------------------------------------------------------------------------------
//
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <unistd.h>

// include libs for external camera vision with yolo v5 or v8
#include "CamCv.h"
#include "ExpansionShader.h"            // selects camera type with its properties

#define CAPTURE_INPUT 0                 // 0 input from the number capture device
//#define CAPTURE_INPUT "sp360.mp4"     // Kodak SP360 4K の Fish Eye video example
//#define CAPTURE_INPUT "theta.mp4"     // THETA S の Equirectangular video example

// How to expand the background image (see ExpansionShader.h)
//constexpr int shader_selection(6);      // Kodak SP360 4K
constexpr int shader_selection(7);        // THETA S の Dual Fisheye image
//constexpr int shader_selection(2);      // THETA S の Equirectangular image

// Resolution of the camera used to obtain the background image (if 0, it will be obtained from the camera)
constexpr int capture_width(shader_type[shader_selection].width);
constexpr int capture_height(shader_type[shader_selection].height);

// Frame rate of the camera used to obtain the background image (if 0, get it from the camera)
constexpr int capture_fps(0);

#include <cmath>
constexpr float kPi_2 = (2.0f * std::atan(1.0f));

#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"                                   // Uses DYNAMIXEL SDK library

// Control table address
// for L42-10-S300-R
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_P_GAIN_POS             594
#define P_POS_GAIN_SLOW                 64
#define P_POS_GAIN_FAST                 500
#define ADDR_PRO_PRESENT_POSITION       611

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID_LEFT                          1                   // Dynamixel ID: 1
#define DXL_ID_RIGHT                         2                   // Dynamixel ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB2"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -150000             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

enum arm_states_e {
  // arm_states
  NO_WAVE,
  START_WAVE,
  FAST_WAVE
};

// global move hands state
int g_moveHands;
int loop_active = 0;

int main(int argc, char const *argv[]) {

  int lastHands = arm_states_e::NO_WAVE;

  // initialise the camera 
  CamCv camera;
  if (!camera.open(CAPTURE_INPUT, capture_width, capture_height, capture_fps))
  {
    throw std::runtime_error("Can't open capture device.");
  }

  // if the version of yolo was requested use it, if garbled then default it to v8, using stoi as better than atoi for human typed values
  if (argc >= 2 ) {
	try {
        camera.start(std::stoi(argv[2]), capture_width, capture_height);
	}
    catch (const std::invalid_argument& e) {
        std::cout << "[" << i << "]: " << "invalid argument :- options are 0=yolov5 1=yolov8 defaulting to use v8" << std::endl;
		camera.start(DRAWCYCLE_YOLO8, capture_width, capture_height);                     // for yolov8
    }
    catch (const std::out_of_range& e) {
         std::cout << "[" << i << "]: " << "out of range :- options are 0=yolov5 1=yolov8 defaulting to use v8" << std::endl;
		 camera.start(DRAWCYCLE_YOLO8, capture_width, capture_height);                     // for yolov8
    }
  } else {	  
    camera.start(DRAWCYCLE_YOLO8, capture_width, capture_height);                          // for yolov8
  }

  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);

  // Initialize PacketHandler Structs
  packetHandler();

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position max to min repeated

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position

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

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_LEFT, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
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
  write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_RIGHT, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
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
  
  // start main loop
  while (true) {

    // read and analyse the camera frame 
    camera.transmit();
    g_moveHands = camera.wave_hands;	

    // now move hands according to the vision stream	
	switch(g_moveHands) {
	  
      case arm_states_e::NO_WAVE:
	  {
	    if (lastHands != g_moveHands) {
            // diable arm ctrl
            loop_active = 0;	
		}
      }
	  break;
	  
	  case arm_states_e::START_WAVE:
      {
		// ------------- enable control if from otherstate ------------------------------
	    if (lastHands != g_moveHands) {		
            // enable arm ctrl
            loop_active = 1;
			p_band = P_POS_GAIN_SLOW;
		}
      }
	  break;
	  
	  case arm_states_e::FAST_WAVE:
      {
		// ------------- enable control if from otherstate ------------------------------
	    if (lastHands != g_moveHands) {		
            // enable arm ctrl
            loop_active = 1;
            p_band = P_POS_GAIN_FAST;
		}
      }
	  break;
	  
	}
	lastHands = g_moveHands;

    // do if we are commanded to do motion
    while (loop_active==1)
    {

      // Write proporitonal gain for position
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_LEFT, ADDR_PRO_P_GAIN_POS, p_band);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }
	  
      // Write goal position
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_LEFT, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index]);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }

      do
      {
        // Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_LEFT, ADDR_PRO_PRESENT_POSITION);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
          printRxPacketError(PROTOCOL_VERSION, dxl_error);
        }

        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_LEFT, dxl_goal_position[index], dxl_present_position);

      } while ((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

      // Write proporitonal gain for position
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_RIGHT, ADDR_PRO_P_GAIN_POS, p_band);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }
	  
      // Write goal position
      write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_RIGHT, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index]);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        printRxPacketError(PROTOCOL_VERSION, dxl_error);
      }

      do
      {
        // Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_RIGHT, ADDR_PRO_PRESENT_POSITION);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
          printRxPacketError(PROTOCOL_VERSION, dxl_error);
        }

        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID_RIGHT, dxl_goal_position[index], dxl_present_position);

      } while ((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
	  
      // Change goal position reverse the direction
      if (index == 0)
      {
        index = 1;
      }
      else
      {
        index = 0;
      }
	  
      // read and analyse another camera frame 
      camera.transmit();
      g_moveHands = camera.wave_hands;	

      // now move hands according to the vision stream	
	  switch(g_moveHands) {
	  
        case arm_states_e::NO_WAVE:
	    {
	      if (lastHands != g_moveHands) {
            // diable arm ctrl
            loop_active = 0;	
		  }
        }
	    break;
	  
	    case arm_states_e::START_WAVE:
        {
		  // ------------- enable control if from otherstate ------------------------------
	      if (lastHands != g_moveHands) {		
            // enable arm ctrl
            loop_active = 1;
			p_band = P_POS_GAIN_SLOW;
		  }
        }
	    break;
	  
	    case arm_states_e::FAST_WAVE:
        {
		  // ------------- enable control if from otherstate ------------------------------
	      if (lastHands != g_moveHands) {		
            // enable arm ctrl
            loop_active = 1;
            p_band = P_POS_GAIN_FAST;
		  }
        }
	    break;
	  
	  }
	  lastHands = g_moveHands;
    }	
	
  }

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_LEFT, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }

  write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_RIGHT, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  
  // Close port
  closePort(port_num);
  
  return 0;
}
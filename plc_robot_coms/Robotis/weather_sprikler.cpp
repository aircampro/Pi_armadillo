//
// sprinkle water or dispence grit, dependent on weather prediction from a cloud server currently defined as api.openweathermap.org 
//
#include<iostream>
#include<vector>
#include<string>
#include<cassert>
#include <unistd.h>
#include<curl/curl.h>

#include <stdio.h>
#include <string.h>

#include <memory>

#include <string>
#include <cstdio>
#include <stdexcept>
#include <vector>

// if we are using C++20 with format uncomment the define below this line
// #define C20_HAS_FORMAT
#ifdef C20_HAS_FORMAT
#include <format>
#endif

// for dynamixel drives
#include <stdlib>
#include "dynamixel_sdk.h"                                   // Uses DYNAMIXEL SDK library

// Robotis YM080-230-R099-RH fwd/rev drive sprinkler and pump (dot11) and gritter drive
#define SPRKLR_ID 1
#define SPRKLR_OP_MODE 11
#define SPRKLR_MODE_CTL_REG 32
#define SPRKLR_FWD(s) (s&0xFFFE) 
#define SPRKLR_RVS(s) (s|0x1)
#define SPRKLR_FWD_BIT 0
#define SPRKLR_RVS_BIT 1
#define SPRKLR_VELO_CTL_REG 528
#define SPRKLR_TORQ_CTL_REG 512
#define DXL_MOVING_STATUS_THRESHOLD 20

#define GRTR_ID 2
#define GRTR_OP_MODE 11
#define GRTR_MODE_CTL_REG 32
#define GRTR_FWD(s) (s&0xFFFE) 
#define GRTR_RVS(s) (s|0x1)
#define GRTR_FWD_BIT 0
#define GRTR_RVS_BIT 1
#define GRTR_VELO_CTL_REG 528                              
#define GRTR_TORQ_CTL_REG 512

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

#define S_FAST_SPD 4000
#define S_SLOW_SPD 1000
#define S_VSLOW_SPD 300
#define G_SPD 2500

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

/*
    0	Current Control Mode DYNAMIXEL only controls current(torque) regardless of speed and position. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/position controllers.
    1	Velocity Control Mode This mode controls velocity. This mode is identical to the Wheel Mode(endless) from existing DYNAMIXEL. This mode is ideal for wheel-type robots.
    3   (Default) Position Control Mode	This mode controls position. This mode is identical to the Joint Mode from existing DYNAMIXEL. Operating position range is limited by the Max Position Limit(48) and the Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees.
    4	Extended Position Control Mode(Multi-turn)	This mode controls position. This mode is identical to the Multi-turn Position Control from existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn wrists or conveyer systems or a system that requires an additional reduction gear. Note that Max Position Limit(48), Min Position Limit(52) are not used on Extended Position Control Mode.
    5	Current-based Position Control Mode	This mode controls both position and current(torque). Up to 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for a system that requires both position and current control such as articulated robots or grippers.
    16	PWM Control Mode (Voltage Control Mode)
*/
typedef enum
{
    CurrentCtl = 0,
    VelocityCtl = 1,
    PositionCtl = 3,
    ExtPositionCtl = 4,
    CurrPositionCtl = 5,	
    PWMCtl = 16
} DynamixelCtlMode_e; 

// for print comment out below
//#define DEBUG

typedef enum
{
    active_fast,
    active_slow,
    active_vslow,
    disabled
} SprinklerActions_e;

// weather prediction states as returned from the server
typedef enum
{
    CLEAR = 1,
    FEW_CLOUDS = 2,
    SCATTERED_CLOUDS = 3,
    BROKEN_CLOUDS = 4,
    SHOWER_RAIN = 9,
    RAIN = 10,
    THUNDERSTORM = 11,
    SNOW = 13,
    MIST = 50
} WeatherPrediction_e;
	  
// data for the cloud weather predictor application
#define WEATHER_API_KEY 234567
#define WEATHER_CITY_ID 12

// if we have not defined it to be C20++ with std::format add this template
#ifndef C20_HAS_FORMAT
namespace detail
{
#if 0
 /* C++ 17 version */
 /* Convert std::string type to const char*, output other types as is */
 template<typename T>
 auto Convert(T&& value)
 {
   /* Convert std::string type to const char* */
   if constexpr (std::is_same<std::remove_cv_t<std::remove_reference_t<T>>, std::string>::value)
   {
     return std::forward<T>(value).c_str();
   }
   /* Output other types as is */
   else
   {
     return std::forward<T>(value);
   }
 }
#else
    /* C++ 14 version */
    /* Convert std::string type to const char* */
    template<typename T, typename std::enable_if<std::is_same<std::remove_cv_t<std::remove_reference_t<T>>, std::string>::value>::type* = nullptr>
    auto Convert(T&& value)
    {
        return std::forward<T>(value).c_str();
    }
    /* Anything other than std::string type is output as is. */
    template<typename T, typename std::enable_if<!std::is_same<std::remove_cv_t<std::remove_reference_t<T>>, std::string>::value>::type* = nullptr>
    auto Convert(T&& value)
    {
        return std::forward<T>(value);
    }
#endif
    /* String formatting (internal processing) */
    template<typename ... Args>
    std::string StringFormatInternal(const std::string& format, Args&& ... args)
    {
        /* Calculate the number of characters after formatting */
        int str_len = std::snprintf(nullptr, 0, format.c_str(), std::forward<Args>(args) ...);
        /* Format failure */
        if (str_len < 0)
        {
            std::runtime_error("String Formatting Error");
        }
        else
        {
            /* Nothing to do */
        }
        /* Calculate the buffer size (string length + null character size) */
        size_t buffer_size = str_len + sizeof(char);
        /* Allocate memory for the buffer size */
        std::unique_ptr<char[]> buffer(new char[buffer_size]);
        /* Format the string */
        std::snprintf(buffer.get(), buffer_size, format.c_str(), args ...);
        /* Convert string to std::string type and output */
        return std::string(buffer.get(), buffer.get() + str_len);
    }
}
/* Format string */
template<typename ... Args>
std::string StringFormat(const std::string& format, Args&& ... args)
{
    /* Convert each parameter type and format string */
    return detail::StringFormatInternal(format, detail::Convert(std::forward<Args>(args)) ...);
}
#endif // end format without C++20

// functions for curl to http server which is weather predictor
size_t onReceive(char* ptr, size_t size, size_t nmemb, std::string* stream) {
    const size_t sizes = size * nmemb;
    stream->append(ptr, sizes);
    return sizes;
}

std::string get_responce() {
    CURL *curl = curl_easy_init();
    if(curl == nullptr) {
        std::cerr << "curl init error" << std::endl;
        curl_easy_cleanup(curl);
        exit(1);
    }

    std::string ret;
    // setup
    //curl_easy_setopt(curl, CURLOPT_URL, "api.openweathermap.org/data/2.5/weather?id=\"WEATHER_CITY_ID\"&units=metric&mode=xml&APPID=\"WEATHER_API_KEY\"");
#ifndef C20_HAS_FORMAT
    std::string text1 = "api.openweathermap.org/data/2.5/weather?id=\"";
    std::string text2 = "\"&units=metric&mode=xml&APPID=\"";
    std::string text3 = "\"";
    std::string api_str = StringFormat("%s%d%s%d%s", text1.c_str(), WEATHER_CITY_ID, text2.c_str(), WEATHER_API_KEY, text3.c_str());
    curl_easy_setopt(curl, CURLOPT_URL, &api_str[0]);
#else
    std::string fmt_s = std::format("api.openweathermap.org/data/2.5/weather?id=\"{}\"&units=metric&mode=xml&APPID=\"{}\"", WEATHER_CITY_ID, WEATHER_API_KEY);	
    curl_easy_setopt(curl, CURLOPT_URL, &fmt_s[0]);
#endif	
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, onReceive);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &ret);

    CURLcode res = curl_easy_perform(curl);
    // using sleep time, Succeed
    sleep(5);
    curl_easy_cleanup(curl);
    if(res != CURLE_OK) {
        std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        exit(1);
    }
    if(ret.length() == 0) {
        std::cerr << "respnceData is null, maybe timeout" << std::endl;
        std::cerr << "responceData: " << ret << std::endl;
        exit(1);
    }
    return ret;
}

int main(int argc, const char* argv[]) {

  int sprinkler_state = SprinklerActions_e::disabled;	
  int sprinkler_speed = 0;
  int gritter_speed = 0;
  int aux_dot_pump = 0;

  // initialise drives
  // connect to the drive and the hand
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);

  // Initialize PacketHandler Structs
  packetHandler();

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_error = 0;                              // Dynamixel error
  
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
  write1ByteTxRx(port_num, PROTOCOL_VERSION, GRTR_ID, GRTR_TORQ_CTL_REG, TORQUE_ENABLE);
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
    printf("Gritter has been successfully connected \n");
  }
  write1ByteTxRx(port_num, PROTOCOL_VERSION, SPRKLR_ID, SPRKLR_TORQ_CTL_REG, TORQUE_ENABLE);
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
    printf("Sprinkler has been successfully connected \n");
  }
  
  // set up the external DOT port1
  write1ByteTxRx(port_num, PROTOCOL_VERSION, SPRKLR_ID, P1_MODE, DOT);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  
  // set mode gritter to velocity control
  write1ByteTxRx(port_num, PROTOCOL_VERSION, GRTR_ID, GRTR_OP_MODE, DynamixelCtlMode_e::VelocityCtl);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  // set mode sprinkler to velocity control
  write1ByteTxRx(port_num, PROTOCOL_VERSION, SPRKLR_ID, SPRKLR_OP_MODE, DynamixelCtlMode_e::VelocityCtl);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  
  // make a call to the server to get the weather prediction
  std::string responce = get_responce();
#if DEBUG
  std::cerr << responce.length() << std::endl;
#endif

  // code for extracting the weather condition from the responce
  // responce should contain for example....> std::string mys = "icon \"7\"";
  //
  char parse_key[] = "icon";
  int w_cond = 0;
  char* ptr = strstr(&responce[0], parse_key);
  ptr = strchr(ptr, '"');

  sscanf(ptr+1, "%d", &w_cond);
#if DEBUG
  printf("weather condition %d\n",w_cond);
#endif  

  // check the weather result that was parsed
  switch(w_cond){
    case WeatherPrediction_e::CLEAR:
      // CLEAR;
	  sprinkler_state = SprinklerActions_e::active_fast;
	  break;
    case WeatherPrediction_e::FEW_CLOUDS:
      // FEW_CLOUDS;
	  sprinkler_state = SprinklerActions_e::active_slow;
	  break;
    case WeatherPrediction_e::SCATTERED_CLOUDS:
      // SCATTERED_CLOUDS;
	  sprinkler_state = SprinklerActions_e::active_slow;
	  break;
    case WeatherPrediction_e::BROKEN_CLOUDS:
      // BROKEN_CLOUDS;
	  sprinkler_state = SprinklerActions_e::active_slow;
	  break;
    case WeatherPrediction_e::SHOWER_RAIN:
      // SHOWER_RAIN;
	  sprinkler_state = SprinklerActions_e::disabled;
	  break;
    case WeatherPrediction_e::RAIN:
      // return RAIN;
	  sprinkler_state = SprinklerActions_e::disabled;
	  break;
    case WeatherPrediction_e::THUNDERSTORM:
      // THUNDERSTORM;
	  sprinkler_state = SprinklerActions_e::disabled;
	  break;
    case WeatherPrediction_e::SNOW:
      // SNOW;
	  sprinkler_state = SprinklerActions_e::gritter;
	  break;
    case WeatherPrediction_e::MIST:
      // MIST;
	  sprinkler_state = SprinklerActions_e::active_vslow;
	  break;
    default:
	  sprinkler_state = SprinklerActions_e::disabled;
	  break;  
  }
  
  switch(sprinkler_state){
    case SprinklerActions_e::active_fast:
	  sprinkler_speed = S_FAST_SPD;
	  gritter_speed = 0;
	  aux_dot_pump = 1;
	  break;
    case SprinklerActions_e::active_slow:
	  sprinkler_speed = S_SLOW_SPD;
	  gritter_speed = 0;
	  aux_dot_pump = 1;
	  break;
    case SprinklerActions_e::active_vslow:
	  sprinkler_speed = S_VSLOW_SPD;
	  gritter_speed = 0;
	  aux_dot_pump = 1;
	  break;
    case SprinklerActions_e::disabled:
	  sprinkler_speed = 0;
	  gritter_speed = 0;
	  aux_dot_pump = 0;
	  break;
    case SprinklerActions_e::gritter:
	  sprinkler_speed = 0;
	  gritter_speed = G_SPD;
	  aux_dot_pump = 0;
	  break;
    default:
	  sprinkler_speed = 0;
	  gritter_speed = 0;
	  aux_dot_pump = 0;
	  break;
  }
  
  // set the drives to the speed and state as requested from the weather prediction server
  
  // sprinkler velocity
  write1ByteTxRx(port_num, PROTOCOL_VERSION, SPRKLR_ID, SPRKLR_VELO_CTL_REG, sprinkler_speed);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
     printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
     printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
  
  // gritter speed
  write1ByteTxRx(port_num, PROTOCOL_VERSION, GRTR_ID, GRTR_VELO_CTL_REG, gritter_speed);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
     printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
     printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }
		
  // aux_dot_p11 output
  write1ByteTxRx(port_num, PROTOCOL_VERSION, SPRKLR_ID, P11_DATA, aux_dot_pump);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printTxRxResult(PROTOCOL_VERSION, dxl_comm_result);
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printRxPacketError(PROTOCOL_VERSION, dxl_error);
  }  
	
  return 0;
}

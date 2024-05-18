/**********************************************************************
 
 Control the robot off the joystick controller
 For Unitree Go2 4 legged robot
 
***********************************************************************/

#include <cmath>
#include <math.h>

// robot movement
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

// joystick
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/client/client.hpp>

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

// light control
#include <unitree/robot/go2/vui/vui_client.hpp>

#include <iostream>
#include <atomic>
#include <thread>
int data = 0;
volatile std::atomic<bool> ready(false);
#define WAIT_FOR_HANDSHAKE 1                              // enable handshake between joystick and robot

#include <algorithm>                                     // for standard clamp

using namespace unitree::common;

// joystick controller
// Remote Control Key Value Consortium
typedef union
{
  struct
  {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

// robot motion
enum robot_mode
{
  /*---Basic motion---*/
  normal_stand,
  balance_stand,
  velocity_move,
  trajectory_follow,
  stand_down,
  stand_up,
  damp,
  recovery_stand,
  /*---Special motion ---*/
  sit,
  rise_sit,
  stretch,
  wallow,
  content,
  pose,
  scrape,
  front_flip,
  front_jump,
  front_pounce,
  stop_move = 99
};

// set a state of the start button to start moving and stop moving on each consecuative press
enum start_but_state
{
   no_press;
   motion_on;
};

volatile robot_mode ROBOT_MODE = stop_move;                                           // initialise the robot off written by joystick and motion controller :- default is stopped
float VELO_X = 0.0f;                                                                  // move velocity
float VELO_Y = 0.0f;
float VELO_Z = 0.0f;
const float JOYSTICK_SPAN = 1.0f;                                                     // set this to be the span of the joystick
float ROLL_ANGLE = 0.0f;
float PITCH_ANGLE = 0.0f;
float YAW_ANGLE = 0.0f;
float BODY_HT = 0.0f;
start_but_state START_PRESS_STATE = no_press;                                         // make the start button a run toggle switch

// type of compensation for the joystick
enum joy_compensation
{
   no_comp;
   comp1;
   comp2;
   comp3;
   comp4;
};

//
// a set of speed corrections from L Robbins for speed values passed to a controller from a joystick
// the optimum one of these can also be tested during the test phase
//
/*-----------------------------------------------------------------------------
 *      JoyStick2Speed():  joystick smooth algorythms by L Robbins
 *
 *  Parameters: double nJoyX, joy_compensation choice
 *  Return:     (none)
 *----------------------------------------------------------------------------*/
double JoyStick2Speed( float nJoyX, joy_compensation choice )
{
    switch (choice)
    {
      case comp1:
        return  ((((double)nJoyX) + pow(((double)nJoyX),2.0f))/2.0f);         // option 1
      case comp2:
        return  (((2.0f*((double)nJoyX)) + pow(((double)nJoyX),2.0f))/3.0f);  // option 2
      case comp3:
        return  ((((double)nJoyX)+(2.0f*pow(((double)nJoyX),2.0f)))/3.0f);    // option 3
      case comp4:
         return  (pow(((double)nJoyX),2.0f));                                 // option 4
      default:
         return(nJoyX);                                                       // option not specified then return it back
    }
}

// joystick
// Callback function for obtaining motion status
void JoystickHandler(const void *message)
{
  unitree_go::msg::dds_::WirelessController_ joystick = *(unitree_go::msg::dds_::WirelessController_ *)message;
  joy_compensation joy_c = comp1;                                  // choose joystick compensation method here
  
  // Remote control raw data
  std::cout << "lx: " << joystick.lx() << std::endl     // left joystick x
            << "ly: " << joystick.ly() << std::endl     // left joystick y
            << "rx: " << joystick.rx() << std::endl     // right joystick x
            << "ry: " << joystick.ry() << std::endl     // right joystick y
            << "keys: " << joystick.keys() << std::endl // key
            << "------" << std::endl;
  xKeySwitchUnion key;
  key.value = joystick.keys();                                            // read the keys

  // Determine whether a button has been pressed and perform a action on the robot via the volatile state variable
  if ((int)key.components.A == 1)
  {
    std::cout << "The key A is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    ROBOT_MODE = scrape;
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  }
  if ((int)key.components.X == 1)
  {
    std::cout << "The key X is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = front_flip;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  }
  if ((int)key.components.B == 1)
  {
    std::cout << "The key B is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = front_jump;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  }
  if ((int)key.components.Y == 1)
  {
    std::cout << "The key Y is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = front_pounce;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  }  
  if ((int)key.components.F1 == 1)
  {
    std::cout << "The key F1 is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = sit;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  } 
  if ((int)key.components.F2 == 1)
  {
    std::cout << "The key F2 is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = rise_sit;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  } 
  if ((int)key.components.start == 1)
  {
    std::cout << "The key start is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    if (START_PRESS_STATE == no_press) {
	START_PRESS_STATE = motion_on;
     } else if (START_PRESS_STATE == motion_on){
	START_PRESS_STATE = no_press;
    }
    ROBOT_MODE = velocity_move;
    VELO_X = std::clamp((JoyStick2Speed(joystick.lx(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);   // Obtain data for left joystick x, range [-1.0~1.0]
    VELO_Y = std::clamp((JoyStick2Speed(joystick.ly(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);   // Obtain left joystick y data, range [-1.0~1.0]
    VELO_Z = std::clamp((JoyStick2Speed(joystick.ry(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);   // Obtain right joystick y data, range [-1.0~1.0]
    ready.store(true, std::memory_order_relaxed);                                            // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                                          // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  } 
  if ((int)key.components.select == 1)
  {
    std::cout << "The key select is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                                                     // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = balance_stand;
    ROLL_ANGLE = std::clamp((JoyStick2Speed(joystick.lx(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);        // range is -1.0 to 1.0
    PITCH_ANGLE = std::clamp((JoyStick2Speed(joystick.ly(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);
    YAW_ANGLE = std::clamp((JoyStick2Speed(joystick.rx(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);
    BODY_HT = std::clamp((JoyStick2Speed(joystick.ry(),joy_c) / JOYSTICK_SPAN),-0.18f,0.03f);
    ready.store(true, std::memory_order_relaxed);                                                     // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                                                   // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  } 
  if ((int)key.components.R2 == 1)
  {
    std::cout << "The key R2 is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = trajectory_follow;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  }
  if ((int)key.components.R1 == 1)
  {
    std::cout << "The key R1 is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = recovery_stand;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  }
  if ((int)key.components.L1 == 1)
  {
    std::cout << "The key L1 is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = wallow;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  }  
  if ((int)key.components.L2 == 1)
  {
    std::cout << "The key L2 is pressed " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
    START_PRESS_STATE = no_press;                                   // reset the start button state if we chose and action next on the keypad
    ROBOT_MODE = pose;
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  } 

  // we pressed start and latched it to the joystick controller  
  if (START_PRESS_STATE == motion_on)
  {
    std::cout << "The start key has latched movement from joystick to on " << std::endl;
    std::atomic_thread_fence(std::memory_order_release);
	VELO_X = std::clamp((JoyStick2Speed(joystick.lx(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);                                         //  Obtain data for left joystick x, range [-1.0~1.0]
	VELO_Y = std::clamp((JoyStick2Speed(joystick.ly(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);                                         // Obtain left joystick y data, range [-1.0~1.0]
	VELO_Z = std::clamp((JoyStick2Speed(joystick.ry(),joy_c) / JOYSTICK_SPAN),-1.0f,1.0f);                                         // Obtain right joystick y data, range [-1.0~1.0]
    ready.store(true, std::memory_order_relaxed);                   // set flag to true to force the action on the robot	
    while (ready.load(std::memory_order_relaxed)) {                 // wait for motion to complete
    }
    std::atomic_thread_fence(std::memory_order_acquire);	
  }
}

// robot
class Sports_Rbot
{
public:
  Sports_Rbot()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Sports_Rbot::HighStateHandler, this, std::placeholders::_1), 1);
  };

  ~Sports_Rbot() {
    std::cout << "Destroyed the robot class" << std::endl;
  }
	
  // this routine is iterative as the robot thread
  void RobotControl()
  {
    ct += dt;
    double px_local, py_local, yaw_local;
    double vx_local, vy_local, vyaw_local;
    double px_err, py_err, yaw_err;
    double time_seg, time_temp;

    unitree::robot::go2::PathPoint path_point_tmp;
    std::vector<unitree::robot::go2::PathPoint> path;

    // we use START_PRESS_STATE = no_press to reset so the start button works continously without the handshake but to make sure we get joy->action enable it
	//
    // wait for a pushbutton instruction (which sets the ready bit) becasue the routine is iterative we do an if and not a while
#ifdef WAIT_FOR_HANDSHAKE
    if (!ready.load(std::memory_order_relaxed)) {
		return;
    }
#endif

    // now aquire the fence as we are going to do an actio
    std::atomic_thread_fence(std::memory_order_acquire);
  
    switch (ROBOT_MODE)
    {
    case normal_stand:                                                        // 0. idle, default stand
      sport_client.SwitchGait(0);                                             // 0:idle; 1:tort; 2:tort running; 3:climb stair; 4:tort obstacle
      sport_client.StandUp();
      break;

    case balance_stand:                                                           // 1. Balance stand (controlled by dBodyHeight + rpy)
      sport_client.Euler(ROLL_ANGLE, PITCH_ANGLE, YAW_ANGLE);                     // roll, pitch, yaw
      sport_client.BodyHeight(BODY_HT);                                           // relative height [-0.18~0.03]
      sport_client.BalanceStand();
      break;

    case velocity_move: // 2. target velocity walking (controlled by velocity + yawSpeed)
      sport_client.Move(VELO_X, VELO_Y, VELO_Z);
      break;

    case trajectory_follow: // 3. path mode walking
      time_seg = 0.2;
      time_temp = ct - time_seg;
      for (int i = 0; i < 30; i++)
      {
        time_temp += time_seg;

        px_local = 0.5 * sin(0.5 * time_temp);
        py_local = 0;
        yaw_local = 0;
        vx_local = 0.5 * cos(0.5 * time_temp);
        vy_local = 0;
        vyaw_local = 0;

        path_point_tmp.timeFromStart = i * time_seg;
        path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
        path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
        path_point_tmp.yaw = yaw_local + yaw0;
        path_point_tmp.vx = vx_local * cos(yaw0) - vy_local * sin(yaw0);
        path_point_tmp.vy = vx_local * sin(yaw0) + vy_local * cos(yaw0);
        path_point_tmp.vyaw = vyaw_local;
        path.push_back(path_point_tmp);
      }
      sport_client.TrajectoryFollow(path);
      break;

    case stand_down: // 4. position stand down.
      sport_client.StandDown();
      break;

    case stand_up: // 5. position stand up
      sport_client.StandUp();
      break;

    case damp: // 6. damping mode
      sport_client.Damp();
      break;

    case recovery_stand: // 7. recovery stand
      sport_client.RecoveryStand();
      break;

    case sit:
      if (flag != sit)
      {
        sport_client.Sit();
        flag = sit;
      }
      break;

    case rise_sit:
      if not (flag == rise_sit)
      {
        sport_client.RiseSit();
        flag = rise_sit;
      }
      break;

    case stretch:
      if not (flag == stretch)
      {
        sport_client.Stretch();
        flag = stretch;
      }
      break;

    case wallow:
      if not (flag == wallow)
      {
        sport_client.Wallow();
        flag = wallow;
      }
      break;
    case content:
      if not (flag == content)
      {
        sport_client.Content();
        flag = content;
      }
      break;
    case pose:
      if not (flag == pose)
      {
        sport_client.Pose(true);
        flag = pose;
      }
      break;

    case scrape:
      if not (flag == scrape)
      {
        sport_client.Scrape();
        flag = scrape;
      }
      break;

    case front_flip:
      if not (flag == front_flip)
      {
        sport_client.FrontFlip();
        flag = front_flip;
      }
      break;

    case front_jump:
      if not (flag == front_jump)
      {
        sport_client.FrontJump();
        flag = front_jump;
      }
      break;
    case front_pounce:
      if not (flag == front_pounce)
      {
        sport_client.FrontPounce();
        flag = front_pounce;
      }
      break;

    case stop_move: // stop move
      sport_client.StopMove();
      break;

    default:
      sport_client.StopMove();
    }
	// release the thread fence and allow another action to be sent
	if (START_PRESS_STATE == no_press) {                                 // change mode to stop unless we hit start for continious movement
	    ROBOT_MODE = stop_move;
    }
	std::atomic_thread_fence(std::memory_order_release);
    ready.store(false, std::memory_order_relaxed);                      // tell other thread we completed the action
	//return;
  };

  // Get initial position
  void GetInitState()
  {
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: pitch x0: " << px0 << ", roll y0: " << py0 << ", yaw z0: " << yaw0 << std::endl;
  };

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;

    std::cout << "Position: " << state.position()[0] << ", " << state.position()[1] << ", " << state.position()[2] << std::endl;
    std::cout << "IMU rpy: " << state.imu_state().rpy()[0] << ", " << state.imu_state().rpy()[1] << ", " << state.imu_state().rpy()[2] << std::endl;
  };

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  double px0, py0, yaw0;  // initial state position sum polarization
  double ct = 0;          // running time
  int flag = 0;           // Special operation strategy
  float dt = 0.005f;      // Control step length
};

int main(int argc, char **argv)
{
  std::string networkInterface = "eth0";
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
	std::cout << "using default of eth0" << std::endl;
    //exit(-1);
  } else {
	networkInterface = argv[1];
  }

  // make network connection to robot	
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  // start joystick controller and handlers
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_suber;
  joystick_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
  joystick_suber->InitChannel(JoystickHandler, 1);

  // create light control instance
  unitree::robot::go2::VuiClient vc;
  // Set request timeout 1.0s
  vc.SetTimeout(1.0f);
  vc.Init();
  int ret;
  
  // robot motion class
  Sports_Rbot sports_motion;
  sleep(1);                                                            // Wait for 1 second to obtain a stable state
  sports_motion.GetInitState();                                        // Get initial position
  
  // start the robot motion thread 
  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(sports_motion.dt * 1000000, std::bind(&Sports_Rbot::RobotControl, &sports_motion));

  // loop forever letting the threads interact
  while (1)
  {
	if (START_PRESS_STATE == motion_on) {
		ret = vc.SetBrightness(10);
	} else {
		ret = vc.SetBrightness(1);
    }		
    sleep(10);
  }
  return 0;
  
}

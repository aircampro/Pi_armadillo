/**
 * @file trani_mover.cpp
 *
 * @brief This file contains the implementation of the WLAction class, which initiates the stand-up behavior of wheel-legged robots. sits down and moves to the joystick
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */
/*
I2C	Pin3:SDA GPIO2
    Pin5:SCL GPIO3

we are using I2C Joystick Unit for M5Stack
The protocol is very simple, when I2C address 0x52 reads from register 0x00 3 bytes, I get x-direction value, y-direction value, btn state, respectively.It will be.
There is no need for initialization.

ini reader git clone https://github.com/benhoyt/inih.git

*/

#define I2C_BASEADDR 0x52                  // address for the m5stick joystick
#define JOYSPAN 100                        // please set as you find this scalar

#include <sys/types.h>
#include <sys/ioctl.h>
#include <dev/iicbus/iic.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include "INIReader.h"

#define IICBUS	"/dev/iic1"

#include "wl_controller_base.h"

/**
 * @brief The WLAction class controls the stand-up behavior of wheel-legged robots.
 */
class WLAction : public ControllerBase
{
public:
  /**
   * @brief Overrides waits for data reception and then initiates the stand-up behavior.
   */
  void starting(double v) override
  {
    std::cout << "Waiting to receive data...\n";
    while (!recv_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "Received\n";

    homing();                                                    // Move to the home position
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait for stability
    standing(v);                                                 // Initiates the stand-up behavior
  }
  
  /**
   * @brief Overrides waits for data reception and then initiates the sit-down behavior.
   */
  void sitting(double v) override
  {
    std::cout << "Waiting to receive data...\n";
    while (!recv_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "Received\n";

    sitDown(v); // Initiates the sit-down behavior
  }

  void moving(double x, double y, int b) override
  {
    std::cout << "Waiting to receive data...\n";
    while (!recv_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "Received\n";
    if (b==1) {
        move_from_joy(x,y);                                                         // Initiates the sit-down behavior
	}
  }

  void change_pd_param(double pb, double pd)
  {
       mod_params(pb, pd);
  }

  void change_dt(double dt)
  {
       set_dt(dt);
  }
private:
  /**
   * @brief Overrides the update function to do nothing, as the stand-up behavior does not require continuous updates.
   */
  void update() override
  {
    // No update needed for stand-up behavior
  }
};

int g_run = 1;
/**
   * @brief Interrupt handler -- set global flag to abort main loop
*/ .
void signalHandler(int n) {
	g_run = 0;
}

/**
 * @brief The main function initializes the WheelLegged instance, creates an instance of WLStandUp, and starts the stand-up behavior.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments. the first one is the robot ip_address
 * @return int Program exit status.
 */
int main(int argc, char *argv[])
{

    // define the signal handlers which will clean up if needed
    //
    if (signal(SIGINT, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGINT " << "\n";
    }

    if (signal(SIGQUIT, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGQUIT " << "\n";
    }
	
    if (signal(SIGFPE, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGFPE " << "\n";
    }

    /*
        cant do the signal can not be trapped

    if (signal(SIGKILL, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGKILL " << "\n";
    }
    */

    if (signal(SIGSEGV, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGSEGV " << "\n";
    }

    if (signal(SIGPIPE, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGPIPE " << "\n";
    }

    if (signal(SIGTERM, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGTERM " << "\n";
    }

    if (signal(SIGCONT, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGCONT " << "\n";
    }

    if (signal(SIGTSTP, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGTSTP " << "\n";
    }

    // kill -10 <pid>
	if (signal(SIGUSR1, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGUSR1 " << "\n";
    }

    // kill -12 <pid>
	if (signal(SIGUSR2, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGUSR2 " << "\n";
    }

    // Bus error (incorrect memory access)
	if (signal(SIGBUS, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGBUS " << "\n";
    }

    // CPU Time Limit Exceeded (4.2BSD) report this error
	if (signal(SIGXCPU, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGBUS " << "\n";
    }
	
    if (signal(SIGHUP, signalHandler) == SIG_ERR) {
        std::cout << "cant catch SIGHUP " << "\n";
    }

    if (signal(SIGALRM, alrm) == SIG_ERR) {
        std::cout << "cant catch SIGALRM " << "\n";
    }
	
  std::string settingsPath = "settings.ini";
  INIReader reader(settingsPath);

  if (reader.ParseError() < 0) {
    std::cerr << "Can't load " << settingsPath << "\n";
    return 1;
  }

  // read the setting sfrom the .ini file
  double p_band = static_cast<double>(reader.GetInteger("PID", "KP", 600)) / 10.0;
  double d_band = static_cast<double>(reader.GetInteger("PID", "KD", 4.5)) / 1000.0;
  int t_band = reader.GetInteger("TIME", "TM", 700);                                               // Movement speed
  int t_step = reader.GetInteger("TIME", "TS", 2);                                                 // Time step
  bool ts_from_file = reader.GetBoolean("TIME", "ts_from_file", false);                            // get time step from file above
	
  int i;
  int fd;
  uint8_t cmdbuff[12] = {0};
  struct iiccmd start_cmd = {.slave = I2C_BASEADDR};
  struct iic_msg write_cmd[1] = {
		{
			.slave = I2C_BASEADDR << 1 | IIC_M_RD,
			.flags = IIC_M_RD,
			.len = sizeof (cmdbuff),
			.buf = cmdbuff,
		}, 
  };
  struct iic_rdwr_data cmd = {
		.msgs =  write_cmd,
		.nmsgs = 1,
  };
  if ((fd = open(IICBUS, O_RDWR)) == -1) {
		fprintf(stderr, "%d Error %s\n", __LINE__, strerror(errno));
		return 1;
  }
	
  limxsdk::Wheellegged *wl = limxsdk::Wheellegged::getInstance();

  std::string robot_ip = "127.0.0.1";
  if (argc > 1)                                                                      // pass the argument from the command line for the bot ip address
  {
    robot_ip = argv[1];
  }
  if (!wl->init(robot_ip))
  {
    exit(1);                                                                         // Exit if initialization fails
  }

  bool is_calibration = false;

  wl->subscribeDiagnosticValue([&](const limxsdk::DiagnosticValueConstPtr& msg) {    // Subscribing to diagnostic values for calibration state
    if (msg->name == "calibration") {                                                // Check if the diagnostic message pertains to calibration
      if (msg->code == 0) {
        is_calibration = true;
      }
    }
  });

  std::cout << "Waiting for calibration to begin." << std::endl;
  while(!is_calibration) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  std::cout << "calibration end......" << std::endl;

  WLAction ctl;                                                                      // create robot controller class
  double tt = static_cast<double>(t_band) / 1000.0;                                  // read 
  if (ts_from_file == true) {
	ctl.change_dt(static_cast<double>(t_step)/1000.0);                            // set the time step from the ini file
  }
  ctl.change_pd_param(p_band, d_band);                                               // initialise pd controller
  ctl.starting(tt);                                                                  // Start the stand-up behavior

  // Main loop
  int state = 0;
  while (state==0)
  {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
	state = 1;
  }
  ctl.sitting(tt);                                                                   // now sit down

  while (state==1)
  {
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
       state = 2;
  }

  while (g_run == 1) {                                                              // read the joystick from i2c and move in accordance
	    if (ioctl(fd, I2CRDWR, &cmd) == -1) {
		fprintf(stderr, "%d Error %s\n", __LINE__, strerror(errno));
	    } else {
		printf("x = %02x y = %02x btn = %02x  \n", cmdbuff[0], cmdbuff[1], cmdbuff[2]);
		int x = cmdbuff[0];
		int y = cmdbuff[1];	
                int b = cmdbuff[2];
		double xr = static_cast<double>(x) / JOYSPAN;
		double yr = static_cast<double>(y) / JOYSPAN;
		ctl.moving(x, y, b);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
	    }
   }

   close(fd);
   ctl.sitting(tt); 
   std::this_thread::sleep_for(std::chrono::milliseconds(100));
   return 0;
	
  return 0;
}

/*
    ==========================================
    Code for RT Robots X7Crane and s17 robot
	inverse kinematics test code
	gravity compensation test code
	==========================================
*/
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <stdlib.h>    
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>

// For getch(), kbhit()
// Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/c%2B%2B/example/protocol2.0/read_write/read_write.cpp
#include <fcntl.h>
//#include <termbits.h>
//
// this is termbits.h hardcoded below --- incase you dont have it
//
typedef unsigned char   cc_t;
typedef unsigned int    tcflag_t;
#define NCCS 19
struct termios {
    tcflag_t c_iflag;       /* input mode flags */
    tcflag_t c_oflag;       /* output mode flags */
    tcflag_t c_cflag;       /* control mode flags */
    tcflag_t c_lflag;       /* local mode flags */
    cc_t c_line;            /* line discipline */
    cc_t c_cc[NCCS];        /* control characters */
};
#include <termios.h>
#define STDIN_FILENO 0
#define ESC_ASCII_VALUE 0x1b

#include "rt_manipulators_cpp/hardware.hpp"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
#include "rt_manipulators_ik.hpp"
#include "rt_manipulators_dynamics.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

// this lists the options that can be performed in this program
enum OptionTypes_e : int
{
  inv_k_LM,
  inv_kin,
  grav_comp,
  grav_comp_Pgain,
  grav_comp_PgainInt,
  inv_kin_picking,
  number_of_OptionTypes
}

int getch() {
  // Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/c7e1eb71c911b87f7bdeda3c2c9e92276c2b4627/c%2B%2B/example/protocol2.0/read_write/read_write.cpp#L100-L114
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int kbhit(void) {
  // Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/c7e1eb71c911b87f7bdeda3c2c9e92276c2b4627/c%2B%2B/example/protocol2.0/read_write/read_write.cpp#L116-L143
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void set_arm_joint_positions(std::vector<manipulators_link::Link> & links,
                             std::vector<double> positions) {
  // Set the current angle of the arm joint to the link
  if (positions.size() != 7) {
    std::cerr << "Please set 7 joint angles in the argument positions" << std::endl;
    return;
  }

  int start_id = 2;  // Link1
  for (int i=0; i < positions.size(); i++) {
    links[start_id + i].q = positions[i];
  }
}

int main(int argc, char *argv[]) {

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 3000000;                          // 3Mbps

  if (argc < 5) {
	std::cerr << "wrong number of args... type,action,x_pos,y_pos,z_pos" << std::endl;
	return -1;
  }
  
  std::string type_of_robot = argv[1];             // X7 or S17
  OptionTypes_e type_of_action = atoi(argv[2]);    // choose the action as one of the OptionTypes_e types
  double x_dir = atof(argv[3]);                    // desired position from command line
  double y_dir = atof(argv[4]);
  double z_dir = atof(argv[5]);  

  if (type_of_robot == "X7" ) {                                                       // this is the files for the links and hardware for the X7 crane
    std::string hardware_config_file = "../config/crane-x7_current.yaml";
    std::string link_config_file = "../config/crane-x7_links.csv";
  } else if (type_of_robot == "S17" ) {                                               // this is the files for the links and hardware for the s17 robot
    std::string hardware_config_file = "../config/sciurus17.yaml";
    std::string link_config_file = "../config/sciurus17_links.csv";
  } else {
	std::cerr << "unsupported robot type X7 crasne or S17 robot are the only 2 types" << std::endl;
    return -1;
  }  

  struct timeval start_time, end_time; 
  long milli_time, seconds, useconds;
  start_time = gettimeofday(&start_time, NULL);
		  
  // Express the link structure of the robot
  // std::vector<manipulators_link::Link> links;
  // you can get the link configuration by reading a CSV file that represents the link configuration.
  // Parse the CSV file to get the link configuration
  std::vector<manipulators_link::Link>  = kinematics_utils::parse_link_config_file(link_config_file);

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "cannot connect to hardware please check usb connection" << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(hardware_config_file)) {
    std::cerr << "failed to load hardware file" << std::endl;
    return -1;
  }
  
// for help reading the link class is shown below
//Link class{
// public:
// std::string name; 
//  int sibling; 
//  int child; 
//  int parent; 
// Eigen::Vector3d p;      // Position in the world coordinate system
// Eigen::Matrix3d R;      // Attitude in the world coordinate system
// Eigen::Vector3d v;      // Speed in the world coordinate system
// Eigen::Vector3d w;      // Angular velocity vector in the world coordinate system 
//   q;                    // Joint position
//  double dq;             // joint velocity
//  double ddq;            // Joint acceleration
// Eigen::Vector3d a;      // Joint axis vector relative to parent link
// Eigen::Vector3d b;      // position relative to parent link
//  double m;              // mass
// Eigen::Vector3d c;      // Centroid position relative to the self-link
// Eigen::Matrix3d I;      // Inertia tensor for self-link 
//   dxl_id;               //  Dynamixel ID
//  double min_q;          // joint position lower limit
// double max_q;           // joint position upper limit double max_q; // joint position lower limit double max_q; // joint position lower limit double max_q; // joint position lower limit double max_q; // joint position lower limit double max_q; // joint position lower limit double max_q; // joint position lower limit double max_q; //
//};

  // To update the link position and posture
  int start_id = 1;
  kinematics::forward_kinematics(links, start_id);
  kinematics_utils::print_links(links, start_id);
  
  // Solve the inverse kinematics to move the hand to any position and posture
  // Before solving inverse kinematicsq_min, enter the lower (radian) and upper (radian) values for each link joint position in the link configurationq_max.

  std::vector<std::string> group_names;
  if (type_of_robot == "X7" ) {
    group_names = {"arm"};
  } else if (type_of_robot == "S17" ) {
    group_names = {"right_arm", "left_arm", "torso"};	
  }	
	  
  // Setting joint range of motion
  if (type_of_robot == "X7" ) {                                                           // ---------- X7 crane --------------
	  
    for (auto link_id : kinematics_utils::find_route(links, 8)) {
      //  Set the lower limit of link ID<link_id> joint position
      //  links[link_id].q_min = -M_PI;
      //  links[link_id].q_max = -M_PI;
      hardware.get_max_position_limit(links[link_id].dxl_id, links[link_id].max_q);
      hardware.get_min_position_limit(links[link_id].dxl_id, links[link_id].min_q);
    }
	
    // Set the arm group's servo maximum acceleration to pi rad/s^2 and maximum speed to pi rad/s.
    if (!hardware.write_max_acceleration_to_group("arm", 1.0 * M_PI)) {                     // crane has only 1 arm
      std::cerr << "arm : failed to set max acceleration" << std::endl;
      return -1;
    }

    if (!hardware.write_max_velocity_to_group("arm", 1.0 * M_PI)) {
      std::cerr << "arm : failed to set max velocity" << std::endl;
      return -1;
    }
	
    // Write (800, 0, 0) to the servo position control PID gain of the arm group.
    if (!hardware.write_position_pid_gain_to_group("arm", 800, 0, 0)) {
      std::cerr << "arm : failed to set the PID controller" << std::endl;
      return -1;
    }

    // set torque on
    if (!hardware.torque_on("arm")) {
      std::cerr << "arm : torque on failed" << std::endl;
      return -1;
    }
	
  } else if (type_of_robot == "S17" ) {                                                      // ---------- Sciurus17 robot --------------
	  
    const int RIGHT = 0;
    const int LEFT = 1;
    const std::vector<int> SIDES = {RIGHT, LEFT};
    std::map<int, int> TARGET_LINK_ID{
      {RIGHT, 12},
      {LEFT, 21},
    };

    for (const auto & side : SIDES) {
      for (auto link_id : kinematics_utils::find_route(links, TARGET_LINK_ID[side])) {
        hardware.get_max_position_limit(links[link_id].dxl_id, links[link_id].max_q);
        hardware.get_min_position_limit(links[link_id].dxl_id, links[link_id].min_q);
      }
    }
	
    for (auto group : group_names) {

      if (!hardware.write_max_acceleration_to_group(group, 1.0 * M_PI)) {
        std::cerr << group << " : failed to set max acceleration." << std::endl;
        return -1;
      }

      if (!hardware.write_max_velocity_to_group(group, 1.0 * M_PI)) {
        std::cerr << group << " : failed to set max velocity" << std::endl;
        return -1;
      }

      if (!hardware.write_position_pid_gain_to_group(group, 800, 0, 0)) {
        std::cerr << group << " : failed to set max PID" << std::endl;
        return -1;
      }

      if (!hardware.torque_on(group)) {
        std::cerr << group << " : torque on failed" << std::endl;
        return -1;
      }
    }
  
  } 
  
  // The algorithm for calculating inverse kinematics by numerical solution is implemented
  Eigen::Vector3d target_p;
  Eigen::Matrix3d target_R;
  kinematics_utils::q_list_t q_list;

  //Target position (0.2m in the X direction, 0.0m in the Y direction, 0.3m in the Z direction)
  target_p << x_dir, y_dir, z_dir;
	
  switch(type_of_action) {
	  
      case inv_k_LM:
      //Target posture (pi/2 rotation around Y axis)
      target_R = kinematics_utils::rotation_from_euler_ZYX(0, M_PI_2, 0);

      // Solve inverse kinematics and obtain joint positions
      if (kinematics::inverse_kinematics_LM(links, 8, target_p, target_R, q_list)==false) {
        std::cerr << "arm : inverse kinematics failed" << std::endl;
        return -1;
      }	

      for (const auto & [target_id, q_value] : q_list) {
        hardware.set_position(links[target_id].dxl_id, q_value);
      }
	  break;

      case inv_kin:
      if (type_of_robot == "X7" ) {
        //samples03::x7_3dof_inverse_kinematics(links, target_p, q_list) is a function that takes the origin of the 6th link as the target 
        //position (target_p) and outputs the target angle of the 1st, 2nd, and 4th joints.
        // Solve inverse kinematics and obtain joint angles
        samples03::x7_3dof_inverse_kinematics(links, target_p, q_list);
      } else if (type_of_robot == "S17" ) {
        // samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list) is a function that takes the origin of the 6th link of the right arm as the target position 
        // (target_p) and outputs the target angle of the 1st, 2nd, and 4th joints of the right arm
        // Solve inverse kinematics and obtain joint angles
        samples03::s17_3dof_right_arm_inverse_kinematics(links, target_p, q_list);
	  }
      // Set joint angle to servo motor
      for (const auto & [target_id, q_value] : q_list) {
         hardware.set_position(links[target_id].dxl_id, q_value);
      }
      break;
  
      case inv_kin_picking:
      if (type_of_robot == "X7" ) {
        // Solve inverse kinematics and obtain joint angles
        samples03::x7_3dof_picking_inverse_kinematics(links, target_p, q_list);
      } else if (type_of_robot == "S17" ) {
        // Solve inverse kinematics and obtain joint angles
        samples03::s17_3dof_right_arm_picking_inverse_kinematics(links, target_p, q_list);		  
      }		  
      // Set joint angle to servo motor
      for (const auto & [target_id, q_value] : q_list) {
        hardware.set_position(links[target_id].dxl_id, q_value);
      }
	  break;

      case grav_comp:	  
      //if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
      //  std::cerr << "failed to start thread" << std::endl;
      //  return -1;
      //}
      //std::this_thread::sleep_for(std::chrono::seconds(5));

      kinematics_utils::link_id_t targ_id = 8;
      // Torque/current ratio A/Nm
      // Fine-tuned based on the parameters described in the Dynamixel e-manual
      samples03_dynamics::torque_to_current_t torque_to_current = {
      {2, 1.0 / 2.20},
      {3, 1.0 / 3.60},
      {4, 1.0 / 2.20},
      {5, 1.0 / 2.20},
      {6, 1.0 / 2.20},
      {7, 1.0 / 2.20},
      {8, 1.0 / 2.20}
      };

      std::cout << "entering loop.... press ESC to exit" << std::endl;
      while (1) {
        if (kbhit()) {
          if (getch() == ESC_ASCII_VALUE) {
            std::cout << "Esc was entered" << std::endl;
            break;
          }
        }
		for (auto group : group_names) {
          // Update posture
          std::vector<double> positions;
          if (hardware.get_positions(group, positions)) {
            set_arm_joint_positions(links, positions);
            kinematics::forward_kinematics(links, 1);
          }

          // Calculate the current value for gravity compensation here
          samples03_dynamics::gravity_compensation(links, targ_id, torque_to_current, q_list);
          for (const auto & [targ_id, q_value] : q_list) {
            hardware.set_current(links[targ_id].dxl_id, q_value);
          }
        }
	  }
	  //hardware.stop_thread();
      break;
	  
      case grav_comp_Pgain:	  
      if (!hardware.start_thread("arm", std::chrono::milliseconds(10))) {             
        std::cerr << "failed to start thread" << std::endl;
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::seconds(5));

      kinematics_utils::link_id_t targ_id = 8;
      // Torque/current ratio A/Nm
      // Fine-tuned based on the parameters described in the Dynamixel e-manual
      samples03_dynamics::torque_to_current_t torque_to_current = {
      {2, 1.0 / 2.20},
      {3, 1.0 / 3.60},
      {4, 1.0 / 2.20},
      {5, 1.0 / 2.20},
      {6, 1.0 / 2.20},
      {7, 1.0 / 2.20},
      {8, 1.0 / 2.20}
      };

      std::cout << "entering loop.... press ESC to exit" << std::endl;
      while (1) {
        if (kbhit()) {
          if (getch() == ESC_ASCII_VALUE) {
            std::cout << "Esc was entered" << std::endl;
            break;
          }
        }
		for (auto group : group_names) {
          // Update posture
          std::vector<double> positions;
          if (hardware.get_positions(group, positions)) {
            set_arm_joint_positions(links, positions);
            kinematics::forward_kinematics(links, 1);
          }

          // Calculate the current value for gravity compensation here
          samples03_dynamics::gravity_compensation(links, targ_id, torque_to_current, q_list);
		  // P control of the servo motor that moves the 3rd and 5th links
		  // It is a feedback control that the more the current angle of the motor is moved away from the target angle, 
		  // the greater the amount of control (current value).
          const double p_gain = 0.5;                                      // Proportional Gain
          const double target_q3 = M_PI_4;                                // set angle of servo motor bottom to 45 degree
          const double target_q5 = -(M_PI_2 + M_PI_4);                    // set angle of servo motor top to 135 degree
          q_list[3] += p_gain * (target_q3 - links[3].q); 
          q_list[5] += p_gain * (target_q5 - links[5].q);
          for (const auto & [targ_id, q_value] : q_list) {
            hardware.set_current(links[targ_id].dxl_id, q_value);
          }
        }
	  }
	  hardware.stop_thread();
      break;

      case grav_comp_PgainInt:	  
      if (!hardware.start_thread("arm", std::chrono::milliseconds(10))) {             
        std::cerr << "failed to start thread" << std::endl;
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::seconds(5));

      kinematics_utils::link_id_t targ_id = 8;
      // Torque/current ratio A/Nm
      // Fine-tuned based on the parameters described in the Dynamixel e-manual
      samples03_dynamics::torque_to_current_t torque_to_current = {
      {2, 1.0 / 2.20},
      {3, 1.0 / 3.60},
      {4, 1.0 / 2.20},
      {5, 1.0 / 2.20},
      {6, 1.0 / 2.20},
      {7, 1.0 / 2.20},
      {8, 1.0 / 2.20}
      };

      std::cout << "entering loop.... press ESC to exit" << std::endl;
      while (1) {
        if (kbhit()) {
          if (getch() == ESC_ASCII_VALUE) {
            std::cout << "Esc was entered" << std::endl;
            break;
          }
        }
		for (auto group : group_names) {
          // Update posture
          std::vector<double> positions;
          if (hardware.get_positions(group, positions)) {
            set_arm_joint_positions(links, positions);
            kinematics::forward_kinematics(links, 1);
          }

          // Calculate the current value for gravity compensation here
          samples03_dynamics::gravity_compensation(links, targ_id, torque_to_current, q_list);
		  // P I control of the servo motor that moves the 3rd and 5th links
		  // havent tried on s17 may need a change of those links and seperate PI (assume same for now)
		  // It is a feedback control that the more the current angle of the motor is moved away from the target angle, 
		  // the greater the amount of control (current value).
          const double p_gain = 0.5;                                      // Proportional Gain
		  const double i_gain = 0.1;                                      // integral term
		  const double i_list_max = 0.9;                                  // max calcualted integral term value is capped at this maxval
          const double target_q3 = M_PI_4;                                // set angle of servo motor bottom to 45 degree
          const double target_q5 = -(M_PI_2 + M_PI_4);                    // set angle of servo motor top to 135 degree
		  gettimeofday(&end_time, NULL);
          seconds = end_time.tv_sec - start_time.tv_sec; //seconds
          useconds = end_time.tv_usec - start_time.tv_usec; //milliseconds
          milli_time = ((seconds) * 1000 + useconds/1000.0);
		  double sam_time = static_cast<double>milli_time;
		  start_time = end_time;
		  double i_list = i_gain * (target_q3 - links[3].q) * ((sam_time)/1000.0f); 
          if (i_list >	i_list_max) {
			 std::cout << "integral term =" << i_list << " capped to " << i_list_max << std::endl;
             i_list = i_list_max;
          }			 
          q_list[3] += p_gain * (target_q3 - links[3].q) + i_list; 
          i_list = i_gain * (target_q5 - links[5].q) * ((sam_time)/1000.0f);
          if (i_list >	i_list_max) {
			 std::cout << "integral term =" << i_list << " capped to " << i_list_max << std::endl;
             i_list = i_list_max;
          }			  
          q_list[5] += p_gain * (target_q5 - links[5].q) + i_list;
          for (const auto & [targ_id, q_value] : q_list) {
            hardware.set_current(links[targ_id].dxl_id, q_value);
          }
        }
	  }
	  hardware.stop_thread();
      break;
	  
      default:
	  std::cout << "inavlid option for action type must be in range " << inv_k_LM << "-" << (number_of_OptionTypes-1) << std::endl;
	  break;
	  
  }

  // turn torque off
  if (type_of_robot == "X7" ) {
    if (!hardware.torque_on("arm")) {
      std::cerr << "arm : torque on failed" << std::endl;
      return -1;
    }	  
  } else if (type_of_robot == "S17" ) {
    for (auto group : group_names) {
      if (!hardware.torque_off(group)) {
        std::cerr << group << " : torque off failed" << std::endl;
        return -1;
      }
    }
  }
  
  hardware.disconnect();
  return 0;
  
}

// --------------------------------------------------------------------------------------------------------------------------
//     Waving Hands Robot analysing for example a Ricoh theta S 360 camera feed with yolov5 or yolov8 object classifacation
//
// -------------------------------------------------------------------------------------------------------------------------
//
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "unitree/robot/channel/channel_subscriber.hpp"
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/h1/loco/h1_loco_client.hpp>

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
unitree_go::msg::dds_::LowState_ state;

enum JointIndex {
  // Right arm
  kRightShoulderPitch = 12,
  kRightShoulderRoll = 13,
  kRightShoulderYaw = 14,
  kRightElbow = 15,
  // Left arm
  kLeftShoulderPitch = 16,
  kLeftShoulderRoll = 17,
  kLeftShoulderYaw = 18,
  kLeftElbow = 19,
};

// define the states for the arm
enum arm_states_e {
   move_forward,
   move_backward
};

const static std::vector<int> arm_joints = {
    JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
    JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
    JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
    JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow};

// save the state to a file (we are using csv so we can write more items if needed)
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>                        //  <stdlib.h> for exit()

void LowStateHandler(const void *message) {
  state = *(unitree_go::msg::dds_::LowState_ *)message;
}

std::vector<float> GetArmPosFromDds(unitree_go::msg::dds_::LowState_ state) {
  std::vector<float> vec = std::vector<float>(8, 0.f);
  for (int i = 0; i < 8; ++i) {
    vec.at(i) = state.motor_state().at(arm_joints.at(i)).q();
  }
  return vec;
}

// global move hands state
int g_moveHands;

int main(int argc, char const *argv[]) {

  int lastHands;
  std::string netIf;
  
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " \033[32m networkInterface yolo_version v5=0 v8=1" << std::endl;
	std::cout << " \033[34m using defaults eth0 yolo_version v8=1 \033[0m " << std::endl;
    netIf = "eth0";
  } else {
    netIf = argv[1];
  }

  // initialise the camera 
  CamCv camera;
  if (!camera.open(CAPTURE_INPUT, capture_width, capture_height, capture_fps))
  {
    throw std::runtime_error("Can't open capture device.");
  }

  // if the version of yolo was requested use it, if garbled then default it to v8, using stoi as better than atoi for human typed values
  if (argc >= 3 ) {
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
  
  // set up client
  unitree::robot::ChannelFactory::Instance()->Init(0, netIf);
  unitree::robot::h1::H1LocoClient client;

  client.Init();
  client.SetTimeout(10.0f);

  // set up feedback
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_>
      lowstate_subscriber;
  lowstate_subscriber.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
          "rt/lowstate"));
  lowstate_subscriber->InitChannel(
      std::bind(&LowStateHandler, std::placeholders::_1), 1);

  // arm ctrl related variables
  std::vector<float> init_pos = std::vector<float>(8, 0.f);
  std::vector<float> jpos = std::vector<float>(8, 0.f);
  //  This shows what the vector means
  //  JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
  //  JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
  //  JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
  //  JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow};
  //  define here your series of arm positions 
  std::vector<float> target_pos1{0.f, kPi_2,  0.f, kPi_2, 0.f, -kPi_2, 0.f, kPi_2};                                        // movement 1 arms up/down
  std::vector<float> target_pos2{0.f, -kPi_2/2.0f,  0.f, -kPi_2/4.0f, 0.f, kPi_2/2.0f, 0.f, -kPi_2/3.0f};                  // movement 2
  std::vector<float> target_pos3{0.f, kPi_2/5.0f,  0.f, -kPi_2/2.0f, 0.f, -kPi_2/7.0f, 0.f, kPi_2/3.0f};                   // movement 3
  std::vector<float> target_pos4{ kPi_2, 0.f, 0.f, kPi_2, -kPi_2, 0.f, 0.f, kPi_2};                                        // movement 4 forward back
  std::vector<float> target_pos5{ -kPi_2, 0.f, 0.f, kPi_2, kPi_2, 0.f, 0.f, kPi_2};                                        // movement 5 forward back
  std::vector<float> target_pos6{ 0.f, 0.f, kPi_2, kPi_2, 0.f, 0.f, -kPi_2, kPi_2};                                        // movement 6 side
  std::vector<float> target_pos7{ 0.f, 0.f, -kPi_2, kPi_2, 0.f, 0.f, kPi_2, kPi_2};                                        // movement 7 side
  std::vector<float> target_pos8{kPi_2/5.0f, 0.f, kPi_2/4.0f, -kPi_2/2.0f, -kPi_2/6.0f, 0.f, 0.f, kPi_2/3.0f};             // movement 8

  std::vector<float> dq = std::vector<float>(8, 0.f);
  std::vector<float> kp = std::vector<float>(8, 80.f);
  std::vector<float> kd = std::vector<float>(8, 2.f);
  std::vector<float> tau_ff = std::vector<float>(8, 0.f);

  // a series of arm position sequences 
  std::vector<int> move_arm_forward = std::vector<int>( 0, 1, 2, 3, 4, 5, 6, 7, 8 );
  std::vector<int> move_arm_backward = std::vector<int>( 8, 7, 6, 5, 4, 3, 2, 1, 0 );
  std::vector<int> move_arm;
  arm_states_e arm_action_state = move_forward;                                 // do each of these actions in the sequence fast mode 
  int arm_single_move = 0;                                                      // do this single action slow mode
  
  float weight = 0.f;
  float weight_rate = 0.6f;

  float control_dt = 0.02f;
  float max_joint_velocity = 0.5f;

  float delta_weight = weight_rate * control_dt;
  float max_joint_delta = max_joint_velocity * control_dt;
  auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  float init_stop_time = 2.f;
  int init_stop_num_steps = static_cast<int>(init_stop_time / control_dt);
  float arm_move_time = 5.f;
  int arm_move_num_steps = static_cast<int>(arm_move_time / control_dt);

  // read from a set-up file which stores the last state of the robot motion to restore to the robot when we boot up
  std::string str_buf;
  std::string str_conma_buf;
  static const char input_csv_file_path[] = "humanoid_state.csv";
  std::string output_csv_file_path = "humanoid_state.csv";
  // input file stream we will do an indiviual open/close as only read at boot-up
  //std::ifstream ifs_csv_file(input_csv_file_path, std::ios_base::in);
  std::ifstream ifs_csv_file;  
  // output file stream
  std::ofstream ofs_csv_file;
  // could open and keep open here as an alternative to open/close file (maybe slower but safer) 
  // std::ifstream ofs_csv_file(output_csv_file_path.c_str(), std::ios_base::out);
  
  try
  {
	    ifs_csv_file.open(input_csv_file_path, std::ios_base::in);                                              // read csv file which stored the walk state for recall
		
	    int line_num = 1;
        while (getline(ifs_csv_file, str_buf)) {                                                                // for each line in the file
		    if (line_num >= 2) {                                                                                // we only read the first line of the states file
				break;
			}
            // create stringstream input
            std::istringstream i_stream(str_buf);

            int file_val = 0;
		    int last_wave_hand_state = 0;
		   
            // read each line in the csv file -- values are separated by a comma
            while (getline(i_stream, str_conma_buf, ',')) {
			    if (file_val == 0) {                                                                             // first saved value
                    try                                                                                          // becasue we wrote it we use atoi (faster)
					{
				        last_wave_hand_state = atoi(str_conma_buf);                                              // start at the next step of the sequence as we were already active
					}
				    catch( std::exception e )
					{
				        std::cout << "error reading the file " << input_csv_file_path << " variables at default" << std::endl;
					}
                } else if (file_val == 1) {                                                                      // second saved value                					
                    try
					{
					    if (last_wave_hand_state == arm_states_e::START_WAVE) {                                  // we were active when the state was saved
				            arm_single_move = atoi(str_conma_buf) + 1;                                           // start at the next step of the sequence as we were already active and done that move
						} else {
				            arm_single_move = atoi(str_conma_buf);                                               // recall the step the mode was inactive when saved
                        }
                    }						
				    catch( std::exception e )
					{
				        std::cout << "error reading the file " << input_csv_file_path << " variables at default" << std::endl;
					}
                }
				++file_val;
            }
			line_num += 1;
        }
		ifs_csv_file.close();
    }
	catch( std::exception e )
	{
        std::cout << "error reading the file " << input_csv_file_path << " variables at default" << std::endl;
    }		
		
    // start main loop
    while (true) {

    // read and analyse the camera frame 
    camera.transmit();
    g_moveHands = camera.wave_hands;	

    // allow additional key pad controls	
    if (std::cin.peek() != EOF) {
      char key;
      std::cin >> key;
      if (key == 'j') {
        client.EnableCtrl();
      }
      if (key == 'k') {
        client.Stop();
      }
      if (key == 'o') {
        client.DecreaseSwingHeight();
      }
      if (key == 'p') {
        client.IncreaseSwingHeight();
      }
      if (key == 'l') {
        client.StandSwitch();
      }
      if (key == 'w') {
        client.Move(0.0, 0.5, 0.0);
      }
      if (key == 's') {
        client.Move(0.0, -0.5, 0.0);
      }
      if (key == 'a') {
        client.Move(0.5, 0.0, 0.0);
      }
      if (key == 'd') {
        client.Move(-0.5, 0.0, 0.0);
      }
      if (key == 'q') {
        client.Move(0., 0.0, 0.5);
      }
      if (key == 'e') {
        client.Move(0., 0.0, -0.5);
      }
      if (key == 'u') {
        client.Move(0.5, 0.5, 0.5);
      }
      if (key == 'j') {
        client.Move(-0.5, -0.5, -0.5);
      }
      if (key == 'o') {
        client.Move(0.5, 0.5, 0.0);
      }
      if (key == 'p') {
        client.Move(0.5, -0.5, 0.0);
      }
      if (key == 'k') {
        client.Move(-0.5, 0.5, 0.0);
      }
      if (key == 'l') {
        client.Move(-0.5, -0.5, 0.0);
      }
    }

    // now move hands according to the vision stream	
	switch(g_moveHands) {
	  
      case arm_states_e::NO_WAVE:
	  {
	    if (lastHands != g_moveHands) {
            // diable arm ctrl
            weight = 1.f;
            jpos = GetArmPosFromDds(state);
            for (int i = 0; i < init_stop_num_steps; ++i) {
              weight -= delta_weight;
              weight = std::clamp(weight, 0.f, 1.f);

              client.ArmCtrl(jpos, dq, kp, kd, tau_ff, weight);

              std::this_thread::sleep_for(sleep_time);
            }
			// write the states to a file for re-start step initialisation
			ofs_csv_file.open(output_csv_file_path.c_str(), std::ios_base::out);                   //  also could open and keep open may be faster 
            if( ! ofs_csv_file ) {                                                                 //  filw wont open
                std::cerr << "File open error ! " << output_csv_file_path << std::endl;
                    // exit(1);
            } else {
				ofs_csv_file.seekg(0, ofs_csv_file.beg);                                            // move to *beginning* of file
				ofs_csv_file << to_string(g_moveHands) << ',';                                      // store the hand action state state in a file for recall during re-start
				ofs_csv_file << to_string(arm_single_move) << ',';                                  // store the start_wave moving state in a file for recall during re-start
				ofs_csv_file << std::endl;
                if (ofs_csv_file.is_open()) {
				    ofs_csv_file.close();
				}
			}	
		}
      }
	  break;
	  
	  case arm_states_e::START_WAVE:
      {
		// ------------- enable control if from disabled ------------------------------
	    if (lastHands == arm_states_e::NO_WAVE) {		
            // enable arm ctrl
            weight = 0.f;
            for (int i = 0; i < init_stop_num_steps; ++i) {
                weight += delta_weight;
                weight = std::clamp(weight, 0.f, 1.f);
                client.ArmCtrl(init_pos, dq, kp, kd, tau_ff, weight * weight);
                std::this_thread::sleep_for(sleep_time);
            }
		}
		// ------------- move arm once per frame/cycle in a rotation ------------------------------
		switch (arm_single_move) {
            case 0:
			{
                // move arm to pos 0
                jpos = GetArmPosFromDds(state);
                for (int i = 0; i < arm_move_num_steps; ++i) {
                    for (int j = 0; j < init_pos.size(); ++j) {
                         jpos.at(j) += std::clamp(init_pos.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                    }

                    client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                    std::this_thread::sleep_for(sleep_time);
                }
				// move bot as well
				client.Move(0.5f, 0.0, 0.0);             // pitch, roll, yaw
				// write the states to a file for re-start step initialisation should we stop in mid steps. remember the move position via the case no.
				ofs_csv_file.open(output_csv_file_path.c_str(), std::ios_base::out);                   //  also could open and keep open may be faster 
                if( ! ofs_csv_file ) {                                                                 //  filw wont open
                    std::cerr << "File open error ! " << output_csv_file_path << std::endl;
                    // exit(1);
                } else {
					ofs_csv_file.seekg(0, ofs_csv_file.beg);                                            // move to *beginning* of file
				    ofs_csv_file << to_string(g_moveHands) << ',';                                      // store the hand action state state in a file for recall during re-start
				    ofs_csv_file << to_string(arm_single_move) << ',';                                  // store the start_wave moving state in a file for recall during re-start
				    ofs_csv_file << std::endl;
                    if (ofs_csv_file.is_open()) {
				        ofs_csv_file.close();
					}
				}
			}
			break;

            case 1:
			{			
                // move arm to pos 1
                jpos = GetArmPosFromDds(state);
                for (int i = 0; i < arm_move_num_steps; ++i) {
                    for (int j = 0; j < init_pos.size(); ++j) {
                        jpos.at(j) += std::clamp(target_pos1.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                    }

                    client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                    std::this_thread::sleep_for(sleep_time);
                }
				// move bot as well
				client.Move(0.0, 0.5f, 0.0);             // pitch, roll, yaw
				// write the states to a file for re-start step initialisation
				ofs_csv_file.open(output_csv_file_path.c_str(), std::ios_base::out);                   //  also could open and keep open may be faster 
                if( ! ofs_csv_file ) {                                                                 //  filw wont open
                    std::cerr << "File open error ! " << output_csv_file_path << std::endl;
                    // exit(1);
                } else {
					ofs_csv_file.seekg(0, ofs_csv_file.beg);                                            // move to *beginning* of file
				    ofs_csv_file << to_string(g_moveHands) << ',';                                      // store the hand action state state in a file for recall during re-start
				    ofs_csv_file << to_string(arm_single_move) << ',';                                  // store the start_wave moving state in a file for recall during re-start
				    ofs_csv_file << std::endl;
                    if (ofs_csv_file.is_open()) {
				        ofs_csv_file.close();
					}
				}
			}
			break;
			
            case 2:
     		{   
                 // move arm to pos 2
                jpos = GetArmPosFromDds(state);
                for (int i = 0; i < arm_move_num_steps; ++i) {
                    for (int j = 0; j < init_pos2.size(); ++j) {
                        jpos.at(j) += std::clamp(target_pos2.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                    }

                    client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                    std::this_thread::sleep_for(sleep_time);
                }
				// move bot as well
				client.Move(-0.5f, 0.0f, 0.0);             // pitch, roll, yaw
				// write the states to a file for re-start step initialisation
				ofs_csv_file.open(output_csv_file_path.c_str(), std::ios_base::out);                   //  also could open and keep open may be faster 
                if( ! ofs_csv_file ) {                                                                 //  filw wont open
                    std::cerr << "File open error ! " << output_csv_file_path << std::endl;
                    // exit(1);
                } else {
					ofs_csv_file.seekg(0, ofs_csv_file.beg);                                            // move to *beginning* of file
				    ofs_csv_file << to_string(g_moveHands) << ',';                                      // store the hand action state state in a file for recall during re-start
				    ofs_csv_file << to_string(arm_single_move) << ',';                                  // store the start_wave moving state in a file for recall during re-start
				    ofs_csv_file << std::endl;
                    if (ofs_csv_file.is_open()) {
				        ofs_csv_file.close();
					}
				}
			}
			break;

            case 3:
     		{   			
                // move arm to pos 3
                jpos = GetArmPosFromDds(state);
                for (int i = 0; i < arm_move_num_steps; ++i) {
                    for (int j = 0; j < init_pos3.size(); ++j) {
                        jpos.at(j) += std::clamp(target_pos3.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                    }

                    client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                    std::this_thread::sleep_for(sleep_time);
                }
				// move bot as well
				client.Move(0.0f, -0.5f, 0.0);             // pitch, roll, yaw
				// write the states to a file for re-start step initialisation
				ofs_csv_file.open(output_csv_file_path.c_str(), std::ios_base::out);                   //  also could open and keep open may be faster 
                if( ! ofs_csv_file ) {                                                                 //  filw wont open
                    std::cerr << "File open error ! " << output_csv_file_path << std::endl;
                    // exit(1);
                } else {
					ofs_csv_file.seekg(0, ofs_csv_file.beg);                                            // move to *beginning* of file
				    ofs_csv_file << to_string(g_moveHands) << ',';                                      // store the hand action state state in a file for recall during re-start
				    ofs_csv_file << to_string(arm_single_move) << ',';                                  // store the start_wave moving state in a file for recall during re-start
				    ofs_csv_file << std::endl;
                    if (ofs_csv_file.is_open()) {
				        ofs_csv_file.close();
					}
				}
			}
			break;
			
			default:
			break;
		}
		++arm_single_move % 4;                                  // do each action in a cycle
      }
	  break;
	  
	  case arm_states_e::FAST_WAVE:
      {
		// ------------- enable control if from disabled ------------------------------
	    if (lastHands == arm_states_e::NO_WAVE) {		
            // enable arm ctrl
            weight = 0.f;
            for (int i = 0; i < init_stop_num_steps; ++i) {
                weight += delta_weight;
                weight = std::clamp(weight, 0.f, 1.f);
                client.ArmCtrl(init_pos, dq, kp, kd, tau_ff, weight * weight);
                std::this_thread::sleep_for(sleep_time);
            }

		}
		if (lastHands != g_moveHands) {                                                            // if we had a state change write the step to file for re-call
			// write the states to a file for re-start step initialisation
			ofs_csv_file.open(output_csv_file_path.c_str(), std::ios_base::out);                   //  also could open and keep open may be faster 
            if( ! ofs_csv_file ) {                                                                 //  filw wont open
                std::cerr << "File open error ! " << output_csv_file_path << std::endl;
                    // exit(1);
            } else {
				ofs_csv_file.seekg(0, ofs_csv_file.beg);                                            // move to *beginning* of file
				ofs_csv_file << to_string(g_moveHands) << ',';                                      // store the hand action state state in a file for recall during re-start
				ofs_csv_file << to_string(arm_single_move) << ',';                                  // store the start_wave moving state in a file for recall during re-start
				ofs_csv_file << std::endl;
                if (ofs_csv_file.is_open()) {
				    ofs_csv_file.close();
				}
			}	
		}

        // ------------- arm motion state engine ------------------------------
        // toggle the mode of action between forward and backward
        if (arm_action_state == move_forward) {
            arm_action = move_arm_forward;
            arm_action_state = move_backward;
        else if (arm_action_state == move_backward) {
            arm_action = move_arm_backward;
            arm_action_state = move_forward;
        }

        // ------------- perform designated arm actions ------------------------
        for (auto& movement_state : arm_action) 
		{		
	    	switch(movement_state) 
			{
		
                case 0:
                {			
                    // move arm to pos 0
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos.size(); ++j) {
                            jpos.at(j) += std::clamp(init_pos.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;

                case 1:	
                {			
                    // move arm to pos 1
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos1.size(); ++j) {
                            jpos.at(j) += std::clamp(target_pos1.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;

                case 2:
     		    {   
                     // move arm to pos 2
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos2.size(); ++j) {
                            jpos.at(j) += std::clamp(target_pos2.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;

                case 3:
     		    {   			
                    // move arm to pos 3
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos3.size(); ++j) {
                            jpos.at(j) += std::clamp(target_pos3.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;
			
                case 4:
     		    {   
                    // move arm to pos 4
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos4.size(); ++j) {
                            jpos.at(j) += std::clamp(target_pos4.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;

                case 5:
     		    {   			
                    // move arm to pos 5
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos5.size(); ++j) {
                            jpos.at(j) += std::clamp(target_pos5.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;
			
                case 6:
     		    {   			
                    // move arm to pos 6
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos6.size(); ++j) {
                            jpos.at(j) += std::clamp(target_pos6.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;
			
                case 7:
     		    {   
                    // move arm to pos 7
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos7.size(); ++j) {
                           jpos.at(j) += std::clamp(target_pos7.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;

                case 8:
     		    {   			
                    // move arm to pos 8
                    jpos = GetArmPosFromDds(state);
                    for (int i = 0; i < arm_move_num_steps; ++i) {
                        for (int j = 0; j < init_pos8.size(); ++j) {
                            jpos.at(j) += std::clamp(target_pos8.at(j) - jpos.at(j), -max_joint_delta, max_joint_delta);
                        }

                        client.ArmCtrl(jpos, dq, kp, kd, tau_ff, 1.f);
                        std::this_thread::sleep_for(sleep_time);
                    }
			    }
			    break;
			
			    default:
			    break;
            }
		}
		ofs_csv_file << to_string(g_moveHands) << ',';                                      // store the hand action state state in a file for recall during re-start
		ofs_csv_file << to_string(arm_single_move) << ',';                                  // store the start_wave moving state in a file for recall during re-start
		ofs_csv_file << std::endl;
      }
	  break;
	  
	}
	lastHands = g_moveHands;
  }

  return 0;
}

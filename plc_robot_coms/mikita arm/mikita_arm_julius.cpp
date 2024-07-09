/* control example of mikita arm robot and gripper
   ref :- https://github.com/kenichi-ohara/MikataArm/blob/main/open_manipulator/open_manipulator_teleop/src/open_manipulator_teleop_joystick.cpp
   requires https://github.com/kenichi-ohara/MikataArm/tree/main/open_manipulator
*/
﻿/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
#include <ros/ros.h>

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */
/* Authors: AirCamPro combined speach with mikita arm example as shown */

#include "open_manipulator_teleop_joystick.h"

/* https://github.com/julius-speech/julius/ */
/*
 * Copyright (c) 1991-2016 Kawahara Lab., Kyoto University
 * Copyright (c) 2000-2005 Shikano Lab., Nara Institute of Science and Technology
 * Copyright (c) 2005-2016 Julius project team, Nagoya Institute of Technology
 * All rights reserved
 */

/* include top Julius library header for speach regognition */
#include <julius/juliuslib.h>

/* #define _use_strmatch if you have the imatix library */
#ifdef _use_strmatch
/* for strmatch https://imatix-legacy.github.io/sfl/sfldoc.htm#TOC281 */
#include "sflstr.h"
/* #define _use_strcmp to use it also a bit legacy now */
#elif _use_strcmp
#include <string.h>
/* use string in c++11 */
#else
#include <stdio.h>
#include <string>
#include <iostream>
#endif
/* define the words spoken to command the robot */
#define SPK_CLOSE "close"
#define SPK_OPEN "open"

OpenManipulatorTeleop::OpenManipulatorTeleop()
    :node_handle_(""),
     priv_node_handle_("~")
{
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  initClient();
  initSubscriber();

  ROS_INFO("OpenManipulator initialization");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  if(ros::isStarted()) {
    ros::shutdown();                                                       // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

void OpenManipulatorTeleop::initClient()
{
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");

}
void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  joy_command_sub_ = node_handle_.subscribe("joy", 10, &OpenManipulatorTeleop::joyCallback, this);
}

// gets the present joint angles
void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint5"))  temp_angle.at(4) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint6"))  temp_angle.at(5) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;

}

// reads the pose and puts into present kinematics position
void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

// joystick callback 
void OpenManipulatorTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->axes.at(1) >= 0.9) setGoal("x+");
  else if(msg->axes.at(1) <= -0.9) setGoal("x-");
  else if(msg->axes.at(0) >=  0.9) setGoal("y+");
  else if(msg->axes.at(0) <= -0.9) setGoal("y-");
  else if(msg->buttons.at(3) == 1) setGoal("z+");
  else if(msg->buttons.at(0) == 1) setGoal("z-");
  else if(msg->buttons.at(5) == 1) setGoal("home");
  else if(msg->buttons.at(4) == 1) setGoal("init");
  else if(msg->buttons.at(2) == 1) setGoal("step_1");
  else if(msg->buttons.at(1) == 1) setGoal("step_2");
/*
  if(msg->buttons.at(2) == 1) setGoal("gripper close");
  else if(msg->buttons.at(1) == 1) setGoal("gripper open");
*/
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if(goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::setGoal(const char* str)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if(str == "x+")
  {
    printf("increase(++) x axis in cartesian space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "x-")
  {
    printf("decrease(--) x axis in cartesian space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "y+")
  {
    printf("increase(++) y axis in cartesian space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "y-")
  {
    printf("decrease(--) y axis in cartesian space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "z+")
  {
    printf("increase(++) z axis in cartesian space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(str == "z-")
  {
    printf("decrease(--) z axis in cartesian space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }

  else if(str == "gripper open")
  {
    printf("open gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.7);
    setToolControl(joint_angle);
  }
  else if(str == "gripper close")
  {
    printf("close gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.7);
    setToolControl(joint_angle);
  }
  /* these are all pre-programmed steps for the robot */
  else if(str == "home")                               
  {
    printf("home pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.18);
    joint_name.push_back("joint3"); joint_angle.push_back(0.620);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.568);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(str == "init")
  {
    printf("init pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.0);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(str == "step_1")
  {
    printf("step_1 pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(1.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.2);
    joint_name.push_back("joint3"); joint_angle.push_back(0.5);
    joint_name.push_back("joint4"); joint_angle.push_back(-0.60);
    joint_name.push_back("joint5"); joint_angle.push_back(0.70);
    joint_name.push_back("joint6"); joint_angle.push_back(-.08);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(str == "step_2")
  {
    printf("step_2 pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.20);
    joint_name.push_back("joint2"); joint_angle.push_back(0.1);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.5);
    joint_name.push_back("joint4"); joint_angle.push_back(0.60);
    joint_name.push_back("joint5"); joint_angle.push_back(-0.70);
    joint_name.push_back("joint6"); joint_angle.push_back(0.28);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

/** 
 * Callback to be called when start waiting speech input. 
 * 
 */
static void status_recready(Recog *recog, void *dummy)
{
  if (recog->jconf->input.speech_input == SP_MIC || recog->jconf->input.speech_input == SP_NETAUDIO) {
     fprintf(stderr, "<<< please speak >>>");
  }
}

/** 
 * Callback to be called when speech input is triggered.
 * 
 */
static void status_recstart(Recog *recog, void *dummy)
{
  if (recog->jconf->input.speech_input == SP_MIC || recog->jconf->input.speech_input == SP_NETAUDIO) {
     fprintf(stderr, "\r                    \r");
  }
}

/** 
   Output of recognition result of 2nd path and command the gripper from voice
 */
static void voc_command_gripper(Recog* recog, void* dummy)
{
 RecogProcess* recogProcess = recog->process_list;

 WORD_ID* seq = recogProcess->result.sent->word;
 int seqnum = recogProcess->result.sent->word_num;
 WORD_INFO* winfo = recogProcess->lm->winfo;

 if (seq != NULL)
 {
   for (int i = 0; i < seqnum; i++)
   {
      printf("%s", winfo->woutput[seq[i]]);
/* if we heard open or close then control the gripper as spoken */
#ifdef _use_strmatch
      if (strmatch(winfo->woutput[seq[i]], SPK_CLOSE))) {
          setGoal("gripper close");
      } else if (strmatch(winfo->woutput[seq[i]], SPK_OPEN))) {
          setGoal("gripper open");		 
      }
#elif _use_strncmp
	  if ((std::strlen(winfo->woutput[seq[i]]) == std::strlen(SPK_CLOSE)) &&
         (std::strncmp(winfo->woutput[seq[i]], SPK_CLOSE, std::strlen(SPK_CLOSE)) == 0)) {
         setGoal("gripper close");
      } else if ((std::strlen(winfo->woutput[seq[i]]) == std::strlen(SPK_OPEN)) &&
         (std::strncmp(winfo->woutput[seq[i]], SPK_OPEN, std::strlen(SPK_OPEN)) == 0)) {
         setGoal("gripper open");
      }
#else
       std::string s1 = std::string(winfo->woutput[seq[i]]);
       std::string s2 = s1;
       std::transform(
        s1.begin(), 
        s1.end(), 
        s2.begin(),                         
        [](char c) { return std::tolower(c); }
      );
      if (s2 == SPK_CLOSE) {
         setGoal("gripper close");
      } else if (s2 = SPK_OPEN) {
         setGoal("gripper open");		  
      }
#endif
   }
   printf("\n");
 }
}

/* ======================================= main program ======================================= */
int main(int argc, char **argv)
{
  const char* jconffile = "sample.jconf";

  /* Assign a new whole parameter structure */
  Jconf* jconf = j_jconf_new();

  /* Read parameters from the jconf file and store values in each configuration instance in jconf */
  if (j_config_load_file(jconf, jconffile) == -1) {
    fprintf(stderr, "failed to load config");
    return -1;
  }

  /* Finally determine the jconf configuration parameters */
  if (j_jconf_finalize(jconf) == FALSE) {
    fprintf(stderr, "param error");
    return -1;
  }

  /* Allocate new memory for the engine instance */
  Recog* recog = j_recog_new();
  recog->jconf = jconf;

  /* Read all models and prepare them for recognition */
  if (j_load_all(recog, jconf) == FALSE) {
    fprintf(stderr, "speach recog model error");
    return -1;
  }
  
  /* Final configuration of the engine instance from all loaded models and settings */
  if (j_final_fusion(recog) == FALSE)
  {
    j_recog_free(recog);
    fprintf(stderr, "failed speach engine fusion");
    return -1;
  }

  /* Register a functions in the callback registry for julius speach recognition engine */
  callback_add(recog, CALLBACK_EVENT_SPEECH_READY, status_recready, NULL);
  callback_add(recog, CALLBACK_EVENT_SPEECH_START, status_recstart, NULL);
  callback_add(recog, CALLBACK_RESULT, voc_command_gripper, NULL);
 
  /* Init ROS node */
  ros::init(argc, argv, "open_manipulator_TELEOP");
  OpenManipulatorTeleop openManipulatorTeleop;

  ROS_INFO("OpenManipulator teleoperation using joystick start");

  /* thia would cause us to block the speach code below 
  ros::spin();
  
  also we cant use
  ros::spinOnce() as j_recognize_stream is a blocking call waiting for speach and we need the joystick control all the time
  
  */
  
  ros::AsyncSpinner s(1);    /* start with one thread */
  ros::Rate rate(10);        /* 10 Hz */
  s.start();

  while(ros::ok()){ 
      /* Initialize the A/D-in device selected in the settings and prepare for recognition */
      if (j_adin_init(recog) == FALSE) {
         fprintf(stderr, "error with microphone A/D");	  
         return -1;
      }
  
      /* Open the audio input stream
         (the input source can also be specified with the 2nd argument of this function instead of jconf)
          raw speech input (microphone etc.) */
      switch(j_open_stream(recog, NULL)) {
          case 0:			                                             /* succeeded */
          break;
          case -1:      		                                         /* error */
          fprintf(stderr, "error in input stream\n");
          return -1;
          case -2:			                                         /* end of recognition process */
          fprintf(stderr, "failed to begin input stream\n");
          return -1;
      }

      /* Recognize input stream */
      if (j_recognize_stream(recog) == -1) {
         fprintf(stderr, "error with recognition");	
         j_recog_free(recog);
         return -1;
      }
 
      /* calling j_close_stream(recog) at any time will terminate
         recognition and exit j_recognize_stream() */
      j_close_stream(recog);
	 
	 rate.sleep();
  }
  
  s.stop();
  /* Free the speach engine instance */
  j_recog_free(recog);
	 
  printf("Teleop. is finished\n");
  return 0;
}

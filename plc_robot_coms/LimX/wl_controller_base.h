/**
 * @file wl_controller_base.h
 * @brief Header file for the base class of the controller.
 * @version 1.0
 * @date 2023-12-11
 *
 * © [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
 *
 */
#pragma once

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <algorithm>
#include <Eigen/Dense>
#include "limxsdk/wheellegged.h"
#include "common/util.h"

/**
 * @brief Base class for the controller.
 *
 */
class ControllerBase
{
public:
  ControllerBase();  // Constructor
  ~ControllerBase(); // Destructor

protected:
  virtual void starting() = 0; // Virtual function for starting the controller
  virtual void update() = 0;   // Virtual function for updating the controller

  /**
   * @brief Start the control loop.
   *
   */
  void startLoop();

  /**
   * @brief Load parameters.
   *
   */
  void loadParameters();

  /**
   * @brief Publish zero torque command.
   *
   */
  void zeroTorque();

  /**
   * @brief Publish damping command.
   *
   */
  void damping();

  /**
   * @brief Perform homing.
   *
   * @param v The average speed of motion.
   */
  void homing(double v = 1.0);

  /**
   * @brief Perform standing.
   *
   * @param v The average speed of motion.
   */
  void standing(double v = 0.3);

  /**
   * @brief Perform sitting down.
   *
   * @param v The average speed of motion.
   */
  void sitDown(double v = 0.5);

  /**
   * @brief Moves the robot to its move position at a given velocity.
  */
  void move_from_joy(double x, double y);

  /**
   * @brief Publish a message.
   *
   * @param msg The message to publish.
   */
  void publish(limxsdk::RobotCmd &msg);

  /**
   * @brief Change PD settings
   */
  void mod_params(double pb, double pd);

  /**
   * @brief Set timestep
   */
  void set_dt(double d);
  
  /**
   * @brief Get the current robot state.
   *
   * @return The current robot state.
   */
  limxsdk::RobotState robotState();

  /**
   * @brief Get the current imu data.
   *
   * @return The current imu data.
   */
  limxsdk::ImuData imuData();

  /**
   * @brief Move to a specific joint position.
   *
   * @param q The target joint positions.
   * @param q_d The desired joint positions.
   * @param speed The movement speed.
   * @param dt The time step.
   */
  void goPos(const Eigen::VectorXd q, const Eigen::VectorXd q_d, double speed, double dt);

  limxsdk::Wheellegged *wl_;    // Pointer to the WheelLegged object
  limxsdk::RobotCmd robot_cmd_; // Robot command
  bool recv_{false};            // Flag indicating if data has been received
  double dt_{2e-3};             // Time step

private:
  double calcCos(double start, double stop, double T, double t) const;
  void robotStateCallback(const limxsdk::RobotStateConstPtr &msg);
  void imuDataCallback(const limxsdk::ImuDataConstPtr &msg);

  limxsdk::RobotState robot_state_; // Robot state
  limxsdk::ImuData imu_data_;       // Robot imu
  std::thread th_loop_;             // Thread for control loop
  std::mutex mtx_;                  // Mutex for thread synchronization

  Eigen::VectorXd joint_kp_;     // Proportional gains for joints
  Eigen::VectorXd joint_kd_;     // Derivative gains for joints
  Eigen::VectorXd damping_kd_;   // Derivative gains for damping
  Eigen::VectorXd home_q_;       // Home joint positions
  Eigen::VectorXd stand_q_;      // Standing joint positions
};
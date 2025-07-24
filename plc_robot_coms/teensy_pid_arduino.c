// Teensy PID from pupper project modified from here https://github.com/cs123-stanford/lab_2_bad_robot_surgeon/blob/main/src/pid.h
//
#pragma once

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

// global variables for the derivative controller
int g_der_state = 0;
float g_last_error = 0.0;
long g_der_start_tm = millis();

// a structure that represents the same as the global variables above
typedef struct {
    int state;
    float last_error;
    long start_ms;
    float term_value;
} derivative_obj_t;

// P control here
float proportional_control(float theta_cur, float theta_target, float Kp) {
    float c = (thets_target - theta_cur) * Kp;
    return c;                                                                     // current in mA
}

// D control here. This function assumes that the desired velocity is determined by the arm fully stopping.
float derivative_control(float omega_cur, float Kd) {
    float c = 0.0;                                                             // computed result
    float e = 0.0;                                                             // error from desired 
    float des_velo = 0.0;                                                      // desired velocity is zero then the arm is stopped
    long time_from_function = millis();
    long der_time_period = time_from_function - g_der_start_tm;                // time from last function call
    g_der_start_tm = time_from_function;	
    if (g_der_state == 0) {
        g_last_error = (des_velo - omega_cur);
        g_der_state = 1;
    } else {
        e = (des_velo - omega_cur);
        c = (e - g_last_error) * Kd; 
		c /= ((float)der_time_period);
        g_last_error = e;		
    }
    return c;                                                                    // Current in mA
}

// example using the derivative_obj_t structure rather using global variables 
void derivative_control_2(float omega_cur, float Kd, derivative_obj_t* dv) {
    float e = 0.0;                                                             // error from desired 
    float des_velo = 0.0;                                                      // desired velocity is zero then the arm is stopped
    long time_from_function = millis();
    long der_time_period = time_from_function - dv->start_ms;                // time from last function call
    dv->start_ms = time_from_function;	
    if (dv->state == 0) {
        dv->last_error = (des_velo - omega_cur);
        dv->state = 1;
    } else {
        e = (des_velo - omega_cur);
        dv->term_value = (e - dv->last_error) * Kd; 
		dv->term_value /= ((float)der_time_period);
        dv->last_error = e;		
    }
}

// PD Controller 
float pd_control(float pos, float vel, float target, float Kp, float Kd)
{
  return (proportional_control(pos, target, Kp) + derivative_control(vel, Kd));
}

template <int n>
BLA::Matrix<n> vectorized_pd(const BLA::Matrix<n> &pos,
                             const BLA::Matrix<n> &vel,
                             const BLA::Matrix<n> &target,
                             float Kp,
                             float Kd)
{
  return (proportional_control(pos, target, Kp) + derivative_control(vel, Kd));
}

float sanitize_current_command(float command, float pos, float vel, float max_current = 2000, float max_pos = 3.141, float max_vel = 30)
{
  /* Sanitize current command to make it safer.

  Clips current command between bounds. Reduces command if actuator outside of position or velocity bounds.
  Max current defaults to 1000mA. Max position defaults to +-180degs. Max velocity defaults to +-5rotations/s.
  */
  command = command > max_current ? max_current : command;
  command = command < -max_current ? -max_current : command;
  if (pos > max_pos || pos < -max_pos)
  {
    Serial.println("WARNING: Actuator position outside of allowed bounds.");
  }
  if (vel > max_vel || vel < -max_vel)
  {
    Serial.println("ERROR: Actuactor velocity outside of allowed bounds. Setting torque to 0.");
    return 0.0;
  }
  return command;
}

template <int n>
BLA::Matrix<n> vectorized_sanitize(const BLA::Matrix<n> &command,
                                   const BLA::Matrix<n> &angles,
                                   const BLA::Matrix<n> &velocities,
                                   float max_current = 2000,
                                   float max_pos = 3.141,
                                   float max_vel = 30)
{
  BLA::Matrix<n> command_copy;
  for (int i = 0; i < n; i++)
  {
    command_copy(i) = sanitize_current_command(command(i), angles(i), velocities(i), max_current, max_pos, max_vel);
  }
  return command_copy;
}
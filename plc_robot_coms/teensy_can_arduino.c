// Example of PD controller for teensy with can bus
//
#include "Arduino.h"
#include "C610Bus.h"

long last_command = 0;                                // To keep track of when we last commanded the motors
C610Bus<CAN2> bus;                                    // Initialize the Teensy's CAN bus to talk to the motors

const int LOOP_DELAY_MILLIS = 5;                      // Wait for 0.005s between motor updates. Change this in Step 10. 
const int PID_SCAN_MILLIS = 10;
const int BB_SCAN_MILLIS = 500;
int g_loop_scan_tm = millis();

const float m1_offset = 0.0;
const float m2_offset = 0.0;

// global variables for the derivative controller
int g_der_state = 0;
float g_last_error = 0.0;
long g_der_start_tm = millis();

// integral controller
float g_acc_err = 0.0;

//Step 6. Function that returns the commanded current according to bang bang control
float bang_bang_control(float m_pos) {
    float I_command = 800.0;
    float result = 0.0f;
    if(m_pos < 0.0) {
      result = I_command;
    } else {
      result = -1.0 * I_command;
    }
    return result
}

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

// I control here. Not used 
float integral_control(float omega_cur, float Ki, float desired_meas) {                                                          
    float e = desired_meas - omega_cur;                                                             // error from desired 
    g_acc_err += e;
    float c = Ki * g_acc_err;                                                                      // computed result
    return c;                                                                                      // Current in mA
}

float reset_integral_term() {
	g_acc_err = 0.0;
}

// PD Control code, draws from proportional_control and derivative_control. This function assumes that the desired velocity is determined by the arm fully stopping. Returns a commanded current
float pd_control(float pos, float vel, float target, float Kp, float Kd)
{
    const int max_cur = 20.0;
	const int min_cur = 0.0;
    float I_command = proportional_control(pos, target, Kp) + derivative_control(vel, Kd); 
    return I_command;                                                             // Current in mA
}

void sanitize_current_command(float &command,float pos, float vel, float max_current = 2000, float max_pos = 3.141, float max_vel = 30, float reduction_factor = 0.1)
{
  /* Sanitize current command to make it safer.

  Clips current command between bounds. Reduces command if actuator outside of position or velocity bounds.
  Max current defaults to 1000mA. Max position defaults to +-180degs. Max velocity defaults to +-5rotations/s.
  */
  command = command > max_current ? max_current : command;
  command = command < -max_current ? -max_current : command;
  if (pos > max_pos || pos < -max_pos)
  {
    Serial.println("ERROR: Actuator position outside of allowed bounds.");
  }
  if (vel > max_vel || vel < -max_vel)
  {
    command = 0;
    Serial.println("ERROR: Actuactor velocity outside of allowed bounds. Setting torque to 0.");
  }
}

// Motorstate struct for keeping the measurement and output updates together
typedef struct {
  float pos;
  float vel;
  float cmd;
} MotorState;

MotorState m_state;                                                     // MotorStrate struct object for updating the motor state of the left leg

// updates the given motorstate object with its current motor position and velocity according to the motor ID
void updateState(MotorState* state, int id) {
  state->pos = bus.Get(id).Position();                                  // update position in radians
  state->vel = bus.Get(id).Velocity();                                  // update velocity in radians/sec
}

// updates the given MotorStates command current
void updateCmd(MotorState* state, float target, float kp, float kd, int *step_no) {
  long act_millis = millis();
  if (*step_no > 6) {
      if ((act_millis - g_loop_scan_tm) > PID_SCAN_MILLIS) {                                  // only do it every PID_SCAN_MILLIS
          state->cmd = pd_control(state->pos, state->vel, target, kp, kd);                    // use this line for PD Control in Steps 7-11
		  g_loop_scan_tm = act_millis;
      }
  } else {
      if ((act_millis - g_loop_scan_tm) > BB_SCAN_MILLIS) {                                  // only do it every BB_SCAN_MILLIS
          state->cmd = bang_bang_control(state->pos);                                            // bang bang control in Step 6. Comment again before Step 7.
          *step_no++                                                                             // increment the step 
		  g_loop_scan_tm = act_millis;
      }		  
  }
}

// This code waits for the user to type s before executing code.
void setup()
{
  // Remove all characters that might have been stored up in the serial input buffer prior to running this program
  while (Serial.available()) {
    Serial.read();
  }
  long last_print = millis();
  while (true)
  {
    char c = Serial.read();
    if (c == 's')
    {
      Serial.println("Starting code.");
      break;
    }
    if (millis() - last_print > 2000) {
      Serial.println("Press s to start.");
      last_print = millis();
    }
  }
}

void loop()
{
  bus.PollCAN();                                              // Check for messages from the motors.
  long now = millis();                                        // start-up time
  int step_v = 0;                                             // initial start-up step

  // PD controller terms
  float Kp = 1000.0;
  float Kd = 0.0;
  
  // if for breaking out with 's'
  if (Serial.available())
  {
    if (Serial.read() == 's')
    {
      bus.CommandTorques(0, 0, 0, 0, C610Subbus::kOneToFourBlinks);
      Serial.println("Stopping.");
      while (true)
      {
      }
    }
  }

  if (now - last_command >= LOOP_DELAY_MILLIS)
  {
    updateState(&m_state, 0);                                // set your motor state to be ID=0, get current motor position and velocity

    // Block to print out the current position and velocity of the motor
    Serial.print("m_pos: ");
    Serial.print(m_state.pos);                                // Print the shaft position of motor 0 in radians.
    Serial.print("\tm_vel: ");
    Serial.print(m_state.vel);                                // Print the shaft velocity of motor 0 in radians/sec.

    float target_position = 0.0;                              // set target_position to the initial starting position (straight up) for Steps 5-10.
    //float time = millis() / 1000.0;                         // used in Step 11. for calculating periodic target positions
    //float target_position = sin(time);                      // set sinusoidal target position in Step 11.

    updateCmd(&m_state, target_position, Kp, Kd, &step_v);             // updates the MotorState object current command based on pd_control values

    sanitize_current_command(m_state.cmd, m_state.pos, m_state.vel);           // Sanitizes your computed current commands to make the robot safer.
    bus.CommandTorques(m_state.cmd, 0, 0, 0, C610Subbus::kOneToFourBlinks);    // Only call CommandTorques once per loop! Calling it multiple times will override the last command.

    last_command = now;
    Serial.println();
  }
}
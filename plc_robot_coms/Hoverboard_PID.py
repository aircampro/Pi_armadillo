#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Example of two wheel traction using Hoverboard
# Based on work by Franz Pucher, Diffbot, (2020), GitHub repository, https://github.com/fjp/diffbot
#
# ported to python by AirCamPro NMO
#
from Hoverboard_Serial_Servo import serial_send_cmd_t, serial_rcv_cmd_t, Hoverboard
import numpy as np
import time

# raw PID controller 
#
def PID(kp, ki, kd, err, error_sum, error_pre, dt, imin, imax):
    error = err
    error_sum += error 
    error_sum = np.clip(delta, imin, imax)
    error_diff = error-error_pre 
    m = (kp * error) + (ki * error_sum) + (kd*error_diff)*dt 
    return m, error_sum, error

# class to operate the motion of the hoverboard_pid
#
class hoverboard_pid():

    def __init__(self, wr, mv, spt, dc=False):
        self.f_ = 0.0;
        self.error_ = 0.0;
        self.error_sum = 0.0;
        self.error_pre = 0.0;
        self.output = 0.0;
        self.out_min_ = 0;
        self.out_max_ = 100.0;
        self.wheel_radius = wr;
        self.max_velocity = mv;
        self.theta_goal = spt;
        self.theta_current = 0.0;
        self.direction_correction = dc;
        
    def init(self, f, p, i, d, i_max, i_min, antiwindup=False):
        self.f_ = f;
        self.initPid(p, i, d, i_max, i_min, antiwindup);
        self.error_ = 0.0;
        self.max_velocity /= self.wheel_radius;
        self.out_min_ = -self.max_velocity;
        self.out_max_ = self.max_velocity;
        
    def initPid( p, i, d, imax, imin, antiwindup=False):
        self.p = p;
        self.i = i;
        self.d = d;
        self.imin = imin;
        self.imax = imax;
        self.aw = antiwindup
        
    def update(self, measured_value, setpoint, dt):
        self.theta_current = measured_value
        self.theta_goal = setpoint
        if self.direction_correction == True:
            self.error_ = self.theta_current - self.theta_goal;	  
        else:            
            self.error_ = self.theta_goal - self.theta_current;	
        # Reset the i_error in case the p_error and the setpoint is zero
        # Otherwise there will always be a constant i_error_ that won't vanish
        if (0.0 == self.theta_goal and 0.0 == error_) :
	        # reset() will reset
	        # p_error_last_ = 0.0;
	        # p_error_ = 0.0;
	        # i_error_ = 0.0;
	        # d_error_ = 0.0;
	        # cmd_ = 0.0;
	        self.reset();
        # Call PID Loop
        self.computeCommand(self.error_, dt);
        # Compute final output including feed forward term
        self.output = self.f_ * self.theta_goal + self.output;
        self.output = self.clamp(self.output, self.out_min_, self.out_max_);
        self.set_speed()
        
    def reset(self):
        self.error_ = 0.0;
        self.error_sum = 0.0;
        self.error_pre = 0.0;
        
    def clamp(self, value, lower_limit, upper_limit):
        return np.clip(value, lower_limit, upper_limit);    
        
    def computeCommand(self, e, dt):
        self.output, self.error_sum, self.error = PID(self.p, self.i, self.d, self.error, self.error_sum, self.error_pre, dt, self.imin, self.imax )
      
    # Convert PID outputs in RAD/S to RPM
    def set_speed(self):
        self.speed = self.output / 0.10472


# class for writing the 2 wheel PID's to the hoverboard controller over serial
#
class Hover_Comms():

    def __init__(self,dc=1.0,lwf=0.3,hwf=0.7,tpr=90,em=9000,msa=45.0):
        self.hover = Hoverboard()
        self.hover.open_port()
        self.hover.set_port()  
        self.dir_correction = dc;  
        # Velocities read from RPM to RAD/S
        self.r_speed = 0.0;
        self.l_speed = 0.0;
        self.b_volts = 0.0;
        self.b_temp = 0.0;
        self.upd = False
        self.ENCODER_MIN = 0
        self.ENCODER_MAX = em
        self.ENCODER_LOW_WRAP_FACTOR = lwf
        self.ENCODER_HIGH_WRAP_FACTOR = hwf
        self.TICKS_PER_ROTATION = tpr
        self.low_wrap = self.ENCODER_LOW_WRAP_FACTOR*(self.ENCODER_MAX - self.ENCODER_MIN) + self.ENCODER_MIN;
        self.high_wrap = self.ENCODER_HIGH_WRAP_FACTOR*(self.ENCODER_MAX - self.ENCODER_MIN) + self.ENCODER_MIN;
        self.last_wheelcountR = self.last_wheelcountL = 0;
        self.multR = self.multL = 0;
        self.lastPosL = 0;
        self.lastPosR = 0;
        self.lastPubPosL = 0.0, self.lastPubPosR = 0.0;
        self.nodeStartFlag = False       
        self.last_read = time.time()
        self.pos_L = 0.0
        self.pos_R = 0.0
        self.MAX_STEER_DEG = msa
        
    # Calculate steering from difference of left and right wheel PID outputs
    def set_speed_steer(self, spd_lft, spd_rht):
        self.avspeed = (spd_lft + spd_rht)/2.0;
        self.steer = (spd_lft - self.avspeed)*2.0;
        max_steer = np.deg2rad(self.MAX_STEER_DEG)                                # max steering angle
        self.steer = np.clip(self.steer, -max_steer, max_steer);
        self.hover.set_steer_speed(self.avspeed, self.steer)  
        
    def get_data(self):
        self.upd = False   
        ord_data, char_data = self.hover.read_data()	
        print(char_data)
        if not self.hover.parse_data(ord_data) == -1:
            self.r_speed = self.dir_correction * (abs(self.hover.speedR_meas) * 0.10472);
            self.l_speed = self.dir_correction * (abs(self.hover.speedL_meas) * 0.10472);
            self.b_volts = self.hover.batVoltage/100.0;
            self.b_temp = self.hover.boardTemp/10.0;
            self.on_encoder_update(self.hover.wheelR_cnt, self.hover.wheelL_cnt)
            print("wheel positions left : ", self.pos_L, "right : ", self.pos_R)
            self.upd = True       
            self.last_read = time.time()
        
    def close_comm(self):
        self.hover.close_port()	
        self.hover = None
        
    def on_encoder_update(self, right, left):
        posL = 0.0, posR = 0.0;

        # Calculate wheel position in ticks, factoring in encoder wraps
        if (right < self.low_wrap) and (self.last_wheelcountR > self.high_wrap):
            self.multR =+ 1;
        elif (right > self.high_wrap) and (self.last_wheelcountR < self.low_wrap):
            self.multR =- 1;
        posR = right + self.multR*(self.ENCODER_MAX-self.ENCODER_MIN);
        self.last_wheelcountR = right;

        if (left < self.low_wrap) and (self.last_wheelcountL > self.high_wrap):
            self.multL =+ 1;
        elif (left > self.high_wrap) and (self.last_wheelcountL < self.low_wrap):
            self.multL =- 1;
        posL = left + self.multL*(self.ENCODER_MAX-self.ENCODER_MIN);
        self.last_wheelcountL = left;

        # When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
        # This section accumulates ticks even if board shuts down and is restarted   
        self.lastPosL = 0.0, self.lastPosR = 0.0;
        self.lastPubPosL = 0.0, self.lastPubPosR = 0.0;
        self.nodeStartFlag = True;
    
        # IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
        # (the board seems to often report 1-3 ticks on startup instead of zero)
        # reset the last read ticks to the startup values
        if((time.time() - self.last_read) > 0.2) and ((abs(posL) < 5) and (abs(posR) < 5)) :
            self.lastPosL = posL;
            self.lastPosR = posR;
        posLDiff = 0;
        posRDiff = 0;

        #if node is just starting keep odom at zeros
	    if(self.nodeStartFlag):
		    self.nodeStartFlag = False;
	    else:
            posLDiff = posL - self.lastPosL;
            posRDiff = posR - self.lastPosR;

        self.lastPubPosL += posLDiff;
        self.lastPubPosR += posRDiff;
        self.lastPosL = posL;
        self.lastPosR = posR;
    
        # Convert position in accumulated ticks to position in radians
        self.pos_L = 2.0*np.pi * self.lastPubPosL/self.TICKS_PER_ROTATION;
        self.pos_R = 2.0*np.pi * self.lastPubPosR/self.TICKS_PER_ROTATION;

# create a speed controller for each wheel and send to the hoverboard controller
#
def two_wheel_traction(wr, mv, target):
    wheel_rad = wr;
    max_vel = mv;
    setp = target;
    # create PID controller for each wheel
    left_wheel = hoverboard_pid(wheel_rad, max_vel, setp)
    right_wheel = hoverboard_pid(wheel_rad, max_vel, setp)
    f = 1.0
    p = 0.0
    i = 0.0
    d = 0.01
    i_max = 1.5
    i_min = -1.5
    left_wheel.init(self, f, p, i, d, i_max, i_min, antiwindup=False)   
    right_wheel.init(self, f, p, i, d, i_max, i_min, antiwindup=False)  
    # create object to send data to hoverboard 
    hb = Hover_Comms() 
    time_ref = time.time()
    hb.set_speed_steer(0,0)  
    # now control using the 2 PID loops and feedback from the hoverboard
    while True:
        hb.get_data()    
        if hb.upd == True:
            time_ref1 = time.time()
            dt = time_ref1 - time_ref
            measured_value = hb.l_speed;
            left_wheel.update(measured_value, setpoint, dt) 
            measured_value = hb.r_speed;
            right_wheel.update(measured_value, setpoint, dt) 
            hb.set_speed_steer(left_wheel.speed, right_wheel.speed)   
            time_ref = time_ref1
            time.sleep(0.1)

LIB_TEST_ON = True	
if __name__ == "__main__":
    if LIB_TEST_ON == True:
        wr = 10.0
        mv = 100.0
        target = 50.0
        two_wheel_traction(wr, mv, target)           
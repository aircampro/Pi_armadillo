#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# wrapper to keigan motor api for 2 wheel drive car and motor commands
#
# https://github.com/keigan-motor/pykeigan_motor/blob/master/pykeigan/controller.py
# https://github.com/keigan-motor/pykeigan_simple_agv/blob/master/twd.py
# pip install pykeigan-moto
#
from pykeigan import usbcontroller
from pykeigan import blecontroller
from pykeigan import utils
import time
import math
import threading
from enum import Enum

"""
Specify the device address (port) using a unique ID.
----------------------
Connecting to a Motor
----------------------
Regarding Motor Device File Specification
The device name displayed by "/dev/ttyUSB0" may change if multiple motors are connected.
If you have multiple motors and want to connect to a specific motor, use the device displayed by "$ls /dev/serial/by-id/".

e.g.) /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DM00LSSA-if00-port0
"""

# curves
CURVE_TYPE_NONE = 0,                                                # Turn off Motion control
CURVE_TYPE_TRAPEZOID = 1,                                           # Turn on Motion control with trapezoidal curve
CURVE_TYPE_DIRECT_POS = 10                                          # Turn off Motion control (Direct position control)
        
# motor meas int
M2 = 0          #2 [ms]
M5 = 1          #5 [ms]
M10 = 2         #10 [ms]
M20 = 3         #20 [ms]
M50 = 4         #50 [ms]
M100 = 5        #100 [ms]
M200 = 6        #200 [ms]
M500 = 7        #500 [ms]
M1000 = 8       #1000 [ms]
M2000 = 9       #2000 [ms]
M5000 = 10      #5000 [ms]
M10000 = 11     #10000 [ms]
     
# interface_type
BLE = 0b1
USB = 0b1000
I2C = 0b10000
BTN = 0b10000000

# safe run action (comms fail default action)
SACT_FREE_MOTOR = 0 # free_motor
SACT_DIS_MOTOR = 1  # disable_motor_action
SACT_STP_MOTOR = 2  # stop_motor
SACT_FIX_MOTOR = 3  # Fix to the current position (move_to)

# autostart
NO_AS = 0     # Without auto start 
NO_TS_AS = 1  # Auto start doing taskset (config by set_taskset_trigger_settings)
AS_PLAY = 2   # Auto start playback motion (config by set_motion_trigger_settings)

# led state (stat)
LED_STATE_OFF = 0      # LED off
LED_STATE_ON_SOLID = 1 # LED solid
LED_STATE_ON_FLASH = 2 # LED flash
LED_STATE_ON_DIM = 3   # LED dim

# PID limit
DELTA_MAX = 25
# PID controller gain values: No load
steer_p = 0.03 # 0.05 Proportional
steer_i = 0.05 # 0.002 Integral
steer_d = 0 # Derivative
# PID controller gain values: With load
steer_load_p = 0.80 # Proportional
steer_load_i = 0.5 # Integral
steer_load_d = 0 # Derivative
eI = 0 # Save the previous integral value of the error
x = 0 # Line position
x_old = 0 # Save the previous line position
CHARGING_TIME_SEC = 10 # Waiting time at the charging station
# Change the PID controller gain depending on whether there is a load (default is unused)
hasPayload = False # With load: True, Without load: False
RUN_CMD_INTERVAL = 0.05 # Run every 0.1 seconds
RUN_BASE_RPM = 50
RUN_LOWER_RPM = 15
STOP_AFTER_RPM = 10
STOP_AFTER_RPM1 = 5

# Calculate the rotation speed (rpm) given to the left and right motors from the line position
def pid_controller(sp=steer_p, si=steer_i, slp=steer_load_p, sli=steer_load_i):

    global steer_p, steer_i, steer_d
    global steer_load_p, steer_load_i, steer_load_d
    global eI, x, x_old 

    steer_p = sp
    steer_i = si
    steer_load_p = slp
    steer_load_i = sli

    # Change PID gain depending on load
    if hasPayload:
        gain_p = steer_load_p
        gain_i = steer_load_i
        gain_d = steer_load_d
    else:
        gain_p = steer_p
        gain_i = steer_i
        gain_d = steer_d        

    eI = eI + RUN_CMD_INTERVAL * x                                        
    eD = (x - x_old) / RUN_CMD_INTERVAL                                    
    delta_v = gain_p * x + gain_i * eI + gain_d * eD
    
    # Anti-windup
    if delta_v > DELTA_MAX:
        eI -= (delta_v - DELTA_MAX) / gain_i
        if eI < 0: eI = 0
        delta_v = DELTA_MAX
    elif delta_v < - DELTA_MAX:
        eI -= (delta_v + DELTA_MAX) / gain_i
        if eI > 0:
            eI = 0
        delta_v = - DELTA_MAX
    
    x_old = x
    rpm = (run_rpm + delta_v, run_rpm - delta_v)
    #print("x =", x, ", rpm =", rpm)
    return rpm

def reset_pid_params():
    eI = 0
    x = 0
    x_old = 0 

class State(Enum):
    """Enumerators representing system states

    Manages state transitions

    Attributes:
    State (Enum): System state

    """
    STATE_IDLE = 0 
    STATE_RUN = 1 
    
# Two Wheel Drive Car
class TWD():
    # port_left, port_right 
    def __init__(self, port_left, port_right, mode="usb", safe_time = 1, safe_option = 1, wheel_d = 100, tread = 400, button_event_cb = None):
        if mode == "usb":
            self.left = usbcontroller.USBController(port_left,False)
            self.right = usbcontroller.USBController(port_right,False)
        else:
            self.left = blecontroller.BLEController(port_left,False)
            self.right = blecontroller.BLEController(port_right,False)        
        self.left.enable_check_sum(True)
        self.right.enable_check_sum(True)
        self.safe_time = safe_time
        self.safe_option = safe_option
        self.curve(0)
        self.safe_setting(True)
        self.round_num = tread / wheel_d                             # Number of revolutions required to rotate 360 ​​degrees
        self.button_setting(button_event_cb)
        self.cur_state = 0
        self.do_pid_reset = False
        self.isDocking = False

    def enable(self):                                                # Motor operation permission
        self.safe_setting(True)                                      # Safety equipment started
        self.left.enable_action()
        self.right.enable_action()

    def disable(self):                                              
        self.left.disable_action()
        self.right.disable_action()

    def button_setting(self, button_event_cb):                       # KeiganMotor Enable the button
        self.left.set_button_setting(30)
        self.right.set_button_setting(30)
        self.left.on_motor_event_cb = button_event_cb
        self.right.on_motor_event_cb = button_event_cb

    def led(self, state, r, g, b):                                    
        self.left.set_led(state, r, g, b)
        self.right.set_led(state, r, g, b)

    def run(self, left_rpm, right_rpm):                               
        self.left.run_at_velocity(utils.rpm2rad_per_sec(left_rpm))
        self.right.run_at_velocity(utils.rpm2rad_per_sec(-right_rpm))
    
    def move_straight(self, rpm, degree, timeout = 0):                # Rotate the left and right motors by the angle specified in degrees to move forward. If the timeout [s] is exceeded, the safety device will resume.
        self.safe_setting(False)                                      
        self.left.set_speed(utils.rpm2rad_per_sec(rpm))
        self.right.set_speed(utils.rpm2rad_per_sec(rpm))        
        self.left.move_by_dist(utils.deg2rad(degree))
        self.right.move_by_dist(utils.deg2rad(-degree))
        # timeout == 0 In this case, the safety device remains disengaged and continues until the operation is complete.
        if timeout > 0:
            time.sleep(timeout)
            self.safe_setting(True)                                    

    def pivot_turn(self, rpm, degree, timeout = 0):
        self.safe_setting(False)                                      
        self.left.set_speed(utils.rpm2rad_per_sec(rpm))
        self.right.set_speed(utils.rpm2rad_per_sec(rpm))
        dist =  2 * math.pi * (self.round_num * degree / 360)          # 2π * (distance of one rotation as seen from directly above * angle/360)         
        self.left.move_by_dist(-dist)
        self.right.move_by_dist(-dist)
        if timeout > 0:
            time.sleep(timeout)
            self.safe_setting(True)    

    def stop(self, timeout = 0):                      
        self.safe_setting(False)                          
        self.left.stop_motor()
        self.right.stop_motor()
        if timeout > 0:
            time.sleep(timeout)    
            self.safe_setting(True)

    def free(self, timeout = 0):                           
        self.safe_setting(False)                         
        self.left.free_motor()
        self.right.free_motor()      
        if timeout > 0: 
            time.sleep(timeout)    
            self.safe_setting(True)                        

    def curve(self, type): 
        self.left.set_curve_type(type)
        self.right.set_curve_type(type)

    def safe_setting(self, isEnabled): 
        if isEnabled:
            self.left.set_safe_run_settings(True, self.safe_time * 1000, self.safe_option) 
            self.right.set_safe_run_settings(True, self.safe_time * 1000, self.safe_option) 
        else:
            self.left.set_safe_run_settings(False, self.safe_time * 1000, self.safe_option)
            self.right.set_safe_run_settings(False, self.safe_time * 1000, self.safe_option)

    def pid_stop(self):
        self.cur_state = State.STATE_IDLE

    def pid_run(self):
        self.cur_state = State.STATE_IDLE

    def pid_reset(self):
        self.do_pid_reset = True

    def pid_dock(self):
        self.isDocking = True

    def pid_undock(self):
        self.isDocking = False

    def pid_scheduler(self):
        global eI, x, x_old 
        if self.cur_state == State.STATE_IDLE:
            return
        # run this constantly every RUN_CMD_INTERVAL 
        t = threading.Timer(RUN_CMD_INTERVAL, self.pid_scheduler)
        t.start()
        if self.do_pid_reset == True:                           
            reset_pid_params()
            self.do_pid_reset = False
            return
        rpm = pid_controller()                                     
        if self.isDocking:
            rpm = (rpm[0] * 0.5, rpm[1] * 0.5)                     # Half speed when docked
    
        if self.cur_state == State.STATE_RUN:
            self.run(rpm[0], rpm[1])                               # Speed ​​command
        
    def disconnectUSB(self):
        self.left.disconnect()
        self.right.disconnect()    

    def reconnectUSB(self):
        self.left.reconnect()
        self.right.reconnect()  

    def startDBG(self):
        self.left.start_debug()  
        self.right.start_debug()     

    def endDBG(self):
        self.left.finish_debug()  
        self.right.finish_debug() 

    def set_max_speed(self, rpm):        
        self.left.set_max_speed(utils.rpm2rad_per_sec(rpm))  
        self.right.set_max_speed(utils.rpm2rad_per_sec(rpm))

    def set_min_speed(self, rpm):        
        self.left.set_min_speed(utils.rpm2rad_per_sec(rpm))  
        self.right.set_min_speed(utils.rpm2rad_per_sec(rpm))

    def set_max_torque(self, t):        
        self.left.set_max_torque(t)  
        self.right.set_max_torque(t)

    def set_acc(self, a):        
        self.left.set_acc(a)  
        self.right.set_acc(a)

    def set_dec(self, a):        
        self.left.set_dec(a)  
        self.right.set_dec(a)

    def set_teaching_interval(self, a):        
        self.left.set_teaching_interval(a)  
        self.right.set_teaching_interval(a)

    def set_playback_interval(self, a):        
        self.left.set_playback_interval(a)  
        self.right.set_playback_interval(a)

    def set_qcurrent_p(self, a):        
        self.left.set_qcurrent_p(a)  
        self.right.set_qcurrent_p(a)

    def set_qcurrent_i(self, a):        
        self.left.set_qcurrent_i(a)  
        self.right.set_qcurrent_i(a)

    def set_qcurrent_d(self, a):        
        self.left.set_qcurrent_d(a)  
        self.right.set_qcurrent_d(a)

    def set_speed_p(self, a):        
        self.left.set_speed_p(a)  
        self.right.set_speed_p(a)

    def set_speed_i(self, a):        
        self.left.set_speed_i(a)  
        self.right.set_speed_i(a)

    def set_speed_d(self, a):        
        self.left.set_speed_d(a)  
        self.right.set_speed_d(a)

    def set_position_p(self, a):        
        self.left.set_position_p(a)  
        self.right.set_position_p(a)

    def set_position_i(self, a):        
        self.left.set_position_i(a)  
        self.right.set_position_i(a)

    def set_position_d(self, a):        
        self.left.set_position_d(a)  
        self.right.set_position_d(a)

    def set_pos_control_threshold(self, a):        
        self.left.set_pos_control_threshold(a)  
        self.right.set_pos_control_threshold(a)

    def reset_all_pid(self, a):        
        self.left.reset_all_pid(a)  
        self.right.reset_all_pid(a)

    def set_motor_measurement_interval(self, a):        
        self.left.set_motor_measurement_interval(a)  
        self.right.set_motor_measurement_interval(a)

    def set_motor_measurement_settings(self, a="on"):  
        if a == "on":    
            self.left.set_motor_measurement_settings(0x3)  
            self.right.set_motor_measurement_settings(0x3)
        else:
            self.left.set_motor_measurement_settings(0x0)  
            self.right.set_motor_measurement_settings(0x0)        

    def set_limit_current(self, a):        
        self.left.set_limit_current(a)  
        self.right.set_limit_current(a)

    def set_interface(self, a):        
        self.left.set_interface(a)  
        self.right.set_interface(a)

    def set_safe_run_settings(self, a, t, sact):        
        self.left.set_safe_run_settings(a, t, sact)  
        self.right.set_safe_run_settings(a, t, sact)

    def set_own_color(self, r, g, b):        
        self.left.set_own_color(r%256, g%256, b%256)  
        self.right.set_own_color(r%256, g%256, b%256)

    def set_imu_measurement_interval(self, a):        
        self.left.set_imu_measurement_interval(a)  
        self.right.set_imu_measurement_interval(a)

    def set_imu_measurement_settings(self, a="on"):  
        if a == "on":    
            self.left.set_imu_measurement_settings(0x3)  
            self.right.set_imu_measurement_settings(0x3)
        else:
            self.left.set_imu_measurement_settings(0x0)  
            self.right.set_imu_measurement_settings(0x0) 

    def set_autostart_setting(self, a):        
        self.left.set_autostart_setting(a)  
        self.right.set_autostart_setting(a)

    def set_button_setting(self, a):        
        self.left.set_button_setting(a)  
        self.right.set_button_setting(a)

    def set_notify_pos_arrival_settings(self, a=True, t, s):        
        self.left.set_notify_pos_arrival_settings(a, t, s)  
        self.right.set_notify_pos_arrival_settings(a, t, s)

    def save_to_flash(self):        
        self.left.save_all_registers()  
        self.right.save_all_registers()

    def reset_all_registers(self):        
        self.left.reset_all_registers()  
        self.right.reset_all_registers()

    def disable_action(self):        
        self.left.disable_action()  
        self.right.disable_action()

    def enable_action(self):        
        self.left.enable_action()  
        self.right.enable_action()

    def preset_position(self, a):        
        self.left.preset_position(a)  
        self.right.preset_position(a)

    def run_forward(self):        
        self.left.run_forward()  
        self.right.run_forward()

    def run_reverse(self):        
        self.left.run_reverse()  
        self.right.run_reverse()

    def run_at_velocity(self, a):        
        self.left.run_at_velocity(a)  
        self.right.run_at_velocity(a)

    def move_to_pos(self, p, s):        
        self.left.move_to_pos(p, s)  
        self.right.move_to_pos(p, s)

    def move_to_pos_wait(self, p, s):        
        self.left.move_to_pos_wait(p, s)  
        self.right.move_to_pos_wait(p, s)
        
    def move_by_dist(self, d, s):        
        self.left.move_by_dist(d, s)  
        self.right.move_by_dist(d, s)

    def move_by_dist_wait(self, d, s):        
        self.left.move_by_dist_wait(d, s)  
        self.right.move_by_dist_wait(d, s)

    def disable_action_task(self):        
        self.left.disable_action_task()  
        self.right.disable_action_task()

    def enable_action_task(self):        
        self.left.enable_action_task()  
        self.right.enable_action_task()

    def free_motor(self):        
        self.left.free_motor()  
        self.right.free_motor()

    def stop_motor(self):        
        self.left.stop_motor()  
        self.right.stop_motor()

    def hold_torque(self):        
        self.left.hold_torque()  
        self.right.hold_torque()

    def stop_doing_taskset(self):        
        self.left.stop_doing_taskset()  
        self.right.stop_doing_taskset()

    def start_doing_taskset(self, index):        
        self.left.start_doing_taskset(index)  
        self.right.start_doing_taskset(index)

    def start_playback_motion(self, index, r, n):        
        self.left.start_doing_taskset(index, r, n)  
        self.right.start_doing_taskset(index, r, n)

    def prepare_playback_motion(self, index, r, n):        
        self.left.prepare_playback_motion(index, r, n)  
        self.right.prepare_playback_motion(index, r, n)

    def start_playback_motion_from_prep(self):        
        self.left.start_playback_motion_from_prep()  
        self.right.start_playback_motion_from_prep()

    def stop_playback_motion(self):        
        self.left.stop_playback_motion()  
        self.right.stop_playback_motion()

    def start_recording_taskset(self, index):        
        self.left.start_recording_taskset(index)  
        self.right.start_recording_taskset(index)

    def stop_recording_taskset(self, index):        
        self.left.stop_recording_taskset(index)  
        self.right.stop_recording_taskset(index)

    def erase_taskset(self, index):        
        self.left.erase_taskset(index)  
        self.right.erase_taskset(index)

    def erase_all_tasksets(self):        
        self.left.erase_all_tasksets()  
        self.right.erase_all_tasksets()

    def set_trigger_taskset_settings(self, index, rept):        
        self.left.set_trigger_taskset_settings(index, rept)  
        self.right.set_trigger_taskset_settings(index, rept)

    def start_teaching_motion(self, index, t_ms):        
        self.left.start_teaching_motion(index, t_ms)  
        self.right.start_teaching_motion(index, t_ms)

    def prepare_teaching_motion(self, index, t_ms):        
        self.left.prepare_teaching_motion(index, t_ms)  
        self.right.prepare_teaching_motion(index, t_ms)

    def start_teaching_motion_from_prep(self):        
        self.left.start_teaching_motion_from_prep()  
        self.right.start_teaching_motion_from_prep()

    def stop_teaching_motion(self):        
        self.left.stop_teaching_motion()  
        self.right.stop_teaching_motion()

    def erase_motion(self, idx):        
        self.left.erase_motion(idx)  
        self.right.erase_motion(idx)

    def erase_all_motions(self):        
        self.left.erase_all_motions()  
        self.right.erase_all_motions()

    def read_gpio_status(self):        
        a= self.left.read_gpio_status()  
        b= self.right.read_gpio_status()
        return a, b

    def reboot(self):        
        self.left.reboot()  
        self.right.reboot()

    def set_led(self, stat, r, g, b):        
        self.left.set_led(stat, r, g, b)  
        self.right.set_led(stat, r, g, b)

    def set_right_led(self, stat, r, g, b):         
        self.right.set_led(stat, r, g, b)

    def set_left_led(self, stat, r, g, b):        
        self.left.set_led(stat, r, g, b)  

    def turn_right( self ):
        self.enable()                                         # It may be disabled due to line loss
        self.free(0.1)                                        # Stop, timeout 0.5 seconds
        self.move_straight(20, 380, 4)                        # Go straight.
        self.pivot_turn(20, -90, 3)                           # Rotate 90°
        self.stop(0.1) 

    def turn_left( self ):
        self.enable()                                         
        self.free(0.1)                                       
        self.move_straight(20, 380, 4)                      
        self.pivot_turn(20, 90, 3)                          
        self.stop(0.1) 

    def turn_round(self):
        self.pivot_turn(20, 180, 10)

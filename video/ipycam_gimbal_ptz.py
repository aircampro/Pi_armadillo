#!/usr/bin/env python3
"""
Hardware PTZ Example

Demonstrates how to attach external hardware controllers (motors, servos, 
gimbals, etc.) to receive PTZ commands from ONVIF clients.

The PTZHardwareHandler protocol defines callbacks for all PTZ operations.
Implement the methods you need and register your handler with the PTZ controller.

example from https://github.com/olkham/IPyCam/blob/main/examples/hardware_ptz.py

added dynamixel motor driver control for PT Servo
added stepper using C28BYJ48 and NEMA17_L298N motor driver control for PT

"""
import time
import cv2
from typing import Optional
from ipycam import IPCamera, CameraConfig, PTZHardwareHandler
import dynamix_driver                                                                                                       # driver included in this folder

# uses this stepper motor https://rajguruelectronics.com/Product/1467/28BYJ-48%20-%205V%20Stepper%20Motor.pdf
# im using 2 different libs for GPIO for each stepper motor to show there example (you can just choose 1 if you like)
from gpiozero import OutputDevice as stepper
import datetime
import sys
import pickle
import threading
import ctypes

class C28BYJ48:

    def __init__(self, IN1, IN2, IN3, IN4):
        self.IN1 = stepper(IN1)
        self.IN2 = stepper(IN2)
        self.IN3 = stepper(IN3)
        self.IN4 = stepper(IN4)
        # Setting GPIO
        self.stepPins = [self.IN1, self.IN2, self.IN3, self.IN4]
        self.sequenceNumber = 0
        self.stepCount = 8

    def seq(self):
        return [[1,0,0,1], [1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],[0,0,1,0],[0,0,1,1],[0,0,0,1]]

    def SetRoll(self, angle, wait):
        if angle <= 0 : return 0,0
        steps = int((angle * 2) * 4096 / 360)
        interval = float(wait * 60.0 / steps)
        return steps, interval

    def SetPinVoltage(self, NSeq):
        self.sequenceNumber = NSeq
        for pin in range(0,4):
            xPin = self.stepPins[pin]
            seq = self.seq()
            if seq[self.sequenceNumber][pin]!=0:
                xPin.on()
            else:
                xPin.off()

    def UnSetPinVoltage(self):
        for pin in range(0,4):
            xPin = self.stepPins[pin]
            xPin.off()

    def run(self, angle, wait, direction):
        # Set to 1 for clockwise
        # Set to -1 for anti-clockwise
        self.direction = direction
        steps, interval = self.SetRoll(angle, wait)
        for i in range(0, steps):
            self.SetPinVoltage(self.sequenceNumber)
            self.sequenceNumber += self.direction
            if self.sequenceNumber >= self.stepCount:
                self.sequenceNumber = 0
            if self.sequenceNumber < 0:
                self.sequenceNumber = self.stepCount + self.direction
            time.sleep(interval)

    def do_steps(self, steps, interval):
        for i in range(0, steps):
            self.SetPinVoltage(self.sequenceNumber)
            self.sequenceNumber += self.direction
            if self.sequenceNumber >= self.stepCount:
                self.sequenceNumber = 0
            if self.sequenceNumber < 0:
                self.sequenceNumber = self.stepCount + self.direction
            time.sleep(interval)

# uses NEMA 17 Stepper Motor (on Amazon)
# L298N Dual H-Bridge DC Stepper Motor Drive ref:- https://qiita.com/zoo_dj/items/0c7f632967e266bac64a
# sudo pip install rpimotorlib
#
import time 
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib 
import numpy as np

class NEMA17_L298N:

    def __init__(self, GpioPins = [17, 18, 27, 22]):
        self.GpioPins = GpioPins
        self.mtr = RpiMotorLib.BYJMotor('MyMotorTilt', 'Nema')
        time.sleep(0.002)		
    # cycle 180 
    # T Period [s]
    # N_period No. of periods
    def do_180_cycle(self, T = 2, N_period = 2):
        amp = 72
        unit_angle = 7.2
        s_angle = 1.8                                     # [deg] 180
        N = int(4*amp/unit_angle)
        theta = amp*np.sin(np.linspace(0, 2*np.pi, N))    # Angle time history [deg]
        dt = T/len(theta)                                 # time-step [s]
        omega = np.gradient(theta, dt)/360                # [Hz]
        omega_pos = abs(omega)
        pm = omega < 0                                    # boolean for CW/CCW 
        wait = (1/omega_pos)*(s_angle/360)/2
        for _ in np.arange(N_period):
            for k in np.arange(N):
                self.mtr.motor_run(GpioPins, wait[k], 1, pm[k], False, "half", 0)
    def do_angle(self, a, r=0, T = 2, N_period = 1):
        amp = 72
        unit_angle = 7.2
        s_angle = a/100                                   # [deg] 180
        N = int(4*amp/unit_angle)
        theta = amp*np.sin(np.linspace(0, 2*np.pi, N))    # Angle time history [deg]
        dt = T/len(theta)                                 # time-step [s]
        omega = np.gradient(theta, dt)/360                # [Hz]
        omega_pos = abs(omega)
        pm = omega < 0                                    # boolean for CW/CCW 
        wait = (1/omega_pos)*(s_angle/360)/2
        for _ in np.arange(N_period):
            for k in np.arange(N):
                if r == 1:
                    self.mtr.motor_run(GpioPins, wait[k], 1, pm[k], True, "half", 0)                # reverse dir
                else:
                    self.mtr.motor_run(GpioPins, wait[k], 1, pm[k], False, "half", 0)               # forward dir
    def stop(self):
        for pin in self.GpioPins:
            GPIO.output(pin, 0)
    def __del__(self):
        GPIO.cleanup()
 
class PrintingPTZHandler:
    """
    Example hardware handler that prints all PTZ commands.
    Replace this with your actual hardware control code.
    """
    def on_continuous_move(self, pan_speed: float, tilt_speed: float, zoom_speed: float) -> None:
        """Called when continuous movement starts"""
        print(f"[HW] Continuous move: pan={pan_speed:+.2f}, tilt={tilt_speed:+.2f}, zoom={zoom_speed:+.2f}")

    def on_stop(self) -> None:
        """Called when movement should stop"""
        print("[HW] Stop all movement")

    def on_absolute_move(self, pan: Optional[float], tilt: Optional[float], zoom: Optional[float]) -> None:
        """Called for absolute positioning"""
        print(f"[HW] Absolute move: pan={pan}, tilt={tilt}, zoom={zoom}")

    def on_relative_move(self, pan_delta: float, tilt_delta: float, zoom_delta: float) -> None:
        """Called for relative movement"""
        print(f"[HW] Relative move: Δpan={pan_delta:+.2f}, Δtilt={tilt_delta:+.2f}, Δzoom={zoom_delta:+.2f}")

    def on_goto_preset(self, token: str, pan: float, tilt: float, zoom: float) -> None:
        """Called when moving to a preset"""
        print(f"[HW] Goto preset '{token}': pan={pan:.2f}, tilt={tilt:.2f}, zoom={zoom:.2f}")

    def on_goto_home(self) -> None:
        """Called when returning to home position"""
        print("[HW] Goto home")

class ServoExample:
    """
    Example showing how you might control servos.
    In this example i have added the dynamixel serial servo driver 
    """
    def __init__(self, port="/dev/ttyS0", baudrate=9600, pan_id=1, tilt_id=2, pa=90, ta=90, acc=None, pidparam=None):
        # Example: Using dynamixel motors
        self.serial_driver = dynamix_driver.DynamixelIO(port, baudrate)      # connect to dynamixel drives via serial connection
        self.pan_servo = pan_id
        self.serial_driver.set_drive_mode(self.pan_servo)                    # Sets the drive mode for EX-106 motors to forward
        self.serial_driver.set_torque_enabled(self.pan_servo, 1)             # enable torgue
        self.tilt_servo = tilt_id
        self.serial_driver.set_drive_mode(self.tilt_servo)                   # Sets the drive mode for EX-106 motors to forward
        self.serial_driver.set_torque_enabled(self.tilt_servo, 1)            # enable torgue
        self.pan_angle = pa                                                  # 0-180 degrees
        self.tilt_angle = ta
        self.pan_counts = 0                                                  # counts for DAC
        self.tilt_counts = 0                                                 # counts for DAC
        self.reslps = self.serial_driver.get_encoder_resolution(self.pan_servo)         # get reso;ution for the drive type
        self.reslts = self.serial_driver.get_encoder_resolution(self.tilt_servo)        # get reso;ution for the drive type
        self.pan_rc = self.serial_driver.get_max_raw(self.pan_servo)                    # get max raw value for position for the drive type
        self.tilt_rc = self.serial_driver.get_max_raw(self.tilt_servo)    
        self.run = False                                                     # wait until continous move active
        if acc is not None:                                                  # set to non-default
            self.serial_driver.set_acceleration(self.pan_servo, acc)         # change acceleration of motors 
            self.serial_driver.set_acceleration(self.tilt_servo, acc)    
        if pidparam is not None:                                             # set to non-default (tuple or list) 
            self.serial_driver.set_p_gain(self.pan_servo, pidparam[0])       # set pid controller parameters to non-default         
            self.serial_driver.set_i_gain(self.pan_servo, pidparam[1]) 
            self.serial_driver.set_d_gain(self.pan_servo, pidparam[2]) 
            self.serial_driver.set_p_gain(self.tilt_servo, pidparam[3])         
            self.serial_driver.set_i_gain(self.tilt_servo, pidparam[4]) 
            self.serial_driver.set_d_gain(self.tilt_servo, pidparam[5]) 
        self.db = 2
        self.on_absolute_move(self.pan_angle, self.tilt_angle)              # go to initial position
        print("[Servo] Initialized (simulation mode)")

    def on_continuous_move(self, pan_speed: float, tilt_speed: float, zoom_speed: float) -> None:
        # For continuous movement, you'd typically start a loop
        # that keeps adjusting position based on speed
        self.run = True
        self.serial_driver.set_torque_enabled(self.pan_servo, 1)              # torgue on   
        self.serial_driver.set_torque_enabled(self.tilt_servo, 1)             # torgue on 
        print(f"[Servo] Would move at speeds: pan={pan_speed}, tilt={tilt_speed}")
        while self.run == True:
            if pan_speed is not None:
                cur_pos = self.serial_driver.get_position(self.pan_servo)                                    # get cur pos  
                if cur_pos >= (self.pan_rc - self.db):                                                       # max position
                    self.serial_driver.set_drive_mode(self.pan_servo, is_reverse=True)                       # reverse
                elif cur_pos <= self.db:
                    self.serial_driver.set_drive_mode(self.pan_servo)                                        # forward      
            if tilt_speed is not None:
                cur_pos = self.serial_driver.get_position(self.tilt_servo)                                   # get cur pos  
                if cur_pos >= (self.tilt_rc - self.db):                                                      # max position
                    self.serial_driver.set_drive_mode(self.tilt_servo, is_reverse=True)                      # reverse
                elif cur_pos <= self.db:
                    self.serial_driver.set_drive_mode(self.tilt_servo)                                       # forward 
            if pan_speed is not None and tilt_speed is not None:
                self.serial_driver.set_multi_speed(((self.pan_servo, pan_speed), (self.tilt_servo, tilt_speed))) 
            elif pan_speed is not None:
                self.serial_driver.set_speed(self.pan_servo, pan_speed)             
            elif tilt_speed is not None:
                self.serial_driver.set_speed(self.tilt_servo, tilt_speed)
            time.sleep(0.1)
        self.serial_driver.set_torque_enabled(self.pan_servo, 0)              # torgue off   
        self.serial_driver.set_torque_enabled(self.tilt_servo, 0)             # torgue off  

    def on_stop(self) -> None:
        if self.run == True:
            self.run = False
        self.serial_driver.set_torque_enabled(self.pan_servo, 0)              # torgue off   
        self.serial_driver.set_torque_enabled(self.tilt_servo, 0)             # torgue off           
        print("[Servo] Motors stopped")

    def on_absolute_move(self, pan: Optional[float], tilt: Optional[float], zoom: Optional[float], speed: Optional[float]) -> None:
        self.serial_driver.set_torque_enabled(self.pan_servo, 1)                                         # torgue on   
        self.serial_driver.set_torque_enabled(self.tilt_servo, 1)                                        # torgue on 
        if pan is not None:
            self.pan_angle = int((pan + 1) * 90)                                                         # Convert normalized -1..1 to servo angle 0..180
            cur_pos = self.serial_driver.get_position(self.pan_servo)                                    # get cur pos
            act_angle = (self.reslps * cur_pos)                                                          # resl * raw counts = angle
            if act_angle < self.pan_angle:
                self.serial_driver.set_drive_mode(self.pan_servo)                                        # forward  
            else:
                self.serial_driver.set_drive_mode(self.pan_servo, is_reverse=True)                       # reverse
            self.pan_counts = int(self.reslps * self.pan_angle)
            if speed is not None:
                self.serial_driver.set_position_and_speed(self.pan_servo, self.pan_counts, speed)
            else:
                self.serial_driver.set_position(self.pan_servo, self.pan_counts)            
            print(f"[Servo] Pan servo -> {self.pan_angle}°")
            # self.pan_servo.angle = self.pan_angle

        if tilt is not None:
            self.tilt_angle = int((tilt + 1) * 90)
            cur_pos = self.serial_driver.get_position(self.tilt_servo)                                    # get cur pos
            act_angle = (self.reslts * cur_pos)                                                           # resl * raw counts = angle
            if act_angle < self.tilt_angle:
                self.serial_driver.set_drive_mode(self.tilt_servo)                                        # forward  
            else:
                self.serial_driver.set_drive_mode(self.tilt_servo, is_reverse=True)                       # reverse
            self.tilt_counts = int(self.reslts * self.tilt_angle)
            if speed is not None:
                self.serial_driver.set_position_and_speed(self.tilt_servo, self.tilt_counts, speed)
            else:
                self.serial_driver.set_position(self.tilt_servo, self.tilt_counts)  
            print(f"[Servo] Tilt servo -> {self.tilt_angle}°")
            # self.tilt_servo.angle = self.tilt_angle 

    def on_relative_move(self, pan_delta: float, tilt_delta: float, zoom_delta: float, speed: Optional[float]) -> None:
        self.serial_driver.set_torque_enabled(self.pan_servo, 1)                                         # torgue on   
        self.serial_driver.set_torque_enabled(self.tilt_servo, 1)                                        # torgue on 
        if pan_delta != 0:
            if pan_delta > 0:
                self.serial_driver.set_drive_mode(self.pan_servo)                                        # forward  
            else:
                self.serial_driver.set_drive_mode(self.pan_servo, is_reverse=True)                       # reverse
            self.pan_angle = max(0, min(180, self.pan_angle + int(pan_delta * 90)))                      # Adjust by delta (convert to degrees: delta * 90)
            self.pan_counts = int(self.reslps * self.pan_angle)
            if speed is not None:
                self.serial_driver.set_position_and_speed(self.pan_servo, self.pan_counts, speed)
            else:
                self.serial_driver.set_position(self.pan_servo, self.pan_counts)  
            print(f"[Servo] Pan servo adjusted to {self.pan_angle}°")

        if tilt_delta != 0:
            if tilt_delta > 0:
                self.serial_driver.set_drive_mode(self.tilt_servo)   
            else:
                self.serial_driver.set_drive_mode(self.tilt_servo, is_reverse=True) 
            self.tilt_angle = max(0, min(180, self.tilt_angle + int(tilt_delta * 90)))
            self.tilt_counts = int(self.reslts * self.tilt_angle)
            if speed is not None:
                self.serial_driver.set_position_and_speed(self.tilt_servo, self.tilt_counts, speed)
            else:
                self.serial_driver.set_position(self.tilt_servo, self.tilt_counts) 
            print(f"[Servo] Tilt servo adjusted to {self.tilt_angle}°") 

    def on_goto_preset(self, token: str, pan: float, tilt: float, zoom: float) -> None:
        self.on_absolute_move(pan, tilt, zoom)

    def on_goto_home(self) -> None:
        self.serial_driver.set_torque_enabled(self.pan_servo, 1)                                     # torgue on   
        self.serial_driver.set_torque_enabled(self.tilt_servo, 1)                                    # torgue on 
        self.pan_angle = 90
        self.tilt_angle = 90
        cur_pos = self.serial_driver.get_position(self.pan_servo)                                    # get cur pos
        act_angle = (self.reslps * cur_pos)                                                          # resl * raw counts = angle
        if act_angle < self.pan_angle:
            self.serial_driver.set_drive_mode(self.pan_servo)                                        # forward  
        else:
            self.serial_driver.set_drive_mode(self.pan_servo, is_reverse=True)                       # reverse
        cur_pos = self.serial_driver.get_position(self.tilt_servo)                                   # get cur pos
        act_angle = (self.reslts * cur_pos)                                                          # resl * raw counts = angle
        if act_angle < self.tilt_angle:
            self.serial_driver.set_drive_mode(self.tilt_servo)                                       # forward  
        else:
            self.serial_driver.set_drive_mode(self.tilt_servo, is_reverse=True)                      # reverse
        self.pan_counts = int(self.reslps * self.pan_angle)
        self.tilt_counts = int(self.reslts * self.tilt_angle)
        self.serial_driver.set_position(self.pan_servo, self.pan_counts)
        self.serial_driver.set_position(self.tilt_servo, self.tilt_counts)      
        cur_pos = self.serial_driver.get_position(self.pan_servo)                                    # get cur pos
        ps_angle = (self.reslps * cur_pos)                                                           # resl * raw counts = angle  
        cur_pos = self.serial_driver.get_position(self.tilt_servo)                                   # get cur pos
        ts_angle = (self.reslts * cur_pos)                                                           # resl * raw counts = angle
        while ((ts_angle < (self.tilt_angle - self.db)) or (ts_angle > (self.tilt_angle + self.db))) or ((ps_angle < (self.pan_angle - self.db)) or (ps_angle > (self.pan_angle + self.db))):
            cur_pos = self.serial_driver.get_position(self.pan_servo)                                # get cur pos
            ps_angle = (self.reslps * cur_pos)                                                       # resl * raw counts = angle  
            cur_pos = self.serial_driver.get_position(self.tilt_servo)                               # get cur pos
            ts_angle = (self.reslts * cur_pos)                                                       # resl * raw counts = angle 
            if ps_angle < self.pan_angle:
                self.serial_driver.set_drive_mode(self.pan_servo)                                    # forward  
            else:
                self.serial_driver.set_drive_mode(self.pan_servo, is_reverse=True)                   # reverse
            if ts_angle < self.tilt_angle:
                self.serial_driver.set_drive_mode(self.tilt_servo)                                   # forward  
            else:
                self.serial_driver.set_drive_mode(self.tilt_servo, is_reverse=True)                  # reverse
            time.sleep(0.1)            
        self.serial_driver.set_torque_enabled(self.pan_servo, 0)                                     # torgue off   
        self.serial_driver.set_torque_enabled(self.tilt_servo, 0)                                    # torgue off         
        print("[Servo] Servos centered (home position)")

# Class wrapper for threading 
# you can also kill task e.g. if op_task.is_alive(): op_task.raise_exception()
#
class twe(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return

    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')

# ============================== various gimbal control classes ===================================
#
#
import time
import sys
from cymbal.camera_gimbal import Storm32Controller

class STorM32:
    """
    Example for STorM32 gimbal 3-Axis controller.
    """
    def __init__(self, b=115200, p="/dev/ttyAMA0"):
        self.controller = Storm32Controller(port=p, baudrate=b)
        if not self.controller.connect():
            print("ERROR: Failed to connect to Storm32 gimbal")
            sys.exit(-1)

    def __del__(self):
        self.controller.disconnect()   
        
    def home(self):
        self.controller.center()  

    def go2angle(self, p=None, r=None, y=None):
        if p is not None:
            pitch = p
        else:
            pitch = 0
        if r is not None:
            roll = r
        else:
            roll = 0
        if y is not None:
            yaw = y
        else:
            yaw = 0            
        self.controller.set_angle(pitch, roll, yaw)    

    def get_status(self):
        return self.controller.get_status()

from cymbal.spotlight_gimbal import SpotlightController

class SpotLight:
    """
    Example for SpotLight gimbal 3-Axis controller.
    """
    def __init__(self, pp=17, yp=27):
        self.controller = SpotlightController( pitch_pin=pp, yaw_pin=yp, use_stabilization=True)
        if not self.controller.initialize():
            print("ERROR: Failed to connect to SpotLight gimbal")
            sys.exit(-1)

    def __del__(self):
        self.controller.close()   
        
    def home(self):
        self.controller.center()  

    def stop(self):
        self.controller.stop()  

    def go2pos(self, p=None, y=None):
        if p is not None:
            pitch = p
        else:
            pitch = 0
        if y is not None:
            yaw = y
        else:
            yaw = 0            
        self.controller.set_position(pitch, yaw)    
        for i in range(30):
            self.controller.stabilize()
            time.sleep(0.1)
 
    def set_speed(self, p=None, y=None):
        if p is not None:
            pitch = p
        else:
            pitch = 0
        if y is not None:
            yaw = y
        else:
            yaw = 0            
        self.controller.set_speed(pitch, yaw)

    def stabilize(self, r=100):
        for i in range(r):
            self.controller.stabilize()
            time.sleep(0.1)
            
    def get_orientation(self):
        return self.controller.get_orientation()

from pygimbal import control

class pyGimbal:
    """
    Example for pygimbal compatible 3-Axis controller.
    """
    def __init__(self, b=115200, p='/dev/ttyUSB0'):
        self.master_gimbal = control.init_mav_connector(p, 1, 154, b)
        self.system = master_gimbal.source_system
        self.component = master_gimbal.source_component
        self.maintain_connection_threading = twe(target=control.maintain_connection, args=[self.master_gimbal], daemon=True)
        self.maintain_connection_threading.start()
        self.orientation_theading = twe(target=control.get_orientation, args=[self.master_gimbal], daemon=True)
        self.orientation_theading.start()  
        
    def __del__(self):
        if self.maintain_connection_threading.is_alive(): self.maintain_connection_threading.raise_exception()  
        if self.orientation_theading.is_alive(): self.orientation_theading.raise_exception()
        self.maintain_connection_threading.join()
        self.orientation_theading.join()

    def home(self):
        control.control_gimbal(self.master_gimbal, self.system, self.component, tilt=90, roll=0, pan=90)  

    def go2angle(self,p=None,r=None,y=None):
        if p is not None:
            pitch = p
        else:
            pitch = 0
        if r is not None:
            roll = r
        else:
            roll = 0
        if y is not None:
            yaw = y
        else:
            yaw = 0            
        control.control_gimbal(self.master_gimbal, self.system, self.component, tilt=y, roll=r, pan=p) 

class GimbalExample:
    """
    Example for gimbal 3-Axis controller. choose from the classes above
    """
    def __init__(self):
        self.pan_position = 0                                          
        self.tilt_position = 0
        self.zoom_position = 0
        self.gimbal = STorM32()                                                                  # we chose the storm32 gimbal (select type from above)                   
        self.run = False

    def __del__(self):
        del STorM32                                                                               # clean-up

    def on_continuous_move(self, pan_dly: float, tilt_speed: float, zoom_speed: float) -> None:
        print(f"Unsupported for STorM32 [Gimbal] Running at speed: pan={pan_speed}, tilt={tilt_speed}")

    def on_stop(self) -> None:
        self.run = False
        print("[Gimbal] Motors stopped")

    def on_absolute_move(self, pan: Optional[float], tilt: Optional[float], zoom: Optional[float]) -> None:
        if pan is not None:
            self.pan_position = pan
        else:
            self.pan_position = 0        

        if tilt is not None:
            self.tilt_position = tilt
        else:
            self.tilt_position = 0

        if zoom is not None:
            self.zoom_position = zoom
        else:
            self.aoom_position = 0
        self.gimbal.go2angle(self.pan_position, self.tilt_position, self.zoom_position)
        print(f"[Gimbal] adjusted to {self.pan_position} {self.tilt_position} {self.zoom_position}") 

    def on_relative_move(self, pan_delta: float, tilt_delta: float, zoom_delta: float) -> None:
        if pan_delta != 0:
            delta_steps = int(pan_delta)
            self.pan_position += delta_steps

        if tilt_delta != 0:
            delta_steps = int(tilt_delta)
            self.tilt_position += delta_steps

        if zoom_delta != 0:
            delta_steps = int(zoom_delta)
            self.zoom_position += delta_steps
        self.gimbal.go2angle(self.pan_position, self.tilt_position, self.zoom_position)
        print(f"[Gimbal] adjusted to {self.pan_position} {self.tilt_position} {self.zoom_position}") 

    def on_goto_preset(self, token: str, pan: float, tilt: float, zoom: float) -> None:
        self.on_absolute_move(pan, tilt, zoom)

    def on_goto_home(self) -> None:
        print("[Gimbal] Homing motors...")
        self.gimbal.home()

def main():
    """Demo: Hardware PTZ (digital PTZ disabled)"""
    config = CameraConfig.load("camera_config.json")
    camera = IPCamera(config=config)

    # Disable digital PTZ when using hardware controllers
    # This prevents frame cropping and passes video through unchanged
    camera.ptz.enable_digital_ptz = True

    # Add hardware handlers - you can add multiple!
    # Commands will be sent to ALL registered handlers
    # Option 1: Simple printing handler for debugging
    #printer = PrintingPTZHandler()
    #camera.ptz.add_hardware_handler(printer)

    # Option 2: Add a servo controller (uncomment to use)
    #servo = ServoExample()
    #camera.ptz.add_hardware_handler(servo)

    # Option 4: Add a gimbal controller (uncomment to use)
    cFlag = 0                                                                          # set to 1 if you want to load saved state or 0 to initialise the gimbal 
    if cFlag == 1:
        print("\033[34m re-loading saved gimbal motor controller! \033[0m")
        try:
            with open('gimbal.pickle', 'rb') as f:
                gimbal = pickle.load(f)
        except:
            gimbal = GimbalExample() 
            print("\033[34m no saved file gimbal controller initialised! \033[0m")            
    else:
        gimbal = GimbalExample()
    camera.ptz.add_hardware_handler(gimbal)

    # Note: To enable BOTH hardware and digital PTZ simultaneously, set:
    # camera.ptz.enable_digital_ptz = True
    if not camera.start():
        print("Failed to start camera")
        return

    print(f"\n{'='*60}")
    print(f"  Hardware PTZ Demo")
    print(f"{'='*60}")
    print(f"  Web UI: http://{config.local_ip}:{config.onvif_port}/")
    print(f"  RTSP:   {config.main_stream_rtsp}")
    print(f"{'='*60}")
    print("\nUse the web UI or an ONVIF client to control PTZ.")
    print("Hardware commands will be printed to the console.")
    print("\nPress Ctrl+C to stop\n")

    # Open webcam
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print("Error: Could not open webcam")
        camera.stop()
        return
    countr = 0
    try:
        while camera.is_running:
            ret, frame = cap.read()
            if not ret:
                break
            camera.stream(frame)                                                                   # Stream handles PTZ, timestamp, and frame pacing automatically
            if (countr == 100) and (cFlag == 1):                                                   # dump the controller state every 100 frames so we recall position if we close 
                with open('gimbal.pickle', 'wb') as f:
                    pickle.dump(gimbal, f)
                countr = 0
            else:
                countr += 1
                counter %= 10000
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        cap.release()
        camera.stop()
        del GimbalExample

def hardware_only_example():
    """
    Demo: Hardware-only PTZ (no digital cropping)
    
    Use this when you have a physical PTZ camera and don't
    need digital PTZ simulation.
    """
    config = CameraConfig.load("camera_config.json")
    camera = IPCamera(config=config)

    # Disable digital PTZ - frames pass through unchanged
    camera.ptz.enable_digital_ptz = False

    # Add your hardware controller
    camera.ptz.add_hardware_handler(PrintingPTZHandler())

    # ... rest of setup ...
    print("Hardware-only mode: Digital PTZ disabled")

if __name__ == "__main__":

    main()

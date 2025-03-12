#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#   Configurable stanley controller set-up is sc_config.ini and planned course is target_course_1st.csv
#   uses cubic spline planner for course calculation.
#
"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

 Some code is taken from here https://github.com/AtsushiSakai/PythonRobotics/tree/master
 I have incorporatd the relavant functions to avoid whole repository needed above for this trial
 
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib
import pandas as pd
import numpy as np
import configparser
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from angle import angle_mod
import cubic_spline_planner

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

k = 0.5                                                         # control gain
Kp = 1.0                                                        # speed proportional gain
dt = 0.1                                                        # [s] time difference
L = 2.9                                                         # [m] Wheel base of vehicle
msa = 30.0                                                      # max steering angle
max_steer = np.radians(msa)                                     # [rad] max steering angle
init_angle = 20.0                                               # initial angle in degrees
show_animation = True                                           # draw animation or not

init_xpos = 0.0                                                 # start position and velocity
init_ypos = 5.0
init_velo = 0.0

target_speed = 30.0 / 3.6                                       # [m/s]
max_simulation_time = 100.0
pomin = 0.0                                                     # min speed
pomax = 300.0                                                   # max speed

lock = threading.Lock()                                         # create a lock to protect the writes to the globals
gRange = 0.4                                                    # lidar range
gTwister = Twist()                                              # global twist object to pass to ROS object
RUN_IT = 1

try:
    config_ini = configparser.ConfigParser()                                                  # read the config.ini for various configurations
    config_ini.read('sc_config.ini', encoding='utf-8')
    sections = config_ini.sections()                                                          # check if we have the PID section included
    for sec in sections:
        if sec == "PROP":                                                                     # proportional speed controller
            Kp = float(config_ini['PROP']['PB'])
            dt = float(config_ini['PROP']['DT'])
            L = float(config_ini['PROP']['L'])
            pomax = float(config_ini['PROP']['MAX_SP'])                                                                                
            pomin = float(config_ini['PROP']['MIN_SP'])
            target_speed = float(config_ini['PROP']['TS'])
        elif sec == "INIT":                                                                   # init settings
            init_xpos = float(config_ini['INIT']['X'])
            init_ypos = float(config_ini['INIT']['Y'])
            init_velo = float(config_ini['INIT']['V'])
            init_angle = float(config_ini['INIT']['A'])
        elif sec == "STANLEY":                                                                # stanley
            k = float(config_ini['STANLEY']['K'])
            msa = float(config_ini['STANLEY']['MAX_ST_ANG_DEG'])
            max_steer = np.radians(msa)                                                       # [rad] max steering angle
            max_simulation_time = float(config_ini['STANLEY']['MST'])
        elif sec == "LIDAR":                                                                  # lidar avoidance
            gRange = float(config_ini['LIDAR']['RNG'])
except:
    print(f"Something is wrong with reading sc_config.ini file #{i}")
    continue

# ROS object that moves with avoidance
class MoveWithAvoidance:
    def __init__(self):
        rospy.init_node('avoidance', anonymous=True)
        
        self.cmd_vel_pub = rospy.Publisher('/planner/cmd_vel', Twist, queue_size=10)                          # Publisher publishes velocity to the craft

        scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)                                   # Subscriber looks for obstacles
        
        self.min_range = None

    def callback_scan(self, data):
        fov = np.deg2rad(60)
        min_range = data.range_max
        min_idx = -1
        angle = data.angle_min
        for idx, r in enumerate(data.ranges):
            angle += data.angle_increment
            if -fov<angle<fov:
                if r<min_range:
                    min_range = r
                    min_idx = idx
        if min_idx < len(data.ranges)/2.0:
            self.direction = "RIGHT"
        else:
            self.direction = "LEFT"
        self.min_range = min_range

    def process(self):
        global gTwister
        global RUN_IT
        r = rospy.Rate(10)
        while (not rospy.is_shutdown()) or (RUN_IT == 0):
            vel = Twist()
            if self.min_range is not None:
                if self.min_range >= gRange:
                    vel.linear.x = gTwister.linear.x
                    vel.angular.x = gTwister.angular.x
                    vel.angular.y = gTwister.angular.y
                    vel.angular.z = gTwister.angular.z
                else:
                    vel.linear.x = 0.0
                    if self.direction == "RIGHT":
                        vel.angular.z = 0.5
                    elif self.direction == "LEFT":
                        vel.angular.z = -0.5
            self.cmd_vel_pub.publish(vel)
            r.sleep()
            
# state engine for stanley controller                
class State:
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super().__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt

# P control action
def p_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return np.clip((Kp * (target - current)), pomin, pomax)

# stanley controller
def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    return angle_mod(angle)


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),-np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def do_stanley():
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    #ax = [0.0, 100.0, 100.0, 50.0, 60.0]
    #ay = [0.0, 0.0, -30.0, -20.0, 0.0]
    global gTwister
    
    # read the target course from the data file and use the cubic spline planner to make the planned course
    df = pd.read_csv('target_course_1st.csv')                                          
    ax=df['ax'].values                                                                   
    ay=df['ay'].values                                                                   

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    # Initial state
    state = State(x=init_xpos, y=init_ypos, yaw=np.radians(init_angle), v=init_velo)

    last_idx = len(cx) - 1                                            # get number of iterations
    time = 0.0                                                        # get starting variables
    x = [state.x]                                                     # initialise arrays with first start point for x,y,yaw,v,t
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)                                      # calculate first index

    while max_simulation_time >= time and last_idx > target_idx:                          # iterate on time or if you reach goal
        ai = p_control(target_speed, state.v)                                             # p control for the speed
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        time += dt

        x.append(state.x)                                                                  # append the data to each array for plotting
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        # set the object for ROS to move the bot
        lock.acquire()
        gTwister.linear.x = state.v
        gTwister.angular.x = state.x
        gTwister.angular.y = state.y
        gTwister.angular.z =  state.yaw
	    lock.release()
        
        if show_animation:  
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Print Error if we did not meet target
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:  
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

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
            
if __name__ == '__main__':
    global RUN_IT
    avoidance = MoveWithAvoidance()
    controls_task = twe(name = 'Thread Stanley', target=do_stanley, args=(), kwargs={})
    try:
        controls_task.start()
        try:
            avoidance.process()
        except rospy.ROSInitException:
            pass
        controls_task.join()
        RUN_IT=0
    except KeyboardInterrupt:
        RUN_IT=0
        if controls_task.is_alive(): controls_task.raise_exception()
        controls_task.join()

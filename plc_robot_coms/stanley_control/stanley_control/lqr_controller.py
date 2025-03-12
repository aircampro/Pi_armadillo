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
import math
import scipy.linalg as la

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
show_animation = True                                           # draw animation or not

init_xpos = 0.0                                                 # start position and velocity
init_ypos = 5.0
init_velo = 0.0
init_angle = 20.0                                               # initial angle in degrees

target_speed = 30.0 / 3.6                                       # [m/s]     
max_simulation_time = 100.0
pomin = 0.0                                                     # min speed
pomax = 300.0                                                   # max speed

lock = threading.Lock()                                         # create a lock to protect the writes to the globals
gRange = 0.4                                                    # lidar range
gTwister = Twist()                                              # global twist object to pass to ROS object
RUN_IT = 1

T = 500.0                                                       # max simulation time
goal_dis = 0.3
stop_speed = 0.05
    
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
        elif sec == "LQR":                                                                
            T = float(config_ini['LQR']['T'])
            stop_speed = float(config_ini['LQR']['SS'])
            goal_dis = float(config_ini['LQR']['GD'])                                                     
            max_simulation_time = float(config_ini['LQR']['MST'])
            msa = float(config_ini['LQR']['MAX_ST_ANG_DEG'])
            max_steer = np.radians(msa) 
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

"""

Path tracking simulation with LQR speed and steering control

author Atsushi Sakai (@Atsushi_twi)

"""

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

# === Parameters =====

# LQR parameter
lqr_Q = np.eye(5)
lqr_R = np.eye(2)

class StateLQR:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def lqr_update(state, a, delta):

    if delta >= max_steer:
        delta = max_steer
    if delta <= - max_steer:
        delta = - max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state


def pi_2_pi(angle):
    return angle_mod(angle)


def solve_dare(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    x = Q
    x_next = Q
    max_iter = 150
    eps = 0.01

    for i in range(max_iter):
        x_next = A.T @ x @ A - A.T @ x @ B @ \
                 la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
        if (abs(x_next - x)).max() < eps:
            break
        x = x_next

    return x_next


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_dare(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    eig_result = la.eig(A - B @ K)

    return K, X, eig_result[0]


def lqr_speed_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, sp, Q, R):
    ind, e = calc_nearest_index(state, cx, cy, cyaw)

    tv = sp[ind]

    k = ck[ind]
    v = state.v
    th_e = pi_2_pi(state.yaw - cyaw[ind])

    # A = [1.0, dt, 0.0, 0.0, 0.0
    #      0.0, 0.0, v, 0.0, 0.0]
    #      0.0, 0.0, 1.0, dt, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 0.0]
    #      0.0, 0.0, 0.0, 0.0, 1.0]
    A = np.zeros((5, 5))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = v
    A[2, 2] = 1.0
    A[2, 3] = dt
    A[4, 4] = 1.0

    # B = [0.0, 0.0
    #     0.0, 0.0
    #     0.0, 0.0
    #     v/L, 0.0
    #     0.0, dt]
    B = np.zeros((5, 2))
    B[3, 0] = v / L
    B[4, 1] = dt

    K, _, _ = dlqr(A, B, Q, R)

    # state vector
    # x = [e, dot_e, th_e, dot_th_e, delta_v]
    # e: lateral distance to the path
    # dot_e: derivative of e
    # th_e: angle difference to the path
    # dot_th_e: derivative of th_e
    # delta_v: difference between current speed and target speed
    x = np.zeros((5, 1))
    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt
    x[4, 0] = v - tv

    # input vector
    # u = [delta, accel]
    # delta: steering angle
    # accel: acceleration
    ustar = -K @ x

    # calc steering input
    ff = math.atan2(L * k, 1)  # feedforward steering angle
    fb = pi_2_pi(ustar[0, 0])  # feedback steering angle
    delta = ff + fb

    # calc accel input
    accel = ustar[1, 0]

    return delta, ind, e, th_e, accel


def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def do_simulation(cx, cy, cyaw, ck, speed_profile, goal):

    global gTwister
         
    state = StateLQR(x=init_xpos, y=init_ypos, yaw=np.radians(init_angle), v=init_velo)

    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]

    e, e_th = 0.0, 0.0

    while T >= time:
        dl, target_ind, e, e_th, ai = lqr_speed_steering_control(
            state, cx, cy, cyaw, ck, e, e_th, speed_profile, lqr_Q, lqr_R)

        state = lqr_update(state, ai, dl)

        if abs(state.v) <= stop_speed:
            target_ind += 1

        time = time + dt

        # check goal
        dx = state.x - goal[0]
        dy = state.y - goal[1]
        if math.hypot(dx, dy) <= goal_dis:
            print("@Goal")
            break

        x.append(state.x)
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
        
        if target_ind % 1 == 0 and show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "ob", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2))
                      + ",target index:" + str(target_ind))
            plt.pause(0.0001)

    return t, x, y, yaw, v


def calc_speed_profile(cyaw, target_speed):
    speed_profile = [target_speed] * len(cyaw)

    direction = 1.0

    # Set stop point
    for i in range(len(cyaw) - 1):
        dyaw = abs(cyaw[i + 1] - cyaw[i])
        switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

        if switch:
            direction *= -1

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

        if switch:
            speed_profile[i] = 0.0

    # speed down
    for i in range(40):
        speed_profile[-i] = target_speed / (50 - i)
        if speed_profile[-i] <= 1.0 / 3.6:
            speed_profile[-i] = 1.0 / 3.6

    return speed_profile


def lqrmain():
    print("LQR steering control tracking start!!")

    #ax = [0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0]
    #ay = [0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0]
    #  target course
    
    # read the target course from the data file and use the cubic spline planner to make the planned course
    df = pd.read_csv('target_course_1st.csv')                                          
    ax=df['ax'].values                                                                   
    ay=df['ay'].values     
    goal = [ax[-1], ay[-1]]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

    sp = calc_speed_profile(cyaw, target_speed)

    t, x, y, yaw, v = do_simulation(cx, cy, cyaw, ck, sp, goal)

    if show_animation:  # pragma: no cover
        plt.close()
        plt.subplots(1)
        plt.plot(ax, ay, "xb", label="waypoints")
        plt.plot(cx, cy, "-r", label="target course")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()
        plt.subplots(1)

        plt.plot(t, np.array(v)*3.6, label="speed")
        plt.grid(True)
        plt.xlabel("Time [sec]")
        plt.ylabel("Speed [m/s]")
        plt.legend()

        plt.subplots(1)
        plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="yaw")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("yaw angle[deg]")

        plt.subplots(1)
        plt.plot(s, ck, "-r", label="curvature")
        plt.grid(True)
        plt.legend()
        plt.xlabel("line length[m]")
        plt.ylabel("curvature [1/m]")

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
    controls_task = twe(name = 'Thread LQR', target=lqrmain, args=(), kwargs={})
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

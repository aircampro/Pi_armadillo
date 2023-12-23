#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python library to allow universal robot control from python code running on the master server
#
# Ref:: Universal Robots The URScript Programming Language
#
# https://s3-eu-west-1.amazonaws.com/ur-support-site/32554/scriptManual-3.5.4.pdf
#
# Universal Robots' robots UR3, UR5 and UR10 are controlled by the common control box CB3.1 SW3.5 and are supported by this interface
#
import socket
import time
import math

# check the python version
import sys
import platform
if (sys.version_info.major == 3):
    MY_PY = 3
elif (sys.version_info.major == 2):    
    MY_PY = 2
else:
    print("unknown python version detected ", sys.version_info.major)
    sys.exit(1)

# set the robot ip address in this global
HOST = "192.168.250.110"
PORT = 30002

# useful math conversions
def deg_2_rad(deg):
    return (math.pi*deg)/180.0

def rad_2_deg(rad):
    return (180.0*rad)/math.pi

def ft_2_mtr(ft):
    return ft*0.3048

def mtr_2_ft(mtr):
    return mtr*3.2808

# Returns the RPY vector corresponding to ’rotation_vector’ where the
# rotation vector is the axis of rotation with a length corresponding to the
# angle of rotation in radians.
def rotvec2rpy(s, vecin):   
    s.send(toBytes("rotvec2rpy("+str(vecin)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# Returns the rotation vector corresponding to ’rpy_vector’ where the
# RPY (roll-pitch-yaw) rotations are extrinsic rotations about the X-Y-Z axes
# (corresponding to intrinsic rotations about the Z-Y’-X” axes).
def rpy2rotvec(s, vecin):   
    s.send(toBytes("rpy2rotvec("+str(vecin)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv    
    
# convert to bytes    
def toBytes(str):
    return bytes(str.encode('utf-8'))

def robot_connect(h=HOST,p=PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((h, p))
    return s

def robot_disconnect(s):
    s.close()

# disconnect robot to another remote server
def robot_remote_disconnect(s):
    s.send(toBytes("socket_close(socket_name="socket_0")" + "\n"))

# connect robot to another remote server edit data sent if required in function
def robot_remote_connect(s):
    s.send(toBytes("socket_open(\"192.168.5.1\", 50000,\"socket_0\")" + "\n"))
    
# The following is a script that brings the robot to a fully extended arm
# The array of the first argument contains the angle of each joint in radians. 
# Please note that you do not enter it in degrees. The second argument is the acceleration of the robot, 
# and the third argument is the velocity.    
def extend_arm_fully(s):
    s.send(toBytes("movej([0,-1.57,0,0,-1.57,1.57], a=1.40, v=1.04)" + "\n"))
    time.sleep(3)
    data = s.recv(1024)
    if (MY_PY == 3):
        data = data.decode('utf-8')
    print("Receved", repr(data))
    print("Program finish")

# The following is a program to get the current location and move it 100mm in the x direction.
def move_100m_X(s):
    s.send(toBytes(
        "def myProg():"+"\n"                        # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"  # get the start position 
        +"pos_end = pose_add(begin_pos, p[0.100, 0.0, 0.0, 0.0, 0.0, 0.0])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"   # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                              # end of program

# The following is a program to get the current location and move it speicified radian angles in the x.y.z direction.
def move_XYZ_to_angle(s, x, y, z):
    s.send(toBytes(
        "def myProg():"+"\n"                        # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"  # get the start position 
        +"pos_end = pose_add(begin_pos, p[0.0, 0.0, 0.0,"+str(x)+","+str(y)+","+str(z)+"])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"   # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                              # end of program

# The following is a program to get the current location and move it speicified ammounts in the x.y.z direction.
def move_XYZ_to_dist(s, x=0.1, y=0.1, z=0):
    s.send(toBytes(
        "def myProg():"+"\n"                        # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"  # get the start position 
        +"pos_end = pose_add(begin_pos, p["+str(x)+","+str(y)+","+str(z)+", 0.0, 0.0, 0.0])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"   # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                              # end of program

def move_XYZ_dist_to_pose_array(s,tup):
    s.send(toBytes(
        "def myProg():"+"\n"                        # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"  # get the start position 
        +"pos_end = pose_add(begin_pos, p["+str(tup[0])+","+str(tup[1])+","+str(tup[2])+", 0.0, 0.0, 0.0])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"   # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                              # end of program

def move_XYZ_to_pose_array(s,tup):
    s.send(toBytes(
        "def myProg():"+"\n"                        # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"  # get the start position 
        +"pos_end = pose_add(begin_pos, p["+str(tup[0])+","+str(tup[1])+","+str(tup[2])+","+str(tup[3])+","+str(tup[4])+","+str(tup[5])+"])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"   # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                              # end of program
        
# The following is a program to get the current location and move it 100mm in reverse in the x direction.
def move_100m_Xback(s):
    s.send(toBytes(
        "def myProg():"+"\n"                        # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"  # get the start position 
        +"pos_end = pose_add(begin_pos, p[-0.100, 0.0, 0.0, 0.0, 0.0, 0.0])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"   # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                              # end of program
    
# The following is a program to get the current location and move it 100mm in the y direction.
def move_100m_Y(s):
    s.send(toBytes(
        "def myProg():"+"\n"                        # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"  # get the start position 
        +"pos_end = pose_add(begin_pos, p[0.0, 0.100, 0.0, 0.0, 0.0, 0.0])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"   # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                              # end of program
    
# The following is a program to get the current location and move it 100mm in the z direction.
def move_100m_Z(s):
    s.send(toBytes(
        "def myProg():"+"\n"                        # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"  # get the start position 
        +"pos_end = pose_add(begin_pos, p[0.0, 0.0, 100.0, 0.0, 0.0, 0.0])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"   # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                              # end of program

# The following is a program to get the current location and move it 100mm back in the z direction.
def move_100m_Zback(s):
    s.send(toBytes(
        "def myProg():"+"\n"                                                       # start of program
        +"begin_pos = get_actual_tcp_pose()" +"\n"                                 # get the start position 
        +"pos_end = pose_add(begin_pos, p[0.0, 0.0, -100.0, 0.0, 0.0, 0.0])" +"\n" # find the end position
        +"movel(pos_end , a=0.39, v=0.05)" + "\n"                                  # move to from start pos to end pos at accel=a and velocity=v
        +"end" +"\n"))                                                             # end of program
    
def force_mode(s, pose_to, sv, wr, ty, li):
    s.send(toBytes(
        "def myProg():"+"\n"
        "task_frame=p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n" # pose vector
        "selection_vector="+str(sv)+"\n"                                           # A 6d vector of 0s and 1s.
        "wrench="+str(wr)+"\n"                                                     # wrench
        "type="+str(ty)+"\n"                                                       # A type.
        "limits="+str(li)+"\n"                                                     # limits
        +"force_mode(task_frame, selection_vector, wrench, type, limits)" + "\n"
        +"end" +"\n"))   
    
def force2normal(s):    
    s.send(toBytes("end_force_mode" + "\n"))

def end_freedrive(s):    
    s.send(toBytes("end_freedrive_mode()" + "\n"))

def end_teach(s):    
    s.send(toBytes("end_teach_mode()" + "\n"))

def go_freedrive(s):   
    s.send(toBytes("freedrive_mode()" + "\n")) 

def go_teach(s):   
    s.send(toBytes("teach_mode()" + "\n")) 

def modbus_add_signal(s, modbus_ip = "172.140.17.11", slave_no = 255, signal_add = 5, signal_type = 1, signal_name = "output1"):   
    s.send(toBytes(
        "def myProg():"+"\n"
        "modbus_ip="+"\""+str(modbus_ip)+"\""+"\n" 
        "slave_no="+str(slave_no)+"\n" 
        "signal_add="+str(signal_add)+"\n" 
        "signal_type="+str(signal_type)+"\n" 
        "signal_name="+str(signal_name)+"\n" 
        "modbus_add_signal(modbus_ip, slave_no, signal_add, signal_type, signal_name)" + "\n"
        +"end" +"\n"))    

def modbus_del_signal(s, s_name):   
    s.send(toBytes("modbus_delete_signal(" + str(s_name) + ") \n"))

def modbus_get_signal(s, s_name):   
    s.send(toBytes("modbus_get_signal_status(" + str(s_name) + ",False) \n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def modbus_set_register(s, s_name, val=0):   
    s.send(toBytes("modbus_set_output_register(" + str(s_name) + "," + str(val) + ",False) \n"))

def modbus_set_signal(s, s_name, val=True):   
    s.send(toBytes("modbus_set_output_signal(" + str(s_name) + "," + str(val) + ",False) \n"))

# sets modbus io failsafe state
def modbus_set_runstate(s, s_name, runs=0):   
    s.send(toBytes("modbus_set_runstate_dependent_choice(" + str(s_name) + "," + str(runs) + ") \n"))

def modbus_set_update_rate(s, s_name, freq=20):   
    s.send(toBytes("modbus_set_signal_update_frequency(" + str(s_name) + "," + str(freq) + ") \n"))
    
def get_tick_count(s):   
    s.send(toBytes("get_conveyor_tick_count()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# returns
# The current actual joint angular position vector in rad : [Base,
# Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
def get_actual_joint_positions(s):   
    s.send(toBytes("get_actual_joint_positions()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# returns
# The current actual joint angular velocity vector in rad/s:
# [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
def get_actual_joint_speeds(s):   
    s.send(toBytes("get_actual_joint_speeds()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
# Returns the 6d pose representing the tool position and orientation
# specified in the base frame. The calculation of this pose is based on
# the actual robot encoder readings
def get_actual_tcp_pose(s):   
    s.send(toBytes("get_actual_tcp_pose()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# Returns The speed of the TCP retuned in a pose structure. The first three values
# are the cartesian speeds along x,y,z, and the last three define the
# current rotation axis, rx,ry,rz, and the length |rz,ry,rz| defines the
# angular velocity in radians/s.
def get_actual_tcp_speed(s):   
    s.send(toBytes("get_actual_tcp_speed()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# Returns Returns the 6d pose representing the tool flange position and
# orientation specified in the base frame, without the Tool Center Point
# offset. The calculation of this pose is based on the actual robot
# encoder readings.
def get_actual_tool_flange_pose(s):   
    s.send(toBytes("get_actual_tool_flange_pose()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def get_control_box_temp(s):   
    s.send(toBytes("get_controller_temp()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_joint_temp(s,num=1):   
    s.send(toBytes("get_joint_temp("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# Reads the current value of a specific Euromap67 input signal. See
# http://universal-robots.com/support for signal specifications.
def get_euromap_in(s,num=1):   
    s.send(toBytes("get_euromap_input("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
# Reads the current value of a specific Euromap67 input signal. See
# http://universal-robots.com/support for signal specifications.
def get_euromap_out(s,num=1):   
    s.send(toBytes("get_euromap_output("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def set_euromap_out(s, num=1, value=True):   
    s.send(toBytes("set_euromap_output("+str(num)+","+str(value)+")"+ "\n"))

def toggle_euromap_out(s, num=1):  
    value = not get_euromap_out(num) 
    s.send(toBytes("set_euromap_output("+str(num)+","+str(value)+")"+ "\n"))

# set euromap failsafe state 
# An integer: 0 = preserve program state,
# 1 = set low when a program is not
# running, 2 = set high when a program is
# not running.
def set_euromap_runstate_dependent_choice(s, num=1, runstate=2):   
    s.send(toBytes("set_euromap_runstate_dependent_choice("+str(num)+","+str(runstate)+")"+ "\n"))

def set_runstate_configurable_digital_output_to_value(s, num=1, runstate=2):   
    s.send(toBytes("set_runstate_configurable_digital_output_to_value("+str(num)+","+str(runstate)+")"+ "\n"))

def set_runstate_gp_boolean_output_to_value(s, num=1, runstate=2):   
    s.send(toBytes("set_runstate_gp_boolean_output_to_value("+str(num)+","+str(runstate)+")"+ "\n"))

def set_runstate_standard_analog_output_to_value(s, num=1, runstate=2):   
    s.send(toBytes("set_runstate_standard_analog_output_to_value("+str(num)+","+str(runstate)+")"+ "\n"))

def set_runstate_standard_digital_output_to_value(s, num=1, runstate=2):   
    s.send(toBytes("set_runstate_standard_digital_output_to_value("+str(num)+","+str(runstate)+")"+ "\n"))

def set_runstate_tool_digital_output_to_value(s, num=1, runstate=2):   
    s.send(toBytes("set_runstate_tool_digital_output_to_value("+str(num)+","+str(runstate)+")"+ "\n"))

# analog input domains: 0: 4-20mA, 1: 0-10V
def set_standard_analog_input_domain(s, num=1, dom=0):   
    s.send(toBytes("set_standard_analog_input_domain("+str(num)+","+str(dom)+")"+ "\n"))

def set_standard_analog_out(s, num=1, v=1.1):   
    s.send(toBytes("set_standard_analog_out("+str(num)+","+str(v)+")"+ "\n"))

def set_standard_digital_out(s, num=1, v=True):   
    s.send(toBytes("set_standard_digital_out("+str(num)+","+str(v)+")"+ "\n"))

# analog input domains: 0: 4-20mA, 1: 0-10V
def set_tool_analog_input_domain(s, num=1, v=1):   
    s.send(toBytes("set_tool_analog_input_domain("+str(num)+","+str(v)+")"+ "\n"))

def set_tool_digital_out(s, num=1, v=True):   
    s.send(toBytes("set_tool_digital_out("+str(num)+","+str(v)+")"+ "\n"))

def set_tool_voltage(s, num=12):   
    s.send(toBytes("set_tool_voltage("+str(num)+")"+ "\n"))

def socket_get_var(s, name, socket_name="’socket_0’"):   
    s.send(toBytes("socket_get_var("+str(name)+","+str(socket_name)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def socket_read_ascii_float(s, name=4, socket_name="’socket_0’"):   
    s.send(toBytes("socket_read_ascii_float("+str(name)+","+str(socket_name)+", timeout=2)"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def socket_read_ascii_float(s, name=4, socket_name="’socket_0’"):   
    s.send(toBytes("socket_read_ascii_float("+str(name)+","+str(socket_name)+", timeout=2)"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def socket_read_binary_integer(s, num=1, socket_name="’socket_0’"):   
    s.send(toBytes("socket_read_binary_integer("+str(num)+","+str(socket_name)+", timeout=2)"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def socket_read_byte_list(s, num=1, socket_name="’socket_0’"):   
    s.send(toBytes("socket_read_byte_list("+str(num)+","+str(socket_name)+", timeout=2)"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def socket_read_line(s, socket_name="’socket_0’"):   
    s.send(toBytes("socket_read_line("+str(socket_name)+", timeout=2)"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# you can add prefix=">", suffix="<" for example sending ">hello<>world<"
def socket_read_string(s, socket_name="’socket_0’"):   
    s.send(toBytes("socket_read_string("+str(socket_name)+", timeout=2)"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# Creates a new Remote Procedure Call (RPC) handle. Please read the
# subsection ef{Remote Procedure Call (RPC)} for a more detailed
# description of RPCs.
# The URL to the RPC server. Currently two protocols are
# supported: pstream and http. The pstream URL looks
# like "<ip-address>:<port>", for instance "127.0.0.1:8080"
def rpc_factory(s, x="xmlrpc", url="’http://127.0.0.1:8080/RPC2’"):   
    s.send(toBytes("rpc_factory(\""+str(x)+"\","+str(url)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# this is an example of threading 2 tasks in the robot modify as you like
# a test of running 2 threads one modifying a global 100 times, the other doing a movement abs altering a local variable
def run_two_threads_test(s):   
    s.send(toBytes(
        "def myProg():"+"\n"
        "global a = 0.1"+"\n"
        "thread myThread1():"+"\n"
        "  # Do some stuff"+"\n"
        "  i = 0"+"\n"
        "  while i < 100:"+"\n"
        "    if a < 1:"+"\n"
        "      a = (cos(a)/10.0) + a"+"\n"
        "    else:"+"\n"
        "      a = -(sin(a)*10.0) + a"+"\n" 
        "    end"+"\n"
        "    i = i + 1"+"\n"
        "  end"+"\n"        
        "  return False"+"\n"
        "end"+"\n"
        "thread myThread2():"+"\n"
        "  b = 1"+"\n"
        "  c = 0.1"+"\n"
        "  while b < 2:"+"\n"
        "    if c > 0.5:"+"\n"
        "      c = c - 0.1"+"\n"
        "    else:" + "\n"
        "      c = c + 0.1"+"\n"
        "    end"+"\n"
        "    enter_critical"+"\n"
        "    # Do some critical stuff it wont do other thread until this is complete"+"\n"
        "    movej([a, 1.57, c, 3.14, -1.57, 1.57],a=1.4, v=1.05, t=0, r=0)"+"\n"
        "    exit_critical"+"\n"
        "  end"+"\n"  
        "  return False"+"\n"
        "end"+"\n"
        "thrd1 = run myThread1()"+"\n"
        "thrd2 = run myThread2()"+"\n"
        "# wait for thread1 completion"+"\n"
        "join thrd1"+"\n"
        "# stop second thread"+"\n"
        "kill thrd2"+"\n"
        +"end" +"\n")) 
        
def two_thread_example(s, pose):   
    s.send(toBytes(
        "def myProg():"+"\n"
        "camera = rpc_factory(\"xmlrpc\", \"http://127.0.0.1/RPC2\")"+"\n"   
        "if (! camera.initialize(\"RGB\")):" + "\n"
        "  popup(\"Camera was not initialized\")" + "\n"
        "camera.takeSnapshot()" + "\n"
        "target = camera.getTarget()" + "\n"
        +"end" +"\n")) 
     
def socket_send_byte(s, byte=8, socket_name="’socket_0’"):   
    s.send(toBytes("socket_send_byte("+str(byte)+","+str(socket_name)+", timeout=2)"+ "\n"))

def socket_send_int(s, integ=8, socket_name="’socket_0’"):   
    s.send(toBytes("socket_send_int("+str(integ)+","+str(socket_name)+")"+ "\n"))

def socket_send_line(s, lin="hello from server", socket_name="’socket_0’"):   
    s.send(toBytes("socket_send_line("+str(lin)+","+str(socket_name)+")"+ "\n"))

def socket_send_string(s, st="hello", socket_name="’socket_0’"):   
    s.send(toBytes("socket_send_string(\""+str(st)+"\","+str(socket_name)+")"+ "\n"))

def socket_set_var(s, nm="X_POS", value=12, socket_name="’socket_0’"):   
    s.send(toBytes("socket_set_var(\""+str(nm)+"\","+str(value)+","+str(socket_name)+")"+ "\n"))
    
# set internal memory flag (0-32) i.e. internal memory bits
def set_flag(s, num=32, val=True):   
    s.send(toBytes("get_configurable_digital_out("+str(num)+","+str(val)+")"+ "\n"))

# Action Options are: Either "ignore", "pause" or "stop"
# the program on a violation of the
# minimum frequency in Hz. The default action is "pause"
def rtde_set_watchdog(s, num="'input_int_register_0'", min_freq=20, action="’pause’"):   
    s.send(toBytes("rtde_set_watchdog("+str(num)+","+str(min_freq)+","+str(action)+")"+ "\n"))

def modbus_send_custom_command(s, i=1, s=10, f=6, d=[17,32,2,88]):   
    s.send(toBytes("modbus_send_custom_command("+str(i)+","+str(s)+","+str(f)+","+str(d)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def read_input_boolean_register(s, num=1):   
    s.send(toBytes("read_input_boolean_register("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def read_input_float_register(s, num=1):   
    s.send(toBytes("read_input_float_register("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def read_input_integer_register(s, num=1):   
    s.send(toBytes("read_input_integer_register("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def read_output_boolean_register(s, num=1):   
    s.send(toBytes("read_output_boolean_register("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def write_output_boolean_register(s, num=1, val=False):   
    s.send(toBytes("write_output_boolean_register("+str(num)+","+str(val)+")"+ "\n"))
    
def read_output_float_register(s, num=1):   
    s.send(toBytes("read_output_float_register("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def write_output_float_register(s, num=1, val=1.11):   
    s.send(toBytes("write_output_float_register("+str(num)+","+str(val)+")"+ "\n"))
    
def read_output_integer_register(s, num=1):   
    s.send(toBytes("read_output_integer_register("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def write_output_integer_register(s, num=1, val=111):   
    s.send(toBytes("write_output_integer_register("+str(num)+","+str(val)+")"+ "\n"))
    
def read_port_bit(s, num=1):   
    s.send(toBytes("read_port_bit("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def write_port_bit(s, num=1, val=False):   
    s.send(toBytes("write_port_bit("+str(num)+","+str(val)+")"+ "\n"))
    
def read_port_register(s, num=1):   
    s.send(toBytes("read_port_regsiter("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def write_port_regsiter(s, num=1, val=100):   
    s.send(toBytes("write_port_regsiter("+str(num)+","+str(val)+")"+ "\n"))
    
def get_configurable_digital_out(s, num=1):   
    s.send(toBytes("get_configurable_digital_out("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_standard_digital_in(s, num=1):   
    s.send(toBytes("get_standard_digital_in("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_tool_digital_in(s, num=1):   
    s.send(toBytes("get_tool_digital_in("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_tool_digital_out(s, num=1):   
    s.send(toBytes("get_tool_digital_out("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def get_standard_digital_out(s, num=1):   
    s.send(toBytes("get_standard_digital_out("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def get_standard_analog_in(s, num=1):   
    s.send(toBytes("get_standard_analog_in("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_tool_analog_in(s, num=1):   
    s.send(toBytes("get_tool_analog_in("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def get_standard_analog_out(s, num=1):   
    s.send(toBytes("get_standard_analog_out("+str(num)+")"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def get_tool_current(s):   
    s.send(toBytes("get_tool_current()"+ "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def is_robot_at_rest(s):   
    s.send(toBytes("is_steady()"+"\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def is_robot_safe(s, pose):   
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose="+"p["+str(pose[0])+","+str(pose[1])+","+str(pose[2])+","+str(pose[3])+","+str(pose[4])+","+str(pose[5])+"]"+"\n" # pose vector  
        "is_within_safety_limits(pose)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
# Return Value
# The joint torque vector in Nm: [Base, Shoulder, Elbow, Wrist1,
# Wrist2, Wrist3
def get_joint_torques(s):   
    s.send(toBytes("get_joint_torques()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

#Return Value
#The current target joint angular position vector in rad: [Base,
#Shoulder, Elbow, Wrist1, Wrist2, Wrist3
def get_target_joint_positions(s):   
    s.send(toBytes("get_target_joint_positions()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_target_joint_speeds(s):   
    s.send(toBytes("get_target_joint_speeds()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_target_tcp_pose(s):   
    s.send(toBytes("get_target_tcp_pose()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_target_tcp_speed(s):   
    s.send(toBytes("get_target_tcp_speed()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def get_tcp_force(s):   
    s.send(toBytes("get_tcp_force()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# Return Value
#X, Y, and Z composant of the measured acceleration in
#SI-units (m/s^2).
def get_tool_accelerometer_reading(s):   
    s.send(toBytes("get_tool_accelerometer_reading()" + "\n"))
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# defaults a quad encoder 
# types
# 0 is no encoder, pulse decoding is disabled.
# 1 is quadrature encoder, input A and B must be
# square waves with 90 degree offset. Direction of the
# conveyor can be determined.
# 2 is rising and falling edge on single input (A).
# 3 is rising edge on single input (A).
# 4 is falling edge on single input (A).
# The controller can decode inputs at up to 40kHz 
def conveyor_pulse_decode(s, type=1, inp1=0, inp2=1):   
    if type == 2:
        s.send(toBytes("conveyor_pulse_decode("+str(type)+","+str(inp1)+")" + "\n"))
    else :    
        s.send(toBytes("conveyor_pulse_decode("+str(type)+","+str(inp1)+","+str(inp2)+")" + "\n"))
    
def force_mode_set_damping(s, damp):   
    s.send(toBytes("force_mode_set_damping("+str(damp)+")" + "\n"))

def power_off_robot(s, damp):   
    s.send(toBytes("powerdown"+ "\n"))
    
# pose is a tuple e.g. (x, y, z, angle_x_radians, angle_y_radians, angle_z_radians) p_v = (0.1,0.2,0.4,1.1,0.9,0.1) or vector [0.1,0.2,0.4,1.1,0.9,0.1]   
def move_circular(s, pose_via, pose_to, a=1.2, v=0.25, r=0.05, m=0):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_via="+"p["+str(pose_via[0])+","+str(pose_via[1])+","+str(pose_via[2])+","+str(pose_via[3])+","+str(pose_via[4])+","+str(pose_via[5])+"]"+"\n" # pose vector via 2nd position
        "pose_to="+"p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n"        # pose vector final 3rd position        
        " movec(pose_via, pose_to, a="+str(a)+", v="+str(v)+", r="+str(r)+", m="+str(m)+")" + "\n"
        +"end" +"\n")) 
        
def move_in_linear_joint_space(s, pose_to, v=0.25, r=0.05, t=0):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_to="+"p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n" # pose vector       
        " movej(pose_to, a="+str(a)+", v="+str(v)+", t="+str(t)+", r="+str(r)+")" + "\n"
        +"end" +"\n")) 

# sets the center of gravity by default we set to Y axis on earth
def set_gravity(s, x=0,y=9.82,z=0):
    s.send(toBytes(
        "def myProg():"+"\n"
        "x="+str(x)+"\n"  
        "y="+str(y)+"\n" 
        "z="+str(z)+"\n"         
        "set_gravity(x,y,z)" + "\n"
        +"end" +"\n")) 

#will set the acceleration for a robot that is rotated "theta" radians
#around the x-axis of the robot base coordinate system
def set_gravity_on_rotation(s, theta=math.pi/4):
    s.send(toBytes(
        "def myProg():"+"\n"
        "theta="+str(theta)+"\n"           
        " set_gravity([0, 9.82*sin(theta), 9.82*cos(theta)])" + "\n"
        +"end" +"\n")) 
        
# move default is 100m in x-direction      
def move_in_linear_tool_space(s, pose=(0.1,0,0,0,0,0), v=0.25, r=0.05, t=0):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose="+"p["+str(pose[0])+","+str(pose[1])+","+str(pose[2])+","+str(pose[3])+","+str(pose[4])+","+str(pose[5])+"]"+"\n" # pose vector      
        " movel(pose, a="+str(a)+", v="+str(v)+", t="+str(t)+", r="+str(r)+")" + "\n"
        +"end" +"\n"))

# → the position of the simulated
# robot with joint angles in radians representing rotations of
# base, shoulder, elbow, wrist1, wrist2 and wrist3
# pos=(b,s,e,w1,w2,w3) e.g. pos=[0.0,1.57,-1.57,0,0,3.14]
def set_pos(s, pos):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pos=["+str(pos[0])+","+str(pos[1])+","+str(pos[2])+","+str(pos[3])+","+str(pos[4])+","+str(pos[5])+"]"+"\n" # positions vector       
        "set_pos("+pos+")" + "\n"
        +"end" +"\n"))        

#spd: joint speeds [rad/s] base, shoulder, elbow, wrist1, wrist2 and wrist3
#a: joint acceleration [rad/s^2] (of leading axis)
#t: time [s] before the function returns (optional)       
def speed_joint(s, spd, a=0.25, t=0.05):
    s.send(toBytes(
        "def myProg():"+"\n"
        "spd=["+str(spd[0])+","+str(spd[1])+","+str(spd[2])+","+str(spd[3])+","+str(spd[4])+","+str(spd[5])+"]"+"\n" # speed vector 
        "speedj(spd, "+str(a)+","+str(t)+")" + "\n"
        +"end" +"\n")) 

def stop_joint(s, dec=2):
    s.send(toBytes(
        "def myProg():"+"\n" 
        "stopj("+str(dec)+")" + "\n"
        +"end" +"\n"))
        
def stop_tool(s, dec=2, arot=0.01):
    s.send(toBytes(
        "def myProg():"+"\n" 
        "stopl("+str(dec)+", aRot="+str(arot)+")" + "\n"
        +"end" +"\n")) 
        
def track_conveyor_circular(s, pose=(0.1,0,0,0,0,0), ticks=500.0, t='true'):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose="+"p["+str(pose[0])+","+str(pose[1])+","+str(pose[2])+","+str(pose[3])+","+str(pose[4])+","+str(pose[5])+"]"+"\n" # pose vector 
        "track_conveyor_circular(pose, "+str(ticks)+","+str(t)+")" + "\n"
        +"end" +"\n")) 

def track_conveyor_linear(s, pose=(1,0,0,0,0,0), ticks=1000.0):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose="+"p["+str(pose[0])+","+str(pose[1])+","+str(pose[2])+","+str(pose[3])+","+str(pose[4])+","+str(pose[5])+"]"+"\n" # pose vector 
        "track_conveyor_linear(pose, "+str(ticks)+")" + "\n"
        +"end" +"\n")) 

def set_payload_cog(s, cog=(1,0,0)):
    s.send(toBytes(
        "def myProg():"+"\n"
        "cog="+"[ "+str(cog[0])+","+str(cog[1])+","+str(cog[2])+" ]"+"\n"  # center of gravity 
        "set_payload_cog(cog)" + "\n"
        +"end" +"\n")) 

def set_payload_mass(s, mss=3):
    s.send(toBytes(
        "def myProg():"+"\n"
        "mss="+str(mss)+"\n"                                               # payload mass 
        "set_payload_mass(mss)" + "\n"
        +"end" +"\n"))

def set_tool_center_point(s, pose=(1,0.2,0.1,0,0,0)):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose="+"p["+str(pose[0])+","+str(pose[1])+","+str(pose[2])+","+str(pose[3])+","+str(pose[4])+","+str(pose[5])+"]"+"\n" # pose vector 
        "set_tcp(pose)" + "\n"
        +"end" +"\n")) 

def set_analog_out_type(s, chan, typ):
    s.send(toBytes(
        "def myProg():"+"\n"
        "chan="+str(chan)+"\n"                                               # channel
        "typ="+str(typ)+"\n"                                                 #  0: 4-20mA, 1: 0-10V        
        "set_analog_outputdomain(chan, typ)" + "\n"
        +"end" +"\n"))
        
# When alpha is 0, returns p_from. When alpha is 1, returns p_to. As alpha
# goes from 0 to 1, returns a pose going in a straight line (and geodetic
# orientation change) from p_from to p_to. If alpha is less than 0, returns
# a point before p_from on the line. If alpha is greater than 1, returns a
# pose after p_to on the line.
def interpolate_pose(s, pose_from, pose_to, alpha=0):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_from="+"p["+str(pose_from[0])+","+str(pose_from[1])+","+str(pose_from[2])+","+str(pose_from[3])+","+str(pose_from[4])+","+str(pose_from[5])+"]"+"\n" # pose vector from 2nd position
        "pose_to="+"p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n"        # pose vector final 3rd position        
        "interpolate_pose(pose_from, pose_to, "+str(alpha)+")" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# Return Value
# Distance between the two tool positions (without
# considering rotations)
def point_dist(s, pose_from, pose_to):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_from="+"p["+str(pose_from[0])+","+str(pose_from[1])+","+str(pose_from[2])+","+str(pose_from[3])+","+str(pose_from[4])+","+str(pose_from[5])+"]"+"\n" # pose vector from 2nd position
        "pose_to="+"p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n"        # pose vector final 3rd position        
        "point_dist(pose_from, pose_to)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# pose_from: tool pose
# qnear: list of joint positions (Optional)
# maxPositionError: the maximum allowed position
# error (Optional)
# maxOrientationError: the maximum allowed orientation
# error (Optional)
def get_inverse_kinetics(s, pose_from, qnear=[0.,3.14,1.57,.785,0,0]):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_tool="+"p["+str(pose_from[0])+","+str(pose_from[1])+","+str(pose_from[2])+","+str(pose_from[3])+","+str(pose_from[4])+","+str(pose_from[5])+"]"+"\n" # pose vector from 2nd position
        "qnear="+"["+str(qnear[0])+","+str(qnear[1])+","+str(qnear[2])+","+str(qnear[3])+","+str(qnear[4])+","+str(qnear[5])+"]"+"\n"        # pose vector final 3rd position        
        "get_inverse_kin(pose_tool, qnear, maxPositionError=1e-10, maxOrientationError=1e-10)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def pose_dist(s, pose_from, pose_to):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_from="+"p["+str(pose_from[0])+","+str(pose_from[1])+","+str(pose_from[2])+","+str(pose_from[3])+","+str(pose_from[4])+","+str(pose_from[5])+"]"+"\n" # pose vector from 2nd position
        "pose_to="+"p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n"        # pose vector final 3rd position        
        "pose_dist(pose_from, pose_to)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def pose_invert(s, pose_from):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_from="+"p["+str(pose_from[0])+","+str(pose_from[1])+","+str(pose_from[2])+","+str(pose_from[3])+","+str(pose_from[4])+","+str(pose_from[5])+"]"+"\n" # pose vector from 2nd position
        "pose_inv(pose_from)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
def pose_add(s, pose_from, pose_to):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_from="+"p["+str(pose_from[0])+","+str(pose_from[1])+","+str(pose_from[2])+","+str(pose_from[3])+","+str(pose_from[4])+","+str(pose_from[5])+"]"+"\n" # pose vector from 2nd position
        "pose_to="+"p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n"        # pose vector final 3rd position        
        "pose_add(pose_from, pose_to)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def pose_sub(s, pose_from, pose_to):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_from="+"p["+str(pose_from[0])+","+str(pose_from[1])+","+str(pose_from[2])+","+str(pose_from[3])+","+str(pose_from[4])+","+str(pose_from[5])+"]"+"\n" # pose vector from 2nd position
        "pose_to="+"p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n"        # pose vector final 3rd position        
        "pose_sub(pose_from, pose_to)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

def pose_trans(s, T_from, pose_to):
    s.send(toBytes(
        "def myProg():"+"\n"
        "T_from="+"p["+str(T_from[0])+","+str(T_from[1])+","+str(T_from[2])+","+str(T_from[3])+","+str(T_from[4])+","+str(T_from[5])+"]"+"\n" # pose vector from 2nd position
        "pose_to="+"p["+str(pose_to[0])+","+str(pose_to[1])+","+str(pose_to[2])+","+str(pose_to[3])+","+str(pose_to[4])+","+str(pose_to[5])+"]"+"\n"        # pose vector final 3rd position        
        "pose_trans(pose_from, pose_to)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv

# Wrench transformation
# Move the point of view of a wrench.
def wrench_transformation(s, pose_from, w_from):
    s.send(toBytes(
        "def myProg():"+"\n"
        "pose_from="+"p["+str(pose_from[0])+","+str(pose_from[1])+","+str(pose_from[2])+","+str(pose_from[3])+","+str(pose_from[4])+","+str(pose_from[5])+"]"+"\n" # The transformation to the new point of view (pose)
        "w_from="+"["+str(w_from[0])+","+str(w_from[1])+","+str(w_from[2])+","+str(w_from[3])+","+str(w_from[4])+","+str(w_from[5])+"]"+"\n"        # wrench to transform in list format [F_x, F_y, F_z,M_x, M_y, M_z]        
        "wrench_trans(T_from, w_from)" + "\n"
        +"end" +"\n")) 
    time.sleep(3)
    data_rcv = s.recv(1024)
    if (MY_PY == 3):
        data_rcv = data_rcv.decode('utf-8')
    return data_rcv
    
# ======================================================================   
# run your sequence here for funtional tests or robot automated running
# ======================================================================
def main():

    # first connect to the default robot then then other 2
    s1 = robot_connect()
    extend_arm_fully(s1)   
    move_100m_X(s1)
    s2 = robot_connect("192.168.250.111", 30003)
    move_100m_Y(s2)
    move_100m_Z(s1)
    move_100m_Y(s1)
    move_100m_Xback(s2)
    move_100m_Zback(s2)
    move_100m_Y(s1)
    tup = (0.1, 0.05, 0.1)                                 # move in x y z plane
    move_XYZ_dist_to_pose_array(s2, tup)
    pose_a = [0.76, 0.57, -0.67, 3.14, -1.57, 1.57]        # go to this pose of distance and angles
    move_XYZ_to_pose_array(s1, pose_a)   
    var = get_euromap_in(s1,3)
    print("robot1 euromap 3 state = ",var)
    var1 = get_euromap_in(s2,3)
    print("robot2 euromap 3 state = ",var1)
    s3 = robot_connect("192.168.250.112", 30004)
    # ==== find which of the 3 robots is closest to this pose
    pose_to = [0.16, 0.17, -0.17, 3.04, -1.07, 0.57] 
    my_pose = get_actual_tcp_pose(s1)
    d1 = pose_dist(s1, my_pose, pose_to)
    print("robot1 distance to pose = ",d1)
    my_pose = get_actual_tcp_pose(s2)
    d2 = pose_dist(s2, my_pose, pose_to)
    print("robot2 distance to pose = ",d1)
    my_pose = get_actual_tcp_pose(s3)
    d3 = pose_dist(s3, my_pose, pose_to)
    print("robot3 distance to pose = ",d1)
    robot_disconnect(s1)   
    robot_disconnect(s2)  
    robot_disconnect(s3)     
    
if __name__ == '__main__':
    main()

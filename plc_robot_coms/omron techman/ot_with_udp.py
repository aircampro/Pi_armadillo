#!/usr/bin/env python
#
# ref:- https://github.com/jvdtoorn/techmanpy/wiki/External-Script
# OMRON Techman Cobot https://www.tm-robot.com/en/tm5-900/
#
import asyncio
import sys
import techmanpy
import csv
import time
O_TM_IP='169.254.130.10'

# move to joint angles 
async def tm_move(poz=[10, -10, 10, -10, 10, -10], SP=0.10, TI=200, rip=O_TM_IP):
    async with techmanpy.connect_sct(robot_ip=rip) as conn:
        await conn.move_to_joint_angles_ptp(poz, SP, TI)

def tm_robot_move(POZZ, SP, TI, rip=O_TM_IP):
    asyncio.run(tm_move(POZZ, SP, TI, rip))

async def tm_move_line(poz=[10, 20, 4, 0, 0, 45] |, SP=0.10, TI=200, rip=O_TM_IP):
    async with techmanpy.connect_sct(robot_ip=rip) as conn:
        await conn.move_to_point_line(poz, SP, TI)

# move to point line 
def tm_robot_move_line(POZZ, SP, TI, rip=O_TM_IP):
    asyncio.run(tm_move_line(POZZ, SP, TI, rip))

def save_dictionary_to_csv(params_dict):
    with open("robot.csv", 'w') as file:
        writer = csv.writer(file)
        for parameter, value in params_dict.items():
            writer.writerow([parameter, value])

# listen and save to csv
async def tm_listen(rip=O_TM_IP):
    async with techmanpy.connect_svr(robot_ip=rip) as conn:
        conn.add_broadcast_callback(save_dictionary_to_csv)
        await conn.keep_alive()
        
# move to a hardcoded position
#
async def move_home(ip):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_joint_angles_ptp([10, -10, 10, -10, 10, -10], 0.10, 200)
   
# To set the tool center point (TCP) to an existing configuration:
#
async def set_tool_centre_point(ip, tcp, weight, intertia):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.set_tcp(tcp, weight, inertia)

def tm_set_tcp(ip, tcp, weight, intertia):
    asyncio.run(set_tool_centre_point(ip, tcp, weight, intertia))      

#tcp: string | mandatory arg
#Identifier of a tool center point in TMFlow
#weight: float (kg) | optional kwarg, default = None
#The weight of the tool
#inertia: [IX, IY, IZ, X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | optional kwarg, default = None
#Tool's moment of inertia and its frame of reference

async def set_load_weight(ip, weight):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.set_load_weight(weight)

def tm_set_load_weight(ip, weight):
    asyncio.run(set_load_weight(ip, weight))
#weight: float (kg)

# To set the motion frame of reference (base) to a custom configuration:
async def set_base(ip, b):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.set_base(b)

def tm_set_base(ip, b=[10, 10, 10, 0, 0, 0]):
    asyncio.run(set_base(ip, b))
 
#base: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#can also be base: string | mandatory arg
#Identifier of a base in TMFlow
#Frame of reference which will be set as the base
   
#To move to a relative point offset:
async def move_to_a_relative_point_offset(ip, relative_point_goal, speed_perc, acceleration_duration, blending_perc, relative_to_tcp=False, use_precise_positioning=False):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_relative_point_ptp(relative_point_goal, speed_perc, acceleration_duration, [blending_perc, relative_to_tcp, use_precise_positioning])

def tm_move_to_a_relative_point_offset(ip, relative_point_goal, speed_perc, acceleration_duration, blending_perc, relative_to_tcp=False, use_precise_positioning=False):
    asyncio.run(move_to_a_relative_point_offset(ip, relative_point_goal, speed_perc, acceleration_duration, blending_perc, relative_to_tcp, use_precise_positioning))
    
#relative_point_goal: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#Relative motion goal expressed in point and rotation offset
#speed_perc: float (%) | mandatory arg
#The speed percentage at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending_perc: float (%) | optional kwarg, default = 0.0
#The blending percentage for the motion
#relative_to_tcp: bool | optional kwarg, default = False
#Move relative to the TCP. If not enabled, the active base will be used
#use_precise_positioning: bool | optional kwarg, default = False
#Use precise positioning. This will result in a more accurate pose, but can cause more stress to the joints

#To move to absolute joint angles:
async def move_to_a_absolute_joint_angles(ip, joint_angles_goal, speed_perc, acceleration_duration, blending_perc, use_precise_positioning=False):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_joint_angles_ptp(joint_angles_goal, speed_perc, acceleration_duration, [blending_perc, use_precise_positioning])

def tm_move_to_a_absolute_joint_angles(ip, joint_angles_goal, speed_perc, acceleration_duration, blending_perc, use_precise_positioning=False):
    asyncio.run(move_to_a_absolute_joint_angles(ip, joint_angles_goal, speed_perc, acceleration_duration, blending_perc, use_precise_positioning))

#joint_angles_goal: [J1 (deg), J2 (deg), J3 (deg), J4 (deg), J5 (deg), J6 (deg)] | mandatory arg
#Motion goal expressed in joint angles
#speed_perc: float (%) | mandatory arg
#The speed percentage at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending_perc: float (%) | optional kwarg, default = 0.0
#The blending percentage for the motion
#use_precise_positioning: bool | optional kwarg, default = False
#Use precise positioning. This will result in a more accurate pose, but can cause more stress to the joints

#To move to relative joint angles offset:
async def move_to_relative_joint_angles_ptp(ip, relative_joint_angles_goal, speed_perc, acceleration_duration, blending_perc, use_precise_positioning=False):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_relative_joint_angles_ptp(relative_joint_angles_goal, speed_perc, acceleration_duration, [blending_perc, use_precise_positioning])

def tm_move_to_relative_joint_angles_ptp(ip, relative_joint_angles_goal, speed_perc, acceleration_duration, blending_perc, use_precise_positioning=False):
    asyncio.run(move_to_relative_joint_angles_ptp(ip, relative_joint_angles_goal, speed_perc, acceleration_duration, blending_perc, use_precise_positioning))
    
#relative_joint_angles_goal: [J1 (deg), J2 (deg), J3 (deg), J4 (deg), J5 (deg), J6 (deg)] | mandatory arg
#Relative motion goal expressed in joint angles offset
#speed_perc: float (%) | mandatory arg
#The speed percentage at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending_perc: float (%) | optional kwarg, default = 0.0
#The blending percentage for the motion
#use_precise_positioning: bool | optional kwarg, default = False
#Use precise positioning. This will result in a more accurate pose, but can cause more stress to the joints

#To move to a relative path offset:

async def move_to_a_relative_path_offset(ip, relative_point_goal, velocity, acceleration_duration, blending_perc, relative_to_tcp=False):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_relative_point_path(relative_point_goal, velocity, acceleration_duration, [blending_perc, relative_to_tcp])

def tm_move_to_a_relative_path_offset(ip, relative_point_goal, velocity, acceleration_duration, blending_perc, relative_to_tcp=False):
    asyncio.run(move_to_a_relative_path_offset(ip, relative_point_goal, velocity, acceleration_duration, blending_perc, relative_to_tcp))
    
#relative_point_goal: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#Relative motion goal expressed in point and rotation offset
#velocity: int (mm/s) | mandatory arg
#The speed at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending_perc: float (%) | optional kwarg, default = 0.0
#The blending percentage for the motion
#relative_to_tcp: bool | optional kwarg, default = False
#Move relative to the TCP. If not enabled, the active base will be used

#To move to an absolute point:

async def move_to_a_absolute_path(ip, tcp_point_goal, velocity, acceleration_duration, blending_perc):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_point_path(tcp_point_goal, velocity, acceleration_duration, [blending_perc])

def tm_move_to_a_absolute_path(ip, tcp_point_goal, velocity, acceleration_duration, blending_perc):
    asyncio.run(move_to_a_absolute_path(ip, tcp_point_goal, velocity, acceleration_duration, blending_perc))

#tcp_point_goal: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#Motion goal expressed in point and rotation
#velocity: int (mm/s) | mandatory arg
#The speed at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending_perc: float (%) | optional kwarg, default = 0.0
#The blending percentage for the motion

#To move to absolute joint angles:

async def move_to_joint_angles_path(ip, joint_angles_goal, velocity, acceleration_duration, blending_perc):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_joint_angles_path(joint_angles_goal, velocity, acceleration_duration, [blending_perc])

def tm_move_to_joint_angles_path(ip, joint_angles_goal, velocity, acceleration_duration, blending_perc):
    asyncio.run(move_to_joint_angles_path(ip, joint_angles_goal, velocity, acceleration_duration, blending_perc))
    
#joint_angles_goal: [J1 (deg), J2 (deg), J3 (deg), J4 (deg), J5 (deg), J6 (deg)] | mandatory arg
#Motion goal expressed in joint angles
#velocity: int (mm/s) | mandatory arg
#The speed at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending_perc: float (%) | optional kwarg, default = 0.0
#The blending percentage for the motion

#To move to relative joint angles offset:
async def move_to_relative_joint_angles_path(ip, relative_joint_angles_goal, velocity, acceleration_duration, blending_perc):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_relative_joint_angles_path(relative_joint_angles_goal, velocity, acceleration_duration, [blending_perc])

def tm_move_to_relative_joint_angles_path(ip, relative_joint_angles_goal, velocity, acceleration_duration, blending_perc):
    asyncio.run(move_to_relative_joint_angles_path(ip, relative_joint_angles_goal, velocity, acceleration_duration, blending_perc))
    
#relative_joint_angles_goal: [J1 (deg), J2 (deg), J3 (deg), J4 (deg), J5 (deg), J6 (deg)] | mandatory arg
#Relative motion goal expressed in joint angles offset
#velocity: int (mm/s) | mandatory arg
#The speed at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending_perc: float (%) | optional kwarg, default = 0.0
#The blending percentage for the motion

#Line motion
#To move to an absolute point:

async def move_to_point_line(ip, tcp_point_goal, speed, acceleration_duration, blending, speed_is_velocity=False, blending_is_radius=False, use_precise_positioning=False):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_point_line(tcp_point_goal, speed, acceleration_duration, [blending, speed_is_velocity, blending_is_radius, use_precise_positioning])

def tm_move_to_point_line(ip, tcp_point_goal, speed, acceleration_duration, blending, speed_is_velocity=False, blending_is_radius=False, use_precise_positioning=False):
    asyncio.run(move_to_point_line(ip, tcp_point_goal, speed, acceleration_duration, blending, speed_is_velocity, blending_is_radius, use_precise_positioning))
    
#tcp_point_goal: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#Motion goal expressed in point and rotation
#speed: float (%) | mandatory arg
#The speed at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending: float (%) | optional kwarg, default = 0.0
#The blending for the motion
#speed_is_velocity: bool | optional kwarg, default = False
#Describe the speed as a velocity. This changes the speed argument to int (mm/s)
#blending_is_radius: bool | optional kwarg, default = False
#Describe the blending as a radius. This changes the blending argument to int (deg)
#use_precise_positioning: bool | optional kwarg, default = False
#Use precise positioning. This will result in a more accurate pose, but can cause more stress to the joints

#To move to a relative point offset:
async def move_to_relative_point_line(ip, relative_point_goal, speed, acceleration_duration, blending, relative_to_tcp=False, speed_is_velocity=False, blending_is_radius=False, use_precise_positioning=False):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_to_relative_point_line(relative_point_goal, speed, acceleration_duration, [blending, relative_to_tcp, speed_is_velocity, blending_is_radius, use_precise_positioning])

def tm_move_to_point_line(ip, relative_point_goal, speed, acceleration_duration, blending, relative_to_tcp=False, speed_is_velocity=False, blending_is_radius=False, use_precise_positioning=False):
    asyncio.run(move_to_relative_point_line(ip, relative_point_goal, speed, acceleration_duration, blending, relative_to_tcp, speed_is_velocity, blending_is_radius, use_precise_positioning))
    
#relative_point_goal: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#Relative motion goal expressed in point and rotation offset
#speed: float (%) | mandatory arg
#The speed at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#blending: float (%) | optional kwarg, default = 0.0
#The blending for the motion
#relative_to_tcp: bool | optional kwarg, default = False
#Move relative to the TCP. If not enabled, the active base will be used
#speed_is_velocity: bool | optional kwarg, default = False
#Describe the speed as a velocity. This changes the speed argument to int (mm/s)
#blending_is_radius: bool | optional kwarg, default = False
#Describe the blending as a radius. This changes the blending argument to int (deg)
#use_precise_positioning: bool | optional kwarg, default = False
#Use precise positioning. This will result in a more accurate pose, but can cause more stress to the joints

#Circle motion
#To move on a circular path:
async def move_on_circle(ip, tcp_point_1, tcp_point_2, speed, acceleration_duration, arc_angle, blending_perc, speed_is_velocity=False, use_precise_positioning=False):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        await conn.move_on_circle(tcp_point_1, tcp_point_2, speed, acceleration_duration, [arc_angle, blending_perc, speed_is_velocity, use_precise_positioning])

def tm_move_on_circle(ip, tcp_point_1, tcp_point_2, speed, acceleration_duration, arc_angle, blending_perc, speed_is_velocity=False, use_precise_positioning=False):
    asyncio.run(move_on_circle(ip, tcp_point_1, tcp_point_2, speed, acceleration_duration, arc_angle, blending_perc, speed_is_velocity, use_precise_positioning))
    
#tcp_point_1: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#Motion keypoint expressed in point and rotation
#tcp_point_2: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#Motion keypoint expressed in point and rotation
#speed: float (%) | mandatory arg
#The speed at which the motion should maximally occur
#acceleration_duration: int (ms) | mandatory arg
#The duration for accelerating to the maximum speed
#arc_angle: int (deg) | optional kwarg, default = 0
#The arc angle on which the TCP will keep the same pose and move from current point to the assigned arc angle via the given point and end point. If this value is 0, the TCP will move from current point and pose to end point and pose via the point on arc with linear interpolation on pose
#blending_perc: float (%) | optional kwarg, default = 0.0
#The blending for the motion
#speed_is_velocity: bool | optional kwarg, default = False
#Describe the speed as a velocity. This changes the speed argument to int (mm/s)
#use_precise_positioning: bool | optional kwarg, default = False
#Use precise positioning. This will result in a more accurate pose, but can cause more stress to the joints

async def pvt_point_transation(ip, tcp_point_goal, tcp_point_velocities_goal, duration):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        trsct = conn.start_transaction()
        trsct.enter_point_pvt_mode()        
        trsct.add_pvt_point(tcp_point_goal, tcp_point_velocities_goal, duration)
        trsct.exit_pvt_mode()
        await trsct.submit()    

def tm_pvt_point_transation(ip, tcp_point_goal, tcp_point_velocities_goal, duration):
    asyncio.run(pvt_point_transation(ip, tcp_point_goal, tcp_point_velocities_goal, duration))

#tcp_point_goal: [X (mm), Y (mm), Z (mm), RX (deg), RY (deg), RZ (deg)] | mandatory arg
#Motion goal expressed in point and rotation
#tcp_point_velocities_goal: [VX (mm/s), VY (mm/s), VZ (mm/s), VRX (deg/s), VRY (deg/s), VRZ (deg/s)] | mandatory arg
#Motion velocity goal expressed in point and rotation
#duration: float (s) | mandatory arg
#Intended duration of the motion

async def pvt_joint_transation(ip, joint_angles_goal, joint_angle_velocities_goal, duration):
    async with techmanpy.connect_sct(robot_ip=ip) as conn:
        trsct = conn.start_transaction()
        trsct.enter_point_pvt_mode()        
        trsct.add_pvt_joint_angles(joint_angles_goal, joint_angle_velocities_goal, duration)
        trsct.exit_pvt_mode()
        await trsct.submit()    

def tm_pvt_joint_transation(ip, joint_angles_goal, joint_angle_velocities_goal, duration):
    asyncio.run(pvt_joint_transation(ip, joint_angles_goal, joint_angle_velocities_goal, duration))

#joint_angles_goal: [J1 (deg), J2 (deg), J3 (deg), J4 (deg), J5 (deg), J6 (deg)] | mandatory arg
#Motion goal expressed in joint angles
#joint_angle_velocities_goal: [VJ1 (deg/s), VJ2 (deg/s), VJ3 (deg/s), VJ4 (deg/s), VJ5 (deg/s), VJ6 (deg/s)] | mandatory arg
#Motion velocity goal expressed in joint angles
#duration: float (s) | mandatory arg
#Intended duration of the motion

# =====================================================================================================================================

cur_status = '0'

# unit actions for each key press on the remote UDP console
#
def go_right(poz=[10, 0, 0, 0, 0, 0] |, SP=0.10, TI=200, rip=O_TM_IP):
    tm_move_line(poz, SP, TI, rip)
def go_left(poz=[-10, 0, 0, 0, 0, 0] |, SP=0.10, TI=200, rip=O_TM_IP):
    tm_move_line(poz, SP, TI, rip)
def go_fwd(poz=[0, 10, 0, 0, 0, 0] |, SP=0.10, TI=200, rip=O_TM_IP):
    tm_move_line(poz, SP, TI, rip)
def go_back(poz=[0, -10, 0, 0, 0, 0] |, SP=0.10, TI=200, rip=O_TM_IP):
    tm_move_line(poz, SP, TI, rip)
def go_up(poz=[0, 0, 10, 0, 0, 0] |, SP=0.10, TI=200, rip=O_TM_IP):
    tm_move_line(poz, SP, TI, rip)
def go_dn(poz=[0, 0, -10, 0, 0, 0] |, SP=0.10, TI=200, rip=O_TM_IP):
    tm_move_line(poz, SP, TI, rip)
def go_zero(poz=[0, 0, 0, 0, 0, 0] |, SP=0.10, TI=200, rip=O_TM_IP):
    tm_move_line(poz, SP, TI, rip)
def do_circle(ip=O_TM_IP, tcp_point_1=[10, 10, 0, 0, 0, 0], tcp_point_2=[20, 20, 20, 0, 90, 0], speed=0.10, acceleration_duration=100, arc_angle=45, blending_perc=20, speed_is_velocity=False, use_precise_positioning=False))
    tm_move_on_circle(ip=O_TM_IP, tcp_point_1, tcp_point_2, speed, acceleration_duration, arc_angle, blending_perc, speed_is_velocity, use_precise_positioning) 
def do_joint_angles(ip, joint_angles_goal=[90, 40, 20, 10, -60, -70], joint_angle_velocities_goal=[1, 2, 0.4, 0.10, 1, 1], duration=300):
    tm_pvt_joint_transation(ip, joint_angles_goal, joint_angle_velocities_goal, duration)
def rst_joint_angles(ip, joint_angles_goal=[0, 0, 0, 0, 0, 0], joint_angle_velocities_goal=[2, 2, 2, 2, 2, 2], duration=300):
    tm_pvt_joint_transation(ip, joint_angles_goal, joint_angle_velocities_goal, duration)
    
# routines which procees the UDP requests
#    
def perform_chosen_action():
    if cur_status == 'R':
        go_right()
    elif cur_status == 'L':
        go_left()
    elif cur_status == 'A':
        go_fwd()
    elif cur_status == 'B':
        go_back()
    elif cur_status == 'U':
        go_up()
    elif cur_status == 'D':
        go_dn()
    elif cur_status == 'S':
        go_zero()
    elif cur_status == 'C':
        do_circle()
    elif cut_status == 'J':
        do_joint_angles()
    elif cut_status == 'P':
        rst_joint_angles()
        
# threads which interact with remote GUI on UDP port 8888 (by default)
#  
# Download an Android or iOS UDP sending/receiving APP. You can use any UDP sending/receiving APP. 
# we use a free APP called “UDP Sender/receiver” in Google Play Store. 
# If you are using iPhone, you can search UPD sender to get similar APP.
# After opening UPD Sender/Receiver APP, you will following UI:
#
RUN_ALL = True     
def conn_remote_udp(ipa=O_TM_IP, port=8888):
    UDP_IP = ipa
    UDP_PORT = port

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)                  # UDP Socket to remote control the craft
        sock.bind((UDP_IP, UDP_PORT))                                            # bind the socket
    except socket.error:
        print("failed to open socket")
        sock = None
    return sock
    
def read_remote_udp(sock):    
    run_f = True
    while run_f==True:
        print(cur_status)
        sock.settimeout(0.1)                                                 # set socket timeout
        try:
            data, addr = sock.recvfrom(1024)                                 # buffer size is 1024 bytes
        except socket.timeout:
            perform_chosen_action()                                          # read the command & do it
            continue
        except socket.error:
            try:
                sock.close()
            except socket.error:
                print("failed to close the socket")
            run_f = False            
        if data == b'R':
            cur_status = 'R'
        elif data == b'L':
            cur_status = 'L'
        elif data == b'A':
            cur_status = 'A'
        elif data == b'B':
            cur_status = 'B'
        elif data == b'U':
            cur_status = 'U'
        elif data == b'D':
            cur_status = 'D'
        elif data == b'S':
            cur_status = 'S'
        elif data == b'J':
            cur_status = 'J'
        elif data == b'P':
            cur_status = 'P'
        elif data == b'C':
            cur_status = 'C'
        elif data == b'X':
            cur_status = 'S'
            try:
                sock.close()
            except socket.error:
                print("failed to close the socket")
            run_f = False
        elif data == b'Z':
            cur_status = 'S'
            perform_chosen_action() 
            RUN_ALL = False
            
if __name__ == "__main__":

    while RUN_ALL==True:
        sockh = conn_remote_udp()                                           # bind UDP listener
        if not sockh == None:
            read_remote_udp(sockh)                                          # listen on the socket

    try:
        sockh.close()
    except socket.error:
        print("failed to close the socket")        
    del sockh
    

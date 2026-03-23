#! /usr/bin/env python3
#
# https://docs.hello-robot.com/0.3/python/moving/
#
# Example of how to use the API to control hello robot stretch
#
import stretch_body.robot
import time
import numpy as np
from argparse import ArgumentParser

# function to use trajectory (must be done for smothness) to draw a circle
def draw_circle_trajectory(r, n, diameter_m=0.2, arm_init = 5, lift_init = 10):
    t = np.linspace(0, 2*np.pi, n, endpoint=True)
    x = (diameter_m / 2) * np.cos(t) + arm_init
    y = (diameter_m / 2) * np.sin(t) + lift_init
    circle_mat = np.c_[x, y]                                  # create array from x,y points
    time_dt = 15 / n
    for i in range(n):
        pt = circle_mat[i]
        pt_t = i * time_dt
        r.arm.trajectory.add(t_s=pt_t, x_m=pt[0])
        r.lift.trajectory.add(t_s=pt_t, x_m=pt[1])
    r.follow_trajectory()
    time.sleep(n * time_dt + 0.5)

if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument('arm_init', help='arm init pos')
    parser.add_argument('lift_init', help='lift init pos')  
    parser.add_argument('wrist_yaw', help='wrist joint yaw')
    parser.add_argument('wrist_pitch', help='wrist joint pitch')
    parser.add_argument('wrist_roll', help='wrist joint roll')   
    parser.add_argument('arm', help='arm move')
    parser.add_argument('lift', help='lift move')
    parser.add_argument('head', help='head move')   
    parser.add_argument('basem', help='base move')
    parser.add_argument('basea', help='base angle')    
    parser.add_argument('dia', help='circle diameter')  
    args = parser.parse_args();
 
    r = stretch_body.robot.Robot()
    did_startup = r.startup()
    print(f'Robot connected to hardware: {did_startup}')

    is_homed = r.is_homed()
    print(f'Robot is homed: {is_homed}')

    # set each part to move then execute it
    r.arm.move_to(args.arm)               # 0.2 meters
    r.lift.move_to(args.lift)             # 0.6 meters
    r.head.move_to(args.head)             # 0.1 meters
    r.push_command()
    r.wait_command()

    r.base.translate_by(args.basem)       # 0.2 meters
    r.push_command()
    r.wait_command()

    r.base.rotate_by(args.basea)          # 0.1 radians
    r.push_command()
    r.wait_command()

    print(f'Wrist joints: {r.end_of_arm.joints}')                    # ['wrist_yaw', 'wrist_pitch', 'wrist_roll', 'stretch_gripper']
    r.end_of_arm.move_to('wrist_yaw', args.wrist_yaw)
    r.end_of_arm.move_to('wrist_pitch', args.wrist_pitch)
    r.end_of_arm.move_to('wrist_roll', args.wrist_roll)
    r.push_command()
    # gripper : accepts values between -100 to 100, where the gripper is fully closed at -100 and fully open at 100.
    #
    # grab the pen
    #
    r.end_of_arm.move_to('stretch_gripper', 100)
    r.push_command()
    r.wait_command()

    time.sleep(2)
    r.end_of_arm.move_to('stretch_gripper', 0)
    r.push_command()
    r.wait_command()

    time.sleep(2)
    r.end_of_arm.move_to('stretch_gripper', -100)
    r.push_command()
    r.wait_command()
    
    draw_circle_trajectory(r, 100, args.dia, args.arm_init, args.lift_init)
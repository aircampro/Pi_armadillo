#!/usr/bin/env python
#
# These ROS examples for joint and gripper control are based on this part of this book https://github.com/rt-net/AI-Robot-Book-chapter6/blob/master/crane_plus_commander/crane_plus_commander
# we are controlling the RT Crane and gripper via ROS publishers to the crane joints and the gripper open/close ammount 
#
#
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading

# from our libraries
from KBHit import KBHit
from kinematics_funcs import gripper_in_range, joint_in_range, kinematic_action, forward_kinematics, to_gripper_ratio, inverse_kinematics, from_gripper_ratio

# Node that publishes directives to topics for CRANE+ V2
class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        self.gripper_names = [
            'crane_plus_joint_hand']
        self.publisher_joint = self.create_publisher(JointTrajectory, 'crane_plus_arm_controller/joint_trajectory', 10)
        self.publisher_gripper = self.create_publisher(JointTrajectory,'crane_plus_gripper_controller/joint_trajectory', 10)
        # set up a range of pre-programmed poses to be commanded 
        self.poses = {}
        self.poses['zeros'] = [0, 0, 0, 0]
        self.poses['ones'] = [1, 1, 1, 1]
        self.poses['home'] = [0.0, -1.16, -2.01, -0.73]
        self.poses['carry'] = [-0.00, -1.37, -2.52, 1.17]
        
    # publish joint
    def publish_joint(self, q, time):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        msg.points[0].time_from_start = Duration(seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_joint.publish(msg)

    # publish gripper
    def publish_gripper(self, gripper, time):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['crane_plus_joint_hand']
        msg.points = [JointTrajectoryPoint()]
        msg.points[0].positions = [float(gripper)]
        msg.points[0].time_from_start = Duration(seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.publisher_gripper.publish(msg)

def main():
    # ROS init
    rclpy.init()

    # create the commander object
    commander = Commander()

    # Run rclpy.spin() in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(commander,))
    thread.start()

    # sleep for 1 sec.
    time.sleep(1.0)

    # set each joint and the gripper to 0
    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = 0
    dt = 5
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    time.sleep(dt)
    
    # go to pre-determined pose 
    commander.publish_joint(commander.poses['home'], dt)    
    
    # create keyboard input control object
    kb = KBHit()

    print('1, 2, 3, 4, 5, 6, 7, 8, 9, 0 increment/decrement joints + gripper by 0.1 and set')
    print('q w e r t y u i o p increment/decrement  joints + gripper by inc ammount and set')
    print('a s d increment/decrement the value of the inc ammount')
    print('z x c v b n m , . / increment/decrement  joints + gripper and elbow up values for kinematics')
    print('l list values for kinematics')
    print('k perform kinemtica with values in the above list')
    print('Esc exit')
    incv = 0.5

    # for inverse kinematics
    j = [0.0]*5
    eu = 0.0
    
    # Ctrl+c KeyboardInterrupt 
    try:
        while True:
            # remember last position
            joint_prev = joint.copy()
            gripper_prev = gripper

            # Target time sent with target joint values
            dt = 0.2

            # read keyboard commands
            if kb.kbhit():
                c = kb.getch()
                # set each value of movement as requested, each joint and then the gripper
                if c == '1':
                    joint[0] -= 0.1
                elif c == '2':
                    joint[0] += 0.1
                elif c == '3':
                    joint[1] -= 0.1
                elif c == '4':
                    joint[1] += 0.1
                elif c == '5':
                    joint[2] -= 0.1
                elif c == '6':
                    joint[2] += 0.1
                elif c == '7':
                    joint[3] -= 0.1
                elif c == '8':
                    joint[3] += 0.1
                elif c == '9':
                    gripper -= 0.1
                elif c == '0':
                    gripper += 0.1
                if c == 'q':
                    joint[0] -= incv
                elif c == 'w':
                    joint[0] += incv
                elif c == 'e':
                    joint[1] -= incv
                elif c == 'r':
                    joint[1] += incv
                elif c == 't':
                    joint[2] -= incv
                elif c == 'y':
                    joint[2] += incv
                elif c == 'u':
                    joint[3] -= incv
                elif c == 'i':
                    joint[3] += incv
                elif c == 'o':
                    gripper -= incv
                elif c == 'p':
                    gripper += incv
                elif c == 'a':
                    incv += 0.05
                elif c == 's':
                    incv -= 0.05
                elif c == 'd':
                    incv = 0.0
                # set up and perform the kinematics
                if c == 'z':
                    j[0] -= incv
                elif c == 'x':
                    j[0] += incv
                elif c == 'c':
                    j[1] -= incv
                elif c == 'v':
                    j[1] += incv
                elif c == 'b':
                    j[2] -= incv
                elif c == 'n':
                    j[2] += incv
                elif c == 'm':
                    j[3] -= incv
                elif c == ',':
                    j[3] += incv
                elif c == '.':
                    j[4] -= incv
                elif c == '/':
                    j[4] += incv
                elif c == '#':
                    eu -= incv
                elif c == ';':
                    eu += incv
                elif c == 'l':
                    print(f"values for kinematics joints={j} elbow_up={eu} button_inc{incv}")      # print the data to pass to the inverse kinematic function        
                elif c == 'k':  
                    joint, gripper = kinematic_action(j, eu)                                       # perfrom the inverse kenmatics with the data set-up above            
                elif c == ' ':                                                                     # means reset
                    joint = [0.0, 0.0, 0.0, 0.0]
                    gripper = 0
                    dt = 1.0
                elif ord(c) == 27:                                                                 # Esc means exit
                    break

                # check range constraints for movement
                if not all(joint_in_range(joint)):
                    print('joint out of range')
                    joint = joint_prev.copy()
                if not gripper_in_range(gripper):
                    print('gripper out of range')
                    gripper = gripper_prev

                # If we requested a joint movement or a gripper movement then publish this to ROS
                publish = False
                if joint != joint_prev:
                    print((f'joint: [{joint[0]:.2f}, {joint[1]:.2f}, ' f'{joint[2]:.2f}, {joint[3]:.2f}]'))
                    commander.publish_joint(joint, dt)
                    publish = True
                if gripper != gripper_prev:
                    print(f'gripper: {gripper:.2f}')
                    commander.publish_gripper(gripper, dt)
                    publish = True
                # slight pause after we publish the requested movement to the gripper/crane
                if publish:
                    time.sleep(dt)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass

    # try carry for 5 seconds
    dt = 5
    commander.publish_joint(commander.poses['carry'], dt)  
    time.sleep(dt)
    
    # stop and reset close gripper
    joint = [0.0, 0.0, 0.0, 0.0]
    gripper = 0
    commander.publish_joint(joint, dt)
    commander.publish_gripper(gripper, dt)

    rclpy.shutdown()
    print('robot controls completed')
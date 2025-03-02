#!/usr/bin/env python
#
# general kinematics functions
#
from math import pi, cos, sin, acos, atan2, sqrt

# define constraints
RAD = pi/180.0
JOINT_MIN = [ -150*RAD, -104.55*RAD, -146.48*RAD, -105.43*RAD]
JOINT_MAX = [ 150.0*RAD, 102.2*RAD, 140.62*RAD, 101.32*RAD]
GRIPPER_MIN = -40.62*RAD
GRIPPER_MAX = 38.27*RAD
# used in forward and reverse kinematics
L1 = 0.0456 + 0.026
L2 = 0.083
L3 = 0.0935
L4 = 0.045 + 0.045

def joint_in_range(joint):
    return list(map(lambda x, min, max: min <= x <= max, joint, JOINT_MIN, JOINT_MAX))

def gripper_in_range(gripper):
    return GRIPPER_MIN <= gripper <= GRIPPER_MAX

# constrain between xmin and xmax
def clip(x, xmin, xmax):
    r = x
    if x < xmin:
        r = xmin
    elif x > xmax:
        r = xmax
    return r

# constrain joint between JOINT_MIN, JOINT_MAX as above
def cliped_joint(joint):
    return list(map(clip, joint, JOINT_MIN, JOINT_MAX))

# constrain gripper between GRIPPER_MIN, GRIPPER_MAX as above
def cliped_gripper(gripper):
    return clip(gripper, GRIPPER_MIN, GRIPPER_MAX)

def normalize_angle(x):
    if x < -pi:
        x += 2*pi*int(-x/(2*pi)+0.5)
    elif x > pi:
        x -= 2*pi*int(x/(2*pi)+0.5)
    return x

def forward_kinematics(joint):
    [q1, q2, q3, q4] = joint
    # To make it easier to think, change the 0 angle and rotation direction of q2 and use it.
    q2 = pi/2 - q2
    r = L2*cos(q2) + L3*cos(q2+q3) + L4*cos(q2+q3+q4)
    x = r*cos(q1)
    y = r*sin(q1)
    z = L1 + L2*sin(q2) + L3*sin(q2+q3) + L4*sin(q2+q3+q4)
    pitch = q2 + q3 + q4
    return [x, y, z, pitch]

def inverse_kinematics(endtip, elbow_up):
    [x, y, z, pitch] = endtip
    r = sqrt(x**2 + y**2)
    if r == 0.0:
        return None  
    q1 = atan2(y, x)
    rr = r - L4*cos(pitch)
    zz = z - L4*sin(pitch)
    d = sqrt(rr**2 + (zz - L1)**2)
    cq3 = (d**2 - L2**2 - L3**2)/(2*L2*L3)
    if abs(cq3) > 1.0:
        return None  
    phi = atan2(zz - L1, rr)
    psi = acos((d**2 + L2**2 - L3**2)/(2*L2*d))
    # To make it easier to think, change the 0 angle and rotation direction of q2 and use it
    if elbow_up:
        q2 = normalize_angle(phi + psi)
        q3 = -acos(cq3)
    else:
        q2 = normalize_angle(phi - psi)
        q3 = acos(cq3)
    q4 = normalize_angle(pitch - q2 - q3)
    # Convert q2 to CRANE+ specifications
    q2 = pi/2 - q2
    return [q1, q2, q3, q4]

def to_gripper_ratio(gripper):
    ratio = (gripper - GRIPPER_MIN) / (GRIPPER_MAX - GRIPPER_MIN)
    return ratio

def from_gripper_ratio(ratio):
    gripper = GRIPPER_MIN + ratio * (GRIPPER_MAX - GRIPPER_MIN)
    return gripper

def calc_kinematics():
    kb = KBHit()
    while True:
        try:
            joint = [0.0]*4
            joint[0] = float(input('joint1: '))
            joint[1] = float(input('joint2: '))
            joint[2] = float(input('joint3: '))
            joint[3] = float(input('joint4: '))
            gripper = float(input('gripper: '))
        except ValueError:
            break
        endtip = forward_kinematics(joint)
        ratio = to_gripper_ratio(gripper)
        [x, y, z, pitch] = endtip
        print(f'x: {x}, y: {y}, z: {z}, pitch: {pitch}, ratio: {ratio}')
        try:
            elbow_up = bool(int(input('elbow_up: ')))
        except ValueError:
            break
        print(f'elobow_up: {elbow_up}')
        joint = inverse_kinematics(endtip, elbow_up)
        gripper = from_gripper_ratio(ratio)
        if joint is not None:
            print('joint:', joint, ' gripper:', gripper)
        else:
            print("error ocurred")
        if kb.kbhit():
            print("press ESC to exit")
            c = kb.getch()
            if ord(c) == 27:                           # ESC to exit the test
                break

# perform the kinemtics with the parameters passed to the function and return joints and gripper for publishing                
def kinematic_action(jpos, elbow_up_v):
    if len(jpos) >= 5:
        try:
            joint = [0.0]*4
            joint[0] = float(jpos[0])
            joint[1] = float(jpos[1])
            joint[2] = float(jpos[2])
            joint[3] = float(jpos[3])
            gripper = float(jpos[4])
        except ValueError:
            break
        endtip = forward_kinematics(joint)
        ratio = to_gripper_ratio(gripper)
        [x, y, z, pitch] = endtip
        print(f'x: {x}, y: {y}, z: {z}, pitch: {pitch}, ratio: {ratio}')
        try:
            elbow_up = bool(int(elbow_up_v))
        except ValueError:
            break
        print(f'elbow_up: {elbow_up}')
        joint = inverse_kinematics(endtip, elbow_up)
        gripper = from_gripper_ratio(ratio)
        if joint is not None:
            print('joint:', joint, ' gripper:', gripper)
        else:
            print("error ocurred")
        return joint, gripper
        
# set TRue to run a test with cTrl-C to close it
TESTING = False
if __name__ == '__main__':
    if TESTING == True:
        calc_kinematics()
	
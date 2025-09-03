#moto
#A Python library for controlling Yaskawa MOTOMAN robots with the MotoROS option.
#
#Installation
#pip3 install git+https://github.com/tingelst/moto.git --upgrade
#On the robot side, the moto library employs the ROS-Industrial robot driver found here: https://github.com/ros-industrial/motoman. Follow the official tutorial for installing the necessary files on the robot controller.
#
#Example
#The highest level API is defined in the Moto class.
#
from moto import Moto
# Connect to the robot controller with the defined ip address <robot_ip> and define the R1 control group with six degrees of freedom.

m = Moto(
    "<robot_ip>",
    [
        ControlGroupDefinition(
            groupid="R1",
            groupno=0,
            num_joints=6,
            joint_names=[
                "joint_1_s",
                "joint_2_l",
                "joint_3_u",
                "joint_4_r",
                "joint_5_b",
                "joint_6_t",
            ],
        ),
    ],
)
# If your robot system has multiple control groups, e.g. a positioner with two degrees of freedom, these can be defined as follows:
m2 = Moto(
    "<robot_ip>",
    [
        ControlGroupDefinition(
            groupid="R1",
            groupno=0,
            num_joints=6,
            joint_names=[
                "joint_1_s",
                "joint_2_l",
                "joint_3_u",
                "joint_4_r",
                "joint_5_b",
                "joint_6_t",
            ],
        ),
        ControlGroupDefinition(
            groupid="S1",
            groupno=1,
            num_joints=2,
            joint_names=[
                "joint_1",
                "joint_2",
            ],
        ),
    ],
)
# The system supports up to 4 control groups.

# Each control group can be accessed and introspected by name:
CTL_GRP="R1"
def yaskawaGetControlGroupPos(m,ctrlGrp):
    r1 = m.control_groups[ctrlGrp]
    print(r1.position)
    return r1.position

# Motion
# To be able to send trajectores, you must first start the trajectory mode:
def yaskawaStartTrajMode(m):
    m.motion.start_trajectory_mode()
    
# The API for sending trajectories is still under development. For now, to move joint 1 you can e.g. do:

def yaskawaStartTrajMode(m):
    robot_joint_feedback = m.state.joint_feedback_ex()
    return robot_joint_feedback

def yaskawaSetTraj(m,robot_joint_feedback):
    p0 = JointTrajPtFullEx(
        number_of_valid_groups=2,
        sequence=0,
        joint_traj_pt_data=[
            JointTrajPtExData(
                groupno=0,
                valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                time=0.0,
                pos=robot_joint_feedback.joint_feedback_data[0].pos,
                vel=[0.0] * 10,
                acc=[0.0] * 10,
            ),
            JointTrajPtExData(
                groupno=1,
                valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                time=0.0,
                pos=robot_joint_feedback.joint_feedback_data[1].pos,
                vel=[0.0] * 10,
                acc=[0.0] * 10,
            ),
        ],
    )

    p1 = JointTrajPtFullEx(
        number_of_valid_groups=2,
        sequence=1,
        joint_traj_pt_data=[
            JointTrajPtExData(
                groupno=0,
                valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                time=5.0,
                pos=np.deg2rad([10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                vel=[0.0] * 10,
                acc=[0.0] * 10,
            ),
            JointTrajPtExData(
                groupno=1,
                valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                time=5.0,
                pos=np.deg2rad([10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                vel=[0.0] * 10,
                acc=[0.0] * 10,
            ),
        ],
    )

    m.motion.send_joint_trajectory_point(p0) # Current position at time t=0.0
    m.motion.send_joint_trajectory_point(p1) # Desired position at time t=5.0
    
# ------------ IO ------------------
# You can read and write bits:
M_RD_ADDR=27010
M_WT_ADDR=27010
def yaskawa_io_read_bit(m,read_addr):
    return m.io.read_bit(read_addr)
    
M_WT_VAL=0
def yaskawa_io_write_bit(m,write_addr,val):    
    return m.io.write_bit(write_addr, val)
    
M_BYTE_ADR=1001
def yaskawa_io_read_byte(m,read_addr):    
    return m.io.read_group(read_addr)
    
M_WT_BYTE=67   
def yaskawa_io_write_byte(m,write_addr,val):      
    return m.io.write_group(write_addr, val)

# A python library to control KUKA iiwa robots, the 7R800 and the 14R820, from an external computer
# -*- coding: utf-8 -*-
#
from iiwaPy import iiwaPy
import math
import time

ST = time.time()
# returns the elapsed seconds since the start of the program
def getSecs():
   dt = time.time() - ST
   ms = dt * 1000.0
   return ms
   
kuka_ip='172.31.1.147'
#ip='localhost'
def connectKuka(ip=kuka_ip):
    iiwa=iiwaPy(ip)   
    iiwa.setBlueOn()
    time.sleep(2)
    iiwa.setBlueOff()
    return iiwa

def disconnectKuka(iiwa_h):
    iiwa_h.close()
    
def moveInitPosKuka(iiwa_handle):
    try:
        # Move to an initial position    
        initPos=[0,0,0,-math.pi/2,0,math.pi/2,0];
        initVel=[0.1]
        iiwa_handle.movePTPJointSpace(initPos,initVel)
    except:
        print('coulnt communicate with kuka robot')

def movePosKuka(iiwa_handle, pose, initVel=[0.1]):
    try:
        # Move to an initial position           
        iiwa_handle.movePTPJointSpace(pose,initVel)
    except:
        print('coulnt communicate with kuka robot')

def moveKukaExample(iiwa): 
    try:   
        counter=0
        index=0
        w=0.6
        theta=0
        interval= 2*3.14
        a=3.14/6
    
        jpos=iiwa.getJointsPos()
        jpos0_6=jpos[index]
        iiwa.realTime_startDirectServoJoints()
    
        t0=getSecs()
        t_0=getSecs()
        while theta<interval:
            theta=w*(getSecs()-t0)
            jpos[index]=jpos0_6-a*(1-math.cos(theta))
        
            if (getSecs()-t_0)>0.002:
                iiwa.sendJointsPositions(jpos)
                t_0=getSecs()
                counter=counter+1
            
            
        deltat= getSecs()-t0;
        iiwa.realTime_stopDirectServoJoints()

        # Move to an initial position    
        jPos=[math.pi/3,0,0,-math.pi/2,0,math.pi/2,0];
        vRel=[0.1]
        iiwa.movePTPJointSpace(jPos,vRel)
    
    except:
        print('an error occurred with kuka robot movement')
    
def KukaGetCartesianPos(iiwa):
    # Get current cartezian position
    cPos=iiwa.getEEFPos()
    print(cPos)
    return cPos
    
#move 30
move_val=30
def KukaMoveX(iiwa,x_val,velocity,plus=1):
    # Move ptp along x axis
    print('Moving on a line along the X axis')
    vel=[velocity]; # mm/sec
    print('With a linear velocity')
    print(vel)
    if (plus == 1):
        cPos[0]=cPos[0]+x_val
    else:
        cPos[0]=cPos[0]-x_val    
    iiwa.movePTPLineEEF(cPos,vel,plus=1)
    
def KukaMoveY(iiwa,y_val,velocity,plus=1):
    # Move ptp along y axis
    print('Moving on a line along the X axis')
    vel=[velocity]; # mm/sec
    print('With a linear velocity')
    print(vel)
    if (plus == 1):
        cPos[1]=cPos[1]+y_val
    else:
        cPos[1]=cPos[1]-y_val    
    iiwa.movePTPLineEEF(cPos,vel,plus=1)
    
def KukaMoveZ(iiwa,z_val,velocity):
    # Move ptp along z axis
    print('Moving on a line along the Z axis')
    vel=[velocity]; # mm/sec
    print('With a linear velocity')
    print(vel)
    if (plus == 1):
        cPos[2]=cPos[2]+z_val    
    else:
        cPos[2]=cPos[2]-z_val
    iiwa.movePTPLineEEF(cPos,vel)    

def KukaMoveArc(iiwa):
    # Move on an Arc in the XY plange
    # using functiom movePTPArcXY_AC
    theta=[1.98*math.pi]
    c=[cen[0],cen[1]]
    vel=[150]
    print('Moving on an incliend Arc parallel to XY plane')
    iiwa.movePTPArcXY_AC(theta,c,vel)

move_mm=50.0
def KukaMoveInMM(iiwa,move_mm,dir="x"):
    # Move EEF 50mm in X direction
    if dir == "x":
        Pos=[move_mm,0.0,0.0]
    elif dir == "y":
        Pos=[0.0,move_mm,0.0]
    elif dir == "yx":
        Pos=[move_mm,move_mm,0.0]
    elif dir == "yz":
        Pos=[0.0,move_mm,move_mm]
    elif dir == "z":
        Pos=[0.0,0.0,move_mm]
    elif dir == "zx":
        Pos=[move_mm,0.0,move_mm]
    elif dir == "zy":
        Pos=[0.0,move_mm,move_mm]
    elif dir == "xyz":
        Pos=[move_mm,move_mm,move_mm]
    else:
        print("valid options x y and z only")
        return 0
    iiwa.movePTPLineEefRelBase(Pos,vel)
    return 1
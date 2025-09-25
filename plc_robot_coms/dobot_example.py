# example to control Dobot
#
from serial.tools import list_ports
# https://github.com/sammydick22/pydobotplus/blob/master/pydobotplus/dobotplus.py
from pydobotplus import Dobot, CustomPosition
import time

# you can also use a yanl config file to choose the dobot's ports
import sys
import os
import yaml
def get_dobot_port(f="device2_port.yaml"):
    config_file = os.path.join(os.path.dirname(__file__), "..", "config", f)
    with open(config_file, "r") as file:
        config = yaml.safe_load(file)
    return config["device_port"]

# for additional messaging we need this bit neither class was complete
#
from enum import Enum
from pydobotplus.message import Message
# dobot communication message ID raw
class CommunicationProtocolIDs(Enum):

    GET_SET_DEVICE_SN = 0
    GET_SET_DEVICE_NAME = 1
    GET_POSE = 10
    RESET_POSE = 11
    GET_ALARMS_STATE = 20
    CLEAR_ALL_ALARMS_STATE = 21
    SET_GET_HOME_PARAMS = 30
    SET_HOME_CMD = 31
    SET_GET_HHTTRIG_MODE = 40
    SET_GET_HHTTRIG_OUTPUT_ENABLED = 41
    GET_HHTTRIG_OUTPUT = 42
    SET_GET_ARM_ORIENTATION = 50
    SET_GET_END_EFFECTOR_PARAMS = 60
    SET_GET_END_EFFECTOR_LAZER = 61
    SET_GET_END_EFFECTOR_SUCTION_CUP = 62
    SET_GET_END_EFFECTOR_GRIPPER = 63
    SET_GET_JOG_JOINT_PARAMS = 70
    SET_GET_JOG_COORDINATE_PARAMS = 71
    SET_GET_JOG_COMMON_PARAMS = 72
    SET_GET_PTP_JOINT_PARAMS = 80
    SET_GET_PTP_COORDINATE_PARAMS = 81
    SET_GET_PTP_JUMP_PARAMS = 82
    SET_GET_PTP_COMMON_PARAMS = 83
    SET_PTP_CMD = 84
    SET_CP_CMD = 91
    SET_GET_EIO = 131
    SET_QUEUED_CMD_START_EXEC = 240
    SET_QUEUED_CMD_STOP_EXEC = 241
    SET_QUEUED_CMD_CLEAR = 245
    GET_QUEUED_CMD_CURRENT_INDEX = 246

def set_jog_common_params(dobo, a=100, b=100):
    msg = Message()
    msg.id = CommunicationProtocolIDs.SET_GET_JOG_COMMON_PARAMS.value
    msg.ctrl = 0x03
    msg.params = bytearray([])
    msg.params.extend(bytearray(struct.pack('f', a)))
    msg.params.extend(bytearray(struct.pack('f', b)))
    return dobo._send_command(msg)

def set_jog_joint_params(dobo, v_x, v_y, v_z, v_r, a_x, a_y, a_z, a_r):
    msg = Message()
    msg.id = CommunicationProtocolIDs.SET_GET_JOG_JOINT_PARAMS.value
    msg.ctrl = 0x03
    msg.params = bytearray([])
    msg.params.extend(bytearray(struct.pack('f', v_x)))
    msg.params.extend(bytearray(struct.pack('f', v_y)))
    msg.params.extend(bytearray(struct.pack('f', v_z)))
    msg.params.extend(bytearray(struct.pack('f', v_r)))
    msg.params.extend(bytearray(struct.pack('f', a_x)))
    msg.params.extend(bytearray(struct.pack('f', a_y)))
    msg.params.extend(bytearray(struct.pack('f', a_z)))
    msg.params.extend(bytearray(struct.pack('f', a_r)))
    return dobo._send_command(msg)

def set_end_effector_params(dobo, a=59.7, b=0, c=0, d=0):
    msg = Message()
    msg.id = CommunicationProtocolIDs.SET_GET_END_EFFECTOR_PARAMS.value
    msg.ctrl = 0x03
    msg.params = bytearray([])
    msg.params.extend(bytearray(struct.pack('f', a)))
    msg.params.extend(bytearray(struct.pack('f', b)))
    msg.params.extend(bytearray(struct.pack('f', c)))
    msg.params.extend(bytearray(struct.pack('f', d)))
    return dobo._send_command(msg)

if __name__ == "__main__":

    # the first dobot is using the first avai;able port
    available_ports = list_ports.comports()
    print(f'available ports: {[x.device for x in available_ports]}')
    port = available_ports[2].device

    dobot1 = Dobot(port=port)

    # set-up dobot parameters
    dobot1._set_jog_coordinate_params(50, 50, 50, 50, 50, 50, 50, 50)
    set_jog_common_params(dobot1, 100, 100)
    set_jog_joint_params(dobot1, 50, 50, 50, 50, 50, 50, 50, 50)
    dobot1._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
    dobot1._set_ptp_coordinate_params(velocity=200, acceleration=200)
    dobot1._set_ptp_jump_params(20, 100)
    dobot1._set_ptp_common_params(velocity=30, acceleration=30)    
    set_end_effector_params(dobot1)

    # get alarms
    al = dobot1.get_alarms()
    print("alarms", al) 
    # clear alarms
    dobot1.clear_alarms()
    
    # Create a custom position
    pos1 = CustomPosition(x=200, y=50, z=50)

    # Move using direct coordinates
    dobot1.move_to(x=200, y=50, z=50)

    # Move using custom position
    dobot1.move_to(position=pos1)

    # add movement to the current pose you can also use .move_rel method
    pose_p = dobot1.get_pose()
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    r = pose.position.r
    j1 = pose.joints.j1
    j2 = pose.joints.j2
    j3 = pose.joints.j3
    j4 = pose.joints.j4
    print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')

    # move 20 in X plane
    dobot1.move_to(x + 20, y, z, r, wait=False)
    # now move 20 in Y plane
    dobot1.move_to(x, y+20, z, r, wait=True)                                  # we wait until this movement is done before continuing
    time.sleep(2)
    # now move 20 in Z plane keeping the Y
    dobot1.move_to(x, y+20, z+20, r, wait=True)                               # we wait until this movement is done before continuing
    
    # Control the conveyor belt
    dobot1.conveyor_belt(speed=0.5, direction=1)
    dobot1.conveyor_belt_distance(speed_mm_per_sec=50, distance_mm=200, direction=1)

    # suction cup
    dobot1.suck(True)
    time.sleep(3)
    dobot1.suck(False)

    # gripper
    dobot1.grip(True)
    time.sleep(3)
    dobot1.grip(False)

    # laser controls
    dobot1.laze(255,True)
    time.sleep(3)
    dobot1.laze(100,True)
    time.sleep(3)
    dobot1.laze(0,False)

    # set IR on port GP1
    dobot1.set_ir(True, port=dobot1.PORT_GP1)
    time.sleep(3)
    dobot1.set_ir(False, port=dobot1.PORT_GP1)
    
    dobot1.close()

    # 2nd dobot port from yaml file
    p = get_dobot_port()
    dobot2 = Dobot(port=p)
    dobot2.set_color(True)
    r, g, b = dobot2.get_color()  
    # move by 10 10 10
    dobot2.move_rel(10, 10, 10)  
    dobot2.set_hht_trig_output(True)
    time.sleep(2)
    dobot2.set_hht_trig_output(False)   
    dobot2.close()
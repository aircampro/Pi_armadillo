# LoRa Gateway Code which receives a packet from OPC over LoRa and commands the Futaba Drive and Kuka Robot
#
import socket
import struct
from network import LoRa
import sys
import time

# kuka robot class
from iiwaPy import iiwaPy
import math

# futaba drive class
from "../plc_robot_comms/FutabaRS_Serial_Servo" import Futaba_RS_Servo

# A basic package header, B: 1 byte for the deviceId, B: 1 byte for the pkg size, %ds: Formatted string for string
_LORA_PKG_FORMAT = "!BB%ds"
# A basic ack package, B: 1 byte for the deviceId, B: 1 byte for the pkg size, B: 1 byte for the Ok (200) or error messages
_LORA_PKG_ACK_FORMAT = "BBB"

# for kuka robot
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

if __name__ == "__main__": 

    # Open a LoRa Socket, use rx_iq to avoid listening to our own messages
    # Please pick the region that matches where you are using the device:
    # Asia = LoRa.AS923
    # Australia = LoRa.AU915
    # Europe = LoRa.EU868
    # United States = LoRa.US915
    if len(sys.argv) > 1:
        if not str(sys.argv[1]).lower.find("asia") == -1:
            country = LoRa.AS923
        elif not str(sys.argv[1]).lower.find("aus") == -1:
            country = LoRa.AU915
        elif not str(sys.argv[1]).lower.find("eur") == -1:
            country = LoRa.EU868
        elif not str(sys.argv[1]).lower.find("us") == -1:
            country = LoRa.US915
        else:
            print("invalid country defaulted 868Mhz europe valid opts asia, aus, eur, us")
            country = LoRa.EU868		
    else:
        country = LoRa.EU868
    lora = LoRa(mode=LoRa.LORA, rx_iq=True, region=country)
    lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lora_sock.setblocking(False)

    # create kuka robot instance (connected on ethernet)
    kuka_handle = connectKuka()
    moveInitPosKuka(kuka_handle)

    # create futaba servo drive instance (connected on serial)
    frs_servo = FutabaRS_Servo()
    frs_servo.open_port()
    frs_servo.torque_on()
	frs_servo.servo_reset()

    while (True):
        recv_pkg = lora_sock.recv(512)
        if (len(recv_pkg) > 2):
            recv_pkg_len = recv_pkg[1]
            device_id, pkg_len, msg = struct.unpack(_LORA_PKG_FORMAT % recv_pkg_len, recv_pkg)
            rcv_data = msg.decode('utf-8')
            print('Device: %d - Pkg:  %s' % (device_id, rcv_data))
            pose_elements = rcv_data.split(":")[1].split(",")                                   # parse the received message
            pose_elements = rcv_vals[:-2]                                                       # pose to robot 
            drive_angle = recv_vals[-2:]                                                        # position and time to the drive
            movePosKuka(kuka_handle, pose_elements)
            id, ackv = frs_servo.target_position(drive_angle[0], drive_angle[1]) 
            if ackv == 'ACK':                                                                   # if we dont ack dont ACK back       		
                ack_pkg = struct.pack(_LORA_PKG_ACK_FORMAT, device_id, 1, 200)                  # now reply with an ACK	
                lora_sock.send(ack_pkg)
    frs_servo.close_port()
    disconnectKuka(kuka_handle)  
    lora_sock.close()    
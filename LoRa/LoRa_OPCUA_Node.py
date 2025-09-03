# The _LORA_PKG_FORMAT is used as a method of identifying the different devices within a network. 
# The _LORA_PKG_ACK_FORMAT is a simple ack package as a response to the nodes package.
#
# LoRa Node Example reads opc point and sends it over LoRa to a Futaba Drive and Kuka Robot which are connected to the LoRa G/W
#
import os
import socket
import time
import struct
from network import LoRa
from opcua import Client,ua        # For OPC Client connection
import sys

# A basic package header, B: 1 byte for the deviceId, B: 1 byte for the pkg size
_LORA_PKG_FORMAT = "BB%ds"
_LORA_PKG_ACK_FORMAT = "BBB"
DEVICE_ID = 0x01

# OPC endpoint definition
ENDPOINT=”opc.tcp://DESKTOP-7EN1MN2:4840″
# define the node point containing the item which has the pose data for the robot
NODES=’ns=4;s=GVL2RoboDK.kuka1.pos.ActPos’
# define the node point containing the item which has the position and time for the drive
NODET=’ns=4;s=GVL2RoboDK.futaba1.pos.ActPos’

if __name__ == "__main__":  
    # connect client to the endpoint
    client=Client(ENDPOINT)
    client.connect()
    print(‘Connected to OPCUA server..’)
    # get the nodes from the opc server
    NodeFromServer=client.get_node(NODES)
    NodeFromServer1=client.get_node(NODET)
    client.load_type_definitions()    
    # Open a Lora Socket, use tx_iq to avoid listening to our own messages
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
    lora = LoRa(mode=LoRa.LORA, tx_iq=True, region=country)
    lora_sock = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
    lora_sock.setblocking(False)

    while(True):
        Actpos=NodeFromServer.get_value()                       # kuka pose
        DrivePos=NodeFromServer1.get_value()                  # robotis drive position and time
        pose_val_msg = f"POSE : {Actpos.x} , {Actpos.y} , {Actpos.z} , {Actpos.a} , {Actpos.b} , {Actpos.c}, {DrivePos.x}, {DrivePos.y} "
        pose_val_msg = pose_val_msg.encode('utf-8')
        pkg = struct.pack(_LORA_PKG_FORMAT % len(pose_val_msg), DEVICE_ID, len(pose_val_msg), pose_val_msg)
        lora_sock.send(pkg)

        # Wait for the response from the gateway. no ACK and timeout then resend it.
        waiting_ack = True
        time_s = time.time()
        LOOP_TO = 5
        while (waiting_ack) and ((time.time() - time_s) < LOOP_TO):
            recv_ack = lora_sock.recv(256)
            if (len(recv_ack) > 0):
                device_id, pkg_len, ack = struct.unpack(_LORA_PKG_ACK_FORMAT, recv_ack)
                if (device_id == DEVICE_ID):
                    if (ack == 200):
                        waiting_ack = False
                        print("ACK")
                    else:
                        waiting_ack = False
                        print("Message Failed")
        time.sleep(5)
    lora_sock.close()
    client.close()       
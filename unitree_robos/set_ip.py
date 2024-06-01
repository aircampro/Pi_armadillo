# Set the IP address of the z1 robot
# Before running the program, please check the network with the robot
import re
import os
import sys
import socket
import struct

def check_ip(ipAddr):
    compile_ip = re.compile('(((\d{1,2})|(1\d{2})|(2[0-4]\d)|(25[0-5]))\.){3}((\d{1,2})|(1\d{2})|(2[0-4]\d)|(25[0-5]))')
    if compile_ip.match(ipAddr):
        return True
    else:
        print("[ERROR] Please input the valid IP address")
        return False

def ping_ip(ip):
    cmd = "ping " + str(ip) + " -c 1 -w 1 >/dev/null"  #send once, wait 1s
    if  os.system(cmd):
        print("[ERROR]IP: "+ip+" Destination Host Unreachable")
        return False
    else:
        return True

if __name__ == '__main__':
    # 1. check arm valid
    def_arm_ip = '192.168.123.110'               # use default ip
    arm_ip_addr = ''
    if not ping_ip(def_arm_ip):
        # 2. read IP address input
        print("Please enter the IP address to of the z1 gripper, which should be input as *.*.*.* ")
        while True:
            arm_ip_addr = input("Arm not pinging check connection and enter its correct IP to connect:   ")
            if check_ip(arm_ip_addr):
                break
            if not ping_ip(arm_ip_addr):
                exit_this = input("\007 \033[32m cannot communicate exiting..... type q to quit \033[0m")
                if exit_this == "q" or exit_this == "Q":
                    sys.exit(-1)
    else:
        arm_ip_addr = def_arm_ip
        
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = ("0.0.0.0", 8881)
    armAddr = (arm_ip_addr, 8880)
    server_socket.bind(address)
    pack_format = struct.Struct('6B')

    # 2. read IP address input
    print("Please enter the IP address to set, which should be input as *.*.*.* ")
    ipAddr = ''
    while True:
        ipAddr = input("IP to set:   ")
        if check_ip(ipAddr):
            break
    ipset = re.split('\.', ipAddr)
    
    # 3. send udp socket to set IP address
    send_data = [7, 1, int(ipset[0]), int(ipset[1]), int(ipset[2]), int(ipset[3])]
    send_data = pack_format.pack(*send_data)
    server_socket.sendto(send_data, armAddr)
    data, client = server_socket.recvfrom(6)
    data = pack_format.unpack(data)
    ipAddr = str(data[2])+'.'+str(data[3])+'.'+str( data[4])+'.'+str(data[5])
    print("Set correctly! Current robot IP is", ipAddr)
    print("Please repower the robot and change the IP setting in the z1_controller/config/config.xml")
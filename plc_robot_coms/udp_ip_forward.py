# -*- coding: utf-8 -*-
#
# simple UDP port forwarding 
#
import socket
UDP_IP_LISTEN = "10.168.1.12"
UDP_PORT_LISTEN = 161
UDP_IP_TO = "10.168.1.119"
UDP_PORT_TO = 161

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)     # UDP
    sock.bind((UDP_IP_LISTEN, UDP_PORT_LISTEN))
    src_port = 0

    while True:
        data, (host, port) = sock.recvfrom(4096)                 # receive
        if src_port != 0 and host != UDP_IP_LISTEN:              # receive from remote server
            sock.sendto(data, (UDP_IP_LISTEN, src_port))
        else:                                                    # send to remote server
            src_port = port
            sock.sendto(data, (UDP_IP_TO, UDP_PORT_TO))
        print(f"forwarded {host} {port}")

if __name__ == '__main__':
    main()

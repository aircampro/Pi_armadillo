#
# Port Forward example with Scapy
# Use of Raspi as a simple NAT Router
# sudo sysctl -w net.ipv4.ip_forward=0
#
from scapy.all import *
from scapy.layers.inet import IP, TCP, UDP
import select
import os
import sys

# Recreate IP Packet 
def recreate_ip_packet(ip_pkt):
    # Removal Check-Sum before packet re-build
    if ip_pkt.haslayer(TCP):
        del ip_pkt.getlayer(TCP).chksum
    if ip_pkt.haslayer(UDP):
        del ip_pkt.getlayer(UDP).chksum
    if ip_pkt.haslayer(ICMP):
        del ip_pkt.getlayer(ICMP).chksum
    del ip_pkt.chksum

    result = IP(ip_pkt.build())                         # re-build the packet with the new ip address (information)
    return result

# checks if ssh packet
def is_ssh_packet(target_packet):
    if not target_packet.haslayer(TCP):
        return False
    tcp_layer = target_packet.getlayer(TCP)
    return tcp_layer.dport == 22
    
class ipTOS:
    std = 0b000	        # standard
    pref = 0b001	    # preference
    immed = 0b010	    # immediate
    breakin = 0b011	    # Breaking
    prior = 0b100	    # Priority Bulletin
    important = 0b101	# important
    int_net = 0b110	    # Internetwork Control
    net = 0b111	        # Network Control

# set the router TOS for none then gTOS = None
# but you can change to any of the above in this router
#
gTOS = ipTOS.prior

# you can also change the protocol byte of the message None means no change
class ipProto:
    ICMP = 1
    TCP = 6
    UDP = 17
    ESP = 50
gPROTO_SWAP = None             # e.g. ipProto.UDP
    
### Main ###
# Inside Terminal Information #
in_ip_addr1 = '192.168.10.1'
in_ip_port1 = 1111
in_ip_addr2 = '192.168.10.2'
in_ip_port2 = 2222

# NAT Table List
nat_table1 = {}
nat_table2 = {}

# Transform List: IP address, Dest Port, NAT Table
trans_list = [[in_ip_addr1, in_ip_port1, nat_table1],\
              [in_ip_addr2, in_ip_port2, nat_table2]]

# NAT Outside & Inside Mac Address & IP Address
out_port_name='enp0s3'                                                    # eth0
in_port_name='enp0s8'                                                     # eth1

out_eth_mac = get_if_hwaddr(out_port_name)
out_eth_ip = get_if_addr(out_port_name)	                                  # 10.1.1.254
in_eth_mac = get_if_hwaddr(in_port_name)
in_eth_ip = get_if_addr(in_port_name)	                                  # 192.168.10.254

# if you want router to swap ports 
in_to_out_port = None
out_to_in_port = None

# Layer2 socket
out_sock = conf.L2socket(iface=out_port_name)
in_sock = conf.L2socket(iface=in_port_name)

# Not send RST in order not to disconnect from NAT router
if os.name == 'posix':
    os.system('sudo iptables -A OUTPUT -p tcp --tcp-flags RST RST -j DROP')

while True:
    # select()
    read_sockets, _, _ = select.select([out_sock, in_sock], [], [])

    for s in read_sockets:
        # Data receive
        p = s.recv()

        # Drop non TCP/UDP packet
        if not p or (not p.haslayer(TCP) and not p.haslayer(UDP)) :
            print("Not TCP/UDP packet")
            continue
        if is_ssh_packet(p):
            print("SSH packet")
            continue
                    
        # From Outside to Inside (Initiate from Outside)
        if p.dst == out_eth_mac:
            ip_pkt_out_to_in = p.getlayer(IP)
            found = 0
            for i in range(len(trans_list)):
                if ip_pkt_out_to_in.dport == trans_list[i][1]:
                    found = 1
                    # Keep Source IP and Source Port at NAT Table
                    trans_list[i][2][ip_pkt_out_to_in.src] = ip_pkt_out_to_in.sport
                    print(trans_list[i][2])
                    # Replace IP address with local IP adress re-routed
                    ip_pkt_out_to_in.dst = trans_list[i][0]
                    # if you defined a port change make it
                    if not out_to_in_port == None:
                        ip_pkt_out_to_in.dport = out_to_in_port
                    # Fields that determine the quality of communication = type of service
                    if not gTOS == None:
                        ip_pkt_out_to_in.tos = gTOS  
                    # if you want to change the protocol byte                        
                    if not gPROTO_SWAP == None: 
                        ip_pkt_out_to_in.proto = gPROTO_SWAP                      
                    # Recreate IP address
                    pkt_to_ip_addr = recreate_ip_packet(ip_pkt_out_to_in)
                    print(pkt_to_ip_addr)
                    send(pkt_to_ip_addr, verbose=0)
                    break
            if found == 0:
                print("Closed Port: " + str(ip_pkt_out_to_in.dport))

        # From Inside to Outside
        elif p.dst == in_eth_mac:
            ip_pkt_in_to_out = p.getlayer(IP)
            found = 0
            for i in range(len(trans_list)):
                if ip_pkt_in_to_out.src == trans_list[i][0]:
                    found = 1
                    # Get destination port from destination IP at NAT Table
                    dport = trans_list[i][2].get(ip_pkt_in_to_out.dst)
                    #print(dport)
                    if dport is None:
                        print("No data in NAT Table")
                        continue
                    # Replace IP address with send IP address
                    ip_pkt_in_to_out.src = out_eth_ip
                    # if you defined a port change make it
                    if not in_to_out_port == None:
                        ip_pkt_in_to_out.sport = in_to_out_port
                    # Recreate IP address
                    pkt_from_ip_addr = recreate_ip_packet(ip_pkt_in_to_out)
                    print(pkt_from_ip_addr)
                    send(pkt_from_ip_addr, verbose=0)
                    break
            if found == 0:
                print("Not allowed inside IP address")
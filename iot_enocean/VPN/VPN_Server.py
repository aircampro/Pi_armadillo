#!/usr/bin/env python3
# example using library here... https://github.com/kstm-su/dnsvpn/tree/master
# VPN Server example
#
import time
from lib.tuntap import TunThread
from lib import dns
from lib import query
from lib.packet import Packet, PacketPool
import os
import fcntl
import time
import sys, socket, struct
import signal

# define process lock so it can only run one instance
def process_lock():
    lockfile = os.path.splitext(os.path.abspath(__file__))[0] + '.lock'
    lockfp = open(lockfile, "w")
    try:
        fcntl.flock(lockfp, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except IOError:
        return
    return lockfp

# convert mac to a string
def mac2s(m):
    return ':'.join(hex(num)[2:].zfill(2) for num in m)

# get the mac address
def get_mac(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', bytes(ifname[:15], "ascii")))
    finally:
        s.close()
    return info[18:24]

# get ip address (ifconfig)
def get_addr(interface):
    SIOCGIFADDR = 0x8915

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ifreq  = struct.pack('16s16x', interface)
        ifaddr = fcntl.ioctl(s.fileno(), SIOCGIFADDR, ifreq)
    finally:
        s.close()

    _, sa_family, port, in_addr = struct.unpack('16sHH4s8x', ifaddr)
    print(socket.inet_ntoa(in_addr))
	
# set the hostname here	
hostname = b'vpn.bgpat.net'
query.Field.HostName.default = hostname
query.Field.HostName.pattern = hostname
Packet.hostname = hostname

txpool = PacketPool()
rxpool = PacketPool()

class VPNServer(TunThread):
    daemon = True
    name = 'tun0'
    addr = '192.168.200.1'
    gateway = '192.168.200.1'

    def receive(self, data):
        global rxpool
        pkt = Packet(data)
        rxpool.push(pkt)

class DNSServer(dns.ServerThread):
    addr = '0.0.0.0'
    daemon = True

    def receive(self, req, addr, port, data):
        txInit = query.TxInitialize(req)
        txSend = query.TxSend(req)
        rxRecv = query.Receive(req)
        rxPoll = query.Polling(req)
        try:
            if txInit.params is not None:
                params = txInit.params
                res = self.txinit(params['id'], params['count'])
                print('txinit', params)
            elif txSend.params is not None:
                id = txSend.params['id']
                seq = txSend.params['sequence']
                res = self.txsend(id, seq, txSend.params['data'])
                print('txsend', id, seq)
            elif rxRecv.params is not None:
                params = rxRecv.params
                res = self.rxrecv(params['id'], params['sequence'])
                print('rxrecv', params, req)
            elif rxPoll.params is not None:
                res = self.rxpoll()
                print('rxpoll')
            else:
                res = query.Error()
        except:
            res = query.Error()
            print('exception', req, addr, port, data.qd.__dict__)
        print('< ', res.__class__.__name__, res.params)
        return {
            'value': bytes(res),
            'type': res.type,
        }

    def txinit(self, id, count):
        global txpool
        txpool[id] = Packet(count)
        return query.Ok(count=count, sequence=count)

    def txsend(self, id, seq, data):
        global txpool
        pkt = txpool[id]
        pkt[seq] = data
        remain = pkt.count - len(pkt)
        if not remain:
            tun.send(pkt.unpack())
            del txpool[id]
        return query.Ok(count=pkt.count, sequence=seq)

    def rxrecv(self, id, seq):
        global rxpool
        if id not in rxpool:
            print('recv error', id, seq)
            print('rxpool', list(rxpool.keys()))
            return query.Error()
        pkt = rxpool[id]
        if seq in pkt:
            del pkt[seq]
        keys = list(pkt.keys())
        if len(keys):
            i = keys[0]
            return query.RxSend(data=pkt[i], sequence=i, id=id)
        else:
            del rxpool[id]
            return query.Error()

    def rxpoll(self):
        global rxpool
        if rxpool.empty():
            return query.Error()
        pkt = rxpool.front()
        if pkt.id in rxpool:
            return query.Error()
        rxpool[pkt.id] = pkt
        rxpool.pop()
        return query.RxInitialize(data=pkt[0], count=pkt.count, id=pkt.id)

g_actv == True
def signal_handler(signum, frame):
    g_actv = False
	
# run the main vpn server
#
def main():
    global g_actv
    signal.signal(signal.SIGUSR1, signal_handler)             # kill -10
    signal.signal(signal.SIGUSR2, signal_handler)             # kill -12	
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
	
    lock = process_lock()
    if not lock:
        print("lock error")
        exit(1)

    # print mac and ip for the eth0 nic
    interfce = "eth0"
    get_addr(interfce)
    print(mac2s(get_mac(interfce)))
	
    tun = VPNServer()
    tun.start()
    dnsd = DNSServer()
    dnsd.start()
    while g_actv == True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            g_actv = False
    lock.close()
    tun.join()
    dnsd.join()
    	
if __name__ == '__main__':
    main()	
	
#!/usr/bin/env python3
# VPN Client example
#
import time
import socket
from lib.tuntap import TunThread
from lib import dns
from lib import query
from lib.packet import Packet
from lib.connection import Connection, ConnectionPool
from lib.resolv import get_resolv_conf

addr = get_resolv_conf()['nameserver'][0]
hostname = b'vpn.bgpat.net'
query.Field.HostName.default = hostname
query.Field.HostName.pattern = hostname
Packet.hostname = hostname
dns.Client.protocol = 'UDP'

class TxConnection(Connection):

    def main(self):
        init = query.TxInitialize(id=self.data.id, count=self.data.count)
        req = {
            'value': bytes(init),
            'type': init.type,
        }
        client = dns.Client(addr=addr, data=req)
        self.receive(client)

    def receive(self, client):
        if not len(client.answers):
            raise Exception('No answers')
        for ans in client.answers:
            ok = query.Ok(ans)
            if ok.params is None:
                continue
            count = ok.params['count']
            seq = ok.params['sequence']
            if seq < count:  
                del self.data[seq]
                if not len(self.data):  
                    return
        self.send()

    def send(self):
        i = list(self.data.keys())[0]
        req = query.TxSend(data=self.data[i], sequence=i, id=self.data.id)
        client = dns.Client(addr=addr, data={
            'value': bytes(req),
            'type': req.type,
        })
        self.receive(client)

class RxConnection(Connection):

    def main(self):
        self.send(0)

    def receive(self, client):
        seq = None
        for ans in client.answers:
            rxSend = query.RxSend(ans)
            if rxSend.params is not None:
                print('rxsend', rxSend.params)
                seq = rxSend.params['sequence']
                self.data[seq] = rxSend.params['data']
        if seq is None:
            raise Exception('No answers')
        self.send(seq)

    def send(self, seq):
        global tun
        req = query.Receive(sequence=seq, id=self.data.id, padding=16)
        if len(self.data) >= self.data.count:
            tun.send(self.data.unpack())
        client = dns.Client(addr=addr, data={
            'value': bytes(req),
            'type': req.type,
        })
        if len(self.data) < self.data.count:
            self.receive(client)

txconn = ConnectionPool(TxConnection)
rxconn = ConnectionPool(RxConnection)

class VPNClient(TunThread):
    daemon = True
    name = 'tun0'
    addr = '192.168.200.2'
    gateway = '192.168.200.1'

    def receive(self, data):
        pkt = Packet(data)
        txconn.push(pkt)
        print('tx work:%d queue:%d' % (len(txconn), txconn.queue.qsize()))

# runs the VPN client
def main():
    tun = VPNClient()
    tun.start()

    while True:
        req = query.Polling(padding=16)
        try:
            client = dns.Client(addr=addr, data={
                'value': bytes(req),
                'type': req.type,
            })
        except socket.timeout:
            continue
        for ans in client.answers:
            init = query.RxInitialize(ans)
            if init.params is None:
                time.sleep(0.5)
                continue
            print('rxinit', init.params)
            pkt = Packet(init.params['count'])
            pkt.id = init.params['id']
            pkt[0] = init.params['data']
            rxconn.push(pkt)
            print('rx work:%d queue:%d' % (len(rxconn), rxconn.queue.qsize()))

if __name__ == '__main__':
    main()
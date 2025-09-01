# client sigfox
#
# ref:- https://docs.pycom.io/tutorials/networks/sigfox/
#
from network import Sigfox
import socket
import sys

# buffer size, default ip address and port number
omron_cj_rcv_buf_sz = 1024                                              # receive buffer size
omron_cj_ip = "192.168.0.1"
omron_cj_port = 8555

# omron plc is on a UDP Ethernet network
#
def connectUDP(host, port):
    UDP_client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)   
    UDP_client.settimeout(10)   
    try:
        UDP_client.connect((host, port))
        return UDP_client
    except socket.error:
        UDP_client.close()
        return -1

# omron plc is on a TCP Ethernet network
#
def connectTCP(host, port):
    import socket
    TCP_client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)   
    TCP_client.settimeout(10)
    try:
        TCP_client.connect((host, port))
        return TCP_client
    except socket.error:
        TCP_client.close()
        return -1

if __name__ == "__main__":

    # connect with sigfox
    sigfox = Sigfox(mode=Sigfox.FSK, frequency=868000000)
    sf = socket.socket(socket.AF_SIGFOX, socket.SOCK_RAW)
    sf.setblocking(True)
    # connect with PLC
	om_handle = connectUDP(omron_cj_ip, omron_cj_port)
    if om_handle == -1:
        print("cannot connect to PLC")
        sys.exit(-1)

    while True:
        r = sf.recv(omron_cj_rcv_buf_sz)
		om_handle.send(r)
        rr = om_handle.recv(omron_cj_rcv_buf_sz)
        sf.send(rr)
        time.sleep(0.1)

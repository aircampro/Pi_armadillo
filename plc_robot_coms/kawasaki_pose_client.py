#
# EMULATION OF KAWASAKI ROBOT AS CLIENT FOR TESTING
# should make connection get pose and then clsoe connection tp server
#
import socket
import threading

IPADDR = "192.168.0.12"
PORT = 10000

RECV1=0
SEND_ID=1
RECV2=2
ENDIT=3
# start with the state engine as initital receive
STATE=RECV1

sock = socket.socket(socket.AF_INET)
sock.settimeout(10)
sock.connect((IPADDR, PORT))

# receive function
def recv_data(sock):
    while True:
        try:
            data = sock.recv(1024)
            if data == b"":
                break
            print(data.decode("utf-8"))
        except ConnectionResetError:
            break
        if not data.decode("utf-8").find("POSE") == -1:
            data_dec = data.decode("utf-8").split(",")
            x = data_dec[1]
            y = data_dec[2]
            z = data_dec[3]
            print("pose recv = x %s y %s z %s" % (x,y,z))
            STATE=ENDIT
#           STATE=RECV1          if you want to continuosly cycle
        elif not data.decode("utf-8").find("1") == -1:
            STATE=SEND_ID
    sock.shutdown(socket.SHUT_RDWR)
    sock.close()

# start the recieve thread
thread = threading.Thread(target=recv_data, args=(sock,))
thread.start()

# always
while True:
    my_id_msg = b"ID_2\x13\x10"
    if (STATE==SEND_ID):
        try:
            sock.send(my_id_msg.encode("utf-8"))
            STATE=RECV2
        except ConnectionResetError:
            break
    elif (STATE==ENDIT):
        break
        
sock.shutdown(socket.SHUT_RDWR)
sock.close()
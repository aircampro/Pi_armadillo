# KAWASAKI ROBOT SERVER 
# SENDS A POSE TO EACH ROBOT WHEN IT RECEIVES THE SPECIFIED ID
#
import socket
import threading

IPADDR = "192.168.0.12"
PORT = 10000

sock_sv = socket.socket(socket.AF_INET)
sock_sv.bind((IPADDR, PORT))
sock_sv.listen()

# list all the clients here
client_list = []

POSE_1 = b"POSE,10,0,1,END\x13\x10"
POSE_2 = b"POSE,20,10,0,END\x13\x10"
POSE_3 = b"POSE,0,10,1,END\x13\x10"
POSE_4 = b"POSE,0,0,10,END\x13\x10"

def recv_client(sock, addr):
    while True:
        try:
            data = sock.recv(1024)
            if data == b"":
                break

            print("$ say client:{}".format(addr))

            # for each client we handle robots 1 to 4 by sending a pose otherwise just echo what came in
            for client in client_list:
                if not data.find("ID_1") == -1:
                    client[0].send(POSE_1)
                elif not data.find("ID_2") == -1:
                    client[0].send(POSE_2)
                if not data.find("ID_3") == -1:
                    client[0].send(POSE_3)
                elif not data.find("ID_4") == -1:
                    client[0].send(POSE_4)
                else:
                    client[0].send(data)

        except ConnectionResetError:
            break

    # close the connection to the client
    client_list.remove((sock, addr))
    print("- close client:{}".format(addr))

    sock.shutdown(socket.SHUT_RDWR)
    sock.close()

# main program
while True:
    sock_cl, addr = sock_sv.accept()
    # initially send a handshake to the client
    sock.cl.send(b"1\x13\x10")
    client_list.append((sock_cl, addr))
    print("+ join client:{}".format(addr))

    thread = threading.Thread(target=recv_client, args=(sock_cl, addr))
    thread.start()
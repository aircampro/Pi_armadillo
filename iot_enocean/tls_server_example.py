# TLS server example
import socket, ssl

URL = '127.0.0.1'
PORT = 10023

context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
context.load_cert_chain(certfile="keys/server.crt", keyfile="keys/server.key")

bindsocket = socket.socket()
bindsocket.bind((URL, 10023))
bindsocket.listen(5)

def do_something(connstream, data):
    print '---------------------'
    print data + '\n'
    print '---------------------'
    print

def deal_with_client(connstream):
    data = connstream.read()
    # null data means the client is finished with us
    while data:
        if not do_something(connstream, data):
            # we'll assume do_something returns False
            # when we're finished with client
            break
        data = connstream.read()
    # finished with client

while True:
    newsocket, fromaddr = bindsocket.accept()
    connstream = context.wrap_socket(newsocket, server_side=True)
    try:
        deal_with_client(connstream)
    finally:
        connstream.shutdown(socket.SHUT_RDWR)
        connstream.close()
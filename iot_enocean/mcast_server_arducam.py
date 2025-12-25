#!/usr/bin/python
#
# multicast server arducam
#
import sys, getopt
import asyncore
import numpy as np
import pickle
import socket
import struct
import cv2
from RpiCamera import Camera
from time import gmtime, strftime
import sys
import argparse
from ctypes import CDLL
arducam_vcm = CDLL('libarducam_vcm.so')                                                # arducam i2c autofocus (need to enable i2c) ref:- https://qiita.com/ma2shita/items/c180f818a675741bf660

class range_check():
    def __init__(self):
        self.min = 0
        self.max = 1023
    def __contains__(self, val):
        return (self.min <= val and val <= self.max)
    def __iter__(self):
        return iter(("Integer", "{} <= N <= {}".format(self.min, self.max)))

mc_ip_address = '224.0.0.1'
port = 1024
chunk_size = 4096
camera = Camera()
camera.start_preview(False)

def getImgAndTimestamp():
	# represent the frame as a numpy array
    frame = camera.getFrame()
    img = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)       
	cvMat = np.asanyarray(img)
	ts=strftime("%H:%M:%S", gmtime())
    return cvMat, ts

class DevNullHandler(asyncore.dispatcher_with_send):

    def handle_read(self):
        print(self.recv(1024))

    def handle_close(self):
        self.close()
 
class EtherSenseServer(asyncore.dispatcher):
    def __init__(self, address):
        asyncore.dispatcher.__init__(self)
        print("\033[32m Launching Arducam B0176 Camera Server.... \033[0m")
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        print('sending acknowledgement to', address)
        self.frame_data = ''
        self.connect((address[0], 1024))
        self.packet_id = 0        

    def handle_connect(self):
        print("connection received")

    def writable(self):
        return True

    def update_frame(self):
	    img, timestamp = getImgAndTimestamp()
        if img is not None:
	        # convert the image to a string for broadcast
            data = pickle.dumps(img)
	        # capture the lenght of the data portion of the message	
            length = struct.pack('<I', len(data))
	        # include the current timestamp for the frame
            ts = struct.pack('<d', timestamp)
	        # for the message for transmission
            self.frame_data = ''.join([length, ts, data])

    def handle_write(self):
	    # first time the handle_write is called
        if not hasattr(self, 'frame_data'):
            self.update_frame()
        if len(self.frame_data) == 0:
            self.update_frame()
        else:
	        # send the remainder of the frame_data until there is no data remaining for transmition
            remaining_size = self.send(self.frame_data)
            self.frame_data = self.frame_data[remaining_size:]

    def handle_close(self):
        self.close()

class MulticastServer(asyncore.dispatcher):
    def __init__(self, host = mc_ip_address, port=1024):
        asyncore.dispatcher.__init__(self)
        server_address = ('', port)
        self.create_socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind(server_address) 	

    def handle_read(self):
        data, addr = self.socket.recvfrom(42)
        print('Recived Multicast message %s bytes from %s' % (data, addr))
	    # Once the server recives the multicast signal, open the frame server
        focus = data.split("::")[1]                                                             # set focus
        arducam_vcm.vcm_write(int(focus))
        EtherSenseServer(addr)
        print(sys.stderr, data)

    def writable(self): 
        return False # don't want write notifies

    def handle_close(self):
        self.close()

    def handle_accept(self):
        channel, addr = self.accept()
        print('received %s bytes from %s' % (data, addr))

def main():
    # initalise the multicast receiver 
    server = MulticastServer()
    # hand over excicution flow to asyncore
    asyncore.loop()
   
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--focus", type=int, dest="focus", required=True, choices=range_check(), help="Camera focus 0(far)..1023(near)")
    args = parser.parse_args()
    print("Focus set to {}".format(args.focus), file=sys.stderr)
    arducam_vcm.vcm_init()
    arducam_vcm.vcm_write(args.focus)
    main()


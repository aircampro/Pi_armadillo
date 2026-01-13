# coding:utf-8
#
# Interface to m5STack Camera over wifi and create a socket server 
#
# https://github.com/m5stack/M5Stack-Camera/tree/master/wifi/wifi_ap/firmware/M5Camera
# https://docs.m5stack.com/en/unit/m5camera?id=video
# https://openelab.io/collections/m5stack-cameras	
#
import io
import numpy as np
import requests
import cv2
import time
import socket
import numpy as np
import cv2
import time
import configparser
import getIpAddress as gIA

M5IP="192.168.4.1"                                 # M5Camera IP Adress                                        
FPS=10                                             # frames per second                                    
def get_image_from_m5Stack(writ=False):
    res = requests.get(f"http://{M5IP}/jpg")                   
    bin_data = io.BytesIO(res.content)
    file_bytes = np.asarray(bytearray(bin_data.read()), dtype=np.uint8)
    img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
    if writ == True:
        cv2.imwrite("m5camera.jpg", img)
    return img

config = configparser.ConfigParser()
config.read('./config.ini', 'UTF-8')
INDENT = '    '

IMAGE_WIDTH = int(config.get('pixels', 'image_width'))
IMAGE_HEIGHT = int(config.get('pixels', 'image_height'))
IMAGE_QUALITY = 30

SERVER_IP = str(config.get('server', 'ip'))
SERVER_PORT = int(config.get('server', 'port'))
if SERVER_IP == "AUTO":
    l = [d.get('name') for d in gIA.get_ip()]
    print(l)
    for k in gIA.get_ip():
        if (k.get('name') == 'wlan0'):
            HOST_wlan0 = k['address']
        elif (k.get('name') == 'eth0'):
            HOST_eth0 = k['address']

    if HOST_wlan0 is not None:
        SERVER_IP = HOST_wlan0
    else:
        SERVER_IP = HOST_eth0

if SERVER_IP == "AUTO" or SERVER_IP is None:
    print("SERVER_IP is {}".format(SERVER_IP))
    exit()

HEADER_SIZE = int(config.get('packet', 'header_size'))

# listen
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((SERVER_IP, SERVER_PORT))
s.listen(1)
soc, addr = s.accept()

print('Server {')
print(INDENT + 'IP   : {},'.format(SERVER_IP))
print(INDENT + 'PORT : {}'.format(SERVER_PORT))
print('}')

print('Client {')
print(INDENT + 'IP   : {},'.format(addr[0]))
print(INDENT + 'PORT : {}'.format(addr[1]))
print('}')

while True:
    loop_start_time = time.time()
    frames = get_image_from_m5Stack()
    color_frame = frames[0]
    depth_frame = frames[1]
    resized_img = cv2.resize(color_frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
    (status, encoded_img) = cv2.imencode('.jpg', resized_img, [int(cv2.IMWRITE_JPEG_QUALITY), IMAGE_QUALITY])
    packet_body = encoded_img.tostring()
    packet_header = len(packet_body).to_bytes(HEADER_SIZE, 'big')
    packet = packet_header + packet_body
    try:
        soc.sendall(packet)
    except socket.error as e:
        print('Connection closed.')
        break
    # FPS
    time.sleep(max(0, 1 / FPS - (time.time() - loop_start_time)))

s.close()
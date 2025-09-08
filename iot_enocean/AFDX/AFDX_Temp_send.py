# send data using AFDX ARIC664 protocol 
# with encryption it choses for the actual temperature value
# ability to change passwords has not been included so that it can first be tested with the wind speed unit first
#
import json
from enum import Enum
from twofish import Twofish
import triplesec
from salsa20 import XSalsa20_xor
import random
import socket
import time
import sys
import signal
# AFDX settings
UDP_PORT = 5005
BUFFER_SIZE = 512
BROADCAST_IP = "255.255.255.255"

# https://github.com/FiloCara/pyS7
#
from pyS7 import S7Client

def get_temp_from_s7(addrs="192.168.5.100",r=0):

    # Create a new 'S7Client' object to connect to S7-300/400/1200/1500 PLC.
    client = S7Client(address=addrs, rack=r, slot=1)

    # Establish connection with the PLC
    client.connect()

    # Define area tags to read the temperature from
    tags = [
        "QR26"                               # Read REAL at address 26 in output area
    ]

    # Read the data from the PLC using the specified tag list
    data = client.read(tags=tags)
    client.close()
    return data
    
# types of encryption we support
class choose_cipher(Enum):
    NONE = 0
    TWOFISH = 1
    TRIPLESEC = 2
    XSALSA20 = 3
NUM_OF_CIPHER = 3

# types of AFDX messages
messageTypes = {
      'SYNC': 'SYNC',
      'DATA': 'DATA',
      'ALERT': 'ALERT',
      'ACK': 'ACK',
      'JOIN': 'JOIN',
      'LEAVE': 'LEAVE',
      'HEARTBEAT': 'HEARTBEAT'
    }

priorities = {
      'A': '0',                               # Critical
      'B': '1',                               # Operational  
      'C': '2'                                # Background
    }

def make_join_msg(did="TMP_PROBE_01", rn):
    jn_msg = {
      "type": messageTypes['JOIN'],
      "device": did,
      "seq": rn
    }
    return jn_msg

def make_ack_msg(did="TMP_PROBE_01", rn):
    sync_msg = {
      "type": messageTypes['ACK'],
      "device": did,
      "timestamp": time.time()
      "seq": rn
    }
    return sync_msg
    
def make_temp_msg(temp_probe=0, did="TMP_PROBE_01"):
    temperature_msg = {
      "type": messageTypes['DATA'],
      "device": did,
      "data": temp_probe
    }
    return temperature_msg

def make_alert_msg(msg="over-temp alarm", did="TMP_PROBE_01"):    
    alert_msg = {
      "type": messageTypes['ALERT'],
      "device": did,
      "message": msg
    }
    return alert_msg

# list of the default passwords
pw_list = ["TWOFISH", "TRIPLESEC", "XSALSA20"]

# all passwords and secret keys have been hard-coded below  
#    
# put (JSON) dict to a string to encode and send as a ip message
#
def dict2json_str_byte(json_dict, encryp=choose_cipher.NONE.value):
    json_str = json.dumps(json_dict)
    msg = json_str.encode('utf-8')
    if encryp == choose_cipher.TWOFISH.value:
        #pw = b'*secret*'
        pw = pw_list[choose_cipher.TWOFISH.value-1]
        T = Twofish(pw)
        msg = T.encrypt(msg)  
    elif encryp == choose_cipher.TRIPLESEC.value:
        #pw = b'* password *'
        pw = pw_list[choose_cipher.TRIPLESEC.value-1]
        msg = triplesec.encrypt(msg, pw)     
    if encryp == choose_cipher.XSALSA20.value:
        IV = urandom(24)
        #KEY = b'*secret**secret**secret**secret*'
        KEY = pw_list[choose_cipher.XSALSA20.value-1]
        msg = XSalsa20_xor(msg, IV, KEY)         
    return msg

# received ip message -> decoded string to JSON dict
#
def json_str2dict(msg, decryp=choose_cipher.NONE.value):
    if encryp == choose_cipher.TWOFISH.value:
        #pw = b'*secret*'
        pw = pw_list[choose_cipher.TWOFISH.value-1]
        T = Twofish(pw)
        msg = T.decrypt(msg).decode()  
    elif encryp == choose_cipher.TRIPLESEC.value:
        #pw = b'* password *'
        pw = pw_list[choose_cipher.TRIPLESEC.value-1]
        msg = triplesec.decrypt(msg, pw).decode()     
    elif encryp == choose_cipher.XSALSA20.value:
        IV = urandom(24)
        #KEY = b'*secret**secret**secret**secret*'
        KEY = pw_list[choose_cipher.XSALSA20.value-1]
        msg = XSalsa20_xor(msg, IV, KEY).decode() 
    json_str = msg.decode('utf-8')
    json_dict = json.loads(json_str)
    return json_dict

# dict -> file(JSON)
def dict2json_file(json_dict, filename):
    with open(filename, 'w') as f:
        json.dump(json_dict, f)
    print('dict2json_file -> ', filename)

# file(JSON) -> dict
def json_file2dict(filename):
    with open(filename, 'r') as f:
        json_dict = json.load(f)
    print('json_file2dict -> ', type(json_dict))
    return json_dict

# send out message
def send_msg(message):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock2:
        sock2.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)                 # this allows as many masters as you like can change to specific ip only
        sock2.sendto(message, (BROADCAST_IP, UDP_PORT))
        print(f"Broadcast sent @ {time.time():.3f}")

# The name of this device
#
if len(sys.argv) == 1:
    MY_DEVICE="TMP_PROBE_01"
else:
    MY_DEVICE=str(sys.argv[1])

RUN_F = True
def sig_handler(s):
    global RUN_F
    RUN_F = False

if __name__ == "__main__":
    signal.signal(signal.SIGUSR1, sig_handler)
    signal.signal(signal.SIGUSR2, sig_handler)
    while RUN_F == True:
        try:
            so = 1
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("", UDP_PORT))
            print("[RECEIVER] Listening on port", UDP_PORT)
            data, addr = sock.recvfrom(BUFFER_SIZE)
            recv_time = time.time()
            sock.close()
            so = 0
            #message = data.decode("utf-8")
            msg_json = json_str2dict(data, decryp=choose_cipher.NONE.value)
            device = addr[0]
            if msg_json['type'] == 'SYNC' and msg_json['device'] == MY_DEVICE:
                print(f"[SYNC] {recv_time:.3f} | {device} |")
                # generate random number for ciphertext
                rn = random.choice(NUM_OF_CIPHER)
                if rn == 0: 
                    rn = 1                                                         # force to always cipher it.
                # send ACK
                sm = make_ack_msg(did=MY_DEVICE, rn)
                sb = dict2json_str_byte(sm, 0)                                     # reads sync and ack without encryption
                send_msg(sb)
                # get TEMP and send encrypted DATA message 
                tv = get_temp_from_s7()
                tm = make_temp_msg(temp_probe=tv,did=MY_DEVICE)
                tb = dict2json_str_byte(tm, rn)
                send_msg(tb)
            elif msg_json['type'] == 'LEAVE' and msg_json['device'] == MY_DEVICE:    # reads leave without encryption
                print(f"[LEAVE] {recv_time:.3f} | {device} | generate new password list currently unsupoerted in this device")
                # =========== on this unit it will just print this message ===============
                # generate new password list
                # send to master with JOIN message (keep doing until ACK received back)
                # now update passwords
            else:
                print(f"[RECV] {recv_time:.3f} | {device} |  {msg_json['type']}")
        except Exception as e:
            print(f"[ERROR] Parse failed: {e}")
    if so == 1:
        sock.close()
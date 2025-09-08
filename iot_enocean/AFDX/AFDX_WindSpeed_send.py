# Send data using AFDX ARIC664 protocol 
# with the encryption it choses for the actual wind speed value
#
# Normal behaviour is to recv SYNC and reply with ACK (confirming the cipher to use) and DATA (cifered)
#
# This unit also has got an ability to change its passwords when requested to do so using a LEAVE message
# it will generate all new ones for each cifer and reply with a JOIN message, this is confirmed by the 
# master using the new password and sending a HEARTBEAT message as confirmation of this, the new passwords
# will at that point be updated in this unit.
#
# ------------------------- security advancement ---------------------------------
# units are passive so will not communicate without being requested by the master
# values are sent over encrypted cifer
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

# ref :- https://github.com/gpiozero/gpiozero
#
from gpiozero import Button
import math

wind_count = 0
TINTERVAL = 2

def reset_wind():
    global wind_count
    wind_count = 0

def spin():
    global wind_count
    wind_count = wind_count + 1
    print("spin" + str(wind_count))

# To implement this formula in Python, you can use the math library. 
# For example, if you measured 17 signals from your anemometer in 5 seconds, your wind speed could be calculated like this:
#
def calc_speed(wind_interval = 5, wind_count = 17, radius_cm = 9.0):
    circumference_cm = (2 * math.pi) * radius_cm
    rotations = wind_count / 2.0
    dist_cm = circumference_cm * rotations
    speed = dist_cm / wind_interval
    return speed

# =============== password generator from random numbers (change words as you like) ============================
#	
def f(a):b=int(a/60)*10+10;d[a]=d[a-a%b]+(' et ','_')[a%10!=1or a>80]+d[a%b]

d=dict(zip([i for i in range(17)]+[i for i in range(20,70,10)]+[80,100],'zerr00oe ein2x dw1o trhee fiur4 fife sicx6 7seben ea8ght n9yne teen eleaven twilve thirtyne quatorze quinze seizer vingt0 tren76te quarante fifty sixty quatre_vingt hondred'.split()))

[f(v)for v in range(100)if(v in d)<1]

d[80]+='s'

# now d[] contains 0 to hundred
d[0] = ''
def get10(a):
    return d[a]

def getNum(a,b):
    m = ['', '', 'too', 'thyreee', 'fure', 'fyyve', 'siiix', 'sefeen', 'agiht', 'noyne']
    if 2 <= a <= 9:
        return m[a] + '@' + b + '!'
    if 1 == a:
        return b + '_'
    return '_'

def get1000(a): 
    return getNum(a,'thoozand') 

def get100(a): 
    return getNum(a,'undered')

# types of encryption we support
class choose_cipher(Enum):
    NONE = 0
    TWOFISH = 1
    TRIPLESEC = 2
    XSALSA20 = 3
NUM_OF_CIPHER = 3

# This function creates random numbers and translates them to words
# we create the following lists from the function
# returns a - index list, b - password strings, c - random number sequence generated
#
def random_passwd(no_to_make=NUM_OF_CIPHER):
    a = []
    b = []
    c = []
    for i in range(no_to_make):
        m = [random.choice(range(3)), random.choice(range(10)), random.choice(range(100))]
        mm = m[0] * 10 + m[1]
        print(int(mm) if mm > 0 else '' , "{0:02d}".format(m[2]), ',', get1000(m[0]) + get100(m[1]) + get10(m[2]), sep='')
        nn = int(mm)*100 + int(m[2])
        c.append(int(nn))
        mm = nn % no_to_make
        a.append(int(mm))
        m = get1000(m[0]) + get100(m[1]) + get10(m[2])
        b.append(m)
    return a, b, c
 
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
    
def make_data_msg(data_v=0, did="TMP_PROBE_01"):
    data_msg = {
      "type": messageTypes['DATA'],
      "device": did,
      "data": data_v
    }
    return data_msg

def make_alert_msg(msg="over-temp alarm", did="TMP_PROBE_01"):    
    alert_msg = {
      "type": messageTypes['ALERT'],
      "device": did,
      "message": msg
    }
    return alert_msg

# list of the default passwords
pw_list = ["TWOFISH", "TRIPLESEC", "XSALSA20"]
prev_pw_list = []

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
        sock2.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)                         # allows as many masters as you like can change to specific ip
        sock2.sendto(message, (BROADCAST_IP, UDP_PORT))
        print(f"Broadcast sent @ {time.time():.3f}")

# the name for this unique device
if len(sys.argv) == 1:
    MY_DEVICE="WIND_SPEED_01"
else:
    MY_DEVICE=str(sys.argv[1])
NO_JOIN_ACK = 10

RUN_F = True
def sig_handler(s):
    global RUN_F
    RUN_F = False
    
if __name__ == "__main__":
    signal.signal(signal.SIGUSR1, sig_handler)
    signal.signal(signal.SIGUSR2, sig_handler)
    reset_wind()
    wind_speed_sensor = Button(5)
    wind_speed_sensor.when_pressed = spin
    st_time = time.time()
    cnt = wind_count
    speed = 0.0
    join_sent = 0
    rn = random.choice(NUM_OF_CIPHER)
    if rn == 0: 
        rn = 1
    so = 0
    while RUN_F == True:
        if (time.time() - st_time) >= TINTERVAL:
            speed = calc_speed((time.time() - st_time), (wind_count - cnt))
            st_time = time.time()
            cnt = wind_count
            print(st_time)
            print(f"wind speed {speed}")
        try:
            so = 1
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)                                     # set-up to receive from UDP socket
            sock.bind(("", UDP_PORT))
            sock.settimeout(10)                                                                         # socket timetime set to 10 secs  
            print("[RECEIVER] Listening on port", UDP_PORT)
            data, addr = sock.recvfrom(BUFFER_SIZE)
            recv_time = time.time()
            sock.close()
            so = 0
            #message = data.decode("utf-8")
            msg_json = json_str2dict(data, decryp=choose_cipher.NONE.value)
            device = addr[0]
            if msg_json['type'] == 'SYNC' and msg_json['device'] == MY_DEVICE:                              # when you get the SYNC send the ACK then DATA message
                print(f"[SYNC] {recv_time:.3f} | {device} ")
                rn = random.choice(NUM_OF_CIPHER)                                                           # generate random number for ciphertext
                if rn == 0: 
                    rn = 1
                sm = make_ack_msg(did=MY_DEVICE, rn)                                                        # send ACK
                sb = dict2json_str_byte(sm, 0)
                send_msg(sb)
                tm = make_data_msg(data_v=speed, did=MY_DEVICE)                                             # send wind speed
                tb = dict2json_str_byte(tm, rn)
                send_msg(tb)
            elif msg_json['type'] == 'LEAVE' and msg_json['device'] == MY_DEVICE:                           # when you get the LEAVE get new passwords and JOIN
                print(f"[LEAVE] {recv_time:.3f} | {device} ")
                a, b, c = random_passwd()                                                                   # generate new passwords for each cipher
                sm = make_join_msg(did=MY_DEVICE, b)                                                        # send JOIN msg with new passwords
                sb = dict2json_str_byte(sm, rn)                                                             # send using the current cipher
                send_msg(sb)
                join_sent = 1
                st_t = time.time()
            elif join_sent == 1:
                if (time.time() - st_t) >= NO_JOIN_ACK:                                                      # timed out getting HEARTBEAT for new join             
                    sm = make_join_msg(did=MY_DEVICE, b)                                                     # resend JOIN until master is using new passwords and sent HB
                    sb = dict2json_str_byte(sm, rn)
                    send_msg(sb)  
                    st_t = time.time() 
                else:
                    prev_pw_list = pw_list                                                                  # save the current pw list
                    pw_list = b                                                                             # make new passwords the password HB is sent after this
                    msg_json = json_str2dict(data, rn)   
                    # if msg_json['type'] == 'HEARTBEAT' and msg_json['device'] == MY_DEVICE and msg_json['seq'] == b : 
                    if msg_json['type'] == 'HEARTBEAT' and msg_json['device'] == MY_DEVICE: 
                        join_sent = 0                                                                       # sucessfully re-joined
                    else:
                        pw_list = prev_pw_list                                                              # put passwds back as no HB confirmation yet                
            else:
                print(f"[RECV] {recv_time:.3f} | {device} {msg_json['type']}")
        except Exception as e:
            print(f"[ERROR] Parse failed: {e}")
    if so == 1:
        sock.close()
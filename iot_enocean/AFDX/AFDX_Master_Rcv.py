# Master receiver modified secure AFDX protocol
#
import json
from enum import Enum
from twofish import Twofish
import triplesec
from salsa20 import XSalsa20_xor
import random
import socket
import time
import signal
# https://github.com/FiloCara/pyS7
#
from pyS7 import S7Client, DataType, S7Tag, MemoryArea

# write the values received on AFDX net from the devices to a Siemens S7 Series plc
def write_vals_s7(v1, v2, v3, v4):

    # Create a new 'S7Client' object to connect to S7-300/400/1200/1500 PLC.
    # Provide the PLC's IP address and slot/rack information
    client = S7Client(address="192.168.5.100", rack=0, slot=1)

    # Establish connection with the PLC
    client.connect()

    # Define area tags to write
    tags = [
        S7Tag(memory_area=MemoryArea.DB, db_number=5, data_type=DataType.REAL, start=50, bit_offset=0, length=4) # => Sequence of REAL of length 4 starting at address 50 of DB5 
    ]

    # Defines values to write
    values = [
        (v1, v2, v3, v4)
    ]

    # Write data to the PLC using tags and values
    client.write(tags=tags, values=tags)
    
UDP_PORT = 5005
BUFFER_SIZE = 512

# devices and cipher code
dev_dict = {"WIND_SPEED_01":0, "TEMP_PROBE_01":0, "WIND_SPEED_02":0, "TEMP_PROBE_02":0}

# devices and each cipher password generated 
pw_dict = {"WIND_SPEED_01":["TWOFISH", "TRIPLESEC", "XSALSA20"], "TEMP_PROBE_01":["TWOFISH", "TRIPLESEC", "XSALSA20"], "WIND_SPEED_02":["TWOFISH", "TRIPLESEC", "XSALSA20"], "TEMP_PROBE_02":["TWOFISH", "TRIPLESEC", "XSALSA20"]}

# period for asking each unit to leave the network and re-join with new passwords
UNIT_CHANGE_PW_PERIOD = 10000

# period for sending a HB (used only to confirm a password change, but i send periodically in case you want a keep alive or it was missed)
HB_PERIOD = 10

# types of encryption we support
class choose_cipher(Enum):
    NONE = 0
    TWOFISH = 1
    TRIPLESEC = 2
    XSALSA20 = 3
NUM_OF_CIPHER = 4

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

# makes a join message which requests to join data stream with new passwords 
def make_join_msg(did="TMP_PROBE_01", rn):
    jn_msg = {
      "type": messageTypes['JOIN'],
      "device": did,
      "seq": rn
    }
    return jn_msg

# makes a leave message which requests the remote unit to leave and join with new passwords
def make_leave_msg(did="TMP_PROBE_01"):
    lv_msg = {
      "type": messageTypes['LEAVE'],
      "device": did,
    }
    return lv_msg

# makes HB message to acknowledge change of cipher and send data in that encryption    
def make_hb_msg(did="TMP_PROBE_01", rn):
    sync_msg = {
      "type": messageTypes['HEARTBEAT'],
      "device": did,
      "timestamp": time.time()
      # "seq": rn                                 - we wont send incase it gets seen
    }
    return sync_msg

# makes ACK message to acknowledge change of cipher and send data in that encryption    
def make_ack_msg(did="TMP_PROBE_01", rn):
    sync_msg = {
      "type": messageTypes['ACK'],
      "device": did,
      "timestamp": time.time()
      "seq": rn
    }
    return sync_msg

# makes data message    
def make_data_msg(data_v=0, did="TMP_PROBE_01"):
    data_msg = {
      "type": messageTypes['DATA'],
      "device": did,
      "data": data_v
    }
    return data_msg

# makes alarm alert message 
def make_alert_msg(msg="over-temp alarm", did="TMP_PROBE_01"):    
    alert_msg = {
      "type": messageTypes['ALERT'],
      "device": did,
      "message": msg
    }
    return alert_msg

# all passwords and secret keys have been hard-coded below  
#    
# put (JSON) dict to a string to encode and send as a ip message
#
def dict2json_str_byte(dev, json_dict, encryp=choose_cipher.NONE.value):
    json_str = json.dumps(json_dict)
    msg = json_str.encode('utf-8')
    if encryp == choose_cipher.TWOFISH.value:
        # pw = b'*secret*'
        pw = str(pw_dict[dev][0]).encode('utf-8')
        T = Twofish(pw)
        msg = T.encrypt(msg)  
    elif encryp == choose_cipher.TRIPLESEC.value:
        #pw = b'* password *'
        pw = str(pw_dict[dev][1]).encode('utf-8') 
        msg = triplesec.encrypt(msg, pw)     
    if encryp == choose_cipher.XSALSA20.value:
        IV = urandom(24)
        #KEY = b'*secret**secret**secret**secret*'
        KEY = pw = str(pw_dict[dev][2]).encode('utf-8')
        msg = XSalsa20_xor(msg, IV, KEY)         
    return msg

# received ip message -> decoded string to JSON dict
#
def json_str2dict(dev, msg, decryp=choose_cipher.NONE.value):
    if encryp == choose_cipher.TWOFISH.value:
        # pw = b'*secret*'
        pw = str(pw_dict[dev][0]).encode('utf-8')
        T = Twofish(pw)
        msg = T.decrypt(msg).decode()  
    elif encryp == choose_cipher.TRIPLESEC.value:
        #pw = b'* password *'
        pw = str(pw_dict[dev][1]).encode('utf-8')        
        msg = triplesec.decrypt(msg, pw).decode()     
    elif encryp == choose_cipher.XSALSA20.value:
        IV = urandom(24)
        #KEY = b'*secret**secret**secret**secret*'
        KEY = pw = str(pw_dict[dev][2]).encode('utf-8')
        msg = XSalsa20_xor(msg, IV, KEY).decode() 
    json_str = msg.decode('utf-8')
    json_dict = json.loads(json_str)
    return json_dict

# send out message as a braodcast to each station on the network
def send_msg(message):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock2:
        sock2.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)              # this allows as many masters as you like can change to specific ip only
        sock2.sendto(message, (BROADCAST_IP, UDP_PORT))
        print(f"Broadcast sent @ {time.time():.3f}")

RUN_F = True
def sig_handler(s):
    global RUN_F
    RUN_F = False

if __name__ == "__main__":
    ws001, ws002, tt001, tt002 = 0
    signal.signal(signal.SIGUSR1, sig_handler)
    signal.signal(signal.SIGUSR2, sig_handler)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)                      # open UDP port for receiving
    sock.bind(("", UDP_PORT))                                                    # bind socket for listening
    sock.settimeout(10)                                                          # socket timetime set to 10 secs   
    print("[RECEIVER] Listening on port", UDP_PORT)
    st_time = time.time()                                                        # take timer reference point
    st_time1 = time.time()                                                        # take timer reference point   
    while RUN_F == True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)                              # read port for chars
            recv_time = time.time()
            message = data.decode("utf-8")
            device = addr[0]
            if "ACK" in message:                                                  # ACK sent back for SYNC
                msg_json = json_str2dict(0, data)
                dev_nm = msg_json["device"]                                       # get device name
                ci = msg_json["seq"]                                              # get cipher for that device and message
                dev_dict[dev_nm] = ci                                             # save that in the device cipher dictionary           
            else:
                msg_json = json_str2dict(dev_nm, data, dev_dict[dev_nm])          # use the cipher related to the last ACK message
                try:                
                    if msg_json['type'] == 'DATA':                                # received cipher encoded data
                        print(msg_json['device']," ",msg_json['data']," ",device)
                        if msg_json['device'] == "WIND_SPEED_01":
                            ws001 = msg_json['data']
                        elif msg_json['device'] == "TMP_PROBE_01"
                            tt001 = msg_json['data']   
                        if msg_json['device'] == "WIND_SPEED_02":
                            ws002 = msg_json['data']
                        elif msg_json['device'] == "TMP_PROBE_02"
                            tt002 = msg_json['data']                            :
                        write_vals_s7(ws001, tt001, ws002, tt002)
                    elif msg_json['type'] == 'JOIN':                              # received a message to say join with a new password list
                        pw_dict[msg_json['device']] = msg_json['seq']             # change to use new passwords list  
                        am = make_hb_msg(msg_json['device'], msg_json['seq'])     # send HEARTBEAT to confirm the password changes (otherwise remote keeps sending JOIN)
                        ast = dict2json_str_byte(msg_json['device'], am, dev_dict[dev_nm])            # use same cipher
                        send_msg(ast)                        
                except:                                                           # didnt decode so try each cipher ?
                    for d in dev_dict:                                            # for each remote unit
                        for i in range(0,NUM_OF_CIPHER):                          # check all the ciphers its safer than just all devices in the table
                            msg_json = json_str2dict(d, data, i)
                            try:                
                                if msg_json["type"] == 'DATA':
                                    print(msg_json['device']," ",msg_json['data']," ",device)
                                    break
                                elif msg_json['type'] == 'JOIN':
                                    pw_dict[msg_json['device']] = msg_json['seq']             # change to use new passwords list  
                                    am = make_hb_msg(msg_json['device'], msg_json['seq'])     # send HEARTBEAT to confirm the password changes (otherwise remote keeps sending JOIN)
                                    ast = dict2json_str_byte(msg_json['device'], am, i)
                                    send_msg(ast)
                            except:
                                pass                

        except Exception as e:
            print(f"[ERROR] Socket read and Parse failed: {e}")

        # periodically send LEAVE messages to each remote unit to change its password and re-join the network 
        # if the device does not support the LEAVE message the password changes will be ignored
        #        
        ct = time.time()
        if (ct - st_time) >= UNIT_CHANGE_PW_PERIOD:
            lm = make_leave_msg(did="TMP_PROBE_01") 
            lst = dict2json_str_byte("TMP_PROBE_01", lm)                 # use no cipher
            send_msg(lst)    
        elif (ct - st_time) >= (UNIT_CHANGE_PW_PERIOD*2):
            lm = make_leave_msg(did="WIND_SPEED_01") 
            lst = dict2json_str_byte("WIND_SPEED_01", lm)                 # use no cipher
            send_msg(lst)    
        elif (ct - st_time) >= (UNIT_CHANGE_PW_PERIOD*3):
            lm = make_leave_msg(did="WIND_SPEED_02") 
            lst = dict2json_str_byte("WIND_SPEED_02", lm)                 # use no cipher
            send_msg(lst) 
        elif (ct - st_time) >= (UNIT_CHANGE_PW_PERIOD*4):
            lm = make_leave_msg(did="WIND_SPEED_02") 
            lst = dict2json_str_byte("WIND_SPEED_02", lm)                 # use no cipher
            send_msg(lst) 
            st_time = ct                                                  # start timing again 
        # periodically send HEARTBEAT messages
        if (ct - st_time1) >= HB_PERIOD:
            for p in pw_dict:        
                am = make_hb_msg(p, pw_dict[p])                   # send HB to confirm the password changes (otherwise remote keeps sending JOIN)
                ast = dict2json_str_byte(p, am, dev_dict[p])      # use same cipher
                send_msg(ast) 
            st_time1 = ct                        
    sock.close()
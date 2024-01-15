# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#
# camera side is splitting the file to chunks of the size specified CHUNK_SZ=1024
#
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
import chunk
import os
import xbee
import subprocess
from subprocess import PIPE
import psutil 
import sys

CHUNK_SZ=1024                               # default chunk size will warn and change if network does not support
HOME_ROOT='/usr/mark/zigbeecam'
CAM_PIC_FILE=‘sony_camera.ARW’

# PL (transmission power level)] <XBee3><S2C><S2>
# Set or view the power level at which the device transmits conducted power.
# (Note) When operating on channel 26 (CH = 0x1A), the output power will be limited and
# Cannot exceed 8 dBm regardless of PL setting.
# Setting value XBee3-PRO XBee3 Non-PRO S2C S2 (boost mode enabled)
# 0 -5 dBm -5 dBm -5 dBm -8 dBm
# 1 +3 dBm -1 dBm -1 dBm -4 dBm
# 2 +8 dBm +2 dBm +1 dBm -2 dBm
# 3 +15dBm +5 dBm +3 dBm 0 dBm
# 4 +19dBm +8 dBm +5 dBm +2 dBm
# Default
# 4
# use if you need restricted power
def set_power_limit(lim=4):
    xbee.atcmd('PL', lim)

# The following commands affect the UART serial interface
# ［BD(UART baud rate) <XBee3><S2C><S2
# This command sets the serial interface baud rate for communication between the device's UART port and the host.
# The device interprets any value between 0x12C and 0x0EC400 as the custom baud rate.
# The custom baud rate is not guaranteed and the device will attempt to find the closest baud rate achievable.
# After setting a non-standard baud rate, query the BD to find the actual operating baud rate before applying the change.
# The following table shows some examples of sent BD parameters versus stored parameters.
# Parameter Description
# 0x0 1200 baud
# 0x1 2400 baud
# 0x2 4800 baud
# 0x3 9600 baud
# 0x4 19200 baud
# 0x5 38400 baud
# 0x6 57600 baud
# 0x7 115200 baud
# 0x8 230,400 baud
# 0x9 460,800 baud
# 0xA 921,600 baud
# Parameter range
# Standard baud rate : 0x0-0x07 <S2
# Standard baud rate : 0x0-0x0A <XBee3><S2C>
# Non-standard baud rate : 0x80 -0x0E1000 <S2>
# Non-standard baud rate : 0x12C-0x0EC400 <XBee3> <S2C>
# Default
# 0x03 (9600 baud)
#
# ［NB (Parity)] <XBee3><S2C><S2
# Sets or reads the serial parity setting for UART communication.
# Parameter Description
#　0 No parity
#　1 Even parity
#　2 Odd parity
#3 Mark parity (always "1") <S2C><S2
#
# ［SB (stop bit)] <XBee3><S2C><S2
# Sets or displays the number of stop bits for UART communication.
# Parameter Description
#　0 One stop bit
#　1 Two stop bits
# Default
#　0
#
def set_comms_params(baud=0xA, par=1, sb=0):
    network_settings = {"BD": baud, "NB": par, "SB": sb}
    for command, value in network_settings.items():
        xbee.atcmd(command, value)
    xbee.atcmd("AC")                                 # write changes
    time.sleep(1)

# ［AP (API enabled) <XBee3><S2C><S2
# Determines the API mode of the UART interface.
# Parameter Description <XBee3><S2C>
#　0 API disabled (operates in transparent mode)
#　1 API Enabled
#　2 API enabled (with escape control character)
# Default
#　Default
# Parameter Description <S2
#　1 API enabled
#　2 API enabled with escape control characters
# Default
#　1
# ［AO (API option) <XBee3><S2C><S2
# Sets the serial output options for received API frames.
# The current option selects the type of received API frame to send the UART of the received RF data packet.
# Unused bits should be left clear to prevent future API options from being
# not accidentally enabled.
# Bit Field:
# Bit Description <XBee3>
#　0 0 =Native API output (0x90 frame type)
#　　　　1 = Explicit API output (0x91 frame type)
#　1 Pass-through of unsupported ZDO requests.
#　2 Supported ZDO request pass-through.
# 3 Binding request pass-through.
# Parameter range
# 0 - 0xFF
# Parameter Description <S2C><S2> 0
#　　0 Default API receive indicator enabled
#　　1 Explicit receive data indicator API frame enable (0x91)
#　　3 As with Simple_Desc_req, Active_EP_req, and Match_Desc_req,
#　　　　　　Enables ZDO passthrough of ZDO requests to serial ports not supported by the stack.
# Parameter range
# 0 - 3
# Default
# 0
# ［RO (Packetization Timeout)] <XBee3><S2C><S2
# Sets or reads the number of characters of silence between characters required before sending
# Starts when the device is operating in transparent mode.
# Use RO only when the device is operating in transparent mode (AP = 0).
# To transmit characters as they arrive rather than buffering them into a single RF packet, set RO to 0.
# The RO command applies only to transparent mode, not to API mode.
# Parameter range
# 0 - 0xFF (x number of characters)
# Default
# [TO (send option)] <XBee3><S2C><S2>
# Advanced options used for outgoing data transmission for devices operating in transparent mode (AP = 0)
# Bitfield to set.
# When operating in API mode, if the API frame's Send Options field is 0,
# The TO parameter value is used instead.
# Bit field:
# Bit Description
# 0 Retries and root repair have been disabled.
# 5 Enable APS end-to-end encryption (if EE = 1).
# 6 Use extended timeouts.
# Parameter range
# 0 - 0xFF
# Default
# 0
def set_api_options(api=1, ao=0, ro=0xFF, to=0):
    network_settings = {"AP": api, "AO": ao, "RO": ro, "TO": to}
    for command, value in network_settings.items():
        xbee.atcmd(command, value)
    xbee.atcmd("AC")                                 # write changes
    time.sleep(1)
    
def set_api_options_encrypt(api=0, ao=0, ro=0xFF, to=5):
    network_settings = {"AP": api, "AO": ao, "RO": ro, "EE" : 1, "TO": to}
    for command, value in network_settings.items():
        xbee.atcmd(command, value)
    xbee.atcmd("AC")                                 # write changes
    time.sleep(1)

# ［NP (maximum packet payload bytes)]<XBee3><S2C><S2
# Reads the maximum number of RF payload bytes that can be sent in a transmission based on the current parameter settings.
# Using APS encryption (API transmit option bit enabled) reduces the maximum payload size by 9 bytes.
# Use source routing (AR < 0xFF) to further reduce the maximum payload size by passing through a number of hops.
# further reducing the maximum payload size.
# (Note: NP returns a hexadecimal value. For example, if NP returns 0x54, this corresponds to 84 bytes
def get_max_packet_sz():
    return int(xbee.atcmd('NP'))   

# (10) FS INFO
#　Reports on the size of the file system,
#　displays the number of bytes in use, the number of bytes available, bad and total marks.
#　As with most multi-line AT command output, the report ends with a blank line.
# Example output:
#　　　204800 used
#　　　695296 free
#　　　0 bad
#　　　90009696 total
def get_remote_free_space():
    file_sys_info = xbee.atcmd("FS INFO")
    pos1 = file_sys_info.find('\n')            # find first CR 
    aa = file_sys_info[pos1+1:]                # line after first \n
    pos2 = aa.find('\n')                       # advance to next CR
    free_line = aa[:pos2]                      # info before the 2nd \n should be free
    exp="\d+"                                  # get numerical digits
    bytes_free=int(re.findall(exp, free_line)) # get the number of bytes free
    return bytes_free

def get_file_size(file):
    file_size = os.path.getsize(file)  
    return file_size

def get_local_free_space():
    dsk = psutil.disk_usage('/')
    return dsk.free 
    
# Takes the picture and chunks it to managable parts to read over the zigbee link
#
def take_pictures_and_chunk():
    # change dir to the home
    os.chdir(HOME_ROOT)

    # take a picture and create ARW file
    # example here i will call a picture take on the sony alpha
    CMD = "/home/mark/sony/sony_take_picture"
    proc = subprocess.run(CMD, shell=True, stdout=PIPE, stderr=PIPE)
    print(proc.stdout)
    
    # check handshake value
    # if reading ... wait
    loop = 1
    while (loop == 1):
        file_names = os.listdir(‘status/’) 
        if not read_status.find("reading") == -1:
            loop = 0
        time.sleep_us(100)
    
    # if ready
    # set handshake value to writing
    output="writing"
    with open(‘writing.txt’, ‘w’) as f: 
        f.write(output)

    # take the sony camera ARW image and split it into manageable packets to transmit of the radio
    with open(CAM_PIC_FILE, ‘rb’) as f:
        for i, piece in enumerate(chunk.read(f, CHUNK_SZ)):
            with open(‘chunks/out.{}’. format(i), ‘wb’) as inner:
                inner.write(piece)

    # set handshake to ready
    os.remove(‘status/writing.txt’) 

# Display PAN ID, extended PAN ID, and channel number when connected
#
def get_pan_id():
    operating_network = ["OI", "OP", "CH", "PP"]
    print("Operating network parameters:")
    for cmd in operating_network:
        print("{}: {}".format(cmd, xbee.atcmd(cmd)))
        
# set up zigbee
#
xbee.atcmd("NI", "CameraController")
set_comms_params()
# set BT : 1 if you want to enable bluetooth
# FK = 0 clear public key
network_settings = {"CE": 0, "ID": 0x776, "EE": 0, "NJ": 0xFF, "BT": 0, "FK": 0}      # Join network: router setting CE=0
for command, value in network_settings.items():
    xbee.atcmd(command, value)
xbee.atcmd("AC")                                 # write changes
time.sleep(1) 

# wait for network join
#
while True:
    status = xbee.atcmd('AI')     # Check network participation status
    print('.',end='')
    if status == 0x00:            # Exit the loop when in participating state
        break
    xbee.atcmd('CB',0x01)         # Commissioning (network participation)
    time.sleep_ms(2000)           # 2 seconds wait time processing
print('\n \033[32m ==============Joined================= \033[0m')

get_pan_id() # print network info

# check network maximum packet size and set the chunker accordingly
act_pkt_sz = get_max_packet_sz()
if ( CHUNK_SZ > act_pkt_sz ):
    print("\033[31m max packet size read is {}: you wanted {} bytes - clipped to {} \033[0m".format(act_pkt_sz, CHUNK_SZ, act_pkt_sz))
    CHUNK_SZ = act_pkt_sz
    
# sequence each picture take every DELTA_TM ms
#
DELTA_TM=100
file_names = os.listdir(‘status/’) 
if not read_status.find("writing") == -1:       # just in case this process terminated abnormally look for a left lock file
    os.remove(‘status/writing.txt’)             # clear the locked handshake
    time.sleep_us(int(DELTA_TM/1.5))            # sleep sometime to allow clear up from remote end
fsp = get_local_free_space()                    # ensure there is enough disk space to chunk it for sending
fsz = get_file_size(CAM_PIC_FILE)
if (fsp <= fsz):
    print("\033[31m filesize {}: exceeds free {} \n ---- aborted ---- \033[0m".format(fsz, fsp))
    sys.exit(-2)    
take_pictures_and_chunk()
start = time.ticks_ms()                         # Get value from millisecond counter
time.sleep_us(int(DELTA_TM/1.2))                # wait a reasonable time
while True:
    delta = time.ticks_diff(time.ticks_ms(), start)
    if (delta >= DELTA_TM):
        take_pictures_and_chunk() 
        start = time.ticks_ms()   
    else:
        time.sleep_us(100) 
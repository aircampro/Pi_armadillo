# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#
# Remote reader for the files chunked from the raw camera file over zigbee
#
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
import xbee
import re
import os
import time

# hashlib.algorithms_available
# {'blake2s', 'md4', 'sha512', 'sha3_256', 'whirlpool', 'blake2b', 'blake2s256', 'sha3_512', 'md5', 
# 'sha384', 'md5-sha1', 'sha3_384', 'mdc2', 'ripemd160', 'sha256', 'sha224', 'shake_128', 'sha3_224', 
# 'sha1', 'shake_256', 'blake2b512'}
import hashlib
import sys

HOME_ROOT='/usr/mark/zigbeecam'
NUM_RETRY=3

# ［D6 (AD6 / DIO6 / TH_SPI_ATTN configuration) <XBee3><S2C><S2
# Sets or displays the DIO1 / AD1 configuration (Micro pin 27 / SMT pin 29 / TH pin 16).
# Parameter Description <XBee3><S2C>
#　0 Disabled
#　1 RTS FLow Control
#　2 ADC
# 3 Digital input
# 4 Output LOW on digital output
# 5 HIGH on digital output
# 6 I2C SCL (available in MicroPython only) <Xbee3
#
# controls light LED on D6 from https://www.switch-science.com/products/1052/
#
REMOTE_LED_ACTIVE=1      # set to 1 if its active
def set_led_low():
    xbee.atcmd('D6', 4)  # low

def set_led_high():
    xbee.atcmd('D6', 5)  # set high

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

def show_power_limit(lim=4):   
    print("{}dbm".format(xbee.atcmd('PP'))) 

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
    return int(xbee.atcmd('NP'),16)   

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
    
# calculate the sha256 for the file we got using YMODEM protocol over zigbee
#
BLOCK_SZ=0x200                                 # default block size will warn and change if network does not support
def calc_local_sha256(file_num):
    # set the algo
    algo = 'sha256'

    # create a algo space
    h = hashlib.new(algo)

    # Decide the length to divide as an integer multiple of the block size e.g. 512=0x200
    Length = hashlib.new(algo).block_size * BLOCK_SZ

    # Prepare large binary data
    path = 'out.'+file_num
    with open(path,'rb') as f:
        BinaryData = f.read(Length)

        # loop until there is no more data
        while BinaryData:

            # Calculate by adding to hash object
            h.update(BinaryData)

            # Load more data
            BinaryData = f.read(Length)

    # Outputs the hash object in hexadecimal
    local_sha256=h.hexdigest()
	return local_sha256

# get the picture file in chunks from the camera controller
#
def get_picture_file():
    # change local and remote dir to the home one
    os.chdir(HOME_ROOT)
    xbee.atcmd('FS CD '+HOME_ROOT)
    
    # wait if we are writing on the picture taking robot / drone
    loop = 1
    while (loop == 1):
        read_status=xbee.atcmd('FS LS status/')
        if not read_status.find("writing") == -1:
            loop = 0
        time.sleep_us(100)
 
    # set a file to indicate we are reading
    output="reading"
    if (REMOTE_LED_ACTIVE == 1):
        set_led_high()
    with open(‘reading.txt’, ‘w’) as f: 
        f.write(output)
    xbee.atcmd('FS CD status')
    xbee.atcmd('FS PUT reading.txt')
   
    # change directory and list the files in it.
    xbee.atcmd('FS CD ../chunks')
    file_list=xbee.atcmd('FS LS')
    digits=r"\d+"
    file_ext=re.findall(digits, file_list))

    #does GET on each file out.<list> strip the extension to clean the output from the DIR
    # make the output folder if not existing
    if !os.path.exists('chunks/'):
        os.mkdir('chunks/')
    os.chdir('chunks/')
    fail_get=0                  # set flag to 1 if we get a file with wrong sha retry number of times
    file_ok=-2                  # indicate the file has correcr checksum
    for fnum in file_ext:
        # FS GET filename and verify the SHA256 checksum
        retry=NUM_RETRY
        while ( retry > 0 ) and (fail_get == 0):
            free_sp = get_local_free_space()                                   # check we got enough space to upload the file
            if not (free_sp >= (int(BLOCK_SZ,16)*3)):                          # ensure chunk file size plus 1 block
                print("\033[31m NO SPACE ON DISK got {} need {} \033[0m".format(free_sp,(int(BLOCK_SZ,16)*3)))
            else:
                xbee.atcmd('FS GET out.'+fnum)
                # check SHA-256 of the file
                remote_sha256=xbee.atcmd('FS HASH out.'+fnum)
                time.sleep_us(10)
                local_sha256=calc_local_sha256(fnum)
                if (remote_sha256 == local_sha256):
                    retry = file_ok
                else:
                    retry = retry - 1
                    time.sleep_us(100)
        if (retry != file_ok):
            fail_get=1                              # set fail if we re-tryed enough on one file
   
    # remove the lock file to say we are no longer reading - if you dont to loose data then dont do this alarm and move and try again
    #
    xbee.atcmd('FS CD ../status')
    xbee.atcmd('FS RM reading.txt')
    if (REMOTE_LED_ACTIVE == 1):
        set_led_low()
        
    # concatenate those files read from the Zigbee unit ro a RAW image ARW as from the original camera
    #
    os.chdir(HOME_ROOT)

    # make the output folder if not existing
    if !os.path.exists('joined/'):
        os.mkdir('joined/')
    
    if (fail_get == 0):    
        # code to create the ARW file again from the chunks read from the radio
        with open('joined/camera_image.ARW', mode='wab') as b:
            file_names = os.listdir(‘chunks/’) 
            for file in file_names: 
                with open('chunks/'+file, mode='rb') as a:
                    a_read = a.read()
                    b_write = b.write(a_read)

# Display PAN ID, extended PAN ID, and channel number when connected
#
def get_pan_id():
    operating_network = ["OI", "OP", "CH", "PP"]
    print("Operating network parameters:")
    for cmd in operating_network:
        print("{}: {}".format(cmd, xbee.atcmd(cmd)))

# check we have enough space to get the files
#
fsp = get_local_free_space()                    # ensure there is enough disk space to chunk it for sending
fsz = 1024*1024*6                               # set this to a value that indicates free space on drive
if (fsp <= fsz):
    print("\033[31m filesize {}: exceeds free {} \n ---- aborted ---- \033[0m".format(fsz, fsp))
    sys.exit(-2)  
    
# set up zigbee
#
xbee.atcmd("NI", "Coordinator")
set_comms_params()
# form network (SM must be set to 0 to set CE to 1): coordinator setting
# set BT : 1 if you want to enable bluetooth
# FK = 0 clear public key
network_settings = {"SM": 0, "CE": 1, "ID": 0x777, "EE": 0, "NJ": 0xFF, "BT": 0, "FK": 0}
for command, value in network_settings.items():
    xbee.atcmd(command, value)
xbee.atcmd("AC")                                 # write changes
time.sleep(1) 

# wait for network join
#
while True:
    status = xbee.atcmd('AI')                   # Check network participation status
    print('.',end='')
    if status == 0x00:                          # Exit the loop when in participating state
        break
    xbee.atcmd('CB',0x01)                       # Commissioning (network participation)
    time.sleep_ms(2000)                         # 2 seconds wait time processing
print('\n \033[33m ==============Joined================= \033[0m')

get_pan_id() # print network info
    
# check packet size of network and adjust sha256 block size
act_pkt_sz = get_max_packet_sz()
if ( int(BLOCK_SZ, 16) > act_pkt_sz ):
    print("\033[31m max packet size read is {}: you need at least {} \033[0m".format(act_pkt_sz, BLOCK_SZ))
    BLOCK_SZ=hex(act_pkt_sz/2)                  # set the block size half the maximum tx packet size
    
# sequence each picture get at a duration of every DELTA_TM ms
#
DELTA_TM=100
start = time.ticks_ms()                         # Get value from millisecond counter
time.sleep_us(int(DELTA_TM/1.2))                     # wait a reasonable time
while True:
    delta = time.ticks_diff(time.ticks_ms(), start)
    if (delta >= DELTA_TM):
        get_picture_file() 
        start = time.ticks_ms()   
    else:
        time.sleep_us(100)  

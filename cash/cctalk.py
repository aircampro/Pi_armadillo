# examples using the CCtalk library to communicate with to various money processing device, including coin/bill acceptors and coin hoppers
#
# library used is here on github
# https://github.com/Baldanos/ccTools/blob/master/ccTalk.py
#
# examples of messages can be seen here with further info https://balda.ch/posts/2013/Aug/11/cctalk-part1/
#
import serial
import ccTalk
import time

# Set Parameter
deviceName = '/dev/tty.Bluetooth-Incoming-Port'    # search by `ls -l /dev/tty.*`
baudrateNum = 9600
timeoutNum = 3
print("===== Set Parameter Complete =====\n")

# Read Serial
def read_ser(num_bytes=6):
    readSer = serial.Serial(deviceName, baudrateNum, timeout=timeoutNum)
    string = readSer.read(num_bytes)
    print(string)
    readSer.close()
    print("===== Read Serial Complete =====\n")

# Write Serial
def write_ser(serialCommand):
    writeSer = serial.Serial(deviceName, baudrateNum, timeout=timeoutNum)
    writeSer.write(serialCommand.encode())
    writeSer.close()
    print("===== Write Serial Complete =====\n")


# create the ccTalk Message default is you are address 1 and destination is 2
m = ccTalk.ccTalkMessage()
print m
# <cctalk src=1 dst=2 length=0 header=0>
print m
# <cctalk src=1 dst=5 length=0 header=0>

# now set a simple poll message and check it
# headerTypes 254 : 'Simple poll',
#[    02     ] [  00  ] [  01  ] [  fe  ]        [  ff   ]
#[destination] [length] [source] [payload] [data] [cheksum]
m.setPayload(254)
# print of set
m.getPayloadType()
#'Simple poll'
print(m)
# a poll from 1 to 2 '\x02\x00\x01\xfe\xff'

# now change the destination to machine 5 for example
m = ccTalk.ccTalkMessage(1,5)
# 246 : 'Request manufacturer id',
#[    05     ] [  00  ] [  01  ] [  f6  ]        [  xx   ]
#[destination] [length] [source] [header] [data] [cheksum]
m.setPayload(246)
write_ser(m)
ss=read_ser(8)
#[    01     ] [  03  ] [  05  ] [  00  ] [4e5249] [  ff   ]
#[destination] [length] [source] [header] [ data ] [cheksum]
print(ss)
b=bytes.fromhex(ss[8:14])
print("manuafacturer id : ", b.decode('utf-8'))

# headerTypes 240 : 'Test solenoids',
m.setPayload(240)
write_ser(m)
time.sleep(1)

# 239 : 'Operate motors',
m.setPayload(239)
write_ser(m)
time.sleep(1)

# 238 : 'Test output lines',
m.setPayload(238)
write_ser(m)
time.sleep(1)

#  232 : 'Perform self:check',
m.setPayload(232)
write_ser(m)
time.sleep(1)

# 1 : 'Reset device',
m.setPayload(1)
write_ser(m)
time.sleep(1)

# 148 : 'Read opto voltages',
m.setPayload(148)
m.sigmode = 1           # set the sigmmode 1=use CRC or 0=checksum
write_ser(m)
time.sleep(1)

# 127 : 'Request money out',
m.setPayload(127)
write_ser(m)
time.sleep(1)

# 125 : 'Pay money out', from unit 6
m = ccTalk.ccTalkMessage(1, 6, 125)
write_ser(m)
time.sleep(1)

# 173 : 'Request thermistor reading', on unit 8
m = ccTalk.ccTalkMessage(1, 8, 173)
write_ser(m)
time.sleep(1)

# for unit 7
#        120 : 'Modify hopper balance', to 1000
#        119 : 'Request hopper balance',
#        118 : 'Modify cashbox value', to 1000
#        117 : 'Request cashbox value',
sets = [120, 119, 118, 117]

for ii in sets:
    if (ii % 2) == 0:                      # its a modify not a request so write the value as its in the even stream
        m = ccTalk.ccTalkMessage(1, 7, ii, 1000)  
    else:        
        m = ccTalk.ccTalkMessage(1, 7, ii)
    # if m.sigmode == 0:                          # chooses CRC or checksum
    #     mm = [ m.destination, m.length, m.source, m.payload, m._calculateChecksum() ]
    # else:
    #     mm = [ m.destination, m.length, m.source, m.payload, m._calculateCRC() ]
    write_ser(m)
    if (ii % 2) == 1:                      # its a request not a modify so read it as its in the odd stream
        a = 1
        while a == 1:
            ms=read_ser(2)                      # read address and length of message
            if int(ms[0]) == 1:                 # destination 1 is for use
                a = 0
            else:
                nextbytes = int(ms[1]) + 3      # 3 is source header and checksum
                ss=read_ser(nextbytes)          # clear this read as its not for us   
        nextbytes = int(ms[1]) + 3      # 3 is source header and checksum                
        ss=read_ser(nextbytes)  
        b=bytes.fromhex(ss[2:(2+int(ms[1]))])
        print("read back data segment : ", b.decode('utf-8'), " for command ",m.headerType)
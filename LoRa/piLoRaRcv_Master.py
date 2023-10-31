# Example of using LoRa Class with raspberry pi
# uses LoRa communication module "ES920LR"
#
import lora
import ast
import time
import struct
import sys

# (bw, sf, timeout)
mode = [
    (3, 12, 5), (3, 11, 5), (3, 10, 4), (3, 9, 3), (3, 8, 2), (3, 7, 2),
    (4, 12, 5), (4, 11, 4), (4, 10, 3), (4, 9, 3), (4, 8, 2), (4, 7, 2),
    (5, 12, 4), (5, 11, 3), (5, 10, 2), (5, 9, 2), (5, 8, 2), (5, 7, 2),
    (6, 12, 3), (6, 11, 3), (6, 10, 2), (6, 9, 2), (6, 8, 2), (6, 7, 2)
]

lr = lora.LoRa()

def printable(l):
    x = struct.unpack(str(len(l)) + 'b', l)
    y = ''
    for i in range(len(x)):
        if x[i] >= 0:
            y = y + chr(x[i])
    return y

def sendcmd(cmd):
    # print(cmd)
    lr.write(cmd)
    t = time.time()
    while (True):
        if (time.time() - t) > 5:
            print('panic: %s' % cmd)
            exit()
        line = lr.readline()
        if 'OK' in printable(line):
            # print(line)
            return True
        elif 'NG' in printable(line):
            # print(line)
            return False

def setMode(bw, sf):
    lr.write('config\r\n')
    lr.s.flush()
    time.sleep(0.2)
    lr.reset()
    time.sleep(1.5)

    line = lr.readline()
    while not ('Mode' in printable(line)):
        line = lr.readline()
        if len(line) > 0:
            print(line)

    # send the set-up commands https://ambidata.io/samples/network/lora-2/
    sendcmd('2\r\n')
    sendcmd('bw %d\r\n' % bw)
    sendcmd('sf %d\r\n' % sf)
    sendcmd('q 2\r\n')
    sendcmd('w\r\n')

    lr.reset()
    print('LoRa module set to new mode')
    time.sleep(1)
    sys.stdout.flush()

# continously read and print the microchip flip&click LoRa sensor data
while (True):
    rssi = [None] * len(mode)
    for i in range(len(mode)):
        print('setMode(bw: %d, sf: %d)' % (mode[i][0], mode[i][1]))
        setMode(mode[i][0], mode[i][1])

        t = None if i == 0 else mode[i][2]
        timeout = False
        start = time.time()
        while (True):
            while (True):
                line = lr.readline(t)
                # print(line)
                # sys.stdout.flush()
                if len(line) == 0:                      # TIMEOUT
                    timeout = True
                    break
                if len(line) >= 14:                     # 'rssi(4bytes),pan id(4bytes),src id(4bytes),\r\n length of the header
                    break
            if timeout == True:
                rssi[i] = None
                print('TIMEOUT')
                break;
            data = lr.parse(line)                       # 'rssi(4bytes),pan id(4bytes),src id(4bytes),temp_humidity
            print(data)
            # decode the temperature data from the sensor
            tmp = bytes_to_decimal(*data[3][0:2])
            hum = bytes_to_decimal(*data[3][2:4])
            slot = chr(data[3][4:5])
            rssi[i] = data[0]
            pan_id = data[1]
            src_id = data[2]
            print("temperature : ",tmp," humidity : ",hum," slot : ",slot)
            print("pan id : ",pan_id," src_id : ",src_id)
            s = mode[i][2] - (time.time() - start)
            print('sleep: ' + str(s))
            if i != 0 and s > 0:
                time.sleep(s)
            break        

    print(rssi)
    sys.stdout.flush()


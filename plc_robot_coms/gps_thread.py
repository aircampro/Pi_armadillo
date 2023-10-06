#!/usr/bin/python3
# run python thread to get a gps
import serial
import micropyGPS
import threading
import time
import serial.tools.list_ports

def search_com_port():
    coms = serial.tools.list_ports.comports()
    list_com = []
    for com in coms:
        list_com.append(com.device)
    print('Connected COM ports: ' + str(list_com))
    used_port = list_com[0]
    print('Use COM port: ' + used_port)
    return used_port

def serial_data_check( sentence ):
    if sentence[0] != '$':
        return False

    size = len(sentence)
    sumXor = 0x24

    for cd in sentence:
    # readlineの場合"*00"＜cr＞＜lf＞ → 5
        if size <= 5:
            break
        else:
            size -= 1

    sumXor = sumXor ^ ord(cd)

    # readlineの場合"*00"＜cr＞＜lf＞ → -4 ～ -2
    sum = sentence[(len(sentence) - 4):(len(sentence) - 2)]
    # ↓ int(A,para) para is a decimal number if A is 16
    if int(sum , 16) == sumXor:
        return True
    else:
        return False
 
gps = micropyGPS.MicropyGPS(9, 'dd')                                       # Create a MicroGPS object.
                                                                           # Arguments are time zone difference and output format

def rungps(opt=1):                                                         # Read GPS module and update GPS object
    if (opt == 1):
        s = serial.Serial('/dev/serial0', 9600, timeout=10)
    else:
        com_prt = search_com_port()                                        # get the first known device
        s = serial.Serial(com_prt, 9600, timeout=None)                     # this time i set no timeout
        
    s.readline()                                                           # Throw away the first 1 line because it may read half-baked data
    while True:
        sentence = s.readline().decode('utf-8')                            # Read GPS data and convert it to string
        # if sentence[0] != '$':                      Throw away if the beginning is not 'る'
        if serial_data_check( sentence ) == False:                         # check stx as $ length and checksum
            continue
        for x in sentence:                                                 # Parse the read string to add and update data to the GPS object
            gps.update(x)

gpsthread = threading.Thread(target=rungps, args=())                       # Create a thread to execute the above function
gpsthread.daemon = True
gpsthread.start()                                                          # Start the thread

while True:
    if gps.clean_sentences > 20:                                           # output when you have some good data after 20 words
        h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
        print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
        print('lat lon: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
        print('altitude: %f' % gps.altitude)
        print(gps.satellites_used)
        print('Satellite number: (elevation, azimuth, signal-to-noise ratio)')
        for k, v in gps.satellite_data.items():
            print('%d: %s' % (k, v))
        print('')
    time.sleep(3.0)
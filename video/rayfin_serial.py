#!/usr/bin/env python3
# serial interface to rayfin camera to change some values (udp example in C is shown under plc_robot/zybo examples) 
# rayfin ref:- https://www.subcservices.com/services/api/1.13.27/ 
#
import os
import serial
import serial.tools.list_ports
import threading

#-------------------------
# Print a menu of choice 
#-------------------------
def print_options():
    try:
        val = -1
        tim = -1
        npics = -1
        opt = int(input())
    except:
        print("error")

def print_options():
    try:
        val = -1
        tim = -1
        npics = -1
        print("\x1b[H\x1b[J")                 # clear screen go to top
        print("\033[4m")
        print("\033[35m")
        print("\033[43m Choose Camera Action \033[0m")
        print(" ")
        print("1. Change Contrast")
        print("2. Change Lamp Brightness")
        print("3. Change Strobe Brightness") 
        print("4. Take Still pictures") 
        print("5. Start Recording video") 
        print("6. Stop Recording video") 
        print("7. Start Logging") 
        print("8. Stop Logging") 
        print("9. Start Strobe") 
        print("10. Stop Strobe") 
        print("11. Start Lamp") 
        print("12. Stop Lamp") 
        print("13. Strobe shutter speed") 
        print("14. Start RTSP Stream") 
        print("15. Stop RTSP Stream") 
        print("16. Start continous shoot of stills") 
        print("17. Stop continous shoot of stills") 
        print("18. Auto Exposure") 
        print("19. Manual Exposure") 
        print("20. Decrease Exposure") 
        print("21. Increase Exposure") 
        print("22. Set ISO value") 
        print(" ")
        opt = int(input())
        if opt == 1:
            print("enter the value to change the shutter speed in ns to")
            val = int(input())
        elif opt == 2:
            print("enter the value to change the lamp intensity to")
            val = int(input()) % 100
        elif opt == 3:
            print("enter the value to change the strobe to")
            val = int(input()) % 100
        elif opt == 4:
            print("enter the period of ms between takes")
            tim = int(input())
            print("enter the number of pictures to take")
            npics = int(input())
        elif opt == 5:
            print("enter the durqation in ms")
            tim = int(input())
        if opt == 13:
            print("enter the value to change the strobe shutter speed in ns to")
            val = int(input())
        if opt == 22:
            print("enter the value to change the ISO (50-800) to")
            val = int(input())
            val = min(val,800)
            val = max(val,50)
    except:
        print("invalid action")
        opt = -1
    return opt, val, tim, npics

#------------------------------
# function to write serial data
#------------------------------
def serial_write():
    global Serial_Port

    while(1):
        if Serial_Port !='':
            opt, val, tim, npics = print_options()
            if opt == 1 and not (val == -1):
                data = f"ShutterSpeed:{val}"
            elif opt == 2 and not (val == -1):
                data = f"SetLampBrightness:{val}"
            elif opt == 3 and not (val == -1):
                data = f"StrobeBrightness:{val}"
            elif opt == 4 and not (tim == -1 or npics == -1):
                data = f"Repeat:TakePicture|WaitFor:{tim}|{npics}"
            elif opt == 5 and not (tim == -1):
                data = f"StartRecording|WaitFor:{tim}"
            elif opt == 6:
                data = "StopRecording"
            elif opt == 7:
                data = "StartLogging"
            elif opt == 8:
                data = "StopLogging"
            elif opt == 9:
                data = "EnableStrobe"
            elif opt == 10:
                data = "DisableStrobe"
            elif opt == 11:
                data = "LampOn"
            elif opt == 12:
                data = "LampOff"
            elif opt == 13 not (val == -1):
                data = f"StrobeShutter:{val}"
            elif opt == 14:
                data = "StartStreaming"
            elif opt == 15:
                data = "StopStreaming"
            elif opt == 16:
                data = "StartContinuous"
            elif opt == 17:
                data = "StopContinuous"
            elif opt == 18:
                data = "AutoExposure"
            elif opt == 19:
                data = "ManualExposure"
            elif opt == 20:
                data = "DecreaseExposureValue"
            elif opt == 21:
                data = "IncreaseExposureValue"
            elif opt == 22:
                data = f"UpdateISO:{val}"

            data=data.encode('utf-8')
            Serial_Port.write(data)

#----------------------------------
# function to read back serial data
#----------------------------------
def serial_read():
    global Serial_Port
    while(1):
        if Serial_Port !='':
            #data=Serial_Port.read(1)
            data=Serial_Port.readline()
            data=data.strip()
            data=data.decode('utf-8')
            print(data)

#----------------------
# Open
#----------------------
def serial_open():
    global Serial_Port

    serial_ports={}
    for i,port in enumerate(serial.tools.list_ports.comports()):
        serial_ports[str(i)]=port.device
    # RaspberryPi miniUART/dev/ttyAMA0 else /dev/ttyS0
    if '/dev/ttyAMA0' in serial_ports.values():
        serial_ports[str(len(serial_ports))]='/dev/ttyS0'
    port_val = serial_ports[ input(f'enter serial port {serial_ports}:') ]
    boud_val = int(input('enter board rate in bps:'))
    prty_val = input(f'enter parity as 。[N:None, O:Odd, E:Even]:')
    Serial_Port=serial.Serial(port=port_val, baudrate=boud_val, parity= prty_val)
    print(f'open{port_val}/{boud_val}bps/parity:{prty_val}')

if __name__ == '__main__':

    Serial_Port=''

    #port open
    serial_open()

    # define & start rx and tx threads
    thread_1 = threading.Thread(target=serial_write)
    thread_2 = threading.Thread(target=serial_read)
    thread_1.start()
    thread_2.start()

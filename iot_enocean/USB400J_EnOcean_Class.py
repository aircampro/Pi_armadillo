#/usr/local/bin/python3.7
# -*- encoding: utf-8 -*-
# Python Class using ROHM USB400J USB Dongle to communicate with enOcean Sensors
# coding: UTF-8
#
import serial
import binascii
import threading

class Esp3:
    data_length = 0
    data = bytearray(0)
    option_length = 0
    option = bytearray(0)
    packet_type = b'\x00'
    crc8_header = b'\x00'
    crc8_data = b'\x00'

    def print_data(self):
        print('data_length:%s' % self.data_length)
        print('data:%s' % binascii.hexlify(self.data))
        print('option length:%s' % self.option_length)
        print('option:%s' % binascii.hexlify(self.option))
        print('packet type:%s' % binascii.hexlify(self.packet_type))
        print('crc8 header:%s' % binascii.hexlify(self.crc8_header))
        print('crc8 data:%s' % binascii.hexlify(self.crc8_data))

class Erp2Thread(threading.Thread):

    def __init__(self, esp3):
        super(Erp2Thread, self).__init__()
        self.esp3 = esp3

    def run(self):

        sub_telegram = self.esp3.option[0]        #。Send: 3 / receive: 1
        dbm = self.esp3.option[1] * (-1)          # Retrieve dBm from options.Invert positive and negative

        erp2_header = self.esp3.data[0]           # Retrieve 1 byte as header from data

        if erp2_header == ord(b'\x21'): #Originator-ID 32 bit nio Destination-ID、No extended header、1BS telegram 
            originator_id = binascii.hexlify(self.esp3.data[1:5]) #Extract Sensor ID from data
            status_code = self.esp3.data[5] #Retrieve Sensor status from data
            button_status = ""
            door_status = ""

            if originator_id == 'ZZ:XX:XX:XX': #SID for the unit which is Open/close sensor: STM250J
                if status_code == ord(b'\x09'):  #Translate the status of the sensor into words that humans can understand.The STM250J is only close and open, but the sensor of the button seems to be a similar protocol
                    button_status = "Not pressed."
                    door_status = "It is closed"
                elif status_code == ord(b'\x08'):
                    button_status = "Not pressed."
                    door_status = "It is open."
                elif status_code == ord(b'\x01'):
                    button_status = "is being pushed。"
                    door_status = "It is closed."
                elif status_code == ord(b'\x00'):
                    button_status = "It is pressed"
                    door_status = "It is open。"
                print( 'STM250JーID：%sは%s %s' % (originator_id , door_status, button_status) )
            elif originator_id == 'XX:XX:XX:XX': # SID for the unit which is Open/close sensor: CS-EO429J
                if status_code == '08':
                    door_status = 'open'
                elif status_code == '09':
                    door_status = 'close'
                else:
                    door_status = 'non'
                print( 'センサーID：%sは%s' % (originator_id , door_status) )
            elif originator_id == 'XX:XX:XX:XX': # SID for the unit which is Open/close sensor: STM429J
                if status_code == ord(b'\x06'):
                    door_status = 'open'
                elif status_code == ord(b'\x08'):
                    door_status = 'close'
                else:
                    door_status = 'non'
                print( 'STM429JーID：%sは%s' % (originator_id , door_status) )
            elif originator_id == '04:00:7a:fc': # SID for the unit which is STM431J temperature sensor
                val = int(dataList[7],16)
                temp = (255.0-val)/255.0*40.0
                print( 'STM431J ーID：%sは%s' % (originator_id , temp) )
            else:
                print( 'Unknown Device ーID：%s' % (originator_id) )
                continue            

def main():
    ser = serial.Serial('/dev/ttyUSB0',57600,timeout = 1)

    sync_start = False
    while True:

        denbun = ser.read()
        esp3 = Esp3()

        if sync_start or denbun == b'\x55':   # Start syncing with 0x55
            sync_start = True

            denbun = ser.read()    #Read data length
            data_length = 256 * ord(denbun)
            denbun = ser.read()
            data_length = data_length + ord(denbun)

            esp3.data_length = data_length

            denbun = ser.read()   #Read option length
            option_length = ord(denbun)

            esp3.option_length = option_length

            denbun = ser.read()  #Read packet types
            packet_type = denbun

            esp3.packet_type = packet_type

            denbun = ser.read() #CRC8(Header)
            crc8_header = denbun

            esp3.crc8_header = crc8_header

            data = bytearray(data_length)
            for loop_counter in range(data_length):
                denbun = ser.read() #Read data, read length
                data[loop_counter ] = denbun

            esp3.data = data

            option = bytearray(option_length)
            for loop_counter in range(option_length):
                denbun = ser.read() #Option No load, length min load
                option[loop_counter] = denbun

            esp3.option = option

            denbun = ser.read() #CRC8(Data)
            crc8_data = denbun

            esp3.crc8_data = crc8_data

            sync_start = False

        if esp3.packet_type == b'\x0A':  #Only those with packet type ERP2 are processed by thread

            erp2_thread = Erp2Thread(esp3)

            erp2_thread.start()

if __name__ == '__main__':
    main()

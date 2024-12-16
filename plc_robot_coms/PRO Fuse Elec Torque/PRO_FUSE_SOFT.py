#!/usr/bin/env python3
#
# Interface library for Electronic Programmable screwdriver
# Driver for https://www.hp-vanguard.com/products/2015101425/
# Electric Torque Driver - Pro Fuse HP Vanguard Inc
#
# Screws will be fed to the screwdriver by pneumatic air.
# The screwdriver has two servo motors built in - the first motor for z-axis movement (UP/DOWN) 
# and the second one for tightening the screw
#
import socket
import json
# uncomment if you have connection to omron
# from aphyt import omron
# eip_instance = omron.n_series.NSeriesEIP()
# eip_instance.connect_explicit('192.168.251.1')
# eip_instance.register_session()
# eip_instance.update_variable_dictionary()

# Address and port for the Electronic Torque Driver
ETD_IP='192.168.20.99'
ETD_PORT=5000
ETD_PORT_C=5100
  
###################### SET UP Functions ###########################
def UUID_get():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((ETD_IP, ETD_PORT))
            s.send(bytes([0]))
            data = s.recv(1024)
            s.close()
        except:
            print('UUID cant be accessed')
            data = b'\x00'
    if data == b'\xfe':
        print('UUID not accessed correctly')
    return(data)

def Z_Homing(UUID):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((ETD_IP, ETD_PORT))
            s.send(UUID +b'\04')
            data3 = s.recv(1024)
            s.close()
        except:
            print('the command was not excuted')
            data = b'\x00'
            data3 =  b'\x00'
        if data3 == b'\xff':
            print('the command was not excuted correctly')
        elif data3 == b'\xfd':
            print('the command sent is not correct')
    return (data3)

#################### ACTION FUNCTIONS#####################################

def PRO_SCREW_pik():
    data,UUID = Z_Homing()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
       try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID +b'\01' + b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
                data1 = b'\x00'
       except:
           print('the command was not excuted')
       if data1 == b'\xff':
           print('the command was not excuted correctly')
       elif data1 == b'\xfd':
           print('the command sent is not correct')

def PRO_SCREW_in(UUID):
    data = Z_Homing(UUID)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID + b'\01' + b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
                data1 = b'\x00'
        except:
            print('the command was not excuted')
        if data1 == b'\xff':
            print('the command was not excuted correctly')
        elif data1 == b'\xfd':
            print('the command sent is not correct')
    return (data1)

def PRO_SCREW_out(UUID):
    data = Z_Homing(UUID)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID + b'\03' + b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
                data1 =  b'\x00'
        except:
            print('the command was not excuted')
        if data1 == b'\xff':
            print('the command was not excuted correctly')
        elif data1 == b'\xfd':
            print('the command sent is not correct')

    return (data1)
    
################################ ACTION AND RETURN FUNCTIONS ######################################################

def PRO_SCREW_pik_RE():
    data,UUID = Z_Homing()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID +b'\01' + b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
        except:
           print('the command was not excuted')
        if data1 == b'\xff':
           print('the command was not excuted correctly')
        elif data1 == b'\xfd':
           print('the command sent is not correct')

    return (data1)

def PRO_SCREW_in_RE():
    data,UUID = Z_Homing()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID + b'\02' + b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
        except:
            print('the command was not excuted')
        if data1 == b'\xff':
            print('the command was not excuted correctly')
        elif data1 == b'\xfd':
            print('the command sent is not correct')

    return (data1)

def PRO_SCREW_out_RE():
    data,UUID = Z_Homing()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID + b'\03' + b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
        except:
            print('the command was not excuted')
        if data1 == b'\xff':
            print('the command was not excuted correctly')
        elif data1 == b'\xfd':
            print('the command sent is not correct')

    return (data1)

###################################PARAMETER SETTING########################

def PRO_PARA_IN(UUID):
    data = Z_Homing(UUID)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID + bytes([0x0c]) + b'\00\00\00\01')
                print(UUID + bytes([0x0c]) + b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
        except:
            print('the command was not excuted')
        if data1 == b'\xff':
            print('the command was not excuted correctly')
        elif data1 == b'\xfd':
            print('the command sent is not correct')

    return (data1)

def PRO_CANCEL_C():
    data,UUID = Z_Homing()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT_C))
                s.send(UUID + b'\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
        except:
            print('the command was not excuted')
        if data1 == b'\xff':
            print('the command was not excuted correctly')
        elif data1 == b'\xfd':
            print('the command sent is not correct')

    return (data1)

def PRO_DATA_IN(eip_instance, UUID):
    data = Z_Homing(UUID)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID + bytes([0x1b]) + b'\00\00\00\01'+ b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
                data1 = b'\xff'
        except:
            print('the command was not excuted')
            data1 = b'\xff'
        if data1 == b'\xff':
            print('the command was not excuted correctly')
        elif data1 == b'\xfd':
            print('the command sent is not correct')
        try:
            data1 = str(data1).replace("b'", '')
            data1 = str(data1).replace("x00", '')
            data1 = str(data1).replace("xff", '')
            data1 = str(data1).replace("xf", '')
            data1 = str(data1).replace("\\\\", '')
            data1 = data1[1:]
            data1 = str(data1).replace('{"Result List":[{"Result Element":', '')
            data1 = str(data1).replace("}]}", '')
            data1 = str(data1).replace("'", '')
            data1 = json.loads(data1)
            reply = eip_instance.write_variable('ADT1', str(data1["Date"]))
            reply = eip_instance.write_variable('RPG_no', str(int(data1["Number"])))
            reply = eip_instance.write_variable('TC1', str(int(data1["Turn Count"] * 10)))
            reply = eip_instance.write_variable('MaxTV', str(int(data1["Max Torque"] * 10)))
            reply = eip_instance.write_variable('TT', str(int(data1["Time"] * 10)))
            reply = eip_instance.write_variable('ZEP', str(int(data1["Z End Position"] * 100)))
            reply = eip_instance.write_variable('ZCP', str(int(data1["Z Current"] * 100)))
            print(int(data1["Operation Result"]))
            reply = eip_instance.write_variable('R1', True if int(data1["Operation Result"]) == 0 else False)
        except:
            print('the command was not excuted')
            data1 = 'none'
    return ()

def PRO_DATA_IN2(eip_instance, UUID):
    data = Z_Homing(UUID)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            if data == b'\x01':
                s.connect((ETD_IP, ETD_PORT))
                s.send(UUID + bytes([0x1b]) + b'\00\00\00\01'+ b'\00\00\00\01')
                data1 = s.recv(1024)
                s.close()
            else:
                print('No command was sent')
                data1 = b'\xff'
        except:
            print('the command was not excuted')
            data1 = b'\xff'
        if data1 == b'\xff':
            print('the command was not excuted correctly')
        elif data1 == b'\xfd':
            print('the command sent is not correct')
        try:
            data1 = str(data1).replace("b'", '')
            data1 = str(data1).replace("x00", '')
            data1 = str(data1).replace("xff", '')
            data1 = str(data1).replace("xf", '')
            data1 = str(data1).replace("\\\\", '')
            data1 = data1[1:]
            data1 = str(data1).replace('{"Result List":[{"Result Element":', '')
            data1 = str(data1).replace("}]}", '')
            data1 = str(data1).replace("'", '')
            data1 = json.loads(data1)
            reply = eip_instance.write_variable('ADT2', str(data1["Date"]))                       # write to eip device this data
            reply = eip_instance.write_variable('RPG_no2', str(int(data1["Number"])))
            reply = eip_instance.write_variable('TC2', str(int(data1["Turn Count"]*10)))
            reply = eip_instance.write_variable('MaxTV2', str(int(data1["Max Torque"]*10)))
            reply = eip_instance.write_variable('TT2', str(int(data1["Time"]*10)))
            reply = eip_instance.write_variable('ZEP2', str(int(data1["Z End Position"]*100)))
            reply = eip_instance.write_variable('ZCP2', str(int(data1["Z Current"]*100)))
            print(int(data1["Operation Result"]))
            reply = eip_instance.write_variable('R2', True if int(data1["Operation Result"]) == 0 else False)
        except:
            print('the command was not excuted')
            data1 = 'none'
    return ()

def Screw_man(SES, con_sta, lad_sta):
    while True:
        try:
            while SES[27] == 0:                                                               # action is ON
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.connect((ETD_IP, ETD_PORT))
                        time.sleep(1)
                        s.close()
                    con_sta[5] = 1                                                            # set link connection status up 
                    while SES[13] == 1 :                                                      # screw in
                        con_sta[6] = 1                                                        # in operation substate
                        PRO_SCREW_in()
                        SES[13] = 0
                        con_sta[6] = 0
                    while SES[14] == 1:                                                       # screw out
                        con_sta[7] = 1                                                        # out operation substate
                        PRO_SCREW_out()
                        SES[14] = 0
                        con_sta[7] = 0
                except:
                    con_sta[5] = 0                                                           # link down
                if SES[27] != 0 or lad_sta[1]  != 0:
                    con_sta[5] = 0                                                           # link down
                   break
        except:
            sys.exit()	

# simple example of the commands for the driver
if __name__ == "__main__":

    UUID = UUID_get()
    screw_data = PRO_SCREW_out(UUID)
    if screw_data not in (b'\xff',b'\x01'):   
        print("screw out...")
        screw_data2 = PRO_SCREW_in(UUID)
        if screw_data2 not in (b'\xff',b'\x01'):   
            print("screw in...")   

    data1 = PRO_SCREW_pik_RE()
    print(data1)     
    data = PRO_SCREW_in_RE()
    time.sleep(1)
    data = PRO_SCREW_out_RE()    
#!/usr/bin/python
# Riello Solar Panel Inverter interface
#
DEF_PORT=502
DEF_IP = "10.0.0.10"

# visulises the data returned from command 0
def createVisualGraph(Graph):
    if len(Graph) < 15:
        return
    Graph = Graph[10:]
    x = []
    y = []
    while(Graph):
        value = Graph[:4]
        #print(value)
        x.append(int.from_bytes(bytes.fromhex(value[0]),"big"))
        ora = str(x[-1])
        y.append(int.from_bytes(bytes.fromhex(value[1] + value[2]),"big"))
        valore = str(y[-1]/100)
        print("Il (X) " + ora + " è (Y) " + valore)
        Graph = Graph[4:]

# returns the data from the corresponding index in the message table
def getData(index):
    o = []
    client.send(get_something[index])
    rcv = client.recv(1024)
    Graph = rcv.hex()
    while(Graph):
        o.append(Graph[:2])
        Graph = Graph[2:]
    return o, rcv

# return the temperature
def showTemperature(Graph):
    if len(Graph) < 11:
        return
    Graph = Graph[10:]
    tempa = str(int.from_bytes(bytes.fromhex(Graph[0]),"big"))
    print("Temperature: " + tempa)
    return tempa

# return the power    
def showPower(Graph):
    if len(Graph) < 13:
        return
    Graph = Graph[11:]
    power = int.from_bytes(bytes.fromhex(Graph[0] + Graph[1]),"big")
    power = power / 10000
    print("Actual Power: " + str(power))
    return power

# use the crc library for the riello CRC check calculation
import crc16
def do_riello_crc(command, endi=0):
    a = crc16.CRC16.RIELLO()                    # create a Riello CRC object from the class imported as crc16
    a.update(command)
    crc_register = a.digest()
    if endi == 1:
        a = (crc_register&0xFF00)>>8
        b = (crc_register&0x00FF)<<8 
        crc_register = a | b      
    crc = crc_register.to_bytes(2, 'big')

    print("crc value = ",crc_register) 
    print(crc)  
    return crc

# print the crc message
def print_crc_msg(crc, xy_plot_a):
    print("\033[32m -------- got the crc check below ----------- \033[0m")
    print("calc : " ,crc)        
    print("sent : " ,xy_plot_a[2:])
    
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    import socket
    import sys
    import time

    # set up the communication over TCP and set up the message table
    host = DEF_IP
    port = int(DEF_PORT)
    # create TCP type socket and coinnect to it
    client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    client.connect((host, port))
    
    # these are the commands in the riello protocol to get the various infos
    get_something = []
    get_something.append(b"\x00\x00\x00\x00\x00\x06\x01\x03\xc0\x00\x00\x30")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x1e\x00\x02")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x1d\x00\x01")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x3d\x00\x01")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x05\x00\x01")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x37\x00\x02")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x39\x00\x02")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x10\x00\x0c")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x01\x00\x0f")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x3b\x00\x02")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x25\x00\x02")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x21\x00\x02")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x1c\x00\x01")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x23\x00\x02")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x10\x20\x00\x01")
    get_something.append(b"\x00\x00\00\x00\x00\x06\x01\x03\x30\x80\x00\x01")

    # now send all the commands above and print the reply messages
    for z in range(0, len(get_something)):
        gda, gd = getData(z)
        crc = do_riello_crc(gd) 
        print("calc crc : ",crc)        
        print("reply message : [",z,"] ",gda)

    # get power and temperature and print it
    xy_plot_a, xy = getData(0)
    crc = do_riello_crc(xy) 
    print_crc_msg(crc, xy_plot_a)
    createVisualGraph(xy_plot_a)
    pow_a, pow = getData(5)
    crc = do_riello_crc(xy) 
    print_crc_msg(crc, pow_a)
    power = showPower(pow_a)
    temprat_a, temprat = getData(12)
    crc = do_riello_crc(xy) 
    print_crc_msg(crc, temprat_a)
    temperture = showTemperature(getData(temprat_a))
    
    # closethe TCP connection
    client.close()





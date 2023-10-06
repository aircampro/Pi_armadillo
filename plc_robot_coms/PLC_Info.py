#!/usr/bin/env python
#

# ========================================= tools for showing and packing data ===================================================

import numpy as np

# make a numpy array byte by byte 
# this can be useful if you need to patch a particular part of the message 
# i.e use the i number which is the position of the byte in the string 
def bytewiseMakeNumpyArrayfromPython(data):
    for i in range(len(data)):
        if (i % 2) == 0:
            if (i == 0):
                dataNum6 = np.array(data[i], 'u1')
            else:
                dataNum6 = np.append(dataNum5, data[i])            
        elif (i % 2) == 1:
            dataNum5 = np.append(dataNum6, data[i])
    if (len(data) == dataNum5.size):
        return dataNum5
    elif (len(data) == dataNum6.size):
        return dataNum6 
    else:
        return 0    

# make a numpy array
def makeNumpyArrayfromPython(data):
    dataNum = np.array[data, 'u1')    
    return dataNum

# make a std char array from a numpy one    
def makePythonArrayfromNumpy(dataNum):
    py_data = []
    for i in range(dataNum.size):
        py_data.append(dataNum[i])
    return py_data

# ----- these encode decode assum the data is a numpy array ----
# ex. data = np.array([0x12, 0x34, 0x56], 'u1')
# use the routines above to convert the data between numpy and std python
def decode_bcd(data):
    """
    Decode 4bit BCD array
      [0x12,0x34,...] --> [1,2,3,4,...]
    """
    binArrayH = (data >> 4) & 0x0F
    binArrayL = data & 0x0F

    binArray = np.empty(data.size * 2, 'u1')
    binArray[::2] = binArrayH
    binArray[1::2] = binArrayL

    return binArray
	
def encode_bcd(data):
    """
    Encode 4bit BCD array
      [1,2,3,4,...] --> [0x12,0x34,...]
    """
    binArrayH = (data[::2] & 0x0F) << 4
    binArrayL = data[1::2] & 0x0F
    binArray = np.zeros_like(binArrayH)

    if data.size % 2 == 0:
        binArray = binArrayH | binArrayL
    else:
        binArray[:-1] = binArrayH[:-1] | binArrayL
        binArray[-1] = binArrayH[-1]

    return binArray    

def unpack_bits(data):
    """
    Expand the bit sequences stored in order from LSB to array of true/false
    [<M107 ... M100>, <M115 ... M108>] --> [<M100>, ... ,<M107>, <M108>, ... ,<M115>]
    """

    #   shapes it for viewing into bytes
    #   ex. [1,2,3] --> [[1],[2],[3]]
    byteArray2D = data.reshape((data.size, 1))

    # shows them as bits
    #   ex. [[1],[2],[3]] --> [[0,0,0,0,0,0,0,1],[0,0,0,0,0,0,1,0],[0,0,0,0,0,0,1,1]]
    byteArray2D_bin = np.unpackbits(byteArray2D, axis=1)
    print(byteArray2D_bin)
    
    # put it back like array([0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0], dtype=uint8)
    return byteArray2D_bin[:, ::-1].flatten()
	
def pack_bits(data):
    """
    Pack an array of bit data into bytes stored in order from LSB
    [<M100>, ... ,<M107>, <M108>, ... ,<M115>]  --> [<M107 ... M100>, <M115 ... M108>]
    """

    # can only work if all the bytes have been specified it must be 8 bits doesnot automatically zero pack 
    if not ((data.size % 8) == 0):
        print("data must be at least 8 bits long and divisible by 8 e.g 101 is not valid must be 00000101 ")
        return 0
        
    # work out how many 8bit bytes it makes
    size8 = -(-data.size // 8) * 8

    # set them like this [1 0 1 1 0 1 0 1 1 0 1 1 0 1 0 1]
    byteArray_bin = np.zeros(size8, 'u1')
    byteArray_bin[:size8] = data

    # make 8bit arrays accoridng to the number of bytes sent
    byteArray2D_bin = byteArray_bin.reshape((size8//8, 8))

    # packed bits returned e.g array([173, 175], dtype=uint8) for 2 bytes e,g a 16 bit input
    return np.packbits(byteArray2D_bin[:, ::-1])

def twosComplement_hex(hexval):
    bits = 32
    val = int(hexval, 16)
    if val & (1 << (bits-1)):
        val -= 1 << bits
    return val
    
# ============================================================== modbus ===========================================================================
#
# pip install pymodbus
# coil,discrtete input, holding register read and write

import time

MODBUS_TCP_SERVER_IP="10.0.0.1"
MODBUS_TCP_PORT=5020

def connectModbusTCP():
    from pymodbus.client.sync import ModbusTcpClient as ModbusClient
    client = ModbusClient(MODBUS_TCP_SERVER_IP, port=MODBUS_TCP_PORT)
    client.connect()
    return client

USB_SER_MOD="/dev/ttyUSB0"
MOD_BAUD=115200

def connectModbusSerial(pt=USB_SER_MOD,meth="rtu",bd=MOD_BAUD,to=0.1):
    from pymodbus.client.sync import ModbusSerialClient
    from pymodbus import exceptions
    client = ModbusSerialClient(port=pt, method=meth, baudrate=bd, timeout=to)
    client.connect()
    return client
	
def readCoilModbus(client,addr=1,num=1):
    # rr = client.read_coils(1, 1) - would read first coil and return it
    try:
        rr = client.read_coils(addr, num)
    except Exception as e:
        print(f"Exception in read_coils = {e}")    
	return rr.bits[0]

def readDiscreteInputModbus(client,addr=1,num=1):
    # rr = client.read_discrete_inputs(0, 1)   
    try:
        rr = client.read_discrete_inputs(addr, num)
    except Exception as e:
        print(f"Exception in read_discrete_inputs = {e}")  
	return rr.bits[0]
	
def readCoilsModbus(client,addr,number):
    # rr = client.read_coils(3, 9) - would read 3rd coil and next 9 bits return it   
    try:
        rr = client.read_coils(addr, number)
    except Exception as e:
        print(f"Exception in read_coils = {e}")  
	return rr.bits
	
def readDiscreteInputsModbus(client,addr,number):
    # rr = client.read_discrete_inputs(3, 9) - would read 3rd coil and next 9 bits return it    
    try:
        rr = client.read_discrete_inputs(addr, number)
    except Exception as e:
        print(f"Exception in read_discrete_inputs = {e}") 
	return rr.bits

def readHoldingRegistersModbus(client,addr,number):
    try:
        rr = client.read_holding_registers(addr, number)
    except Exception as e:
        print(f"Exception in read_holding_registers = {e}") 
	return rr.registers	

def writeCoilModbus(client,addr,boolValue):
    try:
        rr = client.write_coil(addr, boolValue)          # example True or False
    except Exception as e:
        print(f"Exception in write_coil = {e}")     
    return readCoilModbus(addr)	

def writeCoilsModbus(client,addr,boolValueList):
    try:
        rr = client.write_coil(addr, boolValueList)          # example boolValueList=[True,False]
    except Exception as e:
        print(f"Exception in write_coil = {e}")  
    return readCoilsModbus(addr,len(boolValueList))	
    
def writeHoldingRegisterModbus(client,addr,value): 
    try:
        rr = client.write_holding_register(addr, value)
    except Exception as e:
        print(f"Exception in write_holding_register = {e}")  
    return readHoldingRegistersModbus(addr,1)	

def writeHoldingRegistersModbus(client,addr,valuesList):
    try:
        rr = client.write_holding_registers(addr, valuesList)          # example set starting at 9 values = [ 1, 3, 5, 8 ]
    except Exception as e:
        print(f"Exception in write_holding_register = {e}")  
    return readHoldingRegistersModbus(addr,len(valuesList))	

def readHRegs(self, addr, count, unit=1):
    try:
        res = self.client.read_holding_registers(address=addr, count=count, unit=unit)
        # combine the numbers backwards [::2] = odd ones [1::2] = even ones e.g. both bytes
        # e.g. res.registers=[1,1,2,0,1,0,0,0] --> [65537, 2, 1, 0] or d=[3,1,0,4,0,1,0,0] --> [65539, 262144, 65536, 0]
        regs = [x + y * 0x10000 for (x, y) in zip(res.registers[::2], res.registers[1::2])]
    except Exception as e:
        print(f'Exceptio in read_holding_registers = {e}')

    return regs

def readHRegs2JsonList(self, addr, count, unit=1):
    try:
        res = self.client.read_holding_registers(address=addr, count=count, unit=unit)
        # combine the numbers backwards [::2] = odd ones [1::2] = even ones e.g. both bytes
        # e.g. res.registers=[1,1,2,0,1,0,0,0] --> [65537, 2, 1, 0] or d=[3,1,0,4,0,1,0,0] --> [65539, 262144, 65536, 0]
        regs = [x + y * 0x10000 for (x, y) in zip(res.registers[::2], res.registers[1::2])]
    except Exception as e:
        print(f'Exceptio in read_holding_registers = {e}')

    # create the json list
    l = { list : "H registers" }
    l['EX'] = twosComplement_hex('{:x}'.format(regs[0]))
    l['VX'] = twosComplement_hex('{:x}'.format(regs[1]))
    l['ex'] = regs[0]
    l['vx'] = regs[1]
    
    return l
    
def readInputRegistersModbus(client,addr,number):   
    try:
        rr = client.read_input_registers(addr, number)
    except Exception as e:
        print(f"Exception in read_input_registers = {e}")  
	return rr.registers	
	
def disconnectModbusTCP(client):
    client.close()
	
#
#                          Network communication using UDP TCP Socket - use these for mitsubishi and omron communication
#
#
# ======================================== OmRon PLC =========================================================================
#
OMRON_HOST, OMRON_PORT = "192.168.0.1", 8555

def connectTCP(host, port):
    import socket
    TCP_client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)   
    TCP_client.settimeout(10)
    TCP_client.connect((host, port))
    return TCP_client
    
def coonectUDP(host, port):
    import socket
    UDP_client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)   
    UDP_client.settimeout(10)
    UDP_client.connect((host, port))
    return UDP_client

SCTP_PORT=36413
def connectSCTP(host, port):
    import socket
    import sctp
    sk = sctp.sctpsocket_tcp(socket.AF_INET)
    sk.connect((host, SCTP_PORT))
    return sk
    
def diconnectTCP(TCP_client):
    TCP_client.close()

def diconnectUDP(UDP_client):
    UDP_client.close()
    
def disconnectSCTP(sk):
    sk.close()
    
def closeIPHandle(ip_handle):
    ip_handle.close()
   
# OMRON CJ Series - this is the data message that defines what you are reading from the omron PLC 
data_omron_cj = [
    # header
    0x80, #(ICF) 
    0x00, #(RSV) 
    0x02, #(GCT) 
    0x00, #(DNA) 
    0x00, #(DA1) 
    0x00, #(DA2) 
    0x00, #(SNA) 
    0x9C, #(SA1)
    0x00, #(SA2) 
    0x22, #(SID) 

    # command
    0x01, #(MRC) 
    0x01, #(SRC)   (0x01,0x01 read、0x01,0x02 write)
    # data
    0x82,             # Read device specification 0x82=D register
    0x00, 0x00, 0x96, # Specify read area (000096=150)
    0x00,0x07,        # Read Bytes
]

def omronCJReadUDP(UDP_client,data_omron_cj_msg):                          # uses UDP Client connected as above
    try:                                                                   # in this example if cant send it will attempt re-connection once
        UDP_client.send(bytes(data_omron_cj_msg))
    except Exception as e:
        print(f'omron CJ series UDP send error try reconnect: {e}')
        try:
            UDP_client.close()   
        except Exception as e:
            print(f'omron CJ series UDP socket close error try reconnect: {e}') 
        try:
            UDP_client=connectUDP(OMRON_HOST, OMRON_PORT) 
            time.sleep(2)
            UDP_client.send(bytes(data_omron_cj_msg))       
        except Exception as e:
            print(f'omron CJ series UDP socket close error try reconnect: {e}') 
            exit(1)            
    response = UDP_client.recv(1024)
    data1 = [format(i,'02X') for i in response]
    return_data = [0] * 6
    data1 = data1[14:]
    # Convert base 16 to base 10
    # Because the CJ series has a base 10 and a base 16 by register
    # Note conversion in int function
    return_data[0] = int(data1[4] + data1[5], 16)
    return_data[1] = int(data1[7])
    return_data[2] = int(data1[8] + data1[9])
    return_data[3] = int(data1[10] + data1[11])
    return_data[4] = int(data1[13])
    return_data[5] = int(data1[2] + data1[3] + data1[0] + data1[1], 16)
    return return_data

# ================= CJSeries TCP Only ===================== #

# ================= PLC read ================= #
cj_read_protocol = [
    # header
    0x80, #(ICF) 
    0x00, #(RSV) 
    0x02, #(GCT) 
    0x00, #(DNA) 
    plc_node, #(DA1) 
    0x00, #(DA2) 
    0x00, #(SNA) 
    pc_node, #(SA1) 
    0x00, #(SA2) 
    SIA, #(SID) 

    # command
    0x01, #(MRC) 
    0x01, #(SRC) 
    # data
    0x82,             # Read device specification
    0x00, 0x00, 0x96, # read area
    0x00,             # Number of high read Bytes
    0x07,             # Read Bytes
]

def omronCJReadTCP(TCP_client,data_cj_readprotocol):                                         # uses TCP Client connected as above
    # ================= shakehand ================= #
    SIA = 0x55
    shakehand_protocol = [
        # TCP header
        0x46, 0x49, 0x4E, 0x53,  # FINS
        0x00, 0x00, 0x00, 0x0C,  # Length
        0x00, 0x00, 0x00, 0x00,  # Command
        0x00, 0x00, 0x00, 0x00,  # Error Code

        # FINS node address  0x00 = auto recieve
        0x00, 0x00, 0x00, 0x00,
    ]
    TCP_client.send(bytes(shakehand_protocol))
    response = TCP_client.recv(1024)
    data1 = [format(i,'02X') for i in response]
    print("shakehandPLC Result is : ", data1)
    pc_node = int(data1[19], 16)
    plc_node = int(data1[23], 16)
    print("PC node is : ", data1[19])
    print("PLC node is : ", data1[23])

    # ================= PLC ready ================= #
    standby_protocol = [
        # TCP header
        0x46, 0x49, 0x4E, 0x53,  # FINS
        0x00, 0x00, 0x00, 0x1A,  # Length
        0x00, 0x00, 0x00, 0x02,  # Command
        0x00, 0x00, 0x00, 0x00,  # Error Code
    ]
    # standby
    TCP_client.send(bytes(standby_protocol))
    TCP_client.send(bytes(data_cj_readprotocol))
    # wait and read reply
    response = TCP_client.recv(1024)
    data1 = [format(i,'02X') for i in response]
    data1 = data1[30:]
    # Convert base 16 to base 10
    # Because the CJ series has a base 10 and a base 16 by register
    # Note conversion in int function
    return_data = [0] * 6
    return_data[0] = int(data1[4] + data1[5], 16)                   
    return_data[1] = int(data1[7])                                     
    return_data[2] = int(data1[8] + data1[9])                           
    return_data[3] = int(data1[10] + data1[11])                         
    return_data[4] = int(data1[13])                                      
    return_data[5] = int(data1[2] + data1[3] + data1[0] + data1[1], 16) 
    return return_data

# ================= mitsi ===================================
mitsi_host, mitsi_port = "192.168.1.2", 1026

# ================= Qseries TCP and UDP ===================== #
dataQ = [
    0x50,0x00,      # sub-header
    0x00,           # Requesting network number
    0xFF,           # Request destination station number
    0xFF,0x03,      # Requesting unit I/O number
    0x00,           # Request destination multi-drop station number
    0x0C,0x00,      # Request data length (set later)
    0x01,0x00,      # Monitoring timer
    0x01,0x04,      # command (0401 for bulk read, 1401 for bulk write)
    0x00,0x00,      # subcommands (at 0000)
    0x96,0x00,0x00, # Leading device number (000096=150)
    0xB0,           # Device code (ZR register =0xB0, D register =0xa8)
    0x07,0x00       # Device number ZR150 to ZR156
]    
# mitsubishi Q Series
def readQSeries(tcp_udp_connect_obj,dataQmsg):
    dataQmsg[7] = len(dataQmsg[9:]) & 0xFF
    dataQmsg[8] = (len(dataQmsg[9:]) >> 16) & 0xFF
    try:
        tcp_udp_connect_obj.send(bytes(dataQmsg))
    except Exception as e:
        print(f'mitsi Q series send error try reconnect: {e}')
        tcp_udp_connect_obj.close()
        exit(1)
    response = tcp_udp_connect_obj.recv(1024)
    data1 = [format(i,'02X') for i in response]
    return_data = [0] * 6
    return_data[5] = data1[15]
    return_data[4] = data1[17]
    return_data[3] = data1[19]
    return_data[2] = data1[21]
    return_data[1] = data1[23]
    return_data[0] = data1[14] + data1[13] + data1[12] + data1[11]
    convert_data = [int(s, 16) for s in return_data]
    #In Mitsubishi, the device will be a little endian,
    #so inverting is required, as in return_data[0].
    return convert_data
    
# ================= FX3Series TCP and UDP ===================== #
dataFX = [
    0x01,0xFF,            # sub-header
    0x05,0x00,            # Requesting network number
    0x96,0x00,0x00,0x00,  # Leading device number (000096=150)
    0x20,0x44,            # Device code (D register)
    0x07,0x00             # Device count
]
# mitsubishi FX Series
def readFXSeries(tcp_udp_connect_obj,dataFXmsg):
    tcp_udp_connect_obj.send(bytes(dataFXmsg))
    response = tcp_udp_connect_obj.recv(1024)
    data1 = [format(i,'02X') for i in response]
    return_data = [0] * 6
    return_data[0] = data1[6]
    return_data[1] = data1[8]
    return_data[2] = data1[10]
    return_data[3] = data1[12]
    return_data[4] = data1[14]
    return_data[5] = data1[3] + data1[2]
    convert_data = [int(s, 16) for s in return_data]
    return convert_data

# ================= KV Series TCP and UDP ===================== #
# ASCII communication
# Read example
# RDS(\x20)(device type) (device number) (Data format) (\x20) (Number of reads)(\x0D)
# In the case of data memory number 8000
# b"RDS\x20DM008000.U\x200002\x0D"
# Write example
# WRS(\x20)(device type)(device number) (data format)(\x20)(Number of writes)(\x20)(data 1)(\x20)(data 2)(\x20)...(\x0D)
# In the case of data memory number 8000
# b"WRS\x20DM008000.U\x200002\x20AA\x20BB\x0D"

# For the ZF register
# b"RDS\x20ZF061100.U\x201\x0D"
# b"WRS\x20ZF061104.U\x201\x202\x0D"

# ================= communication function ================= #
def readKVSeries(client):
    kvCommand = b"RDS\x20DM008000.U\x200002\x0D"
    client.send(bytes(kvCommand))
    response = client.recv(1024)
    data_sum = ""
    for dt in response[:-2]:
        temp = chr(int(format(dt, "02X"), 16))
        data_sum = data_sum + temp 
    return_data = int(data_sum)
    
def writeKVSeries(client):
    kvCommand = b"WRS\x20DM008000.U\x200002\x20AA\x20BB\x0D"
    client.send(bytes(kvCommand))
    response = client.recv(1024)
    return response


# ================= iQ-R TCP and UDP ===================== #
dataIQR = [
    0x50,0x00,      # sub-header
    0x00,           # Requesting network number
    0xFF,           # Request destination station number
    0xFF,0x03,      # Requesting unit I/O number
    0x00,           # Request destination multi-drop station number
    0x0C,0x00,      # Request data length (set later)
    0x01,0x00,      # Monitoring timer
    0x01,0x04,      # command (0401 for bulk read and 1401 for bulk write)
    0x02,0x00,      # subcommands (at 0000)

    0x6C,0x20,0x00,0x00, # Leading device number (00030D40=200000)
    0xA8,0x00,           # Device code (ZR register =0xB0, D register =0xa8)
    0x02,0x00       # Device points ZR200000 to ZR200005
]
def readIQRSeries(tcp_udp_connect_obj, dataIQRmsg):
    dataIQRmsg[7] = len(dataIQRmsg[9:]) & 0xFF
    dataIQRmsg[8] = (len(dataIQRmsg[9:]) >> 16) & 0xFF
    tcp_udp_connect_obj.send(bytes(dataIQRmsg))
    response = tcp_udp_connect_obj.recv(1024)
    data1 = [format(i,'02X') for i in response]
    return_data = data1[-4:]
    return_data = [int(s) for s in return_data]

# ================= slmp mitsi TCP ===================== #
# '5000 00 FF 03FF 00 0018 0020 0401 0000 D* 000001 0003'

data1 = [
    0x50,0x00,      # sub-header
    0x00,           # Requesting network number
    0xFF,           # Request destination station number
    0xFF,0x03,      # Requesting unit I/O number
    0x00,           # Request destination multi-drop station number
    0x0C,0x00,      # Request data length (set later)
    0x20,0x00,      # Monitoring timer
    0x01,0x04,      # command
    0x00,0x00,      # subcommands
    0x01,0x00,0x00, # Leading device number
    0xA8,           # Device code
    0x03,0x00       # Device count
]

data2 = [
    0x50,0x00,      # sub-header
    0x00,           # Requesting network number
    0xFF,           # Request destination station number
    0xFF,0x03,      # Requesting unit I/O number
    0x00,           # 
    0x00,0x00,      # Request data length (set later)
    0x04,0x00,      # Monitoring timer
    0x01,0x14,      # command (1401H)
    0x00,0x00,      # subcommands
    0x00,0x00,0x00, # Leading device number
    0xA8,           # Device code
    0x05,0x00,      # Device count
    0x11,0x22,      # D0000
    0x33,0x44,      # D0001
    0x55,0x66,      # D0002
    0x77,0x88,      # D0003
    0x99,0xAA,      # D0004
]

def connectMitsiTCP(MIT_HOST = '192.168.1.1', MIT_PORT = 1026):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((MIT_HOST, MIT_PORT))
    sock.settimeout(3)
    return sock
    
def sendDataSlmpMitsi(tcp_udp_connect_obj,data,BUFSIZE=4096):
   # set the data lenght in the message
   data[7] = len(data[9:]) & 0xFF
   data[8] = (len(data[9:]) >> 16) & 0xFF
   tcp_udp_connect_obj.send(bytes(data))
   res = tcp_udp_connect_obj.recv(BUFSIZE)
   print (*[format(i,'02X') for i in res])
   return res

def disconnectMitsiTCP(tcp_udp_connect_obj):
    tcp_udp_connect_obj.close()
    
# =================================== SIEMENS S7-1500 ================================================================
# Siemens requires work to be done in PLC as per web link, to set-up communication block in S7
#
import socket
DESTINATION_ADDR = '192.168.0.1'
SOURCE_PORT, DESTINATION_PORT = 2001, 2000

# http://soup01.com/ja/2020/06/15/post-2254/
# SIEMENS PLC S7-1500 PLC
def connectSiemensS7_1500():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('192.168.0.229', SOURCE_PORT))
    sock.connect((DESTINATION_ADDR, DESTINATION_PORT))
    return sock
    
def readSiemensS7_1500(sock):
    sock.send(b'\x11\x00\x19\x29\x30\x30\x30\x30\x21\x28')
    data = sock.recv(1024)
    print(repr(data))
    return data
    
def disconnectSiemensS7_1500(sock):    
    return sock.close()    
    
# ==========================================allen bradley==================================================================
# ---- AB ------ controlLogix
AB_IP='aaa.bbb.ccc.ddd'

def connectAllebBradleyCL():
    from pylogix import PLC
    ab = PLC()
    ab.IPAddress = AB_IP
    return ab
    
def connectAllebBradleyCL(ab):
    t = ab.Read('TAG_NAME[1].PARTCOUNT')                           # this is the tagname in the allen bradley PLC
    print('Tag Name: ', t.TagName, '\nTag Value: ', t.Value)
    return t.TagName,t.Value
    
def closeAllenBradleyCL(ab):
    ab.Close()
    
# DF1 protocol
#
AB_DF1_TABLE=43
AB_DF1_START_REG=245
AB_DF1_BYTES_READ=10
AB_DF1_PORT=10232
AB_DF1_TARGETIP='192.168.0.32'
AB_DF1_SRC=0x0
AB_DF1_DST=0x1

def connectABDF1Ethernet():
    from df1.df1_client import Df1Client
    from df1.commands import Command0FA2
    from df1.file_type import FileType
    with Df1Client(src=AB_DF1_SRC, dst=AB_DF1_DST) as client:
        client.connect(AB_DF1_TARGETIP, AB_DF1_PORT)
        command = client.create_command(Command0FA2, table=AB_DF1_TABLE, start=AB_DF1_START_REG, bytes_to_read=AB_DF1_BYTES_READ, file_type=FileType.INTEGER)
        reply = client.send_command(command)
        r=reply.get_data(FileType.INTEGER)
        print(r)
        client.Close()
        return r

# ======================== mitsubishi melsec library =============================
import pymcprotocol

M_TARGET_PLC_IP = '192.168.1.2'
M_PORT = 1026

def createMitsiObject(plctype="L"):  # other type e.g "iQ-R"
    pymc = pymcprotocol.Type3E(plctype)
    return pymc
    
def connectMitPLC(pymc):
    ret=0
    try:
        pymc.connect(M_TARGET_PLC_IP, M_PORT)
        ret=1
    except Exception as e:
        print(f'def connect_plc: {e}')
   return ret

HEAD_X = 'X0'
HEAD_Y = 'Y0'
LEN_X = 16
LEN_Y=16

# this example has 3 attempts at getting the data   
def getMitXregs(pymc,head_x,len_x,head_y,len_y):
    retry = 0
    try:
        while (retry <= 2):
            try:             
                bitunits_Xs = []
                bitunits_Ys = []
                bitunits_Xs = pymc.batchread_bitunits(headdevice= head_x, readsize=len_x)
                bitunits_Ys = pymc.batchread_bitunits(headdevice= head_y, readsize=len_y)
                retry=2   
            except Exception as e:
                print(f'def main: {e}')
                pymc.close()
                time.sleep(5)
                if (connectMitPLC(pymc)==1):
                    retry += 1
                else:
                    print("can not re-connect error with network")                         
                    retry=2
    except Exception as e:
        print(f'main: {e}')
 
    finally:
        pymc.close()
        return bitunits_Xs,bitunits_Ys

# this example has 3 attempts at getting the data   

D_START="D100"
D_NUM=10                             # example D100 to D110

def getMitDregs(pymc,d_start,d_num):
    retry = 0
    wordunits_values = []
    try:
        while (retry <= 2):
            try:             
                wordunits_values = pymc3e.batchread_wordunits(headdevice=d_start, readsize=d_num)
                retry=2   
            except Exception as e:
                print(f'def main: {e}')
                pymc.close()
                time.sleep(5)
                if (connectMitPLC(pymc)==1):
                    retry += 1
                else:
                    print("can not re-connect error with network")                         
                    retry=2
    except Exception as e:
        print(f'main: {e}')
 
    finally:
        pymc.close()
        return wordunits_values
        
def setMitXregs(pymc,head_x,bitunits_Xs):
    retry = 0
    try:
        while (retry <= 2):
            try:             
                pymc.batchwrite_bitunits(headdevice=head_x, values=bitunits_Xs) 
                retry=2   
            except Exception as e:
                print(f'def main: {e}')
                pymc.close()
                time.sleep(5)
                if (connectMitPLC(pymc)==1):
                    retry += 1
                else:
                    print("can not re-connect error with network")                         
                    retry=2
    except Exception as e:
        print(f'main: {e}')
 
    finally:
        pymc.close()

RANDOM_LIST=["X0", "X10"]
RAND_VALUE_LIST=[1, 0]
def randomSetXregs(pymc,random_list,r_v_list):
    retry = 0
    try:
        while (retry <= 2):
            try:             
                pymc3e.randomwrite_bitunits(bit_devices=random_list, values=r_v_list) 
                retry=2   
            except Exception as e:
                print(f'def main: {e}')
                pymc.close()
                time.sleep(5)
                if (connectMitPLC(pymc)==1):
                    retry += 1
                else:
                    print("can not re-connect error with network")                         
                    retry=2
    except Exception as e:
        print(f'main: {e}')
 
    finally:
        pymc.close() 

DW_LIST=["D1000", "D1002"] 
DW_VALUES=[1000, 2000] 
DW_DEV=["D1004"] 
DW_DEV_VALS=[655362]

def randomSetDregs(pymc,dw_list,dw_vals,dw_dev,dw_dev_vals):
    retry = 0
    try:
        while (retry <= 2):
            try:             
                pymc3e.randomwrite(word_devices=dw_list, word_values=dw_vals, dword_devices=dw_dev, dword_values=dw_dev_vals) 
                retry=2   
            except Exception as e:
                print(f'def main: {e}')
                pymc.close()
                time.sleep(5)
                if (connectMitPLC(pymc)==1):
                    retry += 1
                else:
                    print("can not re-connect error with network")                         
                    retry=2
    except Exception as e:
        print(f'main: {e}')
 
    finally:
        pymc.close()     

# ===================================== ether CAT ==================================================================================
from pyEtherCAT import MasterEtherCAT
import time
import os
import psutil

def EtherCAT_Init(nic):
    cat = MasterEtherCAT.MasterEtherCAT(nic) # Contains the address of the network card
    return cat

def EtherCAT_SetUp(cat):
    cat.EEPROM_SetUp(cat.ADP) # EEPROM configuration, no modification required
    cat.EEPROM_Stasus(enable=0x00, command=0x04)
    ADDR = 0x0120 # AL control register
    data = 0x0002 # 2h: Request pre-action state
    cat.APWR(IDX=0x00, ADP=cat.ADP, ADO=ADDR, DATA=[
    data & 0xFF, (data >> 8) & 0xFF])
    (DATA, WKC) = cat.socket_read()
    print("[0x{:04x}]= 0x{:04x}".format(ADDR, DATA[0] | DATA[1] << 8))
    ADDR = 0x0120 # AL control register
    data = 0x0002 # 2h: Request pre-action state
    cat.APWR(IDX=0x00, ADP=cat.ADP, ADO=ADDR, DATA=[
    data & 0xFF, (data >> 8) & 0xFF])
    (DATA, WKC) = cat.socket_read()
    print("[0x{:04x}]= 0x{:04x}".format(ADDR, DATA[0] | DATA[1] << 8))
    ADDR = 0x0120 # AL control register
    data = 0x0004 # 4h: Request safe operating state
    cat.APWR(IDX=0x00, ADP=cat.ADP, ADO=ADDR, DATA=[
    data & 0xFF, (data >> 8) & 0xFF])
    (DATA, WKC) = cat.socket_read()
    print("[0x{:04x}]= 0x{:04x}".format(ADDR, DATA[0] | DATA[1] << 8))
    ADDR = 0x0120 # AL control register
    data = 0x0008 # 8h: Request operating state
    cat.APWR(IDX=0x00, ADP=cat.ADP, ADO=ADDR, DATA=[
    data & 0xFF, (data >> 8) & 0xFF])
    (DATA, WKC) = cat.socket_read()
    print("[0x{:04x}]= 0x{:04x}".format(ADDR, DATA[0] | DATA[1] << 8))

def EtherCAT_GPIOMode(cat, data):
    ADDR = 0x0F00 # Digital I/O output data register
    # data = 0x00FF # Output data
    cat.APWR(IDX=0x00, ADP=cat.ADP, ADO=ADDR, DATA=[
    data & 0xFF, (data >> 8) & 0xFF])
    (DATA, WKC) = cat.socket_read()
    print("[0x{:04x}]= 0x{:04x}".format(ADDR, DATA[0] | DATA[1] << 8))

def EtherCAT_GPIO_Out(cat, data):
    ADDR = 0x0F10
    cat.APWR(IDX=0x00, ADP=cat.ADP, ADO=ADDR, DATA=[data & 0xFF, (data >> 8) & 0xFF])
    (DATA,WKC) = cat.socket_read()

def EtherCAT_GPIO_In(cat):
    ADDR = 0x0F18
    cat.APRD(IDX=0x00, ADP=cat.ADP, ADO=ADDR, DATA=[])
    (DATA,WKC) = cat.socket_read()

# toggle output example using above functions uses eth0 nic interface 
def EC_Test_Example():

    cat = EtherCAT_Init("eth0") # EtherCAT network initialization
    # EtherCAT Process to execute EtherCAT state machine
    cat.ADP = 0x0000 # From the 1st PC is 0, after the 2nd PC is -1
    EtherCAT_SetUp(cat) # EtherCAT Slave initialization
    EtherCAT_GPIOMode(cat, 0xFFFF) # GPIO direction setting for EtherCAT slave 0: Input 1: Output

    # EtherCAT Process to execute EtherCAT state machine
    cat.ADP = 0x0000 - 1 # Example This is not necessary if the 2nd unit is not connected
    EtherCAT_SetUp(cat) # EtherCAT Slave initialization
    EtherCAT_GPIOMode(cat, 0xFFFF) # GPIO direction setting for EtherCAT slave 0: Input 1: Output

    # EtherCAT Process to execute EtherCAT state machine
    cat.ADP = 0x0000 - 2 # Example This is not necessary if the 3rd unit is not connected
    EtherCAT_SetUp(cat) # EtherCAT Slave initialization
    EtherCAT_GPIOMode(cat, 0xFFFF) # GPIO direction setting for EtherCAT slave 0: Input 1: Output

    #シ Shift the 1st LED
    TIME = 0.1
    cat.ADP = 0x0000

    flag = 0
    CNT = 0

    try:
        while 1:
        # time.sleep(TIME)
        cat.ADP = 0x0000 - 0
        EtherCAT_GPIO_Out(cat, 0xFFFF)
        time.sleep(TIME)
        cat.ADP = 0x0000 - 1
        EtherCAT_GPIO_Out(cat, 0xFFFF)
        time.sleep(TIME)
        cat.ADP = 0x0000 - 2
        EtherCAT_GPIO_Out(cat, 0xFFFF)
        time.sleep(TIME)
        cat.ADP = 0x0000 - 0
        EtherCAT_GPIO_Out(cat, 0x0000)
        time.sleep(TIME)
        cat.ADP = 0x0000 - 1
        EtherCAT_GPIO_Out(cat, 0x0000)
        time.sleep(TIME)
        cat.ADP = 0x0000 - 2
        EtherCAT_GPIO_Out(cat, 0x0000)
        time.sleep(TIME)
        
#
# ======================================== Yamaha ROUTER SWITCH written in the lua code =================================================================
#
YAMAHA_HOST, YAMAHA_PORT = "192.168.100.1", 11111    # AS PER LUA SCRIPT

def readYamahaRouterTemp(tcp_connect_obj):
    datamsg = b"$send_temp$"
    tcp_connect_obj.send(bytes(datamsg))
    response = tcp_connect_obj.recv(1024)
    return int(response)

# ==================================================== TITAN_ASCII SERIAL USB ================================================================
# Titan IoT Servo Controller: TITAN-SVX-ETH
#
import serial
import time
import serial.tools.list_ports
TITAN_BAUD_RT=115200

def search_com_port():
    coms = serial.tools.list_ports.comports()
    list_com = []
    for com in coms:
        list_com.append(com.device)
    print('Connected COM ports: ' + str(list_com))
    used_port = list_com[0]
    print('Use COM port: ' + used_port)
    return used_port
    
def connect_Titan(opt=1):
    if (opt == 1):
        ser = serial.Serial('/dev/ttyUSB0', TITAN_BAUD_RT, timeout=0.1)
    else:
        com_prt = search_com_port()                                        # get the first known device
        s = serial.Serial(com_prt, TITAN_BAUD_RT, timeout=0.1)                      
    return ser
    
def readTitan(ser):
    ser.write(b'@01:FLT;MST;EX;VX;CURQA;CURDA\r\n')
    line = ser.readline()
    # b'FLT=0x0;MST=0x3;EX=9999;CURQA=-0.0379776;CURDA=0.0160128\r\n'
    #line.decode()[4:-2].split(';') this was in original but it didnt seem to have the leading 4 bytes
    line.decode()[0:-2].split(';')
    # ['FLT=0x0', 'MST=0x3', 'EX=9999', 'CURQA=-0.0379776', 'CURDA=0.0160128']
    for x in line.decode()[0:-2].split(';'):
        k, v = x.split('=')
        print(f"k={v} v={k}")
        d = (k,v)
        a.append(d)    
    return a

def read_Titan_FLT(a):
    try:
        rv = int(a[0][1])
    except ValueError:
        s=a[0][1].split("x")
        if len(s) > 1:
            rv = int(s[1])
        else:
            rv = 0
    return rv   

def read_Titan_MST(a):
    try:
        rv = int(a[1][1])
    except ValueError:
        s=a[1][1].split("x")
        if len(s) > 1:
            rv = int(s[1])
        else:
            rv = 0
    return rv     

def read_Titan_EX(a):
    try:
        rv = int(a[2][1])
    except ValueError:
        rv = float(a[2][1])
    return rv   

def read_Titan_CURQA(a):
    try:
        rv = float(a[3][1])
    except ValueError:
        rv = int(a[3][1])
    return rv 

def read_Titan_CURDA(a):
    try:
        rv = float(a[4][1])
    except ValueError:
        rv = int(a[4][1])
    return rv 

# The Python Gateway is a Swiss army knife for connecting NMEA 0183 and NMEA 2000 devices. 
# It is equipped with two serial ports (one for receiving NMEA 0183 data and one for transmitting), 
# a CAN bus interface (for NMEA 2000) and a USB port for connection to a PC
#
# https://www.yachtd.com/products/python_gateway/getting_started.html
#
import struct
import time
import pyb
import nmea2000

class N2kProdInfo:
    N2K_PROD_INFOR_PGN = 126996
    def __init__(self):
        self.ManModSerial = None
        self.ProductCode = None
        self.ModelID = None
        self.SwCode = None
        self.ModelVersion = None
        self.LoadEquivalency = None
        self.N2kVersion = None
        self.CertificationLevel = None
        self.iDev =	None

class N2k_Maretron_FluidVol:
    MARETRON_FFM100_FTv_PGN = 65287
    def __init__(self):
        self.proprieter = None
        self.SID = None
        self.VolumeInstance = None
        self.FluidType = None
        self.TripVolume = None

class N2k_Maretron_FRate:
    MARETRON_FFM100_FRate_PGN = 65286
    def __init__(self):
        # self.DataLen = None                                mot sure !
        self.proprieter = None
        self.SID = None
        self.FlowRateInstance = None
        self.FluidType = None
        self.FluidFlowRate = None

class N2k_Maretron_Temp:
    MARETRON_TMP100_PGN = 130823
    def __init__(self):
        self.proprieter = None
        self.SID = None
        self.TempInstance = None
        self.TempSource = None
        self.ActualTemperature = None
        self.SetTemperature = None
        
# globals		
N2K_PROD_INFO = N2kProdInfo()                          # make n2k product info structure class
N2K_FV_INFO = N2k_Maretron_FRate()                     # make n2k flow rate structure class
N2K_FR_INFO = N2k_Maretron_FluidVol()                  # make n2k flow vol. structure class
N2K_T_INFO = N2k_Maretron_Temp()                       # make n2k temp. structure class
ENDO=0                                                 # define endianess big or little  

engine_updated = [False, False]
engine_kPa = [0, 0]                                    # boost pressure in NMEA2000 format
engine_rpm = [0, 0]                                    # engine speed in NMEA2000 format

# makes an int 16 number from 2 bytes
def make16(endo, d):
    if endo == 1:
            value = d[1] | (d[0] << 8)
	else:
	    value = d[0] | (d[1] << 8)
    return value

# makes an int32 number from 2 words (4bytes)
def make32(endo, data):
    if endo == 1:
        ProductCodeList = [data[1] | (data[0]<<8), data[3] | (data[2]<<8)]
    else:
        ProductCodeList = [data[0] | (data[1]<<8), data[2] | (data[3]<<8)]
    ProductCode = struct.unpack('@i', struct.pack('@2h', *ProductCodeList))[0]             # make 32bit integer
    return ProductCode

# makes an real32 number from 2 words (4bytes)
def makef32(endo, data):
     if endo == 1:
         ProductCodeList = [data[1] | (data[0]<<8), data[3] | (data[2]<<8)]
     else:
         ProductCodeList = [data[0] | (data[1]<<8), data[2] | (data[3]<<8)]
     ProductCode = struct.unpack('@f', struct.pack('@2h', *ProductCodeList))[0]
     return ProductCode

# makes an real64 number from 4 words (8bytes)
def makef64(endo, data):
    if endo == 1:
        ProductCodeList = [data[1] | (data[0]<<8), data[3] | (data[2]<<8), data[5] | (data[4]<<8), data[7] | (data[6]<<8) ]
    else:
        ProductCodeList = [data[0] | (data[1]<<8), data[2] | (data[3]<<8), data[4] | (data[5]<<8), data[6] | (data[7]<<8) ]
    ProductCode = struct.unpack('@f', struct.pack('@2h', *ProductCodeList))[0]
    return ProductCode
    
# read product info message	
def product_info(n2k, data, msgid, seq):
    global N2K_PROD_INFO
    # Remove unused characters in string byte fields
    N2K_PROD_INFO.ManModSerial = data[0:31].split(b'\xff')[0]
    N2K_PROD_INFO.ManModSerial = N2K_PROD_INFO.ManModSerial.decode().rstrip()
    N2K_PROD_INFO.ProductCode = make32(ENDO, data[32:35])
    N2K_PROD_INFO.ModelID = data[36:67].split(b'\xff')[0]
    N2K_PROD_INFO.ModelID = N2K_PROD_INFO.ModelID.decode().rstrip()
    N2K_PROD_INFO.SwCode = data[68:107].split(b'\xff')[0]
    N2K_PROD_INFO.SwCode = N2K_PROD_INFO.SwCode.decode().rstrip()
    N2K_PROD_INFO.ModelVersion = data[108:131].split(b'\xff')[0]
    N2K_PROD_INFO.ModelVersion = N2K_PROD_INFO.ModelVersion.decode().rstrip()
    N2K_PROD_INFO.LoadEquivalency = make16(ENDO, data[132:133])
    N2K_PROD_INFO.N2kVersion = make16(ENDO, data[134:135])
    N2K_PROD_INFO.CertificationLevel = make16(ENDO, data[136:137])
    N2K_PROD_INFO,iDev = make16(ENDO, data[138:139])

# read maretron flow volume message
def fluid_vol_info(n2k, data, msgid, seq):
    global N2K_FV_INFO
    # Remove unused characters in string byte fields
    N2K_FV_INFO.proprieter = make16(ENDO, data[0:1])
    N2K_FV_INFO.SID = data[2]
    N2K_FV_INFO.VolumeInstance = data[3]
    N2K_FV_INFO.FluidType = data[4]
    N2K_FV_INFO.TripVolume = makef64(ENDO, data[5:12])	

# read maretron flow volume message
def fluid_flo(n2k, data, msgid, seq):
    global N2K_FR_INFO
    # Remove unused characters in string byte fields
    N2K_FR_INFO.proprieter = make16(ENDO, data[0:1])
    N2K_FR_INFO.SID = data[2]
    N2K_FR_INFO.FlowRateInstance = data[3]
    N2K_FR_INFO.FluidType = data[4]
    N2K_FR_INFO.FluidFlowRate = makef64(ENDO, data[5:12])	
      
# read maretron temperature message
def fluid_temp(n2k, data, msgid, seq):
    global N2K_T_INFO
    # Remove unused characters in string byte fields
    N2K_T_INFO.proprieter = make16(ENDO, data[0:1])
    N2K_T_INFO.SID = data[2]
    N2K_T_INFO.TempInstance = data[3]
    N2K_T_INFO.TempSource = data[4]
    N2K_T_INFO.ActualTemperature = makef64(ENDO, data[5:12])
    N2K_T_INFO.SetTemperature = makef64(ENDO, data[13:20])

# initialise asking for the PGN's that we are parsing in the functions above	
def on_init(n2k, state):
    if state == NMEA2000.READY:
        n2k.request(N2kProdInfo.N2K_PROD_INFOR_PGN)
        n2k.request(N2k_Maretron_FluidVol.MARETRON_FFM100_FTv_PGN)
        n2k.request(N2k_Maretron_Temp.MARETRON_TMP100_PGN)
        n2k.request(N2k_Maretron_FRate.MARETRON_FFM100_FRate_PGN)
    
# rotax on canbus
def rx_rotax(can, fifo):
    global engine_updated, engine_kPa, engine_rpm
    msgid, std, rtr, fmi, data = can.recv(1)                            # get received CAN frame

    engine_idx = int((msgid & 0x0F0) == 0x000)

    if msgid == 0x300 or msgid == 0x3A0:                                # boost pressure received
        engine_kPa[engine_idx] = data[5] * 10
    elif msgid == 0x102 or msgid == 0x1A2:                              # engine speed received
        engine_rpm[engine_idx] = (data[1] | (data[2] << 8)) // 2

    engine_updated[engine_idx] = True                                   # set 'received' flag

# rotax on n2k
def rotax_tx():
    global engine_updated, engine_kPa, engine_rpm
    rotsx_pgm = 127488
    engine_data = bytearray(8)                                                                       # frame data for PGN "Engine Parameters, Rapid Update"
    for idx in range(2):
        if engine_updated[idx]:                                                                      # is engine data received?
            engine_updated[idx] = False                                                              # clear 'received' flag
            struct.pack_into(b'<BHHbH', engine_data, 0,
                idx,                                                                                 # engine instance
                engine_rpm[idx],                                                                     # engine speed
                engine_kPa[idx],                                                                     # engine boost pressure
                0x7F,                                                                                # engine tilt/trim
                0xFFFF)                                                                              # reserved
            _= n2k.sendpgn(engine_data, rotsx_pgm)                                                   # send PGN "Engine Parameters, Rapid Update"
                
def main():
    # setup the can bus interface
    can = pyb.CAN(1, baudrate=250_000)
    n2k = nmea2000.NMEA2000(can)

    # set-up callbacks on received n2k packets
    
    # Product Information (fast-packet)
    n2k.rxcallback(0, product_info, pgn=N2kProdInfo.N2K_PROD_INFOR_PGN)
    # maretron
    n2k.rxcallback(0, fluid_vol_info, pgn=N2k_Maretron_FluidVol.MARETRON_FFM100_FTv_PGN)
    n2k.rxcallback(0, fluid_flo, pgn=N2k_Maretron_Temp.MARETRON_TMP100_PGN)
    n2k.rxcallback(0, fluid_temp, pgn=N2k_Maretron_FRate.MARETRON_FFM100_FRate_PGN)    
    # request info start
    n2k.statecallback(on_init)
    # canbus
    can.setfilter(1, can.LIST16, 1, (0x102, 0x1A2, 0x300, 0x3A0))                                        # allow specified id
    can.rxcallback(1, rx_rotax)                                                                          # register callback on reception of CAN frame
                                                                          
    # re-transmit from canbus to n2k
    while True:
        time.sleep(0.1)                                                                                  # 10 Hz
        # send n2k packet with rotax information
        rotax_tx()

if __name__ == '__main__':
    while True:
        try:
            main()
        except Exception as e:
            print(e)

# generate a random for your dmx function choice

# usb is connected to RS485 converter connected on DMX to DJ Stinger Spot
# The Stinger Spot is a small moving head equipped with 10W high-brightness white LEDs.The color
# that can be output is equipped with a total of 8 colors, including white, and 7 types of gobo.Supports DMX mode, 
# sound active mode and auto mode.As it is lightweight at about 3kg, it is easy to carry, too.It is a moving head with 
# excellent cost performance.

#
# https://github.com/YoshiRi/PyDMX
# using USB-RS485 Converter
#
import serial
import time
import numpy as np
import sys

class PyDMX:

    DMX512_STX_LIGHT = 0x00                           # start transmission for lighting  Default Null Start Code for Dimmers per DMX512 & DMX512/1990
    DMX512_STX_MSB_DPT = 0x01                         # Most significant Byte of double precision transmission or Soundlight
    DMX512_STX_FP_16BITW = 0x021                      # Following packet is 256 16-Bi words in Lo byte/Hi byte order
    DMX512_STX_R_A_Gray = 0x03                        # manufacturer specific RA gray or white rabbit co
    DMX512_STX_Checksum = 0x04                        # T_Recursive
    DMX512_STX_Answerback = 0x05                      # T_Recursive
    DMX512_STX_16bitLow = 0x06                        # T_Recursive
    DMX512_STX_COMP_DATA = 0x07                       # T_Recursive
    DMX512_STX_COMP_16BIT = 0x08                      # T_Recursive
    DMX512_STX_EntTech = 0x09                         # Entertainment Technology
    DMX512_STX_MODE_LIGHT = 0x0A                      # Mode Lighting Second universe of 512 channels on one data link
    DMX512_STX_GODDARD = 0x0B                         # Goddard Design
    DMX512_STX_SGM = 0x0C                             # SGM
    DMX512_STX_ENG_ART = 0x0D                         # Engineering Arts
    DMX512_STX_CETronics = 0x0E                       # CE Tronics
    DMX512_STX_MORPH = 0x0F                           # Morpheus Lights
    DMX512_STX_MPAS1 = 0xBB                           # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS2 =0xCB                            # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS3 =0xDE                            # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS4 =0xDF                            # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS5 =0xE0                            # Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_MPAS6 =0xED                            # Download Dimmer Information Martin Professional A/S Undisclosed. Conflicts with area reserved for Future Expansion of the DMX512 Standard.
    DMX512_STX_AVO_DCS =0xFF                          # AVO Dimmer Curve Select
    DMX512_STX_DCD =0xEH                              # e:cue control GmbH Device configuration data
    DMX512_STX_NSI_DCD =0xE1                          # NSI colortran Dim/Non-Dim Control
    DMX512_STX_NSI_ENR =0xE0                          # NSI colortran ENR Mode Control
    DMX512_STX_EDC_PRIO =0xDD                         # Alternate start code DD is for use in transmitting per channel priority for use in merging streams in multi-source DMX applications. Priorities will range from 0 at the low end, which means do not use the data in the corresponding slot, to 200, which means use this data over any slot data supplied with a priority between 0 and 199. Values above 200 are reserved for future use.
    DMX512_STX_E1_SIP =0xD0                           # E1 system info packet
    DMX512_STX_E1_RDM =0xCC                           # E1 E1.20 (RDM) start code
    DMX512_STX_SUN_SPECIAL =0xAA                      # Sun
    DMX512_STX_PLASA_SPECIAL =0x92                    # BSR E1.45 Alternate START Code
    DMX512_STX_E1_MAN_ID =0x91                        # 2-byte Manufacturer ID serves as an identifier that the data following in that packet is proprietary to that entity and should be ignored by all others
    DMX512_STX_PLASA_UTF8 =0x90                       # utf8 packet
    DMX512_STX_MP_SPECIAL =0x8B                       # Martin Professional A/S
    DMX512_STX_CLAY_PAKY_SPECIAL =0x8A                # CLAY PAKY S.p.A
    DMX512_STX_ANYTRON_SYNC =0x83                     # To synchronise both the memory contents and the internal clocks of lighting control equipment. Min packet length 24 bytes. Max 512.
    DMX512_STX_WYBRON_SPECIAL =0x57                   # Wybron, Inc.
    DMX512_STX_E1_TEST =0x55                          # E1 Test
    DMX512_STX_LP_SPECIAL =0x50                       # LightProcessor Ltd
    DMX512_STX_OSCAR_BACKUP =0x4F                     # Oscar Lighting AB Backup States
    DMX512_STX_AVO_SPECIAL =0x4D                      # Avolites Ltd.Proprietary function with ART2000 products
    DMX512_STX_LUX_STX =0x4C                          # 4Ch is the START Code used for OpenDMX messages, a protocol developed for use on LUX Italia products, and published by them for royalty-free use by anyone. Visit the LUX Italia website (http://www.luxitalia.eu) for a copy of the specification.
    DMX512_STX_ENFIS_SPECIAL =0x48                    # ASC is used for passing proprietary data for applications such as factory test, configuration, and software update.
    DMX512_STX_GVA_CONFIG =0x47                       # Manufacturer-specific configuration data and remote control
    DMX512_STX_COEMAR_SPECIAL =0x44                   # Coemar Spa
    DMX512_STX_CITY_SPECIAL =0x43                     # Purpose: firmware updates and product configuration
    DMX512_STX_LSC_SPECIAL =0x42                      # Proprietary remote peripheral control
    DMX512_STX_MICROLITE_SPECIAL =0x41                # mircolite
    DMX512_STX_SAND_SPECIAL =0x3F                     # SAND
    DMX512_STX_AVAB_SPECIAL =0x3E                     # AVAB spsecific info
    DMX512_STX_AVAB_SMART =0x3D                       # AVAB Smart 16 Bit Format
    DMX512_STX_AVAB_INTERNAL =0x3C                    # AVAB Internal Functions
    DMX512_STX_TIR_SPECIAL =0x35                      # Programmable DMX512-based LED controllers. Alternate START code used to specify various operating parameters for DMX512 network and standalone operation. Min.frames: 15; Max frames: 40
    DMX512_STX_TESI_SPECIAL =0x33                     # The start code's purpose is to send/receive application-specific information and execute product software update
    DMX512_STX_PR_FIRMWARE =0x30                      # The Start Code purpose for now is to be able to perform firmware updates to our products. In the future we might add more functions to it.
    DMX512_STX_JOHN_SPECIAL =0x2A                     # johnson
    DMX512_STX_HES_SPECIAL =0x26                      # high end systems
    DMX512_STX_GDS_SPECIAL =0x22                      # gds
    DMX512_STX_ELETRO_SPECIAL =0x21                   # eletro lab
    DMX512_STX_RJULIAT_SPECIAL =0x1E                  # Robert Juliat Used to update old products which do not have RDM capabilities but still supported. Also, to modify remotely some factory settings.
    DMX512_STX_DROSS_SPECIAL =0x1D                    # Dangeross Design
    DMX512_STX_KLH_RTC =0x1C                          # The alternate byte is to provide real-time updates for triple precision data ( either 20 or 24 bit ) for use with Photon Cannon project. This will allow the use of the standard to control 100 lamp fixtures at greater than 75 Hz update rate
    DMX512_STX_ESCAPE_SPECIAL =0x1B                   # The purpose is to toggle inner program of the receiver. If the Start code is 0, the machine answers with Program 1. If the Start code is 27, the machine answers with Program 2. Rest of the DMX trame would remain exactly the same as in USITT description. The purpose for us is to control different types of machines with the same DMX values (Program 2) using a switch box sending Start Code 27 and predetermined DMX values. The program run by Start Code 0 would be adapted to a fader control.
    DMX512_STX_IT_SPECIAL =0x1A                       # integrated theatre
    DMX512_STX_HUB_SPECIAL =0x19                      # hubbell
    DMX512_STX_AND_SPECIAL =0x18                      # andera
    DMX512_STX_E1_TEXT =0x17                          # E1 ANSI E1.11 Text Packet
    DMX512_STX_AL_TEXT =0x17                          # Artistic License Text Packet (matches use in ANSI E1.11)
    DMX512_STX_PER_SPECIAL =0x16                      # Peradise We build specialFx and moving set parts. The startcode will be used to identify data that is used for tacticle feedback from devices (Position, status, errors, etc)
    DMX512_STX_CDCA_CONFIG =0x15                      # Firmware update and configuration info
    DMX512_STX_SSI_SPECIAL =0x14                      # We are implementing a message-based protocol that is optimized for safe and secure motor control, embedded with lighting control. A unique Start Code is the ideal way to identify this alternate data type on the DMX network, while keeping packets as short as possible.
    DMX512_STX_ZERO88_SPECIAL =0x13                   # zero 88
    DMX512_STX_BJA_FIRMWARE =0x12                     # For updating the firmware in my equipment and to control it (Reset). My packet size will between the 3 and 256 bytes.
    DMX512_STX_TBS_SPECIAL =0x11                      # Tokyo Broadcast systems
    DMX512_STX_ADB_SPECIAL =0x10                      # ADB
    DMX512_STX_ELDO_FIRM =0xDO                        # eldo LED Configuration, firmware updates and standalone configuration. Framelength to vary from 12 to 512 bytes.

    def __init__(self,COM='COM8',Cnumber=512,Brate=250000,Bsize=8,StopB=2,use_prev_data=True,preserve_data_name="preserved_data.txt"):
        #start serial
        self.channel_num = Cnumber
        self.ser = serial.Serial(COM,baudrate=Brate,bytesize=Bsize,stopbits=StopB)
        self.data = np.zeros([self.channel_num+1],dtype='uint8')
        self.data[0] = 0                                                                                       # StartCode
        self.sleepms = 50.0
        self.breakus = 176.0
        self.MABus = 16.0
        # save filename
        self.preserve_data_name = preserve_data_name
        self.use_prev_data = use_prev_data
        # load preserved DMX data
        if use_prev_data:
            try:
                self.load_data()
            except:
                print("Its first time run or something is wrong. please check data format!")

    def set_stx_to_manufacturer(manu_code=self.DMX512_STX_JOHN_SPECIAL): # set the stx code to a manufacturer default johnson
        self.data[0] = manu_code
        
    def set_random_data(self):
        self.data[1:self.channel_num+1]= np.random.rand(self.channel_num)*255

    def set_data(self,id,data):
        self.data[id]=data

    def set_datalist(self,list_id,list_data):
        try:
            for id,data in zip(list_id,list_data):
                self.set_data(id,data)
        except:
            print('list of id and data must be the same size!')

    def send(self):
        # Send Break : 88us - 1s
        self.ser.break_condition = True
        time.sleep(self.breakus/1000000.0)
        
        # Send MAB : 8us - 1s
        self.ser.break_condition = False
        time.sleep(self.MABus/1000000.0)
        
        # Send Data
        self.ser.write(bytearray(self.data))
        
        # Sleep
        time.sleep(self.sleepms/1000.0) # between 0 - 1 sec

    def sendzero(self):
        self.data = np.zeros([self.channel_num+1],dtype='uint8')
        self.send()

    def load_data(self):
        self.data = np.loadtxt(self.preserve_data_name,dtype='int')        

    def preserve_data(self):
        np.savetxt(self.preserve_data_name,self.data)        

    def __del__(self):
        print('Close serial server!')
        # close with preserving current DMX data, I guess you may not need to reset DMX signal in this option.
        if self.use_prev_data:
            self.preserve_data()
        else:
            self.sendzero()
        self.ser.close()

    def __exit__(self):
        print('Close serial server!')
        # close with preserving current DMX data, I guess you may not need to reset DMX signal in this option.
        if self.use_prev_data:
            self.preserve_data()
        else:
            self.sendzero()
        self.ser.close()
        
TIM_DLY=1                                           # set default delay to 1 second
argc = len(sys.argv)
if argc >= 2:
    try:
        TIM_DLY=float(sys.argv[1])                  # can be read as first argument to this program
    except:
        print("argument must be valid number!")
        
# write a list of your action functions
dmx_func_list = ["funcA", "funcB", "funcC", "funcD", "funcE", "funcF", "funcG", "funcH", "funcI" ]
NUMBER_OF_DMX_FUNCS=len(dmx_func_list)                              # number of unique actions
DMX_SEQ_LEN=15                                                      # humber of actions to perform

if __name__ == '__main__':

    dmx = PyDMX('/dev/ttyUSB0')                       # the port you have connected the rs485 converter

    def funcA():
        dmx.set_data(3, 51)                           # to pink
        dmx.send()
        time.sleep(TIM_DLY)
        dmx.set_data(3, 16)                           # to orange
        dmx.send()
        time.sleep(TIM_DLY)
        dmx.set_data(4, 129)                          # gobo scroll fast/slo
        dmx.send()
        dmx.set_data(5, 241)                          # random strobe
        dmx.send()

    def funcB():
        dmx.set_data(3, 25)                           # to yellow
        dmx.send()
        time.sleep(TIM_DLY)
        dmx.set_data(3, 30)                           # to green
        dmx.send()
        time.sleep(TIM_DLY)
        dmx.set_data(4, 67)                           # gobo open shake
        dmx.send()
        dmx.set_data(5, 140)                          # shutter fast open slow close
        dmx.send()

    def funcC():
        dmx.set_data(3, 44)                           # colour to light blue
        dmx.send()
        dmx.set_data(9, 81)                           # dimmer theater
        dmx.send()

    def funcD():
        for i in range(0,10):
            dmx.set_random_data()                     # random set
            dmx.send()
            time.sleep(TIM_DLY)            

    def funcE():
        dmx.set_data(8, 251)                           # sound active mode
        dmx.send()

    def funcF():
        dmx.set_data(8, 200)                           # reset mode
        dmx.send()

    def funcG():
        dmx.set_data(8, 111)                           # blackout with gobo change
        dmx.send()

    def funcH():
        dmx.set_data(8, 91)                            # blackout with color change
        dmx.send()

    def funcI():
        dmx.set_data(8, 91)                            # blackout with pan/tilt change
        dmx.send()
        
    # single functor choice
    q=np.random.rand(1)
    dmx_our_choice = round(q[0]%NUMBER_OF_DMX_FUNCS)-1
    choice_dmx_functor = dmx_func_list[dmx_our_choice]
    if choice_dmx_functor == "funcA":
        call_dmx_functor = funcA
    elif choice_dmx_functor == "funcB":
        call_dmx_functor = funcB
    elif choice_dmx_functor == "funcC":
        call_dmx_functor = funcC  
    elif choice_dmx_functor == "funcD":
        call_dmx_functor = funcD   
    elif choice_dmx_functor == "funcE":
        call_dmx_functor = funcE  
    elif choice_dmx_functor == "funcF":
        call_dmx_functor = funcF  
    elif choice_dmx_functor == "funcG":
        call_dmx_functor = funcG  
    elif choice_dmx_functor == "funcH":
        call_dmx_functor = funcH  
    elif choice_dmx_functor == "funcI":
        call_dmx_functor = funcI          
    call_dmx_functor()

    # multiple functor choice - choose DMX_SEQ_LEN operations in the sequence 
    choice_dmx_functors = []
    call_dmx_functors = []
    for i in range(0,DMX_SEQ_LEN-1):
        q=np.random.rand(1)
        choice_dmx_functors.append(dmx_func_list[round(q[0]%NUMBER_OF_DMX_FUNCS)-1])       
        if choice_dmx_functors[i] == "funcA":
            call_dmx_functors.append(funcA)
        elif choice_dmx_functors[i] == "funcB":
            call_dmx_functors.append(funcB)
        elif choice_dmx_functors[i] == "funcC":
            call_dmx_functors.append(funcC)
        elif choice_dmx_functors[i] == "funcD":
            call_dmx_functors.append(funcD)
        elif choice_dmx_functors[i] == "funcE":
            call_dmx_functors.append(funcE)
        elif choice_dmx_functors[i] == "funcF":
            call_dmx_functors.append(funcF)
        elif choice_dmx_functors[i] == "funcG":
            call_dmx_functors.append(funcG)
        elif choice_dmx_functors[i] == "funcH":
            call_dmx_functors.append(funcH)
        elif choice_dmx_functors[i] == "funcI":
            call_dmx_functors.append(funcI)
        call_dmx_functors[i]()
        time.sleep(TIM_DLY)

    # delete class instance and close    
    del dmx   
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

class PyDMX:
    def __init__(self,COM='COM8',Cnumber=512,Brate=250000,Bsize=8,StopB=2,use_prev_data=True,preserve_data_name="preserved_data.txt"):
        #start serial
        self.channel_num = Cnumber
        #self.ser = serial.Serial(COM,baudrate=Brate,bytesize=Bsize,stopbits=StopB)
        self.data = np.zeros([self.channel_num+1],dtype='uint8')
        self.data[0] = 0 # StartCode
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

TIM_DLY=1                                       # set default delay to 1 second
if argc >= 1:
    try:
        TIM_DLY=float(argv[1])                  # can be read as first argument to this program
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
    dmx_our_choice = round(q[0]*NUMBER_OF_DMX_FUNCS)-1
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
        choice_dmx_functors.append(dmx_func_list[round(q[0]*NUMBER_OF_DMX_FUNCS)-1])       
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
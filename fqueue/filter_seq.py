#!/usr/bin/env python
# =================================================================================================
#
# Example of running a sequence of unit operations in a given order listed via the .ini set-up file
# the .ini file detrmines the order of the operations in the sequence, they are ran in this order
# unless manual interaction, ESD, or trip conditions have ocurred
# Example of SP88 batch controls
#
# ================================================================================================
#
import time
import threading
import ctypes
import pickle 
import argparse

import configparser

# for interfacing with gpio in raspberry pi
import pigpio

# gpio numbers mapping
ESD_IO = 27
LO_LEVEL = 5
LOLO_LEVEL = 6
HI_LEVEL = 17                       
SPARE2 = 27
SPARE3 = 22                       
MANUAL_PB = 23
SPARE4 = 24
SPARE5 = 25
SPARE6 = 26 

# dummy i/o and debounce seconds
DIN1 = 1
DIN2 = 0
DIN3 = 0
DIN4 = 0
DIN5 = 0
DIN6 = 0
DIN7 = 0
DEBOUNCE = 1
DEBOUNCE2 = 60

# define all the possible steps for any sequence of operations on this unit
#
from enum import Enum, auto

# this is a list of all unit operations possible on this plant (unit)
#
class SequenceSteps(Enum):
    FILTERING = auto()	
    DRAIN_DOWN = auto()
    WAIT_FOR_LOW_LEVEL = auto()
    BOOK_AIRBLOWER = auto()	
    OPEN_AIR_VLVS = auto()
    START_AIRBLOWER = auto()
    AIRBLOWING = auto()
    STOP_AIRBLOWER = auto()
    CLOSE_AIR_VLVS = auto()
    BOOK_WASHWATER = auto()	
    OPEN_WASH_VLVS = auto()
    START_WASHPUMP = auto()
    WASHING = auto()
    STOP_WASHPUMP = auto()
    CLOSE_WASH_VLVS = auto()	
    BOOK_WASH_AIR = auto()
    OPEN_WASH_AIR = auto()
    START_WASH_AIR = auto()
    COMBINED_WASH_BLOW = auto()
    STOP_WASH_AIR = auto()
    CLS_WASH_AIR = auto()
    SETTLE = auto()
    SLOW_FILL = auto()
    FILL = auto()
    SLOW_RAMP_OUTLET = auto()
    ISOLATE = auto()
    ISOLATED = auto()
    MAN_OUT = auto()
    RESTORE = auto()
    RESTART = auto()
    SKIP = auto()
    
# recipe parameters
#
RAMP_UP_PERIOD = 20
RAMP_WT_PERIOD = 2
RAMP_DN_PERIOD = 20
RMP_STEP_TM = 0.1
SETTLE_PERIOD = 600
FILL_UP_PERIOD = 150
SLOW_START_PERIOD = 12000

# outputs to valves and pumps (for example modbus relays in i/o or PLC)
A = 0b1                                             # air blower inlet
B = 0b10                                            # air blower outlet
C = 0b100                                           # wash water inler
D = 0b1000                                          # wash water outlet
E = 0b10000                                         # inlet valve                                       
F = 0b100000
G = 0b1000000
H = 0b10000000

# define the alarm handler levels
FTO_ALM=4                                          # failed to open / start
FTC_ALM=5                                          # failed to close / stop
SEQ_TO=6                                           # unit operation timeout

# adafruit MCP4725 DAC on i2c
# sudo pip3 install adafruit-circuitpython-mcp4725
import board
import busio
import adafruit_mcp4725
# Initialize I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
# Initialize MCP4725. and create object
dac = adafruit_mcp4725.MCP4725(i2c)

# enums which define what happens after release of the ESD button
RETURN_OFF = 0
RETURN_BACK = 1

# external book equipment variables (will be read from external unit which runs the shared resource we are booking
# TBD : interface
set_ab = 0
book_back_ab = 0
ab_run = 0
set_wp = 0
book_wash = 0
wp_run = 0

# manual interaction values for each valve (i.e each of the device(s) in this unit)
# TBD : interface
hmiA = 0
hmiB = 0
hmiC = 0
hmiD = 0
hmiE = 1
hmiPID_AM = 1
hmi_OvPos = 60
hmi_LCSpt = 66
hmi_recover_mode = 0

# Class wrapper for threading 
# you can also kill task e.g. if op_task.is_alive(): op_task.raise_exception()
#
class twe(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return

    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')
            
# this class represents the unit operations of the sequence
#
class Wash_sequence():
    def __init__(self, i_state=0, out=0, delv=20.0, delv_slo=1.0):
        print("Inititialised Ramp function sequence class")
        self._out = 0
        self._omax = 4095
        self._omin = 10
        self._delta = delv                                              # fast ramp mode
        self._delta_slo = delv_slo                                      # slow ramp mode
        self._state = i_state                                           # state engine state
        self._pstate = i_state                                          # saved state before isolate trip occured
        self._last_state = self._state 
        self._pb = 1.0
        self._run = True
        self._t = time.time()
        self._opword = E                                                # inititalise with E (inlet) active
        self.seq_pos = 0
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._dac = adafruit_mcp4725.MCP4725(self._i2c)	
        self.repeats = 0
        self._pi = pigpio.pi()
        self._esd_return_state = RETURN_OFF                            # default esd return state
        self.n = 0
        self._mem_state = 0

    # hard wired ESD with actions        
    def cb_esd(gpio, level, tick):
        print (f'# {gpio=} : {level=} : {tick/(1000*1000)=}s')
        if level == 1:                                                                              # hard wired ESD pressed
            self._state = SequenceSteps.ISOLATE.value
        elif level == 0 and self._esd_return_state == RETURN_OFF:                                   # set to restart when released           
            self._state == SequenceSteps.RESTART.value
        elif level == 0 and self._esd_return_state == RETURN_BACK:                                  # set to restore when released 
            self._state = SequenceSteps.RESTORE.value  
            
    def init_esd(self, esd_gpio):
        self._pi.set_mode(esd_gpio, pigpio.INPUT)                                                    # set-up esd with a callback function
        self._pi.set_pull_up_down(esd_gpio,pigpio.PUD_UP)
        cb = self._pi.callback(esd_gpio, pigpio.EITHER_EDGE, self.cb_esd)                            # call back function will be called on either edge

    def init_io_input(self, gpio_no):
        self._pi.set_mode(gpio_no, pigpio.INPUT)
        self._pi.set_pull_up_down(gpio_no,pigpio.PUD_UP)  

    def init_io_output(self, gpio_no):
        self._pi.set_mode(gpio_no, pigpio.OUTPUT) 
        
    def ramp_up(self, delta):
        if self._out + delta <= self._omax:
            self._out += delta
        else:
            self._out = self._omax		
        time.sleep(RMP_STEP_TM)
                    	
    def ramp_down(self, delta):
        if self._out - delta >= self._omin:
            self._out -= delta
        else:
            self._out = self._omin
        time.sleep(RMP_STEP_TM)

    def drive_out(self):
        self._dac.raw_output = self._out

    def isolate(self):
        if self._state == SequenceSteps.ISOLATE.value:
            self._opword = 0
            #set_out = 0
            self._state = SequenceSteps.ISOLATED.value     
           
    # perfrom main operation sequence 
    #  
    # this routine re-starts exactly where it stopped the ramp function
    # 
    def handler(self, signum, frame):
        print(f' signal handler active (signum={signum})')
        if not ((self._state == SequenceSteps.SLOW_RAMP_OUTLET.value) or (self._state == SequenceSteps.ISOLATE.value)):
            self._pstate = self._state
            self._state = SequenceSteps.ISOLATE.value
            self.isolate()
        if signum == FTO_ALM:
            print("failed to start/open alarm")
        elif signum == FTC_ALM:
            print("failed to start/open alarm")  
        elif signum == SEQ_TO:
            print("unit operation timed out") 
            
    # time out on each unit operation
    #    
    def timer_procedure(self, wait_tim):
        time.sleep(wait_tim)
        if not ((self._state == SequenceSteps.SLOW_RAMP_OUTLET.value) or (self._state == SequenceSteps.ISOLATE.value)):     # activate sig_alrm handler
            signal.alarm(SEQ_TO)                                

    # timeout on device = valve/pump operation, task (to wait and confirm) thread to run conconcurrently with sequecne
    #
    def valve_check(self, wait_tim, vnum, state = True):
        time.sleep(wait_tim)
        valves_list = [A, B, C, D, E]
        valve = valves_list[vnum]
        if state and not (valve and self._inword):                      # activate sig_alrm handler if we didnt reach the state
            signal.alarm(FTO_ALM)                                       # failed to start or open
        elif not state and (valve and self._inword):                    # activate sig_alrm handler if we didnt reach the state
            signal.alarm(FTC_ALM)                                       # failed to close or stop

    # as above with confirmation to continue
    #
    def valve_check_confirm(self, wait_tim, vnum, state = True):
        #valves_list = [A, B, C, D, E]
        #valve = valves_list[vnum]
        valve = vnum
        checking = True        
        while checking == True:
            time.sleep(wait_tim)
            if state and not (valve and self._inword):                      # activate sig_alrm handler if we didnt reach the state
                signal.alarm(FTO_ALM)                                       # failed to start or open
            elif not state and (valve and self._inword):                    # activate sig_alrm handler if we didnt reach the state
                signal.alarm(FTC_ALM)                                       # failed to close or stop
            else:
                checking = False
            
    def drain_down_operation(self):
        print("start draining opertion...")
        signal.signal(signal.SIGALRM, self.handler)
        self.n = time.time()
 
        if self._state == SequenceSteps.FILTERING.value:
            self._state == SequenceSteps.DRAIN_DOWN.value
            
        timer = twe(target=self.timer_procedure, args=(wait_tim=3000), kwargs={})  
        timer.start()        
        # wait for drain down ramp up the output to drain it out      
        while self._state == SequenceSteps.DRAIN_DOWN.value:  
            self._opword &= ~E                                                   # close inlet valve  
            print("identifying the change",time.time()-self.n)
            if (time.time()-self.n > 0.1) and (self._mem_state == 1):            # we went offline and are recalling this step
                adapted_start_point = time.time() - (self.n - self._t)
                print("adapted the start point as we were offline",adapted_start_point, time.time()-adapted_start_point)
                self._t = adapted_start_point
            self._last_state = self._state    
            self.ramp_up(self._delta)
            self.n = time.time()
            self._mem_state = 1
            print("ramping up...",self._out,self._opword)
            if (self.n - self._t) >= RAMP_UP_PERIOD:
                self._state = SequenceSteps.WAIT_FOR_LOW_LEVEL.value
                self._t = self.n
                print("ramped up to...",self._out,self._opword)
                self._mem_state = 2
                timer.raise_exception()

        #self._out = 0                                                           # close outlet valve
        signal.alarm(0)
        timer.join()            
        self._run = False                                                        # stop monitor thread as well until next operation begins	
        print("completed opertion...draindown ramp up")

    def wait_low_level(self):
        print("start waiting for low level opertion...")
        signal.signal(signal.SIGALRM, self.handler)

        timer = twe(target=self.timer_procedure, args=(wait_tim=10000), kwargs={})  
        timer.start()          
        
        # wait for drain down ramp up the output to drain it out
        while self._state == SequenceSteps.WAIT_FOR_LOW_LEVEL.value:  
            low_level = self.read_gpio(LO_LEVEL)
            if low_level == 0:
                self._state = SequenceSteps.BOOK_AIRBLOWER.value

        self._out = 0                                                   # close outlet valve      
        signal.alarm(0)
        timer.join()    
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...reached low level")
       
    def book_airblower(self):
        print("start book airblower opertion...")
        signal.signal(signal.SIGALRM, self.handler)
        
        set_ab = 1                                                    # send a signal to the AB system to book it for use with this unit
        
        while self._state == SequenceSteps.BOOK_AIRBLOWER.value:  
            if book_back_ab == 1:
                self._state = SequenceSteps.OPEN_AIR_VLVS.value

        set_ab = 0                                                    # clear the book signal        
        self._run = False                                              # stop monitor thread as well	
        print("completed opertion...book air blower")

    def airblower_valve_op_operation(self):
        print("airblower valve open opertion...")
        signal.signal(signal.SIGALRM, self.handler)
         
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start()        

        while self._state == SequenceSteps.OPEN_AIR_VLVS.value:  
            self._opword |= A | B                                               # open valves 
            if (self._inword & 3)==3:
                self._state = SequenceSteps.START_AIRBLOWER.value
                timer.raise_exception()  
                
        signal.alarm(0)
        timer.join()            
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...airblower valve open")

    # alternative with alarms for each valve if not reached limit
    #
    def airblower_valve_op_operation_alarm(self):
        print("airblower valve open opertion...which alarms if it doesnt reach limit and waits infinitly")
        signal.signal(signal.SIGALRM, self.handler)
  
        chk_a = twe(target=self.valve_check_confirm, args=(wait_tim=60, vnum=A), kwargs={})  
        chk_b = twe(target=self.valve_check_confirm, args=(wait_tim=60, vnum=B), kwargs={})
        chk_a.start()
        chk_b.start()
               
        while self._state == SequenceSteps.OPEN_AIR_VLVS.value:  
            self._opword |= A | B                                                                                               # open valves     
            if (self._inword & 3)==3:
                self._state = SequenceSteps.START_AIRBLOWER.value
                chk_a.raise_exception()  
                chk_b.raise_exception()
                
        signal.alarm(0)
        chk_a.join()
        chk_b.join()        
        self._run = False                                                                                                    # stop monitor thread as well	
        print("completed opertion...airblower valve open")
        
    def airblower_run_operation(self):
        print("airblower run opertion...")
        signal.signal(signal.SIGALRM, self.handler)
 
        self._opword |= A | B                                                                                                 # open valves         
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start() 
       
        while self._state == SequenceSteps.START_AIRBLOWER.value: 
            while ab_run == 0:                                                                                                # wait for the airblower to run
            self._state = SequenceSteps.AIRBLOWING.value   
        while self._state == SequenceSteps.AIRBLOWING.value:             
            if ab_run == 0:                                                                                                   # wait for the airblower to stop
                self._state = SequenceSteps.CLOSE_AIR_VLVS.value
                timer.raise_exception()
                   
        signal.alarm(0)
        timer.join()            
        self._run = False                                                                                                      # stop monitor thread as well	
        print("completed opertion...airblower running")

    def airblower_valve_cl_operation(self):
        print("airblower valve close opertion...")
        signal.signal(signal.SIGALRM, self.handler)
 
       
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start()        

        while self._state == SequenceSteps.CLOSE_AIR_VLVS.value:  
            self._opword &= ~A & ~B                                               # close valves  
            if ((self._inword & 3)==0):
                self._state = SequenceSteps.BOOK_WASHWATER.value
                timer.raise_exception()  
                
        signal.alarm(0)
        timer.join()            
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...airblower valve closed")
 
    def book_wash_pump(self):
        print("start book washpump opertion...")
        signal.signal(signal.SIGALRM, self.handler)
        
        set_wp = 1                                                    # send a signal to the AB system to book it for use with this unit
        
        while self._state == SequenceSteps.BOOK_WASHWATER.value:  
            if book_wash == 1:
                self._state = SequenceSteps.OPEN_WASH_VLVS.value

        set_wp = 0                                                          # clear the book signal        
        self._run = False                                                   # stop monitor thread as well	
        print("completed opertion...book wash pump")

    def wash_pump_valve_op_operation(self):
        print("washpump valve open opertion...")
        signal.signal(signal.SIGALRM, self.handler)
 
       
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start()        

        while self._state == SequenceSteps.OPEN_WASH_VLVS.value:  
            self._opword |= C | D                                            # open valves  
            if (self._inword & 12)==12:
                self._state = SequenceSteps.START_WASHPUMP.value
                timer.raise_exception()  
                
        signal.alarm(0)
        timer.join()            
        self._run = False                                                    # stop monitor thread as well	
        print("completed opertion...wash pump valve open")

    def wash_pump_run_operation(self):
        print("wash pump run opertion...")
        signal.signal(signal.SIGALRM, self.handler)
 
    
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start() 
       
        while self._state == SequenceSteps.START_WASHPUMP.value: 
            self._opword |= C | D                                               # open valves     
            while wp_run == 0:                                                  # wait for the run
            self._state = SequenceSteps.WASHING.value   
        while self._state == SequenceSteps.WASHING.value:             
            if wp_run == 0:                                                     # wait for the stop
                self._state = SequenceSteps.CLOSE_WASH_VLVS.value
                timer.raise_exception()
                   
        signal.alarm(0)
        timer.join()            
        self._run = False                                                       # stop monitor thread as well	
        print("completed opertion...wash pump running")

    def wash_pump_valve_cl_operation(self):
        print("wash pump valve close opertion...")
        signal.signal(signal.SIGALRM, self.handler)
       
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start()        

        while self._state == SequenceSteps.CLOSE_WASH_VLVS.value:  
            self._opword &= ~C & ~D                                               # close valves   
            if ((self._inword & 12)==0):
                self._state = SequenceSteps.SETTLE.value
                timer.raise_exception()  
                
        signal.alarm(0)
        timer.join()            
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...washpump valve close")

    def settle_operation(self):
        print("start settling opertion...")
        signal.signal(signal.SIGALRM, self.handler)
        self.n = time.time()
        
        timer = twe(target=self.timer_procedure, args=(wait_tim=3000), kwargs={})  
        timer.start()        

        while self._state == SequenceSteps.SETTLE.value:  
            print("identifying the change",time.time()-self.n)
            if (time.time()-self.n > 0.1) and (self._mem_state == 6):   # we went offline and are recalling this step
                adapted_start_point = time.time() - (self.n - self._t)
                print("adapted the start point as we were offline",adapted_start_point, time.time()-adapted_start_point)
                self._t = adapted_start_point
            self._last_state = self._state    
            self.n = time.time()
            self._mem_state = 6
            if (self.n - self._t) >= SETTLE_PERIOD:
                self._state = SequenceSteps.FILL.value
                self._t = self.n
                self._mem_state = 7
                timer.raise_exception()
                
        signal.alarm(0)
        timer.join()            
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...settle")

    def fill_operation(self):
        print("start filling opertion...")
        signal.signal(signal.SIGALRM, self.handler)
        self.n = time.time()
        
        self._opword |= E        
        timer = twe(target=self.timer_procedure, args=(wait_tim=3000), kwargs={})  
        timer.start()        

        while self._state == SequenceSteps.FILL.value:  
            print("identifying the change",time.time()-self.n)
            if (time.time()-self.n > 0.1) and (self._mem_state == 8):   # we went offline and are recalling this step
                adapted_start_point = time.time() - (self.n - self._t)
                print("adapted the start point as we were offline",adapted_start_point, time.time()-adapted_start_point)
                self._t = adapted_start_point
            self._last_state = self._state    
            self.n = time.time()
            self._mem_state = 8
            if (self.n - self._t) >= FILL_UP_PERIOD:
                self._state = SequenceSteps.SLOW_RAMP_OUTLET.value
                self._t = self.n
                self._mem_state = 9
                timer.raise_exception()
                
        signal.alarm(0)
        timer.join()            
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...fill")

    def slow_start_operation(self):
        print("start slow start opertion...")
        signal.signal(signal.SIGALRM, self.handler)
        self.n = time.time()
        
        timer = twe(target=self.timer_procedure, args=(wait_tim=3000), kwargs={})  
        timer.start()        

        self._opword |= E                                                   # ensure inlet valve open         
        while self._state == SequenceSteps.SLOW_RAMP_OUTLET.value:  
            print("identifying the change",time.time()-self.n)
            if (time.time()-self.n > 0.1) and (self._mem_state == 10):       # we went offline and are recalling this step
                adapted_start_point = time.time() - (self.n - self._t)
                print("adapted the start point as we were offline",adapted_start_point, time.time()-adapted_start_point)
                self._t = adapted_start_point
            self._last_state = self._state    
            self.ramp_up(self._delta)
            self.n = time.time()
            self._mem_state = 10
            print("ramping up...",self._out,self._opword)
            if (self.n - self._t) >= SLOW_START_PERIOD:
                self._state = SequenceSteps.FILTERING.value
                self._t = self.n
                self._mem_state = 11
                timer.raise_exception()

        #self._out = 0                                                   # close outlet valve
        signal.alarm(0)
        timer.join()            
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...slow start")

    # these are alternative steps that can be chosen to wash air and water combined
    #
    def book_washpump_airblower(self):
        print("start book airblower and washpump opertion...")
        signal.signal(signal.SIGALRM, self.handler)

        if SequenceSteps.BOOK_AIRBLOWER.value:       
            set_ab = 1                                                    # send a signal to the AB system to book it for use with this unit
            set_wp = 1
            self._state == SequenceSteps.BOOK_WASH_AIR.value  
            
        while self._state == SequenceSteps.BOOK_WASH_AIR.value:  
            if book_back == 1 and book_wash == 1:
                self._state = SequenceSteps.OPEN_WASH_AIR.value

        set_wp = 0
        set_ab = 0                                                    # clear the book signal        
        self._run = False                                              # stop monitor thread as well	
        print("completed opertion...book air blower and wash pump")

    def airblower_wash_pump_valve_op_operation(self):
        print("airblower washpump valve open opertion...")
        signal.signal(signal.SIGALRM, self.handler)
       
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start()        

        while self._state == SequenceSteps.OPEN_WASH_AIR.value:  
            self._opword |= A | B | C | D                                               # open valves   
            if (self._inword & 15)==15:
                self._state = SequenceSteps.START_WASH_AIR.value
                timer.raise_exception()  
                
        signal.alarm(0)
        timer.join()            
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...airblower washpump valve open")

    def airblower_washpump_run_operation(self):
        print("combined wash run opertion...")
        signal.signal(signal.SIGALRM, self.handler)
 
        self._opword |= A | B | C | D                                                                                          # open valves         
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start() 
       
        while self._state == SequenceSteps.START_WASH_AIR.value: 
            while ab_run == 0 and wp_run == 0:                                                                                 # wait for the airblower to run
            self._state = SequenceSteps.COMBINED_WASH_BLOW.value   
        while self._state == SequenceSteps.COMBINED_WASH_BLOW.value:             
            if ab_run == 0 and wp_run == 0:                                                                                    # wait for the airblower to stop
                self._state = SequenceSteps.CLS_WASH_AIR.value
                timer.raise_exception()
                   
        signal.alarm(0)
        timer.join()            
        self._run = False                                                                                                      # stop monitor thread as well	
        print("completed opertion...combined wash running")

    def airblower_washpump_valve_cl_operation(self):
        print("combined wash valve close opertion...")
        signal.signal(signal.SIGALRM, self.handler)
         
        timer = twe(target=self.timer_procedure, args=(wait_tim=300), kwargs={})  
        timer.start()        

        while self._state == SequenceSteps.CLS_WASH_AIR.value:  
            self._opword &= ~A & ~B & ~C & ~D                                               # close valves 
            if ((self._inword & 15)==0):
                self._state = SequenceSteps.SETTLE.value
                timer.raise_exception()  
                
        signal.alarm(0)
        timer.join()            
        self._run = False                                                                   # stop monitor thread as well	
        print("completed opertion...combined wash valve closed")
                
    def manual_interaction(self, level_cont):
        print("entered manual operation state...")
        signal.signal(signal.SIGALRM, self.handler)
        time.sleep(1)
        db=0
        
        while self._state == SequenceSteps.MAN_OUT.value:  
            readA, readB, readC, readD, readE, readLC = hmiA, hmiB, hmiC, hmiD, hmiE, hmiPID_AM    # these reads will read the manual operation HMI
            readOvPos = hmi_OvPos                                                                  # manual position of the outlet valve
            readLCSpt = hmi_LCSpt                                                                  # level controller local set-point (target filter level)
            self._opword = int(readA) | (2*int(readB)) | (4*int(readC)) | (8*int(readD)) | (16*int(readE))             # write word to valves
            if readLC == 0:                                                                        # set controller output (valve %open) if manual request to do so
                level_cont.set_output(readOvPos)
            elif readLC == 1:                                                                      # let pid control valve (%open) if requested by HMI
                level_cont.set_auto()
            level_cont.set_local()                                                                 # controller local (level only primary loop)
            level_cont.set_spt(readLCSpt)                                                          # write set-point as per the HMI
     
            DIN7 = self.read_gpio(MANUAL_PB)                                                       # read if we wish to exit manual state
            if not DIN7:                                                                           # we release the manual operation request
                db += 1
                if db > DEBOUNCE:
                    self._state = self._pstate                                                     # return to previous step
                    
        signal.alarm(0)         
        self._run = False                                                                          # stop monitor thread as well	
        print("completed opertion...airblower valve open")
        
    def read_gpio(self, pin):
        return self._pi,read(pin)  

    def save_pickle_file(self, p, o):
        with open(p, 'wb') as f:
            pickle.dump(o, f) 

    # a thread to save state for restore on power failure
    def power_save(self, p, o):
        print("powersave thread...")
        while self._run == True:
            self.save_pickle_file(p, o)
            time.sleep(1.0)
        print("powersave thread ended...")
        
    # perfrom exception continuous logic concurrently to the chosen sequence operation task.
    #                 
    def exception_thread(self, lc):
        print("started exception monitoring...")
        din2_on, din3_on, din4_on, din5_on, din6_on, din7_on = 0, 0, 0, 0, 0, 0 
        cnt1, cnt2, cnt3, cnt4, cnt5, cnt6 = 0, 0, 0, 0, 0, 0
        while self._run == True:
            DIN5 = self.read_gpio(LOLO_LEVEL)                           # low low level trip                       
            DIN7 = self.read_gpio(MANUAL_PB)                            # if pressed allows manual interaction with this unit
            low_level = self.read_gpio(LO_LEVEL)
            high_level = self.read_gpio(HI_LEVEL)
            
            if DIN7 == 1 and din7_on == 0:                              # go to manaul for manual interaction with the unit
                cnt6 += 1
                if cnt6 > DEBOUNCE:
                    self._pstate = self._state
                    self._state = SequenceSteps.MAN_OUT.value           # allow the values to be written from the screen
                    din7_on = 1
                    print("in manual")
                    man_task = twe(name = 'Thread Manual Interaction', target=self.manual_interaction, args=(lc), kwargs={})
                    man_task.start()
                    man_task.join()                                     # waits for this thread to end.. choose to leave manual state
                    print("out manual")
            elif DIN7 == 0:                                             # exited manual go back to last known step 
                cnt6 += 1
                if cnt6 > 2*DEBOUNCE:
            #        if din7_on == 1:                                   # return from manual back to where we where in the sequence
            #            self._state = self._pstate                     # in this case its done from the thread itself
                    din7_on = 0
                    cnt6 = 0

            if DIN5 == 0 and din5_on == 0:                              # very low low level conductivity prove is uncovered (open contact)
                cnt5 += 1
                if cnt5 > DEBOUNCE:
                    self._pstate = self._state
                    # --------------- need mutex around this shutdown -------------------------
                    self._state = SequenceSteps.ISOLATE.value           # isolate all valves on the unit
                    din5_on = 1
                    print("very low level",self._out)
                    time.sleep(0.2)
                    self.isolate()
                    # --------------- end mutex -------------------------
            elif DIN5 == 1:                                             # exited manual go back to last known step 
                cnt5 += 1
                if cnt5 > 2*DEBOUNCE:
                    din5_on = 0
                    cnt5 = 0

            if self._state == SequenceSteps.FILTERING.value:            # filter state only a low level will close the outlet
                if low_level == 0 and din5_on == 0:                     # low level conductivity prove is uncovered (open contact)
                    cnt4 += 1
                    if cnt4 > DEBOUNCE:
                        lc.set_output(0)                  
                        din4_on = 1
                elif low_level == 1:                                    # exited manual go back to last known step 
                    cnt4 += 1
                    if cnt4 > 2*DEBOUNCE:
                        lc.set_auto()
                        din4_on = 0
                        cnt4 = 0

            if self._state == SequenceSteps.FILTERING.value or self._state == SequenceSteps.SLOW_RAMP_OUTLET.value:            # filter state only a hi level will close the outlet
                if high_level == 1 and din5_on == 0:                    # hi level conductivity prove is covered (close contact)
                    cnt3 += 1
                    if cnt3 > DEBOUNCE2:
                        self._opword &= ~E                              # close inlet                  
                        din3_on = 1
                elif high_level == 0:                                   # exited manual go back to last known step 
                    cnt3 += 1
                    if cnt3 > 2*DEBOUNCE2:
                        self._opword |= E                               # open inlet
                        din3_on = 0
                        cnt3 = 0
                        
            self.drive_out()                                            # drive the dac output
                                                 
        self._run = True                                                # re-enable monitor thread for next time we start the main thread
        print("completed exception monitoring...")


# specify 1 for power down recovery otherwise 0
POWER_SAVE = 1

# import the PID routine
import LevelPID

if __name__ == "__main__": 
    try:
        parser = argparse.ArgumentParser(description='command line parse')
        parser.add_argument('-r', '--restore', type=int, dest='cFlag', default=0, help='set to 1 to restore the controller from last saved settings')
        parser.add_argument('-p', '--filter', type=str, dest='pName', default="filter", help='filter name to save/recall parameters for')
        args = parser.parse_args()
        
        nm = args.pName
        if len(nm.split(".")) == 2:                                     # already passed extension
            f_name = nm
        else:
            f_name = nm+".pickle"                                       # create pickle file name
        print(f_name)
    
        if args.cFlag == 1:                                              # if we passed r -1 as arguemnts we re-load the sequence where it last was
            print("\033[36m re-loading saved sequence controller! \033[0m")
            with open(f_name, 'rb') as f:
                r = pickle.load(f)                                       # load the last saved sequencer from pickle file
                print("controller output ", r._out)
        else:                 
            r = Wash_sequence()                                          # create class instance for the sequencer  

        r.init_esd(ESD_IO)                                               # set interrupt pin for GPIO doing emergency shutdown function
        r.init_io_input(LOLO_LEVEL)
        r.init_io_input(MANUAL_PB)
        r.init_io_input(LO_LEVEL)
        r.init_io_input(HI_LEVEL)
                
        lpid = LevelPID.levelPID()                                                                            # create a level PID instance
        pid_task = twe(name = 'Thread Level PID', target=lpid.run_pid_loop, args=(), kwargs={})               # start it
        pid_task.start()
        
        # os.kill PID with SIGUSR1 -- puts it to manual
        
        # list the order of operations in the sequence we wish to run in this batch operation 
        #
        # this might be retrieved from a csv line input like shown belo
        #
        # this is the default sequence if not in the .ini file
        sequence_of_op_tasks = [ r.drain_down_operation, r.wait_low_level, r.book_airblower, r.airblower_valve_op_operation, r.airblower_valve_cl_operation, r.airblower_run_operation, r.book_wash_pump \
                                 r.book_wash_pump, r.wash_pump_valve_op_operation, r.wash_pump_run_operation, r.wash_pump_valve_cl_operation, r.settle_operation, r.fill_operation, r.slow_start_operation ] 
                                 
        # the dictionary lists all possible operations and they are chosen from the list in the .ini file if they are wrong order they will be skipped
        mt = { 'dd_op' : r.drain_down_operation, 'low_levl' : r.wait_low_level, 'book_ab' : r.book_airblower, 'ab_op_vlv' : r.airblower_valve_op_operation, 'ab_cl_vlv' : r.airblower_valve_cl_operation, \
               'ab_run' : r.airblower_run_operation, 'ab_vlv_al' : r.airblower_valve_op_operation_alarm, 'book_wp' : r.book_wash_pump, 'wp_vlv_op' : r.wash_pump_valve_op_operation, \
               'comb_book' : r.book_washpump_airblower, 'comb_vlv_op' : r.airblower_wash_pump_valve_op_operation, 'comb_run' : r.airblower_washpump_run_operation, 'comb_vlv_cl' : r.airblower_washpump_valve_cl_operation, \
               'wp_run' : r.wash_pump_run_operation, 'wp_vlv_cls' : r.wash_pump_valve_cl_operation, 'settle' : r.settle_operation, 'fill' : r.fill_operation, 'slow_start' : r.slow_start_operation  }

        # read configuration .ini file
        config_ini = configparser.ConfigParser()
        config_ini.read('sequence_recipe.ini', encoding='utf-8') 
        sections = config_ini.sections()   
        found = 0 
        for s in sections:
            if s == nm:                                                                                                         # if unit name is found
                csv_str=str(config_ini[nm]['UNIT_OPERATION_LIST']).replace(' ','') 
                csv_list = csv_str.split(",")                
                sequence_of_op_tasks = []
                for z in range(0,len(csv_list)):
                    sequence_of_op_tasks.append(mt[csv_list[z]])                                                                 # make the unit operation list per unit
                found = 1
                break
        if not found:
           print(f"\033[33;44m using default sequence as none was found in sequence_recipe.ini \033[0m")

        # ----------------------------- enter filter state with low level exception monitor ------------------------------
        r._state = SequenceSteps.FILTERING.value
        r._run = True
        lpid.set_auto()                                                                                                            # PID controller in level control mode
        r._opword |= E
        exc_task = twe(name = 'Thread Monitor', target=r.exception_thread, args=(lpid), kwargs={})                                # monitor exceptions
        exc_task.start() 
        time.sleep(300)                                                                                                           # filter in auto
        lpid.set_manual()                                                                                                         # set the PID to manual so the sequence controls the output
        exc_task.raise_exception()                                                                                                # stop the monitor task       
        exc_task.join()  

        # ----------------------------- run the sequence of unit operations ---------------------------------------------            
        for i, tsk in enumerate(sequence_of_op_tasks):                                                                            # run the sequence as 2 concurrent threads, operation and monitor                                              
            if i == r.seq_pos:                                                                                                    # restored from the last sequence position
                print("--------- Seq pos",i,r.seq_pos)
                while (r._state >= SequenceSteps.ISOLATE.value) and (r._state < SequenceSteps.RESTORE.value):                     # ------- wait if we have isolated / manual ---------------
                    lpid.set_output(0)
                start_tasks = 1
                while start_tasks == 1:                                                                                           # re-start if we isolate and re-stored
                    exc_task = twe(name = 'Thread Monitor', target=r.exception_thread, args=(lpid), kwargs={})
                    op_task = twe(name = 'Thread Operation', target=tsk, args=(), kwargs={})
                    if POWER_SAVE == 1:
                        ps = twe(name = 'Thread powersave', target=r.power_save, args=(f_name,r), kwargs={})                       # saves the class in the background for power save restore
                    exc_task.start()                                                                                               # start the threads
                    op_task.start()
                    if POWER_SAVE == 1:
                        ps.start()
                    op_task.join()                                                                                                 # wait for join (completion) of the thread operations
                    exc_task.join()
                    if POWER_SAVE == 1:
                        ps.join()                
                    while (r._state >= SequenceSteps.ISOLATE.value) and (r._state < SequenceSteps.RESTORE.value):                  # ------- wait if we have isolated / manual ---------------
                        lpid.set_output(0)
                        if hmi_recover_mode == 1:                                                                                  # choice to recover from isolate made from HMI
                            r._state = SequenceSteps.RESTORE.value  
                            hmi_recover_mode = 0
                        elif hmi_recover_mode == 2:     
                            r._state = SequenceSteps.SKIP.value                                                                    # adavance to the next step in the sequence  
                            hmi_recover_mode = 0
                        elif hmi_recover_mode == 3:     
                            r._state = SequenceSteps.RESTART.value                                                                  # re-start and go to filtering skipping all remaining sequence (manually completed)  
                            hmi_recover_mode = 0                            
                    if r._state == SequenceSteps.RESTORE.value:                                                                     # chose to restore the sequence 
                        r._state = r._pstate
                    elif r._state == SequenceSteps.SKIP.value:                                                                      # chose to restore the sequence 
                        r._state = r._pstate+1
                        start_tasks = 0   
                        r.seq_pos += 1                         
                    elif r._state == SequenceSteps.RESTART.value:                                                                # chose to restart the sequence 
                        r._state = SequenceSteps.SLOW_RAMP_OUTLET.value                                                             # move the step to here only 
                        r.seq_pos += 1  
                        start_tasks = 0
                    else:
                        r.seq_pos += 1                                                                                              # increment pointer to next operation in the sequence_of_op_tasks
                        start_tasks = 0
        r.seq_pos = 0                                                                                                               # reset the pointer when complete before saving
        with open(f_name, 'wb') as f:
            pickle.dump(r, f)     

        while r._state >= SequenceSteps.FILTERING.value:
            lpid.set_auto()                                                                                                           # set the PID to auto run until interrupted 

        pid_task.raise_exception()
        pid_task.join()
        
    except KeyboardInterrupt:	
        print("\033[31m task sequencer was killed \033[0m")
        op_task.raise_exception()
        exc_task.raise_exception()
        with open(f_name, 'wb') as f:
            pickle.dump(r, f)
        op_task.join()                                                                                                              # wait for join (completion) of the thread operations
        exc_task.join()
        pid_task.raise_exception()                                                                                                  # stop pid
        pid_task.join()
        del lpid
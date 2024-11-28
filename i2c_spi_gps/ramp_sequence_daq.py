# ====================================================================================
# Example of running a sequence of ramp operations as specified for a given patient id
#
# it should be like this............
# read din1+2
# start ramp function set A
# if din2 ramp delta up
# if din3 ramp delta down
# time expired or end_stop di4
# wait for delay remove A
# start ramp down set B
# if din2 ramp delta up
# if din3 ramp delta down
# time expired or end_stop di5 remove B
#
# at any point di6 toggles the operation of C
#
# the program will restart where it was last with the option -r 1
# the actual patient id is specified as -p "pat_name"
# ==================================================================================

import time
import threading
import ctypes
import pickle 
import argparse

# for interfacing with gpio in raspberry pi
import pigpio
import time

# gpio numbers mapping
ESD_IO = 27
RAMP_UP_END = 5
RAMP_DN_END = 6
RAMP_UP_PB = 17                       
RAMP_DN_PB = 27
C_ACTIVE_PB = 22                       
MANUAL_PB = 23
START_PB = 24
# free GPIO25/26 

# dummy i/o
DIN1 = 1
DIN2 = 0
DIN3 = 0
DIN4 = 0
DIN5 = 0
DIN6 = 0
DIN7 = 0
DEBOUNCE = 1

# operation sequence steps
#
WAIT_FOR_START = 0
RAMP_UP = 1
WAIT_DELAY = 2
RAMP_DOWN = 3
STOP_OFF = 4
MAN_OUT = 5
ESD = 6

# recipe parameters
#
RAMP_UP_PERIOD = 20
RAMP_WT_PERIOD = 2
RAMP_DN_PERIOD = 20
RMP_STEP_TM = 0.1
RAMP_WT_PERIOD2 = 10

# outputs (for example modbus relays)
A = 0b1
B = 0b10
C = 0b100
OPWORD = A | B | C

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

class Ramp_sequence():
    def __init__(self, i_state=0, out=0, delv=20.0, delv_slo=1.0):
        print("Inititialised Ramp function sequence class")
        self._out = 0
        self._omax = 4095
        self._omin = 10
        self._delta = delv                                              # fast ramp mode
        self._delta_slo = delv_slo                                      # slow ramp mode
        self._state = i_state
        self._last_state = self._state 
        self._pb = 1.0
        self._run = True
        self._t = time.time()
        self._opword = C                                                # inititalise with C active
        self.seq_pos = 0
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._dac = adafruit_mcp4725.MCP4725(i2c)	
        self.repeats = 0
        self._pi = pigpio.pi()
        self._esd_return_state = RETURN_OFF                            # default esd return state
               
    def cb_esd(gpio, level, tick):
        print (f'# {gpio=} : {level=} : {tick/(1000*1000)=}s')
        if level == 1:
            self._state = ESD
        elif level == 0 and self._esd_return_state == RETURN_OFF:            
            r.seq_pos = 99                                              # disable all further actions
            self._state = WAIT_FOR_START  
        elif level == 0 and self._esd_return_state == RETURN_BACK: 
            self._state = self._last_state                              # return to last step  
            
    def init_esd(self, esd_gpio):
        self._pi.set_mode(esd_gpio, pigpio.INPUT)                       # set-up esd with a callback function
        self._pi.set_pull_up_down(esd_gpio,pigpio.PUD_UP)
        cb = self._pi.callback(esd_gpio, pigpio.EITHER_EDGE, self.cb_esd)  # call back function will be called on either edge

    def init_io_input(self, gpio_no):
        self._pi.set_mode(gpio_no, pigpio.INPUT)
        self._pi.set_pull_up_down(gpio_no,pigpio.PUD_UP)  

    def init_io_output(self, gpio_no):
        self._pi.set_mode(gpio_no, pigpio.OUTPU) 
        
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
        self._dac..raw_output = self._out
                            
    # perfrom main operation sequence 
    # 
    # 1. wait for start
    # 2. ramp up
    # 3. wait
    # 4. ramp down
    # 5. end
    #    
    def main_operation_thread(self):
        print("start opertion...")
        if DIN1 == 1 and self._state == WAIT_FOR_START:
            self._last_state = self._state
            self._state = RAMP_UP
            self._t = time.time()
            print("ramp function begin...",self._opword)
        while self._state == RAMP_UP:  
            self._last_state = self._state
            self._opword |= A                                           # add A      
            self.ramp_up(self._delta)
            n = time.time()
            print("ramping up...",self._out,self._opword)
            if (n - self._t) >= RAMP_UP_PERIOD:
                self._state = WAIT_DELAY
                self._t = n
                print("ramped up to...",self._out,self._opword)
        while self._state == WAIT_DELAY:
            self._last_state = self._state
            self._opword &= ~A                                          # remove A  
            n = time.time()
            if (n - self._t) >= RAMP_WT_PERIOD:
                self._state = RAMP_DOWN
                self._t = n
                print("now ramping down...",self._opword)
        while self._state == RAMP_DOWN:
            self._last_state = self._state
            self._opword |= B                                           # add B   
            self.ramp_down(self._delta)  
            n = time.time()
            print("ramping down...",self._out,self._opword)
            if (n - self._t) >= RAMP_DN_PERIOD:
                self._state = STOP_OFF
                self._t = n
                print("ramped down to...",self._out,self._opword)
        while self._state == STOP_OFF:
            self._last_state = self._state
            self._opword &= ~B                                          # remove B 
            self._out = 0
            n = time.time()
            if (n - self._t) >= 1:
                self._state = WAIT_FOR_START
                self._t = n
        while self._state == MAN_OUT:                                   # goto manual state
            man_val = int(input("enter the manual output required "))
            if man_val > self._omax:
                man_val = self._omax
            elif man_val > self._omin:
                man_val = self._omin
            self._out = man_val               
            time.sleep(0.1)
        while self._state == ESD:                                       # esd present
            self._out = 0
            
        self._run = False                                               # stop monitor thread as well	
        print("completed opertion...")

    # perfrom secondary operation sequence 
    # 
    # repeat no_of_repeats 
    # 1. wait for start
    # 2. ramp up
    # 3. wait
    # 4. ramp down
    # 5. repeats < no_of_repeats goto 2 or repeats > no_of_repeats goto 6
    # 6. end
    # 
    def secondary_operation_thread(self):
        print("start secondary...")
        self.repeats = 0
        no_of_repeats = 2                                              # number of times we repeat these steps in this sequence
        if DIN1 == 1 and self._state == WAIT_FOR_START:
            self._last_state = self._state
            self._state = RAMP_UP
            self._t = time.time()
            print("ramp function begin...",self._opword)
        while self._state == RAMP_UP:  
            self._last_state = self._state
            self._opword |= A                                           # add A      
            self.ramp_up(self._delta_slo)
            n = time.time()
            print("rampeding up...",self._out,self._opword)
            if (n - self._t) >= RAMP_UP_PERIOD:
                self._state = WAIT_DELAY
                self._t = n
                print("ramped up to...",self._out,self._opword)
        while self._state == WAIT_DELAY:
            self._last_state = self._state
            self._opword &= ~A                                          # remove A  
            n = time.time()
            if (n - self._t) >= RAMP_WT_PERIOD2:
                self._state = RAMP_DOWN
                self._t = n
                print("now ramping down...",self._opword)
        while self._state == RAMP_DOWN:
            self._last_state = self._state
            self._opword |= B                                           # add B   
            self.ramp_down(self._delta_slo)  
            n = time.time()
            if (n - self._t) >= RAMP_DN_PERIOD:
                self._state = STOP_OFF
                self._t = n
                print("ramped down to...",self._out,self._opword)
        while self._state == STOP_OFF:
            self._last_state = self._state
            self._opword &= ~B                                          # remove B 
            self._out = 0
            n = time.time()
            if (n - self._t) >= 1:
                if self.repeats == no_of_repeats:
                    self._state = WAIT_FOR_START
                else:
                    self._state = RAMP_UP
                    self.repeats +=1
                self._t = n
        while self._state == MAN_OUT:                                   # goto manual state
            man_val = int(input("enter the manual output required "))
            if man_val > self._omax:
                man_val = self._omax
            elif man_val > self._omin:
                man_val = self._omin
            self._out = man_val               
            time.sleep(0.1)
          			               
        self._run = False                                               # stop monitor thread as well	
        print("completed secondary...")

    def read_gpio(self, pin):
        return self._pi,read(pin)  

    def save_pickle_file(self, p, o):
        with open(p, 'wb') as f:
            pickle.dump(o, f) 

    # a thread to save state for restore on power failure
    def power_save(self, p, o):
        print("powersave thread...")
        while self._run == True:
            self.save_pickle_file(p,o)
            time.sleep(1.0)
        print("powersave thread ended...")
        
    # perfrom exception continuous logic concurrently to the chosen operation task.
    # 
    # reads all gpio then
    # 1. increase/decrease ammount of ramp
    # 2. either end stop reached stop ramp regardless of time
    # 3. drive auxillary relay C 
    # 4. go to manual state
    #                 
    def exception_thread(self):
        print("started exception monitoring...")
        din2_on, din3_on, din4_on, din5_on, din6_on, din7_on = 0, 0, 0, 0, 0, 0 
        cnt1, cnt2, cnt3, cnt4, cnt5, cnt6 = 0, 0, 0, 0, 0, 0
        while self._run == True:
            DIN4 = self.read_gpio(RAMP_UP_END)                          # read gpio inputs for end stops, ramp controls, actuator C, manual state
            DIN5 = self.read_gpio(RAMP_DN_END)
            DIN2 = self.read_gpio(RAMP_UP_PB)                        
            DIN3 = self.read_gpio(RAMP_DN_PB)
            DIN6 = self.read_gpio(C_ACTIVE_PB)                          
            DIN7 = self.read_gpio(MANUAL_PB)
            DIN1 = self.read_gpio(START_PB) 
            
            if DIN2 == 1:                                               # increase ramp ammount
                cnt1 += 1
                if cnt1 > DEBOUNCE:
                    din2_on = 1
            elif DIN2 == 0 and din2_on == 1:                                                                                    
                self._delta = self._delta + self._pb                    # perfrom on falling edge (just shown for example)
                print("increased ramp ammount to...",self._delta)
                din2_on = 0
                cnt1 = 0
            elif DIN3 == 1 and din3_on == 0:                            # decrease ramp ammount
                cnt2 += 1
                if cnt2 > DEBOUNCE:
                    self._delta = self._delta + self._pb                # perfrom on rising edge
                    print("decreased ramp ammount to...",self._delta)
                    din3_on = 1
            elif DIN3 == 0 :
                cnt2 += 1
                if cnt2 > 2*DEBOUNCE:
                    din3_on = 0  
                    cnt2 = 0                 
            if DIN4 == 1 and din4_on == 0:                              # end ramp up 
                cnt3 += 1
                if cnt3 > DEBOUNCE:
                    if not self._state < WAIT_DELAY:
                        self._state = WAIT_DELAY                        # do on rising edge
                    din4_on = 1
                    print("ramp up reached max...",self._out)
            elif DIN4 == 0 :
                cnt3 += 1
                if cnt3 > 2*DEBOUNCE:
                    din4_on = 0        
                    cnt3 = 0          
            if DIN5 == 1 and din5_on == 0:                              # end ramp down
                cnt4 += 1
                if cnt4 > DEBOUNCE:
                    if not self._state == WAIT_FOR_START:
                        self._state = STOP_OFF                          # do on rising edge
                    din5_on = 1
                    print("ramp down reached min...",self._out)
            elif DIN5 == 0 :
                cnt4 += 1
                if cnt4 > 2*DEBOUNCE:
                    din5_on = 0 
                    cnt4 = 0 
            if DIN6 == 1 and din6_on == 0:                              # toggle the C contact
                cnt5 += 1
                if cnt5 > DEBOUNCE:
                    self._opword ^= C                                   # action upon rising edge
                    din6_on = 1
                    print("togled valve C",self._opword)
            elif DIN6 == 0:
                cnt5 += 1
                if cnt5 > 2*DEBOUNCE:
                    din6_on = 0
                    cnt5 = 0
            if DIN7 == 1 and din7_on == 0:                              # go to manaul for setting the output
                cnt6 += 1
                if cnt6 > DEBOUNCE:
                    self._state = MAN_OUT 
                    din7_on = 1
                    print("in manual",self._out)
            elif DIN7 == 0:                                             # exited manual go back to last known step 
                cnt6 += 1
                if cnt6 > 2*DEBOUNCE:
                    if din7_on == 1:                                    # return from manual back to where we where in the sequence
                        self._state = self._last_state
                    din7_on = 0
                    cnt6 = 0
            self.drive_out()                                            # drive the dac output
                                                 
        self._run = True                                                # re-enable monitor thread for next time we start the main thread
        print("completed exception monitoring...")

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

# specify 1 for power down recovery otherwise 0
POWER_SAVE = 1

if __name__ == "__main__": 
    try:
        parser = argparse.ArgumentParser(description='command line parse')
        parser.add_argument('-r', '--restore', type=int, dest='cFlag', default=0, help='set to 1 to restore the controller from last saved settings')
        parser.add_argument('-p', '--patient', type=str, dest='pName', default="seq", help='patient name to save/recall parameters for')
        args = parser.parse_args()

        nm = args.pName
        if len(nm.split(".")) == 2:                                     # already passed extension
            f_name = nm
        else:
            f_name = nm+".pickle"                                       # create pickle file name
        print(f_name)
    
        if args.cFlag == 1:                                             # if we passed r -1 as arguemnts we re-load the sequence where it last was
            print("\033[36m re-loading saved sequence controller! \033[0m")
            with open(f_name, 'rb') as f:
                r = pickle.load(f)                                      # load the last saved sequencer from pickle file
                print("controller output ", r._out)
        else:                 
            r = Ramp_sequence()                                         # create class instance for the sequencer  

        r.init_esd(ESD_IO)                                              # set interrupt pin for GPIO doing emergency shutdown function
        r.init_io_input(RAMP_UP_END)                                    # set the DIN interface to be type inputs
        r.init_io_input(RAMP_DN_END)
        r.init_io_input(RAMP_UP_PB)
        r.init_io_input(RAMP_DN_PB)
        r.init_io_input(C_ACTIVE_PB)
        r.init_io_input(MANUAL_PB)
        r.init_io_input(START_PB)
        
        # list the order of operations in the sequence we wish to run in this batch operation 
        #
        # this might be retrieved from a csv line input like shown below
        # mt = { 'main_op' : r.main_operation_thread, 'sec_op' : r.secondary_operation_thread }
        # csv_list='main_op','sec_op','main_op','main_op'     
        # sequence_of_op_tasks = []
        # for z in range(0,len(csv_list)):
        #     sequence_of_op_tasks.append(mt[csv_list[z].split(",")[0]])        
        sequence_of_op_tasks = [ r.main_operation_thread, r.secondary_operation_thread, r.main_operation_thread, r.main_operation_thread ]   

        for i, tsk in enumerate(sequence_of_op_tasks):                  # run the sequence as 2 concurrent threads, operation and monitor                                              
            if i == r.seq_pos:                                          # restored from the last sequence position
                print("--------- Seq pos",i,r.seq_pos)
                exc_task = twe(name = 'Thread Monitor', target=r.exception_thread, args=(), kwargs={})
                op_task = twe(name = 'Thread Operation', target=tsk, args=(), kwargs={})
                if POWER_SAVE == 1:
                    ps = twe(name = 'Thread powersave', target=r.power_save, args=(f_name,r), kwargs={})
                exc_task.start()                                        # start the threads
                op_task.start()
                if POWER_SAVE == 1:
                    ps.start()
                op_task.join()                                          # wait for join (completion) of the thread operations
                exc_task.join()
                if POWER_SAVE == 1:
                    ps.join()                
                r.seq_pos += 1                                          # increment pointer to next operation in the sequence_of_op_tasks

        r.seq_pos = 0                                                   # reset the pointer when complete before saving
        with open(f_name, 'wb') as f:
            pickle.dump(r, f)                       
    except KeyboardInterrupt:	
        print("\033[31m task sequencer was killed \033[0m")
        op_task.raise_exception()
        exc_task.raise_exception()
        with open(f_name, 'wb') as f:
            pickle.dump(r, f)

# simple example of showing 2 timed procedures with also a way of changing
# a variable via a signal interupt call
#
# the example therefore demonstrates iterprocess communication via the signal SIGUSR1
# this for example could be another process which wants to increase the global counter GOB
#
import signal
import time
import os
import multiprocessing 
from multiprocessing import Process
import sys

# adafruit MCP4725 DAC on i2c
# sudo pip3 install adafruit-circuitpython-mcp4725
import board
import busio
import adafruit_mcp4725
# Initialize I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
# Initialize MCP4725. and create object
dac = adafruit_mcp4725.MCP4725(i2c)

# multiprocessing variables must be used between mp_threads and signal
run_f=multiprocessing.Value('i',1)                                      # global variables which can be written from timer_procedure or signal handler
times=multiprocessing.Value('i',0)
# this global is controlled by this and external processes using kill -SIGUSR1
GOB=multiprocessing.Value('i',0)                                        # externally controlled varible by other processes via kill -SIGUSR1 <PID> signal 
MY_PID=multiprocessing.Value('i',0)
# this global is controlled by external processes using kill -SIGUSR2
NUM_ITER=multiprocessing.Value('i',8)                                   # number of iterations of main loop

TIMED_OPS=10                                                            # number of operations to perform when timed loop has activated
# main 4 times then only timed loop runs until NUM_ITER
ML_TIME=4                                                               # main loop number of times to run
TL_TIME=20                                                              # timed loop run times over NUM_ITER is disabled
# 4 main loop and 2 timed
#ML_TIME=4                                                               # main loop number of times to run
#TL_TIME=2                                                               # timed loop run times over NUM_ITER is disabled
KL_TIME=20                                                              # kill (end entire sequence time)

TIMER_PERIOD=5                                                          # how long timed operation waits until starting and stopping main process
OPERATION_PERIOD=0.7                                                    # time duration between operations

DAC_TM_UP = 1000
DAC_TM_DN = 900
DAC_MN_UP = 3000
DAC_MN_DN = 400

# this is the thread terminator function
def handler(signum, frame):
  print(f' kill signal handler active (signum={signum})')
  sys.exit(-1)

# this is the user signal handler it adds one the the GOB variable
def handler2(signum, frame):
  print(f' signal handler active (signum={signum})')
  #raise OSError("os error raised")
  GOB.value+=1
  print(GOB.value)

def handler3(signum, frame):
  print(f' external trigger to reduce by 1 (signum={signum})')
  with NUM_ITER.get_lock():
    if not NUM_ITER.value <= 0:
      NUM_ITER.value -= 1
              
def timer_procedure():
  # sleeep
  print("started the timer proc")
  time.sleep(TIMER_PERIOD)                                              # wait for the timer period
  # use to kill this timed procedure
  print("times value ",times.value)
  with run_f.get_lock():
    run_f.value = 0                                                     # disable the main loop
  if (times.value >= TL_TIME):                                          # cut of number of times for this operation
    os.kill(os.getpid(), signal.SIGTERM)                                # will kill (stop) this procedure 
  if (times.value >= KL_TIME):
    os.kill(MY_PID.value, signal.SIGTERM)                               # use to kill the whole procedure
  os.kill(os.getpid(), signal.SIGUSR1)                                  # will run function handler counts GOB up
  for f in range(0, TIMED_OPS):
    print("up down (timed) routine active")
    print("value == .",DAC_TM_UP)
    dac = DAC_TM_UP
    time.sleep(OPERATION_PERIOD)
    print("value == ",DAC_TM_DN)
    dac = DAC_TM_DN
  with times.get_lock():
    times.value += 1

  print("end the timer proc")  

if __name__ == "__main__":     
    # signal.SIGALRM can be defined for program signal handling
    signal.signal(signal.SIGUSR1, handler2)                             # define signal handler routines for each signal SIGUSR1 is count GOB 
    signal.signal(signal.SIGUSR2, handler3)                             # define signal handler routines for each signal SIGUSR2 decrease NUM_ITER
    signal.signal(signal.SIGTERM, handler)                              # TERM exits

    MY_PID.value=os.getpid()
    print("my pid is ",MY_PID.value)
    os.kill(os.getpid(), signal.SIGUSR1)                                # issue a SIGUSR1 to this PID to increment the global GOB value (simulate external process sending) IPC

    ni = 0
    while (ni < NUM_ITER.value):
        timer = Process(target=timer_procedure) 
        timer.start()
        # runnin loop
        while run_f.value == 1:                                         # runs continously until timer expires
            print("value.main == ",DAC_MN_DN)
            dac = DAC_MN_DN
            time.sleep(OPERATION_PERIOD)
            print("value main == ",DAC_MN_UP)
            dac = DAC_MN_UP
        if not (times.value >= ML_TIME):                                # run main loop up to ML_TIME times then just let timer work
            with run_f.get_lock():
                run_f.value = 1
        print("exited",times.value)
        print(GOB.value)
        timer.join()
        ni += 1

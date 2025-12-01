#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Example of timed sequence and external signal control operations in python
#
import signal
import time
from multiprocessing import Process
import os
from signal_listener import SignalListener, SignalInterrupt

# signal handler which activates on a 5 second interrupt
def handler(signum, frame):
    print(f"sig {signum}")

# signal handler which activates on a user1 kill
def handler_u1(signum, frame):
    print(f"sig {signum}")
    raise OSError("handler 1 active")

def timer_procedure():
    time.sleep(5)
    os.kill(os.getppid(), signal.SIGUSR1)

if __name__ == '__main__':
    signal.signal(signal.SIGALRM,handler)                    # define signal hansler
    signal.setitimer(signal.ITIMER_REAL, 5,5)                # interrupt timer every 5 seconds by raising signal
    print("starting")
    time.sleep(20)                                           # wair 20 seconds
    print("completed waiting")	
    signal.setitimer(signal.ITIMER_REAL, 0,0)                # disable timed interrupt
    signal.signal(signal.SIGUSR1, handler_u1)                # this is external kill signal can be another python program (we will raise it internally here)
    timer = Process(target=timer_procedure)
    timer.start()	
    print("started time thread")
    while True:
        print("running.")
        time.sleep(0.7)	
    print("stopped time thread")
    timer.join()
    signal.alarm(5)
    time.sleep(4)
    signal.alarm(7)	
    time.sleep(2)
    signal.alarm(6)
    listener = SignalListener(signal.SIGUSR1, signal.SIGUSR2)
    try:
        with listener.listen():
            while True:
                print("in loop.")
                time.sleep(0.7)
    except SignalInterrupt.SIGUSR1:
        print("SIGUSR1")
    except SignalInterrupt.SIGUSR2:
        print("SIGUSR2")
    else:
        print("signal not 1 or 2")
    finally:
        print("completed exercise")
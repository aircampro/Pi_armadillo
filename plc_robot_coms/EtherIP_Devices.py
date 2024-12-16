#!/usr/bin/env python
#
# Example using ethernet IP to various devices
#
import logging
import sys
import time
import threading

import cpppo
logging.basicConfig(**cpppo.log_cfg)

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
			
# Device IP in 1st arg, or 'localhost' (run: python -m cpppo.server.enip.poll_test) ip of NU-EP1
hostname = sys.argv[1] if len(sys.argv) > 1 else '10.0.1.2'

from cpppo.server.enip import poll
# ------------------------ Keyence NU-EP1 ----------------------------------
from cpppo.server.enip.ab import powerflex_750_series as device

params = [
    ('@1/0x1/0x7','SSTRING'),                         # Device name, should read: NU-EP1
    ('@0x66/0x1/0x0325', 'UINT')                      # Current value of sensor, should read: distance between fibers
]

def failure(exc):
    failure.string.append(str(exc))

failure.string = []

def process(par, val):
    process.values[par] = val

process.done = False
process.values = {}

poller = twe(
    name = 'Thread NU-EP1', target=poll.poll, args=(), kwargs={
        'proxy_class':  device,
        'address':      (hostname, 44818),
        'cycle':        0.001,
        'timeout':      0.5,
        'process':      process,
        'failure':      failure,
        'params':       params,
    })

poller.start()

# ------------------------ A-B PowerFlex, ------------------------------
# from cpppo.server.enip.ab import powerflex_750_series as device
params2                 = [ "Motor Velocity", "Output Current" ]

hostname2                = '10.0.1.8'
values2                  = {}                                                                    # { <parameter>: <value>, ... }
poller2                  = twe(
    name = 'Thread AB-PF', target=poll.poll, args=(device,), kwargs={
        'address':      (hostname2, 44810),
        'cycle':        1.0,
        'timeout':      0.5,
        'process':      lambda par,val: values2.update( { par: val } ),
        'params':       params2,
    })
#poller2.daemon           = True
poller2.start()

# -------------- Node ACS Linear Actuator (Tolomatic Stepper), ----------
from cpppo.server.enip.get_attribute import proxy_simple as device2

params3                  = [('@4/100/3','INT'),('@1/1/7','SSTRING')]

hostname3                = '10.0.1.6'
values3                  = {}                                                                    # { <parameter>: <value>, ... }
poller3                  = twe(
    name = 'Thread ACS-LA', target=poll.poll, args=(device2,), kwargs={
        'address':      (hostname3, 44811),
        'cycle':        1.0,
        'timeout':      0.5,
        'process':      lambda par,val: values3.update( { par: val } ),
        'params':       params3,
    })
#poller3.daemon           = True
poller3.start()

if __name__ == "__main__":
    # Monitor the process.values {} and failure.string [] (updated in another Thread)
    try:
        while True:
            while process.values:
                par,val = process.values.popitem()
                print("%s: %16s == %r" % (time.ctime(), par, val))
            
            while failure.string:
                exc = failure.string.pop( 0 )
                print("%s: %s" %(time.ctime(), exc))
            while values2:
                print( "%s: %16s == %r" % (time.ctime(), *values2.popitem()) )
            while values3:
                print( "%s: %16s == %r" % (time.ctime(), *values3.popitem()) )
            time.sleep( 0.1 )
            # time.sleep(0.1)
    finally:
        process.done = True
        if poller.is_alive(): poller.raise_exception()
        if poller2.is_alive(): poller2.raise_exception()
        if poller3.is_alive(): poller3.raise_exception()
        poller.join()
        poller2.join()
        poller3.join()
#!/usr/bin/env python
 
"""
WriteDataByIdentifier AUTOSAR Can Bus 
"""
# 
import isotp
import logging
import time
import threading

# The CAN Bus implemented for the Vector interface
from can.interfaces.vector import VectorBus

class ThreadedApp:
   def __init__(self):
      isotp_params = {
         'stmin' : 0, 
         'blocksize' : 4,
         'wftmax' : 0,
         'll_data_length' : 8,
         'tx_padding' : 0xCC,
         'rx_flowcontrol_timeout' : 1000,
         'rx_consecutive_frame_timeout' : 1000,
         'squash_stmin_requirement' : False,
         'can_fd' : False,
         'tx_data_min_length' : 8
      }
      self.exit_requested = False
      #self.bus = VectorBus(channel='0', bitrate=500000)
      self.bus = VectorBus(channel='0', bitrate=500000, fd=True)
      # this is a list of addressing possible with the library
      #
      # isotp.Address(isotp.AddressingMode.Normal_11bits, rxid=0x123, txid=0x456)
      # isotp.Address(isotp.AddressingMode.Normal_29bits, rxid=0x123456, txid=0x789ABC)
      # isotp.Address(isotp.AddressingMode.NormalFixed_29bits, source_address=0x11, target_address=0x22)
      # isotp.Address(isotp.AddressingMode.Extended_11bits, rxid=0x123, txid=0x456, source_address=0x55, target_address=0xAA)
      # isotp.Address(isotp.AddressingMode.Extended_29bits, rxid=0x123456, txid=0x789ABC, source_address=0x55, target_address=0xAA)
      # isotp.Address(isotp.AddressingMode.Mixed_11bits, rxid=0x123, txid=0x456, address_extension=0x99)
      # isotp.Address(isotp.AddressingMode.Mixed_29bits, source_address=0x11, target_address=0x22, address_extension=0x99)
      #
      addr = isotp.Address(isotp.AddressingMode.NormalFixed_29bits, source_address=0xF1, target_address=0x10) 
      self.stack = isotp.CanStack(self.bus, address=addr, params=isotp_params, error_handler=self.my_error_handler)

   def start(self):
      self.exit_requested = False
      self.thread = threading.Thread(target = self.thread_task)
      self.thread.start()

   def stop(self):
      self.exit_requested = True
      if self.thread.isAlive():
         self.thread.join()
   
   def send_msg(self, msg):
      self.stack.send(msg)

   def receive_msg(self):
      return self.stack.recv()
      
   def my_error_handler(self, error):
      logging.warning('IsoTp error happened : %s - %s' % (error.__class__.__name__, str(error)))

   def thread_task(self):
      while self.exit_requested == False:
         self.stack.process()                                        # Non-blocking
         #time.sleep(self.stack.sleep_time())                        # Variable sleep time based on state machine state
         time.sleep(0.001)                                           # Variable sleep time based on state machine state

   def shutdown(self):
      self.stop()
      self.bus.shutdown()

def sendrecv( app, msg ):
   
   if msg[0] == 0x00:
      tsleep = msg[1]*0x100+msg[2]
      tsleep = tsleep / 1000
      print("sleep : %f [ms]" % tsleep)
      time.sleep(tsleep)
   else:
      print("Send msg : %s" % (msg.hex()))
      #app.stack.send(msg)
      app.send_msg(msg)
      t1 = time.time()
      p2 = 1.0   
      while time.time() - t1 < p2:
         if app.stack.available():
            #payload = app.stack.recv()
            payload = app.receive_msg()
            print("Recv msg : %s" % (payload.hex()))
            if payload[0] == 0x7F and payload[2] == 0x78:
               t1 = time.time()
               p2 = 5.0    
            else:
               break
         time.sleep(0.001)


if __name__ == '__main__':
   app = ThreadedApp()
   app.start()
   
   datas=[
      bytes([0x22, 0x56, 0x78]),
      bytes([0x2E, 0x56, 0x78, 0x11, 0x22, 0x33, 0x44]),
      bytes([0x10, 0x03]),
      bytes([0x22, 0x56, 0x78]),
      bytes([0x2E, 0x56, 0x78, 0x11, 0x22, 0x33, 0x44]),
      bytes([0x27, 0x13]),
      bytes([0x22, 0x56, 0x78]),
      bytes([0x27, 0x14, 0xDE, 0xAD, 0xBE, 0xEF]),
      bytes([0x2E, 0x56, 0x78, 0x11, 0x22, 0x33, 0x44]),
      bytes([0x22, 0x56, 0x78]),
      bytes([0x2E, 0x56, 0x78, 0x01, 0x02, 0x03, 0x04]),
      bytes([0x22, 0x56, 0x78]),
   ]
   
   #while True: if you want to forever
   
   # send the datas listed above
   for i in range(len(datas)):
      sendrecv(app, datas[i])

   print("Exiting")
   app.shutdown()

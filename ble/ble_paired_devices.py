#!/usr/bin/env python 
# -*- coding: utf-8 -*- 
# example of using bleak to read paired BLE devices as listed below
#
import asyncio
from bleak import discover, BleakClient

# here write the names of the paired devices
device1 = "xxxxxxxx"
device2 = "xxxxxxxx"

class Device1Handler(BleakClient):
   """
       Args:
       address_or_device (`BLEDevice` or str): the Bleak device returned after scanning, or a bluetooth device address.
   """
   def __init__(self, address_or_device):
       self._client = BleakClient(address_or_device)
       sself.xxxx = xxx

   def _device1_cb(self, sender, data):
       """
           Write callback device1 function
           For example:
           self.xxx = struct.unpack("hhb",data)
       """
       if self._device1_cb is not None:
           self._device1_cb(self.data)
       else:
           print("device1_cb is None")

   async def connect(self, timeout):
       """Connect to the thermometer device.
       Args:
           timeout (float or None): The timeout to connect to the device in seconds, or None to wait forever.
       Returns:
           Boolean representing the connection status.
       """
       return await self._client.connect(timeout=timeout)

   async def disconnect(self):
       """Disconnect from the thermometer device.
       Returns:
           Boolean representing if device is disconnected.
       """
       self._deivce1_cb = None
       return await self._client.disconnect()

   async def start(self, device1_cb):
       """Setup and/or read from the device.
       Returns right away if the device is not connected.
       """
       if not await self._client.is_connected():
           return
       """
       Write what do you want
       """
       
class Device2Handler(BleakClient):
   """
       Args:
       address_or_device (`BLEDevice` or str): the Bleak device returned after scanning, or a bluetooth device address.
   """
   def __init__(self, address_or_device):
       self._client = BleakClient(address_or_device)
       sself.xxxx = xxx

   def _device2_cb(self, sender, data):
       """
           Write callback device1 function
           For example:
           self.xxx = struct.unpack("hhb",data)
       """
       if self._device2_cb is not None:
           self._device2_cb(self.data)
       else:
           print("device2_cb is None")

   async def connect(self, timeout):
       """Connect to the thermometer device.
       Args:
           timeout (float or None): The timeout to connect to the device in seconds, or None to wait forever.
       Returns:
           Boolean representing the connection status.
       """
       return await self._client.connect(timeout=timeout)

   async def disconnect(self):
       """Disconnect from the thermometer device.
       Returns:
           Boolean representing if device is disconnected.
       """
       self._deivce1_cb = None
       return await self._client.disconnect()

   async def start(self, device1_cb):
       """Setup and/or read from the device.
       Returns right away if the device is not connected.
       """
       if not await self._client.is_connected():
           return
       """
       Write what do you want
       """
       
async def paring_func(device):
   if device.name == device1:
       dev = Device1Handler(device)
       cb = device1_cb
   elif device.name == device2:
       dev = Device2Handler(device)
       cb = device2_cb 
   else:
       print("Unknown device")
       return
   try:
       await dev.connect(None)
   except bleak.exc.BleakError:
       print("Connection Error")
       return
   await dev.start(cb)
   await asyncio.sleep(5)
   await dev.disconnect()

def device1_cb(xxx):
    """
    Write device1 callback
    """

def device2_cb(xxx):
    """
    Write device1 callback
    """
    
async def main():
   names: tuple = (device1, device2, ....)
   paring_tasks: list = []
   while True:
       print("Start scan")
       devices = await discover()
       for d in devices:
           if d.name.startswith(names):
               paring_tasks.append(asyncio.create_task(paring_func(device=d)))
       if paring_tasks:
           [await task for task in paring_tasks]
       paring_tasks = []

if __name__ == "__main__":
    asyncio.run(main())
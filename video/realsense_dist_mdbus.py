#!/usr/bin/env python
#
# relasense depth to modbus tcp slave (server) values 
#
import sys
import pyrealsense2 as rs
import struct
import asyncio
import pymodbus
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.server.sync import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.transaction import ModbusRtuFramer
from pymodbus.payload import BinaryPayloadBuilder
import platform

pipeline = rs.pipeline()                                       # Create a pipeline 
pipeline.start()                                               # Start streaming the data
do_it = 1
NO_IR_INTS = 4                                                 # 1 double == 4 16 bit integers in modbus

MY_TCP_SLAVE="localhost"
TCP_SLAVE_PORT=502

# check the version of python we have for compatibility with asyncio
def chk_python_version():
    my_py = platform.python_version()
    p = my_py.split(".")
    v = 0
    lp = len(p)
    m = 100
    else:
        print(f"unknown version of python {my_py} ?")
    for s in range(0, lp):
        v += (int(p[s])*m)
        m /= 10 
    return v
    
# modbus tcp to HMI data transfer
async def update_datablock(store: ModbusSlaveContext):
    print('start of modbus tcp slave (server) interface')
    while do_it == 1:

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            continue

        width, height = depth_frame.get_width(), depth_frame.get_height()
        dist = depth_frame.get_distance(width // 2, height // 2)
        print(f"The camera is facing an object {dist:.3f} meters away", end="\r")
        # <	Littleendian
        # >	Big Endian
        # !	Network
        # =	Native
        # @	Native (with alignment
        d_bytes = struct.pack('d', dist)                              # make 64bit int from double precision number
        w_bytes = struct.unpack('HHHH', d_bytes)

        for b in range(0, len(w_bytes)):                              # foreach byte
            store.setValues(4, b, w_bytes[b])                         # write distance to object to 4 modbus 16bit registers (double)               		

async def run_tcp():
    global do_it
    try:
        # we are biulding a custom data block area
        builder = BinaryPayloadBuilder(byteorder=Endian.Big)           
        for i in range(0, NO_IR_INTS):                                # make a database for modbus with 4 IR integers 16 bit                               
            builder.add_16bit_int(i)
        block = ModbusSequentialDataBlock(1, builder.to_registers())
        store = ModbusSlaveContext( ir=block )
        context = ModbusServerContext(slaves=store, single=True)
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'AirCamPro'
        identity.ProductCode = 'RDM'
        identity.VendorUrl = 'http://github.com/aircampro'
        identity.ProductName = 'Realsense Camera Pymodbus Server'
        identity.ModelName = 'Realsense Camera Pymodbus Server'
        identity.MajorMinorRevision = '1.0'
        task1 = asyncio.create_task(update_datablock(store))
        task1.set_name("modbus tcp server task started")
        await StartTcpServer(context, identity=identity, address=(MY_TCP_SLAVE, TCP_SLAVE_PORT))
    except Exception as e:
        print("Error on line {}".format(sys.exc_info()[-1].tb_lineno))
        print(e)
    finally:
        do_it = 0
        task1.cancel()                                              # stop updating the live data
        pipeline.stop()                                             # Stop streaming
        return 1

async def order():
    print('start of realsense to modbus server.......')  
    task2 = asyncio.create_task(run_tcp()) 
    r1 = await task2
    
if __name__ == "__main__":
    pymodbus.pymodbus_apply_logging_config("DEBUG") 
    pv = chk_python_version()   
    if pv >= 370:                                                 # python 3.7 or higher    
        try:
            # python 3.7 of higher
            asyncio.run(order(), debug=True )
        except AttributeError:
            # For Python 3.6 a bit more code is required to run the main() task on
            # an event loop.
            loop = asyncio.get_event_loop()
            loop.run_until_complete(order())
            loop.close()
    elif pv >= 300:
        try:
            # For Python 3.6 a bit more code is required to run the main() task on
            # an event loop.
            loop = asyncio.get_event_loop()
            loop.run_until_complete(order())
            loop.close()
        except AttributeError:
            # python 3.7 of higher
            asyncio.run(order(), debug=True )
    else:
        print(f"python version {pv} not able to run asyncio") 
    
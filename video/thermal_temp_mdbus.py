#!/usr/bin/env python
#
# realsense depth from camera SDK to modbus tcp slave (server) values 
# added if you pass arguments of ipaddr tag slot type etc.. to script it will also write to s AB Controllogix PLC
#
# usage <prog> optional: 0-4 (endianess) AB_IP_ADDR AB_TAG CPU_SLOT 0-1 (1==micro 800 series)
#
# https://www.sinoseen.com/thermal-imaging-mini-camera-module-tiny1-c-micro-low-power-consumption
# https://github.com/matbeedotcom/Tiny-1C-Python-Bindings/tree/main
#
import sys
import tiny_thermal_camera
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
from ipycam import IPCamera, CameraConfig, PTZHardwareHandler
import time

# class to choose endian mode
class endian_mode:
    little = 0
    big = 1
    network = 2
    native = 3
    native_ali = 4
    no_of_modes = 5   
e = endian_mode()  

ENDIANESS = e.little                                                              # default little endian
if len(sys.argv) > 1:                                                             # if you pass arguments to script then also write to allen bradley
    if len(sys.argv) > 1:                                                         # arg 1
        ENDIANESS=int(sys.argv[1]) % e.no_of_modes

with tiny_thermal_camera.ThermalCamera() as camera:            # Initialize camera with context manager
    camera.start_streaming()
do_it = 1
NO_IR_INTS = 4                                                 # 1 double == 4 16 bit integers in modbus
NO_REGS=5

MY_TCP_SLAVE="localhost"                                       # modbus tcp ip and port
TCP_SLAVE_PORT=502

# check the version of python we have for compatibility with asyncio
def chk_python_version():
    my_py = platform.python_version()
    p = my_py.split(".")
    v = 0
    lp = len(p)
    m = 100
    for s in range(0, lp):
        v += (int(p[s])*m)
        m /= 10 
    return v

# modbus tcp data transfer
async def update_datablock(store: ModbusSlaveContext):
    global do_it
    config = CameraConfig.load("camera_config.json")
    icamera = IPCamera(config=config)

    # Disable digital PTZ - frames pass through unchanged
    icamera.ptz.enable_digital_ptz = False

    # Add your hardware controller
    icamera.ptz.add_hardware_handler(PrintingPTZHandler())

    if not icamera.start():
        print("Failed to start rtsp stream")
        camera.stop_stream()
        camera.close()
        print("Cleanup completed.")
        sys.exit(-1)

    print('start of modbus tcp slave (server) interface')
    frame_count = 0
    while do_it == 1:

        try:
            temp_frame, image_frame = camera.capture_frame()
            if image_frame is not None:
                # Stream handles PTZ, timestamp, and frame pacing automatically
                icamera.stream(image_frame)
            if temp_frame is not None:
                frame_count += 1
                # Get temperature statistics
                stats = camera.get_temperature_stats(temp_frame)
                hot_pos, hot_temp = camera.find_hotspot(temp_frame)
                # Print status every 10 frames
                if frame_count % 10 == 0:
                    print(f"Frame {frame_count}: "
                          f"Range: {stats['min']:.1f}-{stats['max']:.1f}°C, mean: {stats['mean']:.1f}°C"
                          f"Hotspot: {hot_temp:.1f}°C at {hot_pos}")
                    # <	Littleendian
                    # >	Big Endian
                    # !	Network
                    # =	Native
                    # @	Native (with alignment
                    for i, ech_data in enumerate([stats['min'], stats['max'], stats['mean'], hot_temp, hot_pos]): 
                        if ENDIANESS == e.little:
                            d_bytes = struct.pack('<d', ech_data)                         # make 64bit int from double precision number little end
                        elif ENDIANESS == e.big:
                            d_bytes = struct.pack('>d', ech_data)                         # make 64bit int from double precision number big end 
                        elif ENDIANESS == e.network:
                            d_bytes = struct.pack('!d', ech_data)                         # make 64bit int from double precision number big end 
                        elif ENDIANESS == e.native:
                            d_bytes = struct.pack('=d', ech_data)                         # make 64bit int from double precision number big end 
                        elif ENDIANESS == e.native_ali:
                            d_bytes = struct.pack('@d', ech_data)                         # make 64bit int from double precision number big end             
                        w_bytes = struct.unpack('HHHH', d_bytes)

                        for b in range(0, len(w_bytes)):                                  # foreach byte
                            store.setValues(4, b+(i*4), w_bytes[b])                       # write distance to object to 4 modbus 16bit registers (double)
                time.sleep(0.1)  # ~10 FPS
        except KeyboardInterrupt:
            print("\nStopping monitoring...")
            do_it = 0
        finally:
            # Clean up when done
            camera.stop_stream()
            camera.close()
            icamera.stop()
            print("Cleanup completed.")
    		
# run the modbus tcp server (slave)
async def run_tcp():
    global do_it
    try:
        # we are biulding a custom data block area
        builder = BinaryPayloadBuilder(byteorder=Endian.Big)           
        for i in range(0, (NO_IR_INTS*NO_REGS)):                                # make a database for modbus with 4 IR integers 16 bit                               
            builder.add_16bit_int(i)
        block = ModbusSequentialDataBlock(1, builder.to_registers())
        store = ModbusSlaveContext( ir=block )
        context = ModbusServerContext(slaves=store, single=True)
        identity = ModbusDeviceIdentification()
        identity.VendorName = 'AirCamPro'
        identity.ProductCode = 'TCM'
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
    print('start of thermal camera to modbus server.......')  
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

#
# example using a hokuyo UM-30LX Lidar to move forward using a robotis L42-10-S300-R by an ammount measured by the lidar
#
# uses https://github.com/HiroakiMatsuda/pyurg
import pyurg  
import math
from time import time

# the mode of mewasurement we have chosen
_MODE=4

import os, sys

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(sys.stdin.fileno())
    def getch():
        return sys.stdin.read(1)

os.sys.path.append('../dynamixel_functions_py')             # Path setting

import dynamixel_functions as dynamixel                     # Uses DYNAMIXEL SDK library

# Control table address for L42-10-S300-R https://emanual.robotis.com/docs/en/dxl/pro/l42-10-s300-r/
ADDR_PRO_OP_MODE       = 11                                 # operation mode
VEL_MODE = 1                                                # velocity
POS_MODE = 3                                                # position
ADDR_PRO_TORQUE_ENABLE       = 562                          # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_VELOCITY       = 600                          # paramter in the drive for controlling velocity

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                             # Dynamixel ID: 1
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB1".encode('utf-8')        # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000                       # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000                        # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.portHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.packetHandler()

dxl_comm_result = COMM_TX_FAIL                                                  # Communication result
dxl_goal_vel = [1024, 512]                                                      # Goal velocities 114 rpm, 54 rpm

# Open port
if dynamixel.openPort(port_num):
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    #getch()
    quit()

# Set port baudrate
if dynamixel.setBaudRate(port_num, BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    #getch()
    quit()

# Set mode to velocity 
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OP_MODE, VEL_MODE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
    
# Enable Dynamixel Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel has been successfully connected")


# make instance of the driver class for the lidar
urg = pyurg.Urg()

# select the port you connected to  
urg.set_urg('/dev/ttyUSB0')  

# choose the mode for the lidar and we are degaulting num =1 to one measurement you can have up to 99
if _MODE == 0:
    status = urg.request_ms(0, 1080)
elif _MODE == 1:
    status = urg.request_md(0, 1080)
elif _MODE == 2:
    status = urg.request_gd(0, 1080)
elif _MODE == 3:
    status = urg.request_gs(0, 1080)
elif _MODE == 4:
    status = urg.requese_me(0, 1080)  

# vehicle tire dimensions
dia=65
tire=math.pi*dia   # circumference of tire (mm)
vel_rpm=33.3
# stop time (wind down)
sto_s=0.1
# distance must be at least this value before we move
dist_spt=100

# test its all okay first
urg.check_status(status) 

# for our rnage of meaurements in the test
for i in range(2):  
    dist, intensity, timestamp = urg.get_distance_and_intensity()     # read the lidar
    print("timestamp %s len %s intenisty %s" % (timestamp, len(dist), len(intensity))) 
    for i range(0,len(dist)):
        print("distance mm = ",dist[i])
    if dist[0] > dist_spt :                                # we detected am object in front
        n_revs=dist[0]/tire                                # number of revolutions required to move forward   
        # set velocity to 114 RPM then 54 RPM
        # Write that goal velocity
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_VELOCITY, dxl_goal_vel[i])
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
        # calc velocity in RPS
        rps = dxl_goal_vel[i]*60.0
        # calc time drive needs to be on to do that distance
        ti_s = n_revs / rps 
        # take a time reference to start the clock
        st_s = time()
        while ((time() - st_s) < (ti_s-sto_s)) :                               # wait for the travel time (minus a stop time)
        # now write the goal velocity as zero
        dynamixel.write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_VELOCITY, 0)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
    print("Press any key to do next test...")  
    getch()
    
urg.close_port()

# Disable Dynamixel Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Close port
dynamixel.closePort(port_num)
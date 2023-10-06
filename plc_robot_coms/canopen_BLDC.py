# ===================
# ID 7 Beckhoff IO-Link IP1001 B510-0000   
# ID 6 Beckhoff IP5009-B510
# ID 2 TRINAMIC Su Te  apical NN Corning・holds ー TA PD42-1270
# ID 10  Ri Eito NN Tatari Hikaru holds ー TA ー U NN suites Ro ー RA BLVD-KRD and holds ー TA BLMR5100K-A-B
# ID 9 Maxon the epos2, the 42BLF01 DC BU RA Rayon Su・holds ー TA
# ID 5 Maxon EPOS4, 100W with encoder
# ===================
import canopen
import time

# Start with creating a network representing one CAN bus
network = canopen.Network()

node = canopen.RemoteNode(7,'IP10xx-B51x.eds')
network.add_node(node)

# Connect to the CAN bus
network.connect(bustype='ixxat', channel=0, bitrate=1000000)

# Reset network
node.nmt.state = 'RESET COMMUNICATION'
time.sleep(0.5)
node.nmt.state = 'RESET'
time.sleep(3)
node.nmt.send_command(0x1)  # NMT start

network.check()
print('node state 1) = {0}'.format(node.nmt.state))
print("\n start network")

print("\nstart  ID=7 beckhoff")
print("- - -")
device_name    = node.sdo[0x1008].raw
print('Device name ',device_name)
Hardware_version    = node.sdo[0x1009].raw
print('Hardware version  ',Hardware_version)
Software_version    = node.sdo[0x100a].raw
print('Software version ',Software_version)
print("- - -")

print("\nDisconnect") 

# Disconnect from CAN bus
network.disconnect()

node = canopen.RemoteNode(6,'IP50xx-B51x.eds')
network.add_node(node)

# Connect to the CAN bus
network.connect(bustype='ixxat', channel=0, bitrate=1000000)

# Reset network
node.nmt.state = 'RESET COMMUNICATION'
time.sleep(0.5)
node.nmt.state = 'RESET'
time.sleep(3)
node.nmt.send_command(0x1)  # NMT start

network.check()
print('node state 1) = {0}'.format(node.nmt.state))
print("\n start network")

print("\nstart  ID=6 beckhoff")
print("- - -")
device_name    = node.sdo[0x1008].raw
print('Device name ',device_name)
Hardware_version    = node.sdo[0x1009].raw
print('Hardware version  ',Hardware_version)
Software_version    = node.sdo[0x100a].raw
print('Software version ',Software_version)
print("- - -")

print("\nDisconnect") 

# Disconnect from CAN bus
network.disconnect()

# Add some nodes with corresponding Object Dictionaries
node = canopen.RemoteNode(10,'BLVD-KRD_CANopen_V100.eds')
network.add_node(node)

# Connect to the CAN bus
network.connect(bustype='ixxat', channel=0, bitrate=1000000)

print("\n===start  ID=10 OrientalMotor")
device_name    = node.sdo[0x1008].raw
print('Device name ',device_name)
Motor_temp    = node.sdo[0x407d].raw
print('Motor_temp  ',float(Motor_temp/10.0))
print("") 
Max_torqe    = node.sdo[0x6072].raw
print('Max torqe',Max_torqe)
Target_position    = node.sdo[0x607a].raw
print('Target position',Target_position)
Software_position_limit_min    = node.sdo[0x607d][1].raw
print('Software position limit min',Software_position_limit_min)
Software_position_limit_max    = node.sdo[0x607d][2].raw
print('Software position limit max',Software_position_limit_max)
Positioning_option_code    = node.sdo[0x60f2].raw
print('Positioning option code',Positioning_option_code)
Profile_velocity    = node.sdo[0x6081].raw
print('Prfile velocity',Profile_velocity)
End_velocity     = node.sdo[0x6082].raw
print('End velocity ',End_velocity )
Profile_acceleration    = node.sdo[0x6083].raw
print('Prfile acceleration',Profile_acceleration)
Profile_deceleration     = node.sdo[0x6084].raw
print('Profile deceleration ',Profile_deceleration )
Quick_stop_deceleration    = node.sdo[0x6085].raw
print('Quick stop deceleration',Quick_stop_deceleration)
Quick_stop_option_code    = node.sdo[0x605a].raw
print('Quick stop option code',Quick_stop_option_code)
Halt_option_code    = node.sdo[0x605d].raw
print('Halt option code',Halt_option_code)
Position_window    = node.sdo[0x6067].raw
print('Position window',Position_window)
Following_error_window    = node.sdo[0x6065].raw
print('Following error window',Following_error_window)

# Disconnect from CAN bus
network.disconnect()

node = canopen.RemoteNode(2,'TMCM-1270.eds')
network.add_node(node)

# Connect to the CAN bus
network.connect(bustype='ixxat', channel=0, bitrate=1000000)

print("\n===start  ID=2 Trinamic")
#print(network[node_id])
vendor_id = node.sdo[0x1018][1].raw
Product_code = node.sdo[0x1018][2].raw
Revision_number = node.sdo[0x1018][3].raw

print('vendor_id ',vendor_id)
print('Product_code ',Product_code)
print('Revision_number ',Revision_number)
print("") 

# Disconnect from CAN bus
network.disconnect()

node = canopen.RemoteNode(9,'maxon_motor_EPOS2_2123h_6220h_0000h_0000h.eds')
network.add_node(node)

# Connect to the CAN bus
network.connect(bustype='ixxat', channel=0, bitrate=1000000)

print("\n===start  ID=9 maxon EPOS2")
#print(network[node_id])
vendor_id = node.sdo[0x1018][1].raw
Product_code = node.sdo[0x1018][2].raw
Revision_number = node.sdo[0x1018][3].raw

print('vendor_id ',vendor_id)
print('Product_code ',Product_code)
print('Revision_number ',Revision_number)
print("") 

Motor_Type = node.sdo[0x6402].raw
print('Motor Type 10:Sinusoidal PM BL, 11:Trapezoidal PM BL motor ',Motor_Type)
Continuous_Current_Limit = node.sdo[0x6410][1].raw 
print('Continuous Current Limit ',Continuous_Current_Limit)
Pole_Pair_Number = node.sdo[0x6410][3].raw 
print('Pole Pair Number ',Pole_Pair_Number)
Thermal_Time_Constant_Winding  = node.sdo[0x6410][5].raw 
print('Thermal Time Constant Winding  ',Thermal_Time_Constant_Winding)
Encoder_Pulse_Number   = node.sdo[0x2210][1].raw 
print('Encoder Pulse Number   ',Encoder_Pulse_Number)
Position_Sensor_Type   = node.sdo[0x2210][2].raw 
print('Position Sensor Type  ',Position_Sensor_Type)

Current_Regulator_PGain = node.sdo[0x60f6][1].raw 
print('Current Regulator P-Gain ',Current_Regulator_PGain)
Current_Regulator_IGain = node.sdo[0x60f6][2].raw 
print('Current Regulator I-Gain ',Current_Regulator_IGain)

acceleration     = node.sdo[0x6083].raw
print('acceleration ',acceleration)
deceleration     = node.sdo[0x6084].raw
print('deceleration ',deceleration)
node.sdo[0x6081].raw = 0x03e8  # Profile velocity 1000
velocity     = node.sdo[0x6081].raw
print('velocity     ',velocity)
print("")  
# Disconnect from CAN bus
network.disconnect()

node = canopen.RemoteNode(5,'maxon_motor_EPOS4_0170h_6050h_0000h_0000h.eds')
network.add_node(node)

# Connect to the CAN bus
network.connect(bustype='ixxat', channel=0, bitrate=1000000)

print("\n===start  ID=5 maxon EPOS4")
#print(network[node_id])
vendor_id = node.sdo[0x1018][1].raw
Product_code = node.sdo[0x1018][2].raw
Revision_number = node.sdo[0x1018][3].raw

print('vendor_id ',vendor_id)
print('Product_code ',Product_code)
print('Revision_number ',Revision_number)
print("") 

Motor_Type = node.sdo[0x6402].raw
print('Motor Type 10:Sinusoidal PM BL, 11:Trapezoidal PM BL motor ',Motor_Type)

acceleration     = node.sdo[0x6083].raw
print('acceleration ',acceleration)
deceleration     = node.sdo[0x6084].raw
print('deceleration ',deceleration)
print("")  

# Disconnect from CAN bus
network.disconnect()
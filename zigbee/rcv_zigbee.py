'''
recieves data over zigbee and prints it
'''
import xbee
import time
import binascii

# The following commands affect the UART serial interface
# ［BD(UART baud rate) <XBee3><S2C><S2
# This command sets the serial interface baud rate for communication between the device's UART port and the host.
# The device interprets any value between 0x12C and 0x0EC400 as the custom baud rate.
# The custom baud rate is not guaranteed and the device will attempt to find the closest baud rate achievable.
# After setting a non-standard baud rate, query the BD to find the actual operating baud rate before applying the change.
# The following table shows some examples of sent BD parameters versus stored parameters.
# Parameter Description
# 0x0 1200 baud
# 0x1 2400 baud
# 0x2 4800 baud
# 0x3 9600 baud
# 0x4 19200 baud
# 0x5 38400 baud
# 0x6 57600 baud
# 0x7 115200 baud
# 0x8 230,400 baud
# 0x9 460,800 baud
# 0xA 921,600 baud
# Parameter range
# Standard baud rate : 0x0-0x07 <S2
# Standard baud rate : 0x0-0x0A <XBee3><S2C>
# Non-standard baud rate : 0x80 -0x0E1000 <S2>
# Non-standard baud rate : 0x12C-0x0EC400 <XBee3> <S2C>
# Default
# 0x03 (9600 baud)
#
# ［NB (Parity)] <XBee3><S2C><S2
# Sets or reads the serial parity setting for UART communication.
# Parameter Description
#　0 No parity
#　1 Even parity
#　2 Odd parity
#3 Mark parity (always "1") <S2C><S2
#
# ［SB (stop bit)] <XBee3><S2C><S2
# Sets or displays the number of stop bits for UART communication.
# Parameter Description
#　0 One stop bit
#　1 Two stop bits
# Default
#　0
#
def set_comms_params(baud=0xA, par=1, sb=0):
    network_settings = {"BD": baud, "NB": par, "SB": sb}
    for command, value in network_settings.items():
        xbee.atcmd(command, value)
    xbee.atcmd("AC")                                 # write changes
    time.sleep(1) 
    
# ［D6 (AD6 / DIO6 / TH_SPI_ATTN configuration) <XBee3><S2C><S2
# Sets or displays the DIO1 / AD1 configuration (Micro pin 27 / SMT pin 29 / TH pin 16).
# Parameter Description <XBee3><S2C>
#　0 Disabled
#　1 RTS FLow Control
#　2 ADC
# 3 Digital input
# 4 Output LOW on digital output
# 5 HIGH on digital output
# 6 I2C SCL (available in MicroPython only) <Xbee3
#
# controls light LED on D6 from https://www.switch-science.com/products/1052/
#
REMOTE_LED_ACTIVE=1      # set to 1 if its active
def set_led_low():
    xbee.atcmd('D6', 4)  # low

def set_led_high():
    xbee.atcmd('D6', 5)  # set high

chosen_baud=0x7
set_comms_params(chosen_baud)                                                    # baud to 115200
    
while True:
    status = xbee.atcmd('AI')                                                    # Check network participation status
    print('.',end='')
    if status == 0x00:                                                           # Exit the loop when in participating state
        break
    xbee.atcmd('CB',0x01)                                                        # Commissioning (network participation)
    time.sleep_ms(2000)                                                          # 2 seconds wait time processing
print('\nJoined')

while True:
    set_led_high()                                                               # LED is on
    packet = xbee.receive()                                                      # Receive the packet
    set_led_low()                                                                # LED is off
    if packet:                                                                   # When there is received data
        addr = str(binascii.hexlify(packet['sender_eui64']).decode('utf-8'))
        addr = addr[:8] + ' ' + addr[8:]                                         # Separate the source address for display (8+8 characters)
        payload = str(packet['payload'].decode('utf-8'))                         # Extract the received data
        print(addr + ', ' + payload)                                             # Display address and received data
        print("timestamp was "+payload['ts'])
        

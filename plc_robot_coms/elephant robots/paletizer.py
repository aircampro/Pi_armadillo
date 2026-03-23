#
# example code for elephant robot ultraArmP340 for palletizing connected on serial
#
from pymycobot.ultraArmP340 import ultraArmP340
# from pymycobot import MyPalletizer260 - if use check number of joints
import serial
#import serial.tools.list_ports
import argparse
import platform

# Coordinates of the moved wooden block
# P340 has 3 joints tables below are for it.
#
green_pos = [[74.6, 167.55, 120], [74.6, 167.55, 92.45], [74.6, 167.55, 52.45], [74.6, 167.55, 12.45]]
red_pos = [[112.67, 173.5, 120], [112.67, 173.5, 92.45], [112.67, 173.5, 52.45], [112.67, 173.5, 12.45]]
yellow_pos = [[150.92, 167.61, 120], [150.92, 167.61, 92.45], [150.92, 167.61, 52.45], [150.92, 167.61, 12.45]]

# Coordinates of the wooden block's arriva
cube_pos_g = [[200, -75, 120], [200, -75, 15], [200, -75, 55], [200, -75, 95]]
cube_pos_r = [[150.63, -75, 15], [150.63, -75, 55], [150.63, -75, 95]]
cube_pos_y = [[109.63, -75, 15], [109.63, -75, 55], [109.63, -75, 95]]

# The direction of the wooden block being moved
block_high = [60, 5, 0]

# Direction of the wooden block's arrival position
cube_high = [-30, 10, 10]

parser = argparse.ArgumentParser()
parser.add_argument(
    "--lserial_port",
    type=str,
    default="/dev/ttyUSB0",
    choices=["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyS0"],
    help=(
        "Enter the serial port name e.g. /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyS0."
    ),
)
parser.add_argument(
    "--wserial_port",
    type=str,
    default="COM0",
    choices=["COM0", "COM1", "COM2"],
    help=(
        "Enter the serial port name e.g. COM0 COM1 COM2."
    ),
)
parser.add_argument(
    "--filename",
    type=str,
    help=(
        "filename for gcode file"
    ),
)
args = parser.parse_args()

if platform.system() == "Windows":
    ua = ultraArmP340(args.wserial_port, 115200)
    ua.go_zero()
elif platform.system() == "Linux":
    ua = ultraArmP340(args.lserial_port, 115200)
    ua.go_zero()

# set fan state open
ua.set_fan_state(1)

# The ultraArmP340 must be homed before performing coordinate/angle movements; otherwise, the correct angle/coordinates cannot be obtained.
ua.go_zero()
ua.sleep(0.5)

# Yellow
# Move the first wooden block
# Move to the direction where the wooden block is located
ua.set_angles(block_high, 50)
while ua.is_moving_end() == 0:
ua.sleep(0.1)
ua.set_coords(yellow_pos[0], 50)
while ua.is_moving_end() == 0:
ua.sleep(0.1)

# Reach the location of the wooden block
ua.set_coords(yellow_pos[1], 50)
while ua.is_moving_end() == 0:
ua.sleep(0.1)
# Turn on the suction pump
ua.set_gpio_state(0)
ua.sleep(0.5)

# Move above the desired location
ua.set_angles(cube_high, 50)
ua.sleep(0.5)

# Descend to the desired position
ua.set_coords(cube_pos_y[0], 50)
while ua.is_moving_end() == 0:
ua.sleep(0.5)
# Turn off the suction pump
ua.set_gpio_state(1)
ua.sleep(0.5)

# Move above the current location of the wooden block. 
# This completes one transport action. Subsequent transport actions will follow the same pattern as this one.
ua.set_angles(cube_high, 50)
while ua.is_moving_end() == 0:
ua.sleep(0.1)

#2
ua.set_angles(block_high, 50)
while ua.is_moving_end() == 0:
ua.sleep(0.1)

ua.set_coords(yellow_pos[0], 50)
while ua.is_moving_end() == 0:
ua.sleep(0.1)

ua.set_coords(yellow_pos[2], 50)
while ua.is_moving_end() == 0:
ua.sleep(0.1)
ua.set_gpio_state(0)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

ua.set_coords(cube_pos_y[1], 50)
ua.sleep(0.5)
ua.set_gpio_state(1)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

# Red
# 1
ua.set_angles(block_high, 50)
ua.sleep(0.5)

ua.set_coords(red_pos[0], 50)
ua.sleep(1)

ua.set_coords(red_pos[1], 50)
ua.sleep(1)
ua.set_gpio_state(0)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

ua.set_coords(cube_pos_r[0], 50)
ua.sleep(0.5)
ua.set_gpio_state(1)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

#2
ua.set_angles(block_high, 50)
ua.sleep(0.5)

ua.set_coords(red_pos[0], 50)
ua.sleep(1)

ua.set_coords(red_pos[2], 50)
ua.sleep(1)
ua.set_gpio_state(0)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

ua.set_coords(cube_pos_r[1], 50)
ua.sleep(0.5)
ua.set_gpio_state(1)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

# Green
# 1
ua.set_angles(block_high, 50)
ua.sleep(0.5)

ua.set_coords(green_pos[0], 50)
ua.sleep(1)

ua.set_coords(green_pos[1], 50)
ua.sleep(1)
ua.set_gpio_state(0)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

ua.set_coords(cube_pos_g[0], 50)
ua.sleep(0.5)

ua.set_coords(cube_pos_g[1], 50)
ua.sleep(0.5)
ua.set_gpio_state(1)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

#2
ua.set_angles(block_high, 50)
ua.sleep(0.5)

ua.set_coords(green_pos[0], 50)
ua.sleep(1)

ua.set_coords(green_pos[2], 50)
ua.sleep(1)
ua.set_gpio_state(0)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

ua.set_coords(cube_pos_g[0], 50)
ua.sleep(0.5)

ua.set_coords(cube_pos_g[2], 50)
ua.sleep(0.5)
ua.set_gpio_state(1)
ua.sleep(0.5)

ua.set_angles(cube_high, 50)
ua.sleep(0.5)

# pwm laser
ua.set_pwm(128)
us.sleep(10)
ua.set_pwm(255)
us.sleep(10)
ua.set_pwm(0)

# play gcode file 
ua.play_gcode_file(args.filename)

# set fan state close
ua.set_fan_state(0)

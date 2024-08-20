#!/usr/bin/python
# ref:- https://qiita.com/kazuharayama/items/4fda1ad55b8efa17e2e6
# example reading and controlling oriental motor using ros1 interface
# -*- coding: utf-8 -*-

# for using usb port on pi
# su
# modprobe ftdi_sio vendor=0x06ce product=0x8331
# echo 06ce 8331 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
# exit
#
# $ source devel/setup.bash # Necessary if immediately after build
# $ roslaunch om_modbus_master om_modbusRTU.launch com:="/dev/ttyUSB0" topicID:=1 baudrate:=115200 updateRate:=10 firstGen:="1,2," secondGen:="3,4,"

import rospy
import time
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

# global readbacks
gState_driver = 0
gMotor_spd = 0

# gets speed from drive via callback
def resCallback(res):
  global gMotor_spd
  if(res.slave_id == 1):
    gMotor_spd = res.data[0]

# gets state of drive via callback
def stateCallback(res):
  global gState_driver
  gState_driver = res.state_driver

# Wait for communication to finish
def wait():
  global gState_driver
  time.sleep(0.01)
  while(gState_driver == 1):
    pass

def main():  
  rospy.init_node("sample1", anonymous=True)
  pub = rospy.Publisher("om_query1", om_query, queue_size=1)
  rospy.Subscriber("om_response1", om_response, resCallback)
  rospy.Subscriber("om_state1", om_state, stateCallback)
  msg = om_query()
  time.sleep(1)

  # Writing (rotation speed No.2)
  msg.slave_id = 1
  msg.func_code = 1
  msg.write_addr = 88
  msg.write_num = 8
  msg.data[0] = 0                    # Operation data
  msg.data[1] = 1                    # Method Absolute position Method number
  msg.data[2] = 2000                 # MIN～MAX The value set by the soft limit
  msg.data[3] = 2000                 # drive speed in Hz
  msg.data[4] = 1500                 # Start change rate kHz/s
  msg.data[5] = 1500                 # Specified rate kHz/s
  msg.data[6] = 100                  # current
  msg.data[7] = 1                    # Reflection trigger 1: All data reflected 0
  msg.data[8] = 7176                 # Error (fixed)
  pub.publish(msg)
  wait()

  rospy.spin()

if __name__ == '__main__':
    main()
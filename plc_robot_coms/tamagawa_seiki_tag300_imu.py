#!/usr/bin/env python 
#
# tamagawa seiki serial imu driver
# TAMAGAWA SEIKI TAG300 BI: 10DPH MEMS-IMU
#
import serial
import numpy as np

# class to store the information from the imu
class IMUData:
    """                                                                                                                                        
    data read from the imu                                                                                                                  
    """
    __slots__ = ('counter', '_status', '_angular_velocity_x', '_angular_velocity_y', '_angular_velocity_z', '_linear_acceleration_x', '_linear_acceleration_y', '_linear_acceleration_z')

    def __init__(self, c=0, s=0, avx=0, avy=0, avz=0, lax=0, lay=0, laz=0):
        self.counter = c
        self._status = s
        self._angular_velocity_x = avx
        self._angular_velocity_y = avy
        self._angular_velocity_z = avz
        self._linear_acceleration_x = lax
        self._linear_acceleration_y = lay
        self._linear_acceleration_z = laz
        self._data_ok = 0

# starts data transmission from the imu
def start_imu_data(rate=50, spt='/dev/ttyUSB0', baud_rt=115200):
    io_value = -1
    io_str = " "
    try:
        msg=f"$TSC,BIN,={rate}"
        end_msg = [ 0x0D, 0x0A ]
        init_snd=msg.encode('utf-8') + bytearray(end_msg)  
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
        ser.write(bytearray(init_snd))
        line = ser.readline()
        io_str = line.decode('utf-8')
        io_value = 1
    except Exception as e:
        print(f'failed to send initial message to imu : {e}')
    finally:    
        ser.close()
    return io_value, io_str

# opens serial port
def open_serial_port(spt='/dev/ttyUSB0', baud_rt=115200, use_fog=False):
    ser = None
    try:  
        ser = serial.Serial(spt, baud_rt, timeout=0.1)
    except Exception as e:
        print(f'failed to open serial port to imu : {e}')
    return ser

# closes serial port
def close_serial_port(ser):
    try: 
        ser.close()
    except Exception as e:
        print(f'failed to close imu serial port : {e}')

# reads imu data from serial port and populates IMUData class with the data
def read_imu_data(ser, use_fog=False):
    imu = IMUData()
    try:  
        imu._data_ok = 0
        ser.flushOutput()
        ser.flushInput()
        line = ser.readline()
        rbuf = line.decode('utf-8')
        if (str(rbuf[5]) == 'B' and str(rbuf[6]) == 'I' and str(rbuf[7]) == 'N' and str(rbuf[8]) == ','):     # should read BIN,
            if use_fog == True:
                imu.counter = ((int(rbuf[11]) << 24) & 0xFF000000) | ((int(rbuf[12]) << 16) & 0x00FF0000)
                imu._status = ((int(rbuf[13]) << 8) & 0xFFFFFF00) | (int(rbuf[14]) & 0x000000FF)
                raw_data = ((((int(rbuf[15]) << 8) & 0xFFFFFF00) | (int(rbuf[16]) & 0x000000FF)))
                imu._angular_velocity_x = raw_data * (200 / pow(2, 15)) * np.pi / 180.0                      # LSB & unit [deg/s] => [rad/s]
                raw_data = ((((int(rbuf[17]) << 8) & 0xFFFFFF00) | (int(rbuf[18]) & 0x000000FF)))
                imu._angular_velocity_y = raw_data * (200 / pow(2, 15)) * np.pi / 180.0                      # LSB & unit [deg/s] => [rad/s]
                raw_data_2 = ((int(rbuf[19]) << 24) & 0xFF000000) | ((int(rbuf[20]) << 16) & 0x00FF0000) | ((int(rbuf[21]) << 8) & 0x0000FF00) | (int(rbuf[22]) & 0x000000FF)
                imu._angular_velocity_z = raw_data_2 * (200 / pow(2, 31)) * np.pi / 180.0                    # LSB & unit [deg/s] => [rad/s]
                raw_data = ((((int(rbuf[23]) << 8) & 0xFFFFFF00) | (int(rbuf[24]) & 0x000000FF)))
                imu._linear_acceleration_x = raw_data * (100 / pow(2, 15))                                    # LSB & unit [m/s^2]
                raw_data = ((((int(rbuf[25]) << 8) & 0xFFFFFF00) | (int(rbuf[26]) & 0x000000FF)))
                imu._linear_acceleration_y = raw_data * (100 / pow(2, 15))                                    # LSB & unit [m/s^2]
                raw_data = ((((int(rbuf[27]) << 8) & 0xFFFFFF00) | (int(rbuf[28]) & 0x000000FF)))
                imu._linear_acceleration_z = raw_data * (100 / pow(2, 15))                                    # LSB & unit [m/s^2]
                imu._data_ok = 1
            else:
                imu.counter = ((int(rbuf[11]) << 8) & 0x0000FF00) | (int(rbuf[12]) & 0x000000FF)
                imu._status = ((int(rbuf[13]) << 8) & 0xFFFFFF00) | (int(rbuf[14]) & 0x000000FF)
                raw_data = ((((int(rbuf[15]) << 8) & 0xFFFFFF00) | (int(rbuf[16]) & 0x000000FF)))
                imu._angular_velocity_x = raw_data * (200 / pow(2, 15)) * np.pi / 180                          # LSB & unit [deg/s] => [rad/s]
                raw_data = ((((int(rbuf[17]) << 8) & 0xFFFFFF00) | (int(rbuf[18]) & 0x000000FF)))
                imu._angular_velocity_y = raw_data * (200 / pow(2, 15)) * np.pi / 180                          # LSB & unit [deg/s] => [rad/s]
                raw_data = ((((int(rbuf[19]) << 8) & 0xFFFFFF00) | (int(rbuf[20]) & 0x000000FF)))
                imu._angular_velocity_z = raw_data * (200 / pow(2, 15)) * np.pi / 180                          # LSB & unit [deg/s] => [rad/s]
                raw_data = ((((int(rbuf[21]) << 8) & 0xFFFFFF00) | (int(rbuf[22]) & 0x000000FF)))
                imu._linear_acceleration_x  = raw_data * (100 / pow(2, 15))                                    # LSB & unit [m/s^2]
                raw_data = ((((int(rbuf[23]) << 8) & 0xFFFFFF00) | (int(rbuf[24]) & 0x000000FF)))
                imu._linear_acceleration_y = raw_data * (100 / pow(2, 15))                                     # LSB & unit [m/s^2]
                raw_data = ((((int(rbuf[25]) << 8) & 0xFFFFFF00) | (int(rbuf[26]) & 0x000000FF)))
                imu._linear_acceleration_z = raw_data * (100 / pow(2, 15))                                     # LSB & unit [m/s^2]
                imu._data_ok = 1
        else:
            print('message read is in an incorrect format')
            imu._data_ok = -1
    except Exception as e:
        print(f'failed to read message from imu : {e}')
        imu._data_ok = -1
    return imu

# example to send imu data over ROS
#
import roslib
import rospy
import geometry_msgs.msg
import turtlesim.srv

def main_ros():
    rospy.init_node('imu')
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    rate = rospy.Rate(10.0)
    if (start_imu_data()[0] == 1):
        spo = open_serial_port()
        imu_data = IMUData() 
        cmd = geometry_msgs.msg.Twist()
        while not rospy.is_shutdown() and not spo == None:
            imu_data = read_imu_data(spo)
            if imu_data._data_ok == 1:
                cmd.linear.x = imu_data._linear_acceleration_x
                cmd.linear.y = imu_data._linear_acceleration_y
                cmd.linear.z = imu_data._linear_acceleration_z
                cmd.angular.x = imu_data._angular_velocity_x
                cmd.angular.y = imu_data._angular_velocity_y
                cmd.angular.z = imu_data._angular_velocity_z
                turtle_vel.publish(cmd)
            rate.sleep()
        close_serial_port(spo)  

ROS_RUN = 1
if __name__ == '__main__':

    if ROS_RUN == 1:
        main_ros()  
#!/usr/bin/python
######################################################################
# joystick controller map
#
# buttons[0]: □Square
# buttons[1]: ×Cross
# buttons[2]: ○Circle
# buttons[3]: △Triangle
# buttons[4]: L1
# buttons[５]: R1
# buttons[6]: L2 digital (draw 1/3 to 1)
# buttons[7]: R2 digital (minus 1/3 to 1)
# buttons[8]: Share
# buttons[9]: Options
# buttons[10]: L3
# buttons[11]: R3
# buttons[12]: PS button
# buttons[13]: Touchpad button
#
# axes[0]: L stick horizontal (left=+1, right=-1)
# axes[1]: L stick vertical (up=+1, down=-1)
# axes[2]: R stick horizontal (left=+1, right=-1)
# axes[3]: L2 (neutral=+1, full accel=-1)
# axes[4]: R2 (neutral=+1, full accel=-1)
# axes[5]: R stick vertical (up=+1, down=-1)
# axes[6]: Accelerometer Left (controller left direction is positive)
# axes[7]: Accelerometer Front (the front direction of the controller is the positive direction)
# axes[8]: Accelerometer Up (Controller up direction is positive)
# axes[9]: Axis button(十字キー) LR（L=＋１, R=−１）
# axes[10]: Axis button(十字キー) Up/Down（Up=＋１, Down=−１）
# axes[11]: Gyrometer Roll (clockwise from the front: +, counterclockwise: ー)
# axes[12]: Gyrometer Yaw (clockwise from top: ー, clockwise: +)
# axes[13]: Gyrometer Pitch (Raise the light bar side: ー, lower: +)
#
######################################################################
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import std_srvs.srv
import sensor_msgs.msg
import phoxi_camera.srv as phoxi_camera_srv
import os
import time

CAM_ID="the camera ID for your camera"
# DD for co-ordinate space
DD={"NoValue" : 0, "CameraSpace" : 1, "MarkerSpace" : 3, "RobotSpace" : 4, "CustomSpace" : 5}

# check the hw identification
#
def getHardwareIdentification():
    srv_getHardware = rospy.ServiceProxy(service.get_hardware_indentification, phoxi_camera_srv.GetHardwareIdentification)
    res = srv_getHardware()
    if res.success == True:
        return res.hardware_identification

def saveFrame():
    global published_topics_num

    def callback(data):
        global published_topics_num
        published_topics_num = published_topics_num + 1

    # create subscribers
    rospy.Subscriber(topic.point_cloud, sensor_msgs.msg.PointCloud2, callback)
    rospy.Subscriber(topic.normal_map, sensor_msgs.msg.Image, callback)
    rospy.Subscriber(topic.confidence_map, sensor_msgs.msg.Image, callback)
    rospy.Subscriber(topic.texture, sensor_msgs.msg.Image, callback)

    srv_getFrame = rospy.ServiceProxy(service.get_frame, phoxi_camera_srv.GetFrame)
    res = srv_getFrame(-1)
    time.sleep(5)
    if True == res.success :
        print("got frame ok")
        path = os.getcwd()                                     # it should be ~/.ros
        filename = "/file.ply"
        os.system("rm -f " + path + filename)                  # remove old file
        dir = os.listdir(path)
        srv_saveFrame = rospy.ServiceProxy(service.save_frame, phoxi_camera_srv.SaveFrame)
        res = srv_saveFrame(-1, path + filename)
        dir = set(os.listdir(path)) - set(dir)
        if True == res.success :
            print("frame saved ok")

        
# control functions related to the joystick
#
def control(msg, twist_pub, srv_connect, srv_dicsonnect, srv_connected, srv_startAcq, srv_stopAcq, srv_acquiring, srv_trig, srv_setSpace):   
    # left sticks control robot twist                                    
    L_horizontal = msg.axes[0]                                     # L stick horizontal (left=+1, right=-1)
    L_vertical = msg.axes[1]                                       # L stick vertical (up=+1, down=-1)）
    circle = msg.buttons[13]                                       # Touchpad button
    # when Touchpad pressed then set the velocity
    velocity = [L_horizontal*(1+circle), L_vertical*(1+circle)]

    t = Twist()                                                     # create Twist object
    t.angular.z, t.linear.x = velocity                              # set equal ro the calcualtion from the joystick
    twist_pub.publish(t)                                            # publish the twist to ROS

    # TODO :: add a gimbal for the right stick 
    R_horizontal = msg.axes[2]                                      # R stick horizontal (left=+1, right=-1)
    R_vertical = msg.axes[5]                                        # R stick vertical (up=+1, down=-1)

    # get the current state from the camera
    responce = srv_connected()  
    res = srv_acquiring() 

    # control the camera off the joystick buttons       
    if msg.buttons[0] == 1 and responce.connected == False:         # press square to connect to pHoXi
        response = srv_connect(CAM_ID)
        if response.success == True:
           print("pHoXi connection success")
    elif msg.buttons[1] == 1 and responce.connected == True:        # press cross to disconnect
        srv_dicsonnect()   
        responce = srv_connected()    
        if responce.connected == False:
            print("pHoXi dis-connection success")  
    if msg.buttons[2] == 1 and res.value == False:                  # press circle to start acq to pHoXi
        srv_startAcq()
        res = srv_acquiring()  
        if res.value == True:
            print("pHoXi aquiring")
    elif msg.buttons[3] == 1 and res.value == True:                 # press triangle to stop acq to pHoXi
        srv_stopAcq()   
        res = srv_acquiring()    
        if res.value == False:
            print("pHoXi aquisition has stopped") 
    elif msg.buttons[4] == 1 and responce.connected == True:       # press L1 and trigger capture
        srv_trig()   
        res = srv_acquiring()    
        if res.value == False:
            print("pHoXi aquisition has stopped") 
    elif msg.buttons[5] == 1 and responce.connected == True:       # press R1 and get and save frame
        saveFrame()
    elif msg.buttons[6] == 1 and responce.connected == True:       # press L2 Co-ord space NoValue
        res = srv_setSpace(DD["NoValue"])
        if True == res.success :
            print("set marker space to NoValue")
    elif msg.buttons[7] == 1 and responce.connected == True:       # press R2 Co-ord space CameraSpace
        res = srv_setSpace(DD["CameraSpace"])
        if True == res.success :
            print("set marker space to CameraSpace")
    elif msg.buttons[10] == 1 and responce.connected == True:      # press L3 Co-ord space MarkerSpace
        res = srv_setSpace(DD["MarkerSpace"])
        if True == res.success :
            print("set marker space to MarkerSpace")
    elif msg.buttons[11] == 1 and responce.connected == True:      # press R3 Co-ord space RobotSpace
        res = srv_setSpace(DD["RobotSpace"])
        if True == res.success :
            print("set marker space to RobotSpace")
    elif msg.buttons[12] == 1 and responce.connected == True:      # press PS button Co-ord space CustomSpace
        res = srv_setSpace(DD["CustomSpace"])
        if True == res.success :
            print("set marker space to CustomSpace")
            
def main():
    global CAM_ID
    # robot controls from joystick via ROS
    rospy.init_node('joy_to_twist')
    twist_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
    
    # phoXi camera controls via ROS
    srv_connect = rospy.ServiceProxy(service.connect_camera, phoxi_camera_srv.ConnectCamera)
    srv_dicsonnect = rospy.ServiceProxy(service.disconnect_camera, std_srvs.srv.Empty)
    srv_connected = rospy.ServiceProxy(service.is_connected, phoxi_camera_srv.IsConnected)
    srv_startAcq = rospy.ServiceProxy(service.start_acquisition, std_srvs.srv.Empty)
    srv_stopAcq = rospy.ServiceProxy(service.stop_acquisition, std_srvs.srv.Empty)
    srv_acquiring = rospy.ServiceProxy(service.V2_is_acquiring, phoxi_camera_srv.GetBool)
    srv_trig = rospy.ServiceProxy(service.trigger_image, phoxi_camera_srv.TriggerImage)
    srv_setSpace = rospy.ServiceProxy(service.V2_set_coordinate_space, phoxi_camera_srv.SetCoordinatesSpace)
    
    response = srv_connect(CAM_ID)
    if response.success == True:
        print("pHoXi connection success checking hardware ID")
        hw_id = getHardwareIdentification()
        if (hw_id == CAM_ID) :
            print(" hardware id matches expected")
        else:
            print("mismatch got", hw_id, " needed ", CAM_ID, "\n do you want to update that Y/N?")
            y_n = input()  
            if y_n == "Y" or y_n == "y": 
                CAM_ID = hw_id
        srv_dicsonnect()  
        
    rospy.Subscriber('/joy', Joy, control, twist_pub, srv_connect, srv_dicsonnect, srv_connected, srv_startAcq, srv_stopAcq, srv_acquiring, srv_trig, srv_setSpace)

    rospy.spin()
    
if __name__ == '__main__':
    main()
    

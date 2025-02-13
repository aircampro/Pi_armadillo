#!/usr/bin/env python
#-*- coding:utf-8 -*-
#
# Example of use of IDS Peak SDK for camera ref:- https://www.youtube.com/watch?v=8ZZID-HfTe8
# In this example we rotate the image and publish on ROS a edge detected image which is rotated only if the param flag is set
#
import sys
import cv2
import rospy
import rosparam
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ids_peak import ids_peak
from ids_peak import ids_peak_ipl_extension

def set_exposure(device, exp_value):
    try:
        # Get the NodeMap of the RemoteDevice
        node_map_remote_device = device.RemoteDevice().NodeMaps()[0]
 
        min_exposure_time = 0;
        max_exposure_time = 0;
        inc_exposure_time = 0;
 
        # Get exposure range. All values in microseconds
        min_exposure_time = node_map_remote_device.FindNode("ExposureTime").Minimum()
        max_exposure_time = node_map_remote_device.FindNode("ExposureTime").Maximum()
 
        if node_map_remote_device.FindNode("ExposureTime").HasConstantIncrement():
            inc_exposure_time = node_map_remote_device.FindNode("ExposureTime").Increment()
        else:
            # If there is no increment, it might be useful to choose a suitable increment for a GUI control element (e.g.a slider)
            inc_exposure_time = 1000;
 
        # Get the current exposure time
        exposure_time = node_map_remote_device.FindNode("ExposureTime").Value()

        # clamp between max and min_exposure_time
		set_exp_value = min(max(exp_value,min_exposure_time),max_exposure_time)
        # Set exposure time to maximum
        node_map_remote_device.FindNode("ExposureTime").SetValue(set_exp_value)
 
    except Exception as e:
        str_error = str(e)
        print(str_error)

def set_white_balance(device, wb_r, wb_b, wb_g):  
    try:
        # Get the NodeMap of the RemoteDevice
        node_map_remote_device = device.RemoteDevice().NodeMaps()[0]
        min_gain_red = 0
        max_gain_red = 0
        inc_gain_red = 0
 
        min_gain_green = 0
        max_gain_green = 0
        inc_gain_green = 0
 
        min_gain_blue = 0
        max_gain_blue = 0
        inc_gain_blue = 0
 
        # Set selector to "DigitalRed"
        node_map_remote_device.FindNode("GainSelector").SetCurrentEntry("DigitalRed")
 
        # Get gain range.
        min_gain_red = node_map_remote_device.FindNode("Gain").Minimum()
        max_gain_red = node_map_remote_device.FindNode("Gain").Maximum()
 
        if node_map_remote_device.FindNode("Gain").HasConstantIncrement():
            inc_gain_red = node_map_remote_device.FindNode("Gain").Increment()
 
        # Get current red gain
        gain_red = node_map_remote_device.FindNode("Gain").Value()
        print("white balance red gain ",gain_red) 
        # Set gain red to values
	    # clamp 
        set_red_value = min(max(wb_r,min_gain_red),max_gain_red)
        node_map_remote_device.FindNode("Gain").SetValue(set_red_value)
 
        # Set selector to "DigitalGreen"
        node_map_remote_device.FindNode("GainSelector").SetCurrentEntry("DigitalGreen")
 
        # Get gain range.
        min_gain_green = node_map_remote_device.FindNode("Gain").Minimum()
        max_gain_green = node_map_remote_device.FindNode("Gain").Maximum()
 
        if node_map_remote_device.FindNode("Gain").HasConstantIncrement():
            inc_gain_green = node_map_remote_device.FindNode("Gain").Increment()
 
        # Get current green gain
        gain_green = node_map_remote_device.FindNode("Gain").Value()
        print("white balance green gain ",gain_green) 
	
        # Set gain green to value
	    # clamp 
        set_green_value = min(max(wb_g,min_gain_green),max_gain_green)
        node_map_remote_device.FindNode("Gain").SetValue(set_green_value)
 
        # Set selector to "DigitalBlue"
        node_map_remote_device.FindNode("GainSelector").SetCurrentEntry("DigitalBlue")
 
        # Get gain range.
        min_gain_blue = node_map_remote_device.FindNode("Gain").Minimum()
        max_gain_blue = node_map_remote_device.FindNode("Gain").Maximum()
 
        if node_map_remote_device.FindNode("Gain").HasConstantIncrement():
            inc_gain_blue = node_map_remote_device.FindNode("Gain").Increment()
 
        # Get current blue gain
        gain_blue = node_map_remote_device.FindNode("Gain").Value()
        print("white balance green gain ",gain_blue) 
	
        # Set gain blue to 1.7
	    # clamp 
        set_blue_value = min(max(wb_b,min_gain_blue),max_gain_blue)
        node_map_remote_device.FindNode("Gain").SetValue(set_blue_value)
	
    except Exception as e:
        str_error = str(e)
        print(str_error)

def set_frame_rate(node_map_remote_device,fps):
    try:
        min_frame_rate = 0
        max_frame_rate = 0
        inc_frame_rate = 0
 
        # Get frame rate range. All values in fps.
        min_frame_rate = node_map_remote_device.FindNode("AcquisitionFrameRate").Minimum()
        max_frame_rate = node_map_remote_device.FindNode("AcquisitionFrameRate").Maximum()
 
        if node_map_remote_device.FindNode("AcquisitionFrameRate").HasConstantIncrement():
            inc_frame_rate = node_map_remote_device.FindNode("AcquisitionFrameRate").Increment()
        else:
            # If there is no increment, it might be useful to choose a suitable increment for a GUI control element (e.g. a slider)
            inc_frame_rate = 0.1
 
        # Get the current frame rate
        frame_rate = node_map_remote_device.FindNode("AcquisitionFrameRate").Value()
        print("frame rate (fps) ",frame_rate) 
        
        # Set frame rate to maximum
        set_fps = min(max(fps,min_frame_rate),max_frame_rate)
        node_map_remote_device.FindNode("AcquisitionFrameRate").SetValue(set_fps)
    except Exception as e:
        str_error = str(e)
        print(str_error)
  
def prepare_acquisition(m_device):
    global m_dataStream
    try:
        data_streams = m_device.DataStreams()
        if data_streams.empty():
            # no data streams available
            return False
 
        m_dataStream = m_device.DataStreams()[0].OpenDataStream() 
        return True
    except Exception as e:
        # ...
        str_error = str(e) 
        print(str_error)
        m_dataStream = None
    return False

def set_roi(m_node_map_remote_device, x, y, width, height):
    try:
        # Get the minimum ROI and set it. After that there are no size restrictions anymore
        x_min = m_node_map_remote_device.FindNode("OffsetX").Minimum()
        y_min = m_node_map_remote_device.FindNode("OffsetY").Minimum()
        w_min = m_node_map_remote_device.FindNode("Width").Minimum()
        h_min = m_node_map_remote_device.FindNode("Height").Minimum()
 
        m_node_map_remote_device.FindNode("OffsetX").SetValue(x_min)
        m_node_map_remote_device.FindNode("OffsetY").SetValue(y_min)
        m_node_map_remote_device.FindNode("Width").SetValue(w_min)
        m_node_map_remote_device.FindNode("Height").SetValue(h_min)
 
        # Get the maximum ROI values
        x_max = m_node_map_remote_device.FindNode("OffsetX").Maximum()
        y_max = m_node_map_remote_device.FindNode("OffsetY").Maximum()
        w_max = m_node_map_remote_device.FindNode("Width").Maximum()
        h_max = m_node_map_remote_device.FindNode("Height").Maximum()
 
        if (x < x_min) or (y < y_min) or (x > x_max) or (y > y_max):
            return False
        elif (width < w_min) or (height < h_min) or ((x + width) > w_max) or ((y + height) > h_max):
            return False
        else:
            # Now, set final AOI
            m_node_map_remote_device.FindNode("OffsetX").SetValue(x)
            m_node_map_remote_device.FindNode("OffsetY").SetValue(y)
            m_node_map_remote_device.FindNode("Width").SetValue(width)
            m_node_map_remote_device.FindNode("Height").SetValue(height)
 
            return True
    except Exception as e:
        # ...
        str_error = str(e)
        print(str_error)
    return False

def alloc_and_announce_buffers(m_node_map_remote_device):
    try:
        if m_dataStream:
            # Flush queue and prepare all buffers for revoking
            m_dataStream.Flush(peak.DataStreamFlushMode_DiscardAll)
 
            # Clear all old buffers
            for buffer in m_dataStream.AnnouncedBuffers():
                m_dataStream.RevokeBuffer(buffer)
 
            payload_size = m_node_map_remote_device.FindNode("PayloadSize").Value()
 
            # Get number of minimum required buffers
            num_buffers_min_required = m_dataStream.NumBuffersAnnouncedMinRequired()
 
            # Alloc buffers
            for count in range(num_buffers_min_required):
                buffer = m_dataStream.AllocAndAnnounceBuffer(payload_size)
                m_dataStream.QueueBuffer(buffer) 
            return True
    except Exception as e:
        # ...
        str_error = str(e)
        print(str_error)
    return False

def start_acquisition(m_node_map_remote_device):
    if m_dataStream == None:
        return False
    try:
        m_dataStream.StartAcquisition(peak.AcquisitionStartMode_Default, peak.DataStream.INFINITE_NUMBER)
        m_node_map_remote_device.FindNode("TLParamsLocked").SetValue(1)
        m_node_map_remote_device.FindNode("AcquisitionStart").Execute()       
        return True
    except Exception as e:
        # ...
        str_error = str(e) 
        print(str_error)
    return False

def create_transformer_global():
    global m_image_transformer_ipl 
    m_image_transformer_ipl = ids_peak_ipl.ImageTransformer()

def get_bgra8_frame():
    try:
        # Get buffer from device's DataStream. Wait 5000 ms. The buffer is automatically locked until it is queued again.
        buffer = m_data_stream.WaitForFinishedBuffer(5000)
 
        # Create IDS peak IPL image from buffer
        image = ids_peak_ipl_extension.BufferToImage(buffer)
 
        # Convert it to RGBa8 format by debayering
        image_processed = image.ConvertTo(ids_peak_ipl.PixelFormatName_BGRa8, ids_peak_ipl.ConversionMode_Fast)
        # Queue buffer again
        m_data_stream.QueueBuffer(buffer)
        return image_processed
        		
    except Exception as e:
        # ...
        str_error = str(e)
        print(str_error)
    return None
	
def rotate_image_180():
    try:
        # Get buffer from device's DataStream. Wait 5000 ms. The buffer is automatically locked until it is queued again.
        buffer = m_data_stream.WaitForFinishedBuffer(5000)
 
        # Create IDS peak IPL image from buffer
        image = ids_peak_ipl_extension.BufferToImage(buffer)
 
        # Convert it to RGBa8 format by debayering
        image_processed = image.ConvertTo(ids_peak_ipl.PixelFormatName_BGRa8, ids_peak_ipl.ConversionMode_Fast)
        img_bgra8 = image_processed
        
        # Queue buffer again
        m_data_stream.QueueBuffer(buffer)
 
        # Mirror x and y
        m_image_transformer_ipl.MirrorUpDownLeftRightInPlace(image_processed)
 
        # Rotate by 180 degrees
        m_image_transformer_ipl.RotateInPlace(image_processed, ids_peak_ipl.ImageTransformer.RotationAngle_Degree180)
		return img_bgra8, image_processed
		
    except Exception as e:
        # ...
        str_error = str(e)
        print(str_error)
    return None, None

def edge_process_image(frame):
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.blur(grey, (7, 7))
    edges = cv2.Canny(blur, 15.0, 30.0)
    return edges
        
def main():
    # Initialize library
    ids_peak.Library.Initialize()
    
    # ROS
    rospy.init_node("ImgPublishToROS")
    # rospy.on_shutdown(cleanup)                  - if we want to use a clean-up on shutdown
 
    # Create a DeviceManager object
    device_manager = ids_peak.DeviceManager.Instance()
 
    try:
        # Update the DeviceManager
        device_manager.Update()
 
        # Exit program if no device was found
        if device_manager.Devices().empty():
            print("No device found. Exiting Program.")
            return -1
 
        # Open the first device
        device = device_manager.Devices()[0].OpenDevice(ids_peak.DeviceAccessType_Control)
 
        # now call the functions to set the camera up
        exposure_spt=600
        set_exposure(device, exposure_spt)
        last_exp_word = exposure_spt
        wb_r = 1.1
        wb_g = 1.6
        wb_b = 1.7
        set_white_balance(device,wb_r,wb_g,wb_b)

        # Get NodeMap of the RemoteDevice for all accesses to the GenICam NodeMap tree
        m_node_map_remote_device = device.RemoteDevice().NodeMaps()[0]

        if not prepare_acquisition(device):
            print("Could not prepare aquisition. Exiting Program.")
            sys.exit(-2) 
        if not set_roi(m_node_map_remote_device, 16, 16, 128, 128):
            print("Could not set R.O.I. Exiting Program.")
            sys.exit(-3)
        if not alloc_and_announce_buffers(m_node_map_remote_device):
            print("Could not malloc buffer Exiting Program.")
            sys.exit(-4) 
        if not start_acquisition(m_node_map_remote_device):
            print("Could not start aquiring Exiting Program.")
            sys.exit(-5)

        create_transformer_global()
        bridge = CvBridge()                                                              # create ros image bridge
        image_pub = rospy.Publisher("/output/image_raw", Image, queue_size=1)            # create ros image publisher
        pub = rospy.Publisher('cam_sets', String, queue_size=10)                         # create an info string with the camera settings
        r = rospy.Rate(1)                                                                # 10hz
    
        # get each frme from the camera and rotate 180 degrees
        while True:
            exp_word = int(rospy.get_param("~exposure", "default"))                     # read exposure value from a ROS parameter
            if not exp_word == last_exp_word:
                set_exposure(device, exp_word)                                          # set new exposure
                last_exp_word = exp_word
            img, img_r = rotate_image_180()                                              # get the raw frame and one rotated by 180 degrees       
            #try:                                                                       use if you need to convert back from ros formats to openCV
            #    cv_input_image = bridge.imgmsg_to_cv2(img, "bgra8")
            #except CvBridgeError as e:
            #    print(e)
            #output_image = edge_process_image(cv_input_image)   
            if not img == None:                                                          # provided we have image then show and process
                cv2.imshow("raw image", img)  
                if not img_r == None:
                    cv2.imshow("rot 180 image", img_r)                                   # show rotated image 
                word = rospy.get_param("~content", "default")                            # read if we want the rotated image or not from a param in ROS
                if int(word) == 1:
                    output_image = edge_process_image(img_r)  
                else:                    
                    output_image = edge_process_image(img)   
                cv2.imshow("edge detected image", output_image)   
                try:
                    if not rospy.is_shutdown():
                        str = "exposure: %s  white_bal(RGB) : %s %s %s" % (exposure_spt,wb_r,wb_g,wb_b)
                        pub.publish(str)                                                # publish settings to ROS
                        image_pub.publish(bridge.cv2_to_imgmsg(output_image, "mono8"))  # publish image to ROS
                        r.sleep()
                except CvBridgeError as e:
                    print(e)
                key = cv2.waitKey(10)
                if key == 27:
                    break                
    except Exception as e:
        print("EXCEPTION: " + str(e))
        return -2
 
    finally:
        device = None
        m_node_map_remote_device = None
        m_image_transformer_ipl = None
        m_dataStream = None        
        ids_peak.Library.Close()
        cv2.destroyAllWindows()
        
if __name__ == '__main__':
    main()
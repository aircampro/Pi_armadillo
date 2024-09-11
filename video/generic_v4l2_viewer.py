# 
#
# generic_v4l2_viewer.py
#
# multithreded capture and display frames 
# Work with Video4Linux (V4L2) to enable camera functionality and features
#
import cv2
import multiprocessing
import multiprocessing.sharedctypes
import time
import numpy
import sys

# set frame size
if sys.argc > 2:
    WIDTH=int(sys.argv[1])
    HEIGHT=int(sys.argv[2])
else:
    WIDTH=640
    HEIGHT=480

# sudo apt-get install v4l-utils
#
from v4l2py.device import Device, BufferType

def link_cam(id=0):
    cam = Device.from_id(id)
    cam.open()
    return cam

def unlink_cam(cam):
    cam.close()
    
def change_control_param( cam, p_name, pv ): 
    try:
        p_val = int(pv)
        for c in cam.controls.values(): 
            print(c)                                                                          # print each settable value in the camera
            if not str(c).find(p_name) == -1 :                                                # value matches
                min_value = int(c.split(p_name)[1].split("max")[0].split("=")[1])
                max_value = int(c.split(p_name)[1].split("step")[0].split("max")[1].split("=")[1])    
                if p_val > max_value :                                                        # clamp value
                    p_val = max_value
                elif p_val < min_value:
                    p_val = min_value
                if not p_name.find("brightness") == -1:                                      # set camera to the value passed
                    cam.controls.brightness.value = p_val  
                    fb = str(cam.controls.brightness)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val)                    
                elif not p_name.find("saturation") == -1:
                    cam.controls.saturation.value = p_val 
                    fb = str(cam.controls.saturation)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val)                 
                elif not p_name.find("contrast") == -1:
                    cam.controls.contrast.value = p_val 
                    fb = str(cam.controls.contrast)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val) 
                elif not p_name.find("hue") == -1:
                    cam.controls.hue.value = p_val 
                    fb = str(cam.controls.hue)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val) 
                elif not p_name.find("gamma") == -1:
                    cam.controls.gamma.value = p_val    
                    fb = str(cam.controls.gamma)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val) 
                elif not p_name.find("white_balance_temperature") == -1:
                    cam.controls.white_balance_temperature.value = p_val 
                    fb = str(cam.controls.white_balance_temperature)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val)
                elif not p_name.find("sharpness") == -1:
                    cam.controls.sharpness.value = p_val
                    fb = str(cam.controls.sharpness)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val)
                elif not p_name.find("backlight_compensation") == -1:
                    cam.controls.backlight_compensation.value = p_val
                    fb = str(cam.controls.backlight_compensation)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val)
                elif not p_name.find("exposure_time_absolute") == -1:
                    cam.controls.exposure_time_absolute.value = p_val
                    fb = str(cam.controls.exposure_time_absolute)
                    fb_val = int(fb.split("value")[1].split(" ")[0].split("=")[1].split(">")[0])
                    if fb_val == p_val :
                        print("set ",p_name," to ",p_val)
                break                
            else:
                continue
    except:
        print("call to change camera parameter failed")
        
def camera_reader(out_buf, buf1_ready):
    try:
      capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
    except TypeError:
      capture = cv2.VideoCapture(0)

    if capture.isOpened() is False:
      raise IOError

    if isinstance(capture.get(cv2.CAP_PROP_CONVERT_RGB), float):
      capture.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
    else:
      capture.set(cv2.CAP_PROP_CONVERT_RGB, False)

    capture.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    capture.set(cv2.CAP_PROP_FPS, 30)

    while(True):
      try:
        capture_start_time = time.time()
        ret, frame = capture.read()
        if ret is False:
          raise IOError
        #print("Capture FPS = ", 1.0 / (time.time() - capture_start_time))
        bgr_frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV)
        #cv2.imshow('frame2', bgr_frame)
        buf1_ready.clear()
        memoryview(out_buf).cast('B')[:] = memoryview(bgr_frame).cast('B')[:]
        buf1_ready.set()
        print("Capture+Conversion+Copy FPS = ", 1.0 / (time.time() - capture_start_time))
      except KeyboardInterrupt:
        # exit with CTRL + C 
        break
    capture.release()

if __name__ == "__main__":

    # if you want to change params in the camera first do it here
    cam = link_cam()
    change_control_param( cam, "brightness", 200 )  
    change_control_param( cam, "sharpness", 6 )  
    change_control_param( cam, "hue", 76 )
    unlink_cam(cam)
  
    buf1 = multiprocessing.sharedctypes.RawArray('B', HEIGHT*WIDTH*3)
    buf1_ready = multiprocessing.Event()
    buf1_ready.clear()
    p1=multiprocessing.Process(target=camera_reader, args=(buf1,buf1_ready), daemon=True)
    p1.start()

    captured_bgr_image = numpy.empty((HEIGHT, WIDTH, 3), dtype=numpy.uint8)
    while True:
      try:
        display_start_time = time.time()
        buf1_ready.wait()
        captured_bgr_image[:,:,:] = numpy.reshape(buf1, (HEIGHT, WIDTH, 3))
        buf1_ready.clear()
        cv2.imshow('frame', captured_bgr_image)
        cv2.waitKey(1)
        print("Display FPS = ", 1.0 / (time.time() - display_start_time))
      except KeyboardInterrupt:
        # exit if you press CTRL + C 
        print("Waiting camera reader to finish.")
        p1.join(10)
        break

    cv2.destroyAllWindows()

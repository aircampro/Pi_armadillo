#
# ============================================================================================================================
#
# ref drivers https://www.argocorp.com/software/sdk/ICImagingControl/Sample_program/Python_34/image-processing-OpenCV.html
# FPD-Link Camera controlled via input keys with face detection active
#
# ============================================================================================================================
#
import ctypes 
import tisgrabber as tis
import cv2
import numpy as np
import sys

def set_exposure(ic, hGrabber2, val_float):
    if val_float < 0:
        exposureauto = ctypes.c_long()
        ic.IC_SetPropertySwitch(hGrabber2, tis.T("Exposure"), tis.T("Auto"), exposureauto)    
        print("exposure： {0}".format(exposureauto.value))
    else:
        ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("Exposure"), tis.T("Value"), ctypes.c_float(val_float))
        expmin = ctypes.c_float()
        expmax = ctypes.c_float()
        exposure = ctypes.c_float()
        ic.IC_GetPropertyAbsoluteValue(hGrabber2, tis.T("Exposure"), tis.T("Value"), exposure)
        ic.IC_GetPropertyAbsoluteValueRange(hGrabber2, tis.T("Exposure"), tis.T("Value"), expmin, expmax)
        print("exposure ：{0}, value： {1} min ～ {2} max".format(exposure.value, expmin.value, expmax.value))

def set_gain(ic, hGrabber2, val_float):
    if val_float < 0:
        gainauto = ctypes.c_long()
        ic.IC_SetPropertySwitch(hGrabber2, tis.T("Gain"), tis.T("Auto"), gainauto)    
        print("gain： {0}".format(gainauto.value))
    else:
        ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("Gain"), tis.T("Value"), ctypes.c_float(val_float))
        gainmin = ctypes.c_long()
        gainmax = ctypes.c_long()
        gain = ctypes.c_float()
        ic.IC_GetPropertyAbsoluteValue(hGrabber2, tis.T("Gain"), tis.T("Value"), gain)
        ic.IC_GetPropertyValueRange(hGrabber2, tis.T("Gain"), tis.T("Value"), gainmin, gainmax)
        print("gain {0} value： {1} min ～ {2} max".format(gain.value, gainmin.value, gainmax.value))

def set_white_balance(ic, hGrabber2, val_floats):
    if (str(type(val_floats)).find("list")) == -1:
        vf = int(val_float)
    else:
        vf = min(val_float)
                
    if vf < 0:
        whiteBalanceauto = ctypes.c_long()
        ic.IC_SetPropertySwitch(hGrabber2, tis.T("WhiteBalance"), tis.T("Auto"), whiteBalanceauto)    
        print("white balance auto： {0}".format(whiteBalanceauto.value))
    else:
        if (str(type(val_floats)).find("list")) == -1:
            ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Red"), ctypes.c_float(int(val_floats)))
            ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Green"), ctypes.c_float(int(val_floats)))
            ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Blue"), ctypes.c_float(int(val_floats)))     
        else:            
            if len(val_floats) >= 3:
                ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Red"), ctypes.c_float(val_floats[0]))
                ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Green"), ctypes.c_float(val_floats[1]))
                ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Blue"), ctypes.c_float(val_floats[2]))
            elif len(val_floats) == 1:
                ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Red"), ctypes.c_float(val_floats[0]))
                ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Green"), ctypes.c_float(val_floats[0]))
                ic.IC_SetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Blue"), ctypes.c_float(val_floats[0]))        
        whiteBalanceRed = ctypes.c_float()
        whiteBalanceGreen = ctypes.c_float()
        whiteBalanceBlue = ctypes.c_float()
        ic.IC_GetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Red"), whiteBalanceRed)
        ic.IC_GetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Green"), whiteBalanceGreen)
        ic.IC_GetPropertyAbsoluteValue(hGrabber2, tis.T("WhiteBalance"), tis.T("White Balance Blue"), whiteBalanceBlue)
        print("white balance RGB： {0} R： {1} G： {2} B".format(whiteBalanceRed.value, whiteBalanceGreen.value, whiteBalanceBlue.value))

def trigger(ic, hGrabber2):
    ic.IC_PropertyOnePush(hGrabber2, tis.T("Trigger"), tis.T("Software Trigger"))

def read_gpio(ic, hGrabber2):
    inn = ctypes.c_long()
    inn_count=0
    while inn_count<3 :
        ic.IC_PropertyOnePush(hGrabber2,tis.T("GPIO"),tis.T("Read") )
        ic.IC_GetPropertyValue(hGrabber2,tis.T("GPIO"),tis.T("GP In"),inn )
        print("GPIO: %d" %(inn.value) )
        inn_count=inn_count+1
        time.sleep(1)

# default settings for the camera these will go ip/down depending on keys
EXPOS=0.1
GAIN=10.0
WB_RGB_VALS = [2.0, 1.0, 0.2]
DELTA_STEP=0.1

# define the actions determined by the keyboard
def add(x,y):
    return x + y

def sub(x,y):
    return x - y

# place the actions in a list to be selected by the key input
action_list = [ add, sub ]

# codec callback functions
def enumCodecCallback(codecname, userdata):
    """
    Callback function to enumerate codecs.
     ：param codecname: codec name
     ：param userdata: Python object, a list to receive codec names
     : return: 0 means continue, 1 means stop enumerating.
    """
    userdata.append(codecname.decode("utf-8"))
    return 0

enumCodecCallbackfunc = ic.ENUMCODECCB(enumCodecCallback)
codecs = []

# c structure returned from the call back function
#
class CallbackUserdata(ctypes.Structure):
    """ Example of user data passed to a callback function """
    def __init__(self, ):
        self.unused = ""
        self.Value1=0

CALLBK_ACTV = 0
IMG_WIN = 0

# function which is called when a frame is grabbed
#
def frameReadyCallback(hGrabber, pBuffer, framenumber, pData):
    #This is an example of a callback function
    #: param: hGrabber: This is the actual pointer to the grabber object (not allowed)
    #:param: pBuffer: Pointer to the byte of the first pixel
    #：param: framenumber: Number of frames since the stream started
    #：param: pData: pointer to additional user data structures
    global CALLBK_ACTV
    
    print("Callback function call", pData.Value1)
    pData.Value1 = pData.Value1 + 1

    Width = ctypes.c_long()
    Height = ctypes.c_long()
    BitsPerPixel = ctypes.c_int()
    colorformat = ctypes.c_int()

    ic.IC_GetImageDescription(hGrabber, Width, Height, BitsPerPixel, colorformat)

    bpp = int(BitsPerPixel.value/8.0)
    buffer_size = Width.value * Height.value * bpp

    if buffer_size > 0:
        image = ctypes.cast(pBuffer, ctypes.POINTER(ctypes.c_ubyte * buffer_size))

        cvMat = np.ndarray(buffer=image.contents, dtype=np.uint8, shape=(Height.value, Width.value, bpp))
        threshold = 80
        cvMat = cv2.flip(cvMat, 0) 
        cvMat = cv2.cvtColor(cvMat, cv2.COLOR_BGR2GRAY) 
        ret, img_THRESH_BINARY = cv2.threshold(cvMat, threshold, 255, cv2.THRESH_BINARY) 
        resized_img = cv2.resize(img_THRESH_BINARY,(640, 480)) 
        CALLBK_ACTV = 1
        cv2.imshow('CallBackWindow', resized_img) 
        cv2.waitKey(10)
        
#tisgrabber_x64.dll is the name for the windows dll 
# load the c++ DLL
ic = ctypes.cdll.LoadLibrary("./tisgrabber_x64.dll")
tis.declareFunctions(ic)

ic.IC_InitLibrary(0)

# print available codecs
ic.IC_enumCodecs(enumCodecCallbackfunc, codecs)
print("Available codecs:")
for codec in codecs:
    print(codec)

# set the callback function for each frame
frameReadyCallbackfunc = ic.FRAMEREADYCALLBACK(frameReadyCallback)
userdata = CallbackUserdata()
    
# open device connection   
hGrabber = tis.openDevice(ic)

# think i can use the same handle but if not example shows this call first 
# hGrabber2 = ic.IC_ShowDeviceSelectionDialog(None)
hGrabber2 = hGrabber

face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
eye_cascade = cv2.CascadeClassifier("haarcascade_eye.xml")

# default up for the key action
up_or_down = 0

stream_file_name="test_grab.avi"

# if we connected to the camera do until q pressed
if(ic.IC_IsDevValid(hGrabber)):

    #Select codecs
    #Set the codec displayed in the codec list.
    codec = ic.IC_Codec_Create(tis.T("MJPEG Compressor"))

    #Display the codec property page, if any
    if ic.IC_Codec_hasDialog(codec):
        ic.IC_Codec_showDialog(codec)

    ic.IC_SetCodec(hGrabber, codec)                               # set codec
    ic.IC_SetAVIFileName(hGrabber, tis.T(stream_file_name))       # set spool file name
    # Pause Avi Capture
    ic.IC_enableAVICapturePause(hGrabber, 1)                      # pause Avi capture

    # default start the stream    
    stream_started = False
    if ic.IC_StartLive(hGrabber, 1) == tis.IC_SUCCESS:
        stream_started = True
    else:
        print("cant start stream")
        ic.IC_Codec_Release(codec)
        ic.IC_ReleaseGrabber(hGrabber)
        sys.exit(-1)
        
    key = ""
    while key != "q" :
        print("\033[36m  -----------------------------------------  \033[32m")
        print("qー: quit camera")
        print("pー: show image")
        print("fー: show image with face detection")
        print("uー: set to increase")
        print("dー: set to decrease")
        print("zー: select fast action delta")
        print("xー: select slow action delta")
        print("eー: exposure")
        print("gー: gain")
        print("wー: white balance")
        print("tー: trigger")
        print("rー: read gpio")
        print("kー: kill stream and recording")
        print("rー: resume stream spooling to a file")
        print("cー: resume stream with callback & spooling to file")
        print("\033[36m  -----------------------------------------  \033[0m")
        key = input('enter your action please :')
        if key == "p":
            if ic.IC_SnapImage(hGrabber, 2000) == tis.IC_SUCCESS:
                Width = ctypes.c_long()
                Height = ctypes.c_long()
                BitsPerPixel = ctypes.c_int()
                colorformat = ctypes.c_int()

                ic.IC_GetImageDescription(hGrabber, Width, Height, BitsPerPixel, colorformat)

                bpp = int(BitsPerPixel.value / 8.0)
                buffer_size = Width.value * Height.value * bpp

                imagePtr = ic.IC_GetImagePtr(hGrabber)

                imagedata = ctypes.cast(imagePtr, ctypes.POINTER(ctypes.c_ubyte * buffer_size))

                image = np.ndarray(buffer=imagedata.contents, dtype=np.uint8, shape=(Height.value,  Width.value, bpp))

                image = cv2.flip(image, 0)
                image = cv2.erode(image, np.ones((11, 11)))
                        
                resized_img = cv2.resize(image,(640, 480)) 
                cv2.imshow('ImageTaken', resized_img)
                IMG_WIN = 1
                cv2.waitKey(10)
        elif key == "r" and stream_started == False:
            codec = ic.IC_Codec_Create(tis.T("MJPEG Compressor"))
            if ic.IC_Codec_hasDialog(codec):
                ic.IC_Codec_showDialog(codec)
            ic.IC_SetCodec(hGrabber, codec)                         
            ic.IC_SetAVIFileName(hGrabber, tis.T(stream_file_name))       
            ic.IC_enableAVICapturePause(hGrabber, 1)  
            
            if ic.IC_StartLive(hGrabber, 1) == tis.IC_SUCCESS:
                stream_started = True
            else:
                print("cant start stream")
                ic.IC_Codec_Release(codec)
                ic.IC_ReleaseGrabber(hGrabber)
                sys.exit(-1)
        elif key == "c" and stream_started == False:
            ic.IC_SetFrameReadyCallback(hGrabber, frameReadyCallbackfunc, userdata)
            ic.IC_SetContinuousMode(hGrabber, 0) 
            ic.IC_SetPropertySwitch(hGrabber, tis.T("Trigger"), tis.T("Enable"), 1)
            codec = ic.IC_Codec_Create(tis.T("MJPEG Compressor"))
            if ic.IC_Codec_hasDialog(codec):
                ic.IC_Codec_showDialog(codec)
            ic.IC_SetCodec(hGrabber, codec)                         
            ic.IC_SetAVIFileName(hGrabber, tis.T(stream_file_name))       
            ic.IC_enableAVICapturePause(hGrabber, 1)  
            
            if ic.IC_StartLive(hGrabber, 1) == tis.IC_SUCCESS:
                stream_started = True
            else:
                print("cant start stream")
                ic.IC_Codec_Release(codec)
                ic.IC_ReleaseGrabber(hGrabber)
                sys.exit(-1)
        elif key == "k" and stream_started == True:
                ic.IC_enableAVICapturePause(hGrabber, 0)  
                ic.IC_MsgBox(tis.T("Stopped the stream"), tis.T("AVI Capture"))
                ic.IC_StopLive(hGrabber)
                stream_started = False
                ic.IC_Codec_Release(codec)
        elif key == "f":
            if ic.IC_SnapImage(hGrabber, 2000) == tis.IC_SUCCESS:
                # define c function return types
                Width = ctypes.c_long()
                Height = ctypes.c_long()
                BitsPerPixel = ctypes.c_int()
                colorformat = ctypes.c_int()

                # get the size of the image
                ic.IC_GetImageDescription(hGrabber, Width, Height, BitsPerPixel, colorformat)

                # calculate the image buffer size
                bpp = int(BitsPerPixel.value / 8.0)
                buffer_size = Width.value * Height.value * bpp

                # get pointer to image data
                imagePtr = ic.IC_GetImagePtr(hGrabber)

                # cast pointer to the data
                imagedata = ctypes.cast(imagePtr, ctypes.POINTER(ctypes.c_ubyte * buffer_size))

                # make numpy array from the image data
                image = np.ndarray(buffer=imagedata.contents, dtype=np.uint8, shape=(Height.value,  Width.value, bpp))

                # process using OpenCV
                image = cv2.flip(image, 0)
                image = cv2.erode(image, np.ones((11, 11)))

                # convert to grayscale for cascade classifier to work best
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, 1.3, 5)

                for (x, y, w, h) in faces:
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 127, 0), 2)
                    roi_gray = gray[y : y + h, x : x + w]
                    roi_color = image[y : y + h, x : x + w]
                    eyes = eye_cascade.detectMultiScale(roi_gray)
                    for (ex, ey, ew, eh) in eyes:
                        cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 127), 2)
                        
                # re-sixe image and show it
                resized_img = cv2.resize(image,(640, 480)) 
                cv2.imshow('ImageTaken', resized_img)
                IMG_WIN = 1
                cv2.waitKey(10)
        elif key == "u":
            up_or_down = 0
        elif key == "d":
            up_or_down = 1
        elif key == "e":
            EXPOS = action_list[up_or_down](EXPOS, DELTA_STEP)
            set_exposure(ic, hGrabber2, EXPOS)       
        elif key == "g":
            GAIN = action_list[up_or_down](GAIN, DELTA_STEP)
            set_gain(ic, hGrabber2, GAIN) 
        elif key == "w":
            val_floats = (action_list[up_or_down](WB_RGB_VALS[0], DELTA_STEP), action_list[up_or_down](WB_RGB_VALS[1], DELTA_STEP), action_list[up_or_down](WB_RGB_VALS[2], DELTA_STEP)]
            set_white_balance(ic, hGrabber2, val_floats) 
        elif key == "t":
            trigger(ic, hGrabber2)         
        elif key == "r":
            read_gpio(ic, hGrabber2)      
        elif key == "x":
            DELTA_STEP=0.1      
        elif key == "z":
            DELTA_STEP=1.0                     
        else:
            print("not a valid selection")

    if stream_started == True:
        ic.IC_StopLive(hGrabber)
        ic.IC_Codec_Release(codec)

    # close openCV windows if activated
    if IMG_WIN == 1:        
        cv2.destroyWindow('ImageTaken')
    if CALLBK_ACTV == 1:
        cv2.destroyWindow('CallBackWindow')    
else:
    ic.IC_MsgBox(tis.T("No device opened"), tis.T("Simple Live Video"))

ic.IC_ReleaseGrabber(hGrabber)
#ic.IC_ReleaseGrabber(hGrabber2)
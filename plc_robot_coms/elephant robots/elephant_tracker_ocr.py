#!/usr/bin/env python3
# 
# implementation of a face tracker using OAK-D camera and 3x (X,Y,Z) PID loops to move the Elephant Robotics Arm type as chosen
# it can also reads a variety of text messages which will perform an action and movement of the robot arm and gripper if understood
#
# to install OAK-D camera
# sudo curl -fL http://docs.luxonis.com/_static/install_dependencies.sh | bash
# git clone https://github.com/luxonis/depthai.git
# install depthai
# python3 install_requirements.py
# ref and calibration scripts are here :- git clone https://github.com/tech-life-hacking/depthai.git
#
import argparse
import json
import os
import sys
import time
import traceback
from functools import cmp_to_key
from itertools import cycle
from pathlib import Path

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
os.environ["DEPTHAI_INSTALL_SIGNAL_HANDLER"] = "0"
import depthai as dai
import platform
import numpy as np

from depthai_helpers.arg_manager import parseArgs
from depthai_helpers.config_manager import ConfigManager, DEPTHAI_ZOO, DEPTHAI_VIDEOS
from depthai_helpers.metrics import MetricManager
from depthai_helpers.version_check import checkRequirementsVersion
from depthai_sdk import FPSHandler, loadModule, getDeviceInfo, downloadYTVideo, Previews, resizeLetterbox
from depthai_sdk.managers import NNetManager, PreviewManager, PipelineManager, EncodingManager, BlobManager

print('Using depthai module from: ', dai.__file__)
print('Depthai version installed: ', dai.__version__)
args = parseArgs()
if not args.skipVersionCheck and platform.machine() not in ['armv6l', 'aarch64']:
    checkRequirementsVersion()

# use easy OCR to read the text shown in fron of the camera
# e.g. joint : 2 : 2048
# moves joint 2 ro 2048
# joint : 7 : 1000
# moves gripper to 1000
#
import easyocr

class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0, O=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.targetPos=0.5
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.delta_time = 0.1
        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = O

    def update(self, feedback_value):
        error = self.targetPos - feedback_value
        delta_error = error - self.last_error
        self.PTerm = self.Kp * error
        self.ITerm += error * self.delta_time

        if (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard
        if(self.ITerm < -self.windup_guard):
           self.ITerm = -self.windup_guard

        self.DTerm = delta_error / self.delta_time
        self.last_error = error
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setTargetPosition(self, targetPos):
        self.targetPos = targetPos

    def setP(self, target):
        self.Kp = target

    def setI(self, target):
        self.Ki = target

    def setD(self, target):
        self.Kd = target
        
class Trackbars:
    instances = {}

    @staticmethod
    def createTrackbar(name, window, minVal, maxVal, defaultVal, callback):
        def fn(value):
            if Trackbars.instances[name][window] != value:
                callback(value)
            for otherWindow, previousValue in Trackbars.instances[name].items():
                if otherWindow != window and previousValue != value:
                    Trackbars.instances[name][otherWindow] = value
                    cv2.setTrackbarPos(name, otherWindow, value)

        cv2.createTrackbar(name, window, minVal, maxVal, fn)
        Trackbars.instances[name] = {**Trackbars.instances.get(name, {}), window: defaultVal}
        cv2.setTrackbarPos(name, window, defaultVal)


noop = lambda *a, **k: None

def guard(deg, threshhold):
    if deg >= threshhold:
        deg = threshhold
    elif deg <= -threshhold:
        deg =-threshhold
    else:
        pass
    return deg

# elephant robotics object either arm or palletizer on 
# if windows e.g. PORT = "COM3"  
import time
from pymycobot.mycobot import MyCobot
from pymycobot import MyCobotSocket
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Angle
from pymycobot.genre import Coord
from pymycobot import PI_PORT, PI_BAUD  # When using the Raspberry Pi version of mycobot, these two variables can be referenced to initialize MyCobot
ELEPORT = '/dev/ttyAMA0'
ELEBAUD = 1000000
ELEIPADDR = "192.168.10.22"
ELEIPPORT = 9000

# connection type is either cobot on serial=1 or over ip=2 or a palletizer on serial=3 ip =4
#
class Elephant:
    def __init__(self, conn_type=1, ep=ELEPORT, eb=ELEBAUD, eip=ELEIPADDR, eipp=ELEIPPORT):
        if conn_type == 1:
            self.mc = MyCobot(ep, eb)
        elif conn_type == 2:
            self.mc = MyCobotSocket(eip, eipp)	
        elif conn_type == 3:
            self.mc = MyPalletizer(ep, eb)			
        elif conn_type == 4:
            self.mc = MyPalletizerSocket(eip, eipp)
            
    def power_up(self):		
        self.mc.power_on()
		
    def power_down(self):
        self.mc.power_off()
		
    def rotate(self, theta, psi, r):
        self.mc.set_color(255,0,0)
        self.mc.sync_send_angles([psi,r,-r,theta,0,90],100, timeout=0.001)

    def get_current_angles(self):
        angles = self.mc.get_angles()
        print("Current Angles:", angles)
		return angles

    def send_angles(self, ang, speed):
        self.mc.send_angles(ang, speed)	
		
    def open_gripper(self, speed=70):
	    self.mc.set_gripper_state(0, speed)

    def close_gripper(self, speed=70):
	    self.mc.set_gripper_state(1, speed)		

    def open_close_gripper(self, speed=70, ts=2.0):
	    self.mc.set_gripper_state(0, speed)
        time.sleep(ts)
	    self.mc.set_gripper_state(1, speed)

    def set_joint_encoder(self, joint_pos=1, en_val=2048):
        self.mc.set_encoder(joint_pos, en_val)	

    def set_gripper_encoder(self, en_val=2048):
        self.mc.set_encoder(7, en_val)

    def set_joint_encoders(self, joint_posns=[1024, 1024, 1024, 1024, 1024, 1024],spd=20):
        self.mc.set_encoders(joint_posns, spd)

    def set_gripper_value(self, val, spd=30):
        self.mc.set_gripper_value(val, spd)	

    def get_gripper_value(self, val, spd=30):
        return self.mc.get_gripper_value()	

    def release_all_servos(self):
        self.mc.release_all_servos()

    def get_coords(self):
        # Get the current coordinates and pose of the head
        return self.mc.get_coords()	

    def set_coords(self,cords=[207.9, 47, 49.3,-159.69], spd=80):
        self.mc.send_coords(cords, spd, 0)	

    def set_X(self, pos=20, spd=80):
        self.mc.send_coord(Coord.X.value, pos, spd)

    def set_Y(self, pos=20, spd=80):
        self.mc.send_coord(Coord.Y.value, pos, spd)

    def set_Z(self, pos=20, spd=80):
        self.mc.send_coord(Coord.Z.value, pos, spd)
		
    def go_to_init_pose(self):	
        self.rotate(0, -60, 0)

class Demo:
    DISP_CONF_MIN = int(os.getenv("DISP_CONF_MIN", 0))
    DISP_CONF_MAX = int(os.getenv("DISP_CONF_MAX", 255))
    SIGMA_MIN = int(os.getenv("SIGMA_MIN", 0))
    SIGMA_MAX = int(os.getenv("SIGMA_MAX", 250))
    LRCT_MIN = int(os.getenv("LRCT_MIN", 0))
    LRCT_MAX = int(os.getenv("LRCT_MAX", 10))

    def run_all(self, conf):
        self.setup(conf)
        self.run()

    def __init__(self, displayFrames=True, onNewFrame = noop, onShowFrame = noop, onNn = noop, onReport = noop, onSetup = noop, onTeardown = noop, onIter = noop, shouldRun = lambda: True, collectMetrics=False, conn_type=1, ep=ELEPORT, eb=ELEBAUD, eip=ELEIPADDR, eipp=ELEIPPORT):
        self._openvinoVersion = None
        self._displayFrames = displayFrames
        self.toggleMetrics(collectMetrics)

        self.onNewFrame = onNewFrame
        self.onShowFrame = onShowFrame
        self.onNn = onNn
        self.onReport = onReport
        self.onSetup = onSetup
        self.onTeardown = onTeardown
        self.onIter = onIter
        self.shouldRun = shouldRun

        self.theta = 0
        self.psi = -60
        self.r = 0

        # power up cobot
        #
        self.ele_robot = Elephant(conn_type, ep, eb, eip, eipp)
        self.ele_robot.power_up()     
        self.ele_robot.go_to_init_pose()		

        # initialize the pid loops with paramters
        #
        self.PID_upd = 0
        self.XPB = 10
        self.YPB = 6.5
        self.ZPB = 50
        self.XI = 10
        self.YI = 5
        self.ZI = 30
        self.XD = 3.75
        self.YD = 2.5
        self.ZD = 20
        self.PID_delta = 0.2
        self.PosSpt = 0.5
        self.pidX = PID(self.XPB, self.XI, self.XD)
        self.pidY = PID(self.YPB, self.YI, self.YD)
        self.pidZ = PID(self.ZPB, self.ZI, self.ZD)
        self.pidX.setTargetPosition(self.PosSpt)
        self.pidY.setTargetPosition(self.PosSpt)
        self.pidZ.setTargetPosition(self.PosSpt)
        
        # Initialize the OCR reader
        self.reader = easyocr.Reader(['en'], gpu=False)

    def read_text_in_frame(f_crop, det_thresh=50.0, mr=1.1, lt=0.3):

        detections = self.reader.readtext(f_crop, mag_ratio=mr, link_threshold=lt)

        det_text = []
        for detection in detections:
            bbox, text, score = detection

            text = text.upper().replace(' ', '')

            if score > det_thresh:
                det_text.append((score, text))

        # put the highest score last so we can get it by a stack pop instruction
        det_text.sort()
	
        return det_text
    
    def setCallbacks(self, onNewFrame=None, onShowFrame=None, onNn=None, onReport=None, onSetup=None, onTeardown=None, onIter=None, shouldRun=None):
        if onNewFrame is not None:
            self.onNewFrame = onNewFrame
        if onShowFrame is not None:
            self.onShowFrame = onShowFrame
        if onNn is not None:
            self.onNn = onNn
        if onReport is not None:
            self.onReport = onReport
        if onSetup is not None:
            self.onSetup = onSetup
        if onTeardown is not None:
            self.onTeardown = onTeardown
        if onIter is not None:
            self.onIter = onIter
        if shouldRun is not None:
            self.shouldRun = shouldRun

    def toggleMetrics(self, enabled):
        if enabled:
            self.metrics = MetricManager()
        else:
            self.metrics = None

    def setup(self, conf: ConfigManager):
        print("Setting up demo...")
        self._conf = conf
        self._rgbRes = conf.getRgbResolution()
        self._monoRes = conf.getMonoResolution()
        if self._conf.args.openvinoVersion:
            self._openvinoVersion = getattr(dai.OpenVINO.Version, 'VERSION_' + self._conf.args.openvinoVersion)
        self._deviceInfo = getDeviceInfo(self._conf.args.deviceId)
        if self._conf.args.reportFile:
            reportFileP = Path(self._conf.args.reportFile).with_suffix('.csv')
            reportFileP.parent.mkdir(parents=True, exist_ok=True)
            self._reportFile = reportFileP.open('a')
        self._pm = PipelineManager(openvinoVersion=self._openvinoVersion)

        if self._conf.args.xlinkChunkSize is not None:
            self._pm.setXlinkChunkSize(self._conf.args.xlinkChunkSize)

        self._nnManager = None
        if self._conf.useNN:
            self._blobManager = BlobManager(
                zooDir=DEPTHAI_ZOO,
                zooName='face-detection-retail-0004',
            )
            self._nnManager = NNetManager(inputSize=self._conf.inputSize)

            if self._conf.getModelDir() is not None:
                configPath = self._conf.getModelDir() / Path(self._conf.getModelName()).with_suffix(f".json")
                self._nnManager.readConfig(configPath)

            self._nnManager.countLabel(self._conf.getCountLabel(self._nnManager))
            self._pm.setNnManager(self._nnManager)

        self._device = dai.Device(self._pm.pipeline.getOpenVINOVersion(), self._deviceInfo, usb2Mode=self._conf.args.usbSpeed == "usb2")
        if self.metrics is not None:
            self.metrics.reportDevice(self._device)
        if self._deviceInfo.desc.protocol == dai.XLinkProtocol.X_LINK_USB_VSC:
            print("USB Connection speed: {}".format(self._device.getUsbSpeed()))
        self._conf.adjustParamsToDevice(self._device)
        self._conf.adjustPreviewToOptions()
        if self._conf.lowBandwidth:
            self._pm.enableLowBandwidth(poeQuality=self._conf.args.poeQuality)
        self._cap = cv2.VideoCapture(self._conf.args.video) if not self._conf.useCamera else None
        self._fps = FPSHandler() if self._conf.useCamera else FPSHandler(self._cap)

        if self._conf.useCamera or self._conf.args.sync:
            self._pv = PreviewManager(display=self._conf.args.show, nnSource=self._conf.getModelSource(), colorMap=self._conf.getColorMap(),
                                dispMultiplier=self._conf.dispMultiplier, mouseTracker=True, lowBandwidth=self._conf.lowBandwidth,
                                scale=self._conf.args.scale, sync=self._conf.args.sync, fpsHandler=self._fps, createWindows=self._displayFrames,
                                depthConfig=self._pm._depthConfig)

            if self._conf.leftCameraEnabled:
                self._pm.createLeftCam(self._monoRes, self._conf.args.monoFps,
                                 orientation=self._conf.args.cameraOrientation.get(Previews.left.name),
                                 xout=Previews.left.name in self._conf.args.show and (self._conf.getModelSource() != "left" or not self._conf.args.sync))
            if self._conf.rightCameraEnabled:
                self._pm.createRightCam(self._monoRes, self._conf.args.monoFps,
                                  orientation=self._conf.args.cameraOrientation.get(Previews.right.name),
                                  xout=Previews.right.name in self._conf.args.show and (self._conf.getModelSource() != "right" or not self._conf.args.sync))
            if self._conf.rgbCameraEnabled:
                self._pm.createColorCam(self._nnManager.inputSize if self._conf.useNN else self._conf.previewSize, self._rgbRes, self._conf.args.rgbFps,
                                  orientation=self._conf.args.cameraOrientation.get(Previews.color.name),
                                  fullFov=not self._conf.args.disableFullFovNn,
                                  xout=Previews.color.name in self._conf.args.show and (self._conf.getModelSource() != "color" or not self._conf.args.sync))

            if self._conf.useDepth:
                self._pm.createDepth(
                    self._conf.args.disparityConfidenceThreshold,
                    self._conf.getMedianFilter(),
                    self._conf.args.sigma,
                    self._conf.args.stereoLrCheck,
                    self._conf.args.lrcThreshold,
                    self._conf.args.extendedDisparity,
                    self._conf.args.subpixel,
                    useDepth=Previews.depth.name in self._conf.args.show or Previews.depthRaw.name in self._conf.args.show,
                    useDisparity=Previews.disparity.name in self._conf.args.show or Previews.disparityColor.name in self._conf.args.show,
                    useRectifiedLeft=Previews.rectifiedLeft.name in self._conf.args.show and (
                                self._conf.getModelSource() != "rectifiedLeft" or not self._conf.args.sync),
                    useRectifiedRight=Previews.rectifiedRight.name in self._conf.args.show and (
                                self._conf.getModelSource() != "rectifiedRight" or not self._conf.args.sync),
                )

            self._encManager = None
            if len(self._conf.args.encode) > 0:
                self._encManager = EncodingManager(self._conf.args.encode, self._conf.args.encodeOutput)
                self._encManager.createEncoders(self._pm)

        if len(self._conf.args.report) > 0:
            self._pm.createSystemLogger()

        if self._conf.useNN:
            self._nn = self._nnManager.createNN(
                pipeline=self._pm.pipeline, nodes=self._pm.nodes, source=self._conf.getModelSource(),
                blobPath=self._blobManager.getBlob(shaves=self._conf.shaves, openvinoVersion=self._nnManager.openvinoVersion),
                useDepth=self._conf.useDepth, minDepth=self._conf.args.minDepth, maxDepth=self._conf.args.maxDepth,
                sbbScaleFactor=self._conf.args.sbbScaleFactor, fullFov=not self._conf.args.disableFullFovNn,
                flipDetection=self._conf.getModelSource() in (
                "rectifiedLeft", "rectifiedRight") and not self._conf.args.stereoLrCheck,
            )

            self._pm.addNn(
                nn=self._nn, sync=self._conf.args.sync, xoutNnInput=Previews.nnInput.name in self._conf.args.show,
                useDepth=self._conf.useDepth, xoutSbb=self._conf.args.spatialBoundingBox and self._conf.useDepth
            )

    def run(self):
        self._device.startPipeline(self._pm.pipeline)
        self._pm.createDefaultQueues(self._device)
        if self._conf.useNN:
            self._nnManager.createQueues(self._device)

        self._sbbOut = self._device.getOutputQueue("sbb", maxSize=1, blocking=False) if self._conf.useNN and self._conf.args.spatialBoundingBox else None
        self._logOut = self._device.getOutputQueue("systemLogger", maxSize=30, blocking=False) if len(self._conf.args.report) > 0 else None

        if self._conf.useDepth:
            self._medianFilters = cycle([item for name, item in vars(dai.MedianFilter).items() if name.startswith('KERNEL_') or name.startswith('MEDIAN_')])
            for medFilter in self._medianFilters:
                # move the cycle to the current median filter
                if medFilter == self._pm._depthConfig.postProcessing.median:
                    break
        else:
            self._medianFilters = []

        if self._conf.useCamera:
            cameras = self._device.getConnectedCameras()
            if dai.CameraBoardSocket.LEFT in cameras and dai.CameraBoardSocket.RIGHT in cameras:
                self._pv.collectCalibData(self._device)

            self._cameraConfig = {
                "exposure": self._conf.args.cameraExposure,
                "sensitivity": self._conf.args.cameraSensitivity,
                "saturation": self._conf.args.cameraSaturation,
                "contrast": self._conf.args.cameraContrast,
                "brightness": self._conf.args.cameraBrightness,
                "sharpness": self._conf.args.cameraSharpness
            }

            if any(self._cameraConfig.values()):
                self._updateCameraConfigs()

            self._pv.createQueues(self._device, self._createQueueCallback)
            if self._encManager is not None:
                self._encManager.createDefaultQueues(self._device)
        elif self._conf.args.sync:
            self._hostOut = self._device.getOutputQueue(Previews.nnInput.name, maxSize=1, blocking=False)

        self._seqNum = 0
        self._hostFrame = None
        self._nnData = []
        self._sbbRois = []
        self.onSetup(self)

        try:
            while self.shouldRun():
                self._fps.nextIter()
                self.onIter(self)
                self.loop()
        except StopIteration:
            pass
        finally:
            self.stop()

    def stop(self):
        print("Stopping demo...")
        self._device.close()
        del self._device
        self._pm.closeDefaultQueues()
        if self._conf.useCamera:
            self._pv.closeQueues()
            if self._encManager is not None:
                self._encManager.close()
        if self._nnManager is not None:
            self._nnManager.closeQueues()
        if self._sbbOut is not None:
            self._sbbOut.close()
        if self._logOut is not None:
            self._logOut.close()
        self._fps.printStatus()
        self.onTeardown(self)

    def read_text_move_cobot(self, f=self._hostFrame):
        # read text(s) in frame and if highest confidence is recognised move the cobot with the command and value read
        #        
        frame_crop_gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        _, frame_crop_thresh = cv2.threshold(frame_crop_gray, 64, 255, cv2.THRESH_BINARY_INV)
        texts_seen = self.read_text_in_frame(frame_crop_thresh)
        if texts_seen is not None:
            for t in texts_seen:
                print(f"text seen {t[1]} score {t[0]}")
            highest_conf_text = str(texts_seen.pop()[1])
            
            if not highest_conf_text.find("x coord :") == -1:	
                xpos = int(highest_conf_text.split(":")[1].replace(' ',''))	
                self.ele_robot.set_X(xpos)	
            elif not highest_conf_text.find("y coord :") == -1:	
                ypos = int(highest_conf_text.split(":")[1].replace(' ',''))	
                self.ele_robot.set_Y(ypos)
            elif not highest_conf_text.find("z coord :") == -1:	
                zpos = int(highest_conf_text.split(":")[1].replace(' ',''))	
                self.ele_robot.set_Z(zpos)
            elif not highest_conf_text.find("grip :") == -1:	
                gval = int(highest_conf_text.split(":")[1].replace(' ',''))	
                self.ele_robot.set_gripper_value(gval)
            elif not highest_conf_text.find("joint :") == -1:	
                joint = int(highest_conf_text.split(":")[1].replace(' ',''))
                jval = int(highest_conf_text.split(":")[2].replace(' ',''))				
                self.ele_robot.set_joint_encoder(joint, jval)

    def loop(self):
        if self._conf.useCamera:
            self._pv.prepareFrames(callback=self.onNewFrame)
            if self._encManager is not None:
                self._encManager.parseQueues()

            if self._sbbOut is not None:
                sbb = self._sbbOut.tryGet()
                if sbb is not None:
                    self._sbbRois = sbb.getConfigData()
                depthFrames = [self._pv.get(Previews.depthRaw.name), self._pv.get(Previews.depth.name)]
                for depthFrame in depthFrames:
                    if depthFrame is None:
                        continue

                    for roiData in self._sbbRois:
                        roi = roiData.roi.denormalize(depthFrame.shape[1], depthFrame.shape[0])
                        topLeft = roi.topLeft()
                        bottomRight = roi.bottomRight()
                        # Display SBB on the disparity map
                        cv2.rectangle(depthFrame, (int(topLeft.x), int(topLeft.y)), (int(bottomRight.x), int(bottomRight.y)), self._nnManager._bboxColors[0], 2)
        else:
            readCorrectly, rawHostFrame = self._cap.read()
            if not readCorrectly:
                raise StopIteration()
			
            self._nnManager.sendInputFrame(rawHostFrame, self._seqNum)
            self._seqNum += 1

            if not self._conf.args.sync:
                self._hostFrame = rawHostFrame
            self._fps.tick('host')

        if self._nnManager is not None:
            inNn = self._nnManager.outputQueue.tryGet()
            if inNn is not None:
                self.onNn(inNn)
                if not self._conf.useCamera and self._conf.args.sync:
                    self._hostFrame = Previews.nnInput.value(self._hostOut.get())
                self._nnData = self._nnManager.decode(inNn)
                self._fps.tick('nn')

        # move cobot to track the face
        # nnData[0] shows the coordinates of the four corners of the BoundingBox that surround the face detected by the OAK-D camera
        # we use these to find the center point of the face bounding box
        # spatialCoordinates.z is a method that returns the measurement result of the distance between the camera and the face.
        #
        if self._conf.useNN:
            if inNn is not None:
                try:
                    x = (self._nnData[0].xmin + self._nnData[0].xmax) / 2
                    y = (self._nnData[0].ymin + self._nnData[0].ymax) / 2
                    z = int(self._nnData[0].spatialCoordinates.z) / 1000
                    print(f"Face co-ords for tracker PID loop X={x}, Y={y}, Z={z}")
                    self.pidX.update(x)
                    self.pidY.update(y)
                    self.pidZ.update(z)
                    self.psi += self.pidX.output
                    self.theta += self.pidY.output
                    self.r += self.pidZ.output

                    self.psi = guard(self.psi, 90)
                    self.theta = guard(self.theta, 160)
                    self.r = guard(self.r, 90)

                    self.ele_robot.rotate(self.theta, self.psi, self.r)
                except IndexError:
                    pass

        if self._conf.useCamera:
            if self._nnManager is not None:
                self._nnManager.draw(self._pv, self._nnData)
            self._pv.showFrames(callback=self._showFramesCallback)
        elif self._hostFrame is not None:  
            # read text in frame and if recognised move the cobot
            #        
            self.read_text_move_cobot()
                
            debugHostFrame = self._hostFrame.copy()
            if self._nnManager is not None:
                self._nnManager.draw(debugHostFrame, self._nnData)
            self._fps.drawFps(debugHostFrame, "host")
            if self._displayFrames:
                cv2.imshow("host", debugHostFrame)

        if self._logOut:
            logs = self._logOut.tryGetAll()
            for log in logs:
                self._printSysInfo(log)

        if self._displayFrames:
            key = cv2.waitKey(1)
            if key == ord('q'):
                raise StopIteration()
            elif key == ord('m'):
                nextFilter = next(self._medianFilters)
                self._pm.updateDepthConfig(self._device, median=nextFilter)

            # adjust camera params or PID tuning 
            #
            if self._conf.args.cameraControlls:
                update = True

                if key == ord('t'):
                    self._cameraConfig["exposure"] = 10000 if self._cameraConfig["exposure"] is None else 500 if self._cameraConfig["exposure"] == 1 else min(self._cameraConfig["exposure"] + 500, 33000)
                    if self._cameraConfig["sensitivity"] is None:
                        self._cameraConfig["sensitivity"] = 800
                elif key == ord('g'):
                    self._cameraConfig["exposure"] = 10000 if self._cameraConfig["exposure"] is None else max(self._cameraConfig["exposure"] - 500, 1)
                    if self._cameraConfig["sensitivity"] is None:
                        self._cameraConfig["sensitivity"] = 800
                elif key == ord('y'):
                    self._cameraConfig["sensitivity"] = 800 if self._cameraConfig["sensitivity"] is None else min(self._cameraConfig["sensitivity"] + 50, 1600)
                    if self._cameraConfig["exposure"] is None:
                        self._cameraConfig["exposure"] = 10000
                elif key == ord('h'):
                    self._cameraConfig["sensitivity"] = 800 if self._cameraConfig["sensitivity"] is None else max(self._cameraConfig["sensitivity"] - 50, 100)
                    if self._cameraConfig["exposure"] is None:
                        self._cameraConfig["exposure"] = 10000
                elif key == ord('u'):
                    self._cameraConfig["saturation"] = 0 if self._cameraConfig["saturation"] is None else min(self._cameraConfig["saturation"] + 1, 10)
                elif key == ord('j'):
                    self._cameraConfig["saturation"] = 0 if self._cameraConfig["saturation"] is None else max(self._cameraConfig["saturation"] - 1, -10)
                elif key == ord('i'):
                    self._cameraConfig["contrast"] = 0 if self._cameraConfig["contrast"] is None else min(self._cameraConfig["contrast"] + 1, 10)
                elif key == ord('k'):
                    self._cameraConfig["contrast"] = 0 if self._cameraConfig["contrast"] is None else max(self._cameraConfig["contrast"] - 1, -10)
                elif key == ord('o'):
                    self._cameraConfig["brightness"] = 0 if self._cameraConfig["brightness"] is None else min(self._cameraConfig["brightness"] + 1, 10)
                elif key == ord('l'):
                    self._cameraConfig["brightness"] = 0 if self._cameraConfig["brightness"] is None else max(self._cameraConfig["brightness"] - 1, -10)
                elif key == ord('p'):
                    self._cameraConfig["sharpness"] = 0 if self._cameraConfig["sharpness"] is None else min(self._cameraConfig["sharpness"] + 1, 4)
                elif key == ord(';'):
                    self._cameraConfig["sharpness"] = 0 if self._cameraConfig["sharpness"] is None else max(self._cameraConfig["sharpness"] - 1, 0)
                elif key == ord('U'):
                    self.PID_upd = 0
                elif key == ord('D'):
                    self.PID_upd = 1
                elif key == ord('X'):
                    if self.PID_upd == 0:
                        self.XPB += self.PID_delta
                    elif self.PID_upd == 1:
                        self.XPB -= self.PID_delta
                    self.pidX.setP(self.XPB)
                elif key == ord('Y'):
                    if self.PID_upd == 0:
                        self.YPB += self.PID_delta
                    elif self.PID_upd == 1:
                        self.YPB -= self.PID_delta)
                    self.pidY.setP(self.YPB)
                elif key == ord('Z'):
                    if self.PID_upd == 0:
                        self.ZPB += self.PID_delta
                    elif self.PID_upd == 1:
                        self.ZPB -= self.PID_delta
                    self.pidZ.setP(self.ZPB)
                elif key == ord('C'):
                    if self.PID_upd == 0:
                        self.XI += self.PID_delta
                    elif self.PID_upd == 1:
                        self.XI -= self.PID_delta
                    self.pidX.setI(self.XI)
                elif key == ord('V'):
                    if self.PID_upd == 0:
                        self.YI += self.PID_delta
                    elif self.PID_upd == 1:
                        self.YI -= self.PID_delta
                    self.pidY.setI(self.YI)
                elif key == ord('B'):
                    if self.PID_upd == 0:
                        self.ZI += self.PID_delta
                    elif self.PID_upd == 1:
                        self.ZI -= self.PID_delta
                    self.pidZ.setI(self.ZI)
                elif key == ord('N'):
                    if self.PID_upd == 0:
                        self.XD += self.PID_delta
                    elif self.PID_upd == 1:
                        self.XD -= self.PID_delta
                    self.pidX.setD(self.XD)
                elif key == ord('M'):
                    if self.PID_upd == 0:
                        self.YD += self.PID_delta
                    elif self.PID_upd == 1:
                        self.YD -= self.PID_delta
                    self.pidY.setD(self.YD)
                elif key == ord('K'):
                    if self.PID_upd == 0:
                        self.ZD += self.PID_delta
                    elif self.PID_upd == 1:
                        self.ZD -= self.PID_delta
                    self.pidZ.setD(self.ZD)
                elif key == ord('P'):
                    print(f"X axis pid p={self.XPB} i={self.XI} d={self.XD}")
                    print(f"Y axis pid p={self.YPB} i={self.YI} d={self.YD}")
                    print(f"Z axis pid p={self.ZPB} i={self.ZI} d={self.ZD}")
                elif key == ord('R'):                    
                    self.PID_upd = 0
                    self.XPB = 10
                    self.YPB = 6.5
                    self.ZPB = 50
                    self.XI = 10
                    self.YI = 5
                    self.ZI = 30
                    self.XD = 3.75
                    self.YD = 2.5
                    self.ZD = 20
                    self.PID_delta = 0.2
                    self.PosSpt = 0.5
                    self.pidX = PID(self.XPB, self.XI, self.XD, self.pidX.output)
                    self.pidY = PID(self.YPB, self.YI, self.YD, self.pidY.output)
                    self.pidZ = PID(self.ZPB, self.ZI, self.ZD, self.pidZ.output)
                    self.pidX.setTargetPosition(self.PosSpt)
                    self.pidY.setTargetPosition(self.PosSpt)
                    self.pidZ.setTargetPosition(self.PosSpt)
                else:
                    update = False

                if update:
                    self._updateCameraConfigs()

    def _createQueueCallback(self, queueName):
        if self._displayFrames and queueName in [Previews.disparityColor.name, Previews.disparity.name, Previews.depth.name, Previews.depthRaw.name]:
            Trackbars.createTrackbar('Disparity confidence', queueName, self.DISP_CONF_MIN, self.DISP_CONF_MAX, self._conf.args.disparityConfidenceThreshold,
                     lambda value: self._pm.updateDepthConfig(self._device, dct=value))
            if queueName in [Previews.depthRaw.name, Previews.depth.name]:
                Trackbars.createTrackbar('Bilateral sigma', queueName, self.SIGMA_MIN, self.SIGMA_MAX, self._conf.args.sigma,
                         lambda value: self._pm.updateDepthConfig(self._device, sigma=value))
            if self._conf.args.stereoLrCheck:
                Trackbars.createTrackbar('LR-check threshold', queueName, self.LRCT_MIN, self.LRCT_MAX, self._conf.args.lrcThreshold,
                         lambda value: self._pm.updateDepthConfig(self._device, lrcThreshold=value))

    def _updateCameraConfigs(self):
        parsedConfig = {}
        for configOption, values in self._cameraConfig.items():
            if values is not None:
                for cameraName, value in values:
                    newConfig = {
                        **parsedConfig.get(cameraName, {}),
                        configOption: value
                    }
                    if cameraName == "all":
                        parsedConfig[Previews.left.name] = newConfig
                        parsedConfig[Previews.right.name] = newConfig
                        parsedConfig[Previews.color.name] = newConfig
                    else:
                        parsedConfig[cameraName] = newConfig

        if hasattr(self, "_device"):
            if self._conf.leftCameraEnabled and Previews.left.name in parsedConfig:
                self._pm.updateLeftCamConfig(self._device, **parsedConfig[Previews.left.name])
            if self._conf.rightCameraEnabled and Previews.right.name in parsedConfig:
                self._pm.updateRightCamConfig(self._device, **parsedConfig[Previews.right.name])
            if self._conf.rgbCameraEnabled and Previews.color.name in parsedConfig:
                self._pm.updateColorCamConfig(self._device, **parsedConfig[Previews.color.name])

    def _showFramesCallback(self, frame, name):
        returnFrame = self.onShowFrame(frame, name)
        #self.read_text_move_cobot(returnFrame)
        return returnFrame if returnFrame is not None else frame


    def _printSysInfo(self, info):
        m = 1024 * 1024 # MiB
        if not hasattr(self, "_reportFile"):
            if "memory" in self._conf.args.report:
                print(f"Drr used / total - {info.ddrMemoryUsage.used / m:.2f} / {info.ddrMemoryUsage.total / m:.2f} MiB")
                print(f"Cmx used / total - {info.cmxMemoryUsage.used / m:.2f} / {info.cmxMemoryUsage.total / m:.2f} MiB")
                print(f"LeonCss heap used / total - {info.leonCssMemoryUsage.used / m:.2f} / {info.leonCssMemoryUsage.total / m:.2f} MiB")
                print(f"LeonMss heap used / total - {info.leonMssMemoryUsage.used / m:.2f} / {info.leonMssMemoryUsage.total / m:.2f} MiB")
            if "temp" in self._conf.args.report:
                t = info.chipTemperature
                print(f"Chip temperature - average: {t.average:.2f}, css: {t.css:.2f}, mss: {t.mss:.2f}, upa0: {t.upa:.2f}, upa1: {t.dss:.2f}")
            if "cpu" in self._conf.args.report:
                print(f"Cpu usage - Leon OS: {info.leonCssCpuUsage.average * 100:.2f}%, Leon RT: {info.leonMssCpuUsage.average * 100:.2f} %")
            print("----------------------------------------")
        else:
            data = {}
            if "memory" in self._conf.args.report:
                data = {
                    **data,
                    "ddrUsed": info.ddrMemoryUsage.used,
                    "ddrTotal": info.ddrMemoryUsage.total,
                    "cmxUsed": info.cmxMemoryUsage.used,
                    "cmxTotal": info.cmxMemoryUsage.total,
                    "leonCssUsed": info.leonCssMemoryUsage.used,
                    "leonCssTotal": info.leonCssMemoryUsage.total,
                    "leonMssUsed": info.leonMssMemoryUsage.used,
                    "leonMssTotal": info.leonMssMemoryUsage.total,
                }
            if "temp" in self._conf.args.report:
                data = {
                    **data,
                    "tempAvg": info.chipTemperature.average,
                    "tempCss": info.chipTemperature.css,
                    "tempMss": info.chipTemperature.mss,
                    "tempUpa0": info.chipTemperature.upa,
                    "tempUpa1": info.chipTemperature.dss,
                }
            if "cpu" in self._conf.args.report:
                data = {
                    **data,
                    "cpuCssAvg": info.leonCssCpuUsage.average,
                    "cpuMssAvg": info.leonMssCpuUsage.average,
                }

            if self._reportFile.tell() == 0:
                print(','.join(data.keys()), file=self._reportFile)
            self.onReport(data)
            print(','.join(map(str, data.values())), file=self._reportFile)


def prepareConfManager(in_args):
    confManager = ConfigManager(in_args)
    confManager.linuxCheckApplyUsbRules()
    if not confManager.useCamera:
        if str(confManager.args.video).startswith('https'):
            confManager.args.video = downloadYTVideo(confManager.args.video, DEPTHAI_VIDEOS)
            print("Youtube video downloaded.")
        if not Path(confManager.args.video).exists():
            raise ValueError("Path {} does not exists!".format(confManager.args.video))
    return confManager

def runOpenCv():
    confManager = prepareConfManager(args)
    demo = Demo()
    demo.run_all(confManager)

if __name__ == "__main__":
    use_cv = args.guiType == "cv"
    if use_cv:
        args.guiType = "cv"
        runOpenCv()
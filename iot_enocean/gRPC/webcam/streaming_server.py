# webcam_streaming_service.py
# class lib for gRPC webstreaming service
#
from queue import Queue
import time
import threading
import base64

import cv2
# import the libs generated for protobuf
import webcam_streaming_pb2
import webcam_streaming_pb2_grpc

# gRPC webstreaming service from usb webcam usbp == usb port number wuth camera connected
#
class WebcamStreamingService( webcam_streaming_pb2_grpc.WebcamStreamingServicer ):
    def __init__(self, usbp=0, m=0, b=False, w=640, h=480, f=30):
        super(WebcamStreamingService, self).__init__()
        self.__capture_data_queue_list = []
        self.__capture_thread = threading.Thread(target=self.__capture_image, name="Capture Thread" )
        self.__capture_thread.setDaemon(True)
        self.usb_port = usbp
        self.mode = m
        self.b64 = b    
        self.w = w
        self.h = h
        self.f = f
        
    def __capture_image(self):
        cap = cv2.VideoCapture(self.usb_port, cv2.CAP_DSHOW)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        actv = True
        while actv == True:
            try:
                ret, frame = cap.read()
                if ret:
                    if self.mode == 1:
                        captureBuffer = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    elif self.mode == 2: 
                        captureBuffer = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    else:
                        captureBuffer = frame
                        
                    _, buf = cv2.imencode(".jpg", captureBuffer)

                    if self.b64 == True:
                        b64e = base64.b64encode(buf)
                        buf = b64e
                        
                    for queue in self.__capture_data_queue_list:
                        queue.put(buf.tobytes())

                    # f=30fps default
                    time.sleep(1 / self.f)
            except KeyboardInterrupt:
                actv = False
                cap.release()
                
    def StartWebcamStreaming(self, request, context):
        print(f"WebcamStreaming Start! Request from {request.clientName}")
        capture_data_queue = Queue()
        self.__capture_data_queue_list.append(capture_data_queue)

        if not self.__capture_thread.is_alive():
            self.__capture_thread.start()

        actv = True
        while actv == True:                                      # Capture Thread
            try:
                data = capture_data_queue.get()
                yield webcam_streaming_pb2.CaptureImage(imageBytes=data)
                capture_data_queue.task_done()
            except KeyboardInterrupt:
                actv = False
                print("keyboard interrupt!")

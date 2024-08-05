# thermal_camera_init.py adapted from ref:- https://github.com/soorajsknair93/PureThermal2-FLIR-Lepton3.5-Interfacing-Python
#
from queue import Queue
from uvctypes import *
import numpy as np
import cv2

class ThermalCamera:
    def __init__(self, s_fmt, v_fmt):

        self.BUF_SIZE = 2
        self.q = Queue(self.BUF_SIZE)
        self.PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(self.py_frame_callback)
        self.strm_fmt = s_fmt
        self.vs_fmt = v_fmt
        self.ctx = POINTER(uvc_context)()
        self.dev = POINTER(uvc_device)()
        self.devh = POINTER(uvc_device_handle)()
        self.ctrl = uvc_stream_ctrl()
        self.init_thermal_data_frames(self.strm_fmt, self.vs_fmt)
        self.state = 0
        
    def __exit__(self, exc_type, exc_value, traceback):
        if self.state == 2:
            libuvc.uvc_unref_device(self.dev)  
        if self.state >= 1:      
            libuvc.uvc_exit(self.ctx)
        
    def py_frame_callback(self, frame, userptr):

        array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
        data = np.frombuffer(
            array_pointer.contents, dtype=np.dtype(np.uint16)
        ).reshape(
            frame.contents.height, frame.contents.width
        )
        if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
            return

        if not self.q.full():
            self.q.put(data)

    def init_thermal_data_frames(self, strm_format, vs_format):
        res = libuvc.uvc_init(byref(self.ctx), 0)
        if res < 0:
            print("uvc_init error")
            exit(1)

        try:
            res = libuvc.uvc_find_device(self.ctx, byref(self.dev), PT_USB_VID, PT_USB_PID, 0)
            if res < 0:
                print("uvc_find_device error")
                exit(1)
            self.state = 1
            
            try:
                res = libuvc.uvc_open(self.dev, byref(self.devh))
                if res < 0:
                    print("uvc_open error")
                    exit(1)

                print("device opened!")
                self.state = 2
                
                print_device_info(self.devh)
                print_device_formats(self.devh)
                set_manual_ffc(self.devh)
                if vs_format == 0:
                    frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_Y16)
                elif vs_format == 1:
                    frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_GREY)
                elif vs_format == 2:
                    frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_YUYV)
                elif vs_format == 3:
                    frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_NV12)
                elif vs_format == 4:
                    frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_YU12)
                elif vs_format == 5:
                    frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_BGR3)
                elif vs_format == 6:
                    frame_formats = uvc_get_frame_formats_by_guid(self.devh, VS_FMT_GUID_RGB565)
                    
                if len(frame_formats) == 0:
                    print("device does not support Y16")
                    exit(1)

                # choose the output format
                if strm_format == 0:
                    libuvc.uvc_get_stream_ctrl_format_size(self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_Y16,
                                                           frame_formats[0].wWidth, frame_formats[0].wHeight,
                                                           int(1e7 / frame_formats[0].dwDefaultFrameInterval)
                                                           )
                elif strm_format == 1:                                                                      
                    libuvc.uvc_get_stream_ctrl_format_size(self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_BGR,
                                                           frame_formats[0].wWidth, frame_formats[0].wHeight,
                                                           int(1e7 / frame_formats[0].dwDefaultFrameInterval)
                                                           )
                elif strm_format == 2:  
                    libuvc.uvc_get_stream_ctrl_format_size(self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_RGB,
                                                           frame_formats[0].wWidth, frame_formats[0].wHeight,
                                                           int(1e7 / frame_formats[0].dwDefaultFrameInterval)
                                                           )
                elif strm_format == 3:  
                    libuvc.uvc_get_stream_ctrl_format_size(self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_I420,
                                                           frame_formats[0].wWidth, frame_formats[0].wHeight,
                                                           int(1e7 / frame_formats[0].dwDefaultFrameInterval)
                                                           )
                elif strm_format == 4:  
                    libuvc.uvc_get_stream_ctrl_format_size(self.devh, byref(self.ctrl), UVC_FRAME_FORMAT_UYVY,
                                                           frame_formats[0].wWidth, frame_formats[0].wHeight,
                                                           int(1e7 / frame_formats[0].dwDefaultFrameInterval)
                                                           )
            except Exception as e:
                print("Error while opening camera")
                print(e)
                libuvc.uvc_unref_device(self.dev)
        except Exception as e:
            print("Error while opening camera")
            print(e)
            libuvc.uvc_exit(self.ctx)

    def read_thermal_data(self):
        res = libuvc.uvc_start_streaming(self.devh, byref(self.ctrl), self.PTR_PY_FRAME_CALLBACK, None, 0)
        if res < 0:
            print("uvc_start_streaming failed: {0}".format(res))
            exit(1)

        try:
            while True:
                data = self.q.get(True, 500)
                if data is not None:
                    yield data
        except Exception as e:
            print("Error while reading from camera")
            print(e)
        finally:
            print("______________STOPPED_____________________")
            libuvc.uvc_stop_streaming(self.devh)

    def raw_to_8bit(self, data):
        cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
        np.right_shift(data, 8, data)
        return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

    def performffc(self):
        perform_manual_ffc(self.devh)

    def print_shutter_info(self):
        print_shutter_info(self.devh)

    def setmanualffc(self):
        set_manual_ffc(self.devh)

    def setautoffc(self):
        set_auto_ffc(self.devh)
		
#!/usr/bin/env python
#
# Set-up Ricoh 360 degree camera (any PTP like parrot sequoia and stream its output
#
# ref:- https://github.com/Parrot-Developers/sequoia-ptpy/tree/master
import ptpy
from ptpy.transports.usb import find_usb_cameras
from threading import Thread, Event
import sys
import logging
from rainbow_logging_handler import RainbowLoggingHandler
from time import sleep, time
# ref:- https://github.com/olkham/IPyCam/blob/main/examples/hardware_ptz.py
from ipycam import IPCamera, CameraConfig, PTZHardwareHandler

# Set up log
log = logging.getLogger('Live')
formatter = logging.Formatter(
    '%(levelname).1s '
    '%(relativeCreated)d '
    '%(name)s'
    '[%(threadName)s] '
    '%(message)s'
)
handler = RainbowLoggingHandler(
    sys.stderr,
)
level = 'INFO'
log.setLevel(level)
handler.setFormatter(formatter)
log.addHandler(handler)
cam_no=1
dwnload = 0
rtsp_st = 1
option = rtsp_st                                        # choose to stream or save the picture data
CONST_CONFIG_FILENAME = "/ipycam/go2rtc.yml"
confg = None
with open(CONST_CONFIG_FILENAME, 'r') as yml:
	confg = yaml.safe_load(yml)
	print(confg)
# get the relevant port data from yaml config file
webrtc = int(confg['webrtc']['listen'],split(":")[1])
rtsp_pt = int(confg['rtsp']['listen'],split(":")[1])
api_pt = int(confg['api']['listen'],split(":")[1])
rtmp_pt = int(confg['rtmp']['listen'],split(":")[1])
onvif_start = 8079                                                              # where to start the onvif port mapping
redirect_port = 8079                                                            # where to start the redirect port mapping

# Set up threads and events
finished = Event()

def setup_cam(camera, bl=20.0, fn=180, wb=3, scm=1):
    bat_low_level = bl
    with camera.session():
        valu = camera.get_device_prop_desc('FNumber')
        tries = 0
        while valu.CurrentValue != fn and tries < 3:
            tries += 1
            sleep(1)
            attempt = camera.set_device_prop_value('FNumber', fn)
            valu = camera.get_device_prop_desc('FNumber')
            print('Attempt response: {}'.format(attempt))
            print('Current FNumber value: {}'.format(valu.CurrentValue/100.0))
        valu = camera.get_device_prop_desc('WhiteBalance')
        tries = 0
        while valu.CurrentValue != wb and tries < 3:
            tries += 1
            sleep(1)
            attempt = camera.set_device_prop_value('WhiteBalance', wb)
            valu = camera.get_device_prop_desc('WhiteBalance')
            print('Attempt response: {}'.format(attempt))
            print('Current WhiteBalance value: {}'.format(valu.CurrentValue))
        valu = camera.get_device_prop_desc('StillCaptureMode')
        tries = 0
        while valu.CurrentValue != scm and tries < 3:
            tries += 1
            sleep(1)
            attempt = camera.set_device_prop_value('StillCaptureMode', scm)
            valu = camera.get_device_prop_desc('StillCaptureMode')
            print('Attempt response: {}'.format(attempt))
            print('Current StillCaptureMode value: {}'.format(valu.CurrentValue))
        valu = camera.get_device_prop_desc('BatteryLevel')
        print('Current BatteryLevel value: {}'.format(valu.CurrentValue))
        if valu.CurrentValue < bat_low_level:
            print("not enough battery")
            sys.exit(-1)

# captures
def capture_thread(camera):
    '''Initiate captures regularly for camera'''
    with camera.session():
        info = camera.get_device_info()
        while not finished.is_set():
            capture = camera.initiate_capture()
            if capture.ResponseCode == 'OK':
                log.info(
                    '{}: successfully initiated capture'
                    .format(info.SerialNumber)
                )
            sleep(.1)

# saves to file
def download_thread(camera):
    '''Download all non-folders in events from camera'''
    with camera.session():
        caminfo = camera.get_device_info()
        while not finished.is_set():
            event = camera.event()
            if event and event.EventCode == 'ObjectAdded':
                handle = event.Parameter[0]
                info = camera.get_object_info(handle)
                # Download all things that are not groups of other things.
                if info.ObjectFormat != 'Association':
                    log.info(
                        '{}: downloading {}'
                        .format(caminfo.SerialNumber, info.Filename)
                    )
                    tic = time()
                    obj = camera.get_object(handle)
                    toc = time()
                    log.info('{}: {:.1f}MB/s'.format(
                        caminfo.SerialNumber,
                        len(obj.Data) / ((toc - tic) * 1e6))
                    )
                    with open(info.Filename, mode='w') as f:
                        f.write(obj.Data)
                        
# streams the frames captured
def rtsp_thread(camera):
    '''create rtsp stream for each camera'''
    # The RICOH THETA Z1 supports approximately 23-megapixels resolution (6720 x 3360 pixels)
    global cam_no, onvif_start, rtsp_pt
    config = CameraConfig(name=f"PTP Camera No. {cam_no}",
        manufacturer="RICOH",
        model="THETA Z1",
        serial_number=f"RTZ1-{cam_no}",
        # Stream settings
        main_width=6720,
        main_height=3360,
        main_fps=10,
        main_bitrate="2M",
        main_stream_name: str = f"video_main_{cam_no}"        
        # Substream settings
        sub_width=640,
        sub_height=360,
        sub_bitrate="512K",
        sub_stream_name: str = f"video_sub_{cam_no}"
        # Hardware acceleration
        hw_accel="auto",                                                          # Options: "auto", "nvenc", "qsv", "cpu"
        # Network ports (optional, uses defaults if not specified)
        onvif_port=f"{onvif_start}",
        rtsp_port=f"{rtsp_pt}",
		go2rtc_api_port=f"{api_pt}",
		rtmp_port=f"{rtmp_pt}"
        # web_port=8081,
	)
    csp = IPCamera(config=config)
    # Disable digital PTZ - change if you are using some ptz control for the cams
    csp.ptz.enable_digital_ptz = False
    csp.ptz.add_hardware_handler(PrintingPTZHandler())
    if not csp.start():
        print(f"Failed to start rtsp stream for camera {cam_no}")
        sys.exit(-1)
    print(f"  rtsp thread ptp camera {cam_no} ")
    print(f"  Web UI: http://{config.local_ip}:{config.onvif_port}/")
    print(f"  RTSP:   {config.main_stream_rtsp}")
    print("\nUse the web UI or an ONVIF client to control PTZ.")
    print("Hardware commands will be printed to the console.")
    print("\nPress Ctrl+C to stop\n")  
    cam_no += 1 
    onvif_start += 1
    rtsp_pt += 1    
    with camera.session():
        caminfo = camera.get_device_info()
        while not finished.is_set():
            event = camera.event()
            if event and event.EventCode == 'ObjectAdded':
                handle = event.Parameter[0]
                info = camera.get_object_info(handle)
                # Download all things that are not groups of other things.
                if info.ObjectFormat != 'Association':
                    log.info(
                        '{}: downloading {}'
                        .format(caminfo.SerialNumber, info.Filename)
                    )
                    tic = time()
                    obj = camera.get_object(handle)
                    toc = time()
                    log.info('{}: {:.1f}MB/s'.format(
                        caminfo.SerialNumber,
                        len(obj.Data) / ((toc - tic) * 1e6))
                    )
                    csp.stream(obj.Data)
    csp.stop()

def main():
    # Find each connected USB camera try to instantiate it and set up a capture and
    # download thread for it if successful.
    threads = []
    for i, device in enumerate(find_usb_cameras()):

        try:
            camera = ptpy.PTPy(device=device)
            info = camera.get_device_info()
            caminfo = (info.Manufacturer, info.Model, info.SerialNumber)
            if (
                    'InitiateCapture' not in info.OperationsSupported or
                    'GetObject' not in info.OperationsSupported
            ):
                raise Exception(
                    '{} {} {} does not support capture or download...'
                    .format(*caminfo)
                )
            log.info(
                'Found {} {} {}'
                .format(*caminfo)
            )
        except Exception as e:
            log.error(e)
            continue

        setup_cam(camera)

        capture = Thread(
            name='PHOTO{:02}'.format(i),
            target=capture_thread,
            args=(camera,)
        )
        threads.append(capture)

        if option == dwnload:
            download = Thread(
                name='DWNLD{:02}'.format(i),
                target=download_thread,
                args=(camera,)
            )
            threads.append(download)
        elif option == rtsp_st:    
            rtsp_stream = Thread(
                name='RTSP{:02}'.format(i),
                target=rtsp_thread,
                args=(camera,)
            )
            threads.append(rtsp_stream)

    for thread in threads:
        thread.start()

    # Let the threads run for 60 seconds.
    sleep(60)
    finished.set()

    # Wait for them to finish running.
    for thread in threads:
        if thread.is_alive():
            thread.join()

if __name__ == "__main__":
    main()

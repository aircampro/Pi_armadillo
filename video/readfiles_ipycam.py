#!/usr/bin/env python3
"""
From Video File Example ref:- https://github.com/olkham/IPyCam/blob/main/examples/video_file.py

Stream a directory of video files in a loop through IPyCam.

and create a proxy file for use with this http proxy server https://github.com/KazuCocoa/http_proxy/tree/main

for remote key exit
# https://www.ishikawasekkei.com/index.php/2019/10/21/python-programing-inkey/
# pip install blessed

"""
import time
import cv2
import sys
from ipycam import IPCamera, CameraConfig
from datetime import datetime
import yaml
from os import listdir
from os.path import isfile, join
import threading
from blessed import Terminal
import sys
import shlex
from subprocess import Popen, PIPE, STDOUT

# read the ipycam config yaml file
CONST_CONFIG_FILENAME = "/ipycam/go2rtc.yml"
PROXY_FILE = "config/config.exs"
youtube_urls = [ "https://www.youtube.com/watch?v=z70ZrSZNi-8", "https://www.youtube.com/watch?v=0IxXUwwkwt8", "https://www.youtube.com/watch?v=ArDgtO9vpvo"]

confg = None
with open(CONST_CONFIG_FILENAME, 'r') as yml:
	confg = yaml.safe_load(yml)
	print(confg)

# get the relevant port data from yaml config file
webrtc = confg['webrtc']['listen'],split(":")[1]
rtsp_pt = confg['rtsp']['listen'],split(":")[1]
api_pt = confg['api']['listen'],split(":")[1]
rtmp_pt = confg['rtmp']['listen'],split(":")[1]
onvif_start = 8079
redirect_port = 8079
redir_ports = []
http_urls = []

def main_stream(video_path: str):

    uniq = datetime.now().strftime(f'%Y%m%d%H%M%S')                             # create a unique id for the stream
    onvif_start += 1                                                            # for each increment the onvif port 

    # Create camera
    config = CameraConfig(name="My Custom Camera",
        manufacturer="MyCompany",
        model="CustomCam-Pro",
        serial_number=f"{uniq}",
        # Stream settings
        main_width=1280,
        main_height=720,
        main_fps=30,
        main_bitrate="2M",
        main_stream_name: str = f"video_main_{uniq}"        
        # Substream settings
        sub_width=640,
        sub_height=360,
        sub_bitrate="512K",
        sub_stream_name: str = f"video_sub_{uniq}"
        # Hardware acceleration
        hw_accel="auto",                                                          # Options: "auto", "nvenc", "qsv", "cpu"
        # Network ports (optional, uses defaults if not specified)
        onvif_port=f"{onvif_start}",
        rtsp_port=f"{rtsp_pt}",
		go2rtc_api_port=f"{api_pt}",
		rtmp_port=f"{rtmp_pt}"
        # web_port=8081,
	)
    camera = IPCamera(config)

    if not camera.start():
        print("Failed to start camera")
        return

    print(f"\n{'='*60}")
    print(f"  Streaming video file: {video_path}")
    print(f"{'='*60}")
    print(f"  Web UI:      http://{config.local_ip}:{config.onvif_port}/")
    print(f"  RTSP Stream: {config.main_stream_rtsp}")
	print(f"  Preview http://{config.local_ip}:{config.go2rtc_api_port}/stream.html?src={config.main_stream_name}")
    print(f"{'='*60}\n")
	# place the urls and ports in a list for using with the proxy file
    redirect_port += 1	
    redir_ports.append(redirect_port)
    http_urls.append(f"http://{config.local_ip}:{config.onvif_port}/")
    redirect_port += 1	
    redir_ports.append(redirect_port)
    http_urls.append(f"http://{config.local_ip}:{config.go2rtc_api_port}/stream.html?src={config.main_stream_name}")

    # Open video file
    cap = cv2.VideoCapture(video_path)
    time.sleep(1.0)

    if not cap.isOpened():
        print(f"Error: Could not open video file: {video_path}")
        camera.stop()
        return

    # Get video properties
    video_fps = cap.get(cv2.CAP_PROP_FPS)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Video: {video_fps:.1f} fps, {frame_count} frames\n")
    print("Press Ctrl+C to stop\n")

    frame_num = 0
    start_time = time.time()
    t = Terminal()                                                                              # read termianl key using curses lib
    with t.cbreak():
        try:
            while camera.is_running:                                                            # Loop video
                ret, frame = cap.read()
                if not ret:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    frame_num = 0
                    print("Looping video...")
                    continue
                # Add frame counter
                cv2.putText(frame, f"Frame {frame_num}/{frame_count} {frame_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                camera.stream(frame)
                frame_num += 1
                # Use video's original FPS for pacing
                target_frame_time = 1.0 / video_fps
                elapsed = time.time() - start_time
                sleep_time = (frame_num * target_frame_time) - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                k = t.inkey(timeout=0.001)
                if not k :
                    pass
                elif k.is_sequence:
                    if k.name == 'KEY_ESCAPE':
                        cap.release()
                        camera.stop()
                        cv2.destroyAllWindows()
                        break
                    print(f'"{k.name}"pressed ESC to exit')
                else:
                    print(f'"{k}"pressed「ESC to exit')					
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            cap.release()
            camera.stop()
            cv2.destroyAllWindows()

# this class wraps the subprocess for shell and pipe
# 
class Shell(object):
    def __init__(self):
        self.p  = None
    def cmd(self, cmd):
        p_args = {'stdin'     : None,
                  'stdout'    : PIPE,
                  'stderr'    : STDOUT,
                  'shell'     : False,}
        return self._execute(cmd, p_args)
    def pipe(self, cmd):
        p_args = {'stdin'     : self.p.stdout,
                  'stdout'    : PIPE,
                  'stderr'    : STDOUT,
                  'shell'     : False,}
        return self._execute(cmd, p_args)
    def _execute(self, cmd, p_args):
        try :
            self.p = Popen(shlex.split(cmd), **p_args)
            return self
        except :
            print('Command Not Found: %s' % (cmd))
    def commit(self):
        result = self.p.communicate()[0]
        status = self.p.returncode
        return (status, result)

def main():
    threadList = list()
    #mocapFbxPath = 'J:\\hx\\work\\prod\\mcp\\motiondata\\export'                          # make this your home video directory
    mocapFbxPath = '/work/prod/mcp/motiondata/export'
    onlyfiles = [f for f in listdir(mocapFbxPath) if isfile(join(mocapFbxPath, f))]
    for _file in onlyfiles:
        worker = threading.Thread(target=main_stream, args=(_file))
        worker.start()
        threadList.append(worker)
    time.sleep(10)                                                                         # wait after starting all the streams
    # header and footer of the proxy config file
    l0="use Mix.Config"
    l00="\n"
    l1="config :http_proxy,"
    l2="  proxies: ["
    l3="            ]"
    file = open(PROXY_FILE, 'w')                                                           # write the proxy file
    file.write(l0)
    file.write(l00)
    file.write(l1)
    file.write(l2)
    for z,u in enumerate(http_urls):                                                      # camera urls are re-mapped
        file.write(f"             %{port: {redir_ports[z]},")
        file.write(f"               to:   {u}},")
    pp = int(redir_ports[z])
    for u in youtube_urls:                                                                # add hard coded urls to proxy file
        pp += 1    
        file.write(f"             %{port: {pp},")
        file.write(f"               to:   {u}},")
    file.write(l3)
    file.close()
    shl = Shell()                                                                         # create a shell and run the procy server
    (return_code, stdout) = shl.cmd(f'mix deps.get').commit()
    time.sleep(10)
    (return_code, stdout) = shl.cmd(f'mix clean').commit()
    time.sleep(10)
    (return_code, stdout) = shl.cmd(f'mix run --no-halt &').commit()
    # wait for every thread finish its work
    for worker in threadList:
        worker.join()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Basic Screen capture Example

Demonstrates the simplest way to use IPyCam with a screen dumper. ref:- https://github.com/olkham/IPyCam/blob/main/examples
e.g. remote telemetry observer

"""
import time
import cv2
from ipycam import IPCamera, CameraConfig
from PIL import ImageGrab

# output stream window size
w=1280
h=720

def main():

    config = CameraConfig(
        name="Screenshot Content Camera",
        main_width=w,
        main_height=h,
        main_fps=30,
    )
    camera = IPCamera(config)
    if not camera.start():
        print("Failed to start camera")
        return 
    print(f"\n{'='*60}")
    print(f"  IPyCam is running!")
    print(f"{'='*60}")
    print(f"  Web UI:      http://{camera.config.local_ip}:{camera.config.onvif_port}/")
    print(f"  RTSP Stream: {camera.config.main_stream_rtsp}")
    print(f"  ONVIF:       {camera.config.onvif_url}")
    print(f"{'='*60}\n")
    print("Press Ctrl+C to stop\n")
    frame_count = 0

    try:
        while camera.is_running:
            frame = ImageGrab.grab()  
            frame = frame.resize((w,h))			
            camera.stream(frame)
            frame_count += 1
            # Print stats every 5 seconds
            if frame_count % (camera.config.main_fps * 5) == 0:
                if camera.stats:
                    print(f"Stats: {camera.stats.frames_sent} frames, "
                          f"{camera.stats.actual_fps:.1f} fps, "
                          f"{camera.stats.bitrate_mbps:.2f} Mbps")
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        camera.stop()
        print("Done!")

if __name__ == "__main__":
    main()
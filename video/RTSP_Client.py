#!/usr/bin/env python3
# ffplay rtsp://{username}:{password}@{camera_ip}:{port}/{path}
# ffprobe rtsp://username:password@camera_ip:port/path
# RTSP Client
#
import cv2

# RTSP URL 
username="root"
password="pass001"
camera_ip="10.0.5.6"

# common RTSP links for various manufacturers camera_ip
VID=0
PIC=1
channel_id=0
links = {'hikvision': ['Streaming/Channels/{channel_id}/', 'ISAPI/Streaming/channels/{channel_id}/picture'], 
'axis': ['axis-media/media.amp', 'axis-cgi/jpg/image.cgi'],
'rasppi': ['', 'snap?action=snapshot']}
# e.g. rtsp_url = links['hikvision'][VID]

if len(sys.argv) >= 1:
    rtsp_url = sys.argv[1]
else:
    rtsp_url = f"rtsp://{username}:{password}@{camera_ip}"

cap = cv2.VideoCapture(rtsp_url)

if not cap.isOpened():
    print("RTSP stream would not open closing..")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("cant get frame.. exiting")
        break

    cv2.imshow("RTSP Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
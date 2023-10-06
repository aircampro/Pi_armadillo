import cv2

video_capture = cv2.VideoCapture('sample.mov')

width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = video_capture.get(cv2.CAP_PROP_FPS)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height))

while (1):
    retval, frame = video_capture.read()
    if retval is False:
        break

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    video_writer.write(frame_gray)

cv2.destroyAllWindows()
video_capture.release()
video_writer.release()
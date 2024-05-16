#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ===========================================================================================
#
# when camera action detected to camera move the gripper robot over ROS
#
# ===========================================================================================
'''
Yolo and face cascade classifier that then controls a robot gripper with a random action set when activity is found on camera feed
'''

# open CV
import cv2, time

# import argparse and open CV
import argparse

# load the YOLO class
from yolo import YOLO_Class

# load the rospy communication task to control the robot
from rospy_gripper import move_and_collect_data

# function to do the frame analysis and action on the gripper via ROS class
#
def do_analysis(typ, frame, conf_skip=True):

    if typ == "cas":                                                             # we are using cascade to look for faces
        # convert to grayscale the image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist( gray )
        faces = faceCascade.detectMultiScale(gray, 1.1, 3, 0, (10, 10))
        # for each face seen perform a gesture on the robot
        for (x, y, w, h) in faces:
            # call the robot gripper motion control class
            try:
                move_and_collect_data()
            except rospy.ROSInterruptException:
                pass      
            # draw the rectangle
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        if len(faces[0]) > 0:
            # show the frames 
            frame = cv2.resize(frame, (FRAME_DW,FRAME_DH))
            cv2.imshow('~~Cascade~~', frame)
            writer.write(frame)                       # write the output frame to disk                                                        
    elif typ == "yo8detect" or typ == "yo8classify" or typ == "yo8seg" or typ == "yo8pose": 
       # Predict the model
       results = model.predict(frame, save=True, conf=0.5)
       # get predict result
       for result in results:
            boxes = result.boxes                       # Boxes object for bbox outputs
            masks = result.masks                       # Masks object for segmentation masks outputs
            names = result.names
            orig_img = result.orig_img
            orig_shape = result.orig_shape
            speed = result.speed
            # call the robot gripper motion control class
            try:
                move_and_collect_data()
            except rospy.ROSInterruptException:
                pass               
        # Display the annotated frame
        if len(results[0]) > 0:
            frame = cv2.resize(frame , (FRAME_DW,FRAME_DH))
            cv2.imshow("~~YOLOv8~~", frame)
            writer.write(frame)                                                         # write the output frame to disk	
    elif typ == "yo8custom": 
       # Predict the model
       results = results = model.track(source=frame, show=False, save=True, device=0, conf=0.5, save_txt=True, save_conf=True, tracker="bytetrack.yaml")
       # get predict result
       for result in results:
            boxes = result.boxes                       # Boxes object for bbox outputs
            masks = result.masks                       # Masks object for segmentation masks outputs
            names = result.names
            orig_img = result.orig_img
            orig_shape = result.orig_shape
            speed = result.speed
            # call the robot gripper motion control class
            try:
                move_and_collect_data()
            except rospy.ROSInterruptException:
                pass               
        # Display the annotated frame
        if len(results[0]) > 0:
            frame = cv2.resize(frame , (FRAME_DW,FRAME_DH))
            cv2.imshow("~~YOLOv8 custom~~", frame)
            writer.write(frame)                                                         # write the output frame to disk            
    else:                                                                               # we are using yolo
        width, height, inference_time, results = yolo.inference(frame)
        for detection in results:
            l = int(len(results))
            print(l)
            id, name, confidence, x, y, w, h = detection
            print(confidence)
            if (confidence < 0.3) and (conf_skip == False):                            # defaulted off and set default 0.5 in class
               continue
            
            # call the robot gripper motion control class
            try:
                move_and_collect_data()
            except rospy.ROSInterruptException:
                pass
                
            cx = x + (w / 2)
            cy = y + (h / 2)

            # draw a bounding box rectangle and label on the image
            color = (0, 255, 255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = "%s (%s)" % (name, round(confidence, 2))
            cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        if len(results[0]) > 0:
            frame = cv2.resize(frame, (FRAME_DW,FRAME_DH))
            cv2.imshow("~~Yolo~~", frame) 
            writer.write(frame)                                                         # write the output frame to disk 
        
# set the camera frame size
FRAME_W = 320
FRAME_H = 240

# raspberry pi cam frame rate
RASPI_FRAME_RATE = 32

# set the display frame size
FRAME_DW = 540
FRAME_DH = 300

writer = None
    
if __name__ == '__main__': 
       
    # parse the command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', '--network', default="normal", help='Network Type: normal / tiny / prn / cas / yo8detect / yo8custom / yo8classify / yo8seg / yo8pose')
    ap.add_argument('-cam', '--camera', default="pi", help='Camera Type: pi / webcam')
    ap.add_argument('-s', '--size', default=416, help='Size for yolo')
    ap.add_argument('-c', '--confidence', default=0.2, help='Confidence for yolo')
    ap.add_argument('-co', '--coco', default=0, help='convert coco for the custom set')
    ap.add_argument('-o', '--output', default="output.mp4", help='name for the output file')
    args = ap.parse_args()

    # choose the detection method
    if args.network == "normal":
        # to get datamodel for yolov3
        # wget https://github.com/cansik/yolo-hand-detection/releases/download/pretrained/cross-hands.cfg
        # wget https://github.com/cansik/yolo-hand-detection/releases/download/pretrained/cross-hands.weights
        print("\007 loading yolo...")
        yolo = YOLO_Class("models/cross-hands.cfg", "models/cross-hands.weights", ["hand"])
        yolo.size = int(args.size)
        yolo.confidence = float(args.confidence)
    elif args.network == "prn":
        # to get datamodel for yolo
        # wget https://github.com/cansik/yolo-hand-detection/releases/download/pretrained/cross-hands-tiny.cfg
        # wget https://github.com/cansik/yolo-hand-detection/releases/download/pretrained/cross-hands-tiny.weights
        print("\007 loading yolo-tiny-prn...")
        yolo = YOLO_Class("models/cross-hands-tiny-prn.cfg", "models/cross-hands-tiny-prn.weights", ["hand"])
        yolo.size = int(args.size)
        yolo.confidence = float(args.confidence)
    elif args.network == "cas":
        # get datamodel for cascade
        # wget https://raw.githubusercontent.com/Itseez/opencv/master/data/lbpcascades/lbpcascade_frontalface.xml
        # set the file path for cascade classifier
        print("\007 loading cascade...")
        cascPath = 'lbpcascade_frontalface.xml'
        faceCascade = cv2.CascadeClassifier(cascPath)
    elif args.network == "yo8detect":                   # git clone https://github.com/ultralytics/yolov5; cd yolov5 ; pip install -r requirements.txt
        # pip3 install ultralytics
        # pip3 install torch torchvision torchaudio
        import torch
        from ultralytics import YOLO
        # Load a model
	    # detect: yolov8n.pt yolov8s.pt yolov8m.pt yolov8l.pt yolov8x.pt
        # segment: yolov8s-seg.pt yolov8m-seg.pt yolov8l-seg.pt yolov8x-seg.pt
        # pose: yolov8s-pose.pt yolov8m-pose.pt yolov8l-pose.pt yolov8x-pose.pt yolov8x-pose-p6
        # classification: yolov8s-cls.pt yolov8m-cls.pt yolov8l-cls.pt yolov8x-cls.pt
        print("\007 loading Yolo v8 detect...")
        model = YOLO('yolov8n.pt')
    elif args.network == "yo8classify":
        # pip3 install ultralytics
        # pip3 install torch torchvision torchaudio
        import torch
        from ultralytics import YOLO
        # Load a model
	    # detect: yolov8n.pt yolov8s.pt yolov8m.pt yolov8l.pt yolov8x.pt
        # segment: yolov8s-seg.pt yolov8m-seg.pt yolov8l-seg.pt yolov8x-seg.pt
        # pose: yolov8s-pose.pt yolov8m-pose.pt yolov8l-pose.pt yolov8x-pose.pt yolov8x-pose-p6
        # classification: yolov8s-cls.pt yolov8m-cls.pt yolov8l-cls.pt yolov8x-cls.pt
        print("\007 loading Yolo v8 classify...")
        model = YOLO('yolov8n-cls.pt')
    elif args.network == "yo8seg":
        # pip3 install ultralytics
        # pip3 install torch torchvision torchaudio
        import torch
        from ultralytics import YOLO
        # Load a model
	    # detect: yolov8n.pt yolov8s.pt yolov8m.pt yolov8l.pt yolov8x.pt
        # segment: yolov8s-seg.pt yolov8m-seg.pt yolov8l-seg.pt yolov8x-seg.pt
        # pose: yolov8s-pose.pt yolov8m-pose.pt yolov8l-pose.pt yolov8x-pose.pt yolov8x-pose-p6
        # classification: yolov8s-cls.pt yolov8m-cls.pt yolov8l-cls.pt yolov8x-cls.pt
        print("\007 loading Yolo v8 segmentation...")
        model = YOLO('yolov8n-seg.pt')
    elif args.network == "yo8pose":
        # pip3 install ultralytics
        # pip3 install torch torchvision torchaudio
        import torch
        from ultralytics import YOLO
        # Load a model
	    # detect: yolov8n.pt yolov8s.pt yolov8m.pt yolov8l.pt yolov8x.pt
        # segment: yolov8s-seg.pt yolov8m-seg.pt yolov8l-seg.pt yolov8x-seg.pt
        # pose: yolov8s-pose.pt yolov8m-pose.pt yolov8l-pose.pt yolov8x-pose.pt YOLOv8x-pose-p6
        # classification: yolov8s-cls.pt yolov8m-cls.pt yolov8l-cls.pt yolov8x-cls.pt
        print("\007 loading Yolo v8 pose...")
        model = YOLO('yolov8n-pose.pt')
    elif args.network == "yo8custom":
        # pip3 install ultralytics
        # pip3 install torch torchvision torchaudio
        import torch
        from ultralytics import YOLO
        # Load a model
	    # detect: yolov8n.pt yolov8s.pt yolov8m.pt yolov8l.pt yolov8x.pt
        # segment: yolov8s-seg.pt yolov8m-seg.pt yolov8l-seg.pt yolov8x-seg.pt
        # pose: yolov8s-pose.pt yolov8m-pose.pt yolov8l-pose.pt yolov8x-pose.pt yolov8x-pose-p6
        # classification: yolov8s-cls.pt yolov8m-cls.pt yolov8l-cls.pt yolov8x-cls.pt
        model = YOLO('yolov8n.pt')
        if args.coco == 1:
            from ultralytics.data.converter import convert_coco
            convert_coco(labels_dir='.\annotations\instances_val2017')
            # Train the model to your customn labeled dataset in yaml
            results = model.train(data=r"coco.yaml", epochs=10, batch=1, device=0)
        else:
            # Train the model to your customn labeled dataset in yaml
            results = model.train(data=r"data/data.yaml", epochs=100, batch=12, device=0)
        # Evaluate the model's performance on the validation set
        results = model.val()
        print(results)
        # Export the model
        model.export(format="torchscript")
        time.sleep(0.1)
        # Load new model
        print("\007 loading Yolo v8 custom labelled dataset...")
        # The optimal learning model best.pt and the final learning model last.pt was saved by above
        model = YOLO('runs/detect/train/weights/best.pt')
    else:
        # to get datamodel for yolo
        # wget https://github.com/cansik/yolo-hand-detection/releases/download/pretrained/cross-hands-tiny-prn.cfg
        # wget https://github.com/cansik/yolo-hand-detection/releases/download/pretrained/cross-hands-tiny-prn.weights
        print("\007 loading yolo-tiny...")
        yolo = YOLO_Class("models/cross-hands-tiny.cfg", "models/cross-hands-tiny.weights", ["hand"])
        yolo.size = int(args.size)
        yolo.confidence = float(args.confidence)

	if (writer is None):
		fourcc = cv2.VideoWriter_fourcc(*"MJPG")                 # initialize our video writer
		writer = cv2.VideoWriter(args.output, fourcc, 30, (frame.shape[1], frame.shape[0]), True)
            
    if args.camera == "pi":
        # library for controlling the camera
        from picamera.array import PiRGBArray
        from picamera import PiCamera

        # rasperry pi cam is used
        camera = PiCamera()
        camera.resolution = (FRAME_W, FRAME_H)
        camera.framerate = RASPI_FRAME_RATE
        rawCapture = PiRGBArray(camera, size=(FRAME_W, FRAME_H))
        time.sleep(0.1)
        # for raspberry pi cam loop for each frame until end pressed
        for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            frame = image.array
            do_analysis(args.network, frame)                      # analyse the frame and do the action        
            key = cv2.waitKey(1) & 0xFF
            rawCapture.truncate(0)
            if key == ord("q"):
                break
        writer.release()
    else:
        # std web cam is used
        vc = cv2.VideoCapture(0)                                   # start the webcam capture stream
        fps = int(vc.get(cv2.CAP_PROP_FPS))
        w = int(vc.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(vc.get(cv2.CAP_PROP_FRAME_HEIGHT))
        brightness = int(vc.get(cv2.CAP_PROP_BRIGHTNESS))
        contrast = int(vc.get(cv2.CAP_PROP_CONTRAST))
        auto_exposure = int(vc.get(cv2.CAP_PROP_AUTO_EXPOSURE))
        auto_focus = int(vc.get(cv2.CAP_PROP_AUTOFOCUS))
        auto_WB = int(vc.get(cv2.CAP_PROP_AUTO_WB))
        fourcc = int(vc.get(cv2.CAP_PROP_FOURCC))
        print("frames per second = ",fps," w = ",w," h = ",h," brightness = ",brightness)
        print("contrast = ",contrast," auto exposure = ",auto_exposure," auto focus = ",auto_focus," white balance = ",auto_WB)        
        if vc.isOpened():                                          # try to get the first frame
            rval, frame = vc.read()       
        else:
            rval = False
        time.sleep(0.1)        
        while rval:                                               # while we have frames
            do_analysis(args.network, frame)                      # analyse the frame and do the action      
            rval, frame = vc.read()                               # readthe next frame from the video stream
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        writer.release()
        vc.release()        
        
    cv2.destroyAllWindows()                                       # close all windows

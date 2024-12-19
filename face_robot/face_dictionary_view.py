#!/usr/bin/env python
#
# Raspberry Pi with pan tilt hat and camera
# using onyx dataset to recognize the faces put in the dictionary
#
import cv2
import os
import glob
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
import streamlit as st
from streamlit_server_state import server_state, server_state_lock
# camera on Pimeroni :: Pan Tilt Hat  https://github.com/pimoroni/pantilt-hat
import pantilthat
pantilthat.light_mode(pantilthat.WS2812)
pantilthat.light_type(pantilthat.GRBW)
r, g, b, w = 0, 80, 40, 0

COSINE_THRESHOLD = 0.363
NORML2_THRESHOLD = 1.128

GLOB_PATH = 'mark/sample_data/*.npy'
FACE_DETECTOR_WEIGHTS = 'mark/onnx/yunet_n_640_640.onnx'
FACE_RECOGNIZER_WEIGHTS = 'mark/onnx/face_recognizer_fast.onnx'

def match(recognizer, feature1, dictionary) -> tuple[bool,set[str,float]]:
    for element in dictionary:
        user_id, feature2 = element
        score = recognizer.match(feature1, feature2, cv2.FaceRecognizerSF_FR_COSINE)
        if score > COSINE_THRESHOLD:
            return True, (user_id, score)
    return False, ("", 0.0)

def get_camera() -> Picamera2:
    with server_state_lock["camera"]:
        if "camera" not in server_state:
            camera = Picamera2()
            camera.configure(camera.create_preview_configuration(main={
                "format": 'XRGB8888',
                "size": (640, 480)
            }))
            camera.start()
            camera.set_controls({'AfMode': controls.AfModeEnum.Continuous})
            server_state.camera = camera
    return server_state.camera

def recognize_face(face_recognizer, dictionary, image, face) -> np.ndarray:

    aligned_face = face_recognizer.alignCrop(image, face)
    feature = face_recognizer.feature(aligned_face)

    result, user = match(face_recognizer, feature, dictionary)

    box = list(map(int, face[:4]))
    color = (0, 255, 0) if result else (0, 0, 255)
    thickness = 2
    cv2.rectangle(image, box, color, thickness, cv2.LINE_AA)

    id, score = user if result else ("unknown", 0.0)
    text = "{0} ({1:.2f})".format(id, score)
    position = (box[0], box[1] - 10)
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.6
    cv2.putText(image, text, position, font, scale, color, thickness, cv2.LINE_AA)

    return image

def main():

    for x in range(18):
        pantilthat.set_pixel(x, r, g, b, w)

    p, t = 0, 0
    pantilthat.pan(p)
    pantilthat.tilt(t)
    
    st.markdown("# Face Recognition Application")

    dictionary = []
    files = glob.glob(GLOB_PATH)
    for file in files:
        feature = np.load(file)
        user_id = os.path.splitext(os.path.basename(file))[0]
        dictionary.append((user_id, feature))

    face_detector = cv2.FaceDetectorYN_create(FACE_DETECTOR_WEIGHTS, "", (0, 0))
    face_recognizer = cv2.FaceRecognizerSF_create(FACE_RECOGNIZER_WEIGHTS, "")
    
    camera = get_camera()

    MAX_PAN = 360
    MAX_TLT = 90
    
    try:
        image_loc = st.empty()
        mode = 0
        while True:
            image = camera.capture_array()

            channels = 1 if len(image.shape) == 2 else image.shape[2]
            if channels == 1:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            if channels == 4:
                image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            
            height, width, _ = image.shape
            face_detector.setInputSize((width, height))

            result, faces = face_detector.detect(image)
            faces = faces if faces is not None else []
  
            if faces == []:
                if mode == 0:
                    p += 10
                    mode = 1
                elif mode == 1:
                    t += 5
                if p == MAX_PAN:
                    p = 0
                if t == MAX_TLT:
                    t = 0 
                    mode = 0                    
                pantilthat.pan(p)
                pantilthat.tilt(t)
    
            for face in faces:
                image = recognize_face(face_recognizer, dictionary, image, face)

            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            image_loc.image(image)

    except KeyboardInterrupt:
        print('app stop pressed!!')
    except Exception as e:
        st.markdown("### Error。")
        st.markdown(f'{e}')

    camera.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
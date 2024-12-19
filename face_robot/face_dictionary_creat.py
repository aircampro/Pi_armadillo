#!/usr/bin/env python
# 
# face_dictionary_creat.py
# generate face recognizer dictionary
#
import os
import sys
import argparse
import numpy as np
import cv2

def main():

    parser = argparse.ArgumentParser("generate face feature dictionary from an face image")
    parser.add_argument("image", help="input face image file path (./<face名>.jpg)")
    args = parser.parse_args()
    print(args.image)

    path = args.image
    directory = os.path.dirname(args.image)
    if not directory:
        directory = os.path.dirname(__file__)
        path = os.path.join(directory, args.image)

    image = cv2.imread(path)
    if image is None:
        exit()

    channels = 1 if len(image.shape) == 2 else image.shape[2]
    if channels == 1:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if channels == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    weights = "onnx/face_recognizer_fast.onnx"
    face_recognizer = cv2.FaceRecognizerSF_create(weights, "")

    face_feature = face_recognizer.feature(image)
    print(face_feature)
    print(type(face_feature))

    basename = os.path.splitext(os.path.basename(args.image))[0]
    dictionary = os.path.join(directory, basename)
    numpy_dir = 'mark/sample_data/'
    np.save(numpy_dir, face_feature)

if __name__ == '__main__':
    main()
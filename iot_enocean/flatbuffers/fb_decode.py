#!/usr/bin/env python3
# example of decoding flatbuffers for sony camera
# 
import base64
from fbs_test import objectdetection_generated as OD

if __name__ == '__main__':
    #ir_data = inference_list[0]["O"]
    # this is a test replace this with the actual data
    ir_data = 'DAAAAAAABgAKAAQABgAAAAwAAAAAAAYACAAEAAYAAAAEAAAAAgAAAEgAAAAQAAAADAAUAAgABwAMABAADAAAAAAAAAEBAAAACAAAAAAAZj/Q////YwAAAFUAAACtAAAAEQEAAAwAEAAAAAcACAAMAAwAAAAAAAABFAAAAAAAfz8MABQABAAIAAwAEAAMAAAArAAAAFgAAAD+AAAAwwAAAA=='
    print(ir_data)

    ir_data_dec = base64.b64decode(ir_data)
    ir_data_obj = OD.ObjectDetectionTop.GetRootAsObjectDetectionTop(ir_data_dec,0)
    ir_Score = ir_data_obj.Perception().ObjectDetectionList(0).Score()
    print("Score:", ir_Score)

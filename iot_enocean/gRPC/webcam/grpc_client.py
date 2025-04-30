# grpc_client.py
# gRPC Client for video stream
#
import socket
import sys

import cv2
import grpc
import numpy as np
import base64

import webcam_streaming_pb2
import webcam_streaming_pb2_grpc

class GrpcClient:
    def __init__( self, connect_ip: str, port: str = "50051", b: bool =False):
        super(GrpcClient, self).__init__()

        # connect to the gRPC server 
        channel = grpc.insecure_channel(f"{connect_ip}:{port}")

        self.__stub = webcam_streaming_pb2_grpc.WebcamStreamingStub(channel)
        self.b64 = b
        
    def start_webcam_streaming(self):
        print("gRPC client: webcam streaming Start")

        # clientName is this pcs name
        request = webcam_streaming_pb2.ClientName( clientName=socket.gethostname() )

        responses = self.__stub.StartWebcamStreaming(request)

        for response in responses:
            frame_byte = response.imageBytes
            if self.b64 == True:
                b64d = base64.b64decode(frame_byte)
                frame_byte = b64d
                
            # サーバから受け取ったバイト列を一次元配列として解釈
            data_array = np.frombuffer(frame_byte, dtype=np.uint8)

            # 3チャネルのBGRカラーイメージに変換
            image = cv2.imdecode(data_array, cv2.IMREAD_COLOR)

            cv2.imshow("Received Frame", image)

            cv2.waitKey(1)


if __name__ == "__main__":
    # fisr arg is the ip to conenct to i.e the gRPC server
    # loopback 127.0.0.1
    argv = sys.argv
    connect_ip = argv[1]

    client = GrpcClient(connect_ip)
    client.start_webcam_streaming()
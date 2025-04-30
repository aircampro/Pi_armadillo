# grpc_server.py
# Example of gRPC streaming server
#
from concurrent import futures
import grpc
from webcam_streaming_server import WebcamStreamingService
import webcam_streaming_pb2_grpc

# default port number
p_num=50051

def serve(port_num=p_num):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    try:
        webcam_streaming_pb2_grpc.add_WebcamStreamingServicer_to_server( WebcamStreamingService(), server )
        server.add_insecure_port(f"[::]:{port_num}")
        server.start()
        server.wait_for_termination()
    except KeyboardInterrupt:
        server.stop(0)	
        
if __name__ == "__main__":
    serve()
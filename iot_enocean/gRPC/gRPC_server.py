# pip install grpcio grpcio-tools
#
import grpc
from concurrent import futures
import example_pb2
import example_pb2_grpc

# program Globals to be changed over the gRPC
#
gInputVal = 0
gOutputVal = 0
gTrendMode = 0

class Greeter(example_pb2_grpc.GreeterServicer):
    def gRPCCmdLine(self, request, context):
        global gInputVal, gOutputVal
        try:        
            cmd = str(request.name).split(":")[0]
            val = str(request.name).split(":")[1]
            if cmd == "ADD":
                gInputVal += float(val)
                gTrendMode = 0
            elif cmd == "SUB":
                gInputVal -= float(val)
                gTrendMode = 0
            elif cmd == "REPLACE":
                gInputVal = float(val)
                gTrendMode = 0
            elif cmd == "BOTH":
                gInputVal = float(val)
                gOutputVal = float(str(request.name).split(":")[2])
                gTrendMode = 0
            elif cmd == "SYS":
                gTrendMode = 1		
        except Exception as e:
            print("error occurred : ",e)
        return example_pb2.gRPCReply(message=f"You sent to gRPC server, {request.name}!")

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    example_pb2_grpc.add_GreeterServicer_to_server(Greeter(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()

if __name__ == '__main__':
    serve()
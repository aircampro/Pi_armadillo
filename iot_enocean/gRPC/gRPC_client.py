# example of gRPC client
#
import grpc
import example_pb2
import example_pb2_grpc
import time

# update the global variables in the gRPC server and control the trend
def run():
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = example_pb2_grpc.GreeterStub(channel)
        response = stub.gRPCCmdLine(example_pb2.gRPCRequest(command='ADD : 3'))
        print("Greeter client received: " + response.message)
        time.sleep(0.5)
        response = stub.gRPCCmdLine(example_pb2.gRPCRequest(command='BOTH : 13 : 43'))
        print("Greeter client received: " + response.message)
        time.sleep(0.2)
        response = stub.gRPCCmdLine(example_pb2.gRPCRequest(command='BOTH : 16 : 13'))
        print("Greeter client received: " + response.message)
        time.sleep(0.2)
        response = stub.gRPCCmdLine(example_pb2.gRPCRequest(command='BOTH : 17 : 74'))
        print("Greeter client received: " + response.message)
        time.sleep(0.2)
        response = stub.gRPCCmdLine(example_pb2.gRPCRequest(command='BOTH : 23 : 4.3'))
        print("Greeter client received: " + response.message)
        time.sleep(0.2)
        response = stub.gRPCCmdLine(example_pb2.gRPCRequest(command='SYS'))
        print("Greeter client received: " + response.message)
        time.sleep(0.2)
		
if __name__ == '__main__':
    run()
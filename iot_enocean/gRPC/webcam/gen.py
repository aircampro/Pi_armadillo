# gen.py - compiles the protofile for google protobuf
# to run $>python gen.py
#
from grpc.tools import protoc

protoc.main(
    (
        "",
        "-I.",
        "--python_out=.",  
        "--grpc_python_out=.",  
        "./webcam_streaming.proto",  
    )
)
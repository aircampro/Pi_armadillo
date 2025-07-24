# CMakeLists.txt
# gcc++ sample.cpp -o sample -I/usr/include/opencv4/opencv -I/usr/include/opencv4 -L/usr/local/lib 
# -lopencv_dnn -lopencv_gapi -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video 
# -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core 
# -lstdc+
project(opencv_screenwriter)

cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set (CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -march=native")
set (CMAKE_BUILD_TYPE Debug)

find_package (OpenCV REQUIRED)
#find_package (OpenCV 3.0 REQUIRED)
include_directories(
 include
 ${OpenCV_INCLUDE_DIRS} 
)

add_executable(opencv_screenwriter/opencv_screenwriter.cpp)
# also was referred to as OpenCV_LIBRARIES
target_link_libraries(opencv_screenwriter ${OpenCV_LIBS})
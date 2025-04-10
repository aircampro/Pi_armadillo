# camera being used is :- 
# Arducam 5MP OV5647 Camera Module with CS mount Lens for Raspberry Pi 5/4/3B+/3 (NoIR)
# https://www.arducam.com/product/arducam-5mp-cs-mount-noir-camera-raspberry-pi-b0036/
# 
# Night/Low Light vision
# Wildlife research
# Science projects
# Night time security
# Making artistic photo, etc
#
# yolov10 image detection with rock5 and hailo PCIe AI accelerator
# or image detection using Nvdia jetson nano please comment in IS_JETSON
#
# for jetson libs
#
#$ sudo apt-get update
#$ sudo apt-get install git cmake libpython3-dev python3-numpy
#$ git clone --recursive https://github.com/dusty-nv/jetson-inference
#$ cd jetson-inference
#$ mkdir build
#$ cd build
#$ cmake ../
#$ make -j$(nproc)
#$ sudo make install
#$ sudo ldconfig
# 
# for openCV and gstreamer
#sudo apt-get install libx264-dev libjpeg-dev
#
#sudo apt-get install libgstreamer1.0-dev \
#     libgstreamer-plugins-base1.0-dev \
#     libgstreamer-plugins-bad1.0-dev \
#     gstreamer1.0-plugins-ugly \
#     gstreamer1.0-tools \
#     gstreamer1.0-gl \
#     gstreamer1.0-gtk3
#
#sudo apt-get install gstreamer1.0-qt5
#sudo apt-get install gstreamer1.0-pulseaudio
#
#sudo apt-get update &&\
#sudo apt-get install -y --no-install-recommends \
#        cmake \
#        libeigen3-dev \
#        libgflags-dev \
#        libgoogle-glog-dev \
#        libatlas-base-dev \
#        libsuitesparse-dev \
#        g++ \
#        ninja-build \
#        nano
#
# GStreamer
#sudo apt-get install -y --no-install-recommends libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
#
#pip uninstall --yes opencv-python opencv-contrib-python
#sudo rm -r /usr/lib/python3/dist-packages/cv2.cpython-310-aarch64-linux-gnu.so
#sudo rm -r /usr/local/lib/python3.10/dist-packages/cv2
#
#cd /tmp
#mkdir opencv && cd opencv &&\
#	git clone -b 4.x https://github.com/opencv/opencv.git &&\
#	git clone -b 4.x https://github.com/TriOrb-Inc/opencv_contrib.git &&\
#	mkdir build
#
#cd /tmp/opencv/build
#cmake ../opencv \
#        -GNinja \
#        -D OPENCV_EXTRA_MODULES_PATH='../opencv_contrib/modules' \
#        -D CMAKE_BUILD_TYPE=Release \
#        -D BUILD_EXAMPLES=ON \
#        -D BUILD_OPENCV_PYTHON2=OFF \
#        -D BUILD_OPENCV_PYTHON3=ON \
#        -D PYTHON3_EXECUTABLE=$(which python) \
#        -D PYTHON3_INCLUDE_DIRS=$(python3 -c "from sysconfig import get_paths as gp; print(gp()[\"include\"])") \
#        -D PYTHON3_NUMPY_INCLUDE_DIRS=$(python3 -c "import numpy; print(numpy.get_include())") \
#        -D BUILD_TESTS=OFF \
#        -D BUILD_ZLIB=ON \
#        -D WITH_GSTREAMER=ON \
#        -D SUITESPARSE:BOOL=ON \
#        -D Ceres_DIR=/tmp/ceres-solver/build/lib/cmake/Ceres
#
# ninja
# sudo ninja install
#
#
import cv2
import sys

# any arg other than a 0 will use jetson otherwise use rock5 with Hailo PCIe AI excellerator
#
if (len(sys.argv) > 1) and (not int(sys.argv[1]) == 0):
    IS_JETSON=True
else:   
    IS_JETSON=False

if IS_JETSON == True:
    import jetson.inference
    import jetson.utils
else:
    from HailoYOLODetector import HailoYOLODetector

# nvidia jetson pipeline
def nv_gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

#rock5 pipeline
#"gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,format=NV12,width=1920,height=1080,framerate=30/1 ! videoconvert ! appsink"
def rock5_gstreamer_pipeline(
    capture_width=640,
    capture_height=480,
    display_width=640,
    display_height=480,
    framerate=60,
):
    return (
        "gst-launch-1.0 v4l2src device=/dev/video0 ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            display_width,
            display_height,
        )
    )

# check the nvidia detections and label the frame
def nv_check_detection(img, detections, search_for=None, multiple=0):
    found_datas = []
    for d in detections:
        x1, y1, x2, y2 = int(d.Left), int(d.Top), int(d.Right), int(d.Bottom)
        className = net.GetClassDesc(d.ClassID)
        if className:
            #print("detected {:d} objects in image".format(len(detections)))
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.putText(img, className, (x1+5, y1-15), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255, 0, 255), 2)
        if not search_for == None:                                               # we have specified a search object
            if not str(className).find(search_for) == -1:
                found_data = [ className, x1, y1, x2, y2 ]
                if multiple == 0:
                    break
                else:
                    found_datas.append(found_data)
       if search_for == None:
           return img
       elif multiple == 0:
           return img, found_data
       else:
           return img, found_datas

# main code            
def main():

    global IS_JETSON
    # cap = cv2.VideoCapture(1) - for usb camera

    if IS_JETSON == True:
        net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
        cap = cv2.VideoCapture(nv_gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    else:
        hef_path = 'models/yolov10x.hef'                                             # Update with the correct path to your HEF file
        detector = HailoYOLODetector(hef_path, look_for="traffic light")
        cap = cv2.VideoCapture(rock5_gstreamer_pipeline(), cv2.CAP_GSTREAMER)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            if IS_JETSON == True:
                imgCuda = jetson.utils.cudaFromNumpy(frame)   
                detections = net.Detect(imgCuda)
                if len(detections) > 0 : 
                    bbox_image, look_for_data = nv_check_detection(frame, detections, "traffic light")
            else: 
                bbox_image, look_for_data = detector.detect(frame)
            #print(f"found {look_for_data}")
            cv2.imshow('detections result', bbox_image)
            key = cv2.waitKey(1) & 0xff
            if key == ord('q'):
                break
      
    except Exception as e:
        print(e)
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
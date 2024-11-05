/*
//
//       Use v4l2 to grab frames and write them jpg files 
//       Also try to detect road on sample video frames
//
// ref:- https://github.com/yutyan0119/v4l2_createImage/blob/main/v4l2sample.c
//       https://zenn.dev/turing_motors/articles/programming-v4l2
//      command line parser library https://github.com/tanakh/cmdline
//
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h> 
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <poll.h>
#include <linux/videodev2.h>

#include "cmdline.h"
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

struct buffer {
    void* start;
    size_t length;
};

/* wrapper around ioctrl command
*/
static int xioctl(int fd, int request, void *arg) {
    int r;
    do r = ioctl(fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

/*global variables*/
/*buffer pointer struct*/
struct buffer *buffers;
/*video file descriptor handle */
int _fdv4l2;
/*the number of buffers*/
int n_buffers;

/* open video driver */
void open_device( const char* video_dev ) {
    _fdv4l2 = open(video_dev, O_RDWR);
    if (_fdv4l2 == -1){
        perror("device open");
    }
}

/* Check if your device supports the method you want to use */
void cap_device() {
    struct v4l2_capability cap;
    if (-1 == xioctl(_fdv4l2,VIDIOC_QUERYCAP,&cap)){
        perror("query capability");
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		perror("no video capture");
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)){
		perror("does not support stream");
    }
}

/* configure device */
void set_device( int w, int h) {
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = w;
    fmt.fmt.pix.height = h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    /* set format */
    if (-1 == xioctl(_fdv4l2, VIDIOC_S_FMT, &fmt)){
        perror("Setting Pixel Format");
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /* get format */
    if (-1 == xioctl(_fdv4l2, VIDIOC_G_FMT,&fmt)){
            perror("get format");
    }
    if (fmt.fmt.pix.width != 848 || fmt.fmt.pix.height != 480 || fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG){
        printf("The desired format is not supported\n");
    }
}

/* Request a buffer to the device */
void request_buffer(){
    struct v4l2_requestbuffers req = {0};
    req.count = 3;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(_fdv4l2,VIDIOC_REQBUFS,&req)){
        perror("Requesting Buffer");
    }
    n_buffers = req.count;
    if (req.count < 3) {
        fprintf(stderr, "Insufficient buffer memory on camera\n");
    }
}

/* Mapping Buffer */
void map_buffer(){
    buffers = (struct buffer*)calloc(n_buffers, sizeof(*buffers));
    if (buffers == NULL){
        printf("okasii");
    }
    for (int n_buffer = 0; n_buffer < n_buffers; n_buffer++){
        struct v4l2_buffer buff = {0};
        buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buff.memory = V4L2_MEMORY_MMAP;
        buff.index = n_buffer;
        if (-1 == xioctl(_fdv4l2,VIDIOC_QUERYBUF,&buff)){
            perror("Querying Buffer");
        }
        buffers[n_buffer].length = buff.length;
        buffers[n_buffer].start = mmap (NULL, buff.length, PROT_READ | PROT_WRITE , MAP_SHARED, _fdv4l2, buff.m.offset);
    }
}

/* Enqueue data to the device's buffer */
void enqueue_buffer(int index){
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    if (-1 == xioctl(_fdv4l2, VIDIOC_QBUF, &buf))
        perror("VIDIOC_QBUF");
}

void enqueue_buffers(){
    for (int n_buffer = 0; n_buffer < n_buffers; n_buffer++){
        enqueue_buffer(n_buffer);
    }
}

/* Give instructions that allow output from the device */
void stream_start(){
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(_fdv4l2, VIDIOC_STREAMON, &type)){
        perror("VIDIOC_STREAMON");
    }
}

void set_exposure(int expos, int mode) {
    struct v4l2_queryctrl qctl; 
    struct v4l2_control control; 
    memset(&qctl, 0, sizeof(qctl));
    /*Exposure，
    https://www.linuxtv.org/downloads/legacy/video4linux/API/V4L2_API/spec-single/v4l2.html
    の1.9.6. Camera Control Reference 
    V4L2_CID_EXPOSURE_AUTO    露出と彩度を自動でおこなう．
    V4L2_CID_EXPOSURE_MANUAL    露出と彩度を手動で行う．
    V4L2_CID_EXPOSURE_SHUTTER_PRIORITY    露出を手動で，彩度を自動で行う．
    V4L2_CID_EXPOSURE_APERTURE_PRIORITY    露出を自動で，彩度を主導で行う．
    V4L2_CID_EXPOSURE_ABSOLUTE    露出時間を100μ秒単位で設定する．1が1/10000秒に，10000が１秒に，100000が１０秒にそれぞれ対応するる．
    ここでは，V4L2_CID_EXPOSURE_ABSOLUTEを選択する．
    */
    switch(mode) {
	case 0:
        qctl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        break;
	case 1:
        qctl.id = V4L2_CID_EXPOSURE_AUTO;
        break;
	case 2:
        qctl.id = V4L2_CID_EXPOSURE_MANUAL;
        break;
	case 3:
        qctl.id = V4L2_CID_EXPOSURE_SHUTTER_PRIORITY;
        break;
	case 4:
        qctl.id = V4L2_CID_EXPOSURE_APERTURE_PRIORITY;
        break;
	default:
        qctl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        break;	
    }		
    if (-1 == xioctl(fd, VIDIOC_QUERYCTRL, &qctl)) {
        if (errno != EINVAL) {
            perror("VIDIOC_QUERYCTRL");
            exit(EXIT_FAILURE);
        } else {
            fprintf(stderr, "V4L2_CID_EXPOSURE_ABSOLUTE is not surported\n");
        }
    } else if (qctl.flags & V4L2_CTRL_FLAG_DISABLED) {
        fprintf(stderr, "V4L2_CID_EXPOSURE_ABSOLUTE is not surported\n");
    } else {
        if (qctl.id == V4L2_CID_EXPOSURE_ABSOLUTE) {
            memset(&control, 0, sizeof(control));
            control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
            control.value = exposure;
            if (-1 == xioctl(id, VIDIOC_S_CTRL, &control)) {
                perror("VIDIOC_S_CTRL");
                exit(EXIT_FAILURE);
            }
        }
    }	
}

int make_image( const char* outfl, const char* outfl2 ) {
	
    unsigned char* ImageBuffer = NULL;
    cv::Mat src(480, 640, CV_8UC1); 
    cv::Mat dst;                    
	
    struct pollfd fds[1];
    /* Wait for the device file to become readable*/ 
    fds[0].fd = _fdv4l2;
    fds[0].events = POLLIN;
    int p = poll(fds,1,5000);
    if (-1 == p){
        perror("Waiting for Frame");
        return -1;
    }
    /* Retrieve Image data from Buffer */ 
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(_fdv4l2, VIDIOC_DQBUF, &buf)) {
        perror("Retrieving Frame");
        return -1;
    }
    /*open the file to write out */
    /* int out = open("out.jpg", O_RDWR | O_CREAT, S_IRWXU|S_IRWXO|S_IRWXG); */
    int out = open(outfl, O_RDWR | O_CREAT, S_IRWXU|S_IRWXO|S_IRWXG);
    if (out == -1){
        perror("file error");
        return -1;
    }
    /* write the data from the buffer */
    if (-1 == write(out, buffers[buf.index].start, buffers[buf.index].length)){
        perror("cannot write to output file");
        return -1;
    }
    if (-1 == close(out)) {
        perror("cannot close output file");
        return -1;
    }
    ImageBuffer = static_cast<unsigned char*>buffers[buf.index].start;      // read the raw data from the image capture
    if (ImageBuffer != NULL) {
        memcpy(src.data, ImageBuffer, src.step * src.rows);                 // memcopy the raw image data to an openCV mat
        cv::cvtColor(src, dst, CV_BayerGB2RGB);                             // convert to RGB and copy to dst
    } else {
        std::cerr << "no image buffer retrieved!" << std::endl;
    }
    /* try to detect the center of the road */
    cv::Mat img_bin;
    cv::Mat img_canny;
    cv::cvtColor(dst, img_bin, cv::COLOR_BGR2GRAY);
	// cv::Canny(dst, img_canny, 500.0, 700.0);
    cv::Canny(img_bin, img_canny, 500.0, 700.0);
    /* try to detect a straight line by using the Hough transform */
    vector<Vec4i> lines;
    cv::HoughLines(img_canny, lines, 0.5, CV_PI / 360, 80);
    /* et's overlay the detected straight line on the image and display it */
    std::vector<cv::Vec2f>::iterator it = lines.begin();
    for (; it != lines.end(); ++it) {
        float rho = (*it)[0], theta = (*it)[1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cv::saturate_cast<int>(x0 + 1000 * (-b));
        pt1.y = cv::saturate_cast<int>(y0 + 1000 * (a));
        pt2.x = cv::saturate_cast<int>(x0 - 1000 * (-b));
        pt2.y = cv::saturate_cast<int>(y0 - 1000 * (a));
        cv::line(dst, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }
	
    // cv::imshow("image to rgb with cross lines for road", dst);
	
	// Set and save image compression parameters
	// # IMWRITE_JPEG_QUALITY : IMAGE QUALITY when SAVING as JPEG 0: LOWEST ~ 100: HIGHEST
	// # When no parameters are specified in imwrite, the image quality is 95
	std::vector<int> params(2);
	params[0] = cv::IMWRITE_JPEG_QUALITY;
	params[1] = 100;
	cv::imwrite(outfl2, dst, params);
	
    /*next enqueue index return*/
    return buf.index;
}

/* Tell the device to stop stream */
void stream_stop(){
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(_fdv4l2, VIDIOC_STREAMOFF, &type)){
		perror("stream stop");
    }
}

/* Freeing the mapped memory */
void munmap_buffer(){
    for (int i = 0; i < n_buffers; ++i){
        if (-1 == munmap(buffers[i].start, buffers[i].length)){
		perror("munmap");
        }
    }
    free(buffers);
}

/* Close a device file */
void close_device(){
    close(_fdv4l2);
}

int main(int argc, char *argv[]) {
	
    cmdline::parser p;
    p.add("hoge", 'h', "hoge flag with no value");
    p.add<int>("width", 'w', "display width", false, 848);
    p.add<int>("height", 'h', "display height", false, 480);
    p.add<int>("no_frm", 'n', "number of frames to capture", false, 1);
    p.add<string>("out_file", 'o', "output file name", true, "out.jpg");
    p.add<string>("vid_device", 'v', "video device driver", false, "/dev/video0", cmdline::oneof<string>("/dev/video0", "/dev/video1", "/dev/video3", "/dev/video3"));
    p.add("help", 0, "print help");
    p.add<int>("expos_val", 'e', "exposure value", false, 100);
    p.add<int>("expos_mod", 'm', "exposure mode", false, 0);
	
    if (!p.parse(argc, argv)||p.exist("help")){
        cout << p.error_full() << p.usage();
        return 0;
    }
    const int width = p.get<int>("width");
    const int height = p.get<int>("height");
    const int no_of_frms = p.get<int>("no_frm");
    string out_file = p.get<string>("out_file");
    string vid_device = p.get<string>("vid_device");
    const int expos = p.get<int>("expos_val");
    const int ex_mode = p.get<int>("expos_mod");
	
    open_device(vid_device.c_str());
    cap_device();
    set_exposure(expos, ex_mode);
    set_device(width, height);
    request_buffer();
    map_buffer();
    enqueue_buffers();
    stream_start();
	/* now capture the frames */
    int cnt = 1;
    while (no_of_frms) {
	string out_file_name = out_file + "_" + to_string(cnt) + ".jpg";
	string det_file_name = out_file + "_detect_" + to_string(cnt) + ".jpg";
        int enqueue_index = make_image(out_file_name.c_str(), det_file_name.c_str());
        if (enqueue_index == -1){
            return EXIT_FAILURE;
        }
        enqueue_buffer(enqueue_index);
	no_of_frms--;
        cnt++;
    }
    stream_stop();
    munmap_buffer();
    close_device();
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}

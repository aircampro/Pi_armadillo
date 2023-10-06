/* 

first do this to install the neccessary libraries of uvc and openCV ...

$ git clone https://github.com/libuvc/libuvc.git
$ cd libuvc
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig
$ sudo apt install libopencv-dev

*/

#include "libuvc/libuvc.h"
#include <stdio.h>

/*
    compile as c++ :-  g++-8 -o uvc_test uvc_test.c `pkg-config --cflags opencv` `pkg-config --libs opencv` -luvc -O4
	/opt/intel/oneapi/compiler/2022.2.1/linux/bin/intel64/icpc -o uvc_test uvc_test.c -Wall -lusb-1.0 -O3 -qopt-report -luvc `pkg-config --cflags opencv` `pkg-config --libs opencv`
*/
#include <cv.h>
#include <highgui.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#define	SCALE	10
#define SIZE    12

/*
    if you need to go from new image cv2.0 to old
*/
IplImage& FromMat(IplImage& img, const cv::Mat& m)
{
    CV_Assert(m.dims <= 2);
    cvInitImageHeader(&img, m.size(), cvIplDepth(m.flags), m.channels());
    cvSetData(&img, m.data, (int)m.step[0]);
    return img;
}

/*
    mouse event handler for zoom in/out of image passed as pointer *userdata
*/
void mouseCallback(int event, int mx, int my, int flags, void *userdata)
{
	static cv::Mat zoomImage;
 
	if (zoomImage.empty() == true) {
		zoomImage = cv::Mat_<cv::Vec3b>((SIZE * 2 + 1)*SCALE, (SIZE * 4 + 2)*SCALE, cv::Vec3b::all(0));   		// Prepare an image for zoom in/out display
	}
 
	cv::Mat &image = *(cv::Mat *)userdata;	                                                                    // Refer to the image pointed to by the pointer userdata as an image
	int addx = 0;
	
 	switch (event) {
	    case cv::EVENT_LBUTTONDOWN:                                                                             // magnify - zoom in 
		addx = SIZE * 2 + 1;
		break;
 
	    case cv::EVENT_RBUTTONDOWN:                                                                             // zoom out
		addx = SIZE / 2 + 1;
		break;
 
	    case cv::EVENT_LBUTTONUP:
		// up button action is currently NULL
		break;
 
	    case cv::EVENT_RBUTTONUP:
		// up button action is currently NULL
		break;
	}
	
	for (int y = my - SIZE; y <= my + SIZE; y++) {
		for (int x = mx - SIZE; x <= mx + SIZE; x++) {
			cv::Scalar	color;
			if (x < 0 || y < 0 || x >= image.cols || y >= image.rows) {
				color = cv::Scalar::all(0);                  				//Set the black color if the coordinates (x,y) are outside the range of the image
			} else {
				color = image.at<cv::Vec3b>(y, x);                         	// Retrieve the pixel values of coordinates (x,y)
			}
			int zx = (x - mx + SIZE + addx) * SCALE;                      	// Add a mosaic color to an enlarged image
			int zy = (y - my + SIZE) * SCALE;
			cv::rectangle(zoomImage, cv::Rect(zx, zy, SCALE, SCALE), color, -1, cv::LINE_4);
		}
	}
	cv::imshow("zoomed_image", zoomImage);
}

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. 
 */
void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;
  
  /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!");
    return;
  }
  /* Do the BGR conversion */
  ret = uvc_any2bgr(frame, bgr);
  if (ret) {
    uvc_perror(ret, "uvc_any2bgr");
    uvc_free_frame(bgr);
    return;
  }
  /* Call a user function:
   *
   * my_type *my_obj = (*my_type) ptr;
   * my_user_function(ptr, bgr);
   * my_other_function(ptr, bgr->data, bgr->width, bgr->height);
   */
  /* Call a C++ method:
   *
   * my_type *my_obj = (*my_type) ptr;
   * my_obj->my_func(bgr);
   */
   
   // choose output format ref:- https://shibafu3.hatenablog.com/entry/2016/11/13/151118
   // #define __USE_H264_WMV_ --- defines the required writing codec
#if defined(__USE_WMV_)
   cv::VideoWriter writer("OutVideo.wmv", cv::VideoWriter::fourcc('W', 'M', 'V', '1'), 15.0, cv::Size(1024, 1024));
#elif defined(__USE_MP4V_)
   cv::VideoWriter writer("OutVideo.mp4",  cv::VideoWriter::fourcc('M','P','4','V'), 15.0, cv::Size(640, 512));
#elif defined(__USE_MP4S_)
   cv::VideoWriter writer("OutVideo.mp4",  cv::VideoWriter::fourcc('M','P','4','S'), 15.0, cv::Size(640, 512));
#elif defined(__USE_MOV_)
   cv::VideoWriter writer("OutVideo.mov",  cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 15.0, cv::Size(640, 512));   
#elif defined(__USE_XVID_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 15.0, cv::Size(640, 512));
#elif defined(__USE_DivX_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 15.0, cv::Size(640, 512));
#elif defined(__USE_Div3_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('D', 'I', 'V', '3'), 15.0, cv::Size(640, 512));
#elif defined(__USE_I420_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('I', '4', '2', '0'), 15.0, cv::Size(640, 512));
#elif defined(__USE_IYUV_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('I', 'Y', 'U', 'V'), 15.0, cv::Size(640, 512));
#elif defined(__USE_MJPG_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 15.0, cv::Size(640, 512));
#elif defined(__USE_PIM1_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('P', 'I', 'M', '1'), 15.0, cv::Size(640, 512));
#elif defined(__USE_XVID_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 15.0, cv::Size(640, 512));
#elif defined(__USE_H264_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('H', '2', '6', '4'), 15.0, cv::Size(640, 512));	
#elif defined(__USE_dv25_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('d', 'v', '2', '5'), 15.0, cv::Size(640, 512));	
#elif defined(__USE_dv50_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('d', 'v', '5', '0'), 15.0, cv::Size(640, 512));
#elif defined(__USE_dvc_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('d', 'v', 'c', ' '), 15.0, cv::Size(640, 512));
#elif defined(__USE_dvh1_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('d', 'v', 'h', '1'), 15.0, cv::Size(640, 512));
#elif defined(__USE_dvhd_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('d', 'v', 'h', 'd'), 15.0, cv::Size(640, 512));
#elif defined(__USE_dvsd_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('d', 'v', 's', 'd'), 15.0, cv::Size(640, 512));
#elif defined(__USE_dvsl_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('d', 'v', 's', 'l'), 15.0, cv::Size(640, 512));
#elif defined(__USE_H263_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('H', '2', '6', '3'), 15.0, cv::Size(640, 512));
#elif defined(__USE_M452_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('M', '4', 'S', '2'), 15.0, cv::Size(640, 512));
#elif defined(__USE_MP43_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('M', 'P', '4', '3'), 15.0, cv::Size(640, 512));
#elif defined(__USE_MPG1_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('M', 'P', 'G', '1'), 15.0, cv::Size(640, 512));
#elif defined(__USE_MSS1_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('M', 'S', 'S', '1'), 15.0, cv::Size(640, 512));
#elif defined(__USE_MSS2_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('M', 'S', 'S', '2'), 15.0, cv::Size(640, 512));
#elif defined(__USE_WVC1_WMV_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('W', 'V', 'C', '1'), 15.0, cv::Size(640, 512));
#elif defined(__USE_VP9_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('V', 'P', '9', '0'), 15.0, cv::Size(640, 512));		
#elif defined(__USE_WMV8_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('W', 'M', 'V', '2'), 15.0, cv::Size(1024, 1024));	
#elif defined(__USE_WMV9_)
    cv::VideoWriter writer("OutVideo.wmv",  cv::VideoWriter::fourcc('W', 'M', 'V', '3'), 15.0, cv::Size(1024, 1024));	
#elif defined(__USE_MPEG4_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('M', 'P', '4', '3'), 15.0, cv::Size(640, 512));	
#elif defined(__USE_RGB_AVI_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('D', 'I', 'B', ' '), 15.0, cv::Size(640, 512));	
#elif defined(__USE_X264_)
    cv::VideoWriter writer("OutVideo.avi",  cv::VideoWriter::fourcc('x', '2', '6', '4'), 15.0, cv::Size(640, 512));
#else
   cv::VideoWriter writer("OutVideo.mp4",  cv::VideoWriter::fourcc('M','P','4','V'), 15.0, cv::Size(640, 512));	
#endif

   int y_aria = 0;
   /* Use opencv.highgui to display the image:
   */ 
   IplImage *cvImg = cvCreateImageHeader(
       cvSize(bgr->width, bgr->height),
       IPL_DEPTH_8U,
       3);     
    cvSetData(cvImg, bgr->data, bgr->width * 3); 
    cvNamedWindow("UVC_Video_Stream", CV_WINDOW_AUTOSIZE);
    cvResizeWindow("UVC_Video_Stream", 640, 512);                    // try re-size ? might be detrimental in speed 
    cvShowImage("UVC_Video_Stream", cvImg);
    cvMoveWindow("UVC_Video_Stream", 0, 0);
    /*
      write the image out to a file
    */
    cv::Mat image;                                                   // define new openCV 2.0 format for writing
    image = cv::cvarrToMat(cvImg);                                   // convert old cv format to new format
    if (writer.isOpened() == true) {
       writer << image;                                             // write data to disk
    }	   

    // write out a still jpeg from the image --- probably want to link this to a trigger button
    std::vector<int> params(2);
    params[0] = cv::IMWRITE_JPEG_QUALITY;
    params[1] = 100;                                                // 100 is highest quality 0 = lowest 95 is default
    cv::imwrite("JPEG100.jpg", image, params);
	
    // added this section if you wanted to for example implement edge detection on the imaage
    // comment out as its probably too slow 
    //
    cv::Mat gray,canny;                                              // define new openCV 2.0 format for grayscale and canny edge detection
    cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);                   // grayscale
    cv::Canny(gray,canny, 50, 200);                                  // canny edge detection
    const char* windowName = "Canny Edge Detection";
    cv::imshow(windowName, canny);  
   
    // uncomment if you dont want zoom in/out with the mouse
    // Register a callback function
    // # In this program, the callback function name is mouseCallback
    // # Pass image as a pointer so that the callback function can access the image data
    // in this instance we are performing this on the canny edge detected image
    //
    cv::setMouseCallback(windowName, mouseCallback, (void *)&canny);
   
    // this is converting to hsv space to look at colours in the frame if you need it.
    //
    cv::Mat hsv_video,smooth_video;                                  // here is an example of convert to HSV
    cv::medianBlur(image, smooth_video, 5);
    cv::cvtColor(smooth_video, hsv_video, CV_BGR2HSV);   
    int hue,sat,val;
    cv::Mat dst_img;
		
    for(int y = 0; y < hsv_video.rows; y++) {
        for (int x = 0; x < hsv_video.cols; x++) {
            hue = hsv_video.at<cv::Vec3b>(y, x)[0];
            sat = hsv_video.at<cv::Vec3b>(y, x)[1];
            val = hsv_video.at<cv::Vec3b>(y, x)[2];

            if ((hue < 35 && hue > 20) && sat > 127) {
                dst_img.at<uchar>(y, x) = 255;
                y_aria++; // look for yellow
            }
            else {
                dst_img.at<uchar>(y, x) = 0;
            }
        }
    }
	if (y_aria >1000) 
		puts("1000 yellow pixsels seen");
	
    // now wait for 10 secs and close
    cvWaitKey(10);
    // you can put in a while (1) loop and wait for the keyboard so interrupt like this int key = cv::waitKey(1)
    // and test for this 
    // 	if ((key == 113) || (key == 'q'))
    //	{
    //	   break;．
    //	}
    cvReleaseImageHeader(&cvImg);
    cvDestroyWindow("UVC_Video_Stream");                             // destroy the window
   
    cv::destroyAllWindows();                                         // if you use imshow then keep this in otherwise comment back out
    uvc_free_frame(bgr);
}

int main(int argc, char **argv) 
{
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;
  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx, NULL);
  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }
  puts("UVC initialized");
  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(
      ctx, &dev,
      0, 0, NULL);                                                  /* filter devices: vendor_id, product_id, "serial_num" */
  if (res < 0) {
    uvc_perror(res, "uvc_find_device");                             /* no devices found */
  } else {
    puts("Device found");
    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, &devh);
    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
    } else {
      puts("Device opened");
      /* Print out a message containing all the information that libuvc
       * knows about the device */
      uvc_print_diag(devh, stderr);
      /* Try to negotiate a 640x480 30 fps YUYV stream profile */
      res = uvc_get_stream_ctrl_format_size(
         devh, &ctrl,                                               /* result stored in ctrl */
         UVC_FRAME_FORMAT_BGR,                                      /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
         160, 120, 9                                                /* width, height, fps */
      );
      //res = 0;
      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);
      if (res < 0) {
        uvc_perror(res, "get_mode");                                /* device doesn't provide a matching stream */
      } else {
        /* Start the video stream. The library will call user function cb:
         *   cb(frame, (void*) 12345)
         */
        res = uvc_start_streaming(devh, &ctrl, cb, (void*) 12345, 0);
        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          puts("Streaming...");
          uvc_set_ae_mode(devh, 1);                                    /* e.g., turn on auto exposure */
          sleep(10);                                                   /* stream for 10 seconds */
          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
          puts("Done streaming.");
        }
      }
      /* Release our handle on the device */
      uvc_close(devh);
      puts("Device closed");
    }
    /* Release the device descriptor */
    uvc_unref_device(dev);
  }
  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
  puts("UVC exited");
  return 0;
}
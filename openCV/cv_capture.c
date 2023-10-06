/*

    openCV 1.0 example of using USB webcam
	
*/
#include <iostream>

#include <cv.h>
#include <highgui.h>

char* winname="USB Capture";

CvCapture *capture; // define CV Capture pointer

IplImage *src;     //　pointer open cv 1.0 image format

int main(int argc, char **argv){

	char c;
	cvInitSystem(argc,argv);

	cvNamedWindow(winname, CV_WINDOW_AUTOSIZE);
	cvResizeWindow(winname, src_video_w, src_video_h);
	cvMoveWindow(winname, 0, 0);

	capture = cvCreateCameraCapture(-1);                                         
	// uncomment if you want a file..... capture = cvCreateFileCapture("filename.avi");
	
	if( !capture ){                                          // NULL returned if error
		fprintf(stderr,"Could not initialize capturing...\n");
		return -1;
	}

	for(;;){ //  ever unless ESC key pressed
   
		src = cvQueryFrame( capture );            // get the frame

		if( !src ){
			break;
		}
  
		cvShowImage(winname, src );
      
		c = cvWaitKey(10);
		if( c == 27 ){                          // ESC ascii=27 DEC then exit for loop
			break;
		}
	}
  
	cvReleaseCapture( &capture );               // release the capture

	cvDestroyWindow(winname);

	return 0;
}
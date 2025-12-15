//#include "ardrone/ardrone.h"
#include <iostream>
#include <cv.h>
#include <highgui.h>

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)
CvCapture *capture;                                       // define CV Capture pointer

// --------------------------------------------------------------------------
// main (number of arguments, argument list)
// is the main function
// Return value Successful Completion: 0 Error: -1
// ------------------------------------------------------------------------
int main(int argc, char **argv)
{
    // AR.Drone
    //ARDrone ardrone;
 
    //if (!ardrone.open()) {
    //    printf("ARDroneの初期化に失敗しました\n");
    //    return -1;
    //}
 
    // AR.Drone get image from webcam
    //IplImage *image = ardrone.getImage();
	cvInitSystem(argc,argv);
	
	//IplImage *image = ardrone.getImage();
	capture = cvCreateCameraCapture(-1);  
	IplImage *image = cvQueryFrame( capture ); 
 
    int corner_count = 50;
    IplImage *gray = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    IplImage *prev = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    cvCvtColor(image, prev, CV_BGR2GRAY);
    IplImage *eig_img = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    IplImage *tmp_img = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    IplImage *prev_pyramid = cvCreateImage(cvSize(image->width+8, image->height/3), IPL_DEPTH_8U, 1);
    IplImage *curr_pyramid = cvCreateImage(cvSize(image->width+8, image->height/3), IPL_DEPTH_8U, 1);
    CvPoint2D32f *corners1 = (CvPoint2D32f*)malloc(corner_count * sizeof(CvPoint2D32f));
    CvPoint2D32f *corners2 = (CvPoint2D32f*)malloc(corner_count * sizeof(CvPoint2D32f));
 
    while (!GetAsyncKeyState(VK_ESCAPE)) {
        // AR.Droneの更新
        //if (!ardrone.update()) break;
 
        //image = ardrone.getImage();
        image = cvQueryFrame( capture );
		
        //if (KEY_PUSH(VK_SPACE)) {
        //    if (ardrone.onGround()) ardrone.takeoff();
        //    else                    ardrone.landing();
        //}
 
        //if (!ardrone.onGround()) {
            // 速度指令
            double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
            if (KEY_DOWN(VK_UP))    vx =  0.5;
            if (KEY_DOWN(VK_DOWN))  vx = -0.5;
            if (KEY_DOWN(VK_LEFT))  vr =  0.5;
            if (KEY_DOWN(VK_RIGHT)) vr = -0.5;
            if (KEY_DOWN('Q'))      vz =  0.5;
            if (KEY_DOWN('A'))      vz = -0.5;
            //ardrone.move3D(vx, vy, vz, vr);
        //}
 
        cvCvtColor(image, gray, CV_BGR2GRAY);
 
        corner_count = 50;
        cvGoodFeaturesToTrack(prev, eig_img, tmp_img, corners1, &corner_count, 0.1, 5.0, NULL);
 
        if (corner_count > 0) {
            char *status = (char*)malloc(corner_count * sizeof(char));
 
            CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.3);
            cvCalcOpticalFlowPyrLK(prev, gray, prev_pyramid, curr_pyramid, corners1, corners2, corner_count, cvSize(10, 10), 3, status, NULL, criteria, 0);
 
            for (int i = 0; i < corner_count; i++) {
                cvCircle(image, cvPointFrom32f(corners1[i]), 1, CV_RGB (255, 0, 0));
                if (status[i]) cvLine(image, cvPointFrom32f(corners1[i]), cvPointFrom32f(corners2[i]), CV_RGB (0, 0, 255), 1, CV_AA, 0);
            }
 
            free(status);
        }
 
        cvCopy(gray, prev);
 
        cvShowImage("camera", image);
        cvWaitKey(1);
    }
 
    cvReleaseImage(&gray);
    cvReleaseImage(&prev);
    cvReleaseImage(&eig_img);
    cvReleaseImage(&tmp_img);
    cvReleaseImage(&prev_pyramid);
    cvReleaseImage(&curr_pyramid);
    free(corners1);
    free(corners2);
 
    //ardrone.close();
    cvDestroyWindow("camera");
		
    return 0;

}

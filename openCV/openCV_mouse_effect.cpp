/*

Example of mouse control in openCV C++

cmake_minimum_required(VERSION 2.8)
project(test_cmake CXX)

add_executable(disp_img disp_img.cpp)
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    target_include_directories(disp_img PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(disp_img ${OpenCV_LIBS})
endif()

Apply various filters to an image by using mouse click

*/

#include <opencv2\opencv.hpp>
#include <opencv_lib.hpp>
#include <iostream>

using namespace std;
using namespace cv;

Rect2i rectangle_value;                  // draws rectangle on the screen
Point MousePointer(0,0);                 // saves mouse pointer co-ord
int effect = 0;                          // the effect to apply on the image
Rect2i rectangle_pb;

void mouse_callback(int event, int x, int y, int flags, void *userdata)
{
	bool *isClick = static_cast<bool *>(userdata);
	if (event == EVENT_LBUTTONDOWN) {
		*isClick = true;
		cout << "Left mouse button is pressed." << endl;
		cout << "Draw rectangle\n" << " start position (x, y) : " << x << ", " << y << endl;
		rectangle_value = Rect2i(x, y, 0, 0);                                                                        // draw select start point
	} else if (event == EVENT_LBUTTONUP) {
		*isClick = false;
		cout << "Left mouse button is released." << endl;
		cout << " end   position (x, y) : " << x << ", " << y << endl;
		rectangle_value.width = x - rectangle_value.x;                                                               // draw final selected box
		rectangle_value.height = y - rectangle_value.y;
	} else if (event == EVENT_MBUTTONDOWN) {
		cout << "Middle mouse button is pressed." << endl;
		if (((MousePointer.x >=0) && (MousePointer.x <=10)) && ((MousePointer.y >=0) && (MousePointer.y <=10))) {     // check if we clicked the box
		    cout << "Inside the pushbutton" << endl;		
		}
	} else if (event == EVENT_MBUTTONUP) {
		cout << "Middle mouse button is released." << endl;
	} else if (event == EVENT_RBUTTONDOWN) {
		cout << "Right mouse button is pressed." << endl;
		effect++ % 10;                                                                                                   // change the effect
	} else 	if (event == EVENT_RBUTTONUP) {
		cout << "Right mouse button is released." << endl;
	} else if (event == EVENT_LBUTTONDBLCLK) {
		cout << "Left mouse button is double clicked. " << endl;
	} else if (event == EVENT_RBUTTONDBLCLK) {
		cout << "Right mouse button is double clicked. " << endl;
	} else if (event == EVENT_MOUSEWHEEL) {
		if (getMouseWheelDelta(flags)>0) {
			cout << "Mouse wheel is forward." << endl;
		}
		if (getMouseWheelDelta(flags)<0) {
			cout << "Mouse wheel is backward." << endl;
		}
	} else if (event == EVENT_MOUSEMOVE) {
        MousePointer = Point(x,y);
		cout << "(" << x << ", " << y << ")" << endl;
		if (*isClick) {
			rectangle_value.width = x - rectangle_value.x;
			rectangle_value.height = y - rectangle_value.y;
		}
	}

}

int main(void)
{
	Mat img = imread("./cpp_img.png");
	Mat draw_img = img.clone();
    Mat	dst1, src2;
	bool isClick = false;
	int key;
	string window_name = "example of mouse control";

    namedWindow(window_name, CV_WINDOW_NORMAL);
    moveWindow(window_name, 0, 0);
    resizeWindow(window_name, 640, 480);
	imshow("example", img);

	imshow(window_name, img);
	setMouseCallback(window_name, mouse_callback, &isClick);
	rectangle_pb = Rect2i(10, 10, 10, 10);                                                // rectangle button example at 10,10 width 10 ht 10
	for (;;) {
		key = 0;
		
		if (isClick == true) {
			rectangle(draw_img, rectangle_value, Scalar(255, 0, 0), 3, CV_AA);            // draw selcted area 
		}

        rectangle(draw_img, rectangle_pb, Scalar(255, 0, 0), 3, CV_AA);                   // draw pb area to check click when middle pressed 
		imshow(window_name, draw_img);
		draw_img = img.clone();

        switch(effect) {
            case 0:
	        blur(draw_img, dst1, Size(7, 7));
            break;	
            case 1:
	        medianBlur(draw_img, dst1, 7);
            break;
            case 2:
	        GaussianBlur(draw_img, dst1, Size(7, 7), 0.0);
            break;
            case 3:
	        Laplacian(draw_img, dst1, -1);
            break;
            case 4:
	        Canny(draw_img, dst1, 60.0, 150.0);
            break;	
            case 5:
		    cvtColor(draw_img, src2, COLOR_BGR2GRAY);
	        threshold(src2, dst1, 140, 255, cv::THRESH_BINARY);
            break;
            case 6:
            cvtColor(draw_img, src2, COLOR_BGR2GRAY);
	        threshold(src2, dst1, 0, 255, cv::THRESH_OTSU);
            break;	
			case 7:
			Sobel(draw_img, dst1, CV_16S, 1, 0, 3, 10); 
            case 8:
            Sobel(draw_img, dst1, CV_16S, 0, 1, 3, 10);
			case 9:
	        Laplacian(draw_img, dst1, 3);
	        convertScaleAbs(dst1, dst1, 1, 0);
	        threshold(dst1, dst1, 0, 255, THRESH_BINARY|THRESH_OTSU);
            default:
            break;			
        }		

	    imshow("edge detect", dst1);                                               // draw chosen effected image

		key = waitKey(1);
		if (key == 'q')
			break;
	}

	return 0;
}
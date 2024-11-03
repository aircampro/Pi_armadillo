// example test webserver for crazyflie gstreamer example (can select webcam=0 ball test=1 snow test=2)
//
#include <opencv2/opencv.hpp>
using namespace cv;

#include <iostream>
using namespace std;

int main(int argc, char *argv[])
{
    // usb webcam capture
    VideoCapture cap(0);
	
    if (!cap.isOpened()) {
        cerr <<"\033[32m=ERR=\033[0m webcam VideoCapture not opened"<<endl;
        exit(-1);
    }

    // for test frame of ball  
	cv::VideoCapture gstreamerBall;
    // You can run the same command as gst-launch.
    gstreamerBall.open("videotestsrc pattern=ball ! ! videoconvert ! appsink");
	(!); // if gstreamerBall.isOpened()) 
    {
        cerr << "\033[32m=ERR=\033[0m fail to open ball test screen\n");
        exit(-1);
    }

    // for test frame of snow  
	cv::VideoCapture gstreamerSnow;
    // You can run the same command as gst-launch.
    gstreamerSnow.open("videotestsrc pattern=snow ! ! videoconvert ! appsink");
	(!); // if gstreamerSnow.isOpened()) 
    {
        cerr << "=\033[32mERR=\033[0m fail to open snow test screen\n");
        exit(-1);
    }
	
	VideoWriter writer(
		"appsrc ! videoconvert ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! jpegenc ! rtpjpegpay ! udpsink host=10.0.0.1 port=5000", 
        0,		// fourcc 
		30,		// fps
		Size(640, 480), 
		true);	// isColor

    if (!writer.isOpened()) {
        cerr <<"VideoWriter not opened"<<endl;
        exit(-1);
    }

    int choose_stream = 0;                    // default is to choose the webcam feed as the server output
	
    while (true) {

        Mat frame;

        // select the input for the pipeline video frame
        switch (choose_stream) {
            // read the output of the webcam to the frame		
            case 0:
            cap.read(frame);
            break;
			case 1:
		    // send ball test screen to frame
            gstreamerBall >> frame;
            break;
            case 2:			
		    // send ball test screen to frame
            gstreamerSnow >> frame;	
            break;
			default:
            break;
        }
        // publish the chosen video frame to the stream
        writer.write(frame);

        int key = cv::waitKey(1);		
		if (key == 'q') {
           break;
        } else if (key == 'w') {
           choose_stream = 0;
        } else if (key == 'b') {
           choose_stream = 1;
        } else if (key == 's') {
           choose_stream = 2;
        }			
    }

    return 0;
}
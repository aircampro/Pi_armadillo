// open CV example reading video feed and performing tracker
// controlling a crazyflie drone
//
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <zmq.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

char * messageFmt = "{" \
    "\"version\": 1," \
    "\"client_name\": \"ramp C example\"," \
    "\"ctrl\": {" \
        "\"roll\": %f," \
        "\"pitch\": %f," \
        "\"yaw\": %f," \
        "\"thrust\": %f" \
    "}" \
"}";
char message[512];
#define NOMOVE 0
#define PITCH 1          # pitch bit 0
#define ROLL 2           # roll bit 1
#define YAW 4            # yaw bit 2
#define THRUST 8         # thrust bit 8

using namespace cv;
using namespace std;
 
// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()
 
int main(int argc, char **argv)
{
    // initialise connection to crazyflie
    void *context;
    void *socket_zmq;
    const char *address;
    context = zmq_ctx_new();
    socket_zmq = zmq_socket(context, ZMQ_PUSH);
    address = "tcp://127.0.0.1:1212";

    printf("Connecting the socket ...\n");
    zmq_connect(socket_zmq, address);
 
    // move drone to initial setting
    float thrust = 2.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    int flag = NOMOVE;
    int counter[5];                       // pitch roll yaw thrust reset
    const float pDelta = 0.2f;
    const float rDelta = 0.2f;
    const float yDelta = 0.2f;
    const float tDelta = 0.2f;
    const int pLim = 200;
    const int rLim = 2000;
    const int yLim = 1000;
    const int tLim = 1000;
    const int cLim = 100000;
    int reset_counter = 0;
	
    sprintf(message, messageFmt, roll, pitch, yaw, thrust);
    zmq_send(socket_zmq, message, strlen(message), 0);
    printf("\rPitch = %f Roll = %f Yaw = %f Thrust = %f%%", pitch, roll, yaw, thrust);

    // define the GStreamer Pipeline that we receive from
    // The sink caps for the 'rtpjpegdepay' need to match the src caps of the 'rtpjpegpay' of the sender pipeline
    // Added 'videoconvert' at the end to convert the images into proper format for appsink, without
    // 'videoconvert' the receiver will not read the frames, even though 'videoconvert' is not present
    // in the original working pipeline
	VideoCapture cap("udpsrc port=5000 ! application/x-rtp,media=video,payload=26,clock-rate=90000,encoding-name=JPEG,framerate=30/1 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink", CAP_GSTREAMER);
    
	if (!cap.isOpened()) {
        cerr <<"VideoCapture not opened"<<endl;
        exit(-1);
    }
	
    // List of tracker types in OpenCV 3.4.1
    string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    // vector <string> trackerTypes(types, std::end(types));
 
    // Create a tracker
    string trackerType = trackerTypes[2];
 
    Ptr<Tracker> tracker;
 
    #if (CV_MINOR_VERSION < 3)
    {
        tracker = Tracker::create(trackerType);
    }
    #else
    {
        if (trackerType == "BOOSTING")
            tracker = TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker = TrackerMIL::create();
        if (trackerType == "KCF")
            tracker = TrackerKCF::create();
        if (trackerType == "TLD")
            tracker = TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker = TrackerMedianFlow::create();
        if (trackerType == "GOTURN")
            tracker = TrackerGOTURN::create();
        if (trackerType == "MOSSE")
            tracker = TrackerMOSSE::create();
        if (trackerType == "CSRT")
            tracker = TrackerCSRT::create();
    }
    #endif
    // uncomment to test with a video input Read video
    //VideoCapture video("your_video.mp4");
     
    // Exit if video is not opened
    //if(!video.isOpened())
    //{
    //    cout << "Could not read video file" << endl; 
    //    return 1; 
    //} 
    //Mat frame; 
    //bool ok = video.read(frame); 
	
    // Read first frame from the GStreamer pipeline
    Mat frame;
    cap.read(frame);
 
    // Define initial bounding box 
    Rect2d bbox(287, 23, 86, 320); 
 
    // Uncomment the line below to select a different bounding box 
    // bbox = selectROI(frame, false); 
    // Display bounding box. 
    rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 ); 
 
    imshow("Tracking", frame); 
    tracker->init(frame, bbox);
     
//    while(video.read(frame))    -- for test
    while(cap.read(frame))
    {     
        // Start timer
        double timer = (double)getTickCount();
         
        // Update the tracking result
        bool ok = tracker->update(frame, bbox);
         
        // Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double)getTickCount() - timer);
         
        if (ok)
        {
            // Tracking success : Draw the tracked object
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );

            if (counter[3] > tLim) {
                flag = flag | THRUST;
            }
	    if (flag && THRUST) {
	        thrust += tDelta;
                flag = flag ^ THRUST;            // one shot
	    }				
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
			// use counters to trigger movement in the event of nothing tracked
            if (counter[0] > pLim) {
                flag = flag | PITCH;
            } else if (counter[1] > rLim) {
                flag = flag | ROLL;
            } else if (counter[2] > yLim) {
                flag = flag | YAW;
            }				
	    if (flag && PITCH) {
	        pitch += pDelta;
                pitch ^= PITCH;
	     } else if (flag && YAW) {
		yaw += yDelta;
                yaw ^= YAW;
	     } else if (flag && ROLL) {
		roll += rDelta;
                roll ^= ROLL;
            }
	    for (int j=0; j<3 ; j++) {                               // increment counters
                counter[j]++;
            }
            if (reset_counter == 1) || (counter[4] > cLim) {         // send a reset counter to advance the next timed movement	- either from key or timer		
		for (int j=0; j<3 ; j++) {                           // reset counters
                    counter[j] = 0;
                }
                reset_counter = 0;
            }				
        }
		
		// move the drone as specified
        sprintf(message, messageFmt, roll, pitch, yaw, thrust);
        zmq_send(socket_zmq, message, strlen(message), 0);
        printf("\rPitch = %f Roll = %f Yaw = %f Thrust = %f%%", pitch, roll, yaw, thrust);
	
        // Display tracker type on frame
        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
         
        // Display FPS on frame
        putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
 
        // Display frame.
        imshow("Tracking", frame);
         
        // Exit if ESC pressed.
        int k = waitKey(1);
        if(k == 27) {
            break;
        } else if (k == 'R') {
            pitch = 0.0f;
            roll = 0.0f;
            yaw = 0.0f;
        } else if (k == 'S') {
            thrust = 0;
        } else if (k == 'T') {
            reset_counter = 1;
        } else if (k == 'U') {
            thrust += tDelta;
        } else if (k == 'D') {
            thrust -= tDelta;
        }			
    }
    sprintf(message, messageFmt, 0.0f, 0.0f, 0.0f, 0.0f);
    zmq_send(socket_zmq, message, strlen(message), 0);
    printf("\rStopping\n");

    zmq_close(socket_zmq);
    zmq_ctx_destroy(context);
}

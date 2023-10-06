// example of apple HSL video streaming image capture is from yolo darknet
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

using namespace cv;
using namespace std;
using namespace dnn;

int main()
{
    VideoWriter writer(
        "appsrc ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 
        ! x264enc ! mpegtsmux ! hlssink playlist-root=http://172.22.222.39:8080/live/
        location=C:/streamServer/hls/segment%05d.ts
        playlist-location=C:/streamServer/hls/playlist.m3u8 ",
        0,
        20,
        Size(800, 600),
        true);

    VideoCapture cap("/samp.MOV");
    std::string model = "/yolov3-tiny.weights";  
    std::string config = "/yolov3-tiny.cfg"; 
    Net network = readNet(model, config, "Darknet");
    network.setPreferableBackend(DNN_BACKEND_DEFAULT);
    network.setPreferableTarget(DNN_TARGET_OPENCL);

    for (;;)
    {
        if (!cap.isOpened()) {
            cout << "Video Capture Fail" << endl;
            break;
        }
        Mat img;
        cap >> img;
        static Mat blobFromImg;
        //blobFromImage(img, blobFromImg, 1.0, Size(Width, Height), Scalar(), swapRB, false, CV_8U);
        //network.setInput(blobFromImg, "", scale, mean);
        bool swapRB = true;
        blobFromImage(img, blobFromImg, 1, Size(416, 416), Scalar(), swapRB, false);
        cout << blobFromImg.size() << endl;
        float scale = 1.0 / 255.0;
        Scalar mean = 0;
        network.setInput(blobFromImg, "", scale, mean);
        Mat outMat;
        network.forward(outMat);
        // rows represent number of detected object (proposed region)
        int rowsNoOfDetection = outMat.rows;

        // The columns looks like this, The first is region center x, center y, width
        // height, The class 1 - N is the column entries, which gives you a number, 
        // where the biggest one corresponding to most probable class. 
        // [x ; y ; w; h; class 1 ; class 2 ; class 3 ;  ; ;....]
        // [x ; y ; w; h; class 1 ; class 2 ; class 3 ;  ; ;....]
        int colsCoordinatesPlusClassScore = outMat.cols;


        // Loop over the number of the detected objects. 
        for (int j = 0; j < rowsNoOfDetection; ++j)
        {
            // for each row, the score is from element 5 up to number of classes
            //index (5 - N columns)
            Mat scores = outMat.row(j).colRange(5, colsCoordinatesPlusClassScore);
            Point PositionOfMax;
            double confidence;

            // This function find indexes of min and max confidence and related
            // index of element. 
            // The actual index is matched the concrete class of the object.
            // First parameter is Mat which is row [5fth - END] scores,
            // Second parameter will give you the min value of the scores. NOT needed 
            // confidence gives you a max value of the scores. This is needed, 
            // Third parameter is index of minimal element in scores
            // the last is the position of the maximum value. This is the class!!
            minMaxLoc(scores, 0, &confidence, 0, &PositionOfMax);

            if (confidence > 0.0001)
            {
                int centerX = (int)(outMat.at<float>(j, 0) * img.cols); 
// thease four lines are
                int centerY = (int)(outMat.at<float>(j, 1) * img.rows); 
// first column of the tow
                int width = (int)(outMat.at<float>(j, 2) * img.cols + 20); 
// identify the position 
                int height = (int)(outMat.at<float>(j, 3) * img.rows + 100); 
// of proposed region
                int left = centerX - width / 2;
                int top = centerY - height / 2;


                stringstream ss;
                ss << PositionOfMax.x;
                string clas = ss.str();
                int color = PositionOfMax.x * 10;
                putText(img, clas, Point(left, top), 1, 2, Scalar(color, 255, 255), 2, false);
                stringstream ss2;
                ss << confidence;
                string conf = ss.str();
                
                rectangle(img, Rect(left, top, width, height), Scalar(color, 0, 0), 2, 8, 0);
            }
        }
        resize(img, img, Size(800, 600));
        namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
        imshow("Display window", img);
        waitKey(25);
        resize(img, img, Size(800, 600));
        writer.write(img);
    }

}
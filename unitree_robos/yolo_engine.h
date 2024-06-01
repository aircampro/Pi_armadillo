//#pragma once
#ifndef __YoloLib
#define __YoloLib

#include <iostream>
#include <opencv2\opencv.hpp>
#include <fstream>
#include <filesystem>
#include <vector>
//#include <onnxruntime_cxx_api.h>

// put the default names for your yolo files here....
#define DEFAULT_ONNX_FILE_PATH "awz.onnx"
#define DEFAULT_NAMES_FILE_PATH "awz.names"
#define DEFAULT_ONNX_FILE_PATH_YOLOV5 "yolov5s.onnx"
#define DEFAULT_NAMES_FILE_PATH_YOLOV5 "yolov5s.names"
//#define DEFAULT_CLASSIFICATION_SIZE 12

#ifdef _GPUX
#define DEFAULT_AI_INPUT_WIDTH 1280.0f
#define DEFAULT_AI_INPUT_HEIGHT 1280.0f
#else
#define DEFAULT_AI_INPUT_WIDTH 640.0f
#define DEFAULT_AI_INPUT_HEIGHT 640.0f
#endif

//#define DEFAULT_SCORE_THRESHOLD  0.2f
//#define DEFAULT_NMS_THRESHOLD  0.60f
//#define DEFAULT_CONF_THRESHOLD  0.2f

extern float DEFAULT_SCORE_THRESHOLD;
extern float DEFAULT_NMS_THRESHOLD;
extern float DEFAULT_CONF_THRESHOLD;

// Text parameters.
#define FONT_SCALE_LABEL  0.4f
#define FONT_FACE_LABEL   FONT_HERSHEY_SIMPLEX
#define THICKNESS_FONT_LABEL   1

//for sumnail big font
#define FONT_SCALE_PERSON  0.75f
#define FONT_FACE_PERSON   FONT_HERSHEY_SIMPLEX
#define THICKNESS_FONT_PERSON   2
#define TEXT_POINT_PERSON  Point(10, 20)

#define FONT_SCALE_TIME  0.3f
#define FONT_FACE_TIME   FONT_HERSHEY_SIMPLEX
#define THICKNESS_FONT_TIME   1

#define THICKNESS_BOX   2
#define THICKNESS_FONT   1

#define YOLOV5 0
#define YOLOV8 1

struct YoloAIParametors
{
public:
    int yolo_version;

    float input_width;
    float input_height;

    float score_threshold;
    float nms_threshold;
    float confidence_thresgold;

    int clssification_size;
    std::string onnx_file_name;
    std::string names_file_name;

    YoloAIParametors();
    
    //YoloAIParametors(
    //float _input_width = DEFAULT_AI_INPUT_WIDTH,
    //float _input_height = DEFAULT_AI_INPUT_HEIGHT,
    //float _score_threshold = DEFAULT_SCORE_THRESHOLD,
    //float _nms_threshold = DEFAULT_NMS_THRESHOLD,
    //float _confidence_thresgold = DEFAULT_CONF_THRESHOLD);

    ~YoloAIParametors();
};
struct CvFontParam
{
    float scale;
    int face;
    int thickness;
};

struct YoloFontsParam
{
    CvFontParam label;
    CvFontParam person;
    CvFontParam time;

    int thickness_box;
};

//Structure for organizing data for each recognition
//struct DetectedObject {
//    float x;      // X coordinate of the center of the bounding box
//    float y;      // Y coordinate of the center of the bounding box
//    float width;  // bounding box width
//    float height; // bounding box height
//    float confidence; // confidence
//    int classId;  // class ID
//};
//Convert the cv::Mat data obtained from the YOLOv5  YOLOv5のnet.forward()method to a vector of DetectedObject structure
//std::vector<DetectedObject> convertDetections(const std::vector<cv::Mat>& outputs, float confThreshold);

class YoloObjectDetection
{
    bool count_of_person = false;
    bool count_of_time = false;
    //volatile  bool _busy = false;

public:
    YoloAIParametors YP;
private:
    YoloFontsParam YFP;

public:
    YoloObjectDetection();

    int number_of_persons = 0;
	int wave_hand_state = 0;             // set to state robot should wave hands
    std::vector<cv::Mat> detections;     // Process the image.
    std::vector<std::string> list_of_class;
    std::vector<std::string> class_list_view = { "person", "forklift", "tractor", "driver", "truck", "excavator", "wheelloder", "grader", "bulldozer", "pallet", "cargo" , "car", "rabbit", "bird", "dog" };
    cv::dnn::Net net;

    std::vector<cv::Mat>& _pre_process(cv::Mat& input_image);

    cv::Mat _post_process(
        bool draw_image,
        const cv::Mat& input_image, 
        std::string _header, std::string& _ost
    );

    int init_yolo(
        YoloAIParametors yp,
        bool __count_of_person, bool __count_of_time);

public:
    std::atomic<bool> busy = false;//使っていないかも
};

cv::Mat post_process_str(
    YoloAIParametors yp,
    YoloFontsParam yfp,
    bool draw_image,                // Normally, if set to true or false, no drawing processing will be performed. Returns only text. For analysis
    const cv::Mat& input_image, std::vector<cv::Mat>& outputs, const std::vector<std::string>& class_name,
    int& number_of_persons, std::vector<std::string>& class_list_view,
    std::string _header,            // date etc.
    std::string& _ost,              // Storage location of string containing AI analysis results
    int _version,
	int& wave_hand_state
);

//LPCWSTR _A2CW(const std::string& ascii);
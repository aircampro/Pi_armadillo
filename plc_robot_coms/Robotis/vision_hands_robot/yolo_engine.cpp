#ifndef __yolo_det_
#define __yolo_det_

#define NOMINMAX
#include <atltypes.h>
#include <opencv2\opencv.hpp>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <shellapi.h> 
#include <filesystem>
#include <commdlg.h>
#include <random>

//for yolo
#include <vector>
#include <iterator>
#include <windows.h>
#include <CommCtrl.h>

#include "yolo_engine.h"

//#include "logfile.h"

using namespace cv;
using namespace std;
using namespace cv::dnn;

//#define DEFAULT_SCORE_THRESHOLD  0.2f
//#define DEFAULT_NMS_THRESHOLD  0.60f
//#define DEFAULT_CONF_THRESHOLD  0.2f

const float DEFAULT_SCORE_THRESHOLD = 0.4f;
const float DEFAULT_NMS_THRESHOLD = 0.5f;
const float DEFAULT_CONF_THRESHOLD = 0.4f;

//https://learnopencv.com/object-detection-using-yolov5-and-opencv-dnn-in-c-and-python/
//https://github.com/spmallick/learnopencv/tree/master/Object-Detection-using-YOLOv5-and-OpenCV-DNN-in-CPP-and-Python
// 
// Constants.

// Colors.
Scalar BLACK = Scalar(0, 0, 0);
Scalar BLUE = Scalar(255, 178, 50);
Scalar KOIBLUE = Scalar(255, 32, 32);
Scalar BLUEGREEN = Scalar(240, 240, 32);
Scalar YELLOW = Scalar(0, 255, 255);
Scalar RED = Scalar(0, 0, 255);
Scalar GREY = Scalar(64, 64, 64);
Scalar DARKGREEN = Scalar(32, 200, 32);

// define the state of the robot from the video feed
enum arm_states_e {
  // arm_states
  NO_WAVE,
  START_WAVE,
  FAST_WAVE
};
#define EATER_THRESHOLD 5  // whan count of objects are above set to fast

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
YoloAIParametors::YoloAIParametors()
{
    yolo_version = YOLOV5;
    input_width = DEFAULT_AI_INPUT_WIDTH;
    input_height = DEFAULT_AI_INPUT_HEIGHT;
    score_threshold = DEFAULT_SCORE_THRESHOLD;
    nms_threshold = DEFAULT_NMS_THRESHOLD;
    confidence_thresgold = DEFAULT_CONF_THRESHOLD;

    clssification_size=0;
    onnx_file_name = std::string();
    names_file_name = std::string();
}

YoloAIParametors::~YoloAIParametors()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// put the label in the box or out of the box in the visual
#define LABEL_IN_BOX false
#define LABEL_OUT_BOX true
//void draw_label(Mat& input_image, string label, int left, int top, float _font_scale, int _thickness_font, int _fontface, cv::Scalar _rect_color)
void draw_label(Mat& input_image, string label, int left, int top, CvFontParam cfp, cv::Scalar _rect_color)
{
    // Display the label at the top of the bounding box.
    int baseLine;
    Size label_size = getTextSize(label, cfp.face, cfp.scale, cfp.thickness, &baseLine);
    top = max(top, label_size.height);

    if (LABEL_IN_BOX)
    {
        rectangle(input_image, Point(left, top), Point(left + 105, top + 11), _rect_color, -1);
        putText(input_image, label, Point(left, top + label_size.height), cfp.face, cfp.scale, BLACK, cfp.thickness);
        //putText(input_image, label, Point(left, top + label_size.height), _fontface, _font_scale, YELLOW, _thickness_font);
    }
    if (LABEL_OUT_BOX)
    {
        rectangle(input_image, Point(left-1, top-11), Point(left + 105, top), _rect_color, -1);
        putText(input_image, label, Point(left, top -11 + label_size.height), cfp.face, cfp.scale, BLACK, cfp.thickness);
    }
}


// Structure for organizing data for each recognition
struct DetctionData
{
    float cx;
    float cy;
    // Box dimension.
    float w;
    float h;
    // Bounding box coordinates.
    int left;
    int top;
    int width;
    int height;

};

// detected object
struct DetectedObject
{
    cv::Rect bbox; // bounding box

    float cx;
    float cy;
    // Box dimension.
    float w;
    float h;

    float confidence; // confidence
    int classID;      // class ID
    int objectID;

    DetectedObject();
    DetectedObject(
        float _cx,
        float _cy,
        float _w,
        float _h,
        cv::Rect bbox, // bbox
        float confidence, // conf
        int classID, // class ID
        int objectID
    );
};

DetectedObject::DetectedObject()
{
    cx=0.0;
    cy=0.0;
    w=0.0;
    h=0.0;
    confidence = 0.0;
    classID = 0;
    objectID = 0;
    bbox = cv::Rect(0, 0, 0, 0);
}
DetectedObject::DetectedObject
(
    float _cx,
    float _cy,
    float _w,
    float _h,
    cv::Rect _bbox,      // bounding box rectangle
    float _confidence,   // confidence level
    int _classID,        // class ID
    int _objectID
)
{
    cx = _cx;
    cy = _cy;
    w = _w;
    h = _h;
    confidence = _confidence;
    classID = _classID;
    objectID = _objectID;
    bbox = _bbox;
}

// function to extract the object data 
int extract_object_data(
    vector<Mat>& _outputs, 
    const vector<string>& class_name, 
    vector<DetectedObject>& _objects, 
    int yolo_version,
    float _confidence_thresgold,
    float _x_factor, 
    float _y_factor
)
{
    int rows = 0;
    int dimensions = 0;
    float* _data = nullptr;

    if (yolo_version == YOLOV8)
    { 
        //https://github.com/ultralytics/ultralytics/issues/1852
        rows = _outputs[0].size[2];
        dimensions = _outputs[0].size[1];
        _outputs[0] = _outputs[0].reshape(1, dimensions);
        cv::transpose(_outputs[0], _outputs[0]);
        float* data = (float*)_outputs[0].data;               //Is it a pointer to the data area in float representation
        
        for (int i = 0; i < rows; ++i)
        {
            DetectedObject _obj;    //一Time data storage

            float* classes_scores = data + 4;
            cv::Mat scores(1, (int)class_name.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;
            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            if (maxClassScore > _confidence_thresgold)
            {
                //confidences.push_back((float));
                //class_ids.push_back(class_id.x);
                _obj.classID = class_id.x;
                _obj.confidence = maxClassScore;
                _obj.cx = data[0];
                _obj.cy = data[1];
                _obj.w = data[2];
                _obj.h = data[3];

                //Set the bounding box value according to the image size
                _obj.bbox.x = int((_obj.cx - 0.5 * _obj.w) * _x_factor);
                _obj.bbox.y = int((_obj.cy - 0.5 * _obj.h) * _y_factor);
                _obj.bbox.width = int(_obj.w * _x_factor);
                _obj.bbox.height = int(_obj.h * _y_factor);

                _objects.push_back(_obj);
            }
            data += dimensions;
        }
    }
    else if (yolo_version == YOLOV5)
    {
        rows = _outputs[0].size[1];                             // How many objects detected
        dimensions = _outputs[0].size[2];                       // dimensions of each object
        float* _data = (float*)_outputs[0].data;                // pointer to float represnetation of the frame data

        for (int i = 0; i < rows; ++i)
        {
            DetectedObject _obj;                                //一Time data storage
            _obj.confidence = _data[4];
            if (_obj.confidence >= _confidence_thresgold)
            {
                float* _classes_scores = _data + 5;
                Mat scores(1, (int)class_name.size(), CV_32FC1, _classes_scores);
                cv::Point class_id;
                double maxClassScore;
                minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);
                _obj.classID = class_id.x;

                _obj.confidence = _data[4];
                _obj.cx = _data[0];
                _obj.cy = _data[1];
                _obj.w = _data[2];
                _obj.h = _data[3];

                // Set the bounding box value according to the image size
                _obj.bbox.x = int((_obj.cx - 0.5 * _obj.w) * _x_factor);
                _obj.bbox.y = int((_obj.cy - 0.5 * _obj.h) * _y_factor);
                _obj.bbox.width = int(_obj.w * _x_factor);
                _obj.bbox.height = int(_obj.h * _y_factor);

                //_obj.bbox.x = max(_obj.bbox.x, 0);
                //_obj.bbox.y = max(_obj.bbox.y, 0);

                _objects.push_back(_obj);
            }
            _data += dimensions;
        }
    }

    return 0;
}

// perform pre processing of image
int pre_process(vector<Mat>& outputs,Mat& input_image, Net& net, float input_width= DEFAULT_AI_INPUT_WIDTH, float input_height= DEFAULT_AI_INPUT_HEIGHT)
{
    Mat blob;
    int ret = 0;
    
    //vector<Mat> outputs;
    
    if (false)
    {
        // A try is inserted under the call of this function
        _TRYCAT(blobFromImage(input_image, blob, 1.0/255.0, Size((int)input_width, (int)input_height), Scalar(), true, false));
        _TRYCAT(net.setInput(blob));
        // Forward propagate.
        _TRYCAT(net.forward(outputs, net.getUnconnectedOutLayersNames()));

    }
   //こっちNULLチェック どっちがいいのやら
    else 
    {
        if (input_image.data == NULL)
        {
            ret = 1;
            //LOGMSG("input_image.data is null");
        }
        else if (input_image.empty())
        {
            ret = 2;
            //LOGMSG("input_image.data is empty");
        }
        else
        {
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////
            //
            // https://github.com/opencv/opencv/issues/23977
            // OpenCV YOLOv8 bounding box dimensions are 0 when performing discovery on ONNX model using CUDA build #23977
            // https://github.com/ultralytics/ultralytics/issues/3682
            // OpenCV 4.7.0
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////
            blobFromImage(input_image, blob, 1.0/255.0, Size((int)input_width, (int)input_height), Scalar(), true, false);
            net.setInput(blob);
            // Forward propagate.
            //LOGMSG2("outputs1", outputs);
            //TRYCAT(net.forward(outputs, net.getUnconnectedOutLayersNames()));
            net.forward(outputs, net.getUnconnectedOutLayersNames());
            //LOGMSG2("outputs2", outputs);
            ret = 0;
        }
    }    
    return ret;
}

// Set to false to display only the specified object
#define VIEW_ALL true 
//Output AI results to stream
//Drawing function omitted
//Should be integrated with post_process eventually
//Output the AI ​​result as a string to _ost
//Separate results with commasB
//_header is the header added at the beginning of the item; camera number, frame number, time, etc.
//#define CLASSIFICATION_SIZE 11
Mat post_process_str(
    YoloAIParametors yp,
    YoloFontsParam yfp,
    bool draw_image,                 // Normally, if set to true or false, no drawing processing will be performed.
    const Mat& input_image,          // Overwrite this Mat and return it
    vector<Mat>& outputs,            // AI classification information
    const vector<string>& class_name,
    int& number_of_persons, std::vector<std::string>& class_list_view, 
    std::string _header,             // header
    std::string& _ost,               // output string of infos
    int _version,                    //yolo version
	int& wave_hand_state
)
{
    // If nothing is detected, return it as is
    if (outputs.size() == 0)
        return input_image;

    cv::Mat output_image = input_image;

#ifdef _DEBUG
    if (1)
    {
        // Copy to temporary data as it is transformed inside the function
        vector<Mat> _tmp_outputs = outputs;
        vector<DetectedObject> _objects;
        extract_object_data(_tmp_outputs, class_name, _objects, yp.yolo_version, yp.score_threshold, input_image.cols / yp.input_width, input_image.rows / yp.input_height);
    }
#endif

    //std::vector<DetectedObject> _detections;
    //_detections=convertDetections(outputs, yp._confidence_thresgold );


    // Initialize vectors to hold respective outputs while unwrapping     detections.
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;
    // Resizing factor. Resize elements
    float x_factor = input_image.cols / yp.input_width;
    float y_factor = input_image.rows / yp.input_height;

    int rows = outputs[0].size[1];                     // How many objects detected?
    int dimensions = outputs[0].size[2];               // Number of elements in each object
    bool yolov8 = false;

    // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
    // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
    if (0)
    {
        if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8) gpuの場合は成り立たない
        {
            yolov8 = true;
            rows = outputs[0].size[2];
            dimensions = outputs[0].size[1];

            outputs[0] = outputs[0].reshape(1, dimensions);
            cv::transpose(outputs[0], outputs[0]);
        }
        float* data = (float*)outputs[0].data;         // pointer to float representation of the frame data
    }
    // 25200 for default size 640.
    // Iterate through 25200 detections.
    if (_version == YOLOV8)
    {
        //https://github.com/ultralytics/ultralytics/issues/1852
        rows = outputs[0].size[2];
        dimensions = outputs[0].size[1];
        outputs[0] = outputs[0].reshape(1, dimensions);
        cv::transpose(outputs[0], outputs[0]);

        float* data = (float*)outputs[0].data;        // pointer to float representation of the frame data
        for (int i = 0; i < rows; ++i)
        {
            float* classes_scores = data + 4;

            cv::Mat scores(1, (int)class_name.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double maxClassScore;

            minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

            if (maxClassScore > yp.score_threshold)
            {
                confidences.push_back((float)maxClassScore);
                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];

                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);

                int width = int(w * x_factor);
                int height = int(h * y_factor);

                boxes.push_back(cv::Rect(left, top, width, height));
            }
            data += dimensions;
        }
    }
    else if(_version == YOLOV5)                                   //yolov5
    {
        float* data = (float*)outputs[0].data;                           // pointer to float representation of the frame data
        for (int i = 0; i < rows; ++i)
        {
            //yolov5
            float confidence = data[4];
            // Discard bad detections and continue.
            if (confidence >= yp.confidence_thresgold)
            {
                float* classes_scores = data + 5;
                // Create a 1x85 Mat and store class scores of 80 classes.
                Mat scores(1, (int)class_name.size(), CV_32FC1, classes_scores);
                // Perform minMaxLoc and acquire the index of best class  score.
                Point class_id;
                double max_class_score;
                minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
                // Continue if the class score is above the threshold.
                if (max_class_score > yp.score_threshold)
                {
                    // Store class ID and confidence in the pre-defined respective vectors.
                    confidences.push_back(confidence);
                    class_ids.push_back(class_id.x);
                    // Center.
                    float cx = data[0];
                    float cy = data[1];
                    // Box dimension.
                    float w = data[2];
                    float h = data[3];
                    // Bounding box coordinates.
                    int left = int((cx - 0.5 * w) * x_factor);
                    int top = int((cy - 0.5 * h) * y_factor);
                    int width = int(w * x_factor);
                    int height = int(h * y_factor);
                    // Store good detections in the boxes vector.
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
            data += dimensions;
        }
    }
    ostringstream _os;
    Scalar RECTCOLOR;
    // Draw the enclosing rectangle
    vector<int> indices;
    // function of cudnn
    // The extracted id array is put into indices, a function that determines and extracts identity from the degree of overlap of detected objects
    cv::dnn::NMSBoxes(boxes, confidences, yp.score_threshold, yp.nms_threshold, indices);
	
	// initialze the counters for each frame
    number_of_persons = 0;
	int number_of_crop_eaters = 0;
	
    for (int i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        int left    = box.x      ;
        int top     = box.y      ;
        int width   = box.width  ;
        int height  = box.height ;

        bool draw_rect = VIEW_ALL;

        // read the class names from the output and check what they are
		//
        string object_name(class_name[class_ids[idx]]);

        int count_class_list_view = 0;
        // Rotate as many times as the number of object types you want to display
        while (count_class_list_view < class_list_view.size())
        {
            // If it is an object that you want to display, set a display flag
            if (object_name.compare(class_list_view[count_class_list_view]) == 0)
                draw_rect = true;
            count_class_list_view++;
        }

        // Added processing Counting and changing colors depending on the object
		//
		// check the class names if we have birds or rabbits wave hands
		//
		// when person or tractor or dog is seen we no longer need to wave the scarecrow robot
		//
		

        if (object_name.compare("person") == 0 ||          
            object_name.compare("driver") == 0) ||
			(object_name.compare("tractor") == 0 ||
            object_name.compare("dog") == 0)                         		// condition stops waving			
        {
			if (count_of_person == true) {
                number_of_persons++;
			}
            RECTCOLOR = BLUE;
        }
        else if (object_name.compare("bird") == 0) ||
                 object_name.compare("rabbit") == 0)                       // condition starts waving		
        {
		    number_of_crop_eaters++;
            RECTCOLOR = BLUEGREEN;				
        }
        else if (object_name.compare("forklift") == 0)
        {
            RECTCOLOR = KOIBLUE;
        }
        else if (object_name.compare("excavator") == 0 ||
            object_name.compare("bulldozer") == 0 ||
            object_name.compare("wheelloder") == 0 ||
            object_name.compare("grader") == 0)
        {
            RECTCOLOR = YELLOW;
        }
        else
            RECTCOLOR = DARKGREEN;
	
        if (draw_image && draw_rect)                                        // If you only write data, do not draw
        {
            // Draw bounding box.
            rectangle(output_image, Point(left, top), Point(left + width, top + height), RECTCOLOR, yfp.thickness_box);
            // Get the label for the class name and its confidence.
            string label = format("%.2f", confidences[idx]);
            label = class_name[class_ids[idx]] + ":" + label;
            // Draw class labels.
            draw_label(output_image, label, left, top, yfp.label, RECTCOLOR);
        }
        
        // Added processing Save the analysis results to the stream
        _os << _header << ","
            << i << ","
            << indices.size() << ","
            << idx << ","
            << class_ids[idx] << ","
            << confidences[idx] << ","
            << class_name[class_ids[idx]] << ","
            << yp.score_threshold << ","
            << yp.nms_threshold << ","
            << yp.confidence_thresgold << ","
            << left << ","
            << top << ","
            << (left + width) << ","
            << (top + height) << ","
            << width << ","
            << height << ","
            << yp.onnx_file_name << ","
            << yp.names_file_name << ","
            << input_image.cols << ","
            << input_image.rows << endl;

    }
	if (indices.size() == 0) {                           // nothing is detected
	    wave_hand_state = NO_WAVE;                       // turn the waving robot off 
	} else if (number_of_crop_eaters <= EATER_THRESHOLD) && (number_of_persons == 0) {
        wave_hand_state = START_WAVE;
    } else if (number_of_crop_eaters >= EATER_THRESHOLD) && (number_of_persons == 0) {
        wave_hand_state = FAST_WAVE;	
    } else if (number_of_persons >= 1) {
	    wave_hand_state = NO_WAVE;                       // turn the waving robot off 
    }		
    _ost = _os.str().c_str();
    return input_image;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
YoloObjectDetection::YoloObjectDetection()
{
    YP.yolo_version = YOLOV5;
    YP.input_width = DEFAULT_AI_INPUT_WIDTH;
    YP.input_height = DEFAULT_AI_INPUT_HEIGHT;
    YP.score_threshold = (float)DEFAULT_SCORE_THRESHOLD;
    YP.nms_threshold = (float)DEFAULT_NMS_THRESHOLD;
    YP.confidence_thresgold = (float)DEFAULT_CONF_THRESHOLD;

    YFP.label.scale      = (float)FONT_SCALE_LABEL;
    YFP.label.face       = FONT_FACE_LABEL;
    YFP.label.thickness  = THICKNESS_FONT_LABEL;

    YFP.person.scale       = (float)FONT_SCALE_PERSON;
    YFP.person.face        = FONT_FACE_PERSON;
    YFP.person.thickness   = THICKNESS_FONT_PERSON;

    YFP.time.scale        = (float)FONT_SCALE_TIME;
    YFP.time.face         = FONT_FACE_TIME;
    YFP.time.thickness    = THICKNESS_FONT_TIME;

    YFP.thickness_box = THICKNESS_BOX;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize AI
// Load the distinguished name list and onnx file
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int YoloObjectDetection::init_yolo(
//    string filepath_of_names, 
//    string filepath_of_onnx,
//    //int clssification_size, 
//    float _iw, float _ih, float _sc_th,float _nms_th, float _conf_th, 
//    bool _count_of_person, 
//    bool __count_of_time)
// When selecting a GPU device, include cuda_runtime.h and specify cudaSetDevice(int device).
// #include <cuda_runtime.h>
// cudaSetDevice(0);
int YoloObjectDetection::init_yolo(YoloAIParametors yp, bool _count_of_person, bool __count_of_time)
{
    //USES_CONVERSION;
    YP = yp;

    count_of_person = _count_of_person;
    count_of_time = __count_of_time;

    ifstream ifs(YP.names_file_name);
    string line;

    //wostringstream _msg;
    ostringstream _msg;
    _msg << "names: " << YP.names_file_name << std::endl
        << "onnx: " << YP.onnx_file_name << std::endl
        << "input width: " << YP.input_width << std::endl
        << "input height: " << YP.input_height << std::endl;

    list_of_class.clear();
    YP.clssification_size = 0;
    while (getline(ifs, line))
    {
        // Read classification name from file
        list_of_class.push_back(line);
        ++YP.clssification_size;
    }
     // Load model. yolov8 causes an error in readNet or readNetFromONNX
    try
    {
        //cudaSetDevice(0);
        //LOGMSG("dnn::readNetFromONNX:" <<YP.onnx_file_name);
        net = cv::dnn::readNetFromONNX(YP.onnx_file_name);
        //net = cv::dnn::readNet(YP.onnx_file_name);

#ifdef _CUDA
        net.setPreferableBackend(DNN_BACKEND_CUDA);
        net.setPreferableTarget(DNN_TARGET_CUDA_FP16);
#else
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
#endif
    }
    catch (const cv::Exception& e)
    {
        const char* err_msg = e.what();
		std::cerr << e,what << std::endl;
        //LOGMSG(err_msg);
        return 0;
    }
    ifs.close();
    return 1;
}


vector<Mat>& YoloObjectDetection::_pre_process(Mat& input_image)
{
    if (input_image.empty() || input_image.data == NULL || input_image.cols == 0; input_image.rows == 0)
        return detections;

    pre_process(detections, input_image, net, YP.input_width, YP.input_height);

    //TRYCAT_CV_STD(detections = pre_process(input_image, net, _input_width, _input_height));

    return detections;
}

cv::Mat YoloObjectDetection::_post_process(
    bool draw_image,                          // Usually true. If set to false, the analysis data will only be written as text without drawing processing
    const cv::Mat& input_image, 
    std::string _header, std::string& _ost
)
{
    Mat img;
    if (input_image.data == NULL)
        return img;
    
    number_of_persons = 0;

    img = post_process_str(
        YP,
        YFP,
        draw_image,
        input_image.clone(),        // original image
        detections,
        list_of_class,             // List of classification names read from file
        number_of_persons, class_list_view,
        _header,                   // header
        _ost,                      // formatted output string of the data
        YP.yolo_version,
		wave_hand_state
        );

    string label = format("Count of Persons = %i ", number_of_persons);
    // Sometimes a null pointer occurs. The cause is unknown.
    if (img.data!=NULL)
    {
        putText(img, label, TEXT_POINT_PERSON, YFP.person.face, YFP.person.scale, BLACK, YFP.person.thickness + 1);
        putText(img, label, TEXT_POINT_PERSON, YFP.person.face, YFP.person.scale, BLUE, YFP.person.thickness);
         
        ostringstream _os;
        _os << "AI:" << YP.onnx_file_name;
    }
    else
    {
        cout << "null pointer:"<<__FILE__<<":"<<__LINE__<<endl;
    }

    // Display processing time if asked to do so by the count_of_time flag
    if (count_of_time)
    {
        vector<double> layersTimes;
        double freq = getTickFrequency() / 1000;
        double t = net.getPerfProfile(layersTimes) / freq;
        string label = format("Inference time = %.2f ms", t);
        putText(img, label, Point(20, 40), YFP.time.face, YFP.time.scale, RED, YFP.time.thickness);
    }
    return img;
}
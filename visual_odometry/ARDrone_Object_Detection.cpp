// --------------------------------------------------------------------------------
// main(up to 7, As defined in Keys below)
// Description  : Using ARDrone Object detection
// Return value : SUCCESS:0  ERROR:-1
//
// for ARDrone library ref :- https://github.com/puku0x/cvdrone/blob/master/samples
// ----------------------------------------------------------------------------------
#include "ardrone/ardrone.h"

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <ctype.h>
#include <algorithm>                  
#include <iterator>                    
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>


// this is for the openCV command line parser
static const char* Keys =
{ "{feature_detector   | 0       | feature detector algorithm }"
  "{ref_img            | "A.bmp" | image to search for }"
  "{help h usage       |         | there is one single parameter an integer representing the feature extraction alogrithm   }"
};

void features(cv::Mat &target, cv::Mat &scene, cv::Mat &t_gray, cv::Mat &s_gray, cv::Mat &dst, int num)
{
    // Frequency for time calculations
    double f = 1000.0 / cv::getTickFrequency();

    int64 time_s;                    // start time
    double time_detect;              // detection end time
    double time_match;               // matching end time
	 
    // Feature point detection and feature calculation
    cv::Ptr<cv::Feature2D> feature;
    std::stringstream ss;

    // cv::GFTTDetector	goodFeaturesToTrack (feature check out)	-
    // cv::AgastFeatureDetector	AGAST (feature check out)	-
    // cv::FastFeatureDetector	FAST (feature check out)	-
    // cv::MSER	MSER (feature check out)	-
    // cv::BRISK	BRISK	Binary
    // cv::KAZE	KAZE	Scale
    // cv::ORB	ORB	Binary
    // cv::AKAZE	A-KAZE	Binary
    // The extra module is something like this:

    // Name	Method	Representation of features
    // cv::xfeatures2d::StarDetector	StarDetector (Feature point description)	-
    // cv::xfeatures2d::MSDDetector	MSD (feature check out)	-
    // cv::xfeatures2d::LATCH	LATCH (Feature description)	Binary
    // cv::xfeatures2d::LUCID	LUCID (Feature Description)	?
    // cv::xfeatures2d::BriefDescriptorExtractor	BRIEF (Feature Description)	Binary
    // cv::xfeatures2d::DAISY	DAISY (Feature description)	Real vector
    // cv::xfeatures2d::FREAK	FREAK (Feature description)	Binary
    // cv::xfeatures2d::SIFT	SIFT	Real vector
    // cv::xfeatures2d::SURF	SURF	Real vector
    switch (num)
    {
        case 0:
        feature = cv::xfeatures2d::SIFT::create();
        ss << "SIFT";
        break;
		
        case 1:
        feature = cv::xfeatures2d::SURF::create();
        ss << "SURF";
        break;
		
        case 2:
        feature = cv::ORB::create();
        ss << "ORB";
        break;
		
        case 3:
        feature = cv::AKAZE::create();
        ss << "A-KAZE";
        break;
		
        case 4:
        feature = cv::BRISK::create();
        ss << "BRISK";
        break;
		
        case 5:
        feature = cv::KAZE::create();
        ss << "KAZE";
        break;
		
        case 6:
        feature = cv::GFTTDetector::create();
        ss << "GFTT";
        break;
		
        case 7:
        feature = cv::AgastFeatureDetector::create();
        ss << "AGAST";
        break;
		
        case 8:
        feature = cv::FastFeatureDetector::create();
        ss << "FAST";
        break;
		
        case 9:
        feature = cv::MSER::create();
        ss << "MSER";
        break;
		
        case 10:
        feature = cv::xfeatures2d::StarDetector::create();
        ss << "STAR";
        break;
		
        case 11:
        feature = cv::xfeatures2d::MSDDetector::create();
        ss << "MSD";
        break;
		
        case 12:
        feature = cv::xfeatures2d::LATCH::create();
        ss << "LATCH";
        break;
		
        case 13:
        feature = cv::xfeatures2d::LUCID::create();
        ss << "LUCID";
        break;
		
        case 14:
        feature = cv::xfeatures2d::BriefDescriptorExtractor::create();
        ss << "BRIEF";
        break;
		
        case 15:
        feature = cv::xfeatures2d::DAISY::create();
        ss << "DAISY";
        break;
		
        case 16:
        feature = cv::xfeatures2d::FREAK::create();
        ss << "FREAK";
        break;
		
        default:
        std::cout << "invalid case entered" << std::endl;
        break;
    }
    std::cout << "--- using feature algorithm " << ss.str() << "） ---" << std::endl;

    //********************************************
    // Keypoint detection and feature description
    //********************************************
    std::vector<cv::KeyPoint> kpts1, kpts2;
    cv::Mat desc1, desc2;

    feature->detectAndCompute(t_gray, cv::noArray(), kpts1, desc1);

    time_s = cv::getTickCount();                                              //  Start
    feature->detectAndCompute(s_gray, cv::noArray(), kpts2, desc2);
    time_detect = (cv::getTickCount() - time_s)*f;                            //  Stop

    if (desc2.rows == 0){
        std::cout << "WARNING: Unable to detect feature pointsず" << std::endl;
        return;
    }

    //*******************
    // matcher
    //*******************
    auto matchtype = feature->defaultNorm(); // SIFT, SURF: NORM_L2
                                             // BRISK, ORB, KAZE, A-KAZE: NORM_HAMMING
    cv::BFMatcher matcher(matchtype);
    std::vector<std::vector<cv::DMatch >> knn_matches;


    time_s = cv::getTickCount(); // Start
    // Top 2 points
    matcher.knnMatch(desc1, desc2, knn_matches, 2);
    time_match = (cv::getTickCount() - time_s)*f; //  Stop


    //***************************************
    // Narrow down the corresponding points
    //***************************************
    const auto match_par = .6f; //Threshold of corresponding points
    std::vector<cv::DMatch> good_matches;

    std::vector<cv::Point2f> match_point1;
    std::vector<cv::Point2f> match_point2;

    for (size_t i = 0; i < knn_matches.size(); ++i) {
        auto dist1 = knn_matches[i][0].distance;
        auto dist2 = knn_matches[i][1].distance;

        //Keep the good points (from the similarity between the most similar point and the next similar point)
        if (dist1 <= dist2 * match_par) {
            good_matches.push_back(knn_matches[i][0]);
            match_point1.push_back(kpts1[knn_matches[i][0].queryIdx].pt);
            match_point2.push_back(kpts2[knn_matches[i][0].trainIdx].pt);
        }
    }

    //Homography matrix estimation
    cv::Mat masks;
    cv::Mat H;
    if (match_point1.size() != 0 && match_point2.size() != 0) {
        H = cv::findHomography(match_point1, match_point2, masks, cv::RANSAC, 3.f);
    }

    //Extract only the corresponding points used in RANSAC
    std::vector<cv::DMatch> inlierMatches;
    for (auto i = 0; i < masks.rows; ++i) {
        uchar *inlier = masks.ptr<uchar>(i);
        if (inlier[0] == 1) {
            inlierMatches.push_back(good_matches[i]);
        }
    }
    //Display feature points
    cv::drawMatches(target, kpts1, scene, kpts2, good_matches, dst);

    //Display only corresponding points of inliers
    cv::drawMatches(target, kpts1, scene, kpts2, inlierMatches, dst);

    if (!H.empty()) {

        //
        // Get the corner from the target object image (target object is "detected")
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cv::Point2f(.0f, .0f);
        obj_corners[1] = cv::Point2f(static_cast<float>(target.cols), .0f);
        obj_corners[2] = cv::Point2f(static_cast<float>(target.cols), static_cast<float>(target.rows));
        obj_corners[3] = cv::Point2f(.0f, static_cast<float>(target.rows));

        // Estimate the projection onto the scene
        std::vector<cv::Point2f> scene_corners(4);
        cv::perspectiveTransform(obj_corners, scene_corners, H);

        // Connect lines between corners (mapped object in scene - scene image)
        float w = static_cast<float>(target.cols);
        cv::line(dst, scene_corners[0] + cv::Point2f(w, .0f), scene_corners[1] + cv::Point2f(w, .0f), cv::Scalar(0, 255, 0), 4);
        cv::line(dst, scene_corners[1] + cv::Point2f(w, .0f), scene_corners[2] + cv::Point2f(w, .0f), cv::Scalar(0, 255, 0), 4);
        cv::line(dst, scene_corners[2] + cv::Point2f(w, .0f), scene_corners[3] + cv::Point2f(w, .0f), cv::Scalar(0, 255, 0), 4);
        cv::line(dst, scene_corners[3] + cv::Point2f(w, .0f), scene_corners[0] + cv::Point2f(w, .0f), cv::Scalar(0, 255, 0), 4);
    }

    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 40), cv::FONT_HERSHEY_SIMPLEX, beta-.1, cv::Scalar(255, 255, 255), 1, CV_AA);
    ss.str("");
    ss << "Detection & Description";
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 70), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(0, 255, 255), 1, CV_AA);
    ss.str("");
    ss << "Time: " << time_detect << " [ms]";
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 95), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(0, 255, 255), 1, CV_AA);
    ss.str("");
    ss << "Matching";
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 120), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(0, 255, 255), 1, CV_AA);
    ss.str("");
    ss << "Time: " << time_match << " [ms]";
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 145), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(0, 255, 255), 1, CV_AA);

    ss.str("");
    ss << "--Matches--";
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 170), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(255, 255, 0), 1, CV_AA);
    ss.str("");
    ss << "Good Matches: " << good_matches.size();
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 190), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(255, 255, 0), 1, CV_AA);

    ss.str("");
    ss << "Inlier: " << inlierMatches.size();
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 220), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(255, 255, 0), 1, CV_AA);

    ss.str("");
    auto ratio = .0;
    if (good_matches.size() != .0)
        ratio = inlierMatches.size()*1.0 / good_matches.size();
    ss << "Inlier ratio: " << ratio;
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 240), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(255, 255, 0), 1, CV_AA);


    ss.str("");
    ss << "Target KeyPoints: " << kpts1.size();
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 270), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(255, 0, 255), 1, CV_AA);
    ss.str("");
    ss << "Scene KeyPoints: " << kpts2.size();
    cv::putText(dst, ss.str(), cv::Point(10, target.rows + 290), cv::FONT_HERSHEY_SIMPLEX, beta - .1, cv::Scalar(255, 0, 255), 1, CV_AA);

    std::cout << "Detection time: " << time_detect << " [ms]" << std::endl;
    std::cout << "match time: " << time_match << " [ms]" << std::endl;
    std::cout << "Good Matches: " << good_matches.size() << std::endl;
    std::cout << "Inlier: " << inlierMatches.size() << std::endl;
    std::cout << "Inlier ratio: " << ratio << std::endl;
    std::cout << "target Keypoints: " << kpts1.size() << std::endl;
    std::cout << "scene Keypoints: " << kpts2.size() << std::endl;
    std::cout << "target match_points: " << match_point1.size() << std::endl;
    std::cout << "scene match_points: " << match_point2.size() << std::endl;

}

int main(int argc, char *argv[])
{
	// parse the command line for set-up
    cv::CommandLineParser parser(argc, argv, Keys);
    int feature_choice = parser.get<int>("feature_detector");	
    cv::String ref_image = parser.get<cv::String>("ref_img");
    feature_choice = feature_choice % 17;                            // force to be in range if invalid
	
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}

	if (!parser.check())
	{
		parser.printErrors();
		return -1;
	}
	
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Create a window
    cv::namedWindow("match");
    cv::resizeWindow("match", 0, 0);
	
    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "%" << std::endl;

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to Play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;

    // read calibration data
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("camera.xml", CV_STORAGE_READ);
    fs["intrinsic"] >> cameraMatrix;
    fs["distortion"] >> distCoeffs;

    // if you want to keep swapping the cameras 
	int LeftRight = 1;         // set if we are working with 2 cams Left and Right in stereo vision 	
	int numOfCams = 4;         // default number
	
    // define the CV matrices and read the reference image
    cv::Mat img1, img2, bgi1, bgi2, first_frame, dst;
    cv::undistort(cv::imread(ref_image), img1, cameraMatrix, distCoeffs);
    cv::cvtColor(img1, bgi1, COLOR_BGR2GRAY);

    int mode = 0;
		
    // Main loop
    while (1) {
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Take off / Landing 
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)    vx =  1.0;
        if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)  vr =  1.0;
        if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
        if (key == 'j') vy =  1.0;
        if (key == 'l') vy = -1.0;
        if (key == 'q') vz =  1.0;
        if (key == 'a') vz = -1.0;

        // Change camera with manual input from keyboard
        if (key == 'c') ardrone.setCamera(++mode % numOfCams);

        // Get the scene image to compare with the reference image 
        first_frame = ardrone.getImage();
        cv::undistort(first_frame, img2, cameraMatrix, distCoeffs);	
        cv::cvtColor(img2, bgi2, COLOR_BGR2GRAY);

        // For SIFT or SURF, initModule_nonfree() is required
        if( 2 > feature_choice ){
            cv::initModule_nonfree();
        }
	
        // Feature inspection and feature description are in the features2d module (basic) and xfeatures2d module (Extra).
        // The basic modules include the following for the selection of feature
        //
        // Name	                     Method	               Representation of features
        // cv::GFTTDetector	         goodFeaturesToTrack  (feature check out)	-
        // cv::AgastFeatureDetector	 AGAST                (feature check out)	-
        // cv::FastFeatureDetector	 FAST                 (feature check out)	-
        // cv::MSER	                 MSER                 (feature check out)	-
        // cv::BRISK	             BRISK	              Binary
        // cv::KAZE	                 KAZE	              Scale
        // cv::ORB	                 ORB	              Binary
        // cv::AKAZE	             A-KAZE	              Binary
        // The extra module is something like this:

        // Name	                            Method	          Representation of features
        // cv::xfeatures2d::StarDetector	StarDetector      (Feature point description)	-
        // cv::xfeatures2d::MSDDetector	    MSD               (feature check out)	-
        // cv::xfeatures2d::LATCH	        LATCH             (Feature description)	Binary
        // cv::xfeatures2d::LUCID	        LUCID             (Feature Description)	?
        // cv::xfeatures2d::BriefDescriptorExtractor BRIEF    (Feature Description)	Binary
        // cv::xfeatures2d::DAISY	        DAISY             (Feature description)	Real vector
        // cv::xfeatures2d::FREAK	        FREAK             (Feature description)	Binary
        // cv::xfeatures2d::SIFT	        SIFT	          Real vector
        // cv::xfeatures2d::SURF	        SURF	          Real vector
        //
        features(img1, img2, bgi1, bgi2, dst, feature_choice);
		
        // move the frone according to the keys 
        ardrone.move3D(vx, vy, vz, vr);
		if (LeftRight != 1) {
		    // set last frame to first frame when we cycle we will read the next one as second.
            // img1 = img2;
			img2.copyTo(img1);
		}
    }

    // See you
    ardrone.close();

    return 0;
}


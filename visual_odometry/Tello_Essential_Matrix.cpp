// --------------------------------------------------------------------------------
// main(up to 7, As defined in Keys below)
// Description  : Using ARDrone calculate essential matrix from 2 camera frames
//                Estimation of camera position and posture by 5-point algorithm
// Return value : SUCCESS:0  ERROR:-1
//
// for Tello Drone library ref :- // https://github.com/hakuturu583/tello_driver
//
#include <tello_driver/udp_client.h>
#include <tello_driver/tello_command_builder.h>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

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
{ "{feature_detector   | "ORB" | feature detector algorithm }"
"{desc_extractor       | "ORB"  | descriptor extractor algorithm        }"
"{descriptor_matcher   | "BruteForce"  | descriptor matcher algorithm        }"
"{matcher_method       | "match"  | matcher method        }"
"{cross_check          | 0 | cross check with NORM_HAMMING as matcher  }"
"{knn_num_good         | 2 | when using knn matcher number of good points  }"
"{radius_thresh        | 1 | when using radius matcher threshold  }"
"{help h usage         |   | print this message   }"
};

using namespace tello_driver::tello_commands;
using namespace cv;
using namespace std;
	
int main(int argc, char *argv[])
{
	// parse the command line for set-up
    cv::CommandLineParser parser(argc, argv, Keys);
    cv::String feature_algo = parser.get<cv::String>("feature_detector");
    cv::String desc_ext_algo = parser.get<cv::String>("desc_extractor");
    cv::String desc_match_algo = parser.get<cv::String>("descriptor_matcher");	
    cv::String matcher_method = parser.get<cv::String>("matcher_method");
    int num_good_pts = parser.get<int>("knn_num_good");	
    double radius_thresh = parser.get<double>("radius_thresh");	
    int use_cross_check = parser.get<int>("cross_check");
	
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

    // Check selection of feature point algorithm FAST，FASTX，STAR，SIFT，SURF，ORB，BRISK，
    // MSER，GFTT，HARRIS，Dense，SimpleBlob
    if( "FAST" != feature_algo && "FASTX" != feature_algo && "STAR" != feature_algo && "SIFT" != feature_algo &&
	    "SURF" != feature_algo && "ORB" != feature_algo && "BRISK" != feature_algo && "MSER" != feature_algo &&
        "GFTT" != feature_algo && "HARRIS" != feature_algo && "Dense" != feature_algo && "SimpleBlob" != feature_algo ) {
        std::cout << "inavlid feature algorithm specified " << std::endl;
        std::cout << "valid = FAST，FASTX，STAR，SIFT，SURF，ORB，BRISK, MSER，GFTT，HARRIS，Dense，SimpleBlob" << std::endl;
        return -1;
    }
    // For SIFT or SURF, initModule_nonfree() is required
    if( "SIFT" == algorithm || "SURF" == algorithm ){
        cv::initModule_nonfree();
    }
	// DescriptorExtractor	SIFT，SURF，BRIEF，BRISK，ORB，FREAK
    if( "SIFT" != desc_ext_algo && "SURF" != desc_ext_algo && "BRIEF" != desc_ext_algo && "BRISK" != desc_ext_algo &&
	    "ORB" != desc_ext_algo && "FREAK" != desc_ext_algo ) {
        std::cout << "invalid DescriptorExtractor algorithm specified " << std::endl;
        std::cout << "valid = SIFT，SURF，BRIEF，BRISK，ORB，FREAK" << std::endl;
        return -1;
    }
    // DescriptorMatcher	BruteForce，BruteForce-L1，BruteForce-SL2，
    // BruteForce-Hamming，BruteForce-Hamming(2)，
    // FlannBased
	// DescriptorExtractor	SIFT，SURF，BRIEF，BRISK，ORB，FREAK
    if( "BruteForce" != desc_match_algo && "BruteForce-L1" != desc_match_algo && "BruteForce-SL2" != desc_match_algo && "BruteForce-Hamming" != desc_match_algo &&
	    "BruteForce-Hamming(2)" != desc_match_algo  && "FlannBased" != desc_match_algo  ) {
        std::cout << "invalid DescriptorMatcher algorithm specified " << std::endl;
        std::cout << "valid = BruteForce，BruteForce-L1，BruteForce-SL2，BruteForce-Hamming，BruteForce-Hamming(2)， FlannBased" << std::endl;
        return -1;
    }
    // matcher method alogrithm match, knn, radius		
    if( "match" != matcher_method && "knn" != matcher_method && "radius" != matcher_method   ) {
        std::cout << "invalid MatcherMethod algorithm specified valid = match knn or radius" << std::endl;
        return -1;
    }
	
    // Tello.Drone class
    tello_driver::UdpClient client;
    tello_driver::TelloCommandBuilder builder;

    // Initialize
	client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildInitCommand());

    // Create a window
    cv::namedWindow("match");
    cv::resizeWindow("match", 0, 0);
	
    // Battery

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to Play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'T' -- Takeoff                   *" << std::endl;
    std::cout << "*    'L' -- Landing                   *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;

    // turn on video camera and start capturing frames
	client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_VIDEO,builder.buildMoveCommand(mission_commands::STREAM_ON,0));
    cv::VideoCapture capture{"udp://0.0.0.0:11111", cv::CAP_FFMPEG};
	cv::VideoCapture cap;	
    capture >> cap;		
	cv::Mat c_frame;
    cap >> c_frame;	
	
    // read calibration data
    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("camera.xml", CV_STORAGE_READ);
    fs["intrinsic"] >> cameraMatrix;
    fs["distortion"] >> distCoeffs;

    // if you want to keep swapping the cameras (the drone doesnt have 2 cameras, so this is fixed as is)
    int swapCamsOn = 0;        // set to 1 if you want to rotate cameras when not in LeftRight mode i.e. difference between conscutive pictures  
	int LeftRight = 0;         // set if we are working with 2 cams Left and Right in stereo vision 	
	int numOfCams = 1;         // default number
	if (LeftRight == 1) {
       numOfCams = 2;          // these should be a left right stereo pair
    }	
	
    // define the CV matrices we will use to read and correct the image using the cal data
    cv::Mat imgA, imgB, first_frame, sec_frame;
	if (LeftRight != 1) {
        // Get first image
        first_frame = c_frame;
        cv::undistort(first_frame, imgA, cameraMatrix, distCoeffs);	
    }
	
    // Main loop
    while (1) {
		
		// video process
        capture >> cap;		
        cap >> c_frame;	
	
        // Key input
        int key = cv::waitKey(33);
        if (key == 0x1b) break;

        // Take off / Landing 
        if (key == 'T') {
            client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildTakeOffCommand());
        } else if (key == 'L') {
			client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildLandCommand());
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

        // Get next image
	    if (LeftRight == 1) {
            // Get first image
            first_frame = c_frame;
            cv::undistort(first_frame, imgA, cameraMatrix, distCoeffs);	
        }
		if ((swapCamsOn == 1) || (LeftRight == 1)) {                    // thres is no meaning of swapCamera but if you put it on it will take consqutive frames
           capture >> cap;		
           cap >> c_frame;	
		}
        sec_frame = c_frame;
        cv::undistort(sec_frame, imgB, cameraMatrix, distCoeffs);	

        // ORB
        // cv::OrbFeatureDetector detector;
        // cv::OrbDescriptorExtractor extractor;
        // Feature extraction
        // detector.detect(imgA, keypointsA);
        // detector.detect(imgB, keypointsB);
        // extractor.compute(imgA, keypointsA, descriptorsA);
        // extractor.compute(imgB, keypointsB, descriptorsB);
		
		// FeatureDetector	FAST，FASTX，STAR，SIFT，SURF，ORB，BRISK，
        // MSER，GFTT，HARRIS，Dense，SimpleBlob
        cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( feature_algo );             // example also SURF
		
        // DescriptorExtractor	SIFT，SURF，BRIEF，BRISK，ORB，FREAK
        cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create( desc_ext_algo );   // example also SURF
		
        // DescriptorMatcher	BruteForce，BruteForce-L1，BruteForce-SL2，
        // BruteForce-Hamming，BruteForce-Hamming(2)，
        // FlannBased
        //
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create( desc_match_algo );       // also BruteForce BruteForce-Hamming BruteForce-L1 BruteForce-SL2 FlannBased NORM_HAMMING
        cv::BFMatcher matcher_crosschk(cv::NORM_HAMMING, true);                                          // we can also use this matcher instead crossCheck=True 
		
        // Feature point detection
        cv::Mat descriptorsA, descriptorsB;
        std::vector<cv::KeyPoint> keypointsA, keypointsB;
        detector->detect(imgA, keypointsA);
        detector->detect(imgB, keypointsB);	
        extractor->compute(imgA, keypointsA, descriptorsA);
        extractor->compute(imgB, keypointsB, descriptorsB);

        // matching
        std::vector<cv::DMatch> matches;

        if (matcher_method == "match") {
           if (use_cross_check == 0) {
              matcher->match(descriptorsA, descriptorsB, matches);                          // Find the best points
           } else {
              matcher_crosschk.match(descriptorsA, descriptorsB, matches);
           }
	    } else if (matcher_method == "knn") {
           if (use_cross_check == 0) {
              matcher->knnMatch(descriptorsA, descriptorsB, matches, num_good_pts);         // find top num_good_pts good points
           } else {
              matcher_crosschk.knnMatch(descriptorsA, descriptorsB, matches, num_good_pts);
           }		   
	    } else {
           if (use_cross_check == 0) {
              matcher->radiusMatch(descriptorsA, descriptorsB, matches, radius_thresh);     // Find points whose distance is less than or equal to the threshold value in the feature description space
           } else {
              matcher_crosschk.radiusMatch(descriptorsA, descriptorsB, radius_thresh);
           }	
        }		        
	   
       // Maximum/minimum distance
       double max_dist = 0; double min_dist = DBL_MAX;
       for (int j = 0; j < (int)matches.size(); j++){ 
         double dist = matches[j].distance;
         if (dist < min_dist) min_dist = dist;
         if (dist > max_dist) max_dist = dist;
       }
 
       // Keep only good pairs
       double cutoff = 5.0 * (min_dist + 1.0);
       std::set<int> existing_trainIdx;
       std::vector<cv::DMatch> matches_good;
       for (int j = 0; j < (int)matches.size(); j++) { 
            if (matches[j].trainIdx <= 0) matches[j].trainIdx = matches[j].imgIdx;
            if (matches[j].distance > 0.0 && matches[j].distance < cutoff) {
                if (existing_trainIdx.find(matches[j].trainIdx) == existing_trainIdx.end() && matches[j].trainIdx >= 0 && matches[j].trainIdx < (int)keypointsB.size()) {
                   matches_good.push_back(matches[j]);
                   existing_trainIdx.insert(matches[j].trainIdx);
                }
            }
       }
 
       // display
       cv::Mat matchImage;
       cv::drawMatches(imgA, keypointsA, imgB, keypointsB, matches_good, matchImage, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
       cv::imshow("match", matchImage);
 
       // Sufficient corresponding points 5 pairs or more required
       if (matches_good.size() > 100) {
            // stereo pair
            std::vector<cv::Point2f>ptsA, ptsB;
            for (int j = 0; j < (int)matches_good.size(); j++) {
               ptsA.push_back(keypointsA[matches_good[j].queryIdx].pt);
               ptsB.push_back(keypointsB[matches_good[j].trainIdx].pt);
            }
 
            // Focal length and lens principal point
            double fovx, fovy, focal, pasp;
            cv::Point2d pp;
            cv::calibrationMatrixValues(cameraMatrix, cv::Size(imgA.cols, imgA.rows), 0.0, 0.0, fovx, fovy, focal, pp, pasp);
 
            // Calculate the fundamental matrix using the 5-point algorithm
            cv::Matx33d E = cv::findEssentialMat(ptsA, ptsB, focal, pp);
            std::cout << E << std::endl;
        }
 
        // move the drone according to the keys 
        if (vx > 0) {	
		    client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::UP,vx));
        } else {
	        client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::DOWN,abs(vx)));
	    }
        if (vr > 0) {
            client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::LEFT,vr));
        } else {
	        client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::RIGHT,abs(vr)));
	    }
        if (vy > 0) {	
            client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::FORWARD,vy));
        } else {
            client.send(tello_driver::TELLO_IP,tello_driver::TELLO_PORT_CMD,builder.buildMoveCommand(move_commands::BACKWARD,abs(vy)));
        }		
		if (LeftRight != 1) {
		    // set last frame to first frame when we cycle we will read the next one as second.
            // imgA = imgB;
			imgB.copyTo(imgA);
		}
    }

    // See you
    ardrone.close();

    return 0;
}


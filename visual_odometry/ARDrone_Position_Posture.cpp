// --------------------------------------------------------------------------------
// main(up to 7, As defined in Keys below)
// Description  : Using ARDrone Estimation of camera position and posture by 5-point algorithm
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
{ "{feature_detector   | "SURF" | feature detector algorithm }"
"{desc_extractor       | "SURF"  | descriptor extractor algorithm        }"
"{descriptor_matcher   | "BruteForce"  | descriptor matcher algorithm        }"
"{matcher_method       | "match"  | matcher method        }"
"{cross_check          | 0 | cross check with NORM_HAMMING as matcher  }"
"{knn_num_good         | 2 | when using knn matcher number of good points  }"
"{radius_thresh        | 1 | when using radius matcher threshold  }"
"{help h usage         |   | print this message   }"
};

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
    int swapCamsOn = 0;        // set to 1 if you want to rotate cameras when not in LeftRight mode i.e. difference between conscutive pictures  
	int LeftRight = 1;         // set if we are working with 2 cams Left and Right in stereo vision 	
	int numOfCams = 4;         // default number
	if (LeftRight == 1) {
       numOfCams = 2;          // these should be a left right stereo pair
    }	
	
    // define the CV matrices we will use to read and correct the image using the cal data
    cv::Mat img1, img2, first_frame, sec_frame;
	if (LeftRight != 1) {
        // Get first image
        first_frame = ardrone.getImage();
        cv::undistort(first_frame, img1, cameraMatrix, distCoeffs);	
    }

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

        // Get next image
	    if (LeftRight == 1) {
            // Get first image
            first_frame = ardrone.getImage();
            cv::undistort(first_frame, img1, cameraMatrix, distCoeffs);	
        }
		if ((swapCamsOn == 1) || (LeftRight == 1)) {
			ardrone.setCamera(++mode % numOfCams);                      // change to the next camera
		}
        sec_frame = ardrone.getImage();
        cv::undistort(sec_frame, img2, cameraMatrix, distCoeffs);	
	
		// FeatureDetector	FAST，FASTX，STAR，SIFT，SURF，ORB，BRISK，
        // MSER，GFTT，HARRIS，Dense，SimpleBlob
        cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( feature_algo );             // example also SURF
		
        // DescriptorExtractor	SIFT，SURF，BRIEF，BRISK，ORB，FREAK
        cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::DescriptorExtractor::create( desc_ext_algo );   // example also SURF
		
        // DescriptorMatcher	BruteForce，BruteForce-L1，BruteForce-SL2，
        // BruteForce-Hamming，BruteForce-Hamming(2)，
        // FlannBased
        //
		// Type	                    Method
        // BruteForce	            L2 Norm - Full Search
        // BruteForce-L1	        L1 Norm - Full Search
        // BruteForce-Hamming	    Humming Distance ・ Full search
        // BruteForce-Hamming(2)	Humming Distance ・ Full search
        // FlannBased	            flann- Nearest Neighbor search
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create( desc_match_algo );       // also BruteForce BruteForce-Hamming BruteForce-L1 BruteForce-SL2 FlannBased NORM_HAMMING
        //
        // Type	        Method	             Available features
        // NORM_L1	    L1 Norm	             SIFT, SURF, etc.
        // NORM_L2	    L2 norm	             SIFT, SURF, etc.
        // NORM_HAMMING	Humming distance	 ORB, BRISK, BRIEF, etc.
        // NORM_HAMMIN2	Humming distance	 ORB
		//
		// In the DescriptorMatcher, only the matching of image 1 → image 2 is performed. The method of matching image 2 → image 1 is also 
		// performed, and the matching result is only the one that matches in both results is called cross check.Doing this may 
		// reduce erroneous responses
        cv::BFMatcher matcher_crosschk(cv::NORM_HAMMING, true);                                          // we can also use this matcher instead crossCheck=True 
		
        // Feature point detection
        cv::Mat descriptors1, descriptors2;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        detector->detect(img1, keypoints1);
        detector->detect(img2, keypoints2);	
        extractor->compute(img1, keypoints1, descriptors1);
        extractor->compute(img2, keypoints2, descriptors2);

        // matching
        // Search for corresponding points
        std::vector<cv::DMatch> dmatch;
        std::vector<cv::DMatch> dmatch12, dmatch21;

        // Method	    Method
        // match	    Find the best points
        // knnMatch	    Find the top k good points
        // radiusMatch	Find points whose distance is less than or equal to the threshold value in the feature description space
        if (matcher_method == "match") {
           if (use_cross_check == 0) {
              matcher->match(descriptors1, descriptors2, dmatch12);                          // Find the best points  for img1 -> img2
              matcher->match(descriptors1, descriptors2, dmatch21);                          // Find the best points  for img2 -> img1
           } else {
              matcher_crosschk.match(descriptors1, descriptors2, dmatch12);
              matcher_crosschk.match(descriptors1, descriptors2, dmatch21);
           }
	    } else if (matcher_method == "knn") {
           if (use_cross_check == 0) {
              matcher->knnMatch(descriptors1, descriptors2, dmatch12, num_good_pts);         // find top num_good_pts good points
              matcher->knnMatch(descriptors1, descriptors2, dmatch21, num_good_pts);         // find top num_good_pts good points
           } else {
              matcher_crosschk.knnMatch(descriptors1, descriptors2, dmatch12, num_good_pts);
              matcher_crosschk.knnMatch(descriptors1, descriptors2, dmatch21, num_good_pts);
           }		   
	    } else {
           if (use_cross_check == 0) {
              matcher->radiusMatch(descriptors1, descriptors2, dmatch12, radius_thresh);     // Find points whose distance is less than or equal to the threshold value in the feature description space
              matcher->radiusMatch(descriptors1, descriptors2, dmatch21, radius_thresh);
           } else {
              matcher_crosschk.radiusMatch(descriptors1, descriptors2, dmatch12, radius_thresh);
              matcher_crosschk.radiusMatch(descriptors1, descriptors2, dmatch21, radius_thresh);
           }	
        }		        
	   

        for (size_t i = 0; i < dmatch12.size(); ++i)
        {
            //Verify that the results of img1 -> img2 and img2 -> img1 match
            cv::DMatch m12 = dmatch12[i];
            cv::DMatch m21 = dmatch21[m12.trainIdx];

            if (m21.trainIdx == m12.queryIdx)
                dmatch.push_back( m12 );
        }

        // Estimate the underlying matrix if there are enough corresponding points
        if (dmatch.size() > 5)
        {
            std::vector<cv::Point2d> p1;
            std::vector<cv::Point2d> p2;
            // Extract the associated feature points and convert them to coordinates when the focal length is 1.0
            for (size_t i = 0; i < dmatch.size(); ++i)
            {
                cv::Mat ip(3, 1, CV_64FC1);
                cv::Point2d p;
 
                ip.at<double>(0) = keypoints1[dmatch[i].queryIdx].pt.x;
                ip.at<double>(1) = keypoints1[dmatch[i].queryIdx].pt.y;
                ip.at<double>(2) = 1.0;
                ip = K.inv()*ip;
                p.x = ip.at<double>(0);
                p.y = ip.at<double>(1);
                p1.push_back( p );

                ip.at<double>(0) = keypoints2[dmatch[i].trainIdx].pt.x;
                ip.at<double>(1) = keypoints2[dmatch[i].trainIdx].pt.y;
                ip.at<double>(2) = 1.0;
                ip = K.inv()*ip;
                p.x = ip.at<double>(0);
                p.y = ip.at<double>(1);
                p2.push_back( p );
            }

            cv::Mat essentialMat = cv::findEssentialMat(p1, p2);
            std::cout << "Essential Matrix\n" << essentialMat << std::endl;

            cv::Mat r, t;
            cv::recoverPose(essentialMat, p1, p2, r, t);
            std::cout << "R:\n" << r << std::endl;
            std::cout << "t:\n" << t << std::endl;
        }
			
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


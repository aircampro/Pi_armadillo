// -------------------------------------------------------------------------------------------------------------------
// main(up to 7, As defined in Keys below)
// Description  : Using ARDrone calculate essential matrix from 2 camera frames
//                Estimation of camera position and posture by 5-point algorithm
// Return value : SUCCESS:0  ERROR:-1
//
// for ARDrone library ref :- https://github.com/puku0x/cvdrone/blob/master/samples
// ---------------------------------------------------------------------------------------------------------------------

/*
CMakeLists.txt is as shown here it assumes a subdirectory contains this source and requires installation of openCV

PCL viewer needs
PCL (Point Cloud Library) 1.9.1
Boost 1.70.0
VTK 8.2.0 version

===========================================================================================================================
cmake_minimum_required(VERSION 3.9 FATAL_ERROR)

if (APPLE)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.11")
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
	
project(ARDrone_Point_Cloud)

find_package(VTK 8.2.0 REQUIRED)
find_package(PCL 1.2 REQUIRED)              // maybe needs this newer version ? -----> find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_package(Boost 1.70.0 REQUIRED COMPONENTS
    date_time coroutine date_time filesystem graph log_setup program_options random regex serialization system thread timer
)

include(${VTK_USE_FILE})
include_directories(${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ARDrone_Point_Cloud src/ARDrone_Point_Cloud.cpp)
target_link_libraries (ARDrone_Point_Cloud ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARY_DIRS})
============================================================================================================================
*/
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
#include <cmath>

//#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

// define how you want to visualizer this data
#define _USE_RAB_WRAPPER                                    // to use the rab wrapper to pcl for pointcloud visualization
//#define _SIMPLE_VIEWR                                       to use simple pcl viewer

std::string my_rab_data = "./pose_pointcloud.rab";

// this is for the openCV command line parser
static const char* Keys =
{ "{feature_detector   | "ORB" | feature detector algorithm }"
"{desc_extractor       | "ORB"  | descriptor extractor algorithm        }"
"{descriptor_matcher   | "NORM_HAMMING"  | descriptor matcher algorithm        }"
"{matcher_method       | "match"  | matcher method        }"
"{cross_check          | 1 | cross check with NORM_HAMMING as matcher  }"
"{knn_num_good         | 2 | when using knn matcher number of good points  }"
"{radius_thresh        | 1 | when using radius matcher threshold  }"
"{help h usage         |   | print this message   }"
};

// used for translation of the matrix 
#define DEGREE_TO_RADIAN(deg) ( (PI*deg)/180.0f )                               // convert degrees to radians
#define RADIAN_TO_DEGREE(rad) ( (180.0f*rad)/PI )                               // convert radians to degrees
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

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
	int numOfCams = 4;         // default number
	
    // define the CV matrices we will use to read and correct the image using the cal data
    cv::Mat imgA, got_frame;

    int mode = 0;
    int num_pics = 15;                  // number of pictures we will take
    int pic_counter = num_pics;
    std::vector<cv::Mat> images;	

    cv::Matx33d R_AB;
    cv::Matx31d t_AB;
    R_AB << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
    t_AB << 0.0, 0.0, 0.0;

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

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

        // Get [num_pics] images then process that data
		if (pic_counter > 0) {
            got_frame = ardrone.getImage();
            cv::undistort(got_frame, imgA, cameraMatrix, distCoeffs);	
			images.push_back(imgA);
            --pic_counter;
		} else {
			pic_counter = num_pics;
		}

        // process the photos taken	
        if (pic_counter == num_pics) {
		
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
			for (int i = 1; i < (int)images.size(); i++) {
				cv::Mat descriptorsA, descriptorsB;
                std::vector<cv::KeyPoint> keypointsA, keypointsB;
                detector->detect(images[i-1], keypointsA);
                detector->detect(images[i],   keypointsB);
                extractor->compute(images[i-1], keypointsA, descriptorsA);
                extractor->compute(images[i],   keypointsB, descriptorsB);

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
                       matcher_crosschk.radiusMatch(descriptorsA, descriptorsB, matches, radius_thresh);
                    }	
                }

                // Maximum/minimum distance
                double min_dist = DBL_MAX;
                for (int j = 0; j < (int)matches.size(); j++) { 
                   double dist = matches[j].distance;
                  if (dist < min_dist) min_dist = (dist < 1.0) ? 1.0 : dist;
                }
 
                // Keep only good pairs
                double cutoff = 5.0 * min_dist;
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
                cv::drawMatches(images[i-1], keypointsA, images[i], keypointsB, matches_good, matchImage, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                cv::imshow("match", matchImage);
                cv::waitKey(1);

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
 
                    // camera pose
                    cv::Matx33d R;
                    cv::Matx31d t;
                    cv::recoverPose(E, ptsA, ptsB, R, t, focal, pp);
 
                    // calculate position and orientation
                    Eigen::Affine3f view;
                    Eigen::Matrix4f _t;
                    R_AB = R_AB * R.t();
                    t_AB = t_AB - R_AB * t;
                    _t << R_AB(0,0), R_AB(0,1), R_AB(0,2), t_AB(0,0),
                    R_AB(1,0), R_AB(1,1), R_AB(1,2), t_AB(1,0),
                    R_AB(2,0), R_AB(2,1), R_AB(2,2), t_AB(2,0);
                    PRINT_MAT(R_AB);
                    std::cout << "Position pitch " << _t(0,3) << " roll " << _t(1,3) << " yaw " << _t(2,3) << std::endl;  // obtained from 4th column
                    std::cout << "Orientation " << _t(0,0) << " , " << _t(0,1)  << " , " << _t(0,2) << std::endl;         // obtained from first three columns  
                    std::cout << "            " << _t(1,0) << " , " << _t(1,1)  << " , " << _t(1,2) << std::endl;
                    std::cout << "            " << _t(2,0) << " , " << _t(2,1)  << " , " << _t(2,2) << std::endl;	
                    // use ros libraries to translate and send the pose via ROS
                    tf2::Matrix3x3 rot_robot(_t(0,0), _t(0,1), _t(0,2), _t(1,0), _t(1,1), _t(1,2), _t(2,0), _t(2,1), _t(2,2));    /* obtained from first three columns */
                    tf2::Vector3 position(_t(0,3), _t(1,3), _t(2,3));                                                             /* obtained from 4th column */
                    tf2::Quaternion orientation;                     
                    rot_robot.getRotation(orientation);                                                                           // from 3x3 matrix
                    geometry_msgs::PoseStamped out;
                    tf2::toMsg(position, out.position);
                    tf2::toMsg(orientation, out.orientation);
                    if(ros::ok()){
                        if( current_state.mode != "OFFBOARD" &&
                            (ros::Time::now() - last_request > ros::Duration(5.0))){
                            if( set_mode_client.call(offb_set_mode) &&
                               offb_set_mode.response.mode_sent){
                               ROS_INFO("Offboard enabled");
                            }
                            last_request = ros::Time::now();
                        } else {
                            if( !current_state.armed &&
                                (ros::Time::now() - last_request > ros::Duration(5.0))){
                                if( arming_client.call(arm_cmd) &&
                                    arm_cmd.response.success){
                                    ROS_INFO("Vehicle armed");
                                }
                                last_request = ros::Time::now();
                            }
                        }

                        local_pos_pub.publish(out);

                        ros::spinOnce();
                        rate.sleep();
                    }
		        }
            }
        }

	// masybe its --> viewer.spinLoop();
    // move the frone according to the keys 
    ardrone.move3D(vx, vy, vz, vr);
    }

    // See you
    ardrone.close();

    return 0;
}


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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/impl/io.hpp>

#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable: 4819)

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
// visualization library can be found here https://github.com/naoya-chiba/RabVisualizer/tree/master
#include "rabv.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

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
#define FMAX(max, val) (val >= max ? val : max)
#define PI (double) (4.0f * atan(1.0f)) 
double math_copysign(double val, double sign)
{
  double ret;
  if (val >= 0.0f)
    ret = (sign >= 0.0f ? val : -val);
  else
    ret = (sign >= 0.0f ? -val : val);
  return ret;
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

#ifdef _SIMPLE_VIEWR
    // initialise the point cloud viewer
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(2.0);
    viewer.initCameraParameters();
#endif

#ifdef _USE_RAB_WRAPPER
	// 1. Make an instance of rabv::Rab Class
	auto rab = rabv::Rab::create();	
	int count_points = 0;
	int min_num_of_points_to_view = 2;
#endif
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
		    std::cout << "Position pitch " << _t(0,3) << " roll " << _t(1,3) << " yaw " << _t(2,3) << std::endl;  // obtained from 4th column
		    std::cout << "Orientation " << _t(0,0) << " , " << _t(0,1)  << " , " << _t(0,2) << std::endl;         // obtained from first three columns  
		    std::cout << "            " << _t(1,0) << " , " << _t(1,1)  << " , " << _t(1,2) << std::endl;
		    std::cout << "            " << _t(2,0) << " , " << _t(2,1)  << " , " << _t(2,2) << std::endl;	
                    // conversion of the 3x3 orientation matrix to quaternion
                    double w = sqrt(FMAX(0.0f, 1.0f + _t(0,0) + _t(1,1) + _t(2,2) )) / 2.0f;
                    double x = sqrt(FMAX(0.0f, 1.0f + _t(0,0) - _t(1,1) - _t(2,2))) / 2.0f;
                    double y = sqrt(FMAX(0.0f, 1.0f - _t(0,0) + _t(1,1) - _t(2,2))) / 2.0f;
                    double z = sqrt(FMAX(0.0f, 1.0f - _t(0,0) - _t(1,1) + _t(2,2))) / 2.0f;
                    x = math_copysign(x, _t(2,1) - _t(1,2));
                    y = math_copysign(y, _t(0,2) - _t(2,0));
                    z = math_copysign(z, _t(1,0) - _t(0,1));
                    Eigen::Quaternionf quat(x, y, z, w);                                                                  // make quat 
		    // calculate the yaw
                    double q0 = w;
                    double q1 = x;
                    double q2 = y;
                    double q3 = z;
                    double quat_yaw = atan2(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));		
                    // calculate angle from w in quaternion
                    double angle = 2.0f * acos(w);
                    if (angle > PI) 
                    {
                        angle -= 2.0f * PI;
                    }
	            std::cout << "Calculated out of orientation matrix is yaw == " << quat_yaw << " angle == " << angle << std::endl;
                    // calculate euler angle from 3x3 matrices
                    double a11 = _t(0,0);
                    double a12 = _t(0,1);
                    double a13 = _t(0,2);
                    double a23 = _t(1,2);
                    double a33 = _t(2,2);   
                    double tx = atan2(-a23, a33);                                 // output Euler angle tx
                    double ty = atan2(a13, sqrt(1.0f-a13*a13));                   // output Euler angle ty
                    double tz = atan2(-a12, a11);                                 // output Euler angle tz
                    std::cout << "Orientation output Euler angle tx " << tx << " Euler angle ty " << ty << " Euler angle tz " << tz << std::endl;  // obtained from 4th column					
#ifdef _SIMPLE_VIEWR					
		    // display in point cloud viewer
                    view = _t;
                    viewer.addCoordinateSystem(1.0, view);
#endif	
#ifdef _USE_RAB_WRAPPER				
		      // use rab viewer to look at pointcloud of pose information
	              ++count_points;
		      pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		      sample_cloud->push_back(pcl::PointXYZ(_t(0,3), _t(1,3), _t(2,3)));
	              pcl::PointCloud<pcl::PointNormal>::Ptr sample_point_normal(new pcl::PointCloud<pcl::PointNormal>());
	              pcl::PointCloud<pcl::Normal>::Ptr sample_normal(new pcl::PointCloud<pcl::Normal>());
		      pcl::PointNormal pn;
		      pn.x = _t(0,3);                                                           // get the position from the 4th column 
		      pn.y = _t(1,3);
		      pn.z = _t(2,3);
		      const auto normal = Eigen::Vector3f(_t(0,3), _t(1,3), _t(2,3)).normalized();
		      pn.normal_x = normal.x();
		      pn.normal_y = normal.y();
		      pn.normal_z = normal.z();
		      sample_normal->push_back(pcl::Normal(normal.x(), normal.y(), normal.z()));
		      sample_point_normal->push_back(pn);

                      if (count_points > min_num_of_points_to_view) {
	                    // 2.1. Add a pointcloud
	                    rab->addCloud(
		                           "sample1",                     // Unique name of the pointcloud
		                           sample_cloud                   // Pointcloud
	                    );

	                    rab->addCloud(
		                            "sample2",                    // Unique name
		                            sample_cloud,                 // Pointcloud
		                            1,                            // Point size
		                            rabv::Color(255, 255, 128),   // Color (r, g, b)
		                            rabv::Point(2.0, 0.0, 0.0),   // Offset of tlanslation (x, y, z)
		                            rabv::Rotation(0.5, 0.0, 0.0) // Rotation (X-axis, Y-axis, Z-axis)
	                    );

	                   // 2.2. Add a pointcloud and normal together
	                   rab->addCloudNormal(
		                                  "sample3",                  // Unique name
		                                  sample_point_normal,        // Pointcloud and normals
		                                  1,                          // Point size
		                                  rabv::Color(255, 0, 128),   // Color (r, g, b)
		                                  rabv::Point(3.0, 0.0, 0.0), // Offset of tlanslation (x, y, z)
		                                  rabv::Rotation(),           // Rotation (none)
		                                  1,                          // Density of normals
		                                  0.05,                       // Length of normals
		                                  1,                          // Width of normals
		                                  rabv::Color(0, 255, 255)    // Color of normals
	                   );
	
	                    // 2.3. Add a normal after "addPointcloud"
	                    rab->addNormal(
		                                "sample1",                  // Unique name of the corresponding pointcloud
		                                sample_normal,              // Pointcloud
                                 		1,                          // Density of normals
                                		0.05,                       // Length of normals
                                		2,                          // Width of normals
                                		rabv::Color(255, 0, 128)    // Color of normals
                            );

	
	                    // 3.1. Add a coordinate system as world coordinate system
	                    rab->addCoordinateSystem(
		                                            "world",             // Unique name of the pointcloud or "world"
		                                            0.3                  // Scale factor
	                    );

	                    // 3.2. Add a coordinate system as pointcloud coordinate system
	                    rab->addCoordinateSystem(
		                                           "sample2",           // Unique name of the pointcloud or "world"
		                                           0.2                  // Scale factor
	                    );

	                    // 9. Visualze the rab data while the window is closed
	                    const auto& viewer1 = rabv::Viewer::create(
		                                                             "Pose Estimation PointCloud",	        // Title of viewer
		                                                             rab			                        // The Rab class data
	                    );
	                    viewer1->spinLoop();
					
	                    // 10.1. Save the rab data (without an instance of rabv::Writer)
	                    rabv::Writer::saveRabFile(
		                                            my_rab_data,        // Filename to save
		                                            rab                 // Rab data (rabv::Rab::Ptr)
	                    );

	                    // 10.2. Save the rab data (by using an instance of rabv::Writer)
	                    auto writer = rabv::Writer::create("./test2.rab");
	                    writer->setRab(rab);
	                    writer->save();
	
	                    // 11. Visualize from the writer's rab data
	                    //const auto& viewer2 = writer->visualize();
	                    //viewer2->spinLoop();
		   }
#endif	
                }
            }
        }
#ifdef _SIMPLE_VIEWR
	viewer.spin();
	// masybe its --> viewer.spinLoop();
#endif
        // move the frone according to the keys 
        ardrone.move3D(vx, vy, vz, vr);

    }


    // See you
    ardrone.close();

    return 0;
}


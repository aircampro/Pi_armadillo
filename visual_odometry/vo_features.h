/*

vo_features.h

The MIT License

Its the original code from AVI Singh with the feature detection alogrythms allowing selection

Copyright (c) 2015 Avi Singh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2\nonfree\nonfree.hpp>

#include <iostream>
#include <ctype.h>
#include <algorithm>                   // for copy
#include <iterator>                    // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 

//this function automatically gets rid of points for which tracking fails

  vector<float> err;					
  Size winSize=Size(21,21);																								
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0))	{
     		  	status.at(i) = 0;
     		  }
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}

     }

}

// Extract and match feature detection can use various algorithms SIFT SURF USURF BRISK ORB FAST FASTX etc...

// This is the original function it uses FAST with hard coded paramters
//
void featureDetection(Mat img_1, vector<Point2f>& points1)	{     //uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;                                         // Threshold for the difference in brightness between the center pixel and the pixels on the circumference around it.．
  bool nonmaxSuppression = true;                                   // Non-Maximum suppression. Suppress (all) values ​​that are not the maximum (= set the value to zero).
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);     // FAST algorythm for corner detection
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}

// Here we shall vary the brightness threshold by a parameter passed
// Threshold for the difference in brightness between the center pixel and the pixels on the circumference around it.．
//
void featureDetection2(Mat img_1, vector<Point2f>& points1, int bright_thres)	{          //uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  bool nonmaxSuppression = true;                                 // Non-Maximum suppression. Suppress (all) values ​​that are not the maximum (= set the value to zero).
  FAST(img_1, keypoints_1, bright_thres, nonmaxSuppression);     // FAST algorythm for corner detection
  KeyPoint::convert(keypoints_1, points1, vector<int>());        // return the key points as point cloud
  
  // Description of feature points uncomment if you want to display it
  //cv::Mat	img_dst;
  //cv::drawKeypoints( img_src, keypoints_1, img_dst );

  std::cout << "Number of feature points:" << keypoints_1.size() << std::endl;

  // Displaying images
  //cv::namedWindow( "Feature point extraction", CV_WINDOW_AUTOSIZE );
  //cv::imshow("feature point extraction", img_dst);
  //cv::waitKey( 0 ); 
}

// This function allows you to change the feature detector alogorithm
//
int featureDetection3(Mat img_1, vector<Point2f>& points1, string& algorithm)	{          //uses any selection of valid algorythm

  // Check selection of feature point algorithm
  if( "FAST" != algorithm && "FASTX" != algorithm && "STAR" != algorithm && "SIFT" != algorithm &&
	  "SURF" != algorithm && "ORB" != algorithm && "BRISK" != algorithm && "MSER" != algorithm &&
      "GFTT" != algorithm && "HARRIS" != algorithm && "Dense" != algorithm && "SimpleBlob" != algorithm ) {
      return -1;
  }
  
  // For SIFT or SURF, initModule_nonfree() is required
  if( "SIFT" == algorithm || "SURF" == algorithm ){
    cv::initModule_nonfree();
  }
  
  // FeatureDetector
  cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( algorithm );  

  // Feature point
  vector<KeyPoint> keypoints_1;
  
  // Get feature points using the chosen alogortyhm
  detector->detect( img_src, keypoints_1 );
  
  KeyPoint::convert(keypoints_1, points1, vector<int>());
  
  // Description of feature points uncomment if you want to display it
  //cv::Mat	img_dst;
  //cv::drawKeypoints( img_src, keypoints_1, img_dst );

  std::cout << "Number of feature points:" << keypoints_1.size() << std::endl;

  // Displaying images
  //cv::namedWindow( "Feature point extraction", CV_WINDOW_AUTOSIZE );
  //cv::imshow("feature point extraction", img_dst);
  //cv::waitKey( 0 ); 
  return 0;
}

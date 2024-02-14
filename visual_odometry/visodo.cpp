/*

Monocular Visual Odometry Example please also look at accompaning links which explain this method well

visodo.cpp

This is a re-production of the follwoing VO Visual Odometry code originally written by Avi Singh
We have added options to change the openCV Feature detector algorithm, and the brightness threshold 

WE have changed the directories to macth our down load of the kitti dataset. to be located in /home/mark
you can also produce your own
the original code can be found here 
https://github.com/avisingh599/mono-vo

purpose : Self-position estimation

This is how you get the kitti dataset

$ cd /home/mark
$ git clone https://github.com/PINTO0309/mono-vo.git
$ wget http://kitti.is.tue.mpg.de/kitti/data_odometry_gray.zip 
$ wget http://kitti.is.tue.mpg.de/kitti/data_odometry_calib.zip 
$ wget http://kitti.is.tue.mpg.de/kitti/data_odometry_poses.zip 
$ unzip data_odometry_gray.zip;unzip data_odometry_calib.zip;unzip data_odometry_poses.zip
$ rm data_odometry_gray.zip;rm data_odometry_calib.zip;rm data_odometry_poses.zip

the odometry data can also be found here each are different datasets which conatin the information 
used to train the model.
https://www.cvlibs.net/datasets/kitti/eval_odometry.php
each one is different they are for example grey scale, rgb color, melodyne lidar point clouds, 
https://www.cvlibs.net/datasets/kitti/eval_odometry.php
https://cs.nyu.edu/~silberman/datasets/
http://www.rawseeds.org/home/
http://www-personal.acfr.usyd.edu.au/nebot/victoria_park.htm
https://www.mrpt.org/malaga_dataset_2009
https://cvg.cit.tum.de/data/datasets/rgbd-dataset

watch nates video on understnading the methods
https://www.youtube.com/watch?v=SXW0CplaTTQ
https://www.youtube.com/watch?v=qdjxd7LSjF0

test it....

$ cd ~/mono-vo
$ wget http://kitti.is.tue.mpg.de/kitti/devkit_odometry.zip
$ unzip devkit_odometry.zip
$ rm devkit_odometry.zip
$ cd devkit/cpp
$ g++ -O3 -DNDEBUG -o evaluate_odometry evaluate_odometry.cpp matrix.cpp

It is based on OpenCV 3.0 and implements the Monocular Visual Odometry method (self-position recognition with a monocular camera).

Algorithm
We use Nister's 5-point algorithm to estimate the basic matrix, and we use FAST features and the Kanade-Lucas-Tomasi Feature Tracker (Lucas–Kanade method) for tracking.
See the report here and the blog post here for more information.

Note:
This project still has low relative scale estimation accuracy.Therefore, the scale information is obtained from the ground truth file of the KITTI dataset.

#####################
CMakeLists.txt is as shown here it assumes a subdirectory contains this source and requires installation of openCV

cmake_minimum_required(VERSION 3.9)
project( mono-vo )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
find_package( OpenCV 3.0 REQUIRED )
find_package(Boost 1.71.0 REQUIRED COMPONENTS
    date_time coroutine date_time filesystem graph log_setup program_options random regex serialization system thread timer
)
//find_package(Boost 1.71.0 REQUIRED COMPONENTS
//    program_options 
//)
include_directories(
   ${Boost_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
)
link_directories(
    ${Boost_LIBRARY_DIRS}
)

file(GLOB viso
    "src/*.h"
    "src/*.cpp"
)
add_executable( vo ${viso} )
target_link_libraries( vo ${OpenCV_LIBS Boost_LIBRARY_DIRS} )

#######################################################

Also
LSD-SLAM famous for SLAM by monocular camera is a direct method ↓
http://vision.in.tum.de/research/vslam/lsdslam

The source code for LSD-SLAM is open to the public, but I don't have enough knowledge to try it yet

The MIT License

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

#include "vo_features.h"

#include <boost/program_options.hpp>              // we chose to use boost parser you can just use arguments argv[1] if easier or tclap etc..

namespace po = boost::program_options;
using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
  
  string line;
  int i = 0;
  ifstream myfile ("/home/mark/Datasets/KITTI_VO/00.txt");
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}


int main( const int ac, const char* const * const av )	{

  // Define options
  po::options_description description("option");
  description.add_options()
  // ("Option name", "Argument (optional)", "Option description")
  ("bright_threshold, bt", po::value<int>()->default_value(80), "Threshold for the difference in brightness between the center pixel and the pixels on the circumference around it.．")
  ("algorithm, a", po::value<std::string>()->default_value("FAST"), "Selection of minutiae algorithm\nFAST, FASTX, STAR, SIFT, SURF, ORB, BRISK, MSER, GFTT, HARRIS, Dense, SimpleBlob")
  ("help, h", "usage : vo -a FASTX -bt 30");

  // Command Line Parsing
  po::variables_map vm;
  try {
      po::store(parse_command_line(ac, av, description), vm);
  } catch (po::error &e) {
  // Nonexistent option ・ throw an exception if the wrong type is specified
     std::cout << e.what() << std::endl;
     return -1;
  }
  po::notify(vm);

  // help, output help (description content) when h is entered
  if (vm.count("help")) {
    std::cout << description << std::endl;
    return 0;
  }

  // Option algorithm  
  std::string algo = "NONE";
  try {
  // Output of option string
    algo = vm["bright_threshold"].as<std::string>();
    std::cout << "selected brightness algorithm : " << algo << std::endl;
  } catch (boost::bad_any_cast &e) {
  // default Throw an exception if an option with no value is not specified
    std::cout << e.what() << std::endl;
    return -1;
  }
  
  // Option brightness threshold
  int bt = -99;
  try {
    bt = vm["bright_threshold"].as<int>();
    std::cout << "selected brightness thresjold : " << bt << std::endl;
  } catch (boost::bad_any_cast &e) {
  // default Throw an exception if an option with no value is not specified
    std::cout << e.what() << std::endl;
    return -1;
  }
  
  Mat img_1, img_2;
  Mat R_f, t_f;                                                         //the final rotation and tranlation vectors  

  ofstream myfile;
  myfile.open ("results1_1.txt");                                       // where we write the results

  double scale = 1.00;
  char filename1[200];
  char filename2[200];
  sprintf(filename1, "/home/mark/Datasets/KITTI_VO/00/image_2/%06d.png", 0);
  sprintf(filename2, "/home/mark/Datasets/KITTI_VO/00/image_2/%06d.png", 1);

  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 50);

  //  read the first two frames from the dataset
  Mat img_1_c = imread(filename1);
  Mat img_2_c = imread(filename2);

  if ( !img_1_c.data || !img_2_c.data ) { 
    std::cout<< " --(!) Error reading images " << std::endl; return -1;
  }

  // we work with grayscale images so convert them 
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

  // feature detection, tracking
  vector<Point2f> points1, points2;                                         //vectors to store the coordinates of the feature points
  // branch on the choice of feature detection
  std::string as_default("NONE");
  if ( algo == as_default ) && ( bt = -99 ) {
      featureDetection(img_1, points1);                                     //detect features in img_1 using FAST
  else if ( algo == as_default ) {
      featureDetection2(img_1, points1, bt);                                //detect features in img_1 using a specified brightness threshold to FAST
  else {
      featureDetection3(img_1, points1, algo);                                //detect features in img_1 using a specified algorithm  
  }
  vector<uchar> status;
  featureTracking(img_1,img_2,points1,points2, status);                 //track those features to img_2

  // TODO: add a fucntion to load these values directly from KITTI's calib files
  // WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
  double focal = 718.8560;
  cv::Point2d pp(607.1928, 185.2157);
  //recovering the pose and the essential matrix
  Mat E, R, t, mask;
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R, t, focal, pp, mask);

  Mat prevImage = img_2;
  Mat currImage;
  vector<Point2f> prevFeatures = points2;
  vector<Point2f> currFeatures;

  char filename[100];

  R_f = R.clone();
  t_f = t.clone();

  clock_t begin = clock();

  namedWindow( "Road facing camera", WINDOW_AUTOSIZE );                     // Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );                             // Create a window for display.

  Mat traj = Mat::zeros(600, 600, CV_8UC3);

  for(int numFrame=2; numFrame < MAX_FRAME; numFrame++)	{
  	sprintf(filename, "/home/mark/Datasets/KITTI_VO/00/image_2/%06d.png", numFrame);
    //cout << numFrame << endl;
  	Mat currImage_c = imread(filename);
  	cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
  	vector<uchar> status;
  	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

  	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);


   for(int i=0;i<prevFeatures.size();i++)	{                               //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
  		prevPts.at<double>(0,i) = prevFeatures.at(i).x;
  		prevPts.at<double>(1,i) = prevFeatures.at(i).y;

  		currPts.at<double>(0,i) = currFeatures.at(i).x;
  		currPts.at<double>(1,i) = currFeatures.at(i).y;
    }

  	scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

    cout << "Scale is " << scale << endl;

    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;

    } 	
    else {
      cout << "scale below 0.1, or incorrect translation" << endl;
    }
    
   // lines for printing results
   // myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

    // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
 	if (prevFeatures.size() < MIN_NUM_FEAT)	{
        //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
        //cout << "trigerring redection" << endl;
        if ( algo == as_default ) && ( bt = -99 ) {
          featureDetection(prevImage, prevFeatures);                                        // detect features in img_1 using FAST
        else if ( algo == as_default ) {
          featureDetection2(prevImage, prevFeatures, bt);                                   // detect features in img_1 using a specified brightness threshold to FAST
        else {
          featureDetection3(prevImage, prevFeatures, algo);                                 // detect features in img_1 using a specified algorithm  
        }
        featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
 	}

    prevImage = currImage.clone();
    prevFeatures = currFeatures;

    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 100;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    imshow( "Road facing camera", currImage_c );
    imshow( "Trajectory", traj );

    waitKey(1);

  }

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << "Total time taken: " << elapsed_secs << "s" << endl;

  //cout << R_f << endl;
  //cout << t_f << endl;

  return 0;
}
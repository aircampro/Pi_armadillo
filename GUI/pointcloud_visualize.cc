//
// This is a pcl point cloud viewer example code which can read XYZ and add XYZRGB for color or load XYZI type data from a lidar output and view
//
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/common/impl/io.hpp>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <string>

using namespace std;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.addCoordinateSystem (3.0);
    viewer.setBackgroundColor(1, 1, 1);                              // white
    cout << "viewerOneOff has set background color" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    cout << "viewerPsycho run" << std::endl;
}

// To Add Color map use the function below
typedef struct {
    double r,g,b;
} COLOR;

COLOR GetColor(double v,double vmin,double vmax)
{
   COLOR c = {255, 255, 255}; // white
   double dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv * 255;
      c.b = 255;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0 * 255;
      c.g = 255;
      c.b = 255 + 4 * (vmin + 0.25 * dv - v) / dv * 255;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv * 255;
      c.g = 255;
      c.b = 0 * 255;
   } else {
      c.r = 255;
      c.g = 255 + 4 * (vmin + 0.75 * dv - v) / dv * 255;
      c.b = 0 * 255;
   }

   return(c);
}

// to rotate the point cloud use the function below
// 
typedef struct {
    double array[3];
} POSITION;

POSITION rot(POSITION p0, double phi_deg, double sita_deg, double psi_deg)
{
    POSITION pz = {0, 0, 0};                               // position rotated around z axis
    POSITION py = {0, 0, 0};                               // position roteted around y axis
    POSITION px = {0, 0, 0};                               // position rotated around x axis

    double RAD = M_PI / 180;                               // convert degree to rad
    double phi = phi_deg * RAD;
    double sita = sita_deg * RAD;
    double psi = psi_deg * RAD;

    double Rx[3][3] = {
        {1, 0, 0},
        {0, cos(phi), sin(phi)},
        {0, -1 * sin(phi), cos(phi)}
    };

    double Ry[3][3] = {
        {cos(sita), 0, -1 * sin(sita)},
        {0, 1, 0},
        {sin(sita), 0, cos(sita)}
    };

    double Rz[3][3] = {
        {cos(psi), sin(psi), 0},
        {-1 * sin(psi), cos(psi), 0},
        {0, 0, 1}
    };

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            pz.array[i] = pz.array[i] + Rz[i][j] * p0.array[j];
        }
    }

   for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            py.array[i] = py.array[i] + Ry[i][j] * pz.array[j];
        }
    }

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            px.array[i] = px.array[i] + Rx[i][j] * py.array[j];
        }
    }
    return (px);
}


int main(int argc, char ** argv)
{   
    std::string rgbFile = "bunny.pcd";
	std::string liFile = "lidar.pcd";
    std::string roStr = "0";
    std::string rxa = "0";
    std::string rya = "0";
    std::string rza = "0";
	
    parse_argument(argc, argv, "-rgbFile", rgbFile);
    parse_argument(argc, argv, "-lidarFile", liFile);
    parse_argument(argc, argv, "-lidarRot", roStr);
    parse_argument(argc, argv, "-lidarRxAngle", rxa);
    parse_argument(argc, argv, "-lidarRyAngle", rya);
    parse_argument(argc, argv, "-lidarRzAngle", rza);
	
    int rotate_it = 0;	
	try {
		rotate_it = std::stoi(roStr);
	}
	catch (...) {
		std::cout << "option rotate 1=x 90, 2=y 90, 3=z 90" << std::endl;
	}

	double lphi_x = 0.0;
	double lsita_y = 0.0;
	double lpsi_z = 0.0;
	try {
		lphi_x = std::stod(rxa);
	}
	catch (...) {
		std::cout << "option rotate angle must be number" << std::endl;
	}
	try {
		lsita_y = std::stod(rya);
	}
	catch (...) {
		std::cout << "option rotate angle must be number" << std::endl;
	}
	try {
		lpsi_z = std::stod(rza);
	}
	catch (...) {
		std::cout << "option rotate angle must be number" << std::endl;
	}
	
    // set Types
    using CloudType = pcl::PointCloud<pcl::PointXYZ>;
    using CloudType2 = pcl::PointCloud<pcl::PointXYZRGB>;
    using CloudType3 = pcl::PointCloud<pcl::PointXYZI>;

    // make a rgb cloud pointer to store the modified points into	
    CloudType2::Ptr rgb_cloud(new CloudType2);

    // read p_cloud data into original cloud and then copy that data to rgb_cloud
    CloudType::Ptr original_cloud(new CloudType);
    pcl::io::loadPCDFile(rgbFile, *original_cloud);
    copyPointCloud(*original_cloud, *rgb_cloud);

    // if you just want to a color (e.g. blue) i.e. add RGB data to rgb_cloud
    const double rd = 0;
	const double gn = 40;
    const double bl = 255;
	
    for (int h = 0; h < rgb_cloud->height; h++) {
        for (int w = 0; w < rgb_cloud->width; w++) {
            pcl::PointXYZRGB &point = rgb_cloud->points[w + h * rgb_cloud->width];
            point.r = rd;
            point.g = gn;
            point.b = bl;
        }
    }

    pcl::visualization::CloudViewer viewer("PointCloudViewer");
    viewer.showCloud(rgb_cloud);
    
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    
    viewer.runOnVisualizationThread(viewerPsycho);
    
    while (!viewer.wasStopped())
    {
    
    }
	
    // now we a add Rainbow Point to rgb_cloud
    // get min & max value first
    double min = INT_MAX; double max = INT_MIN;
    for (int h = 0; h < rgb_cloud->height; h++) {
        for (int w = 0; w < rgb_cloud->width; w++) {
            pcl::PointXYZRGB &point = rgb_cloud->points[w + h * rgb_cloud->width];
            if (point.y < min) {
                min = point.y;
            }
            if (point.y > max) {
                max = point.y;
            }
        }
    }

    // add RGB data to rgb_cloud
    for (int h = 0; h < rgb_cloud->height; h++) {
        for (int w = 0; w < rgb_cloud->width; w++) {
            pcl::PointXYZRGB &point = rgb_cloud->points[w + h * rgb_cloud->width];
            COLOR c = GetColor(point.y, min, max);
            point.r = c.r;
            point.g = c.g;
            point.b = c.b;
        }
    }

    // Now we are Rotating the Point Cloud to rgb_cloud
    // rotate rgb_cloud around axis and name it rgb_cloud
	const double phi_x = 0.0;
	const double sita_y = 0.0;
	const double psi_z = 90.0;	
    for (int h = 0; h < rgb_cloud->height; h++) {
        for (int w = 0; w < rgb_cloud->width; w++) {
            pcl::PointXYZRGB &point = rgb_cloud->points[w + h * rgb_cloud->width];
            POSITION p0 = {point.x, point.y, point.z};
            POSITION p = rot(p0, phi_x, sita_y, psi_z);
            point.x = p.array[0];
            point.y = p.array[1];
            point.z = p.array[2];
        }
    }
		
    viewer.showCloud(rgb_cloud);
    
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    
    viewer.runOnVisualizationThread(viewerPsycho);
    
    while (!viewer.wasStopped())
    {
    
    }

    // now load intensity cloud from the lidar file
    CloudType::Ptr intense_cloud(new CloudType3);
    pcl::io::loadPCDFile(liFile, *intense_cloud);

    bool do_rot = false;
    switch (rotate_it) {
        case 1: 
		{
	        lphi_x = 90.0;
	        lsita_y = 0.0;
	        lpsi_z = 0.0;
        }
        break;
        case 2: 
		{
	        lphi_x = 0.0;
	        lsita_y = 90.0;
	        lpsi_z = 0.0;
        }
        break;
        case 3: 
		{
	        lphi_x = 0.0;
	        lsita_y = 0.0;
	        lpsi_z = 90.0;
        }
        break;
		default:
		{
			std::cout << "option rotate 1=x 90, 2=y 90, 3=z 90" << std::endl;
		}
		break;
    }	
	if ((lphi_x != 0) && ((lsita_y != 0) && (lpsi_z != 0))) {
        do_rot = true;
    }
    if (do_rot) {	
        for (int h = 0; h < intense_cloud->height; h++) {
            for (int w = 0; w < intense_cloud->width; w++) {
                pcl::PointXYZRGB &pointi = intense_cloud->points[w + h * intense_cloud->width];
                POSITION p0 = {pointi.x, pointi.y, pointi.z};
                POSITION p = rot(p0, phi_x, sita_y, psi_z);
                pointi.x = p.array[0];
                pointi.y = p.array[1];
                pointi.z = p.array[2];
            }
        }
	}
	
    viewer.showCloud(intense_cloud);
    
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    
    viewer.runOnVisualizationThread(viewerPsycho);
    
    while (!viewer.wasStopped())
    {
    
    }	
	
    // now delete the pointers
	delete [] original_cloud;
	delete [] intense_cloud;
	delete [] rgb_cloud;
	
    return 0;
}  
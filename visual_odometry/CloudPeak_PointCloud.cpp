/*
 * CloudPeeK a Point Cloud Viewer Application
 * 
 * This application loads a PCD (Point Cloud Binary Data) file asynchronously and streams it to a PointCloudViewer. 
 * Optionally, the points can be colored based on their distance from the origin.
 *
 * Main functionalities:
 *  - Asynchronous loading and processing of PCD files
 *  - Parallel computation for performance optimization
 *  - Batch-wise streaming of points to the viewer for real-time visualization
 *  - Optional color mapping based on point distance
 * 
 * Key components:
 *  - PointCloudViewer: A viewer single header that handles rendering the point cloud.
 *  - loadPCDAsyncToViewer: A function to load PCD data asynchronously and stream it to the viewer.
 * 
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 20.10.2024
 
 ref:- https://github.com/Geekgineer/CloudPeek/tree/main
 
 added point cloud generation routines ACP 21.08.24 to show example of this very good library
 
 */
#include "PointCloudViewer.hpp"  // ref:- get from https://github.com/Geekgineer/CloudPeek/tree/main
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <functional>            // For std::ref and std::cref
#include <tbb/tbb.h>

// uncomment if you want ot generate the data using the PCL Libraries 
// #define USE_PCL_LIBS

#if defined(USE_PCL_LIBS)
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// this function will make a pcd file using the pcl library
inline void make_pcd()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width =10;
    // define parameters
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    //generate point cloud
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("example_point_cloud.pcd", cloud);
    std::cout << "Saved " << cloud.points.size() << " data points to example_point_cloud.pcd" << std::endl;

    for (size_t i = 0; i < cloud.points.size(); ++i)
        std::cout << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
}

#else
	
#include <stdio.h>

struct pt_t {
	float x, y, z;
	pt_t() { ; }
	pt_t(float _x, float _y, float _z) {
		x = _x;
		y = _y;
		z = _z;
	}
};

// macro to make color integer
#define RGB(r, g, b) ((int(r) << 16) + (int(g) << 8) + int(b) )

// this function will make a pcd file without using any external libraries
inline void make_pcd() {

	//***********************************************************
	// make vector of the point types
	std::vector<pt_t> pts;

        // initialise variable for the data generation
	pt_t pt(7,23,35);
	float s = 10, b = 1.8, r = 20, dt = 0.0001;

       // generate the data
	for (int i = 0; i < 500000; i++) {
		pt = pt_t(
			pt.x - dt * s*(pt.x - pt.y),
			pt.y + dt*(-pt.x*pt.z + r*pt.x - pt.y),
			pt.z + dt*(pt.x*pt.y - b*pt.z)
		);
		pts.push_back(pt);
	}
	//***********************************************************
	// write the point cloud as a pcd file
	FILE *fp = fopen("example_point_cloud.pcd", "wb");
	// write header information
	fprintf(fp, "# .PCD v.7 - Point Cloud Data file format\n");
	fprintf(fp, "VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n");
	fprintf(fp, "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n", pts.size(), pts.size());
	// write the pointcloud data segment
	for (int i = 0; i < pts.size(); i++) {
		// iterate through the point cloud data 
		unsigned int col = RGB(pts[i].z * 5, 128 + 3*pts[i].x, 128 + 3 * pts[i].y);
		float xyz[3] = { pts[i].x, pts[i].y, pts[i].z};
		fwrite( xyz, sizeof(float), 3, fp);
		fwrite(&col, sizeof(unsigned int), 1, fp);
	}
	fclose(fp);
        std::cout << "Saved " << pts.size() << " data points to example_point_cloud.pcd" << std::endl;
	for (int i = 0; i < pts.size(); i++) {
		unsigned int col = RGB(pts[i].z * 5, 128 + 3*pts[i].x, 128 + 3 * pts[i].y);
		std::cout << "    " << pts[i].x << " " << pts[i].y << " " << pts[i].z << " color:" << col << std::endl;
	}	
}
#endif

// Optimized function to load PCD asynchronously with optional coloring
inline void loadPCDAsyncToViewer(const std::string& filename, PointCloudViewer& viewer, bool apply_coloring) {
    std::vector<Point> points;
    if (!readPCD(filename, points)) {
        std::cerr << "Failed to load PCD file: " << filename << '\n';
        return;
    }

    size_t total_points = points.size();

    // Determine the maximum distance for normalization
    float max_distance = 50.0f; // Default value meter
    if (apply_coloring) {
        // Compute the actual maximum distance in the dataset using parallel reduction
        max_distance = std::transform_reduce(
            std::execution::par,
            points.begin(),
            points.end(),
            0.0f,
            [](float a, float b) { return std::max(a, b); },
            [&](const Point& p) -> float {
                return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            }
        );

        // To avoid division by zero
        if (max_distance == 0.0f) max_distance = 1.0f;
    }

    // Split the points into smaller batches for streaming
    constexpr size_t batch_size = 10000; // Adjust as needed for real-time streaming
    size_t batches = (total_points + batch_size - 1) / batch_size;

    // Process and stream batches
    for (size_t i = 0; i < batches && viewer.isRunning(); ++i) {
        size_t start_idx = i * batch_size;
        size_t end_idx = std::min(start_idx + batch_size, total_points);
        std::vector<Point> batch(points.begin() + start_idx, points.begin() + end_idx);

        if (apply_coloring) {
            // Color the current batch of points
            colorPointsBasedOnDistance(batch, max_distance);
        }

        viewer.addPoints(batch);
        std::cout << "Added batch " << i + 1 << "/" << batches 
                  << " with " << batch.size() << " points.\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Simulate streaming delay
    }
}

// ----------- generate or read from command line pcl file and view it  -----------  
int main(int argc, char* argv[]) {
	
    make_pcd();	                                                 // make the pointcloud data
    std::string pcd_filename = "example_point_cloud.pcd";        // the generated filename
    bool apply_coloring = true;                                  // Apply color mapping to point cloud (configurable)

    // Override PCD filename if provided via command-line arguments
    if (argc > 1) {
        pcd_filename = argv[1]; 
    } else {
        std::cout << "No PCD file specified. Using default: " << pcd_filename << "\n";
    }

    // Initialize viewer with predefined configuration parameters
    PointCloudViewer viewer(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT, Config::WINDOW_TITLE);

    // Launch async thread to load and stream PCD data to the viewer
    std::thread loader_thread(loadPCDAsyncToViewer, pcd_filename, std::ref(viewer), apply_coloring);

    // Execute the main viewer loop (blocks until viewer window is closed)
    viewer.run();

    // Ensure the loader thread is properly joined before exiting
    if (loader_thread.joinable()) {
        loader_thread.join();
    }

    std::cout << "Viewer has been closed. Exiting application.\n";
    return 0;
}

/*
 * Convert .pcap file from Velodyne VLP-16 to .pcd file and view it using CloudPeeK
 *
 * Author: David Butterworth, 2019. ref :- https://github.com/dbworth/pcap_to_pcd/tree/master
 * using code from PCL and Tsukasa Sugiura.
 * ACP modified for CloudPeeK viewer ref:- https://github.com/Geekgineer/CloudPeek/tree/main
 *
 */
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
//original code used PCL visualizer #include <pcl/visualization/pcl_visualizer.h>

#include "hdl_grabber.h"                    // ref :- https://github.com/dbworth/pcap_to_pcd/tree/master
#include "vlp_grabber.h"

#include "PointCloudViewer.hpp"             // ref:- get from https://github.com/Geekgineer/CloudPeek/tree/main
#include <thread>
#include <chrono>
#include <cmath>
#include <functional>                       // For std::ref and std::cref
#include <tbb/tbb.h>

// macro to make color integer
#define RGB(r, g, b) ((int(r) << 16) + (int(g) << 8) + int(b) )

#define FRAME_CNT_START 35000

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

// Point Type:
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZ PointType;

const std::string makeFilename(const std::string& prefix, size_t idx, size_t idx_length, const std::string& suffix)
{
    std::ostringstream result;
    result << prefix <<
              std::setfill('0') <<
              //std::setw(a_max_length - a_prefix.length()) <<
              std::setw(idx_length) <<
              idx
              << suffix;
    return result.str();
}

bool g_saveframes = true;
unsigned int g_frame_counter = 0;
std::string g_filename = "dummy.pcd";

int main(int argc, char *argv[])
{
    // Command-Line Argument Parsing
    if (pcl::console::find_switch( argc, argv, "-help" ))
    {
        std::cout << "usage: " << argv[0]
                  << " [-ipaddress <192.168.1.70>]"
                  << " [-port <2368>]"
                  << " [-pcap <*.pcap>]"
                  << " [-saveframes]"
                  << " [-help]"
                  << std::endl;
        return 0;
    }

    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;
    
    pcl::console::parse_argument(argc, argv, "-ipaddress", ipaddress);
    pcl::console::parse_argument(argc, argv, "-port", port);
    pcl::console::parse_argument(argc, argv, "-pcap", pcap);
    pcl::console::parse_argument(argc, argv, "-saveframes", g_saveframes);

    std::cout << "-ipadress : " << ipaddress << std::endl;
    std::cout << "-port : " << port << std::endl;
    std::cout << "-pcap : " << pcap << std::endl;
    std::cout << "-saveframes : " << g_saveframes << std::endl;

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // PCL Visualizer commented from original code
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    //viewer->addCoordinateSystem( 3.0, "coordinate" );
    //viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    //viewer->initCameraParameters();
    //viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    // Point Cloud Color Handler
    //pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
    //const std::type_info& type = typeid(PointType);
    //if (type == typeid(pcl::PointXYZ))
    //{
    //    std::vector<double> color = { 255.0, 255.0, 255.0 };
    //    boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
    //    handler = color_handler;
    //}

    /*
    else if( type == typeid( pcl::PointXYZI ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
    }
    
    else if( type == typeid( pcl::PointXYZRGBA ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<PointType>() );
        handler = color_handler;
    }
    */

    //else
    //{
    //    throw std::runtime_error("This PointType is unsupported.");
    //}

    // Retrieved Point Cloud Callback Function and save it to a pcd file
    boost::mutex mutex;
    boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
        [&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr)
        {
            boost::mutex::scoped_lock lock(mutex);

            // Point Cloud Processing

            cloud = ptr;
            std::cerr << "Got new frame with " << cloud->points.size() << " points" << std::endl;
            for (int i = 0; i < cloud->points.size(); i++) {
                unsigned int col = RGB(cloud->points[i].z * 5, 128 + 3*cloud->points[i].x, 128 + 3 * cloud->points[i].y);
		std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " color: " << col << std::endl;
            }            
            // Save each frame to .pcd file
            if (g_saveframes)
            {
                g_filename = makeFilename("cloud", g_frame_counter, 5, ".pcd");
                if (frame_counter > FRAME_CNT_START)
                {
                    //pcl::io::savePCDFileBinary(g_filename, *cloud);
                    pcl::io::savePCDFile(g_filename, *cloud);
                }                
                g_frame_counter++;
                std::cerr << "Saved " << cloud->points.size () << " data points to " << g_filename << std::endl;
            }
        };

    // VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
    if (!pcap.empty())
    {
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );
    }
    else if (!ipaddress.empty() && !port.empty())
    {
        std::cout << "Capture from Sensor..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

    // always returns 0
    //std::cerr << "  " << grabber->getFramesPerSecond() << " fps" << std::endl;

    //while (!viewer->wasStopped())
    //{
         // Update Viewer
    //    viewer->spinOnce();

    //    boost::mutex::scoped_try_lock lock(mutex);
    //    if (lock.owns_lock() && cloud)
    //    {
             // Update Point Cloud
    //        handler->setInputCloud(cloud);
    //        if (!viewer->updatePointCloud(cloud, *handler, "cloud"))
    //        {
    //            viewer->addPointCloud(cloud, *handler, "cloud");
    //        }
    //    }
    //}

    int exit_grabber = 0;
    bool apply_coloring = true;                                  // Apply color mapping to point cloud (configurable)
    while (exit_grabber == 0) {
        boost::mutex::scoped_try_lock lock(mutex);
        if (lock.owns_lock() && cloud)
        {
             // Initialize viewer with predefined configuration parameters
             PointCloudViewer viewer(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT, Config::WINDOW_TITLE);

             // Launch async thread to load and stream PCD data to the viewer
             std::thread loader_thread(loadPCDAsyncToViewer, g_filename, std::ref(viewer), apply_coloring);

             // Execute the main viewer loop (blocks until viewer window is closed)
             viewer.run();

             // Ensure the loader thread is properly joined before exiting
             if (loader_thread.joinable()) {
                loader_thread.join();
		std::cout << "enter 0 to continue or 1 to exit") << std::endl;
		std::cin >> exit_grabber;
		// do it once ! exit_grabber = 1;
             }
	     if (exit_grabber != 0) {
                std::cerr << "Viewer has been closed. Exiting application! " << std::endl;
             }
          }			
    }
	
    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if (connection.connected())
    {
        connection.disconnect();
    }

    return 0;
}

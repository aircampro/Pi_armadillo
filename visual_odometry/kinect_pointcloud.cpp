// ref:-  kinect grabber uses this library https://github.com/UnaNancyOwen/KinectGrabber/tree/Kinect2Grabber/Sample
//
// added removal of Nan points
//
# define NOMINMAX
# define _SCL_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS

# include <iostream>
# include <Windows.h>
# include <pcl/visualization/cloud_viewer.h>
# include <pcl/features/integral_image_normal.h>
# include "kinect2_grabber.h"

typedef pcl::PointXYZ PointType;

void estimateNormal( pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals );
pcl::PointCloud<PointType>::Ptr removeNan(pcl::PointCloud<PointType>::Ptr target);

void main()
{
	try{
		pcl::visualization::PCLVisualizer viewer( "Point Cloud Viewer" );
		pcl::PointCloud<PointType>::Ptr cloud( new pcl::PointCloud<PointType> );
		boost::mutex mutex;
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals( new pcl::PointCloud < pcl::Normal > );

		boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function
			= [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr &new_cloud ){
			boost::mutex::scoped_lock lock( mutex );
			pcl::copyPointCloud( *new_cloud, *cloud );
		};

		pcl::Kinect2Grabber grabber;
		grabber.registerCallback( function );
		grabber.start();

		while ( !viewer.wasStopped() ){
			viewer.spinOnce();

			boost::mutex::scoped_try_lock lock( mutex );
			if ( ( cloud->size() != 0 ) && lock.owns_lock() ){
				cloud = removeNan(cloud); 
				auto ret = viewer.updatePointCloud( cloud, "cloud" );
				if ( !ret ){
					viewer.addPointCloud( cloud, "cloud" );
				}

				estimateNormal( cloud, cloud_normals );
				viewer.removePointCloud( "normals" );
				viewer.addPointCloudNormals<PointType, pcl::Normal>( cloud, cloud_normals, 100, 0.05, "normals" );
			}

			if ( GetKeyState( VK_ESCAPE ) < 0 ){
				break;
			}
		}
	}

	catch ( std::exception& ex ){
		std::cout << ex.what() << std::endl;
	}
}

// Estimate the normal vector.
void estimateNormal( pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals )
{
	// Normal Estimation Class
	pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>  ne;

	ne.setNormalEstimationMethod( ne.AVERAGE_DEPTH_CHANGE );
	ne.setMaxDepthChangeFactor( 0.01 );
	ne.setNormalSmoothingSize( 5.0 );
	ne.setInputCloud( cloud );
	ne.compute( *cloud_normals );
}

// remove Nan data point must not have valid datapoint as any one of the vector data points
pcl::PointCloud<PointType>::Ptr removeNan(pcl::PointCloud<PointType>::Ptr target){
  pcl::PointCloud<pcl::PointType>::Ptr cloud1(new pcl::PointCloud<PointType>);
  int n_point = target->points.size();
  for(int i=0;i<n_point; i++){
    pcl::PointType tmp_point;
    if(std::isfinite(target->points[i].x) || std::isfinite(target->points[i].y) || std::isfinite(target->points[i].z)){
      tmp_point.x = target->points[i].x;
      tmp_point.y = target->points[i].y;
      tmp_point.z = target->points[i].z;
      cloud1->points.push_back(tmp_point);
    }
  }
  cout << "number of valid points:" << cloud1->points.size() << endl;
  return cloud1;
}

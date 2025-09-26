/*

This file is a modified part of the EAGLE EYE project
ref:- https://github.com/MapIV/eagleye/tree/main-ros1/eagleye_rt
file :- velocity_estimator_node.cpp

objective :-
we want to use the realsense camera IMU's with madgewick filter as the imu input into the algorythm
models with IMUs: D435i, D455, D456, and D457 D435i and D455 are equipped with Bosch's BMI055, and D456 and D457 are equipped with BMI085.

Eagle Eye Gives us :-
Subscribed Topics
/nmea_sentence (nmea_msgs/Sentence)
/can_twist (geometry_msgs/TwistStamped)
/rtklib_nav (rtklib_msgs/RtklibNav)
/imu/data_raw (sensor_msgs/Imu)

Main Published Topics
/eagleye/fix (sensor_msgs/NavSatFix)
/eagleye/twist (ngeometry_msgs/TwistStamped)

This is how we publish and filter the realsense camera IMU data
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1
sudo apt install ros-humble-imu-filter-madgwick                 
# madgewick correction ROS package that estimates attitude (angle) with a Madgwick filter from acceleration, gyro, and geomagnetic sensor data and outputs it
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=False -p world_frame:=enu -p publish_tf:=False -r imu/data_raw:=/camera/imu
# name for filter subscriber is imu/data_raw published as imu/data in our example we taking it from the realsense camera 
name to VelocityEstimator class VelocityEstimate method must be of :- sensor_msgs::Imu
https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
the output from the madgewick is so as imu/data
https://wiki.ros.org/imu_filter_madgwick

Therefore we can use this data by doing the following and subscribing to that topic instead

*/
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

// un-comment to go back to original and not use the realsense IMU
#define _USE_REALSENSE

static ros::Publisher velocity_pub;
static ros::Publisher velocity_status_pub;

static rtklib_msgs::RtklibNav rtklib_nav_msg;
static nmea_msgs::Gpgga gga_msg;
static sensor_msgs::Imu imu_msg;

// from eagleye/eagleye_core/navigation/include/navigation/navigation.hpp
// protoype is 
// void VelocityEstimate(const sensor_msgs::Imu, const rtklib_msgs::RtklibNav, const nmea_msgs::Gpgga, geometry_msgs::TwistStamped*);
//
VelocityEstimator velocity_estimator;
static geometry_msgs::TwistStamped velocity_msg;

static std::string yaml_file;
#if defined(_USE_REALSENSE)
static std::string subscribe_imu_topic_name = "imu/data";                                      // take output from realsense imu via madgewick ROS filter
#else
static std::string subscribe_imu_topic_name = "imu/data_tf_converted";
#endif
static std::string subscribe_rtklib_nav_topic_name = "navsat/rtklib_nav";
static std::string subscribe_gga_topic_name = "navsat/gga";

void rtklib_nav_callback(const rtklib_msgs::RtklibNav::ConstPtr& msg)
{
  rtklib_nav_msg = *msg;
}

void gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  gga_msg = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_msg = *msg;

  velocity_estimator.VelocityEstimate(imu_msg, rtklib_nav_msg, gga_msg, &velocity_msg);

  eagleye_msgs::StatusStamped velocity_status;
  velocity_status.header = msg->header;
  velocity_status.status = velocity_estimator.getStatus();
  velocity_status_pub.publish(velocity_status);

  if(velocity_status.status.enabled_status)
  {
    velocity_pub.publish(velocity_msg);
  }

}

void velocity_estimator_node(ros::NodeHandle nh)
{
  nh.getParam("yaml_file",yaml_file);
  velocity_estimator.setParam(yaml_file);

  ros::Subscriber rtklib_sub = nh.subscribe(subscribe_rtklib_nav_topic_name, 1000, rtklib_nav_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber gga_sub = nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber imu_sub = nh.subscribe(subscribe_imu_topic_name, 1000, imu_callback, ros::TransportHints().tcpNoDelay());

  velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("velocity", 1000);
  velocity_status_pub = nh.advertise<eagleye_msgs::StatusStamped>("velocity_status", 1000);

  ros::spin();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_estimator");
  ros::NodeHandle nh;

  velocity_estimator_node(nh);

  return 0;
}

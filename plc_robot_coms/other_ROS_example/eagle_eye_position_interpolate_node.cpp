/*

This file is a modified part of the EAGLE EYE project
ref:- https://github.com/MapIV/eagleye/tree/main-ros1/eagleye_rt
file :- position_interpolate_node.cpp

objective :-
we want to use ublox GPS receivers which connect to the follwing ros node rather than navsat/gga topic
https://github.com/KumarRobotics/ublox/tree/galactic-devel?tab=readme-ov-file
https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html

ros2 launch ublox_gps ublox_gps_node-launch.py
publishes to
/fix

Eagle Eye Gives us :-
Subscribed Topics
/nmea_sentence (nmea_msgs/Sentence)
/can_twist (geometry_msgs/TwistStamped)
/rtklib_nav (rtklib_msgs/RtklibNav)
/imu/data_raw (sensor_msgs/Imu)

full list as follows :-
/eagleye/acc_x_offset
/eagleye/acc_x_scale_factor
/eagleye/angular_velocity_offset_stop
/eagleye/distance
/eagleye/eagleye/fix
/eagleye/eagleye/geo_pose_with_covariance
/eagleye/eagleye/heading_interpolate_3rd
/eagleye/eagleye/pitching
/eagleye/eagleye/pose
/eagleye/eagleye/rolling
/eagleye/enu_absolute_pos
/eagleye/enu_absolute_pos_interpolate
/eagleye/enu_relative_pos
/eagleye/enu_vel
/eagleye/fix
/eagleye/geo_pose_with_covariance
/eagleye/gnss/fix
/eagleye/gnss/gga
/eagleye/gnss/rmc
/eagleye/gnss/rtklib_nav
/eagleye/gnss_compass_pose
/eagleye/gnss_smooth_pos_enu
/eagleye/heading_1st
/eagleye/heading_2nd
/eagleye/heading_3rd
/eagleye/heading_interpolate_1st
/eagleye/heading_interpolate_2nd
/eagleye/heading_interpolate_3rd
/eagleye/height
/eagleye/imu/data_corrected
/eagleye/imu/data_tf_converted
/eagleye/navsat/reliability_gga
/eagleye/pitching
/eagleye/pose
/eagleye/rolling
/eagleye/rtklib/fix
/eagleye/slip_angle
/eagleye/twist
/eagleye/twist_with_covariance
/eagleye/vehicle/twist
/eagleye/velocity
/eagleye/velocity_scale_factor
/eagleye/velocity_status
/eagleye/yaw_rate_offset_1st
/eagleye/yaw_rate_offset_2nd
/eagleye/yaw_rate_offset_stop

Main Published Topics
/eagleye/fix (sensor_msgs/NavSatFix)
/eagleye/twist (ngeometry_msgs/TwistStamped)

*/

#include "ros/ros.h"
#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"
// use ublox interface comment for gga instead
#define _UBLOX

static eagleye_msgs::Position _enu_absolute_pos;
static geometry_msgs::Vector3Stamped _enu_vel;
static eagleye_msgs::Height _height;
static eagleye_msgs::Position _gnss_smooth_pos;
static eagleye_msgs::Heading _heading_interpolate_3rd;

static eagleye_msgs::Position _enu_absolute_pos_interpolate;
static sensor_msgs::NavSatFix _eagleye_fix;
#if defined(_UBLOX)
static sensor_msgs::NavSatFix _ublox_fix;
#else
static nmea_msgs::Gpgga _gga;
#endif
static ros::Publisher _pub1;
static ros::Publisher _pub2;

struct PositionInterpolateParameter _position_interpolate_parameter;
struct PositionInterpolateStatus _position_interpolate_status;

#if defined(_UBLOX)
void gnss_ublox_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  _ublox_fix = *msg;
}
#else
void gga_callback(const nmea_msgs::Gpgga::ConstPtr& msg)
{
  _gga = *msg;
}
#endif

void enu_absolute_pos_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  _enu_absolute_pos = *msg;
}

void gnss_smooth_pos_enu_callback(const eagleye_msgs::Position::ConstPtr& msg)
{
  _gnss_smooth_pos = *msg;
}

void height_callback(const eagleye_msgs::Height::ConstPtr& msg)
{
  _height = *msg;
}

void heading_interpolate_3rd_callback(const eagleye_msgs::Heading::ConstPtr& msg)
{
  _heading_interpolate_3rd = *msg;
}

void enu_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  _enu_vel = *msg;
  _enu_absolute_pos_interpolate.header = msg->header;
  _enu_absolute_pos_interpolate.header.frame_id = "base_link";
  _eagleye_fix.header = msg->header;
  _eagleye_fix.header.frame_id = "gnss";
  position_interpolate_estimate(_enu_absolute_pos, _enu_vel, _gnss_smooth_pos, _height, _heading_interpolate_3rd, _position_interpolate_parameter, &_position_interpolate_status, &_enu_absolute_pos_interpolate, &_eagleye_fix);
  if(_enu_absolute_pos.status.enabled_status && (_eagleye_fix.latitude != 0.0 || _eagleye_fix.longitude != 0.0))
  {
    _pub1.publish(_enu_absolute_pos_interpolate);
    _pub2.publish(_eagleye_fix);
  }

#if defined(_UBLOX)
  else if (_ublox_fix.header.stamp.toSec() != 0)
  {
    sensor_msgs::NavSatFix fix;
    fix.header = _ublox_fix.header;
    fix.latitude = _ublox_fix.latitude;
    fix.longitude = _ublox_fix.longitude;
    fix.altitude = _ublox_fix.altitude;
    // TODO(Map IV): temporary covariance value
    fix.position_covariance[0] = 1.5 * 1.5; // [m^2]
    fix.position_covariance[4] = 1.5 * 1.5; // [m^2]
    fix.position_covariance[8] = 1.5 * 1.5; // [m^2]
    _pub2.publish(fix);
  }
#else
  else if (_gga.header.stamp.toSec() != 0)
  {
    sensor_msgs::NavSatFix fix;
    fix.header = _gga.header;
    fix.latitude = _gga.lat;
    fix.longitude = _gga.lon;
    fix.altitude = _gga.alt + _gga.undulation;
    // TODO(Map IV): temporary covariance value
    fix.position_covariance[0] = 1.5 * 1.5; // [m^2]
    fix.position_covariance[4] = 1.5 * 1.5; // [m^2]
    fix.position_covariance[8] = 1.5 * 1.5; // [m^2]
    _pub2.publish(fix);
  }
#endif
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_interpolate");
  ros::NodeHandle nh;

#if defined(_UBLOX)
  std::string subscribe_ublox_topic_name = "/fix";
#else
  std::string subscribe_gga_topic_name = "navsat/gga";
#endif
  
  std::string yaml_file;
  nh.getParam("yaml_file",yaml_file);
  std::cout << "yaml_file: " << yaml_file << std::endl;

  try
  {
    YAML::Node conf = YAML::LoadFile(yaml_file);

    _position_interpolate_parameter.imu_rate = conf["common"]["imu_rate"].as<double>();
    _position_interpolate_parameter.stop_judgement_threshold = conf["common"]["stop_judgement_threshold"].as<double>();
    _position_interpolate_parameter.sync_search_period = conf["position_interpolate"]["sync_search_period"].as<double>();
    _position_interpolate_parameter.proc_noise = conf["position_interpolate"]["proc_noise"].as<double>();

    std::cout << "imu_rate " << _position_interpolate_parameter.imu_rate << std::endl;
    std::cout << "stop_judgement_threshold " << _position_interpolate_parameter.stop_judgement_threshold << std::endl;
    std::cout << "sync_search_period " << _position_interpolate_parameter.sync_search_period << std::endl;
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[1;31mheading_interpolate Node YAML Error: " << e.msg << "\033[0m" << std::endl;
    exit(3);
  }

  ros::Subscriber sub1 = nh.subscribe("enu_vel", 1000, enu_vel_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub2 = nh.subscribe("enu_absolute_pos", 1000, enu_absolute_pos_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub3 = nh.subscribe("gnss_smooth_pos_enu", 1000, gnss_smooth_pos_enu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub4 = nh.subscribe("height", 1000, height_callback, ros::TransportHints().tcpNoDelay());
#if defined(_UBLOX)
  ros::Subscriber sub5 = nh.subscribe(subscribe_ublox_topic_name, 1000, gnss_ublox_callback, ros::TransportHints().tcpNoDelay());
#else
  ros::Subscriber sub5 = nh.subscribe(subscribe_gga_topic_name, 1000, gga_callback, ros::TransportHints().tcpNoDelay());
#endif
  ros::Subscriber sub6 = nh.subscribe("heading_interpolate_3rd", 1000, heading_interpolate_3rd_callback, ros::TransportHints().tcpNoDelay());
  _pub1 = nh.advertise<eagleye_msgs::Position>("enu_absolute_pos_interpolate", 1000);
  _pub2 = nh.advertise<sensor_msgs::NavSatFix>("fix", 1000);

  ros::spin();

  return 0;
}


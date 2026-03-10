//
// obstacle avoidance ROS2 ref:- https://github.com/DinSopheakPanha1111/Lab2_ws/blob/main/src/autopilot/src/AutoPilot.cpp
//
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
// https://github.com/p-ranav/argparse?ysclid=mmkjn3zq25613883937
#include <argparse/argparse.hpp>

class AutoPilot : public rclcpp::Node
{
public:
    AutoPilot() : Node("autopilot"), last_turn_direction_(0), total_distance_(0.0), exploration_done_(false)
    {
        argparse::ArgumentParser program("obstacle_avoid_ros2");

        program.add_argument("--safe_dist_hys")
        .help("length of safe distance and hysteris")
        .nargs(2)
        .default_value(std::vector<double>{0.3, 0.5})
       .scan<'g', double>();

        program.add_argument("--tot_dist")
        .help("total distance limit")
        .default_value(10.0)
       .scan<'g', double>();

        program.add_argument("--angle")
        .help("angle ranges defaulted to (front ±20°)")
        .default_value(20.0)
       .scan<'g', double>();

        program.add_argument("--move_stdy")
        .help("move steady linX angZ : go forward slow state")
        .nargs(2)
        .default_value(std::vector<double>{0.05, 0.0})
       .scan<'g', double>();

        program.add_argument("--move_avoid")
        .help("move avoidance linX angZ : turning mvoement")
        .nargs(2)
        .default_value(std::vector<double>{0.0, 2.0})
       .scan<'g', double>();
	   
        try {
          program.parse_args(argc, argv);
        }
        catch (const std::exception& err) {
          std::cerr << err.what() << std::endl;
          std::cerr << program;
          return 1;
        }

        auto input = program.get<int>("square");
		auto sdvec = program.get<std::vector<double>>("--safe_dist_hys");
        safe_distance = sdvec.at(0);
        hysteresis = sdvec.at(1);
        total_d_limit = program.get<double>("--tot_dist");
        msvec = program.get<std::vector<double>>("--move_stdy");
        avoidvec = program.get<std::vector<double>>("--move_avoid");
        anglelim = program.get<double>("--angle");
		
        // Subscribers lidar and odometry
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&AutoPilot::scan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&AutoPilot::odom_callback, this, std::placeholders::_1));

        // Publisher robot pose twist to control movement
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "AutoPilot node started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        if (!path_points_.empty())
        {
            auto [prev_x, prev_y] = path_points_.back();
            total_distance_ += std::hypot(x - prev_x, y - prev_y);
        }
        path_points_.push_back({x, y});

        // Stop condition — for example, total distance > 10 meters
        if (total_distance_ > total_d_limit && !exploration_done_)
        {
            RCLCPP_WARN(this->get_logger(), "Exploration finished (area coverage threshold reached).");
            stop_robot();
            exploration_done_ = true;
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (exploration_done_)
            return; // Stop processing once done

        geometry_msgs::msg::Twist cmd_msg;

        // Define angle ranges (front ±20°)
        const float right_angle_min = -anglelim * M_PI / 180.0;
        const float right_angle_max = 0.0;
        const float left_angle_min  = 0.0;
        const float left_angle_max  = anglelim * M_PI / 180.0;

        auto get_index = [&](float angle) -> int {
            return static_cast<int>((angle - msg->angle_min) / msg->angle_increment);
        };

        int right_start = std::max(0, get_index(right_angle_min));
        int right_end   = std::min(static_cast<int>(msg->ranges.size()) - 1, get_index(right_angle_max));
        int left_start  = std::max(0, get_index(left_angle_min));
        int left_end    = std::min(static_cast<int>(msg->ranges.size()) - 1, get_index(left_angle_max));

        float min_right = std::numeric_limits<float>::infinity();
        float min_left  = std::numeric_limits<float>::infinity();

        for (int i = right_start; i <= right_end; ++i)
            if (std::isfinite(msg->ranges[i])) min_right = std::min(min_right, msg->ranges[i]);
        for (int i = left_start; i <= left_end; ++i)
            if (std::isfinite(msg->ranges[i])) min_left = std::min(min_left, msg->ranges[i]);

        // Default: go forward slowly
        cmd_msg.linear.x = msvec.at(0);
        cmd_msg.angular.z = msvec.at(1);

        if (min_left < safe_distance && min_right < safe_distance)
        {
            cmd_msg.linear.x = 0.0;
            if (last_turn_direction_ == 0)
                last_turn_direction_ = 1;                                                    // default turn left
            cmd_msg.angular.z = (last_turn_direction_ == 1) ? 1.0 : -1.0;
        }
        else if (min_left < safe_distance && min_right > safe_distance + hysteresis)
        {
            cmd_msg.linear.x = avoidvec.at(0);
            cmd_msg.angular.z = -avoidvec.at(1);
            last_turn_direction_ = -1;
        }
        else if (min_right < safe_distance && min_left > safe_distance + hysteresis)
        {
            cmd_msg.linear.x = avoidvec.at(0);
            cmd_msg.angular.z = avoidvec.at(1);
            last_turn_direction_ = 1;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Left: %.2f | Right: %.2f | v=%.2f | w=%.2f | dist=%.2f m",
                    min_left, min_right, cmd_msg.linear.x, cmd_msg.angular.z, total_distance_);

        cmd_pub_->publish(cmd_msg);
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;
        cmd_pub_->publish(stop);
        RCLCPP_INFO(this->get_logger(), "Robot stopped.");
    }

    // Members
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    int last_turn_direction_;                                       // -1 = right, 1 = left
    double total_distance_;                                         // in meters
	double total_d_limit = 10.0;
    bool exploration_done_;                                         // stop condition
    std::vector<std::pair<double, double>> path_points_;
    double safe_distance = 0.3;                                     // distance inside then move away
    double hysteresis = 0.05;
	std::vector<double> msvec = {0.05, 0.0};                        // steady state movement vector
	std::vector<double> avoidvec = {0.0, 2.0};                      // avoid condition movement vector 
	double anglelim = 20.0;               
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoPilot>());
    rclcpp::shutdown();
    return 0;
}
//
// Camera Realsense to ROS2 publisher
//
//
#include "CamCv.h"

class CamRsNode : public rclcpp::Node
{
public:
    CamRsNode() : Node("cam_rs_node")
    {
        cv::string keys = "{depth_near_value||}""{depth_far_value||}""{fps||}""{opt 18+ is no option}";
        cv::CommandLineParser parser(argc, argv, keys);
        float depth_near_value = static_cast<float>(parser.get<cv::float>("depth_near_value"));
        float depth_far_value = static_cast<float>(parser.get<cv::float>("depth_far_value"));
        int capture_fps = static_cast<int>(parser.get<cv::int>("fps"));
		opt = static_cast<int>(parser.get<cv::int>("opt"));
        if (!camera.open(capture_fps))
        {
            throw std::runtime_error("Can't open capture device.");
        }
        camera.start(depth_near_value, depth_far_value);

        /* fps publish rate */
        timer_ = create_wall_timer(std::chrono::milliseconds((1/capture_fps)*1000),std::bind(&CamRsNode::update, this));
        RCLCPP_INFO(get_logger(), "Camera Realsense node running");
    }

    ~CamRsNode()
    {
        camera.stop();
    }

private:

    /* copy the grabbed camera frame to the ROS frame and publish this at the rate set by the fps */
    void update()
    {
        camera.transmit(this->now(), opt);                                   // publish grabbed frame with node time stamp
    }

    CamRs camera;                                                            // camera class
    rclcpp::TimerBase::SharedPtr timer_;                                     // wall timer set equal to the fps
	int opt = 0;
	int fr_time = 0;                                                         // write frame time or not 0 none 1,2 write to picture
};

/* run the ROS2 node for the camera */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamRsNode>());
    rclcpp::shutdown();
    return 0;
}
//
// Camera CV to ROS2 publisher
//
//
#include "CamCv.h"

class CamCvNode : public rclcpp::Node
{
public:
    CamCvNode() : Node("cam_cv_node")
    {
        cv::string keys = "{src||}""{height||}""{width||}""{fps||}""{opt||}""{frame_time||}";
        cv::CommandLineParser parser(argc, argv, keys);
        std::string src_path = static_cast<std::string>(parser.get<cv::string>("src"));
        int capture_width = static_cast<int>(parser.get<cv::int>("width"));
        int capture_height = static_cast<int>(parser.get<cv::int>("height"));
        int capture_fps = static_cast<int>(parser.get<cv::int>("fps"));
		opt = static_cast<int>(parser.get<cv::int>("opt"));
		fr_time = static_cast<int>(parser.get<cv::int>("frame_time"));
        try { 
                int port_no = stoi(src_path);  
                if (!camera.open(port_no, capture_width, capture_height, capture_fps))
                {
                    throw std::runtime_error("Can't open capture device.");
                }
            }
            catch (std::exception &e) {
                if (!camera.open(src_path, capture_width, capture_height, capture_fps))
                {
                    throw std::runtime_error("Can't open capture video file.");
                }
            }
        camera.start(fr_time);

        /* fps publish rate */
        timer_ = create_wall_timer(std::chrono::milliseconds((1/capture_fps)*1000),std::bind(&CamCvNode::update, this));
        RCLCPP_INFO(get_logger(), "Camera CV node running");
    }

    ~CamCvNode()
    {
        camera.stop();
    }

private:

    /* copy the grabbed camera frame to the ROS frame and publish this at the rate set by the fps */
    void update()
    {
        camera.transmit(this->now(), opt);                             // publish grabbed frame with node time stamp
    }

    CamCv camera;                                                            // camera class
    rclcpp::TimerBase::SharedPtr timer_;                                     // wall timer set equal to the fps
	int opt = 0;
	int fr_time = 0;                                                         // write frame time or not 0 none 1,2 write to picture
};

/* run the ROS2 node for the camera */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamCvNode>());
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


namespace usb_cam_node
{

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber(rclcpp::NodeOptions options);
  ~ImageSubscriber();
private:
  void imageCallback(sensor_msgs::msg::Image::SharedPtr img);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};


ImageSubscriber::ImageSubscriber(rclcpp::NodeOptions options) : Node("image_subscriber", options)
{
  using std::placeholders::_1;
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&ImageSubscriber::imageCallback, this, _1));
}

ImageSubscriber::~ImageSubscriber()
{
}

void ImageSubscriber::imageCallback(sensor_msgs::msg::Image::SharedPtr img)
{
  auto cv_img = cv_bridge::toCvShare(img, img->encoding);
  if(cv_img->image.empty()) return;
  cv::imshow("Image", cv_img->image);
  cv::waitKey(1);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam_node::ImageSubscriber)
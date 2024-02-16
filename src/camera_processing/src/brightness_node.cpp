#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "opencv2/imgproc.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"


class BrightnessNode : public rclcpp::Node
{
public:
  BrightnessNode()
  : Node("brightness_node")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&BrightnessNode::avg_brightness_callback, this, std::placeholders::_1)
    );
    brightness_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("brightness_status", 10);

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(255).set__step(1);
    param_desc.set__integer_range({range});
    param_desc.description = "Brightness threshold between 0 and 255. Default is 100.";
    this->declare_parameter("threshold", 100, param_desc);
    threshold_ = this->get_parameter("threshold").as_int();
  }

private:
  void avg_brightness_callback(const sensor_msgs::msg::Image & msg) 
  {
    int brightness = calculateBrightness(msg);
    threshold_ = this->get_parameter("threshold").as_int();

    std_msgs::msg::Bool status;
    status.data = brightness > threshold_ ? true : false;
    brightness_status_pub_->publish(status);


    RCLCPP_INFO(this->get_logger(), "Image received with brightness %d", brightness);
  }

  int calculateBrightness(const sensor_msgs::msg::Image & msg) const
  {
    cv::Mat image(msg.height, msg.width, CV_8UC3, const_cast<unsigned char*>(msg.data.data()));
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Scalar avgPixelIntensity = cv::mean(gray);
    return avgPixelIntensity[0];
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brightness_status_pub_;
  int threshold_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrightnessNode>());
  rclcpp::shutdown();
  return 0;
}
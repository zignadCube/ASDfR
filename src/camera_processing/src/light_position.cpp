// 1.1.4 Simple light position indicator
// A very simple way of detecting the position of a bright light in an (otherwise no so bright) image
// is the following:
// 1. Gray-scale the image.
// 2. Apply a threshold on the brightness of each pixel.
// If the threshold is well-chosen (and there are no other bright parts in the image), the
// result is a black image with a large white dot where you shine the light. You may need to
// reduce background light.
// 3. Compute the ‘center of gravity’ (COG) of the white pixels. This COG (in pixel coordinates)
// gives an indication of the position of the center of the light.
// Create a node that, given a camera input, outputs the position of a bright light (if there is any)
// in pixel coordinates. The node should be easy to use, e.g., also when using in a larger project,
// on a different webcam or with different lighting conditions.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "opencv2/imgproc.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "opencv2/highgui.hpp"

class LightPosition : public rclcpp::Node
{
public:
  LightPosition()
  : Node("light_position")
  {
    auto image_topic_desc = rcl_interfaces::msg::ParameterDescriptor{};
    image_topic_desc.description = "The topic on which the camera publishes images. Cannot be changed at runtime. Default is 'image'.";
    image_topic_desc.read_only = true;
    this->declare_parameter("image_topic", "image", image_topic_desc);
    std::string image_topic = this->get_parameter("image_topic").as_string();

    auto thresh_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    rcl_interfaces::msg::IntegerRange range;
    range.set__from_value(0).set__to_value(255).set__step(1);
    thresh_param_desc.set__integer_range({range});
    thresh_param_desc.description = "Brightness threshold between 0 and 255. Default is 100.";
    this->declare_parameter("threshold", 100, thresh_param_desc);

    auto debug_light_position_desc = rcl_interfaces::msg::ParameterDescriptor{};
    debug_light_position_desc.description = "If true, the light position is printed to the console and a debug image is shown. Default is false.";
    this->declare_parameter("debug_light_position", false, debug_light_position_desc);


    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10, std::bind(&LightPosition::light_position_callback, this, std::placeholders::_1)
    );
    light_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("light_position", 10);
  }

private:
    void light_position_callback(const sensor_msgs::msg::Image & msg) 
    {
        threshold_ = this->get_parameter("threshold").as_int();
    
        cv::Mat image(msg.height, msg.width, CV_8UC3, const_cast<unsigned char*>(msg.data.data()));
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, gray, threshold_, 255, cv::THRESH_BINARY);
        cv::Moments m = cv::moments(gray, true);
        cv::Point2f center(m.m10/m.m00, m.m01/m.m00);
    
        geometry_msgs::msg::Point position;
        position.x = center.x;
        position.y = center.y;
        position.z = 0;
        light_position_pub_->publish(position);
    
        bool debug_light_position = this->get_parameter("debug_light_position").as_bool();
        if (debug_light_position)
        {
            this->debug_light_position(gray, center); // Fixed the function call
        }
    }

    void debug_light_position(const cv::Mat & image, const cv::Point2f & center) const
    {
        // image is grayscale, so we need to convert it to BGR to draw the circle
        cv::Mat color_image;
        cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
        cv::circle(color_image, center, 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Light position", color_image);
        cv::waitKey(1);
        RCLCPP_INFO(this->get_logger(), "Light position: x=%f, y=%f", center.x, center.y);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr light_position_pub_;
    int threshold_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPosition>());
  rclcpp::shutdown();
  return 0;
}
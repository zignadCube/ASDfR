#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "opencv2/imgproc.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "opencv2/highgui.hpp"

class ObjectPosition : public rclcpp::Node
{
public:
  ObjectPosition()
  : Node("object_position")
  {
    auto image_topic_desc = rcl_interfaces::msg::ParameterDescriptor{};
    image_topic_desc.description = "The topic on which the camera publishes images. Cannot be changed at runtime. Default is 'image'.";
    image_topic_desc.read_only = true;
    this->declare_parameter("image_topic", "image", image_topic_desc);
    std::string image_topic = this->get_parameter("image_topic").as_string();

    auto debug_object_position_desc = rcl_interfaces::msg::ParameterDescriptor{};
    debug_object_position_desc.description = "If true, the light position is printed to the console and a debug image is shown. Default is false.";
    this->declare_parameter("debug_object_position", false, debug_object_position_desc);


    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, 10, std::bind(&ObjectPosition::object_position_callback, this, std::placeholders::_1)
    );
    object_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("object_position", 10);
  }

private:
    void object_position_callback(const sensor_msgs::msg::Image & msg) 
    {
        cv::Mat image(msg.height, msg.width, CV_8UC3, const_cast<unsigned char*>(msg.data.data()));
        RCLCPP_INFO(this->get_logger(), "Image received: %dx%d", msg.width, msg.height);
        
        // Convert image to HSV color space
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        // Define the lower and upper bounds for the color you want to detect (in HSV)
        cv::Scalar colorLower(60, 50, 50); // Example: lower bound for blue color
        cv::Scalar colorUpper(90, 255, 255); // Example: upper bound for blue color

        // Mask the image to get only the specified color
        cv::Mat mask;
        cv::inRange(hsv, colorLower, colorUpper, mask);

        // Find contours in the mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        double maxArea = 0;
        int maxAreaIdx = -1;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea) {
                maxArea = area;
                maxAreaIdx = i;
            }
        }

        // If a contour is found, calculate its centroid
        cv::Point2f centroid(-1, -1);
        if (maxAreaIdx != -1) {
            cv::Moments M = cv::moments(contours[maxAreaIdx]);
            centroid = cv::Point2f(M.m10 / M.m00, M.m01 / M.m00);
        }

        geometry_msgs::msg::Point position;
        position.x = centroid.x - msg.width/2;
        position.y = centroid.y - msg.height/2;
        position.z = 0;
        object_position_pub_->publish(position);
    
        bool debug_object_position = this->get_parameter("debug_object_position").as_bool();
        if (debug_object_position && centroid.x != -1 && centroid.y != -1)
        {
            this->debug_object_position(mask, position);
        }
    }

    void debug_object_position(const cv::Mat & image, const cv::Point2f & center) const
    {
        cv::Mat color_image;
        cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
        cv::circle(color_image, center, 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("Object Position", color_image);
        cv::waitKey(1);
        RCLCPP_INFO(this->get_logger(), "Object Position: x=%f, y=%f", center.x, center.y);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr object_position_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectPosition>());
  rclcpp::shutdown();
  return 0;
}

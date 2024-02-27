#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui.hpp"

class ImageSub : public rclcpp::Node
{
public:
  ImageSub()
  : Node("image_sub")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10, std::bind(&ImageSub::image_callback, this, std::placeholders::_1)
    );
  }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()));
        cv::imshow("Image window", image);
        cv::waitKey(1);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSub>());
  rclcpp::shutdown();
  return 0;
}
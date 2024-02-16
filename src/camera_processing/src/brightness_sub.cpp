#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class BrightnessSub : public rclcpp::Node
{
public:
  BrightnessSub()
  : Node("brightness_status_sub")
  {
    brightness_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "brightness_status", 10, std::bind(&BrightnessSub::brightness_status_callback, this, std::placeholders::_1)
    );
  }

private:
    void brightness_status_callback(const std_msgs::msg::Bool & msg) 
    {
      RCLCPP_INFO(this->get_logger(), "It is: %s", msg.data ? "bright" : "dark");
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brightness_status_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrightnessSub>());
  rclcpp::shutdown();
  return 0;
}

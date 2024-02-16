#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

class LightPositionSub : public rclcpp::Node
{
public:
  LightPositionSub()
  : Node("light_position_sub")
  {
    light_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "light_position", 10, std::bind(&LightPositionSub::light_position_callback, this, std::placeholders::_1)
    );
  }

private:
    void light_position_callback(const geometry_msgs::msg::Point & msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Light position: x=%f, y=%f", msg.x, msg.y);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr light_position_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPositionSub>());
  rclcpp::shutdown();
  return 0;
}
#include "rclcpp/rclcpp.hpp"

// Define message includes here...
#include "custom_msgs/msg/ros2_xeno.hpp"
#include "custom_msgs/msg/xeno2_ros.hpp"

class SequenceController : public rclcpp::Node
{
public:
    SequenceController()
    : Node("sequence_controller")
    {

        // Create publishers
        setpoint_publisher_ = this->create_publisher<custom_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);

        // Create subscriptions
        encoder_sub_ = this->create_subscription<custom_msgs::msg::Xeno2Ros>("Xeno2Ros", 10, std::bind(&SequenceController::encoder_callback, this, std::placeholders::_1));

        // Create timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SequenceController::timer_callback, this));
    }

    void encoder_callback(const custom_msgs::msg::Xeno2Ros & msg)
    {
        encoder_pos = msg;
        RCLCPP_INFO(this->get_logger(), "Encoder position: x=%f, y=%f", encoder_pos.x, encoder_pos.y);
        
        
    }

    void timer_callback()
    {
        auto message = custom_msgs::msg::Ros2Xeno();

        message.x = 1.0;
        message.y = 1.0;
        RCLCPP_INFO(this->get_logger(), "Left/Right Publishing: '%f'/'%f'", message.x, message.y);
        setpoint_publisher_->publish(message);
    }


    custom_msgs::msg::Xeno2Ros encoder_pos;

    rclcpp::Publisher<custom_msgs::msg::Ros2Xeno>::SharedPtr setpoint_publisher_;
    rclcpp::Subscription<custom_msgs::msg::Xeno2Ros>::SharedPtr encoder_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceController>());
  rclcpp::shutdown();
  return 0;
}
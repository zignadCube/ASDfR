#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

// Define message includes here...
#include "custom_msgs/msg/ros2_xeno.hpp"
#include "custom_msgs/msg/xeno2_ros.hpp"

class SequenceController : public rclcpp::Node
{
public:
    SequenceController()
    : Node("sequence_controller")
    {
        auto channel1_send_value_desc = rcl_interfaces::msg::ParameterDescriptor();
        channel1_send_value_desc.description = "Value to send to Channel 1 (left wheel)";
        // Float range between -1 and 1
        rcl_interfaces::msg::FloatingPointRange channel1_send_value_range;
        channel1_send_value_range.from_value = -1;
        channel1_send_value_range.to_value = 1;
        channel1_send_value_desc.floating_point_range.push_back(channel1_send_value_range);
        this->declare_parameter("channel1_send_value", 0.0, channel1_send_value_desc);

        auto channel2_send_value_desc = rcl_interfaces::msg::ParameterDescriptor();
        channel2_send_value_desc.description = "Value to send to Channel 2 (right wheel)";
        // Float range between -1 and 1
        rcl_interfaces::msg::FloatingPointRange channel2_send_value_range;
        channel2_send_value_range.from_value = -1;
        channel2_send_value_range.to_value = 1;
        channel2_send_value_desc.floating_point_range.push_back(channel2_send_value_range);
        this->declare_parameter("channel2_send_value", 0.0, channel2_send_value_desc);

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
        RCLCPP_INFO(this->get_logger(), "Encoder position: x=%d, y=%d", encoder_pos.x, encoder_pos.y);
        
        
    }

    void timer_callback()
    {
        auto message = custom_msgs::msg::Ros2Xeno();

        message.x = this->get_parameter("channel1_send_value").as_double();
        message.y = this->get_parameter("channel2_send_value").as_double();
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
    return 0;
}
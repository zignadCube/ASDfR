#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

// Define message includes here...
#include "custom_msgs/msg/ros2_xeno.hpp"
#include "custom_msgs/msg/xeno2_ros.hpp"
#include "geometry_msgs/msg/point.hpp"

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

        //object_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>("object_position", 10, std::bind(&SequenceController::object_position_callback, this, std::placeholders::_1));

        // Create timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SequenceController::timer_callback, this));
    }

private:
    void encoder_callback(const custom_msgs::msg::Xeno2Ros & msg)
    {
        encoder_pos = msg;
        RCLCPP_INFO(this->get_logger(), "Encoder position: x=%d, y=%d", encoder_pos.x, encoder_pos.y);
    }

    void object_position_callback(const geometry_msgs::msg::Point & msg)
    {
        object_pos = msg;
        auto message = custom_msgs::msg::Ros2Xeno();

        double diff = 0.01 * object_pos.x;

        message.x = diff;
        message.y = -diff;

        RCLCPP_INFO(this->get_logger(), "Left/Right Publishing: '%f'/'%f'", message.x, message.y);
        setpoint_publisher_->publish(message);
    }

    ///* Don't need timer, sending data only when receiving object position
    void timer_callback()
    {
        auto message = custom_msgs::msg::Ros2Xeno();

        if (timer_count >= 7){
            message.x = 0.0;
            message.y = 0.0;
        }else if (timer_count >= 5){
            message.x = 0.0;
            message.y = 1.0;
        }else if(timer_count >= 0){
            message.x = 1.0;
            message.y = 1.0;
        }

        timer_count++;
        RCLCPP_INFO(this->get_logger(), "Time: %d Left/Right Publishing: '%f'/'%f'", timer_count, message.x, message.y);
        setpoint_publisher_->publish(message);
    }
    //*/

    int timer_count = 0;

    custom_msgs::msg::Xeno2Ros encoder_pos;
    geometry_msgs::msg::Point object_pos;

    rclcpp::Publisher<custom_msgs::msg::Ros2Xeno>::SharedPtr setpoint_publisher_;

    rclcpp::Subscription<custom_msgs::msg::Xeno2Ros>::SharedPtr encoder_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr object_position_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}
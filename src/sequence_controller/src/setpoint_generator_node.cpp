#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


class SetpointsGenerator : public rclcpp::Node
{
public:
  SetpointsGenerator()
  : Node("setpoints_generator"), count_(0)
  {
    auto left_setpoint_desc = rcl_interfaces::msg::ParameterDescriptor{};
    left_setpoint_desc.description = "The setpoint topic for the left motor. Cannot be changed at runtime. Default is /input/left_motor/setpoint_vel.";
    left_setpoint_desc.read_only = true;
    this->declare_parameter("left_setpoint_topic", "/input/left_motor/setpoint_vel", left_setpoint_desc);
    std::string left_setpoint_topic = this->get_parameter("left_setpoint_topic").as_string();

    auto right_setpoint_desc = rcl_interfaces::msg::ParameterDescriptor{};
    right_setpoint_desc.description = "The setpoint topic for the right motor. Cannot be changed at runtime. Default is /input/right_motor/setpoint_vel.";
    right_setpoint_desc.read_only = true;
    this->declare_parameter("right_setpoint_topic", "/input/right_motor/setpoint_vel", right_setpoint_desc);
    std::string right_setpoint_topic = this->get_parameter("right_setpoint_topic").as_string();

    left_setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float64>(left_setpoint_topic, 10);
    left_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SetpointsGenerator::left_timer_callback, this));

    right_setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float64>(right_setpoint_topic, 10);
    right_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SetpointsGenerator::right_timer_callback, this));
  }

private:
    void left_timer_callback()
    {
        auto message = std_msgs::msg::Float64();
        message.data = count_++;
        RCLCPP_INFO(this->get_logger(), "Left Publishing: '%f'", message.data);
        left_setpoint_publisher_->publish(message);
    }

    void right_timer_callback()
    {
        auto message = std_msgs::msg::Float64();
        message.data = count_++;
        RCLCPP_INFO(this->get_logger(), "Right Publishing: '%f'", message.data);
        right_setpoint_publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr left_timer_;
    rclcpp::TimerBase::SharedPtr right_timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_setpoint_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointsGenerator>());
  rclcpp::shutdown();
  return 0;
}
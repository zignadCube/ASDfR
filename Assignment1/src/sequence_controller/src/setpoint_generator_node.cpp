#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"


class SetpointsGenerator : public rclcpp::Node
{
public:
  SetpointsGenerator()
  : Node("setpoints_generator"), count_(0), velL_(4.0), velR_(4.0)
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
    right_setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float64>(right_setpoint_topic, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&SetpointsGenerator::timer_callback, this));
  }

private:
    void timer_callback()
    {
        auto left_message = std_msgs::msg::Float64();
        auto right_message = std_msgs::msg::Float64();

        if (count_ > 10){
          count_ = 0;
          velL_ = -velL_;
          velR_ = -velR_;
        }
        count_++;
        left_message.data = velL_;
        right_message.data = velR_;
        RCLCPP_INFO(this->get_logger(), "Left/Right Publishing: '%f'/'%f'", left_message.data, right_message.data);
        left_setpoint_publisher_->publish(left_message);
        right_setpoint_publisher_->publish(right_message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_setpoint_publisher_;
    int count_;
    double velL_;
    double velR_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointsGenerator>());
  rclcpp::shutdown();
  return 0;
}
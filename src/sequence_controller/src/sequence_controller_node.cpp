#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class SequenceController : public rclcpp::Node
{
public:
    SequenceController()
    : Node("sequence_controller")
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

        auto speed_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.0).set__to_value(1.0).set__step(0.01);
        speed_param_desc.set__floating_point_range({range});
        speed_param_desc.description = "Differential drive control speed";
        this->declare_parameter("speed", 0.05, speed_param_desc);

        // Create publishers
        left_setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float64>(left_setpoint_topic, 10);
        right_setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float64>(right_setpoint_topic, 10);

        // Create subscriptions
        light_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>("light_position", 10, std::bind(&SequenceController::light_position_callback, this, std::placeholders::_1));
        camera_position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("output/camera_position", 10, std::bind(&SequenceController::camera_position_callback, this, std::placeholders::_1));
    }

    void light_position_callback(const geometry_msgs::msg::Point & msg)
    {
        light_pos = msg;
        RCLCPP_INFO(this->get_logger(), "Light position: x=%f, y=%f", light_pos.x, light_pos.y);
    }

    void camera_position_callback(const geometry_msgs::msg::PointStamped & msg)
    {
        camera_pos = msg.point;
        RCLCPP_INFO(this->get_logger(), "Camera position: x=%f, y=%f", camera_pos.x, camera_pos.y);

        speed_ = this->get_parameter("speed").as_double();
        RCLCPP_INFO(this->get_logger(), "Speed is: %f", speed_);
        auto messageL = std_msgs::msg::Float64();
        auto messageR = std_msgs::msg::Float64();

        double diff = speed_*(light_pos.x - camera_pos.x);
        if(diff > 5){
            diff = 5.0;
        }

        messageL.data = diff;
        messageR.data = -diff;

        left_setpoint_publisher_->publish(messageL);
        right_setpoint_publisher_->publish(messageR);
    }


    double speed_;

    geometry_msgs::msg::Point light_pos;
    geometry_msgs::msg::Point camera_pos;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_setpoint_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr light_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr camera_position_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceController>());
  rclcpp::shutdown();
  return 0;
}
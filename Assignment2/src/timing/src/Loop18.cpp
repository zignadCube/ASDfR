#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <time.h>


class Loop : public rclcpp::Node
{
public:
    Loop()
    : Node("loop_node")
    {
        seq_msg_sub_ = this->create_subscription<std_msgs::msg::Int32>(
          "seq_msg", 10, std::bind(&Loop::seq_msg_callback, this, std::placeholders::_1));
        loop_msg_pub_ = this->create_publisher<std_msgs::msg::Int32>("loop_msg", 10);
    }

private:
    void seq_msg_callback(const std_msgs::msg::Int32 & msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Message received from Seq %d", msg.data);
        clock_gettime(CLOCK_MONOTONIC, &msg_send); //start timer for jitter
        loop_msg_pub_->publish(msg);
        clock_gettime(CLOCK_MONOTONIC, &after_msg_send); //end timer for jitter
        long time_diff = (after_msg_send.tv_sec - msg_send.tv_sec) * 1000000000 + (after_msg_send.tv_nsec - msg_send.tv_nsec);
        RCLCPP_INFO(this->get_logger(), "Loop jitter = %ld us", time_diff/1000);
    }

    struct timespec msg_send, after_msg_send;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr seq_msg_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr loop_msg_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Loop>());
    rclcpp::shutdown();
    return 0;
}
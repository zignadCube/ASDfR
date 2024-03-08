#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <time.h>


class Seq : public rclcpp::Node
{
public:
    Seq()
    : Node("seq_node")
    {
        loop_msg_sub_ = this->create_subscription<std_msgs::msg::Int32>(
          "loop_msg", 10, std::bind(&Seq::loop_msg_callback, this, std::placeholders::_1));
        seq_msg_pub_ = this->create_publisher<std_msgs::msg::Int32>("seq_msg", 10);

        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(static_cast<int>(1.0)),
          std::bind(&Seq::timerCallback, this));
    }

private:
    void timerCallback(){
        std_msgs::msg::Int32 message;
        message.data = counter%100;
        counter++;

        clock_gettime(CLOCK_MONOTONIC, &msg_send); //start timer for RTT and jitter
        seq_msg_pub_->publish(message);
        clock_gettime(CLOCK_MONOTONIC, &after_msg_send); //end timer for jitter
        long time_diff = (after_msg_send.tv_sec - msg_send.tv_sec) * 1000000000 + (after_msg_send.tv_nsec - msg_send.tv_nsec);
        RCLCPP_INFO(this->get_logger(), "Seq jitter = %ld us", time_diff/1000);
    }

    void loop_msg_callback(const std_msgs::msg::Int32 & msg) 
    {
        clock_gettime(CLOCK_MONOTONIC, &msg_received); //end timer for RTT
        long time_diff = (msg_received.tv_sec - msg_send.tv_sec) * 1000000000 + (msg_received.tv_nsec - msg_send.tv_nsec);
        total += time_diff;
        count++;
        RCLCPP_INFO(this->get_logger(), "Message received from Loop %d, RTT = %ld us", msg.data, time_diff/1000);
    }

    int counter = 0;

    struct timespec msg_send, msg_received, after_msg_send;
    long long total = 0;
    int count = 0;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr loop_msg_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr seq_msg_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Seq>());
    rclcpp::shutdown();
    return 0;
}
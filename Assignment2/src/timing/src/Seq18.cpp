#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <time.h>

#define NUM_LOOPS 10000


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
        message.data = counter;
        counter++;

        if(counter < NUM_LOOPS){
            clock_gettime(CLOCK_MONOTONIC, &start_send[counter]); //start timer for RTT
            seq_msg_pub_->publish(message);
        }else if(end == false){
            end = true;
            RCLCPP_INFO(this->get_logger(), "End of measurement, writing to file...");
            long time_diff;
            FILE *fptr;
            fptr = fopen("measurements_ros2_nodes_wo_stress.txt", "w");
            for(int i = 0; i < NUM_LOOPS; i++){
                time_diff = (end_measurement[i].tv_sec - start_send[i].tv_sec) * 1000000000 + (end_measurement[i].tv_nsec - start_send[i].tv_nsec);
                fprintf(fptr, "%ld, ", time_diff);
            }
            fclose(fptr);
            RCLCPP_INFO(this->get_logger(), "End of writing to file");
        }
    }

    void loop_msg_callback(const std_msgs::msg::Int32 & msg) 
    {
        int received_msg = msg.data;
        if(received_msg < NUM_LOOPS){
            clock_gettime(CLOCK_MONOTONIC, &end_measurement[received_msg]);
        }
    }

    int counter = 0;
    struct timespec end_measurement[NUM_LOOPS];
    struct timespec start_send[NUM_LOOPS];
    bool end = false;

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
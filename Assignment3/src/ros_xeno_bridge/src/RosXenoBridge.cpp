#include "RosXenoBridge.hpp"

RosXenoBridge::RosXenoBridge() : Node("RosXenoBridge")
{
  // Initialize cross buffers & polling
  initialize();

  // Initialize subscribers, publishers, parameters etc. here...
  RCLCPP_INFO(this->get_logger(), "Creating publisher for topic 'Xeno2Ros'");
  publisher_ = this->create_publisher<custom_msgs::msg::Xeno2Ros>("Xeno2Ros", 1000);

  RCLCPP_INFO(this->get_logger(), "Subscribing to topic 'Ros2Xeno'");
  subscription_ = this->create_subscription<custom_msgs::msg::Ros2Xeno>("Ros2Xeno", 10, std::bind(&RosXenoBridge::Ros2Xeno_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Creating a timer");
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RosXenoBridge::timer_callback, this));
}

void RosXenoBridge::initialize()
{
  // Get ROS-Xeno file descriptor
  rfd = open("/dev/evl/xbuf/Ros-Xeno", O_RDWR);
  if (rfd < 0)
    error(1, errno, "open('/dev/evl/xbuf/Ros-Xeno')");

  // Get Xeno-ROS file descriptor
  xfd = open("/dev/evl/xbuf/Xeno-Ros", O_RDWR);
  if (xfd < 0)
    error(1, errno, "open('/dev/evl/xbuf/Xeno-Ros')");

  printf("ros fd: %d, xeno fd : %d \n", rfd, xfd);

  return;
}

void RosXenoBridge::Ros2Xeno_callback(const custom_msgs::msg::Ros2Xeno::SharedPtr msg)
{
  #if DEBUG_ROS2XENO
    RCLCPP_INFO(this->get_logger(), "Received data will be sent to Xenomai");
  #endif
  custom_msgs::msg::Ros2Xeno msg_copy = *msg;
  write(rfd, &msg_copy, sizeof(msg_copy));
}

void RosXenoBridge::timer_callback()
{
  custom_msgs::msg::Xeno2Ros msg;
  int ret;

  ret = read(xfd, &msg, sizeof(msg));
  if(ret>0)
  {
    #if DEBUG_XENO2ROS
      RCLCPP_INFO(this->get_logger(), "Received data from Xenomai");
    #endif
    publisher_->publish(msg);
  }
}

// The main function starts the node and "spins" it, i.e. handles all ROS2-related events such as receiving messages on topics
// You rarely need to add anything else to this function for ROS2 nodes
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosXenoBridge>());
  rclcpp::shutdown();
  return 0;
}

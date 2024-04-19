#ifndef ROS_XENO_BRIDGE_HPP
#define ROS_XENO_BRIDGE_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <poll.h>
#include <error.h>
#include <errno.h>
#include <iostream>
#include <fstream>
#include <fcntl.h>

#include "config.h"

// Define message includes here...
#include "custom_msgs/msg/ros2_xeno.hpp"
#include "custom_msgs/msg/xeno2_ros.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

class RosXenoBridge : public rclcpp::Node {
public:
    RosXenoBridge();
    int rfd, xfd = 0;

private:
    //
    void initialize();

    /// Callback functions.
    /**
     * @brief Callback to process any incoming Image messages.
     * 
     * @param img The image that was received.
    */
    void Ros2Xeno_callback(const custom_msgs::msg::Ros2Xeno::SharedPtr msg);
    //void image_callback(sensor_msgs::msg::Image::ConstSharedPtr img);
    void timer_callback();
    /// Private variables.
    // ...
    
    /// Subscriber variables.
    rclcpp::Subscription<custom_msgs::msg::Ros2Xeno>::SharedPtr subscription_;

    /// Publisher variables.
    rclcpp::Publisher<custom_msgs::msg::Xeno2Ros>::SharedPtr publisher_;
    
    /// Timer variables.
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif /* ROS_XENO_BRIDGE_HPP */


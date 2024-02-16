#ifndef RELBOT_SIMULATOR_HPP_
#define RELBOT_SIMULATOR_HPP_

// CPP library headers
#include <cstdio>
#include <chrono>
#include <memory>

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// Standard Message types
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

// Sensor message types related to images
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"

// Geometry messages, used for location notation
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// OpenCV imshow
#include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#include <cv_bridge/cv_bridge.h>

// Self written headers
#include "DynamicsSimulation.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief RELbot simulator node.
 *
 * ROS2 parameters are described below:
 *
 * @param wheel_base_width [double]: Width in meters between the robot's powered wheels. Default 0.209.
 * @param wheel_radius [double]: Wheel radius in meters for the robot's powered wheels. Default 0.05.
 * @param time_step [double]: Simulator time step in seconds. Default 0.02 (corresponding to 50 Hz).
 * @param time_constant [double]: Time constant to make motor velocities behave as 1st order system. Default 0.3.
 * @param use_twist_cmd [bool]: Which command mode to use: true is Twist, false is Individual Motors. Default is false.
 * @param global_frame [string]: Name of global frame. Default is "map".
 * @param base_link_frame [string]: Name of base link frame in URDF. Default is "base_link".
 * @param right_wheel_joint [string]: Name of right wheel joint in URDF. Default is "right_wheel_joint".
 * @param left_wheel_joint [string]: Name of left wheel joint in URDF. Default is "left_wheel_joint".
 * @param caster_swivel_joint [string]: Name of caster swivel joint in URDF. Default is "caster_swivel_joint".
 * @param caster_wheel_joint [string]: Name of caster wheel joint in URDF. Default is "caster_wheel_joint".
 * @todo Potentially add topics as parameters here too.
 */

class RELbotSimulator : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new RELbotSimulator object
   *
   * @param time_step time-step defines how quickly the motors are simulated to catch up to commands.
   */
  RELbotSimulator(double time_step);

  const std::string input = "/input";
  const std::string output = "/output";

  // Default input topic names
  const std::string CMD_VEL_TOPIC = input + "/cmd_vel";
  const std::string SETPOINT_VEL_TOPIC = "/setpoint_vel";
  const std::string RIGHT_MOTOR_NAMESPACE = input + "/right_motor";
  const std::string LEFT_MOTOR_NAMESPACE = input + "/left_motor";
  const std::string JOINT_STATES_TOPIC = input + "/joint_states";
  const std::string WEBCAM_IMAGE = "/image";

  // Default ouput topic names
  const std::string CAMERA_POSITION = output + "camera_position";
  const std::string ROBOT_POSE = output + "/pose";

  // Default parameter values
  const double DEFAULT_WHEEL_BASE_WIDTH = 0.209; // [m] Width between the center of both wheels
  const double DEFAULT_WHEEL_RADIUS = 0.05;      // [m] Radius of the wheels
  const double DEFAULT_TIME_STEP = 0.02;         // [s] Time step of the simulation
  const double DEFAULT_TIME_CONSTANT = 0.3;      // [s] Time constant to make motor velocities behave as 1st order system
  const bool DEFAULT_USE_TWIST_CMD = false;      // Which command mode (true is Twist, false is Individual Motors)

  double wheelBaseWidth_ = DEFAULT_WHEEL_BASE_WIDTH;
  double wheelRadius_ = DEFAULT_WHEEL_RADIUS;

private:
  // Topics
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr webcam_input_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr moving_camera_output_topic_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr camera_position_topic_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_topic;

  // Timer to run the simulator step function peridically
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscribers for motor commands
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rightMotorSetpointVelSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr leftMotorSetpointVelSubscriber_;

  // // Publisher for joint state

  // Dynamics simulation timer (plus position output)
  rclcpp::TimerBase::SharedPtr dynamics_timer_;
  DynamicsSimulation dynamics_simulation_ = DynamicsSimulation(this, DEFAULT_TIME_STEP);

  bool useTwistCmd_ = DEFAULT_USE_TWIST_CMD;

  // Container for building the output image as a message type.
  cv::Mat output_image_;
  std::shared_ptr<sensor_msgs::msg::Image> sensor_output_img;

  /**
   * @brief Create all topics for this node
   *
   */
  void create_topics();

  /**
   * @brief Image manipulation method to change webcam image into sub-image which is returned. Internally uses openCV methods
   *
   * @param msg_cam_img webcam image in ROS sensor format
   * @param center_pixel_x center pixel of the view along the x-axis (pan)
   * @param center_pixel_y center pixel of the view along the y-axis (height)
   * @param output_image_dim size of output image (in pixels), will not deterimine window size!
   * @return cv::Mat Used to turn back into sensor_img for between-ros communication
   */
  cv::Mat CreateCVSubimage(const sensor_msgs::msg::Image::SharedPtr msg_cam_img, const int center_pixel_x, const int center_pixel_y, int output_image_dim);

  /**
   * @brief Handles receiving and processing of received webcam images
   *
   * @param msg_cam_img webcam image, in sensor_msgs::msg::Image format
   */
  void webcam_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img);

  /**
   * @brief Main step loop of the system. Calls the hidden Dynamics object to update to the next timestep
   *
   */
  void dynamics_timer_callback();

  /**
   * @brief Callback upon receiving a Twist command velocity. Converts twist into individual motor commands to store
   * internally for the next simulation step.
   *
   * @param cmdVel Velocity (in rad/s) for motor to be set
   */
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr cmdVel);

  /**
   * @brief Callback upon receiving the right motor command velocity. Stores this command internally for the next
   * simulation step.
   *
   * @param setpoinVel setpoint velocity for right motor
   */
  void rightMotorSetpointVelCallback(const std_msgs::msg::Float64::SharedPtr setpointVel);

  /**
   * @brief Callback upon receiving the left motor command velocity. Stores this command internally for the next
   * simulation step.
   *
   * @param setpiontVel setpoint velocity for left motor
   */
  void leftMotorSetpointVelCallback(const std_msgs::msg::Float64::SharedPtr setpointVel);
};
#endif // RELBOT_SIMULATOR_HPP_
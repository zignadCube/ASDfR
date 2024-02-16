#ifndef DYNAMICSSIMULATION_HPP_
#define DYNAMICSSIMULATION_HPP_

#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"

/**
 * @brief First-order dynamic system for the orientation of the camera, including end-stops
 */
class DynamicsSimulation
{
public:
  double x_des_pos, x_cur_pos, y_des_pos, y_cur_pos, theta_cur, theta_des;

  /**
   * @brief Construct a new Dynamics Simulation object
   * 
   * @param parent_node the parent node (RELbot sim) which contains the actual ROS-connections
   * @param time_step Time step
   */
  DynamicsSimulation(rclcpp::Node *parent_node, double time_step);

  /**
   * @brief Construct a new Dynamics Simulation object. Default constructor, not implemented
   * 
   */
  DynamicsSimulation() = default;

  /**
   * @brief Reset the camera angle to (0,0)
   */
  void reset();

  /**
   * @brief Perform one time step; computing the new state.
   *
   */
  void step();

  // void create_parameters();

  double get_x();
  double get_y();
  double get_theta();

  double get_x_limit();
  double get_y_limit();

  void set_vel_right_motor_set_point(const double x);
  void set_vel_left_motor_set_point(const double y);

private:
  const int max_motor_vel = 24;

  rclcpp::Node *parent_node_;

  // Default parameter values
  const double DEFAULT_WHEEL_BASE_WIDTH = 0.209; // [m] Width between the center of both wheels
  const double DEFAULT_WHEEL_RADIUS = 0.05;      // [m] Radius of the wheels
  const double DEFAULT_TIME_STEP = 0.01;         // [s] Time step of the simulation
  const double DEFAULT_TIME_CONSTANT = 0.3;      // [s] Time constant to make motor velocities behave as 1st order system
  // const bool DEFAULT_USE_TWIST_CMD = false;  // Which command mode (true is Twist, false is Individual Motors)

  // Internally stored parameters
  double wheelBaseWidth_ = DEFAULT_WHEEL_BASE_WIDTH;
  double wheelRadius_ = DEFAULT_WHEEL_RADIUS;
  double timeStep_ = DEFAULT_TIME_STEP;
  double timeConstant_ = DEFAULT_TIME_CONSTANT;

  // Robot state
  double xWorld_ = 0.0;          // [m] Base link X coordinate w.r.t. the world frame
  double yWorld_ = 0.0;          // [m] Base link Y coordinate w.r.t. the world frame
  double thetaWorld_ = 0.0;      // [rad] Base link angle (about the z-axis) w.r.t. the world frame
  double angleRightMotor_ = 0.0; // [rad] Current angle of the right motor w.r.t. base frame
  double angleLeftMotor_ = 0.0;  // [rad] current angle of the left motor w.r.t. base frame
  double velRightMotor_ = 0.0;   // [rad/s] Current angular velocity of the right motor w.r.t. base frame
  double velLeftMotor_ = 0.0;    // [rad/s] current angular velocity of the left motor w.r.t. base frame

  // Store last received commands internally
  double velRightMotorSetpoint_ = 0.0;
  double velLeftMotorSetpoint_ = 0.0;

  double vel_limit = 0.2;

  double time_step_; // Approximate time step in seconds
};

#endif // DYNAMICSSIMULATION_HPP_
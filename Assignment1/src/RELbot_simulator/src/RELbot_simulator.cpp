#include "RELbot_simulator.hpp"
#include "DynamicsSimulation.hpp"

RELbotSimulator::RELbotSimulator(double time_step) : Node("RELbot_simulator")
{
  RCLCPP_INFO(get_logger(), "Init");
  // dynamics_simulation_ = DynamicsSimulation(this, time_step);
  create_topics();
  RCLCPP_INFO(get_logger(), "Created Topics");
  dynamics_timer_ = this->create_wall_timer(std::chrono::duration<double>(time_step), std::bind(&RELbotSimulator::dynamics_timer_callback, this));
  RCLCPP_INFO(get_logger(), "Created Timer");
  dynamics_simulation_.reset();
  RCLCPP_INFO(get_logger(), "Reset dyn_sim to 0");
}

void RELbotSimulator::create_topics()
{
  declare_parameter<bool>("use_twist_cmd", RELbotSimulator::DEFAULT_USE_TWIST_CMD);

  RCLCPP_INFO(this->get_logger(), "Creating topics...");
  RCLCPP_INFO(this->get_logger(), "Creating Publishers");

  // outputs
  moving_camera_output_topic_ = this->create_publisher<sensor_msgs::msg::Image>("output/moving_camera", 1);
  camera_position_topic_ = this->create_publisher<geometry_msgs::msg::PointStamped>("output/camera_position", 1);
  robot_pose_topic = this->create_publisher<geometry_msgs::msg::PoseStamped>("output/robot_pose", 1);

  // get param, can be set by ros2 launch command
  useTwistCmd_ = get_parameter("use_twist_cmd").as_bool();

  RCLCPP_INFO(this->get_logger(), "Creating Subscriptions");
  RCLCPP_INFO(this->get_logger(), "Subscribing to %s", RELbotSimulator::WEBCAM_IMAGE.c_str());

  webcam_input_topic_ = this->create_subscription<sensor_msgs::msg::Image>(
      RELbotSimulator::WEBCAM_IMAGE, 10, std::bind(&RELbotSimulator::webcam_topic_callback, this, _1));

  if (RELbotSimulator::useTwistCmd_)
  {
    RCLCPP_INFO(get_logger(), "Using Twist Command mode");

    RELbotSimulator::cmdVelSubscriber_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        RELbotSimulator::CMD_VEL_TOPIC, rclcpp::SensorDataQoS().reliable(),
        std::bind(&RELbotSimulator::cmdVelCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", RELbotSimulator::CMD_VEL_TOPIC.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Using Individual Motors Command mode");

    // The motor topics are in a namespace, construct the full topic name first
    const std::string right_motor_setpoint_vel_topic =
        RELbotSimulator::RIGHT_MOTOR_NAMESPACE + RELbotSimulator::SETPOINT_VEL_TOPIC;
    RELbotSimulator::rightMotorSetpointVelSubscriber_ = create_subscription<std_msgs::msg::Float64>(
        right_motor_setpoint_vel_topic, 10,
        std::bind(&RELbotSimulator::rightMotorSetpointVelCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", right_motor_setpoint_vel_topic.c_str());

    const std::string left_motor_setpoint_vel_topic =
        RELbotSimulator::LEFT_MOTOR_NAMESPACE + RELbotSimulator::SETPOINT_VEL_TOPIC;
    RELbotSimulator::leftMotorSetpointVelSubscriber_ = create_subscription<std_msgs::msg::Float64>(
        left_motor_setpoint_vel_topic, 10,
        std::bind(&RELbotSimulator::leftMotorSetpointVelCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", left_motor_setpoint_vel_topic.c_str());
  }
}

void RELbotSimulator::webcam_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img)
{
  // RCLCPP_INFO(this->get_logger(), "Received webcam frame");

  /*
  Assumption:
  At init, Right-Hand (RH) reference frame or RELbot and world are equal. Relbot moving will shift X (forward/backward), Y (left/right). Theta describes angle w.r.t. world.
  GOAL:
  Forward     - Image shrinks
  Backwards   - Image 'grows', can over-extend to be img + black padding
  Left/Right  - Nothing (assume img is @infinity, so L/R does nothing)
  Theta       - Pan L/R over IMG, can also extend outside borders

  Requirement: Output IMG can't change in actual size, due to allocation issues, so re-scaling might be needed

  Difficulty:
  - Img uses ARRAY coordinates, we use RH-Coord Frame
  */

  double x = dynamics_simulation_.get_x();         // Positive = Forward
  double theta = dynamics_simulation_.get_theta(); // Positive = counter-clockwise rotation

  const double height = msg_cam_img->height;
  const double width = msg_cam_img->width;

  const double img_width_rad = 120 * (M_PI / 180);           // Define 'radial width'  of full cam img)
  const double pixels_per_rad = (width / 2) / img_width_rad; // convert to cam img we get

  int center_pixel_x = (int)(msg_cam_img->width / 2) - (theta * pixels_per_rad); // transform into what our center_x should be

  int center_pixel_y = (int)(msg_cam_img->height / 2); // Mostly static

  int output_image_dim = (int)(height / 2 - (x * height / 10)); // 0 starts at height/2, so somehwat zoomed.

  output_image_ = RELbotSimulator::CreateCVSubimage(msg_cam_img, center_pixel_x, center_pixel_y, output_image_dim);

  // transform img back to sensor_msg
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = get_clock()->now();
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = output_image_;

  sensor_output_img = out_msg.toImageMsg();

  moving_camera_output_topic_->publish(*sensor_output_img.get());

  // make camera_pos topic
  geometry_msgs::msg::PointStamped camera_position;

  // Set header timestamp, useful for ordering messages if they are somehow jumbled
  camera_position.header.stamp = get_clock()->now();
  camera_position.point.set__x(center_pixel_x);
  camera_position.point.set__y(center_pixel_y);
  camera_position_topic_->publish(camera_position);
}

cv::Mat RELbotSimulator::CreateCVSubimage(const sensor_msgs::msg::Image::SharedPtr msg_cam_img, const int center_pixel_x, const int center_pixel_y, int output_image_dim)
{
  cv::Mat resized_frame;
  cv::Mat cv_frame = cv_bridge::toCvCopy(msg_cam_img, "bgr8" /* or other encoding */)->image;

  cv::Size size = cv_frame.size();

  const int window_dim = (int)(msg_cam_img->height / 2); // Dirty calculation of window size we want to show in

  int leftmost_pixel_x = center_pixel_x - window_dim / 2;
  int topmost_pixel_y = center_pixel_y - window_dim / 2;

  // Make edges 'stick, such that we dont get out of bounds
  output_image_dim = std::clamp(output_image_dim, 1, size.height);
  leftmost_pixel_x = std::clamp(leftmost_pixel_x, 0, size.width - output_image_dim);
  topmost_pixel_y = std::clamp(topmost_pixel_y, 0, size.height - output_image_dim);

  // Define the rectangle of interest (pixel location + output width (aka, zoom))
  cv::Rect rect = cv::Rect(leftmost_pixel_x, topmost_pixel_y, output_image_dim, output_image_dim);
  // turn that into valid sub-image
  cv::Mat subImg = cv_frame(rect);
  // resize into default size (so the window stays the same size)
  cv::resize(subImg, resized_frame, cv::Size(window_dim, window_dim), cv::INTER_LINEAR);
  // timer to show we are still receivng an image, ticks every second
  auto &clk = *this->get_clock();
  RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000, "ouput image");

  cv::imshow("output", resized_frame);
  cv::waitKey(1);

  return resized_frame;
}

void RELbotSimulator::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr cmdVel)
{
  // Extract the base link velocity components from the twist
  double linearVelSetpoint = cmdVel->twist.linear.x;
  double angularVelSetpoint = cmdVel->twist.angular.z;

  // Convert the base link velocity setpoints to individual motor setpoints
  // http://wiki.ros.org/diff_drive_controller#Mathematical_Background
  dynamics_simulation_.set_vel_right_motor_set_point((linearVelSetpoint + angularVelSetpoint * wheelBaseWidth_ / 2) / wheelRadius_);
  dynamics_simulation_.set_vel_left_motor_set_point((linearVelSetpoint - angularVelSetpoint * wheelBaseWidth_ / 2) / wheelRadius_);
}

void RELbotSimulator::rightMotorSetpointVelCallback(const std_msgs::msg::Float64::SharedPtr setpointVel)
{
  dynamics_simulation_.set_vel_right_motor_set_point(setpointVel->data);
  // velRightMotorSetpoint_ = setpointVel->data;
}

void RELbotSimulator::leftMotorSetpointVelCallback(const std_msgs::msg::Float64::SharedPtr setpointVel)
{
  dynamics_simulation_.set_vel_left_motor_set_point(setpointVel->data);
  // velLeftMotorSetpoint_ = setpointVel->data;
}

void RELbotSimulator::dynamics_timer_callback()
{
  // Do dynamics integration step
  dynamics_simulation_.step();

  // output robot pose
  geometry_msgs::msg::PoseStamped robot_pose;

  robot_pose.header.stamp = get_clock()->now();
  robot_pose.pose.position.set__x(dynamics_simulation_.get_x());
  robot_pose.pose.position.set__y(dynamics_simulation_.get_y());
  robot_pose.pose.orientation.set__z(dynamics_simulation_.get_theta());
  robot_pose_topic->publish(robot_pose);

  // Output the actual position

  // RCLCPP_INFO(this->get_logger(), "Dynamics timer!   [x,y] = [%f,%f]", pos.x, pos.y);
}

int main(int argc, char *argv[])
{
  printf("RELbot Simulator Node\n-------------------\n");
  rclcpp::init(argc, argv);
  auto RELbot_simulator = std::make_shared<RELbotSimulator>(0.01);
  rclcpp::spin(RELbot_simulator);
  printf("Done Spinning\n-------------------\n");
  rclcpp::shutdown();
  return 0;
}

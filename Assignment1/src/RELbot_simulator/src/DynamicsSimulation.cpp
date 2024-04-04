#include "DynamicsSimulation.hpp"

DynamicsSimulation::DynamicsSimulation(rclcpp::Node *parent_node, double time_step)
    : parent_node_{parent_node}
{
  reset();
  time_step_ = time_step;
}

void DynamicsSimulation::reset()
{
  xWorld_ = 0.0;
  yWorld_ = 0.0;
  thetaWorld_ = 0.0;
  angleRightMotor_ = 0.0;
  angleLeftMotor_ = 0.0;
  velRightMotor_ = 0.0;
  velLeftMotor_ = 0.0;
}

void DynamicsSimulation::step()
{
  double linearVel = wheelRadius_ * (velRightMotor_ + velLeftMotor_) / 2;
  double angularVel = wheelRadius_ * (velRightMotor_ - velLeftMotor_) / wheelBaseWidth_;

  // Calculate state updates
  double xWorldDot = linearVel * cos(thetaWorld_);
  double yWorldDot = linearVel * sin(thetaWorld_);
  double thetaWorldDot = angularVel;
  double angleRightMotorDot = velRightMotor_;
  double angleLeftMotorDot = velLeftMotor_;
  double velRightMotorDot = (velRightMotorSetpoint_ - velRightMotor_) / timeConstant_;
  double velLeftMotorDot = (velLeftMotorSetpoint_ - velLeftMotor_) / timeConstant_;

  // Update the states (using forward euler)
  xWorld_ = xWorld_ + xWorldDot * timeStep_;
  yWorld_ = yWorld_ + yWorldDot * timeStep_;
  thetaWorld_ = thetaWorld_ + thetaWorldDot * timeStep_;
  angleRightMotor_ = angleRightMotor_ + angleRightMotorDot * timeStep_;
  angleLeftMotor_ = angleLeftMotor_ + angleLeftMotorDot * timeStep_;
  velRightMotor_ = velRightMotor_ + velRightMotorDot * timeStep_;
  velLeftMotor_ = velLeftMotor_ + velLeftMotorDot * timeStep_;
}

double DynamicsSimulation::get_x()
{
  return xWorld_;
}

double DynamicsSimulation::get_y()
{
  return yWorld_;
}

double DynamicsSimulation::get_theta()
{
  return thetaWorld_;
}

double DynamicsSimulation::get_x_limit()
{
  return parent_node_->get_parameter("x_limit_rad").as_double();
}

double DynamicsSimulation::get_y_limit()
{
  return parent_node_->get_parameter("y_limit_rad").as_double();
}

void DynamicsSimulation::set_vel_right_motor_set_point(const double x)
{
  velRightMotorSetpoint_ = x;
}

void DynamicsSimulation::set_vel_left_motor_set_point(const double y)
{
  velLeftMotorSetpoint_ = y;
}

/*
This code based on https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

*/


#include <iostream>
#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RobotDriver : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
  RobotDriver() : Node("robot_driver")
  {
    // Set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_joy", 10);
    // Set up the transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  bool driveForwardOdom(double distance)
  {
    // Wait for the listener to get the first message
    geometry_msgs::msg::TransformStamped start_transform;
    try
    {
      start_transform = tf_buffer_->lookupTransform("base_footprint", "odom", tf2::TimePointZero, tf2::durationFromSec(1.0));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return false;
    }

    // We will be sending commands of type "twist"
    geometry_msgs::msg::Twist base_cmd;
    // The command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;

    rclcpp::Rate rate(10.0);
    bool done = false;
    while (!done && rclcpp::ok())
    {
      // Send the drive command
      cmd_vel_pub_->publish(base_cmd);
      rate.sleep();

      // Get the current transform
      geometry_msgs::msg::TransformStamped current_transform;
      try
      {
        current_transform = tf_buffer_->lookupTransform("base_footprint", "odom", tf2::TimePointZero);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        break;
      }

      // See how far we've traveled
      tf2::Transform start, current;
      tf2::fromMsg(start_transform.transform, start);
      tf2::fromMsg(current_transform.transform, current);

      tf2::Transform relative_transform = start.inverse() * current;
      double dist_moved = relative_transform.getOrigin().length();

      if (dist_moved > distance)
        done = true;
    }
    if (done)
      return true;
    return false;
  }

  bool turnOdom(bool clockwise, double degrees)
{
  // Convert degrees to radians
  double radians = degrees * M_PI / 180.0;

  while (radians < 0)
    radians += 2 * M_PI;
  while (radians > 2 * M_PI)
    radians -= 2 * M_PI;

  // Wait for the listener to get the first message
  geometry_msgs::msg::TransformStamped start_transform;
  try
  {
    start_transform = tf_buffer_->lookupTransform("base_footprint", "odom", tf2::TimePointZero, tf2::durationFromSec(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return false;
  }

  // We will be sending commands of type "twist"
  geometry_msgs::msg::Twist base_cmd;
  // The command will be to turn at 0.75 rad/s
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = 0.15;
  if (clockwise)
    base_cmd.angular.z = -base_cmd.angular.z;

  // The axis we want to be rotating by
  tf2::Vector3 desired_turn_axis(0, 0, 1);
  if (!clockwise)
    desired_turn_axis = -desired_turn_axis;

  rclcpp::Rate rate(10.0);
  bool done = false;
  while (!done && rclcpp::ok())
  {
    // Send the drive command
    cmd_vel_pub_->publish(base_cmd);
    rate.sleep();

    // Get the current transform
    geometry_msgs::msg::TransformStamped current_transform;
    try
    {
      current_transform = tf_buffer_->lookupTransform("base_footprint", "odom", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      break;
    }

    // See how far we've turned
    tf2::Transform start, current;
    tf2::fromMsg(start_transform.transform, start);
    tf2::fromMsg(current_transform.transform, current);

    tf2::Transform relative_transform = start.inverse() * current;
    tf2::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    if (fabs(angle_turned) < 1.0e-2)
      continue;

    if (actual_turn_axis.dot(desired_turn_axis) < 0)
      angle_turned = 2 * M_PI - angle_turned;

    if (angle_turned > radians)
      done = true;
  }
  if (done)
    return true;
  return false;
}
};

int main(int argc, char **argv)
{
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);

  auto driver = std::make_shared<RobotDriver>();
  // driver->driveForwardOdom(0.5);
  driver->turnOdom(false, 90);
  rclcpp::shutdown();
  return 0;
}

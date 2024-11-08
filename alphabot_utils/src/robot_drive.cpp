/*
This code based on https://wiki.ros.org/pr2_controllers/Tutorials/Using%20the%20base%20controller%20with%20odometry%20and%20transform%20information

*/


#include <iostream>
#include <memory>
#include <cmath>
#include <stdlib.h>

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
  double Kp_linear = 0.3;  // Proportional gain for linear velocity
  double Kp_angular = 0.4; // Proportional gain for angular velocity

public:
  RobotDriver() : Node("robot_driver")
  {
    // Set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_joy", 10);
    // Set up the transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters
    this->declare_parameter("Kp_linear", 0.5);
    this->declare_parameter("Kp_angular", 0.2);

    //Get initial parameter values
    this->get_parameter("Kp_linear", Kp_linear);
    this->get_parameter("Kp_angular", Kp_angular);
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

    // Send commands of type "twist"
    geometry_msgs::msg::Twist base_cmd;
    // The command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;

    rclcpp::Rate rate(10.0);
    bool done = false;
    while (!done && rclcpp::ok())
    {
      
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

      double error = distance - dist_moved;
      base_cmd.linear.x = error*Kp_linear;
      
      // Send the drive command
      cmd_vel_pub_->publish(base_cmd);

      if (abs(error) < 0.01)
        done = true;

      rate.sleep();

    }

    // Stop the robot
    base_cmd.linear.x = 0;
    cmd_vel_pub_->publish(base_cmd);
    
    return done;
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

    // Initialize the base command
    geometry_msgs::msg::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = 0.0;

    rclcpp::Rate rate(10.0);
    bool done = false;
    while (!done && rclcpp::ok())
    {
      // Get the current transform
      geometry_msgs::msg::TransformStamped current_transform;
      try
      {
        current_transform = tf_buffer_->lookupTransform("/base_footprint", "/odom", tf2::TimePointZero);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        break;
      }

      // Calculate the angle turned
      tf2::Transform start, current;
      tf2::fromMsg(start_transform.transform, start);
      tf2::fromMsg(current_transform.transform, current);

      tf2::Transform relative_transform = start.inverse() * current;
      tf2::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
      double angle_turned = relative_transform.getRotation().getAngle();
      if (fabs(angle_turned) < 1.0e-2)
        continue;

      if (actual_turn_axis.dot(clockwise ? -tf2::Vector3(0, 0, 1) : tf2::Vector3(0, 0, 1)) < 0)
        angle_turned = 2 * M_PI - angle_turned;

      // Calculate the error
      double error = radians - angle_turned;

      // Proportional control: adjust angular velocity based on the error
      base_cmd.angular.z = Kp_angular * error * (clockwise ? -1 : 1);
      std::cout<<"turing speed: "<< base_cmd.angular.z <<std::endl;
      // Send the turn command
      cmd_vel_pub_->publish(base_cmd);

      // Check if the goal is reached
      if (fabs(error) < 0.01) // Threshold to stop
        done = true;

      rate.sleep();
    }

    // Stop the robot
    base_cmd.angular.z = 0;
    cmd_vel_pub_->publish(base_cmd);

    return done;
  }
};

int main(int argc, char **argv)
{

  float distance = 0.5;

  std::cout<<distance<<std::endl;
  // Initialize the ROS 2 node
  rclcpp::init(argc, argv);

  auto driver = std::make_shared<RobotDriver>();
  driver->driveForwardOdom(distance);
  // driver->turnOdom(false, 90);
  // driver->driveForwardOdom(1.0);
  // driver->turnOdom(false, 90);
  // driver->driveForwardOdom(1.0);
  // driver->turnOdom(false, 90);
  // driver->driveForwardOdom(1.0);
  rclcpp::shutdown();
  return 0;
}

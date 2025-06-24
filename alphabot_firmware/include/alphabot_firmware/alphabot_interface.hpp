#ifndef ALPHABOT_INTERFACE_HPP
#define ALPHABOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <string>
#include <cmath>



struct IMU_DATA
{
  double orientation_x = 0.0;
  double orientation_y = 0.0;
  double orientation_z = 0.0;
  double orientation_w = 0.0;
  double angular_velocity_x = 0.0;
  double angular_velocity_y = 0.0;
  double angular_velocity_z = 0.0;
  double linear_acceleration_x = 0.0;
  double linear_acceleration_y = 0.0;
  double linear_acceleration_z = 0.0;
};
namespace alphabot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AlphabotInterface : public hardware_interface::SystemInterface
{
public:
  AlphabotInterface();
  virtual ~AlphabotInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  // virtual CallbackReturn on_configure(const hardware_interface::HardwareInfo & info) override;

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  virtual hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  std::thread spinner_thread_;

  LibSerial::SerialPort arduino_;
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // Model base_model;
  std::chrono::time_point<std::chrono::system_clock> time_;

  volatile long tick_left = 0;
  volatile long tick_right = 0;

  double left_current_pos = 0;
  double right_current_pos = 0;

  double left_prevouse_pos = 0;
  double right_prevouse_pos = 0;

  double WHEEL_DIAMETER = 0.07; //m
  double WHEEL_SEPERATION = 0.230; //m
  double TICK_PER_REVOLUTION = 232.0; 
  double RADIUS_PER_TICK = (2*M_PI)/TICK_PER_REVOLUTION;
  double CONVERT_TO_RPM_FACTOR = 60/(M_PI*WHEEL_DIAMETER);
  double left_velocity = 0;
  double right_velocity = 0;
  rclcpp::Time last_run_;
  // cm traveled each gear tick
  const double DIST_PER_TICK = (WHEEL_DIAMETER * M_PI) / TICK_PER_REVOLUTION;

  double vRFilt = 0;
  double vRPrev = 0;
  double vLFilt = 0;
  double vLPrev = 0;
  

  // IMU data 
  // IMU_DATA imu_data;

};
}  // namespace Alphabot_firmware


#endif  // Alphabot_INTERFACE_HPP
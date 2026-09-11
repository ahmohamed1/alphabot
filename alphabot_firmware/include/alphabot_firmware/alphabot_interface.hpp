#ifndef ALPHABOT_FIRMWARE__ALPHABOT_INTERFACE_HPP_
#define ALPHABOT_FIRMWARE__ALPHABOT_INTERFACE_HPP_

#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float32.hpp>

namespace alphabot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AlphabotInterface : public hardware_interface::SystemInterface
{
public:
  AlphabotInterface() = default;
  ~AlphabotInterface() override;

  // hardware_interface::SystemInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  bool pico_connection_confirmed_{false};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  std::thread spinner_thread_;

  LibSerial::SerialPort arduino_;
  std::string port_;

  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
};

}  // namespace alphabot_firmware

#endif  // ALPHABOT_FIRMWARE__ALPHABOT_INTERFACE_HPP_
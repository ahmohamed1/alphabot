#include "alphabot_firmware/alphabot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace alphabot_firmware
{
AlphabotInterface::AlphabotInterface()
{
}


AlphabotInterface::~AlphabotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("AlphabotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn AlphabotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("AlphabotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  velocity_states_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> AlphabotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> AlphabotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn AlphabotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("AlphabotInterface"), "Starting robot hardware ...");

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0 };
  position_states_ = { 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0 };

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("AlphabotInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("AlphabotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn AlphabotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("AlphabotInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("AlphabotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AlphabotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type AlphabotInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Interpret the string
  if(arduino_.IsDataAvailable())
  {
    std::string message;
    arduino_.ReadLine(message);
    if(message.at(0) == 'v')
    {
      message.erase(0,1);
      std::string delimiter = ",";
      size_t del_pos = message.find(delimiter);
      std::string token_1 = message.substr(0, del_pos);
      std::string token_2 = message.substr(del_pos + delimiter.length());
      velocity_states_.at(0) = std::stod(token_1.c_str());
      velocity_states_.at(1) = std::stod(token_2.c_str());
    }
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type AlphabotInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{

   std::stringstream message_stream;
        message_stream << std::fixed <<std::setprecision(2) << "v" << 
        velocity_commands_[0] <<"," <<velocity_commands_[1] <<"\n";

  try
  {
    arduino_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlphabotInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace alphabot_firmware

PLUGINLIB_EXPORT_CLASS(alphabot_firmware::AlphabotInterface, hardware_interface::SystemInterface)
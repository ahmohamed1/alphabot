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


// CallbackReturn AlphabotInterface::on_configure(const hardware_interface::HardwareInfo & info)
// {
//   return CallbackReturn::SUCCESS;
// }

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
  // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_states_[0]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[0]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &position_states_[1]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[1]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("orientation_x", hardware_interface::HW_IF_POSITION,    &imu_data.orientation_x));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("orientation_y", hardware_interface::HW_IF_POSITION,    &imu_data.orientation_y));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("orientation_z", hardware_interface::HW_IF_POSITION,    &imu_data.orientation_z));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("orientation_w", hardware_interface::HW_IF_POSITION,    &imu_data.orientation_w));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("angular_velocity_x", hardware_interface::HW_IF_VELOCITY,    &imu_data.angular_velocity_x));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("angular_velocity_y", hardware_interface::HW_IF_VELOCITY,    &imu_data.angular_velocity_y));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("angular_velocity_z", hardware_interface::HW_IF_VELOCITY,    &imu_data.angular_velocity_z));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("linear_acceleration_x", hardware_interface::HW_IF_ACCELERATION, &imu_data.linear_acceleration_x));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("linear_acceleration_y", hardware_interface::HW_IF_ACCELERATION, &imu_data.linear_acceleration_y));
  // state_interfaces.emplace_back(hardware_interface::StateInterface("linear_acceleration_z", hardware_interface::HW_IF_ACCELERATION, &imu_data.linear_acceleration_z));

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
  RCLCPP_INFO_STREAM(rclcpp::get_logger("AlphabotInterface"), "New message received: "<< CONVERT_TO_RPM_FACTOR);
    

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0 };
  position_states_ = { 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0 };
  // imu_data.orientation_x = 0.0;
  // imu_data.orientation_y = 0.0;
  // imu_data.orientation_z =0.0;
  // imu_data.orientation_w =0.0;
  // imu_data.angular_velocity_x = 0.0;
  // imu_data.angular_velocity_y = 0.0;
  // imu_data.angular_velocity_z = 0.0;
  // imu_data.linear_acceleration_x = 0.0;
  // imu_data.linear_acceleration_y = 0.0;
  // imu_data.linear_acceleration_z = 0.0;



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

  try
  {
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("AlphabotInterface"), "New message received: "<< CONVERT_TO_RPM_FACTOR <<" , " << message_stream.str());
    arduino_.Write("e\n");
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlphabotInterface"),
                        "Something went wrong while sending the message "
                            << "e\n" << " to the port " << port_);
    return CallbackReturn::ERROR;
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
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    arduino_.ReadLine(message);
    std::stringstream ss(message);
    std::string res;
    int multiplier = 1;
    while(std::getline(ss, res, ','))
    {
      multiplier = res.at(1) == 'p' ? 1 : -1;

      if(res.at(0) == 'r')
      {
        velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(0) += velocity_states_.at(0) * dt;
      }
      else if(res.at(0) == 'l')
      {
        velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
        position_states_.at(1) += velocity_states_.at(1) * dt;
      }
    }
    last_run_ = rclcpp::Clock().now();
  }

  
  // }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type AlphabotInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
// Implement communication protocol with the Arduino
  std::stringstream message_stream;
  char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
  char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
  std::string compensate_zeros_right = "";
  std::string compensate_zeros_left = "";
  if(std::abs(velocity_commands_.at(0)) < 10.0)
  {
    compensate_zeros_right = "0";
  }
  else
  {
    compensate_zeros_right = "";
  }
  if(std::abs(velocity_commands_.at(1)) < 10.0)
  {
    compensate_zeros_left = "0";
  }
  else
  {
    compensate_zeros_left = "";
  }
  
  message_stream << std::fixed << std::setprecision(2) << 
    "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) << 
    ",l" <<  left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

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
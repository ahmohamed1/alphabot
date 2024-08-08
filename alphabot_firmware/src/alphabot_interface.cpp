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
  RCLCPP_INFO_STREAM(rclcpp::get_logger("AlphabotInterface"), "New message received: "<< CONVERT_TO_RPM_FACTOR);
    

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
  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  if(arduino_.IsDataAvailable())
  {
    std::string message;
    arduino_.ReadLine(message);
    std::stringstream ss(message);
    std::string res;
    
    std::string delimiter = ",";
    size_t del_pos = message.find(delimiter);
    std::string token_1 = message.substr(0, del_pos);
    std::string token_2 = message.substr(del_pos + delimiter.length());

    tick_left = std::atof(token_1.c_str());
    tick_right = std::atof(token_2.c_str());

    //---------------------------------------//

    left_current_pos = tick_left * RADIUS_PER_TICK;
    right_current_pos = tick_right * RADIUS_PER_TICK;

    left_velocity = (left_current_pos - left_prevouse_pos) / deltaSeconds;
    right_velocity = (right_current_pos - right_prevouse_pos) / deltaSeconds;
    
    // low pass filter (25 Hz cutoff)
    vLFilt = 0.854*vLFilt + 0.0728* left_velocity + 0.0728*vLPrev;
    vLPrev = vLFilt;
    vRFilt = 0.854*vRFilt + 0.0728* right_velocity + 0.0728*vRPrev;
    vRPrev = vRFilt;


    position_states_[0] = left_current_pos;
    position_states_[1] = right_current_pos;
    velocity_states_[0] = vLFilt;
    velocity_states_[1] = vRFilt;

    left_prevouse_pos = left_current_pos;
    right_prevouse_pos = right_current_pos;
    
  }


  return hardware_interface::return_type::OK;
}


hardware_interface::return_type AlphabotInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
// Implement communication protocol with the Arduino
  double left_rpm = velocity_commands_[1] / WHEEL_DIAMETER;// * CONVERT_TO_RPM_FACTOR;
  double right_rpm = velocity_commands_[0] / WHEEL_DIAMETER;// * CONVERT_TO_RPM_FACTOR;

  std::stringstream message_stream;
  message_stream << std::fixed << std::setprecision(2) << 
  "r" << right_rpm << 
  ",l"  << left_rpm << ",\n";
  
  try
  {
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("AlphabotInterface"), "New message received: "<< CONVERT_TO_RPM_FACTOR <<" , " << message_stream.str());
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
#include "alphabot_firmware/alphabot_interface.hpp"

#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace
{
std::mutex serialMutex;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magneticFieldPublisher;
rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr bumperPublisher;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ledCommandSubscription;

std::vector<std::string> split(const std::string &line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;

  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }

  return fields;
}

bool readDouble(const std::string &text, double &value)
{
  try {
    size_t parsed = 0;
    value = std::stod(text, &parsed);
    return parsed == text.size() && std::isfinite(value);
  } catch (...) {
    return false;
  }
}
}  // namespace

namespace alphabot_firmware
{
AlphabotInterface::~AlphabotInterface()
{
  if (node_) {
    rclcpp::shutdown();
  }

  if (spinner_thread_.joinable()) {
    spinner_thread_.join();
  }

  if (arduino_.IsOpen()) {
    try {
      std::lock_guard<std::mutex> lock(serialMutex);
      arduino_.Close();
    } catch (...) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlphabotInterface"), "Could not close " << port_);
    }
  }
}

CallbackReturn AlphabotInterface::on_init(const hardware_interface::HardwareInfo &hardwareInfo)
{
  const CallbackReturn result = hardware_interface::SystemInterface::on_init(hardwareInfo);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("AlphabotInterface"), "Exactly two wheel joints are required.");
    return CallbackReturn::FAILURE;
  }

  try {
    port_ = info_.hardware_parameters.at("port");
  } catch (const std::out_of_range &) {
    RCLCPP_ERROR(rclcpp::get_logger("AlphabotInterface"), "Missing required hardware parameter: port");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.assign(2, 0.0);
  position_states_.assign(2, 0.0);
  velocity_states_.assign(2, 0.0);
  node_ = rclcpp::Node::make_shared("alphabot_hw_node");

  battery_pub_ = node_->create_publisher<std_msgs::msg::Float32>("battery_voltage", 10);
  bumperPublisher = node_->create_publisher<std_msgs::msg::UInt8>("bumper_state", 10);
  imuPublisher = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  magneticFieldPublisher = node_->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);

  ledCommandSubscription = node_->create_subscription<std_msgs::msg::String>(
      "led_command", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (message->data.rfind("LED,", 0) != 0 || !arduino_.IsOpen()) {
          return;
        }

        try {
          std::lock_guard<std::mutex> lock(serialMutex);
          arduino_.Write(message->data + "\n");
        } catch (...) {
          RCLCPP_ERROR(rclcpp::get_logger("AlphabotInterface"), "Could not send LED command.");
        }
      });

  spinner_thread_ = std::thread([this]() { rclcpp::spin(node_); });
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AlphabotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_POSITION, &position_states_[index]);
    interfaces.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[index]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> AlphabotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t index = 0; index < info_.joints.size(); ++index) {
    interfaces.emplace_back(info_.joints[index].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[index]);
  }

  return interfaces;
}

CallbackReturn AlphabotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    std::lock_guard<std::mutex> lock(serialMutex);
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    arduino_.Write("S,0.000,0.000\n");
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlphabotInterface"), "Could not open " << port_);
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn AlphabotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  try {
    std::lock_guard<std::mutex> lock(serialMutex);
    if (arduino_.IsOpen()) {
      arduino_.Write("S,0.000,0.000\nSTOP\n");
      arduino_.Close();
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlphabotInterface"), "Could not close " << port_);
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AlphabotInterface::read(const rclcpp::Time &, const rclcpp::Duration &period)
{
  while (arduino_.IsOpen() && arduino_.IsDataAvailable()) {
    std::string line;
    try {
      std::lock_guard<std::mutex> lock(serialMutex);
      arduino_.ReadLine(line);
    } catch (...) {
      return hardware_interface::return_type::ERROR;
    }

    // Print the raw Pico line before parsing. Remove line terminators only so
    // each received message appears on one ROS log line.
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    // RCLCPP_INFO(node_->get_logger(), "Pico RX: [%s]", line.c_str());

    const std::vector<std::string> fields = split(line);
    if (fields.empty()) {
      continue;
    }

    // STATE,left_speed,right_speed,battery_voltage,bumper_left,bumper_right,
    // accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,temperature
    if (fields[0] == "STATE" && fields.size() == 16) {
      double values[15];
      bool valid = true;
      for (size_t index = 0; index < 15; ++index) {
        valid = valid && readDouble(fields[index + 1], values[index]);
      }
      if (!valid) {
        RCLCPP_WARN(node_->get_logger(), "Ignoring malformed Pico STATE message");
        continue;
      }

      const double leftSpeed = values[0];
      const double rightSpeed = values[1];

      // Existing interface ordering: index 0 is right, index 1 is left.
      velocity_states_[0] = rightSpeed;
      velocity_states_[1] = leftSpeed;
      position_states_[0] += rightSpeed * period.seconds();
      position_states_[1] += leftSpeed * period.seconds();

      std_msgs::msg::Float32 battery;
      battery.data = static_cast<float>(values[2]);
      battery_pub_->publish(battery);

      const bool leftPressed = values[3] != 0.0;
      const bool rightPressed = values[4] != 0.0;
      std_msgs::msg::UInt8 bumper;
      bumper.data = static_cast<uint8_t>(
          (leftPressed ? 0x01 : 0x00) | (rightPressed ? 0x02 : 0x00));
      bumperPublisher->publish(bumper);

      const rclcpp::Time stamp = node_->now();
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = "imu_link";
      imu.orientation_covariance[0] = -1.0;
      imu.linear_acceleration.x = values[5];
      imu.linear_acceleration.y = values[6];
      imu.linear_acceleration.z = values[7];
      imu.angular_velocity.x = values[8];
      imu.angular_velocity.y = values[9];
      imu.angular_velocity.z = values[10];
      imuPublisher->publish(imu);

      sensor_msgs::msg::MagneticField magneticField;
      magneticField.header.stamp = stamp;
      magneticField.header.frame_id = "imu_link";
      magneticField.magnetic_field.x = values[11];
      magneticField.magnetic_field.y = values[12];
      magneticField.magnetic_field.z = values[13];
      magneticFieldPublisher->publish(magneticField);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Ignoring unknown Pico message: [%s]", line.c_str());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlphabotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const double rightSpeed = std::isfinite(velocity_commands_[0]) ? velocity_commands_[0] : 0.0;
  const double leftSpeed = std::isfinite(velocity_commands_[1]) ? velocity_commands_[1] : 0.0;
  std::ostringstream command;
  command << std::fixed << std::setprecision(3) << "S," << leftSpeed << "," << rightSpeed << "\n";

  try {
    std::lock_guard<std::mutex> lock(serialMutex);
    arduino_.Write(command.str());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AlphabotInterface"), "Could not send " << command.str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace alphabot_firmware

PLUGINLIB_EXPORT_CLASS(alphabot_firmware::AlphabotInterface, hardware_interface::SystemInterface)
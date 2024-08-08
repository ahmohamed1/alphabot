#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <string>
#include <iostream>
#include <libserial/SerialPort.h>

class ImuPublisherNode : public rclcpp::Node {
public:
    ImuPublisherNode() : Node("imu_publisher_node") {
        // Initialize serial port settings
        declare_parameter<std::string>("port", "/dev/ttyACM0");

        port_ = get_parameter("port").as_string();

        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

        // Initialize publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

        // Create a timer to read serial data periodically
        auto timer_callback = [this]() {
            readSerialData();
        };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }

private:
    void readSerialData() {
        std::string message;
        arduino_.ReadLine(message);
        // Array to hold the separated readings
        std::vector<float> readings(8, 0.0);
        // Create a stringstream object to parse the data
        std::stringstream ss(message);
        std::string token;
        int i = 0;
        // Parse the received string, assuming readings are separated by commas
        while (getline(ss, token, ',') && i < 8) {
            // Convert the token to a float and store it in the array
            readings[i] = stof(token);
            i++;
        }

        if (readings.size() == 8) {
            sensor_msgs::msg::Imu imu_msg;

            // Populate IMU message with parsed data
            imu_msg.linear_acceleration.x = readings[0];
            imu_msg.linear_acceleration.y = readings[1];
            imu_msg.linear_acceleration.z = readings[2];

            imu_msg.angular_velocity.x = readings[3];
            imu_msg.angular_velocity.y = readings[4];
            imu_msg.angular_velocity.z = readings[5];

            // Publish the IMU message
            imu_pub_->publish(imu_msg);
        }
    }

    LibSerial::SerialPort arduino_;
    std::string port_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

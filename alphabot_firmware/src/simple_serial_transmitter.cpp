#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <libserial/SerialPort.h>

using std::placeholders::_1;

class SimpleSerialTransmitter : public rclcpp::Node
{
public:
  SimpleSerialTransmitter() : Node("simple_serial_transmitter")
  {
    declare_parameter<std::string>("port", "/dev/ttyACM0");

    port_ = get_parameter("port").as_string();

    sub_ = create_subscription<std_msgs::msg::String>(
        "serial_transmitter", 10, std::bind(&SimpleSerialTransmitter::msgCallback, this, _1));
    
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
    arduino_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    arduino_.SetParity(LibSerial::Parity::PARITY_NONE);
    arduino_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    arduino_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    arduino_.SetDTR(false);
    arduino_.SetRTS(false);
    // Flush buffers.
    arduino_.FlushIOBuffers();
  }

  ~SimpleSerialTransmitter()
  {
    arduino_.Close();
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string port_;
  LibSerial::SerialPort arduino_;

  void msgCallback(const std_msgs::msg::String &msg)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "New message received, publishing on serial: " << msg.data);
    
    std::cout<< "We send: " << msg.data<<std::endl;
    arduino_.Write(msg.data+'\r');
    std::string message;
    arduino_.ReadLine(message);
    // static const std::string delimiter = ",";
    // const size_t del_pos = message.find(delimiter);
    // const std::string token_1 = message.substr(0, del_pos).c_str();
    // const std::string token_2 = message.substr(del_pos + delimiter.length()).c_str();
    // std::cout<< "We recived: " <<std::atoi(token_1.c_str())<<","<< std::atoi(token_2.c_str())<<std::endl;
    std::cout<< "We recived: " <<message<<std::endl;
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSerialTransmitter>());
  rclcpp::shutdown();
  return 0;
}
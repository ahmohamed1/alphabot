#include "alphabot_firmware/alphabot_interface.hpp"

#include <cassert>
#include <chrono>
#include <iostream>
#include <poll.h>
#include <pty.h>
#include <sys/ioctl.h>
#include <unistd.h>

using alphabot_firmware::CallbackReturn;
using hardware_interface::return_type;
using namespace std::chrono_literals;

class SerialPeer
{
public:
  SerialPeer() { assert(openpty(&master_, &slave_, path, nullptr, nullptr) == 0); }
  ~SerialPeer() { close(master_); close(slave_); }
  void prepareOpen()
  {
    // The fixture holds a slave fd open, so LibSerial's TIOCEXCL survives
    // the hardware fd closing. Emulate release on the last real port close.
    assert(ioctl(slave_, TIOCNXCL) == 0);
  }
  void send(const std::string &data)
  {
    assert(::write(master_, data.data(), data.size()) == static_cast<ssize_t>(data.size()));
    std::this_thread::sleep_for(5ms);
  }
  std::string receive()
  {
    std::string result;
    pollfd descriptor{master_, POLLIN, 0};
    while (poll(&descriptor, 1, 20) > 0 && (descriptor.revents & POLLIN)) {
      char buffer[1024];
      const auto count = ::read(master_, buffer, sizeof(buffer));
      assert(count > 0);
      result.append(buffer, count);
    }
    return result;
  }
  char path[128]{};
private:
  int master_ = -1, slave_ = -1;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  SerialPeer peer;
  {
    alphabot_firmware::AlphabotInterface hardware;
    hardware_interface::HardwareInfo info;
    info.name = "test_robot";
    info.type = "system";
    info.hardware_parameters["port"] = peer.path;
    for (const char *name : {"left_wheel_joint", "right_wheel_joint"}) {
      hardware_interface::ComponentInfo joint;
      joint.name = name;
      joint.type = "joint";
      info.joints.push_back(joint);
    }
    assert(hardware.on_init(info) == CallbackReturn::SUCCESS);
    auto states = hardware.export_state_interfaces();
    auto commands = hardware.export_command_interfaces();
    const rclcpp_lifecycle::State lifecycle;
    const rclcpp::Time now(0);
    const auto period = rclcpp::Duration::from_seconds(0.02);
    auto assertCleared = [&]() {
      for (const auto &state : states) assert(state.get_optional().value() == 0);
      for (const auto &command : commands) assert(command.get_optional().value() == 0);
    };
    auto activate = [&]() {
      peer.prepareOpen();
      assert(hardware.on_activate(lifecycle) == CallbackReturn::SUCCESS);
      assert(peer.receive() == "CONNECT\n");
      peer.send("CONNECTED\n");
      assert(hardware.read(now, period) == return_type::OK);
      assertCleared();
    };

    assert(hardware.on_activate(lifecycle) == CallbackReturn::SUCCESS);
    assert(peer.receive() == "CONNECT\n");
    assert(commands[0].set_value(2.0));
    assert(hardware.write(now, period) == return_type::OK);
    assert(peer.receive().empty()); // No motion before the handshake.
    peer.send("CONNE");
    const auto start = std::chrono::steady_clock::now();
    assert(hardware.read(now, period) == return_type::OK);
    assert(std::chrono::steady_clock::now() - start < 100ms);
    peer.send("CTED\n");
    assert(hardware.read(now, period) == return_type::OK);
    assertCleared();

    peer.send("STATE,1.5,1.5,12.7,0,0,0,0,9.8,0,0,0,0,0,0,20,100,200\n");
    assert(hardware.read(now, period) == return_type::OK);
    assert(states[0].get_optional().value() > 0);
    assert(states[1].get_optional().value() == 1.5);
    assert(commands[0].set_value(1.5) && commands[1].set_value(1.5));
    std::this_thread::sleep_for(260ms);
    assert(hardware.write(now, period) == return_type::OK);
    assert(peer.receive() == "HEARTBEAT\nS,1.500,1.500\n");
    peer.send("HEARTBEAT_OK\n");
    assert(hardware.read(now, period) == return_type::OK);
    assert(hardware.on_deactivate(lifecycle) == CallbackReturn::SUCCESS);
    assert(peer.receive() == "STOP\nDISCONNECT\n");
    assertCleared();

    activate();
    // STATE traffic alone must not mask a broken Pi -> Pico direction.
    std::this_thread::sleep_for(1010ms);
    peer.send("STATE,1,1,12.7,0,0,0,0,9.8,0,0,0,0,0,0,20,100,200\n");
    assert(hardware.read(now, period) == return_type::ERROR);
    assert(peer.receive() == "STOP\nDISCONNECT\n");
    assertCleared();
    assert(hardware.write(now, period) == return_type::ERROR);
    assert(peer.receive().empty());

    activate();
    peer.send("LINK_TIMEOUT\n");
    assert(hardware.read(now, period) == return_type::ERROR);
    assert(peer.receive() == "STOP\nDISCONNECT\n");
    assertCleared();

    activate();
    assert(hardware.on_error(lifecycle) == CallbackReturn::SUCCESS);
    assert(peer.receive() == "STOP\nDISCONNECT\n");
    assertCleared();

    // An old firmware that never acknowledges CONNECT must not be allowed to drive.
    peer.prepareOpen();
    assert(hardware.on_activate(lifecycle) == CallbackReturn::SUCCESS);
    assert(peer.receive() == "CONNECT\n");
    std::this_thread::sleep_for(3010ms);
    assert(hardware.read(now, period) == return_type::ERROR);
    assert(peer.receive() == "STOP\nDISCONNECT\n");
    assertCleared();
    activate();
  }
  assert(peer.receive() == "STOP\nDISCONNECT\n"); // Destructor also closes the session.
  std::cout << "PASS: serial handshake, partial input, heartbeat loss, close/reset, reconnect, error, teardown\n";
}

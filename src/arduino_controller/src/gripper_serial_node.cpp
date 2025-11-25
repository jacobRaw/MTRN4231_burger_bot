#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include "custom_interfaces/srv/gripper_server.hpp"

/**
 * @usage ros2 service call /gripper_server custom_interfaces/srv/GripperServer "{command: 'o'}"
 */

class GripperSerialNode : public rclcpp::Node {
public:
  GripperSerialNode() : Node("gripper_serial_node") {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->get_parameter("serial_port", serial_port_);

    // Open serial port
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Configure serial port
    struct termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
      rclcpp::shutdown();
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                      // disable break processing
    tty.c_lflag = 0;                             // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                             // no remapping, no delays
    tty.c_cc[VMIN] = 0;                          // read doesn't block
    tty.c_cc[VTIME] = 5;                         // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);             // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);           // no parity
    tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                     // no hardware flow control

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to %s at 115200 baud", serial_port_.c_str());
    sleep(2);  // allow Teensy to reset

    // Subscribe to topic
    // subscriber_ = this->create_subscription<std_msgs::msg::String>(
    //     "gripper_command", 10,
    //     std::bind(&GripperSerialNode::command_callback, this, std::placeholders::_1));
    // Create a server to receive commands
    srv_ = create_service<custom_interfaces::srv::GripperServer>("gripper_server",
                                                               std::bind(&GripperSerialNode::command_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  ~GripperSerialNode() override {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  void command_callback(const std::shared_ptr<custom_interfaces::srv::GripperServer::Request> request,
                       std::shared_ptr<custom_interfaces::srv::GripperServer::Response> response) {
    std::string cmd = request->command;
    std::string to_send;

    if (cmd == "open" || cmd == "o") {
      to_send = "o\n";
      RCLCPP_INFO(this->get_logger(), "Sent command: open");
    } else if (cmd == "close" || cmd == "c") {
      to_send = "c\n";
      RCLCPP_INFO(this->get_logger(), "Sent command: close");
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown command: %s", cmd.c_str());
      return;
    }

    write(serial_fd_, to_send.c_str(), to_send.size());
    tcdrain(serial_fd_);  // ensure transmission completes

    response->success = true;
    response->message = "Command to " + cmd + " received successfully.";
  }

  int serial_fd_;
  std::string serial_port_;
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::Service<custom_interfaces::srv::GripperServer>::SharedPtr srv_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
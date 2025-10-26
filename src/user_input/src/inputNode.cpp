// UserInput node which reads in the userName and order from /input topic and appropriately 
// starts an action server with the brain to create the burger

#include <chrono>
// #include <functional>
// #include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/command.hpp"

using std::placeholders::_1;

enum class Burgers {
  CheeseBurger,
  VeggieBurger,
  ChickenBurger,
  DoublePattyBurger,
  LoadedBurger
};

class InputNode : public rclcpp::Node
{
public:
  InputNode()
  : Node("InputNode")
  {
    RCLCPP_INFO(this->get_logger(), "InputNode has started.");
    input_sub = this->create_subscription<custom_interfaces::msg::Command>(
      "input", 10, std::bind(&InputNode::topic_callback, this, _1));
  }

private:
  rclcpp::Subscription<custom_interfaces::msg::Command>::SharedPtr input_sub;

  void topic_callback(const custom_interfaces::msg::Command::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "User Name: '%s'", msg->username.data.c_str());
    RCLCPP_INFO(this->get_logger(), "Order: '%s'", msg->order.data.c_str());
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputNode>());
  rclcpp::shutdown();
  return 0;
}

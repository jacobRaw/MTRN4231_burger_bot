// UserInput node which reads in the userName and order from std::cin and appropriately 
// starts an action server with the brain to create the burger

#include <chrono>
// #include <functional>
// #include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "std_msgs/msg/int64.hpp"
// #include "geometry_msgs/msg/pose.hpp"
// #include "tutorial_interfaces/msg/posestamped.hpp"

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class InputNode : public rclcpp::Node
{
public:
  InputNode()
  : Node("InputNode")
  {
    // subscription_ = this->create_subscription<>(
    //   "lab1_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  std::string userName = "";
  std::string burger = "";
  std::cout << "Please enter you user name: ";
  std::cin >> userName;

  std::cout << "Please enter your burger order: ";
  std::cin >> burger;

  std::cout << "Initiating action for " << userName << " with burger order: " << burger << std::endl;

  rclcpp::shutdown();
  return 0;
}

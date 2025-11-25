cmake_minimum_required(VERSION 3.5)
project(arduino_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(serialib REQUIRED)
find_package(custom_interfaces REQUIRED)


add_executable(gripper_serial_node src/gripper_serial_node.cpp)
ament_target_dependencies(gripper_serial_node 
"rclcpp"
"std_msgs"
"custom_interfaces"
)

install(TARGETS
  gripper_serial_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

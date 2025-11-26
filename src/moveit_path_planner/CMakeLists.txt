cmake_minimum_required(VERSION 3.10)
project(moveit_path_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

add_executable(moveit_path_planning_action
  src/moveit_path_planning_action.cpp
)
target_include_directories(moveit_path_planning_action PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_path_planning_action PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_path_planning_action
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

ament_target_dependencies(moveit_path_planning_action
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  custom_interfaces
)

install(TARGETS 
  moveit_path_planning_action
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  ../launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
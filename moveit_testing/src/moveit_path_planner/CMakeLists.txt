cmake_minimum_required(VERSION 3.10)
project(moveit_path_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interface REQUIRED)
find_package(ompl REQUIRED)

add_executable(moveit_path_planning_server
  src/moveit_path_planning_server.cpp
)
target_include_directories(moveit_path_planning_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_path_planning_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_path_planning_server
  "moveit_ros_planning_interface"
  "rclcpp"
  # "moveit_visual_tools"
)

ament_target_dependencies(moveit_path_planning_server
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  custom_interface
)

install(TARGETS 
  moveit_path_planning_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
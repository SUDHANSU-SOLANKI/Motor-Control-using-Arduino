cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(my_robot_hardware SHARED
  src/my_robot_hardware.cpp
)

target_include_directories(my_robot_hardware PUBLIC
  include
)

ament_target_dependencies(my_robot_hardware
  hardware_interface
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  my_robot_hardware.xml
)

install(
  TARGETS my_robot_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES my_robot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

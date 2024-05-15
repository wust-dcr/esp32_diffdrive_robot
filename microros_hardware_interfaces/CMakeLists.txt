cmake_minimum_required(VERSION 3.8)
project(microros_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_library(
  microros_hardware_interfaces
  SHARED
  src/microros_hardware_interfaces.cpp
)
target_include_directories(
  microros_hardware_interfaces
  PUBLIC
  include
)
ament_target_dependencies(
  microros_hardware_interfaces
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
# prevent pluginlib from using boost
target_compile_definitions(microros_hardware_interfaces PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface microros_hardware_interfaces.xml)

install(
  TARGETS
  microros_hardware_interfaces
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  microros_hardware_interfaces
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()

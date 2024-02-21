cmake_minimum_required(VERSION 3.8)
project(sequence_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(setpoint_generator src/setpoint_generator_node.cpp)
ament_target_dependencies(setpoint_generator rclcpp std_msgs geometry_msgs)

add_executable(sequence_controller src/sequence_controller_node.cpp)
ament_target_dependencies(sequence_controller rclcpp std_msgs geometry_msgs)

install(TARGETS
  setpoint_generator
  sequence_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

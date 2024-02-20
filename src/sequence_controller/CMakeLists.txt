cmake_minimum_required(VERSION 3.8)
project(sequence_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(setpoint_generator src/setpoint_generator_node.cpp)
ament_target_dependencies(setpoint_generator rclcpp std_msgs)


install(TARGETS
  setpoint_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

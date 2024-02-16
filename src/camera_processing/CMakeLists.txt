cmake_minimum_required(VERSION 3.8)
project(camera_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgproc highgui)

add_executable(avg_brightness src/brightness_node.cpp)
ament_target_dependencies(avg_brightness rclcpp sensor_msgs std_msgs rcl_interfaces OpenCV)

add_executable(brightness_sub src/brightness_sub.cpp)
ament_target_dependencies(brightness_sub rclcpp std_msgs)

add_executable(light_position src/light_position.cpp)
ament_target_dependencies(light_position rclcpp sensor_msgs geometry_msgs rcl_interfaces OpenCV)

add_executable(light_position_sub src/light_position_sub.cpp)
ament_target_dependencies(light_position_sub rclcpp geometry_msgs)

install(TARGETS
  avg_brightness
  brightness_sub
  light_position
  light_position_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

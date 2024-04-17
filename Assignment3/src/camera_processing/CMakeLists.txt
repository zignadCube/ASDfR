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

add_executable(object_position src/object_position.cpp)
ament_target_dependencies(object_position rclcpp sensor_msgs geometry_msgs rcl_interfaces OpenCV)

install(TARGETS
  object_position
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

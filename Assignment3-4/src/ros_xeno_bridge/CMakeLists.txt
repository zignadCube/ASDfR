cmake_minimum_required(VERSION 3.8)
project(ros_xeno_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(RosXenoBridge src/RosXenoBridge.cpp)

set_property(TARGET RosXenoBridge PROPERTY CXX_STANDARD 17)
                
ament_target_dependencies(RosXenoBridge PUBLIC rclcpp custom_msgs)

target_include_directories(RosXenoBridge PUBLIC include /usr/evl/include ../../include/)

target_link_directories(RosXenoBridge PUBLIC /usr/evl/lib)

target_link_libraries(RosXenoBridge PUBLIC evl)

# Install targets
install(
  TARGETS 
    RosXenoBridge
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

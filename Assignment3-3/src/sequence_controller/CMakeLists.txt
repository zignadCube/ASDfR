cmake_minimum_required(VERSION 3.8)
project(sequence_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(sequence_controller src/sequence_controller.cpp)
ament_target_dependencies(sequence_controller rclcpp custom_msgs rcl_interfaces geometry_msgs)

add_executable(sequence_controller src/sequence_controller_3.3.cpp)
ament_target_dependencies(sequence_controller_33 rclcpp custom_msgs rcl_interfaces geometry_msgs)

install(TARGETS
  sequence_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

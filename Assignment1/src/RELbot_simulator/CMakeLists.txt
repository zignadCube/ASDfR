cmake_minimum_required(VERSION 3.8)
project(RELbot_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Try to force CPP 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wno-psabi) # Suppress warnings about ABI changes in GCC 7.1

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)
find_package(cv_bridge REQUIRED)

# add_subdirectory (../lib lib)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(RELbot_simulator 
src/RELbot_simulator.cpp 
src/DynamicsSimulation.cpp)

ament_target_dependencies(RELbot_simulator 
rclcpp 
std_msgs 
sensor_msgs 
geometry_msgs
cv_bridge)

target_include_directories(RELbot_simulator PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(RELbot_simulator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(${PROJECT_NAME}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS})

install(TARGETS RELbot_simulator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
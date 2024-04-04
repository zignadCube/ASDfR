cmake_minimum_required(VERSION 3.10)
project(xenoThread)

#set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wno-pointer-arith)

# Find dependencies
find_package(Threads REQUIRED)
find_package(custom_msgs REQUIRED)
file(GLOB SOURCES "controller/common/*.cpp")
add_executable  (xenoThread main.cpp 
                src/ico_io_v2.cpp 
                myApp/src/MyApp.cpp
                src/NativeBase.cpp 
                src/Native20Sim.cpp
                controller/LoopController.cpp
                ${SOURCES}
                )

ament_target_dependencies(xenoThread PUBLIC custom_msgs)

# Specify paths to header files
target_include_directories( xenoThread BEFORE PUBLIC 
                            /usr/evl/include 
                            include/ 
                            myApp/include 
                            ../../include/ 
                            controller/ 
                            controller/common/
                            )

# Specify paths to library files
target_link_directories(xenoThread PUBLIC /usr/evl/lib)

target_link_libraries(xenoThread PRIVATE evl)

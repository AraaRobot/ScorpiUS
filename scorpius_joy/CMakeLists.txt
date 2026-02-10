cmake_minimum_required(VERSION 3.8)
project(scorpius_joy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joy REQUIRED)
find_package(scorpius_main REQUIRED)

add_executable(joy_formator src/joy_formator.cpp)

ament_target_dependencies(joy_formator
                          rclcpp
                          sensor_msgs
                          scorpius_main
                          joy)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
            joy_formator
            DESTINATION lib/${PROJECT_NAME}/)
ament_package()

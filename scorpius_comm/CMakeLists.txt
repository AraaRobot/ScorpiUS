cmake_minimum_required(VERSION 3.8)
project(scorpius_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(comm src/comm_node.cpp)

ament_target_dependencies(comm
                          rclcpp)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
            comm
            DESTINATION lib/${PROJECT_NAME}/)

ament_package()

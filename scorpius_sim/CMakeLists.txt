cmake_minimum_required(VERSION 3.8)
project(scorpius_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scorpius_main REQUIRED)

add_executable(controller_sim src/ControllerSim/controller_sim.cpp)

ament_target_dependencies(controller_sim
                          rclcpp
                          scorpius_main)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
            controller_sim
            DESTINATION lib/${PROJECT_NAME}/)
ament_package()

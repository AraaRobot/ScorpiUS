cmake_minimum_required(VERSION 3.8)
project(scorpius_media)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(media src/media_node.cpp)

ament_target_dependencies(media
                          rclcpp)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
            media
            DESTINATION lib/${PROJECT_NAME}/)

ament_package()

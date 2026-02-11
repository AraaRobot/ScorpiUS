cmake_minimum_required(VERSION 3.8)
project(scorpius_gui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
)

set(CMAKE_AUTORCC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt6 COMPONENTS Widgets QUIET)
if(Qt6_FOUND)
  set(QT_WIDGETS_LIB Qt6::Widgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_WIDGETS_LIB Qt5::Widgets)
endif()

add_executable(gui  src/gui.cpp
                    src/gui_window.cpp
                    src/debug_widget/debug_widget.cpp
                    src/debug_widget/debug_widget_manager.cpp
                    ui/main_window.ui
                    ui/DebugWidget.ui)

ament_target_dependencies(gui
                          rclcpp)

target_include_directories(gui PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(gui
  ${QT_WIDGETS_LIB}
)                

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
            gui
            DESTINATION lib/${PROJECT_NAME}/)
ament_package()

cmake_minimum_required(VERSION 3.16)
project(gll_ros2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(gll_core REQUIRED)

set(DEPS rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs tf2_ros)

# ROS メッセージ ⇔ コアの型の変換と、診断メッセージの生成（ROS に依存する IF 層の部品）
add_library(gll_ros2_conversions src/conversions.cpp src/diagnostics.cpp)
target_include_directories(gll_ros2_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gll_ros2_conversions PUBLIC gll::gll_core Eigen3::Eigen)
ament_target_dependencies(gll_ros2_conversions PUBLIC ${DEPS})

add_executable(localizer_node src/localizer_node.cpp src/main.cpp)
target_link_libraries(localizer_node gll_ros2_conversions gll::gll_core)
ament_target_dependencies(localizer_node ${DEPS})
target_compile_options(localizer_node PRIVATE -Wall -Wextra)

install(TARGETS localizer_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS gll_ros2_conversions
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions gll_ros2_conversions)
  ament_add_gtest(test_diagnostics test/test_diagnostics.cpp)
  target_link_libraries(test_diagnostics gll_ros2_conversions)
endif()

ament_package()

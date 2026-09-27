cmake_minimum_required(VERSION 3.16)
project(gll_core VERSION 0.2.0 LANGUAGES CXX)
include(GNUInstallDirs)

# ROS に依存しないコアライブラリ（設計書 7 章）。
# plain CMake でビルドでき、colcon からは build_type=cmake のパッケージとして扱われる。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(GLL_BUILD_TESTS "Build unit tests" ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED COMPONENTS CXX)
# small_gicp（スキャンマッチング。設計書 6 章）: v1.0.1 をソースからビルドしてインストールしておく（README）
find_package(small_gicp 1.0 REQUIRED)
# gll_map（地図のタイル化・動的ロード、測地変換・SE(2) などの基本。../map）。
# colcon ではパッケージとして先にビルドされる。plain CMake でコアだけをビルドする場合は、リポジトリの map/ を一緒にビルドする
find_package(gll_map QUIET)
if(NOT gll_map_FOUND)
  set(GLL_MAP_BUILD_TESTS ${GLL_BUILD_TESTS} CACHE BOOL "")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../map ${CMAKE_CURRENT_BINARY_DIR}/gll_map)
endif()

add_library(gll_core
  src/mahalanobis_gate.cpp
  src/state_estimator.cpp
  src/inv_ekf_se2.cpp
  src/es_ekf_2d.cpp
  src/state_history.cpp
  src/attitude_estimator.cpp
  src/motion_input_builder.cpp
  src/gnss_measurement_builder.cpp
  src/stop_detector.cpp
  src/output_smoother.cpp
  src/status_monitor.cpp
  src/initializer.cpp
  src/source_arbiter.cpp
  src/recovery_manager.cpp
  src/localizer.cpp
  src/scan_preprocessor.cpp
  src/gicp_matcher.cpp
  src/lidar_measurement_builder.cpp
  src/pose_store.cpp
)
add_library(gll::gll_core ALIAS gll_core)
target_include_directories(gll_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(gll_core
  PUBLIC Eigen3::Eigen gll::gll_map
  PRIVATE small_gicp::small_gicp OpenMP::OpenMP_CXX)
target_compile_options(gll_core PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# install / export（gll_ros2 から find_package(gll_core) で使う）
include(CMakePackageConfigHelpers)
install(TARGETS gll_core EXPORT gll_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION include)
install(EXPORT gll_coreTargets NAMESPACE gll:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gll_core)
configure_package_config_file(cmake/gll_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/gll_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gll_core)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/gll_coreConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/gll_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/gll_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gll_core)

# ---------------------------------------------------------------------------
if(GLL_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  set(GLL_TESTS
    test_mahalanobis_gate
    test_estimators
    test_state_history
    test_attitude_estimator
    test_gnss_measurement_builder
    test_output_smoother
    test_localizer_sim
    test_scan_matching
    test_lidar_localization
    test_recovery_manager
  )
  foreach(t ${GLL_TESTS})
    add_executable(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE test)
    target_link_libraries(${t} PRIVATE gll_core GTest::gtest GTest::gtest_main)
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

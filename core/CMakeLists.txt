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
option(GLL_BUILD_TOOLS "Build offline tools (gll_map_tiler)" ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED COMPONENTS CXX)
# small_gicp（スキャンマッチング。設計書 6 章）: v1.0.1 をソースからビルドしてインストールしておく（README）
find_package(small_gicp 1.0 REQUIRED)
# yaml-cpp（maps.yaml と tile_index.yaml）
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(GLL_YAML_CPP yaml-cpp::yaml-cpp)
else()
  set(GLL_YAML_CPP yaml-cpp)
endif()

# GeographicLib: CMake の config ファイルがない環境（Debian/Ubuntu のパッケージなど）もあるので直接探す
find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/UTMUPS.hpp)
find_library(GEOGRAPHICLIB_LIBRARY NAMES GeographicLib Geographic)
if(NOT GEOGRAPHICLIB_INCLUDE_DIR OR NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR "GeographicLib not found (apt: libgeographiclib-dev)")
endif()

add_library(gll_core
  src/geodesy.cpp
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
  src/map_anchor.cpp
  src/map_config.cpp
  src/tile.cpp
  src/pcd_io.cpp
  src/map_tiler.cpp
  src/scan_preprocessor.cpp
  src/gicp_matcher.cpp
  src/map_tile_manager.cpp
  src/lidar_measurement_builder.cpp
  src/pose_store.cpp
)
add_library(gll::gll_core ALIAS gll_core)
target_include_directories(gll_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GEOGRAPHICLIB_INCLUDE_DIR}
)
target_link_libraries(gll_core
  PUBLIC Eigen3::Eigen
  PRIVATE ${GEOGRAPHICLIB_LIBRARY} small_gicp::small_gicp ${GLL_YAML_CPP} OpenMP::OpenMP_CXX)
target_compile_options(gll_core PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# オフラインツール
if(GLL_BUILD_TOOLS)
  add_executable(gll_map_tiler tools/map_tiler_main.cpp)
  target_link_libraries(gll_map_tiler PRIVATE gll_core)
  install(TARGETS gll_map_tiler RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

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
    test_se2
    test_geodesy
    test_mahalanobis_gate
    test_estimators
    test_state_history
    test_attitude_estimator
    test_gnss_measurement_builder
    test_output_smoother
    test_localizer_sim
    test_map_io
    test_scan_matching
    test_map_tile_manager
    test_lidar_localization
    test_recovery_manager
  )
  foreach(t ${GLL_TESTS})
    add_executable(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE test ${GEOGRAPHICLIB_INCLUDE_DIR})
    target_link_libraries(${t} PRIVATE gll_core GTest::gtest GTest::gtest_main ${GEOGRAPHICLIB_LIBRARY})
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

cmake_minimum_required(VERSION 3.16)
project(gll_core VERSION 0.1.0 LANGUAGES CXX)

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
)
add_library(gll::gll_core ALIAS gll_core)
target_include_directories(gll_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GEOGRAPHICLIB_INCLUDE_DIR}
)
target_link_libraries(gll_core PUBLIC Eigen3::Eigen PRIVATE ${GEOGRAPHICLIB_LIBRARY})
target_compile_options(gll_core PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
# install / export（gll_ros2 から find_package(gll_core) で使う）
include(GNUInstallDirs)
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
  )
  foreach(t ${GLL_TESTS})
    add_executable(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE test)
    target_link_libraries(${t} PRIVATE gll_core GTest::gtest GTest::gtest_main)
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

cmake_minimum_required(VERSION 3.16)
project(tiled_pcd_map VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

# 点群地図のタイル化と動的ロードのライブラリ（ROS 非依存。README.md）。
# このリポジトリの gll_core（自己位置推定）が使うが、単体で使える（gll_core には依存しない）。
# plain CMake でビルドでき、colcon からは build_type=cmake のパッケージとして扱われる。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(TILED_PCD_MAP_BUILD_TESTS "Build unit tests" ON)
option(TILED_PCD_MAP_BUILD_TOOLS "Build tools (tiled_pcd_map_tiler, tiled_pcd_map_demo)" ON)
# small_gicp があれば、タイル化のときに点ごとの共分散を計算する（GICP 用）。無くてもビルドできる
option(TILED_PCD_MAP_USE_SMALL_GICP "Compute per-point covariances with small_gicp when it is available" ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(TILED_PCD_MAP_YAML_CPP yaml-cpp::yaml-cpp)
else()
  set(TILED_PCD_MAP_YAML_CPP yaml-cpp)
endif()
# GeographicLib: CMake の config ファイルがない環境（Debian/Ubuntu のパッケージなど）もあるので直接探す
find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/UTMUPS.hpp)
find_library(GEOGRAPHICLIB_LIBRARY NAMES GeographicLib Geographic)
if(NOT GEOGRAPHICLIB_INCLUDE_DIR OR NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR "GeographicLib not found (apt: libgeographiclib-dev)")
endif()

set(TILED_PCD_MAP_WITH_SMALL_GICP OFF)
if(TILED_PCD_MAP_USE_SMALL_GICP)
  find_package(small_gicp 1.0 QUIET)
  find_package(OpenMP COMPONENTS CXX QUIET)
  if(small_gicp_FOUND AND OpenMP_CXX_FOUND)
    set(TILED_PCD_MAP_WITH_SMALL_GICP ON)
  endif()
endif()
message(STATUS "tiled_pcd_map: per-point covariances with small_gicp: ${TILED_PCD_MAP_WITH_SMALL_GICP}")

add_library(tiled_pcd_map
  src/geodesy.cpp
  src/map_anchor.cpp
  src/map_config.cpp
  src/tile.cpp
  src/pcd_io.cpp
  src/map_tiler.cpp
  src/map_region.cpp
  src/map_tile_manager.cpp
)
add_library(tiled_pcd_map::tiled_pcd_map ALIAS tiled_pcd_map)
target_include_directories(tiled_pcd_map
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GEOGRAPHICLIB_INCLUDE_DIR}
)
find_package(Threads REQUIRED)
target_link_libraries(tiled_pcd_map
  PUBLIC Eigen3::Eigen Threads::Threads
  PRIVATE ${GEOGRAPHICLIB_LIBRARY} ${TILED_PCD_MAP_YAML_CPP})
if(TILED_PCD_MAP_WITH_SMALL_GICP)
  target_compile_definitions(tiled_pcd_map PRIVATE TILED_PCD_MAP_HAVE_SMALL_GICP)
  target_link_libraries(tiled_pcd_map PRIVATE small_gicp::small_gicp OpenMP::OpenMP_CXX)
endif()
target_compile_options(tiled_pcd_map PRIVATE -Wall -Wextra -Wpedantic)

# ---------------------------------------------------------------------------
if(TILED_PCD_MAP_BUILD_TOOLS)
  add_executable(tiled_pcd_map_tiler tools/map_tiler_main.cpp)
  target_link_libraries(tiled_pcd_map_tiler PRIVATE tiled_pcd_map)
  # タイル読み込みのデモ用の記録（tools/tile_demo/README.md）
  add_executable(tiled_pcd_map_demo tools/tile_demo_main.cpp)
  target_link_libraries(tiled_pcd_map_demo PRIVATE tiled_pcd_map)
  install(TARGETS tiled_pcd_map_tiler tiled_pcd_map_demo RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# ---------------------------------------------------------------------------
# install / export（find_package(tiled_pcd_map) で使う）
include(CMakePackageConfigHelpers)
install(TARGETS tiled_pcd_map EXPORT tiled_pcd_mapTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION include)
install(EXPORT tiled_pcd_mapTargets NAMESPACE tiled_pcd_map:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tiled_pcd_map)
configure_package_config_file(cmake/tiled_pcd_mapConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/tiled_pcd_mapConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tiled_pcd_map)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/tiled_pcd_mapConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/tiled_pcd_mapConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/tiled_pcd_mapConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/tiled_pcd_map)

# ---------------------------------------------------------------------------
if(TILED_PCD_MAP_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  foreach(t test_se2 test_geodesy test_map_io test_map_tile_manager)
    add_executable(${t} test/${t}.cpp)
    target_include_directories(${t} PRIVATE ${GEOGRAPHICLIB_INCLUDE_DIR})
    target_link_libraries(${t} PRIVATE tiled_pcd_map GTest::gtest GTest::gtest_main ${GEOGRAPHICLIB_LIBRARY})
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()

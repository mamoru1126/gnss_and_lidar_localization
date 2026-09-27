# tiled_pcd_map を find_package で使う最小の例（../../README.md）。
#   cmake -S tiled_pcd_map/examples/follow_path -B build_example -DCMAKE_PREFIX_PATH=<tiled_pcd_map のインストール先>
cmake_minimum_required(VERSION 3.16)
project(tiled_pcd_map_follow_path LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(tiled_pcd_map REQUIRED)
add_executable(follow_path main.cpp)
target_link_libraries(follow_path PRIVATE tiled_pcd_map::tiled_pcd_map)

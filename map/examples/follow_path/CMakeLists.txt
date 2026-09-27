# gll_map を find_package で使う最小の例（map/README.md）。
#   cmake -S map/examples/follow_path -B build_example -DCMAKE_PREFIX_PATH=<gll_map のインストール先>
cmake_minimum_required(VERSION 3.16)
project(gll_map_follow_path LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(gll_map REQUIRED)
add_executable(follow_path main.cpp)
target_link_libraries(follow_path PRIVATE gll::gll_map)

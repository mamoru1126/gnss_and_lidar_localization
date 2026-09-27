// 地図のライブラリ tiled_pcd_map（../tiled_pcd_map）の名前を gll から使えるようにする。
// tiled_pcd_map は gll に依存しないので、名前空間は別にしてある。gll の中では gll::Vec2・gll::SE2・
// gll::MapTileManager のように、どちらの名前も gll:: で書ける。
#pragma once

#include "tiled_pcd_map/math.hpp"

namespace tiled_pcd_map {}

namespace gll {
using namespace ::tiled_pcd_map;
}  // namespace gll

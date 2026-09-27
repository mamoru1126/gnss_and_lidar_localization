// 統合済みの点群地図をタイルに分割する（gll_map_tiler の中身。設計書 5.2 節・6.2 節）。
// 点ごとの共分散は、分割する前の点群全体で計算する（タイルの境界で近傍点が欠けないように）。
#pragma once

#include "gll/map/tile.hpp"

#include <string>
#include <vector>

namespace gll {

struct TilerOptions {
  double tile_size = 20.0;   ///< [m]
  double voxel_size = 0.2;   ///< 間引き [m]（0 以下なら間引かない）
  int num_neighbors = 20;    ///< 点ごとの共分散に使う近傍点数
  int num_threads = 4;
  bool compute_covariance = true;  ///< 点ごとの共分散を計算する（small_gicp なしでビルドした場合は計算できない）
};

struct TilerResult {
  TileIndex index;
  std::vector<TileData> tiles;  ///< index.tiles と同じ順
  std::size_t input_points = 0;
  std::size_t output_points = 0;
  bool has_covariance = false;  ///< タイルに点ごとの共分散を付けたか
};

/// 点ごとの共分散を計算できるか（gll_map を small_gicp 付きでビルドしたか）。
bool tilerCanComputeCovariance();

/// ボクセル（一辺 voxel_size）ごとに点の平均を 1 点にする。
std::vector<Vec3f> voxelDownsample(const std::vector<Vec3f>& points, double voxel_size);

/// 点群（地図座標系）を間引き、（できれば）点ごとの共分散を付けて、タイルに分ける。
TilerResult tileMap(const std::vector<Vec3f>& points, const TilerOptions& options, const std::string& group = "");

/// dir/tiles/<ix>_<iy>.bin と dir/tile_index.yaml を書く。
void writeTiles(const std::string& dir, const TilerResult& result);

}  // namespace gll

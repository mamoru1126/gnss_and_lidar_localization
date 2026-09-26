// 地図タイルの型と入出力（設計書 5.2 節・6.2 節）。
// タイルファイルは独自のバイナリ形式（float32 の xyz と、点ごとの共分散の上三角 6 要素）。
#pragma once

#include "gll/common/types.hpp"

#include <array>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace gll {

/// タイルの識別子。タイルの番号はグループの中でしか一意でないので、グループ ID を含める（v0.9）。
struct TileId {
  std::string group;
  int ix = 0;
  int iy = 0;

  bool operator<(const TileId& o) const { return std::tie(group, ix, iy) < std::tie(o.group, o.ix, o.iy); }
  bool operator==(const TileId& o) const { return group == o.group && ix == o.ix && iy == o.iy; }
  bool operator!=(const TileId& o) const { return !(*this == o); }
  std::string str() const { return group + ":" + std::to_string(ix) + "_" + std::to_string(iy); }
};

/// タイルのメタデータ（tile_index.yaml の 1 行）。
struct TileMeta {
  TileId id;
  std::string file;                ///< タイルファイルのパス（読み込み時に絶対パスにする）
  Vec3 bounds_min = Vec3::Zero();  ///< 点の範囲（地図座標）
  Vec3 bounds_max = Vec3::Zero();
  std::size_t num_points = 0;
};

/// 点ごとの共分散の上三角（xx, xy, xz, yy, yz, zz）。
using PackedCov = std::array<float, 6>;

/// タイルの点群（地図座標系）。
struct TileData {
  TileId id;
  std::vector<Vec3f> points;
  std::vector<PackedCov> covs;  ///< points と同じ数（共分散を持たない場合は空）
};

/// グループ 1 つ分のタイルの索引。
struct TileIndex {
  double tile_size = 20.0;
  double voxel_size = 0.0;   ///< タイル化の前の間引き（記録用）
  int num_neighbors = 0;     ///< 共分散の計算に使った近傍点数（記録用）
  std::vector<TileMeta> tiles;

  /// tile_index.yaml を読む。タイルファイルのパスは、この YAML からの相対パスとして解決する。
  static TileIndex load(const std::string& path, const std::string& group);
  /// tile_index.yaml を書く（タイルファイルのパスは tiles/<ix>_<iy>.bin の相対パスで書く）。
  void save(const std::string& path) const;
};

/// タイル番号（地図座標をタイルサイズで割って切り捨て）。
inline int tileCoord(double v, double tile_size) { return static_cast<int>(std::floor(v / tile_size)); }

/// タイルファイルの入出力。失敗したら std::runtime_error を投げる。
void writeTileFile(const std::string& path, const TileData& tile);
TileData readTileFile(const std::string& path);

/// タイルを読み込むインターフェース（テストで差し替えられるようにする）。
class ITileLoader {
 public:
  virtual ~ITileLoader() = default;
  virtual std::shared_ptr<const TileData> load(const TileMeta& meta) const = 0;
};

/// タイルファイルをそのまま読むローダー。
class BinaryTileLoader : public ITileLoader {
 public:
  std::shared_ptr<const TileData> load(const TileMeta& meta) const override;
};

}  // namespace gll

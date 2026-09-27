// 読み込み中のタイルをまとめた「領域」（MapTileManager が出すもの。README.md）。
//
// MapTileManager は、アクティブグループの読み込み済みタイルがそろうたびに RegionBuilder で領域を作り、差し替える。
// 既定のビルダーは点を持ったタイルを並べるだけ（TileSetRegion）。位置合わせ用の KdTree などを作りたい場合は、
// MapRegion を継承した型を返すビルダーを渡す（このリポジトリの gll_core の GicpMatcher::buildTarget がその例）。
#pragma once

#include "tiled_pcd_map/map_anchor.hpp"
#include "tiled_pcd_map/tile.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace tiled_pcd_map {

class MapRegion {
 public:
  virtual ~MapRegion() = default;

  std::string group;          ///< 地図グループ ID
  MapAnchor anchor;           ///< このグループのアンカー（地図座標 ⇔ UTM。領域の点はこれで UTM にする）
  std::vector<TileId> tiles;  ///< 含まれるタイル
  std::size_t num_points = 0;
};

/// 既定の領域: 読み込んだタイルをそのまま持つ（点はコピーしない）。
class TileSetRegion : public MapRegion {
 public:
  std::vector<std::shared_ptr<const TileData>> tile_data;  ///< tiles と同じ順

  /// 全タイルの点を 1 つの配列にまとめる（地図座標系）。
  std::vector<Vec3f> points() const;
  /// 全タイルの点を UTM（x = Easting、y = Northing、z = 楕円体高）にしてまとめる。
  std::vector<Vec3> pointsUtm() const;
};

/// 領域を作る関数。tiles はアクティブグループの読み込み済みタイル（空のことはない）。
using RegionBuilder = std::function<std::shared_ptr<const MapRegion>(
    const std::string& group, const MapAnchor& anchor, const std::vector<std::shared_ptr<const TileData>>& tiles)>;

/// 既定のビルダー（TileSetRegion を作る）。
std::shared_ptr<const MapRegion> buildTileSetRegion(const std::string& group, const MapAnchor& anchor,
                                                    const std::vector<std::shared_ptr<const TileData>>& tiles);

}  // namespace tiled_pcd_map

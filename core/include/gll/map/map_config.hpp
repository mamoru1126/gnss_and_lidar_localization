// 地図グループの設定（maps.yaml。設計書 5.2 節）。
#pragma once

#include "gll/map/map_anchor.hpp"
#include "gll/map/tile.hpp"

#include <optional>
#include <string>
#include <vector>

namespace gll {

struct MapGroupConfig {
  std::string id;
  std::string tile_index;  ///< tile_index.yaml のパス（読み込み時に絶対パスにする）
  AnchorConfig anchor;
  bool local_anchor = false;  ///< anchor: local（地図座標をそのまま map として出力する）
};

struct MapSetConfig {
  std::optional<int> utm_zone;      ///< maps.yaml に書かれていれば、GNSS の設定と一致するか確かめる
  std::optional<bool> utm_north;
  std::vector<MapGroupConfig> groups;

  /// anchor: local の地図か（map = 地図の座標。GNSS の UTM とは比べられないので、GNSS は使えない）。
  bool local() const { return !groups.empty() && groups.front().local_anchor; }
};

/// maps.yaml を読む。相対パスは maps.yaml のあるディレクトリから解決する。失敗したら std::runtime_error。
/// anchor: local のグループと、そうでないグループは混在できない（座標系が別になるため）。
MapSetConfig loadMapSetConfig(const std::string& path);

/// 読み込んだ地図グループ（アンカーとタイルの索引）。
struct MapGroup {
  std::string id;
  MapAnchor anchor;
  TileIndex index;
};

/// maps.yaml とそこから参照される tile_index.yaml を読み、地図グループの一覧を作る。
std::vector<MapGroup> loadMapGroups(const MapSetConfig& cfg, const UtmProjector& utm);

}  // namespace gll
